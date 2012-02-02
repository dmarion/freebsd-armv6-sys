/*-
 * Copyright (c) 1994-1998 Mark Brinicombe.
 * Copyright (c) 1994 Brini.
 * All rights reserved.
 *
 * This code is derived from software written for Brini by Mark Brinicombe
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Brini.
 * 4. The name of the company nor the name of the author may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY BRINI ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL BRINI OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * from: FreeBSD: //depot/projects/arm/src/sys/arm/at91/kb920x_machdep.c, rev 45
 */

#include "opt_ddb.h"
#include "opt_platform.h"
#include "opt_global.h"

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#define _ARM32_BUS_DMA_PRIVATE
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sysproto.h>
#include <sys/signalvar.h>
#include <sys/imgact.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/linker.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/ptrace.h>
#include <sys/cons.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/buf.h>
#include <sys/exec.h>
#include <sys/kdb.h>
#include <sys/msgbuf.h>
#include <machine/reg.h>
#include <machine/cpu.h>
#include <machine/fdt.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_pager.h>
#include <vm/vm_map.h>
#include <machine/pte.h>
#include <machine/pmap.h>
#include <machine/vmparam.h>
#include <machine/pcb.h>
#include <machine/undefined.h>
#include <machine/machdep.h>
#include <machine/metadata.h>
#include <machine/armreg.h>
#include <machine/bus.h>
#include <sys/reboot.h>

#include <arm/omap/omap4/omap44xx_reg.h>

#ifdef  DEBUG
#define debugf(fmt, args...) printf(fmt, ##args)
#else
#define debugf(fmt, args...)
#endif

/*
 * This is the number of L2 page tables required for covering max
 * (hypothetical) memsize of 4GB and all kernel mappings (vectors, msgbuf,
 * stacks etc.), uprounded to be divisible by 4.
 */
#define KERNEL_PT_MAX	78

/* Define various stack sizes in pages */
#define IRQ_STACK_SIZE	1
#define ABT_STACK_SIZE	1
#define UND_STACK_SIZE	1

extern unsigned char kernbase[];
extern unsigned char _etext[];
extern unsigned char _edata[];
extern unsigned char __bss_start[];
extern unsigned char _end[];

#ifdef DDB
extern vm_offset_t ksym_start, ksym_end;
#endif

extern u_int data_abort_handler_address;
extern u_int prefetch_abort_handler_address;
extern u_int undefined_handler_address;

extern vm_offset_t pmap_bootstrap_lastaddr;
extern int *end;

struct pv_addr kernel_pt_table[KERNEL_PT_MAX];

/* Physical and virtual addresses for some global pages */
vm_paddr_t phys_avail[10];
vm_paddr_t dump_avail[4];
vm_offset_t physical_pages;
vm_offset_t pmap_bootstrap_lastaddr;
vm_paddr_t pmap_pa;

const struct pmap_devmap *pmap_devmap_bootstrap_table;
struct pv_addr systempage;
struct pv_addr msgbufpv;
struct pv_addr irqstack;
struct pv_addr undstack;
struct pv_addr abtstack;
struct pv_addr kernelstack;

void set_stackptrs(int cpu);

static struct trapframe proc0_tf;

static struct mem_region availmem_regions[FDT_MEM_REGIONS];
static int availmem_regions_sz;

static void print_kenv(void);
static void print_kernel_section_addr(void);

static void physmap_init(void);
static int platform_devmap_init(void);

/*
 *  Early Print 
 */
#define DEBUGBUF_SIZE 256
#define LSR_THRE    0x20	/* Xmit holding register empty */
#if defined(SOC_TI_AM335X)
#define EARLY_UART_VA_BASE	0xE4E09000
#define EARLY_UART_PA_BASE	0x44E09000
#elif defined(SOC_OMAP4)
#define EARLY_UART_VA_BASE	0xE8020000
#define EARLY_UART_PA_BASE	0x48020000
#else
#error "DAMN"
#endif
char debugbuf[DEBUGBUF_SIZE];

void early_putstr(unsigned char *str)
{
	volatile uint8_t *p_lsr = (volatile uint8_t*) (EARLY_UART_VA_BASE + 0x14);
	volatile uint8_t *p_thr = (volatile uint8_t*) (EARLY_UART_VA_BASE + 0x00);
	
	do {
		while ((*p_lsr & LSR_THRE) == 0);
		*p_thr = *str;

		if (*str == '\n')
		{
			while ((*p_lsr & LSR_THRE) == 0);
			*p_thr = '\r';
		}
	} while (*++str != '\0');
}

#if (STARTUP_PAGETABLE_ADDR < PHYSADDR) || \
    (STARTUP_PAGETABLE_ADDR > (PHYSADDR + (64 * 1024 * 1024)))
#error STARTUP_PAGETABLE_ADDR is not within init. MMU table, early print support not possible
#endif

void
early_print_init(void)
{
	volatile uint32_t *mmu_tbl = (volatile uint32_t*)STARTUP_PAGETABLE_ADDR;
	mmu_tbl[(EARLY_UART_VA_BASE >> L1_S_SHIFT)] = L1_TYPE_S | L1_S_AP(AP_KRW) | (EARLY_UART_PA_BASE & L1_S_FRAME);
	__asm __volatile ("mcr	p15, 0, r0, c8, c7, 0");	/* invalidate I+D TLBs */
	__asm __volatile ("mcr	p15, 0, r0, c7, c10, 4");	/* drain the write buffer */
	early_putstr("Early printf initialise\n");
}

#define EPRINTF(args...) \
	snprintf(debugbuf,DEBUGBUF_SIZE, ##args ); \
	early_putstr(debugbuf);

void early_a(uint8_t x) {
	char c[5];
	c[0]='a';
	c[1]='0'+x;
	c[2]='-';
	c[3]='\n';
	c[3]=0;
	early_putstr(c);
}

void
dump_l2pagetable(uint32_t pta, uint32_t l1)
{
	int i;
	volatile uint32_t *pt = (volatile uint32_t*)pta;

	for (i=0; i<256;i++) {
		switch (pt[i] & 0x3) {
			case 1:
				EPRINTF("0x%08x -> 0x%08x 64K ",(i<<12) | l1,
					pt[i]&0xFFFF0000);
				EPRINTF("l2pt[0x%03x]=0x%08x ",i, pt[i]);
				EPRINTF("s=%u ",	(pt[i]>>10) &0x1);
				EPRINTF("apx=%u ",	(pt[i]>> 9) &0x1);
				EPRINTF("tex=%u ",	(pt[i]>>12) &0x7);
				EPRINTF("ap=%u ",	(pt[i]>> 4) &0x3);
				EPRINTF("c=%u ",	(pt[i]>> 3) &0x1);
				EPRINTF("b=%u\n",	(pt[i]>> 2) &0x1);
				break;
			case 2:
			case 3:
				EPRINTF("0x%08x -> 0x%08x  4K ",(i<<12) | l1,
					pt[i]&0xFFFFF000);
				EPRINTF("l2pt[0x%03x]=0x%08x ",i, pt[i]);
				EPRINTF("s=%u ",	(pt[i]>>10) &0x1);
				EPRINTF("apx=%u ",	(pt[i]>> 9) &0x1);
				EPRINTF("tex=%u ",	(pt[i]>> 6) &0x7);
				EPRINTF("ap=%u ",	(pt[i]>> 4) &0x3);
				EPRINTF("c=%u ",	(pt[i]>> 3) &0x1);
				EPRINTF("b=%u\n",	(pt[i]>> 2) &0x1);
				break;
		}
	}
}

void
dump_l1pagetable(uint32_t pta)
{
	int i;
	EPRINTF("L1 pagetable starts at 0x%08x\n",pta);
	volatile uint32_t *pt = (volatile uint32_t*)pta;
	for (i=0; i<4096;i++) {
		switch (pt[i] & 0x3) {
			case 1:
				EPRINTF("0x%08x ->             L2 ",i<<20);
				EPRINTF("l1pt[0x%03x]=0x%08x ",i, pt[i]);
				EPRINTF("l2desc=0x%08x ",pt[i] & 0xFFFFFC00);
				EPRINTF("p=%u ",(pt[i]>>9) &0x1);
				EPRINTF("domain=0x%x\n",(pt[i]>>5) &0xF);
				//dump_l2pagetable(pt[i] & 0xFFFFFC00, i<<20);
				break;
			case 2:
				if (pt[i] &0x40000) {
					EPRINTF("0x%08x -> 0x%08x 16M ",i<<20, pt[i] & 0xFF000000);
					EPRINTF("l1pt[0x%03x]=0x%08x ",i, pt[i]);
					EPRINTF("base=0x%02x ", ((pt[i]>>24)));
				} else {
					EPRINTF("0x%08x -> 0x%08x  1M ",i<<20, pt[i] & 0xFFF00000);
					EPRINTF("l1pt[0x%03x]=0x%08x ",i, pt[i]);
					EPRINTF("base=0x%03x ", (pt[i]>>20));
				}
				EPRINTF("nG=%u ",	(pt[i]>>17) &0x1);
				EPRINTF("s=%u ",	(pt[i]>>16) &0x1);
				EPRINTF("apx=%u ",	(pt[i]>>15) &0x1);
				EPRINTF("tex=%u ",	(pt[i]>>12) &0x7);
				EPRINTF("ap=%u ",	(pt[i]>>10) &0x3);
				EPRINTF("p=%u ",	(pt[i]>> 9) &0x1);
				EPRINTF("domain=0x%x ",	(pt[i]>> 5) &0xF);
				EPRINTF("xn=%u ",	(pt[i]>> 4) &0x1);
				EPRINTF("c=%u ",	(pt[i]>> 3) &0x1);
				EPRINTF("b=%u\n",	(pt[i]>> 2) &0x1);
				break;
			case 3:
				EPRINTF("pt[0x%03x] 0x%08x RESV\n",i, pt[i]);
				break;
		}
	}
}

static char *
kenv_next(char *cp)
{

	if (cp != NULL) {
		while (*cp != 0)
			cp++;
		cp++;
		if (*cp == 0)
			cp = NULL;
	}
	return (cp);
}

static void
print_kenv(void)
{
	int len;
	char *cp;

	debugf("loader passed (static) kenv:\n");
	if (kern_envp == NULL) {
		debugf(" no env, null ptr\n");
		return;
	}
	debugf(" kern_envp = 0x%08x\n", (uint32_t)kern_envp);

	len = 0;
	for (cp = kern_envp; cp != NULL; cp = kenv_next(cp))
		debugf(" %x %s\n", (uint32_t)cp, cp);
}

static void
print_kernel_section_addr(void)
{

	debugf("kernel image addresses:\n");
	debugf(" kernbase       = 0x%08x\n", (uint32_t)kernbase);
	debugf(" _etext (sdata) = 0x%08x\n", (uint32_t)_etext);
	debugf(" _edata         = 0x%08x\n", (uint32_t)_edata);
	debugf(" __bss_start    = 0x%08x\n", (uint32_t)__bss_start);
	debugf(" _end           = 0x%08x\n", (uint32_t)_end);
}

static void
physmap_init(void)
{
	int i, j, cnt;
	vm_offset_t phys_kernelend, kernload;
	uint32_t s, e, sz;
	struct mem_region *mp, *mp1;

	phys_kernelend = KERNPHYSADDR + (virtual_avail - KERNVIRTADDR);
	kernload = KERNPHYSADDR;

	/*
	 * Remove kernel physical address range from avail
	 * regions list. Page align all regions.
	 * Non-page aligned memory isn't very interesting to us.
	 * Also, sort the entries for ascending addresses.
	 */
	sz = 0;
	cnt = availmem_regions_sz;
	debugf("processing avail regions:\n");
	for (mp = availmem_regions; mp->mr_size; mp++) {
		s = mp->mr_start;
		e = mp->mr_start + mp->mr_size;
		debugf(" %08x-%08x -> ", s, e);
		/* Check whether this region holds all of the kernel. */
		if (s < kernload && e > phys_kernelend) {
			availmem_regions[cnt].mr_start = phys_kernelend;
			availmem_regions[cnt++].mr_size = e - phys_kernelend;
			e = kernload;
		}
		/* Look whether this regions starts within the kernel. */
		if (s >= kernload && s < phys_kernelend) {
			if (e <= phys_kernelend)
				goto empty;
			s = phys_kernelend;
		}
		/* Now look whether this region ends within the kernel. */
		if (e > kernload && e <= phys_kernelend) {
			if (s >= kernload) {
				goto empty;
			}
			e = kernload;
		}
		/* Now page align the start and size of the region. */
		s = round_page(s);
		e = trunc_page(e);
		if (e < s)
			e = s;
		sz = e - s;
		debugf("%08x-%08x = %x\n", s, e, sz);

		/* Check whether some memory is left here. */
		if (sz == 0) {
		empty:
			printf("skipping\n");
			bcopy(mp + 1, mp,
			    (cnt - (mp - availmem_regions)) * sizeof(*mp));
			cnt--;
			mp--;
			continue;
		}

		/* Do an insertion sort. */
		for (mp1 = availmem_regions; mp1 < mp; mp1++)
			if (s < mp1->mr_start)
				break;
		if (mp1 < mp) {
			bcopy(mp1, mp1 + 1, (char *)mp - (char *)mp1);
			mp1->mr_start = s;
			mp1->mr_size = sz;
		} else {
			mp->mr_start = s;
			mp->mr_size = sz;
		}
	}
	availmem_regions_sz = cnt;

	/* Fill in phys_avail table, based on availmem_regions */
	debugf("fill in phys_avail:\n");
	for (i = 0, j = 0; i < availmem_regions_sz; i++, j += 2) {

		debugf(" region: 0x%08x - 0x%08x (0x%08x)\n",
		    availmem_regions[i].mr_start,
		    availmem_regions[i].mr_start + availmem_regions[i].mr_size,
		    availmem_regions[i].mr_size);

		phys_avail[j] = availmem_regions[i].mr_start;
		phys_avail[j + 1] = availmem_regions[i].mr_start +
		    availmem_regions[i].mr_size;
	}
	phys_avail[j] = 0;
	phys_avail[j + 1] = 0;
}

void CP15ICacheFlush(void)
{
    __asm __volatile("    mov     r0, #0\n\t" 
        "    mcr     p15, #0, r0, c7, c5, #0\n\t");
}

void CP15Ttb0Set(unsigned int ttb)
{
   /* sets translation table base resgister with page table 
    * starting address.
    */ 
    __asm __volatile("   mcr p15, #0, %[value], c2, c0, 0":: [value] "r" (ttb));
//    __asm __volatile("   mcr p15, #0, %[value], c2, c0, 0":: [value] "r" (ttb));
    __asm __volatile("   dsb");
    __asm __volatile("   isb");

}

void CP15BranchPredictorInvalidate(void)
{
    __asm __volatile("    mcr p15, #0, r0, c7, c5, #6"); 
}

void CP15BranchPredictionEnable(void)
{
    __asm __volatile("    mrc p15, #0, r0, c1, c0, #0\n\t"
        "    orr r0, r0, #0x00000800\n\t"
        "    mcr p15, #0, r0, c1, c0, #0\n\t");
}

void CP15DomainAccessClientSet(void)
{
    __asm __volatile("    ldr r0, =0x55555555 \n\t"
        "    mcr p15, 0, r0, c3, c0, 0"); 
}

void CP15TlbInvalidate(void)
{
   /* Invalidates all TLBs.Domain access is selected as
    * client by configuring domain access register,
    * in that case access controlled by permission value
    * set by page table entry
    */  
    __asm __volatile("   mov r1, #0\n\t"
        "   mcr p15, #0, r1, c8, c7, #0\n\t"
        "   ldr r1, =0x55555555\n\t"
        "   mcr p15, #0, r1, c3, c0, #0\n\t");
}


void CP15ICacheEnable(void)
{
    __asm __volatile("    mrc     p15, #0, r0, c1, c0, #0\n\t"
        "    orr     r0,  r0, #0x00001000 \n\t"
        "    mcr     p15, #0, r0, c1, c0, #0 \n\t");
}



void *
initarm(void *mdp, void *unused __unused)
{
	struct pv_addr kernel_l1pt;
	struct pv_addr dpcpu;
	vm_offset_t dtbp, freemempos, l2_start, lastaddr;
	uint32_t memsize, l2size;
	void *kmdp;
	u_int l1pagetable;
	int i = 0, j = 0;

	kmdp = NULL;
	lastaddr = 0;
	memsize = 0;
	dtbp = (vm_offset_t)NULL;

	early_print_init();
	set_cpufuncs();

	/*
	 * Mask metadata pointer: it is supposed to be on page boundary. If
	 * the first argument (mdp) doesn't point to a valid address the
	 * bootloader must have passed us something else than the metadata
	 * ptr... In this case we want to fall back to some built-in settings.
	 */
	mdp = (void *)((uint32_t)mdp & ~PAGE_MASK);

	/* Parse metadata and fetch parameters */
	if (mdp != NULL) {
		preload_metadata = mdp;
		kmdp = preload_search_by_type("elf kernel");
		if (kmdp != NULL) {
			boothowto = MD_FETCH(kmdp, MODINFOMD_HOWTO, int);
			kern_envp = MD_FETCH(kmdp, MODINFOMD_ENVP, char *);
			dtbp = MD_FETCH(kmdp, MODINFOMD_DTBP, vm_offset_t);
			lastaddr = MD_FETCH(kmdp, MODINFOMD_KERNEND,
			    vm_offset_t);
#ifdef DDB
			ksym_start = MD_FETCH(kmdp, MODINFOMD_SSYM, uintptr_t);
			ksym_end = MD_FETCH(kmdp, MODINFOMD_ESYM, uintptr_t);
#endif
		}

		preload_addr_relocate = KERNVIRTADDR - KERNPHYSADDR;
	} else {
		/* Fall back to hardcoded metadata. */
		lastaddr = fake_preload_metadata();
	}

#if defined(FDT_DTB_STATIC)
	/*
	 * In case the device tree blob was not retrieved (from metadata) try
	 * to use the statically embedded one.
	 */
	if (dtbp == (vm_offset_t)NULL)
		dtbp = (vm_offset_t)&fdt_static_dtb;
#endif

	if (OF_install(OFW_FDT, 0) == FALSE)
		while (1);

	EPRINTF("x1\n");
	if (OF_init((void *)dtbp) != 0)
		while (1);
	EPRINTF("x2\n");

	/* Grab physical memory regions information from device tree. */
	if (fdt_get_mem_regions(availmem_regions, &availmem_regions_sz,
	    &memsize) != 0)
		while(1);
	EPRINTF("x3\n");

//	if (fdt_immr_addr(OMAP44XX_L4_PERIPH_VBASE) != 0)
//		while (1);

	/* Platform-specific initialisation FIXME */
	pmap_bootstrap_lastaddr = 0xE0000000 - ARM_NOCACHE_KVA_SIZE;

	pcpu0_init();
	EPRINTF("x4\n");

	/* Calculate number of L2 tables needed for mapping vm_page_array */
	l2size = (memsize / PAGE_SIZE) * sizeof(struct vm_page);
	l2size = (l2size >> L1_S_SHIFT) + 1;

	/*
	 * Add one table for end of kernel map, one for stacks, msgbuf and
	 * L1 and L2 tables map and one for vectors map.
	 */
	l2size += 3;

	/* Make it divisible by 4 */
	l2size = (l2size + 3) & ~3;

#define KERNEL_TEXT_BASE (KERNBASE)
	freemempos = (lastaddr + PAGE_MASK) & ~PAGE_MASK;

	/* Define a macro to simplify memory allocation */
#define valloc_pages(var, np)                   \
	alloc_pages((var).pv_va, (np));         \
	(var).pv_pa = (var).pv_va + (KERNPHYSADDR - KERNVIRTADDR);

#define alloc_pages(var, np)			\
	(var) = freemempos;		\
	freemempos += (np * PAGE_SIZE);		\
	memset((char *)(var), 0, ((np) * PAGE_SIZE));

	while (((freemempos - L1_TABLE_SIZE) & (L1_TABLE_SIZE - 1)) != 0)
		freemempos += PAGE_SIZE;
	valloc_pages(kernel_l1pt, L1_TABLE_SIZE / PAGE_SIZE);

	for (i = 0; i < l2size; ++i) {
		if (!(i % (PAGE_SIZE / L2_TABLE_SIZE_REAL))) {
			valloc_pages(kernel_pt_table[i],
			    L2_TABLE_SIZE / PAGE_SIZE);
			j = i;
		} else {
			kernel_pt_table[i].pv_va = kernel_pt_table[j].pv_va +
			    L2_TABLE_SIZE_REAL * (i - j);
			kernel_pt_table[i].pv_pa =
			    kernel_pt_table[i].pv_va - KERNVIRTADDR +
			    KERNPHYSADDR;

		}
	}
	/*
	 * Allocate a page for the system page mapped to 0x00000000
	 * or 0xffff0000. This page will just contain the system vectors
	 * and can be shared by all processes.
	 */
	valloc_pages(systempage, 1);

	/* Allocate dynamic per-cpu area. */
	valloc_pages(dpcpu, DPCPU_SIZE / PAGE_SIZE);
	dpcpu_init((void *)dpcpu.pv_va, 0);
	EPRINTF("x5\n");

	/* Allocate stacks for all modes */
	valloc_pages(irqstack, (IRQ_STACK_SIZE * MAXCPU));
	valloc_pages(abtstack, (ABT_STACK_SIZE * MAXCPU));
	valloc_pages(undstack, (UND_STACK_SIZE * MAXCPU));
	valloc_pages(kernelstack, (KSTACK_PAGES * MAXCPU));

	init_param1();

	valloc_pages(msgbufpv, round_page(msgbufsize) / PAGE_SIZE);

	/*
	 * Now we start construction of the L1 page table
	 * We start by mapping the L2 page tables into the L1.
	 * This means that we can replace L1 mappings later on if necessary
	 */
	l1pagetable = kernel_l1pt.pv_va;

	/*
	 * Try to map as much as possible of kernel text and data using
	 * 1MB section mapping and for the rest of initial kernel address
	 * space use L2 coarse tables.
	 *
	 * Link L2 tables for mapping remainder of kernel (modulo 1MB)
	 * and kernel structures
	 */
	l2_start = lastaddr & ~(L1_S_OFFSET);
	for (i = 0 ; i < l2size - 1; i++)
		pmap_link_l2pt(l1pagetable, l2_start + i * L1_S_SIZE,
		    &kernel_pt_table[i]);
	EPRINTF("x6\n");

	pmap_curmaxkvaddr = l2_start + (l2size - 1) * L1_S_SIZE;
	
	/* Map kernel code and data */
	pmap_map_chunk(l1pagetable, KERNVIRTADDR, KERNPHYSADDR,
	   (((uint32_t)(lastaddr) - KERNVIRTADDR) + PAGE_MASK) & ~PAGE_MASK,
	    VM_PROT_READ|VM_PROT_WRITE, PTE_CACHE);
	EPRINTF("x7\n");


	/* Map L1 directory and allocated L2 page tables */
	pmap_map_chunk(l1pagetable, kernel_l1pt.pv_va, kernel_l1pt.pv_pa,
	    L1_TABLE_SIZE, VM_PROT_READ|VM_PROT_WRITE, PTE_PAGETABLE);
	EPRINTF("x8\n");

	pmap_map_chunk(l1pagetable, kernel_pt_table[0].pv_va,
	    kernel_pt_table[0].pv_pa,
	    L2_TABLE_SIZE_REAL * l2size,
	    VM_PROT_READ|VM_PROT_WRITE, PTE_PAGETABLE);
	EPRINTF("x9\n");

	/* Map allocated DPCPU, stacks and msgbuf */
	pmap_map_chunk(l1pagetable, dpcpu.pv_va, dpcpu.pv_pa,
	    freemempos - dpcpu.pv_va,
	    VM_PROT_READ|VM_PROT_WRITE, PTE_CACHE);
	EPRINTF("x10\n");

	/* Link and map the vector page */
	pmap_link_l2pt(l1pagetable, ARM_VECTORS_HIGH,
	    &kernel_pt_table[l2size - 1]);
	pmap_map_entry(l1pagetable, ARM_VECTORS_HIGH, systempage.pv_pa,
	    VM_PROT_READ|VM_PROT_WRITE|VM_PROT_EXECUTE, PTE_CACHE);
	EPRINTF("x11\n");

	/* Map pmap_devmap[] entries */
	if (platform_devmap_init() != 0)
		while (1);
	EPRINTF("x12\n");
	
	pmap_devmap_bootstrap(l1pagetable, pmap_devmap_bootstrap_table);
	EPRINTF("x13\n");

	cpu_domains((DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL * 2)) |
	    DOMAIN_CLIENT);
	EPRINTF("x14 kernel_l1pt.pv_pa=0x%08x\n", kernel_l1pt.pv_pa);
	pmap_pa = kernel_l1pt.pv_pa;

	EPRINTF("x14.xxx arm_cache_level=0x%08x arm_cache_loc=%u\n",
		arm_cache_level, arm_cache_loc);
	int u=0;
	for(u=0;u<14;u++) {
		EPRINTF("arm_cache_type[%u]=0x%08x\n",u,arm_cache_type[u]);
	}

	uint32_t cr,acr, l2cacr, myttb,xx,yy;
	__asm __volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(cr));
	__asm __volatile("mrc p15, 0, %0, c1, c0, 1" : "=r"(acr));
	__asm __volatile("mrc p15, 1, %0, c9, c0, 2" : "=r"(l2cacr));
	EPRINTF("CR     = 0x%08x\n",cr);
	EPRINTF("ACR    = 0x%08x\n",acr);
	EPRINTF("L2CACR = 0x%08x\n",l2cacr);
	cr &= ~(1<<2);
	acr &= ~(1<<1);
	EPRINTF("CR     = 0x%08x\n",cr);
	EPRINTF("ACR    = 0x%08x\n",acr);
	EPRINTF("L2CACR = 0x%08x\n",l2cacr);

	xx=0;
	__asm __volatile("mcr    p15, 2, %0, c0, c0, 0" : : "r" (xx));
	__asm __volatile("isb");
	__asm __volatile("mrc    p15, 1, %0, c0, c0, 0" : "=r" (yy)); // CSIDR
	EPRINTF("CSIDR[%x]=0x%08x\n",xx,yy);
	xx=1;
	__asm __volatile("mcr    p15, 2, %0, c0, c0, 0" : : "r" (xx));
	__asm __volatile("isb");
	__asm __volatile("mrc    p15, 1, %0, c0, c0, 0" : "=r" (yy)); // CSIDR
	EPRINTF("CSIDR[%x]=0x%08x\n",xx,yy);
	xx=2;
	__asm __volatile("mcr    p15, 2, %0, c0, c0, 0" : : "r" (xx));
	__asm __volatile("isb");
	__asm __volatile("mrc    p15, 1, %0, c0, c0, 0" : "=r" (yy)); // CSIDR
	EPRINTF("CSIDR[%x]=0x%08x\n",xx,yy);
	xx=3;
	__asm __volatile("mcr    p15, 2, %0, c0, c0, 0" : : "r" (xx));
	__asm __volatile("isb");
	__asm __volatile("mrc    p15, 1, %0, c0, c0, 0" : "=r" (yy)); // CSIDR
	EPRINTF("CSIDR[%x]=0x%08x\n",xx,yy);
	

	CP15TlbInvalidate();
        CP15BranchPredictorInvalidate();
       CP15BranchPredictionEnable();
       CP15DomainAccessClientSet();
       CP15ICacheFlush();
       CP15ICacheEnable();
       early_putstr("OK44\n");
	
//	__asm __volatile("mcr p15, 0, %0, c1, c0, 0" : "=r"(cr));
//	__asm __volatile("mcr p15, 0, %0, c1, c0, 1" : "=r"(acr));
//	cr |= 1<<2;
//	EPRINTF("CR = 0x%08x\n",cr);
//	__asm __volatile("mcr p15, 0, %0, c1, c0, 0" : "=r"(cr));

//	__asm __volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(cr));
//	EPRINTF("CR = 0x%08x\n",cr);

//	__asm __volatile("mrc p15, 1, %0, c9, c0, 2" : "=r"(clidr));
//	EPRINTF("L2CACR = 0x%08x\n",clidr);

        __asm __volatile("   mrc p15, #0, %0, c2, c0, 0" : "=r" (myttb));
//	dump_l1pagetable(STARTUP_PAGETABLE_ADDR);
//	dump_l1pagetable(l1pagetable);
	early_putstr("armv7_dcache_wbinv_all\n");
	armv7_dcache_wbinv_all();
	EPRINTF("TTB=0x%08x\n",myttb);
//	CP15TlbInvalidate();
	setttb(kernel_l1pt.pv_pa);
	EPRINTF("x15 OK\n");
	EPRINTF("TTB_0x%08x\n",myttb);
//	cpu_tlb_flushID();
	EPRINTF("x16\n");
	EPRINTF("kernel_l1pt.pv_pa=0x%08x\n",kernel_l1pt.pv_pa);
        __asm __volatile("   mrc p15, #0, %0, c2, c0, 0" : "=r" (myttb));
	EPRINTF("TTB=0x%08x\n",myttb);
	cpu_domains(DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL * 2));
	EPRINTF("x17\n");

	/*
	 * Only after the SOC registers block is mapped we can perform device
	 * tree fixups, as they may attempt to read parameters from hardware.
	 */
	OF_interpret("perform-fixup", 0);
	EPRINTF("cninit\n");
	cninit();
	EPRINTF("cninit done\n");

	physmem = memsize / PAGE_SIZE;

	debugf("initarm: console initialized\n");
	debugf(" arg1 mdp = 0x%08x\n", (uint32_t)mdp);
	debugf(" boothowto = 0x%08x\n", boothowto);
	debugf(" dtbp = 0x%08x\n", (uint32_t)dtbp);
	EPRINTF("cninit done1\n");
	print_kernel_section_addr();
	EPRINTF("cninit done2\n");
	print_kenv();
	EPRINTF("cninit done3\n");

	/*
	 * Pages were allocated during the secondary bootstrap for the
	 * stacks for different CPU modes.
	 * We must now set the r13 registers in the different CPU modes to
	 * point to these stacks.
	 * Since the ARM stacks use STMFD etc. we must set r13 to the top end
	 * of the stack memory.
	 */
	cpu_control(CPU_CONTROL_MMU_ENABLE, CPU_CONTROL_MMU_ENABLE);
 	EPRINTF("cninit done4\n");

	set_stackptrs(0);

	/*
	 * We must now clean the cache again....
	 * Cleaning may be done by reading new data to displace any
	 * dirty data in the cache. This will have happened in setttb()
	 * but since we are boot strapping the addresses used for the read
	 * may have just been remapped and thus the cache could be out
	 * of sync. A re-clean after the switch will cure this.
	 * After booting there are no gross relocations of the kernel thus
	 * this problem will not occur after initarm().
	 */
	cpu_idcache_wbinv_all();

	/* Set stack for exception handlers */
	data_abort_handler_address = (u_int)data_abort_handler;
	prefetch_abort_handler_address = (u_int)prefetch_abort_handler;
	undefined_handler_address = (u_int)undefinedinstruction_bounce;
	undefined_init();

	proc_linkup0(&proc0, &thread0);
	thread0.td_kstack = kernelstack.pv_va;
	thread0.td_kstack_pages = KSTACK_PAGES;
	thread0.td_pcb = (struct pcb *)
	    (thread0.td_kstack + KSTACK_PAGES * PAGE_SIZE) - 1;
	thread0.td_pcb->pcb_flags = 0;
	thread0.td_frame = &proc0_tf;
	pcpup->pc_curpcb = thread0.td_pcb;

	arm_vector_init(ARM_VECTORS_HIGH, ARM_VEC_ALL);

	dump_avail[0] = 0;
	dump_avail[1] = memsize;
	dump_avail[2] = 0;
	dump_avail[3] = 0;

	pmap_bootstrap(freemempos, pmap_bootstrap_lastaddr, &kernel_l1pt);
	msgbufp = (void *)msgbufpv.pv_va;
	msgbufinit(msgbufp, msgbufsize);
	mutex_init();

	/*
	 * Prepare map of physical memory regions available to vm subsystem.
	 */
	physmap_init();

	/* Do basic tuning, hz etc */
	init_param2(physmem);
	kdb_init();
	return ((void *)(kernelstack.pv_va + USPACE_SVC_STACK_TOP -
	    sizeof(struct pcb)));
}

void
set_stackptrs(int cpu)
{

	set_stackptr(PSR_IRQ32_MODE,
	    irqstack.pv_va + ((IRQ_STACK_SIZE * PAGE_SIZE) * (cpu + 1)));
	set_stackptr(PSR_ABT32_MODE,
	    abtstack.pv_va + ((ABT_STACK_SIZE * PAGE_SIZE) * (cpu + 1)));
	set_stackptr(PSR_UND32_MODE,
	    undstack.pv_va + ((UND_STACK_SIZE * PAGE_SIZE) * (cpu + 1)));
}

#define FDT_DEVMAP_MAX	(2)		// FIXME
static struct pmap_devmap fdt_devmap[FDT_DEVMAP_MAX] = {
	{ 0, 0, 0, 0, 0, }
};


/*
 * Construct pmap_devmap[] with DT-derived config data.
 */
static int
platform_devmap_init(void)
{
	int i = 0;
#if defined(SOC_OMAP4)
	fdt_devmap[i].pd_va = 0xE8000000;
	fdt_devmap[i].pd_pa = 0x48000000;
	fdt_devmap[i].pd_size = 0x1000000;
	fdt_devmap[i].pd_prot = VM_PROT_READ | VM_PROT_WRITE;
	fdt_devmap[i].pd_cache = PTE_NOCACHE;
	i++;
#elif defined(SOC_TI_AM335X)
	fdt_devmap[i].pd_va = 0xE4C00000;
	fdt_devmap[i].pd_pa = 0x44C00000;	/* L4_WKUP */
	fdt_devmap[i].pd_size = 0x400000;	/* 4 MB */
	fdt_devmap[i].pd_prot = VM_PROT_READ | VM_PROT_WRITE;
	fdt_devmap[i].pd_cache = PTE_NOCACHE;
	i++;
#endif

	pmap_devmap_bootstrap_table = &fdt_devmap[0];
	return (0);
}

struct arm32_dma_range *
bus_dma_get_range(void)
{

	return (NULL);
}

int
bus_dma_get_range_nb(void)
{

	return (0);
}

void
cpu_reset()
{
//	omap_prcm_reset();
	printf("Reset failed!\n");
	while (1);
}

