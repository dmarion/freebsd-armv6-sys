/*
 Copyright 2001, 2002 Georges Menie (www.menie.org)
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 putchar is the only external dependency for this file,
 if you have a working putchar, just remove the following
 define. If the function should be called something else,
 replace outbyte(c) by your own function call.
 */
#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <machine/pcb.h>
#include <machine/machdep.h>
#include <machine/undefined.h>
#include <machine/pte.h>
#include <machine/bus.h>
#include <machine/intr.h>
#include <sys/kdb.h>

#include <arm/omap/omap3/omap35xx_reg.h>
//#include <arm/omap/omap3/omap3var.h>


static uint32_t uart_vaddr = 0x00000000;

void early_putchar(unsigned char c);
void early_putstr(unsigned char *str);
int early_printf(const char *format, ...);
int early_sprintf(char *out, const char *format, ...);
int early_panic(const char *format, ...);
void early_print_init(uint32_t hw_addr, uint32_t virt_addr);



static void printchar(char **str, int c)
{
	if (str) {
		**str = c;
		++(*str);
	}
	else
	{
		(void)early_putchar(c);
	}
}

#define PAD_RIGHT 1
#define PAD_ZERO 2

static int prints(char **out, const char *string, int width, int pad)
{
	register int pc = 0, padchar = ' ';
	
	if (width > 0) {
		register int len = 0;
		register const char *ptr;
		for (ptr = string; *ptr; ++ptr) ++len;
		if (len >= width) width = 0;
		else width -= len;
		if (pad & PAD_ZERO) padchar = '0';
	}
	if (!(pad & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			printchar (out, padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) {
		printchar (out, *string);
		++pc;
	}
	for ( ; width > 0; --width) {
		printchar (out, padchar);
		++pc;
	}
	
	return pc;
}

/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12

static int printi(char **out, int i, int b, int sg, int width, int pad, int letbase)
{
	char print_buf[PRINT_BUF_LEN];
	register char *s;
	register int t, neg = 0, pc = 0;
	register unsigned int u = i;
	
	if (i == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return prints (out, print_buf, width, pad);
	}
	
	if (sg && b == 10 && i < 0) {
		neg = 1;
		u = -i;
	}
	
	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';
	
	while (u) {
		t = u % b;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= b;
	}
	
	if (neg) {
		if( width && (pad & PAD_ZERO) ) {
			printchar (out, '-');
			++pc;
			--width;
		}
		else {
			*--s = '-';
		}
	}
	
	return pc + prints (out, s, width, pad);
}

static int print(char **out, int *varg)
{
	register int width, pad;
	register int pc = 0;
	register char *format = (char *)(*varg++);
	char scr[2];
	
	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) {
				width *= 10;
				width += *format - '0';
			}
			if( *format == 's' ) {
				register char *s = *((char **)varg++);
				pc += prints (out, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) {
				pc += printi (out, *varg++, 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'x' ) {
				pc += printi (out, *varg++, 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) {
				pc += printi (out, *varg++, 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == 'u' ) {
				pc += printi (out, *varg++, 10, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'c' ) {
				/* char are converted to int then pushed on the stack */
				scr[0] = *varg++;
				scr[1] = '\0';
				pc += prints (out, scr, width, pad);
				continue;
			}
		}
		else {
		out:
			printchar (out, *format);
			++pc;
		}
	}
	if (out) **out = '\0';
	return pc;
}

/* assuming sizeof(void *) == sizeof(int) */

int early_sprintf(char *out, const char *format, ...)
{
	register int *varg = (int *)(&format);
	return print(&out, varg);
}


#ifdef LSR_THRE
#undef LSR_THRE
#endif
#define LSR_THRE    0x20	/* Xmit holding register empty */


void early_putchar(unsigned char c)
{
	volatile uint8_t *p_lsr = (volatile uint8_t*) (uart_vaddr + 0x14);
	volatile uint8_t *p_thr = (volatile uint8_t*) (uart_vaddr + 0x00);
	
	while ((*p_lsr & LSR_THRE) == 0);
	*p_thr = c;
	
	if (c == '\n')
	{
		while ((*p_lsr & LSR_THRE) == 0);
		*p_thr = '\r';
	}
}

void early_putstr(unsigned char *str)
{
	do {
		early_putchar(*str++);
	} while (*str != '\0');
}

int early_printf(const char *format, ...)
{
	register int *varg = (int *)(&format);
	return print(0, varg);
}

int
early_panic(const char *format, ...)
{
	register int *varg = (int *)(&format);
	early_putstr("PANIC: ");
	print(0, varg);
	
	for (;;);
	
	return 0;
}


void
early_print_init(uint32_t hw_addr, uint32_t virt_addr)
{
	volatile uint32_t *mmu_tbl;

	/* First thing is we have to add a basic mapping to the periphal in the
	 * initial MMU page table.
	 *
	 * In locore.S the initial MMU is setup at a location set by
	 * STARTUP_PAGETABLE_ADDR, this is typically at the start of physical
	 * memory however that is not guarenteed.
	 *
	 * We also know that locore.S maps in 64 1MB setions from PHYSADDR, we can
	 * use this to determine if the MMU table is in the mapped memory and if
	 * so we can edit it so that we have a mapping for our UART.
	 */
	#if (STARTUP_PAGETABLE_ADDR < PHYSADDR) || \
	    (STARTUP_PAGETABLE_ADDR > (PHYSADDR + (64 * 1024 * 1024)))
	#error STARTUP_PAGETABLE_ADDR is not withing initial MMU table, early print support not possible
	#endif

	/* Get a pointer to the MMU table */
	mmu_tbl = (volatile uint32_t*)STARTUP_PAGETABLE_ADDR;
	
	/* Insert a 1MB section to the map at the correct place */
	mmu_tbl[(virt_addr >> L1_S_SHIFT)] = L1_TYPE_S | L1_S_AP(AP_KRW) | (hw_addr & L1_S_FRAME);

	/* We know that the locore.S setup all entries (including the page table one)
	 * without caching enabled - so no need to flush the cache now, we just
	 * need to flush the TLB cache.
	 */
	__asm __volatile ("mcr	p15, 0, r0, c8, c7, 0");	/* invalidate I+D TLBs */
	__asm __volatile ("mcr	p15, 0, r0, c7, c10, 4");	/* drain the write buffer */
	
	/* Finally store the H/W address of the device */
	uart_vaddr = virt_addr;
	
	/* Test the interface */
	early_printf("Early printf initialise\n");
}


