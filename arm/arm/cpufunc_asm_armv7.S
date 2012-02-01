/*-
 * Copyright (C) 2011 MARVELL INTERNATIONAL LTD.
 * All rights reserved.
 *
 * Developed by Semihalf.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of MARVELL nor the names of contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <machine/asm.h>
__FBSDID("$FreeBSD$");

#define TTB (0x59)

.Lcoherency_level:
	.word	_C_LABEL(arm_cache_loc)
.Lcache_type:
	.word	_C_LABEL(arm_cache_type)
.Lway_mask:
	.word	0x3ff
.Lmax_index:
	.word	0x7fff
.Lpage_mask:
	.word	0xfff

ENTRY(armv7_setttb)
	stmdb   sp!, {r0, lr}
 	bl      _C_LABEL(armv7_idcache_wbinv_all) /* clean the D cache */
 	ldmia   sp!, {r0, lr}
// 	mcr     p15, 0, r0, c8, c7, 0   /* invalidate I+D TLBs */
// 	dsb
//	isb
 	mcr	p15, 0, r0, c2, c0, 0	/* Translation Table Base Register 0 (TTBR0) */
 	dsb
 	isb
	RET

ENTRY(armv7_tlb_flushID)
	dsb
	mcr	p15, 0, r0, c8, c7, 0	/* flush I+D tlb */
	mcr	p15, 0, r0, c7, c5, 6	/* flush BTB */
	dsb
	isb
	mov	pc, lr

ENTRY(armv7_tlb_flushID_SE)
	ldr	r1, .Lpage_mask
	bic	r0, r0, r1
	mcr	p15, 0, r0, c8, c7, 1	/* flush D tlb single entry */
	mcr	p15, 0, r0, c7, c5, 6	/* flush BTB */
	dsb
	mov	pc, lr

/* Based on algorithm from ARM Architecture Reference Manual */
ENTRY(armv7_dcache_wbinv_all)
	stmdb	sp!, {r4, r5, r6, r7, r8, r9}

	/* Get cache level */
	ldr	r0, .Lcoherency_level
	ldr	r3, [r0]
	cmp	r3, #0
	beq	Finished
	/* For each cache level */
	mov	r8, #0
Loop1:
	/* Get cache type for given level */
	mov	r2, r8, lsl #2
	ldr	r0, .Lcache_type
	ldr	r1, [r0, r2]

	/* Get line size */
	and	r2, r1, #7
	add	r2, r2, #4

	/* Get number of ways */
	ldr	r4, .Lway_mask
	ands	r4, r4, r1, lsr #3
	clz	r5, r4

	/* Get max index */
	ldr	r7, .Lmax_index
	ands	r7, r7, r1, lsr #13
Loop2:
	mov	r9, r4
Loop3:
	mov	r6, r8, lsl #1
	orr	r6, r6, r9, lsl r5
	orr	r6, r6, r7, lsl r2

	/* Clean and invalidate data cache by way/index */
	mcr	p15, 0, r6, c7, c14, 2
	subs	r9, r9, #1
	bge	Loop3
	subs	r7, r7, #1
	bge	Loop2
Skip:
	add	r8, r8, #1
	cmp	r3, r8
	bne Loop1
Finished:
	isb
	mcr	p15, 0, r0, c7, c5, 5
	ldmia	sp!, {r4, r5, r6, r7, r8, r9}
	RET

ENTRY(armv7_dcache_wbinv_all2)
	push    {r4-r11}
	dmb
	mrc    p15, 1, r0, c0, c0, 1		/* Load CLID register */
	ands    r3, r0, #0x7000000		/* Get LoC */
	mov    r3, r3, lsr #23
	beq    done				/* if LoC is 0, then no need to clean */
	mov    r10, #0				/* start cleaning at cache level 0 */
loop1:	add    r2, r10, r10, lsr #1		/* work out 3x current cache level */
	mov    r1, r0, lsr r2			/* get cache type */
	and    r1, r1, #7			/* mask of the bits for current cache only*/
	cmp    r1, #2				/* cache at this level */
	blt    skip				/* skip if no cache, or just i-cache */
	mcr    p15, 2, r10, c0, c0, 0		/* select current cache level */
	isb					/* isb to sych the new CSSR & CSIDR */
	mrc    p15, 1, r1, c0, c0, 0		/* read the new CSIDR */
	and    r2, r1, #7			/* extract the length of the cache lines */
	add    r2, r2, #4
	ldr    r4, =0x3ff
	ands   r4, r4, r1, lsr #3		/* Max on the way size */
	clz    r5, r4				/* find bit position of way size increment */
	ldr    r7, =0x7fff
	ands    r7, r7, r1, lsr #13		/* Max number of the index size */
loop2:	mov     r9, r4
loop3:	orr     r11, r10, r9, lsl r5		/* factor way and cache number */
	orr    r11, r11, r7, lsl r2		/* factor index number */
	mcr    p15, 0, r11, c7, c14, 2		/* clean & invalidate by set/way */
	subs    r9, r9, #1			/* decrement the way */
	bge     loop3
	subs    r7, r7, #1			/* decrement the index */
	bge     loop2
skip:	add     r10, r10, #2			/*  increment cache number */
	cmp     r3, r10
	bgt     loop1
done:	isb
	pop    {r4-r11}
	RET

ENTRY(armv7_idcache_wbinv_all)
	stmdb	sp!, {lr}
	bl armv7_dcache_wbinv_all2
	mcr	p15, 0, r0, c7, c5, 0	/* Invalidate all I caches to PoU (ICIALLU) */
	dsb
	isb
	ldmia	sp!, {lr}
	RET

/* XXX Temporary set it to 32 for MV cores, however this value should be
 * get from Cache Type register
 */
.Larmv7_line_size:
	.word	32

ENTRY(armv7_dcache_wb_range)
	ldr	ip, .Larmv7_line_size
	sub	r3, ip, #1
	and	r2, r0, r3
	add	r1, r1, r2
	bic	r0, r0, r3
.Larmv7_wb_next:
	mcr	p15, 0, r0, c7, c10, 1	/* Clean D cache SE with VA */
	add	r0, r0, ip
	subs	r1, r1, ip
	bhi	.Larmv7_wb_next
	dsb				/* data synchronization barrier */
	RET

ENTRY(armv7_dcache_wbinv_range)
	ldr	ip, .Larmv7_line_size
	sub     r3, ip, #1
	and     r2, r0, r3
	add     r1, r1, r2
	bic     r0, r0, r3
.Larmv7_wbinv_next:
	mcr	p15, 0, r0, c7, c14, 1	/* Purge D cache SE with VA */
	add	r0, r0, ip
	subs	r1, r1, ip
	bhi	.Larmv7_wbinv_next
	dsb				/* data synchronization barrier */
	RET

/*
 * Note, we must not invalidate everything.  If the range is too big we
 * must use wb-inv of the entire cache.
 */
ENTRY(armv7_dcache_inv_range)
	ldr	ip, .Larmv7_line_size
	sub     r3, ip, #1
	and     r2, r0, r3
	add     r1, r1, r2
	bic     r0, r0, r3
.Larmv7_inv_next:
	mcr	p15, 0, r0, c7, c6, 1	/* Invalidate D cache SE with VA */
	add	r0, r0, ip
	subs	r1, r1, ip
	bhi	.Larmv7_inv_next
	dsb				/* data synchronization barrier */
	RET

ENTRY(armv7_idcache_wbinv_range)
	ldr	ip, .Larmv7_line_size
	sub     r3, ip, #1
	and     r2, r0, r3
	add     r1, r1, r2
	bic     r0, r0, r3
.Larmv7_id_wbinv_next:
	mcr	p15, 0, r0, c7, c5, 1	/* Invalidate I cache SE with VA */
	mcr	p15, 0, r0, c7, c14, 1	/* Purge D cache SE with VA */
	add	r0, r0, ip
	subs	r1, r1, ip
	bhi	.Larmv7_id_wbinv_next
	isb				/* instruction synchronization barrier */
	dsb				/* data synchronization barrier */
	RET

ENTRY_NP(armv7_icache_sync_range)
	ldr	ip, .Larmv7_line_size
.Larmv7_sync_next:
	mcr	p15, 0, r0, c7, c5, 1	/* Invalidate I cache SE with VA */
	mcr	p15, 0, r0, c7, c10, 1	/* Clean D cache SE with VA */
	add	r0, r0, ip
	subs	r1, r1, ip
	bhi	.Larmv7_sync_next
	isb				/* instruction synchronization barrier */
	dsb				/* data synchronization barrier */
	RET

ENTRY(armv7_cpu_sleep)
	dsb				/* data synchronization barrier */
	wfi  				/* wait for interrupt */
	RET

ENTRY(armv7_context_switch)
	dsb
	mcr	p15, 0, r0, c2, c0, 0	/* set the new TTB */
	mcr	p15, 0, r0, c8, c7, 0	/* and flush the I+D tlbs */
	dsb
	isb
	RET

ENTRY(armv7_drain_writebuf)
	dsb
	RET

/* Use Privileged Thread Id register as a holder for pcpu pointer */
ENTRY(get_pcpu)
	mrc p15, 0, r0, c13, c0, 4
	RET

ENTRY(set_pcpu)
	mcr p15, 0, r0, c13, c0, 4
	RET

/* Use Privileged Thread Id register as a holder for tls pointer */
ENTRY(get_tls)
	mrc p15, 0, r0, c13, c0, 3
	RET

ENTRY(set_tls)
	mcr p15, 0, r0, c13, c0, 3
	RET
