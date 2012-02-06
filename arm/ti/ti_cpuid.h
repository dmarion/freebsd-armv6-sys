/*-
 * Copyright (c) 2011
 *	Ben Gray <ben.r.gray@gmail.com>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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

#ifndef _TI_CPUID_H_
#define	_TI_CPUID_H_

#define OMAP_MAKEREV(d, a, b, c) \
	(uint32_t)(((d) << 16) | (((a) & 0xf) << 8) | (((b) & 0xf) << 4) | ((c) & 0xf))

#define OMAP_REV_DEVICE(x)      (((x) >> 16) & 0xffff)
#define OMAP_REV_MAJOR(x)       (((x) >> 8) & 0xf)
#define OMAP_REV_MINOR(x)       (((x) >> 4) & 0xf)
#define OMAP_REV_MINOR_MINOR(x) (((x) >> 0) & 0xf)

#define OMAP3350_REV_ES1_0      OMAP_MAKEREV(0x3530, 1, 0, 0)
#define OMAP3530_REV_ES2_0      OMAP_MAKEREV(0x3530, 2, 0, 0)
#define OMAP3530_REV_ES2_1      OMAP_MAKEREV(0x3530, 2, 1, 0)
#define OMAP3530_REV_ES3_0      OMAP_MAKEREV(0x3530, 3, 0, 0)
#define OMAP3530_REV_ES3_1      OMAP_MAKEREV(0x3530, 3, 1, 0)
#define OMAP3530_REV_ES3_1_2    OMAP_MAKEREV(0x3530, 3, 1, 2)

#define OMAP4430_REV_ES1_0      OMAP_MAKEREV(0x4430, 1, 0, 0)
#define OMAP4430_REV_ES2_0      OMAP_MAKEREV(0x4430, 2, 0, 0)
#define OMAP4430_REV_ES2_1      OMAP_MAKEREV(0x4430, 2, 1, 0)
#define OMAP4430_REV_ES2_2      OMAP_MAKEREV(0x4430, 2, 2, 0)
#define OMAP4430_REV_ES2_3      OMAP_MAKEREV(0x4430, 2, 3, 0)

#define AM335X_DEVREV(x)	((x) >> 28)

#define CHIP_OMAP_3	0
#define CHIP_OMAP_4	1
#define CHIP_AM335X	2

static __inline int ti_chip(void)
{
#if defined(SOC_OMAP4)
	return CHIP_OMAP_4;
#elif defined(SOC_OMAP3)
	return CHIP_OMAP_3;
#elif defined(SOC_TI_AM335X)
	return CHIP_AM335X;
#else
#  error Chip type not defined, ensure SOC_xxxx is defined
#endif
}

uint32_t ti_revision(void);

#endif  /* _TI_CPUID_H_ */
