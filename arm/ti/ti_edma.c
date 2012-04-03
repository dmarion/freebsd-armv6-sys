/*-
 * Copyright (c) 2012 Damjan Marion <dmarion@Freebsd.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/mbuf.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/socket.h>
#include <sys/sysctl.h>

#include <sys/sockio.h>
#include <sys/bus.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/ti/ti_scm.h>
#include <arm/ti/ti_prcm.h>

#define TI_EDMA_NUM_TCS			3
#define TI_EDMA_NUM_IRQS		3
#define TI_EDMA_NUM_DMA_CHS		64
#define TI_EDMA_NUM_QDMA_CHS		8

#define TI_EDMA3CC_PID			0x000
#define TI_EDMA3CC_DCHMAP(p)		(0x100 + ((p)*4))
#define TI_EDMA3CC_DMAQNUM(n)		(0x240 + ((n)*4))
#define TI_EDMA3CC_QDMAQNUM		0x260
#define TI_EDMA3CC_EMCR			0x308
#define TI_EDMA3CC_EMCRH		0x30C
#define TI_EDMA3CC_QEMCR		0x314
#define TI_EDMA3CC_CCERRCLR		0x31C
#define TI_EDMA3CC_DRAE(p)		(0x340 + ((p)*8))
#define TI_EDMA3CC_DRAEH(p)		(0x344 + ((p)*8))
#define TI_EDMA3CC_QRAE(p)		(0x380 + ((p)*4))

#define TI_EDMA3CC_DMAQNUM_SET(c,q)	((0x7 & (q)) << (((c) % 8) * 4))
#define TI_EDMA3CC_DMAQNUM_CLR(c)	(~(0x7 << (((c) % 8) * 4)))
#define TI_EDMA3CC_QDMAQNUM_SET(c,q)	((0x7 & (q)) << ((c) * 4))
#define TI_EDMA3CC_QDMAQNUM_CLR(c)	(~(0x7 << ((c) * 4)))




struct ti_edma_softc {
	device_t		sc_dev;
	struct resource *	mem_res[TI_EDMA_NUM_TCS+1];
	struct resource *	irq_res[TI_EDMA_NUM_IRQS];
	void			*ih_cookie[TI_EDMA_NUM_IRQS];
};

static struct resource_spec ti_edma_mem_spec[] = {
	{ SYS_RES_MEMORY,   0,  RF_ACTIVE },
	{ SYS_RES_MEMORY,   1,  RF_ACTIVE },
	{ SYS_RES_MEMORY,   2,  RF_ACTIVE },
	{ SYS_RES_MEMORY,   3,  RF_ACTIVE },
	{ -1,               0,  0 }
};
static struct resource_spec ti_edma_irq_spec[] = {
	{ SYS_RES_IRQ,      0,  RF_ACTIVE },
	{ SYS_RES_IRQ,      1,  RF_ACTIVE },
	{ SYS_RES_IRQ,      2,  RF_ACTIVE },
	{ -1,               0,  0 }
};

/* Read/Write macros */
#define ti_edma_cc_read_4(reg)		bus_read_4(sc->mem_res[0], reg)
#define ti_edma_cc_write_4(reg, val)	bus_write_4(sc->mem_res[0], reg, val)
#define ti_edma_tc_read_4(c, reg)	bus_read_4(sc->mem_res[c+1], reg)
#define ti_edma_tc_write_4(c, reg, val)	bus_write_4(sc->mem_res[c+1], reg, val)

static void ti_edma_intr_comp(void *arg);
static void ti_edma_intr_mperr(void *arg);
static void ti_edma_intr_err(void *arg);
static void ti_edma_init(struct ti_edma_softc *sc, unsigned int queue_num);

static struct {
	driver_intr_t *handler;
	char * description;
} ti_edma_intrs[TI_EDMA_NUM_IRQS] = {
	{ ti_edma_intr_comp,	"EDMA Completion Interrupt" },
	{ ti_edma_intr_mperr,	"EDMA Memory Protection Error Interrupt" },
	{ ti_edma_intr_err,	"EDMA Error Interrupt" },
};

static int
ti_edma_probe(device_t dev)
{
	if (!ofw_bus_is_compatible(dev, "ti,edma"))
		return (ENXIO);

	device_set_desc(dev, "TI EDMA Controller");
	return (0);
}

static int
ti_edma_attach(device_t dev)
{
	struct ti_edma_softc *sc = device_get_softc(dev);
	uint32_t reg;
	int err;
	int i;

	sc->sc_dev = dev;

	/* Request the memory resources */
	err = bus_alloc_resources(dev, ti_edma_mem_spec, sc->mem_res);
	if (err) {
		device_printf(dev, "Error: could not allocate mem resources\n");
		return (ENXIO);
	}

	/* Request the IRQ resources */
	err = bus_alloc_resources(dev, ti_edma_irq_spec, sc->irq_res);
	if (err) {
		device_printf(dev, "Error: could not allocate irq resources\n");
		return (ENXIO);
	}

	/* Enable Channel Controller */
	ti_prcm_clk_enable(EDMA_TPCC_CLK);

	reg = ti_edma_cc_read_4(TI_EDMA3CC_PID);

	device_printf(dev, "EDMA revision %08x\n", reg);


	/* Attach interrupt handlers */
	for (i = 0; i < TI_EDMA_NUM_IRQS; ++i) {
		err = bus_setup_intr(dev, sc->irq_res[i], INTR_TYPE_MISC |
		    INTR_MPSAFE, NULL, *ti_edma_intrs[i].handler,
		    sc, &sc->ih_cookie[i]);
		if (err) {
			device_printf(dev, "could not setup %s\n",
			    ti_edma_intrs[i].description);
			return (err);
		}
	}

	//ti_edma_init(sc);

	return (0);
}

static device_method_t ti_edma_methods[] = {
	DEVMETHOD(device_probe, ti_edma_probe),
	DEVMETHOD(device_attach, ti_edma_attach),
	{0, 0},
};

static driver_t ti_edma_driver = {
	"ti_edma",
	ti_edma_methods,
	sizeof(struct ti_edma_softc),
};
static devclass_t ti_edma_devclass;

DRIVER_MODULE(ti_edma, simplebus, ti_edma_driver, ti_edma_devclass, 0, 0);
MODULE_DEPEND(ti_edma, ti_prcm, 1, 1, 1);

static void
ti_edma_intr_comp(void *arg)
{
	printf("%s: unimplemented\n", __func__);
}

static void
ti_edma_intr_mperr(void *arg)
{
	printf("%s: unimplemented\n", __func__);
}

static void
ti_edma_intr_err(void *arg)
{
	printf("%s: unimplemented\n", __func__);
}

static void
ti_edma_init(struct ti_edma_softc *sc, unsigned int queue_num)
{
	uint32_t reg;
	int i;

	/* Clear Event Missed Regs */
	ti_edma_cc_write_4(TI_EDMA3CC_EMCR, 0xFFFFFFFF);
	ti_edma_cc_write_4(TI_EDMA3CC_EMCRH, 0xFFFFFFFF);
	ti_edma_cc_write_4(TI_EDMA3CC_QEMCR, 0xFFFFFFFF);

	/* Clear Error Reg */
	ti_edma_cc_write_4(TI_EDMA3CC_CCERRCLR, 0xFFFFFFFF);

	/* Enable DMA channels 0-63 */
	ti_edma_cc_write_4(TI_EDMA3CC_DRAE(0), 0xFFFFFFFF);
	ti_edma_cc_write_4(TI_EDMA3CC_DRAEH(0), 0xFFFFFFFF);

	for (i = 0; i < 64; i++) {
		ti_edma_cc_write_4(TI_EDMA3CC_DCHMAP(i), i<<5);
	}

	/* Initialize the DMA Queue Number Registers */
	for (i = 0; i < TI_EDMA_NUM_DMA_CHS; i++) {
		reg = ti_edma_cc_read_4(TI_EDMA3CC_DMAQNUM(i>>3));
		reg &= TI_EDMA3CC_DMAQNUM_CLR(i);
		reg |= TI_EDMA3CC_DMAQNUM_SET(i, queue_num);
		ti_edma_cc_write_4(TI_EDMA3CC_DMAQNUM(i>>3), reg);
	}

	/* Enable the QDMA Region access for all channels */
	ti_edma_cc_write_4(TI_EDMA3CC_QRAE(0), (1 << TI_EDMA_NUM_QDMA_CHS) - 1);

	/*Initialize QDMA Queue Number Registers */
	for (i = 0; i < TI_EDMA_NUM_QDMA_CHS; i++) {
		reg = ti_edma_cc_read_4(TI_EDMA3CC_QDMAQNUM);
		reg &= TI_EDMA3CC_QDMAQNUM_CLR(i);
		reg |= TI_EDMA3CC_QDMAQNUM_SET(i, queue_num);
		ti_edma_cc_write_4(TI_EDMA3CC_QDMAQNUM, reg);
	}
}

