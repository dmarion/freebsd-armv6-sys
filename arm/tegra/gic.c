/*-
 * Copyright (c) 2011 The FreeBSD Foundation
 * All rights reserved.
 *
 * Developed by Damjan Marion <damjan.marion@gmail.com>
 *
 * Based on OMAP4 GIC code by Ben Gray
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the company nor the name of the author may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
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
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/ktr.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>


 /* We are using GICv2 register naming */

 /* Distributor Registers */
#define GICD_CTLR		0x000			/* v1 ICDDCR */
#define GICD_TYPER		0x004			/* v1 ICDICTR */
#define GICD_IIDR		0x008			/* v1 ICDIIDR */
#define GICD_IGROUPR(n)		(0x0080 + ((n) * 4))	/* v1 ICDISERn */
#define GICD_ISENABLER(n)	(0x0100 + ((n) * 4))	/* v1 ICDISERn */
#define GICD_ICENABLER(n)	(0x0180 + ((n) * 4))	/* v1 ICDICERn */
#define GICD_ISPENDR(n)		(0x0200 + ((n) * 4))	/* v1 ICDISPR */
#define GICD_ICPENDR(n)		(0x0280 + ((n) * 4))	/* v1 ICDICPR */
#define GICD_ICACTIVER(n)	(0x0380 + ((n) * 4))	/* v1 ICDABR */
#define GICD_IPRIORITYR(n)	(0x0400 + ((n) * 4))	/* v1 ICDIPTRn */
#define GICD_ITARGETSR(n)	(0x0800 + ((n) * 4))	/* v1 ICDIPTR */
#define GICD_ICFGR(n)		(0x0C00 + ((n) * 4))	/* v1 ICDICFRn */
#define GICD_SGIR(n)		(0x0F00 + ((n) * 4))	/* v1 ICDSGIRn */

 /* CPU Registers */
#define GICC_CTLR		0x0000			/* v1 ICCICR */
#define GICC_PMR		0x0004			/* v1 ICCPMR */
#define GICC_BPR		0x0008			/* v1 ICCBPR */
#define GICC_IAR		0x000C			/* v1 ICCIAR */
#define GICC_EOIR		0x0010			/* v1 ICCEOIR */
#define GICC_RPR		0x0014			/* v1 ICCRPR */
#define GICC_HPPIR		0x0018			/* v1 ICCHPIR */
#define GICC_ABPR		0x001C			/* v1 ICCABPR */
#define GICC_IIDR		0x00FC			/* v1 ICCIIDR*/



struct arm_gic_softc {
	struct resource *	gic_res[3];
	bus_space_tag_t		gic_c_bst;
	bus_space_tag_t		gic_d_bst;
	bus_space_handle_t	gic_c_bsh;
	bus_space_handle_t	gic_d_bsh;
	uint8_t			ver;
};

static struct resource_spec arm_gic_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },	/* Distributor registers */
	{ SYS_RES_MEMORY,	1,	RF_ACTIVE },	/* CPU Interrupt Intf. registers */
	{ -1, 0 }
};


static struct arm_gic_softc *arm_gic_sc = NULL;

#define	gic_c_read_4(reg)		\
    bus_space_read_4(arm_gic_sc->gic_c_bst, arm_gic_sc->gic_c_bsh, reg)
#define	gic_c_write_4(reg, val)		\
    bus_space_write_4(arm_gic_sc->gic_c_bst, arm_gic_sc->gic_c_bsh, reg, val)
#define	gic_d_read_4(reg)		\
    bus_space_read_4(arm_gic_sc->gic_d_bst, arm_gic_sc->gic_d_bsh, reg)
#define	gic_d_write_4(reg, val)		\
    bus_space_write_4(arm_gic_sc->gic_d_bst, arm_gic_sc->gic_d_bsh, reg, val)


static int
arm_gic_probe(device_t dev)
{
	if (!ofw_bus_is_compatible(dev, "arm,gic"))
		return (ENXIO);
	device_set_desc(dev, "ARM Generic Interrupt Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
arm_gic_attach(device_t dev)
{
	struct		arm_gic_softc *sc = device_get_softc(dev);
	int		rid;
	int		i;
	uint32_t	icciidr;
	uint32_t	nirqs;
	uint32_t	n;
	uint32_t	reg_off;
	uint32_t	bit_off;

	if (arm_gic_sc)
		return (ENXIO);

	if (bus_alloc_resources(dev, arm_gic_spec, sc->gic_res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	/* Distributor Interface */
	sc->gic_d_bst = rman_get_bustag(sc->gic_res[0]);
	sc->gic_d_bsh = rman_get_bushandle(sc->gic_res[0]);

	/* CPU Interface */
	sc->gic_c_bst = rman_get_bustag(sc->gic_res[1]);
	sc->gic_c_bsh = rman_get_bushandle(sc->gic_res[1]);

	arm_gic_sc = sc;

	/* Disable interrupt forwarding to the CPU interface */
	gic_d_write_4(GICD_CTLR, 0x00);

	/* Get the number of interrupts */
	nirqs = gic_d_read_4(GICD_TYPER);
	nirqs = 32 * ((nirqs & 0x1f) + 1);

	icciidr = gic_c_read_4(GICC_IIDR);
	device_printf(dev,"pn 0x%x, arch 0x%x, rev 0x%x, implementer 0x%x nirqs %u\n", 
			icciidr>>20, (icciidr>>16) & 0xF, (icciidr>>12) & 0xf,
			(icciidr & 0xfff), nirqs);

	/* Set all global interrupts to be level triggered, active low. */
	for (i = 32; i < nirqs; i += 32) {
		gic_d_write_4(GICD_ICFGR(i >> 5), 0x00000000);
	}

	/* Disable all interrupts. */
	for (i = 32; i < nirqs; i += 32) {
		gic_d_write_4(GICD_ICENABLER(i >> 5), 0xFFFFFFFF);
	}

	/* Route all interrupts to CPU0 and set priority to 0 */
	for (i = 32; i < nirqs; i += 32) {
		gic_d_write_4(GICD_IPRIORITYR(i >> 5), 0x00000000);
		gic_d_write_4(GICD_ITARGETSR(i >> 5), 0x01010101);
	}

	/* Enable interrupt distribution */
	gic_d_write_4(GICD_CTLR, 0x01);

	/* Enable CPU interface */
	gic_c_write_4(GICC_CTLR, 1);

	gic_d_write_4(GICD_ICENABLER(0), 0xFFFFFFFF);
	gic_d_write_4(GICD_ICENABLER(1), 0xFFFFFFFF);
	gic_d_write_4(GICD_ICENABLER(2), 0xFFFFFFFF);
	return (0);
}

static device_method_t arm_gic_methods[] = {
	DEVMETHOD(device_probe,		arm_gic_probe),
	DEVMETHOD(device_attach,	arm_gic_attach),
	{ 0, 0 }
};

static driver_t arm_gic_driver = {
	"gic",
	arm_gic_methods,
	sizeof(struct arm_gic_softc),
};

static devclass_t arm_gic_devclass;

DRIVER_MODULE(gic, simplebus, arm_gic_driver, arm_gic_devclass, 0, 0);

int
arm_get_next_irq(int last_irq)
{
	uint32_t active_irq;

	printf("arm_get_next_irq %u\n",last_irq);

	/* clean-up the last IRQ */
	if (last_irq != -1) {
		gic_c_write_4(GICC_EOIR, last_irq);
	}

	active_irq = gic_c_read_4(GICC_IAR);
	active_irq &= 0x3FF;

	if (active_irq == 0x3FF) {
		if (last_irq == -1)
			printf("Spurious interrupt detected [0x%08x]\n", active_irq);
		return -1;
	}

	return active_irq;
}

void
arm_mask_irq(uintptr_t nb)
{
	printf("arm_mask_irq %u\n",nb);
	gic_d_write_4(GICD_ICENABLER(nb >> 5), (1UL << (nb & 0x1F)));
}

void
arm_unmask_irq(uintptr_t nb)
{
	printf("arm_unmask_irq %u\n",nb);
	gic_d_write_4(GICD_ISENABLER(nb >> 5), (1UL << (nb & 0x1F)));
}
