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

struct ti_aintc_softc {
	struct resource *	aintc_res[3];
	bus_space_tag_t		aintc_bst;
	bus_space_handle_t	aintc_bsh;
	uint8_t			ver;
};

static struct resource_spec ti_aintc_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};


static struct ti_aintc_softc *ti_aintc_sc = NULL;

#define	aintc_read_4(reg)		\
    bus_space_read_4(ti_aintc_sc->aintc_bst, ti_aintc_sc->aintc_bsh, reg)
#define	aintc_write_4(reg, val)		\
    bus_space_write_4(ti_aintc_sc->aintc_bst, ti_aintc_sc->aintc_bsh, reg, val)


static int
ti_aintc_probe(device_t dev)
{
	if (!ofw_bus_is_compatible(dev, "ti,aintc"))
		return (ENXIO);
	device_set_desc(dev, "TI AINTC Interrupt Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
ti_aintc_attach(device_t dev)
{
	struct		ti_aintc_softc *sc = device_get_softc(dev);

	if (ti_aintc_sc)
		return (ENXIO);

	if (bus_alloc_resources(dev, ti_aintc_spec, sc->aintc_res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	sc->aintc_bst = rman_get_bustag(sc->aintc_res[0]);
	sc->aintc_bsh = rman_get_bushandle(sc->aintc_res[0]);
	return (0);
}

static device_method_t ti_aintc_methods[] = {
	DEVMETHOD(device_probe,		ti_aintc_probe),
	DEVMETHOD(device_attach,	ti_aintc_attach),
	{ 0, 0 }
};

static driver_t ti_aintc_driver = {
	"gic",
	ti_aintc_methods,
	sizeof(struct ti_aintc_softc),
};

static devclass_t ti_aintc_devclass;

DRIVER_MODULE(aintc, simplebus, ti_aintc_driver, ti_aintc_devclass, 0, 0);

int
arm_get_next_irq(int last_irq)
{
	return 0;
}

void
arm_mask_irq(uintptr_t nb)
{
}

void
arm_unmask_irq(uintptr_t nb)
{
}
