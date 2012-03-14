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

/*
 * Driver for TI implementation of Mentor InventraTM MUSBHSFC USB 2.0
 * High-Speed Function Controller found in AM335x SoC
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

#include <net/ethernet.h>
#include <net/bpf.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_vlan_var.h>

#include <netinet/in_systm.h>
#include <netinet/in.h>
#include <netinet/ip.h>

#include <sys/sockio.h>
#include <sys/bus.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm/ti/ti_scm.h>

#include "miibus_if.h"

#include <arm/debug.h> //FIXME

#define NUM_MEM_RESOURCES 5
#define NUM_IRQ_RESOURCES 4
 
struct musbhsfc_am335x_softc {
	struct resource *	mem_res[NUM_MEM_RESOURCES];
	struct resource *	irq_res[NUM_IRQ_RESOURCES];
	bus_space_tag_t		bst[NUM_MEM_RESOURCES];
	bus_space_handle_t	bsh[NUM_MEM_RESOURCES];
};

typedef struct musbhsfc_am335x_softc musbhsfc_am335x_softc_t;

static struct resource_spec musbhsfc_am335x_mem_spec[] = {
	{ SYS_RES_MEMORY,   0,  RF_ACTIVE },
	{ SYS_RES_MEMORY,   1,  RF_ACTIVE },
	{ SYS_RES_MEMORY,   2,  RF_ACTIVE },
	{ SYS_RES_MEMORY,   3,  RF_ACTIVE },
	{ SYS_RES_MEMORY,   4,  RF_ACTIVE },
	{ -1,               0,  0 }
};

static struct resource_spec musbhsfc_am335x_irq_spec[] = {
	{ SYS_RES_IRQ,      0,  RF_ACTIVE },
	{ SYS_RES_IRQ,      1,  RF_ACTIVE },
	{ SYS_RES_IRQ,      2,  RF_ACTIVE },
	{ SYS_RES_IRQ,      3,  RF_ACTIVE },
	{ -1,               0,  0 }
};

static int musbhsfc_am335x_probe(device_t dev);
static int musbhsfc_am335x_attach(device_t dev);
static int musbhsfc_am335x_detach(device_t dev);

static device_method_t musbhsfc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, musbhsfc_am335x_probe),
	DEVMETHOD(device_attach, musbhsfc_am335x_attach),
	DEVMETHOD(device_detach, musbhsfc_am335x_detach),
	DEVMETHOD(device_suspend, bus_generic_suspend),
	DEVMETHOD(device_resume, bus_generic_resume),
	DEVMETHOD(device_shutdown, bus_generic_shutdown),
	DEVMETHOD_END
};

static driver_t musbhsfc_driver = {
	"musbhsfc",
	musbhsfc_methods,
	sizeof(musbhsfc_am335x_softc_t),
};

static devclass_t musbhsfc_devclass;

DRIVER_MODULE(musbhsfc, simplebus, musbhsfc_driver, musbhsfc_devclass, 0, 0);
MODULE_DEPEND(musbhsfc, usb, 1, 1, 1);


static int
musbhsfc_am335x_probe(device_t dev)
{

	if (!ofw_bus_is_compatible(dev, "ti,am335x-musbhsfc"))
		return (ENXIO);

	device_set_desc(dev, "Inventra MUSBHSFC USB 2.0 High-Speed Function Controller");

	return (BUS_PROBE_DEFAULT);
}

static int
musbhsfc_am335x_attach(device_t dev)
{
	musbhsfc_am335x_softc_t *sc = device_get_softc(dev);
	int err;
	int i;

	/* Request the memory resources */
	err = bus_alloc_resources(dev, musbhsfc_am335x_mem_spec,
		sc->mem_res);
	if (err) {
		device_printf(dev, "Error: could not allocate mem resources\n");
		return (ENXIO);
	}

	/* Request the IRQ resources */
	err = bus_alloc_resources(dev, musbhsfc_am335x_irq_spec,
		sc->irq_res);
	if (err) {
		device_printf(dev, "Error: could not allocate irq resources\n");
		return (ENXIO);
	}
	for(i=0;i<NUM_MEM_RESOURCES;i++) {
		sc->bst[i] = rman_get_bustag(sc->mem_res[i]);
		sc->bsh[i] = rman_get_bushandle(sc->mem_res[i]);
	}

	return (0);
}

static int
musbhsfc_am335x_detach(device_t dev)
{
	musbhsfc_am335x_softc_t *sc = device_get_softc(dev);
	return (0);
}
