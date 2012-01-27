/*-
 * Copyright (c) 2011 The FreeBSD Foundation
 * All rights reserved.
 *
 * Developed by Damjan Marion <damjan.marion@gmail.com>
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
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>
#include <sys/timeet.h>
#include <sys/timetc.h>
#include <sys/watchdog.h>
#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/frame.h>
#include <machine/intr.h>

#include <dev/fdt/fdt_common.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>
#include <machine/fdt.h>

struct am335x_timer_softc {
	struct resource *	tmr_res[4];
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	uint32_t		clkfreq;
	struct eventtimer	et;
};

static struct resource_spec am335x_timer_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};


static struct am335x_timer_softc *am335x_timer_sc = NULL;

#define am335x_timer_read_4(reg)	\
    bus_space_read_4(am335x_timer_sc->_bst, am335x_timer_sc->_bsh, reg)
#define am335x_timer_write_4(reg, val)	\
    bus_space_write_4(am335x_timer_sc->_bst, am335x_timer_sc->_bsh, reg, val)


static timecounter_get_t am335x_timer_get_timecount;

static struct timecounter am335x_timer_timecount = {
	.tc_name           = "AM335x Timecouter",
	.tc_get_timecount  = am335x_timer_get_timecount,
	.tc_poll_pps       = NULL,
	.tc_counter_mask   = ~0u,
	.tc_frequency      = 0,
	.tc_quality        = 1000,
};

static unsigned
am335x_timer_get_timecount(struct timecounter *tc)
{
	return 0;
}

static int
am335x_timer_start(struct eventtimer *et, struct bintime *first,
              struct bintime *period)
{
	struct am335x_timer_softc *sc = (struct am335x_timer_softc *)et->et_priv;

	return (0);
}

static int
am335x_timer_stop(struct eventtimer *et)
{
	return (0);
}

static int
am335x_timer_intr(void *arg)
{
	struct am335x_timer_softc *sc = (struct am335x_timer_softc *)arg;

	if (sc->et.et_active)
		sc->et.et_event_cb(&sc->et, sc->et.et_arg);

	return (FILTER_HANDLED);
}

static int
am335x_timer_probe(device_t dev)
{
	if (!ofw_bus_is_compatible(dev, "am335x,timer"))
		return (ENXIO);

	device_set_desc(dev, "TI AM335X Timers");
	return (BUS_PROBE_DEFAULT);
}

static int
am335x_timer_attach(device_t dev)
{
	struct am335x_timer_softc *sc = device_get_softc(dev);
	phandle_t node;
	pcell_t clock;
	void *ihl;

	if (am335x_timer_sc)
		return (ENXIO);

	/* Get the base clock frequency */
	node = ofw_bus_get_node(dev);
	if ((OF_getprop(node, "clock-frequency", &clock, sizeof(clock))) <= 0) {
		device_printf(dev, "missing clock-frequency attribute in FDT\n");
		return (ENXIO);
	}
	sc->clkfreq = fdt32_to_cpu(clock);

	if (bus_alloc_resources(dev, am335x_timer_spec, sc->tmr_res)) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	sc->bst = rman_get_bustag(sc->tmr_res[0]);
	sc->bsh = rman_get_bushandle(sc->tmr_res[0]);

	am335x_timer_sc = sc;
	return (0);
}

static device_method_t am335x_timer_methods[] = {
	DEVMETHOD(device_probe,		am335x_timer_probe),
	DEVMETHOD(device_attach,	am335x_timer_attach),
	{ 0, 0 }
};

static driver_t am335x_timer_driver = {
	"am335x_timer",
	am335x_timer_methods,
	sizeof(struct am335x_timer_softc),
};

static devclass_t am335x_timer_devclass;

DRIVER_MODULE(am335x_timer, simplebus, am335x_timer_driver, am335x_timer_devclass, 0, 0);

void
cpu_initclocks(void)
{
}


void
DELAY(int usec)
{
	int32_t counts_per_usec;
	int32_t counts;
	uint32_t first, last;

	/* Check the timers are setup, if not just use a for loop for the meantime */
	if (am335x_timer_sc == NULL) {
		for (; usec > 0; usec--)
			for (counts = 200; counts > 0; counts--)
				cpufunc_nullop();	/* Prevent gcc from optimizing
							 * out the loop
							 */
		return;
	}

}
