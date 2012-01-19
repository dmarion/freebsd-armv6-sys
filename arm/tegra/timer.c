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

#define TEGRA2_TIMER_TMR_PTV_0		0x00	/*Timer Present Trigger Val (Set) Reg */
#define TEGRA2_TIMER_TMR_PCR_0		0x04	/* Timer Present Count Val (Status) Reg */

#define TEGRA2_TIMERUS_CNTR_1US_0	0x00
#define TEGRA2_TIMERUS_USEC_CFG_0	0x04
#define TEGRA2_TIMERUS_CNTR_FREEZE_0	0x3c

struct tegra2_timer_sc {
	struct resource	*	timer_res[2];
	bus_space_tag_t		timer_bst;
	bus_space_handle_t	timer_bsh;
	struct mtx		timer_mtx;
	struct eventtimer	et;
	int			is_timestamp;
};

static struct resource_spec tegra2_timer_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};

static struct resource_spec tegra2_ts_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ -1, 0 }
};

static struct tegra2_timer_sc *ts_sc = NULL;

#define	timer_read_4(reg)		\
    bus_space_read_4(sc->timer_bst, sc->timer_bsh, reg)
#define	timer_write_4(reg, val)		\
    bus_space_write_4(sc->timer_bst, sc->timer_bsh, reg, val)
#define	ts_read_4(reg)		\
    bus_space_read_4(ts_sc->timer_bst, ts_sc->timer_bsh, reg)
#define	ts_write_4(reg, val)		\
    bus_space_write_4(ts_sc->timer_bst, ts_sc->timer_bsh, reg, val)

static int tegra2_timer_start(struct eventtimer *, struct bintime *first,
	struct bintime *);
static int tegra2_timer_stop(struct eventtimer *et);
static unsigned tegra2_get_ts(struct timecounter *);
static int tegra2_hardclock(void *);

static struct timecounter tegra2_timestamp = {
	.tc_get_timecount = tegra2_get_ts,
	.tc_name = "Tegra 2 Timerstamp",
	.tc_frequency = 10000000,
	.tc_counter_mask = ~0u,
	.tc_quality = 1000,
};

static int
tegra2_timer_probe(device_t dev)
{
	struct	tegra2_timer_sc *sc;

	sc = (struct tegra2_timer_sc *)device_get_softc(dev);

	if (ofw_bus_is_compatible(dev, "nvidia,tegra2-timer")) {
		device_set_desc(dev, "Nvidia Tegra 2 Timer");
		sc->is_timestamp=0;
		return(0);
	}

	if (ofw_bus_is_compatible(dev, "nvidia,tegra2-timestamp")) {
		device_set_desc(dev, "Nvidia Tegra 2 Timestamp");
		sc->is_timestamp=1;
		return(0);
	}

	return (ENXIO);
}

static int
tegra2_timer_attach(device_t dev)
{
	int	error;
	void	*ihl;
	struct	tegra2_timer_sc *sc;
	uint32_t irq_cause, irq_mask;

	sc = (struct tegra2_timer_sc *)device_get_softc(dev);

	if (sc->is_timestamp)
		error = bus_alloc_resources(dev, tegra2_ts_spec, sc->timer_res);
	else
		error = bus_alloc_resources(dev, tegra2_timer_spec, sc->timer_res);

	if (error) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	sc->timer_bst = rman_get_bustag(sc->timer_res[0]);
	sc->timer_bsh = rman_get_bushandle(sc->timer_res[0]);

	if (sc->is_timestamp) {
		/* This is 32-bit timestamp counter */

		/* There should be only one timestamp counter*/
		if (ts_sc != NULL)
			return (ENXIO);

		ts_sc = sc;
		tc_init(&tegra2_timestamp);

	} else {
		/* This is 29-bit timer counter */
		timer_write_4(TEGRA2_TIMER_TMR_PTV_0,0xC0001000);
		printf("TEGRA2_TIMER_TMR_PTV_0 = 0x%08x\n", 
			timer_read_4(TEGRA2_TIMER_TMR_PTV_0));
		printf("TEGRA2_TIMER_TMR_PCR_0 = 0x%08x\n", 
			timer_read_4(TEGRA2_TIMER_TMR_PCR_0));

		if (bus_setup_intr(dev, sc->timer_res[1], INTR_TYPE_CLK,
			tegra2_hardclock, NULL, sc, &ihl) != 0) 
		{
			bus_release_resources(dev, tegra2_timer_spec, sc->timer_res);
			device_printf(dev, "Could not setup interrupt.\n");
			return (ENXIO);
		}
		sc->et.et_frequency = (uint64_t)1000000;
		sc->et.et_name = "Tegra 2 Timer";
		sc->et.et_flags = ET_FLAGS_PERIODIC | ET_FLAGS_ONESHOT;
		sc->et.et_quality = 1000;
		sc->et.et_min_period.sec = 0;
		sc->et.et_min_period.frac =
			((0x00000002LLU << 32) / sc->et.et_frequency) << 32;
		sc->et.et_max_period.sec = 0xfffffff0U / sc->et.et_frequency;
		sc->et.et_max_period.frac =
			((0xfffffffeLLU << 32) / sc->et.et_frequency) << 32;
		sc->et.et_start = tegra2_timer_start;
		sc->et.et_stop = tegra2_timer_stop;
		sc->et.et_priv = sc;
		et_register(&sc->et);
	}

	return(0);
}

static int
tegra2_hardclock(void *arg)
{
	struct	tegra2_timer_sc *sc;
	uint32_t irq_cause;
	
	printf("tegra2_hardclock\n");
#if 0
	irq_cause = read_cpu_ctrl(BRIDGE_IRQ_CAUSE);
	irq_cause &= ~(IRQ_TIMER0);
	write_cpu_ctrl(BRIDGE_IRQ_CAUSE, irq_cause);

	sc = (struct tegra2_timer_sc *)arg;
	if (sc->et.et_active)
		sc->et.et_event_cb(&sc->et, sc->et.et_arg);
#endif
	return (FILTER_HANDLED);
}

static int
tegra2_timer_start(struct eventtimer *et, struct bintime *first,
	struct bintime *period)
{
	struct	tegra2_timer_sc *sc = (struct tegra2_timer_sc *)et->et_priv;
	printf("tegra2_timer_start\n");
}

static int
tegra2_timer_stop(struct eventtimer *et)
{
	struct	tegra2_timer_sc *sc = (struct tegra2_timer_sc *)et->et_priv;
	printf("tegra2_timer_stop\n");
	timer_write_4(TEGRA2_TIMER_TMR_PTV_0, 0);
	return(0);
}

static device_method_t tegra2_timer_methods[] = {
	DEVMETHOD(device_probe, tegra2_timer_probe),
	DEVMETHOD(device_attach, tegra2_timer_attach),
	{ 0, 0 }
};

static driver_t tegra2_timer_driver = {
	"timer",
	tegra2_timer_methods,
	sizeof(struct tegra2_timer_sc),
};

static devclass_t tegra2_timer_devclass;

DRIVER_MODULE(timer, simplebus, tegra2_timer_driver, tegra2_timer_devclass, 0, 0);

static unsigned
tegra2_get_ts(struct timecounter *tc)
{
	return ts_read_4(TEGRA2_TIMERUS_CNTR_1US_0);
}

void
cpu_initclocks(void)
{
	cpu_initclocks_bsp();
}

void
DELAY(int usec)
{
	uint32_t counter;
	uint32_t first, last;
	int val = usec;

	if (ts_sc == NULL) {
		for (; usec > 0; usec--)
			for (counter = 100; counter > 0; counter--)
				;
		return;
	}

	first = ts_read_4(TEGRA2_TIMERUS_CNTR_1US_0);
	while (val > 0) {
		last = ts_read_4(TEGRA2_TIMERUS_CNTR_1US_0);
		if (last < first) {
			last = first;
		}
		val -= (last - first);
		first = last;
	}
}
