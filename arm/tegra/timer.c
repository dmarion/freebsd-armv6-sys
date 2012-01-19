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


// FIXME move to header
#define TEGRA2_CLK_RST_PA_BASE		0x60006000

#define TEGRA2_CLK_RST_OSC_FREQ_DET_REG		0x58
#define TEGRA2_CLK_RST_OSC_FREQ_DET_STAT_REG	0x5C
#define OSC_FREQ_DET_TRIG			(1<<31)
#define OSC_FREQ_DET_BUSY               	(1<<31)

#define TEGRA2_TIMER_TMR_PTV_0		0x000	/*Timer Present Trigger Val (Set) Reg */
#define TEGRA2_TIMER_TMR_PCR_0		0x004	/* Timer Present Count Val (Status) Reg */
 

struct tegra2_timer_softc {
	struct resource	*	timer_res[2];
	bus_space_tag_t		timer_bst;
	bus_space_handle_t	timer_bsh;
	struct mtx		timer_mtx;
	struct eventtimer	et;
};

static struct resource_spec tegra2_timer_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE },
	{ -1, 0 }
};

static struct tegra2_timer_softc *timer_softc = NULL;

static int	tegra2_hardclock(void *);

static int
tegra2_osc_freq_detect(void)
{
	bus_space_handle_t	bsh;
	uint32_t		c;
	uint32_t		r=0;
	int			i=0;

	struct {
		uint32_t val;
		uint32_t freq;
	} freq_det_cnts[] = {
		{ 732,  12000000 },
		{ 794,  13000000 },
		{1172,  19200000 },
		{1587,  26000000 },
		{  -1,         0 },
	};

	printf("Measuring...\n");
	bus_space_map(fdtbus_bs_tag,TEGRA2_CLK_RST_PA_BASE, 0x1000, 0, &bsh);

	bus_space_write_4(fdtbus_bs_tag, bsh, TEGRA2_CLK_RST_OSC_FREQ_DET_REG,
			OSC_FREQ_DET_TRIG | 1 );
	do {} while (bus_space_read_4(fdtbus_bs_tag, bsh,
			TEGRA2_CLK_RST_OSC_FREQ_DET_STAT_REG) & OSC_FREQ_DET_BUSY);

	c = bus_space_read_4(fdtbus_bs_tag, bsh, TEGRA2_CLK_RST_OSC_FREQ_DET_STAT_REG);

	while (freq_det_cnts[i].val > 0) {
		if (((freq_det_cnts[i].val - 3) < c) && (c < (freq_det_cnts[i].val + 3)))
			r = freq_det_cnts[i].freq;
		i++;
	}
	printf("c=%u r=%u\n",c,r );
	bus_space_free(fdtbus_bs_tag, bsh, 0x1000);
	return r;
}


static int
tegra2_timer_probe(device_t dev)
{
	if (!ofw_bus_is_compatible(dev, "nvidia,tegra2-timer"))
		return (ENXIO);

	device_set_desc(dev, "Nvidia Tegra 2 CPU Timer");
	return (0);
}

static int
tegra2_timer_attach(device_t dev)
{
	int	error;
	void	*ihl;
	struct	tegra2_timer_softc *sc;
	uint32_t irq_cause, irq_mask;

	if (timer_softc != NULL)
		return (ENXIO);

	printf("timer_attach2\n");

	sc = (struct tegra2_timer_softc *)device_get_softc(dev);
	timer_softc = sc;

	error = bus_alloc_resources(dev, tegra2_timer_spec, sc->timer_res);
	if (error) {
		device_printf(dev, "could not allocate resources\n");
		return (ENXIO);
	}

	//printf("timer_attach3\n");
	//tegra2_osc_freq_detect();

	sc->timer_bst = rman_get_bustag(sc->timer_res[0]);
	sc->timer_bsh = rman_get_bushandle(sc->timer_res[0]);

	//mtx_init(&timer_softc->timer_mtx, "watchdog", NULL, MTX_DEF);
	//tegra2_watchdog_disable();
	//EVENTHANDLER_REGISTER(watchdog_list, tegra2_watchdog_event, sc, 0);

	if (bus_setup_intr(dev, sc->timer_res[1], INTR_TYPE_CLK,
	    tegra2_hardclock, NULL, sc, &ihl) != 0) {
		bus_release_resources(dev, tegra2_timer_spec, sc->timer_res);
		device_printf(dev, "Could not setup interrupt.\n");
		return (ENXIO);
	}
	return(0);
}

static int
tegra2_hardclock(void *arg)
{
	struct	tegra2_timer_softc *sc;
	uint32_t irq_cause;
	
	printf("tegra2_hardclock\n");
#if 0
	irq_cause = read_cpu_ctrl(BRIDGE_IRQ_CAUSE);
	irq_cause &= ~(IRQ_TIMER0);
	write_cpu_ctrl(BRIDGE_IRQ_CAUSE, irq_cause);

	sc = (struct tegra2_timer_softc *)arg;
	if (sc->et.et_active)
		sc->et.et_event_cb(&sc->et, sc->et.et_arg);
#endif
	return (FILTER_HANDLED);
}


static device_method_t tegra2_timer_methods[] = {
	DEVMETHOD(device_probe, tegra2_timer_probe),
	DEVMETHOD(device_attach, tegra2_timer_attach),

	{ 0, 0 }
};

static driver_t tegra2_timer_driver = {
	"timer",
	tegra2_timer_methods,
	sizeof(struct tegra2_timer_softc),
};

static devclass_t tegra2_timer_devclass;

DRIVER_MODULE(timer, simplebus, tegra2_timer_driver, tegra2_timer_devclass, 0, 0);


void
cpu_initclocks(void)
{
//	cpu_initclocks_bsp();
}

#if 0
void
cpu_startprofclock(void)
{

}

void
cpu_stopprofclock(void)
{

}
#endif

void
DELAY(int usec)
{
	// FIXME: TODO
}
