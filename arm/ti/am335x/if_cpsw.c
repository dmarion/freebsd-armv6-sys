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
 * TI 3 Port Switch Ethernet (CPSW) Driver
 * Found in TI8148, AM335x SoCs
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

//#include <dev/cpsw/if_cpswvar.h>

#include "miibus_if.h"

#define CPSW_SS_OFFSET			0x0000
#define CPSW_SS_IDVER			(CPSW_SS_OFFSET + 0x00)
#define CPSW_SS_SOFT_RESET		(CPSW_SS_OFFSET + 0x08)
#define CPSW_SS_STAT_PORT_EN		(CPSW_SS_OFFSET + 0x0C)

#define CPSW_PORT_OFFSET		0x0100

#define CPSW_CPDMA_OFFSET		0x0800
#define CPSW_CPDMA_TX_CONTROL		(CPSW_CPDMA_OFFSET + 0x04)
#define CPSW_CPDMA_RX_CONTROL		(CPSW_CPDMA_OFFSET + 0x14)
#define CPSW_CPDMA_TX_INTMASK_SET	(CPSW_CPDMA_OFFSET + 0x88)
#define CPSW_CPDMA_CPDMA_EOI_VECTOR	(CPSW_CPDMA_OFFSET + 0x94)
#define CPSW_CPDMA_RX_INTMASK_SET	(CPSW_CPDMA_OFFSET + 0xa8)
#define CPSW_CPDMA_RX_FREEBUFFER(p)	(CPSW_CPDMA_OFFSET + 0xe0 + (p * 0x04))

#define CPSW_CPTS_OFFSET		0x0C00

#define CPSW_ALE_OFFSET			0x0D00
#define CPSW_ALE_CONTROL		(CPSW_ALE_OFFSET + 0x08)
#define CPSW_ALE_PORTCTL(p)		(CPSW_ALE_OFFSET + 0x40 + (p * 0x04))
#define CPSW_ALE_TBLCTL			(CPSW_ALE_OFFSET + 0x20)
#define CPSW_ALE_TBLW2			(CPSW_ALE_OFFSET + 0x34)
#define CPSW_ALE_TBLW1			(CPSW_ALE_OFFSET + 0x38)
#define CPSW_ALE_TBLW0			(CPSW_ALE_OFFSET + 0x3C)

#define CPSW_SL_OFFSET			0x0D80
#define CPSW_SL_MACCONTROL(p)		(CPSW_SL_OFFSET + (0x40 * p) + 0x04)
#define CPSW_SL_SOFT_RESET(p)		(CPSW_SL_OFFSET + (0x40 * p) + 0x0C)
 
#define MDIO_OFFSET			0x1000
#define MDIOCONTROL			(MDIO_OFFSET + 0x04)
#define MDIOUSERACCESS0			(MDIO_OFFSET + 0x80)
#define MDIOUSERPHYSEL0			(MDIO_OFFSET + 0x84)

#define CPSW_WR_OFFSET			0x1200
#define CPSW_WR_SOFT_RESET		(CPSW_WR_OFFSET + 0x04)
#define CPSW_WR_C_RX_THRESH_EN(p)	(CPSW_WR_OFFSET + (0x10 * p) + 0x10)
#define CPSW_WR_C_RX_EN(p)		(CPSW_WR_OFFSET + (0x10 * p) + 0x14)
#define CPSW_WR_C_TX_EN(p)		(CPSW_WR_OFFSET + (0x10 * p) + 0x18)
#define CPSW_WR_C_MISC_EN(p)		(CPSW_WR_OFFSET + (0x10 * p) + 0x1C)

#define CPSW_INTR_COUNT		4
#define CPSW_TX_DESC_NUM	256

/* MII BUS  */
#define CPSW_MIIBUS_RETRIES	5
#define CPSW_MIIBUS_DELAY	1000

struct cpsw_softc {
	struct ifnet	*ifp;
	phandle_t	node;
	device_t	dev;
	device_t	miibus;
	struct mii_data	*mii;
	struct mtx	transmit_lock;			/* transmitter lock */
	struct mtx	receive_lock;			/* receiver lock */
	struct resource	*res[1 + CPSW_INTR_COUNT];	/* resources */
	void		*ih_cookie[CPSW_INTR_COUNT];	/* interrupt handlers cookies */
	struct callout	wd_callout;
	uint32_t	cpsw_if_flags;
};

static struct cpsw_softc *sc_cpsw0 = NULL;

static int cpsw_probe(device_t dev);
static int cpsw_attach(device_t dev);
static int cpsw_detach(device_t dev);
static int cpsw_shutdown(device_t dev);
static int cpsw_suspend(device_t dev);
static int cpsw_resume(device_t dev);

static int cpsw_miibus_readreg(device_t dev, int phy, int reg);
static int cpsw_miibus_writereg(device_t dev, int phy, int reg, int value);

static int cpsw_ifmedia_upd(struct ifnet *ifp);
static void cpsw_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr);

static void cpsw_init(void *arg);
static void cpsw_init_locked(void *arg);
static void cpsw_start(struct ifnet *ifp);
static int cpsw_ioctl(struct ifnet *ifp, u_long command, caddr_t data);

static void cpsw_intr_rx_thresh(void *arg);
static void cpsw_intr_rx(void *arg);
static void cpsw_intr_tx(void *arg);
static void cpsw_intr_misc(void *arg);


static device_method_t cpsw_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		cpsw_probe),
	DEVMETHOD(device_attach,	cpsw_attach),
	DEVMETHOD(device_detach,	cpsw_detach),
	DEVMETHOD(device_shutdown,	cpsw_shutdown),
	DEVMETHOD(device_suspend,	cpsw_suspend),
	DEVMETHOD(device_resume,	cpsw_resume),
	/* MII interface */
	DEVMETHOD(miibus_readreg,	cpsw_miibus_readreg),
	DEVMETHOD(miibus_writereg,	cpsw_miibus_writereg),
	{ 0, 0 }
};


static driver_t cpsw_driver = {
	"cpsw",
	cpsw_methods,
	sizeof(struct cpsw_softc),
};

static devclass_t cpsw_devclass;


DRIVER_MODULE(cpsw, simplebus, cpsw_driver, cpsw_devclass, 0, 0);
DRIVER_MODULE(miibus, cpsw, miibus_driver, miibus_devclass, 0, 0);
MODULE_DEPEND(cpsw, ether, 1, 1, 1);
MODULE_DEPEND(cpsw, miibus, 1, 1, 1);

static struct resource_spec res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ SYS_RES_IRQ, 0, RF_ACTIVE | RF_SHAREABLE },
	{ SYS_RES_IRQ, 1, RF_ACTIVE | RF_SHAREABLE },
	{ SYS_RES_IRQ, 2, RF_ACTIVE | RF_SHAREABLE },
	{ SYS_RES_IRQ, 3, RF_ACTIVE | RF_SHAREABLE },
	{ -1, 0 }
};

static struct {
	driver_intr_t *handler;
	char * description;
} cpsw_intrs[CPSW_INTR_COUNT + 1] = {
	{ cpsw_intr_rx_thresh,"CPSW RX threshold interrupt" },
	{ cpsw_intr_rx,	"CPSW RX interrupt" },
	{ cpsw_intr_tx,	"CPSW TX interrupt" },
	{ cpsw_intr_misc,"CPSW misc interrupt" },
};

/* Read/Write macros */
#define cpsw_read_4(reg)		\
	bus_read_4(sc->res[0], reg)
#define cpsw_write_4(reg, val)		\
	bus_write_4(sc->res[0], reg, val)

/* Locking macros */
#define CPSW_TRANSMIT_LOCK(sc) do {					\
		mtx_assert(&(sc)->receive_lock, MA_NOTOWNED);		\
		mtx_lock(&(sc)->transmit_lock);				\
} while (0)

#define CPSW_TRANSMIT_UNLOCK(sc)	mtx_unlock(&(sc)->transmit_lock)
#define CPSW_TRANSMIT_LOCK_ASSERT(sc)	mtx_assert(&(sc)->transmit_lock, MA_OWNED)

#define CPSW_RECEIVE_LOCK(sc) do {					\
		mtx_assert(&(sc)->transmit_lock, MA_NOTOWNED);		\
		mtx_lock(&(sc)->receive_lock);				\
} while (0)

#define CPSW_RECEIVE_UNLOCK(sc)		mtx_unlock(&(sc)->receive_lock)
#define CPSW_RECEIVE_LOCK_ASSERT(sc)	mtx_assert(&(sc)->receive_lock, MA_OWNED)

#define CPSW_GLOBAL_LOCK(sc) do {					\
		if ((mtx_owned(&(sc)->transmit_lock) ? 1 : 0) !=	\
		    (mtx_owned(&(sc)->receive_lock) ? 1 : 0)) {		\
			panic("mge deadlock possibility detection!");	\
		}							\
		mtx_lock(&(sc)->transmit_lock);				\
		mtx_lock(&(sc)->receive_lock);				\
} while (0)

#define CPSW_GLOBAL_UNLOCK(sc) do {					\
		CPSW_RECEIVE_UNLOCK(sc);				\
		CPSW_TRANSMIT_UNLOCK(sc);				\
} while (0)

#define CPSW_GLOBAL_LOCK_ASSERT(sc) do {				\
		CPSW_TRANSMIT_LOCK_ASSERT(sc);				\
		CPSW_RECEIVE_LOCK_ASSERT(sc);				\
} while (0)


static int
cpsw_probe(device_t dev)
{

	if (!ofw_bus_is_compatible(dev, "ti,cpsw"))
		return (ENXIO);

	device_set_desc(dev, "3-port Switch Ethernet Subsystem");
	return (BUS_PROBE_DEFAULT);
}

static int
cpsw_attach(device_t dev)
{
	struct cpsw_softc *sc;
	struct mii_softc *miisc;
	struct ifnet *ifp;
	uint8_t hwaddr[ETHER_ADDR_LEN];
	int i, error, phy;
	uint32_t reg;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->node = ofw_bus_get_node(dev);

	if (device_get_unit(dev) == 0)
		sc_cpsw0 = sc;

	/* Get phy address from fdt */
	if (fdt_get_phyaddr(sc->node, &phy) != 0) {
		device_printf(dev, "failed to get PHY address from FDT\n");
		return (ENXIO);
	}
	/* Initialize mutexes */
	mtx_init(&sc->transmit_lock, device_get_nameunit(dev),
		"cpsw TX lock", MTX_DEF);
	mtx_init(&sc->receive_lock, device_get_nameunit(dev),
		"cpsw RX lock", MTX_DEF);

	/* Allocate IO and IRQ resources */
	error = bus_alloc_resources(dev, res_spec, sc->res);
	if (error) {
		device_printf(dev, "could not allocate resources\n");
		cpsw_detach(dev);
		return (ENXIO);
	}

	reg = cpsw_read_4(CPSW_SS_IDVER);
	device_printf(dev, "Version %d.%d (%d)\n", (reg >> 8 & 0x7),
		reg & 0xFF, (reg >> 11) & 0x1F);

#if 0
	/* Allocate DMA, buffers, buffer descriptors */
	error = cpsw_allocate_dma(sc);
	if (error) {
		cpsw_detach(dev);
		return (ENXIO);
	}
#endif


#if 0
	cpsw_add_sysctls(sc);
#endif

	/* Allocate network interface */
	ifp = sc->ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL) {
		device_printf(dev, "if_alloc() failed\n");
		cpsw_detach(dev);
		return (ENOMEM);
	}

	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	ifp->if_softc = sc;
	ifp->if_flags = IFF_SIMPLEX | IFF_MULTICAST | IFF_BROADCAST;
	ifp->if_capabilities = IFCAP_VLAN_MTU;
#if 0
	if (sc->cpsw_hw_csum) {
		ifp->if_capabilities |= IFCAP_HWCSUM;
		ifp->if_hwassist = CSUM_IP | CSUM_TCP | CSUM_UDP;
	}
#endif
	ifp->if_capenable = ifp->if_capabilities;

#if 0
#ifdef DEVICE_POLLING
	/* Advertise that polling is supported */
	ifp->if_capabilities |= IFCAP_POLLING;
#endif
#endif

	ifp->if_init = cpsw_init;
	ifp->if_start = cpsw_start;
	ifp->if_ioctl = cpsw_ioctl;

	ifp->if_snd.ifq_drv_maxlen = CPSW_TX_DESC_NUM - 1;
	IFQ_SET_MAXLEN(&ifp->if_snd, ifp->if_snd.ifq_drv_maxlen);
	IFQ_SET_READY(&ifp->if_snd);

#if 0
	cpsw_get_mac_address(sc, hwaddr);
#endif
	ether_ifattach(ifp, hwaddr);
	callout_init(&sc->wd_callout, 0);

	/* Initialze MDIO - ENABLE, PREAMBLE=0, FAULTENB, CLKDIV=0xFF */
	/* TODO Calculate MDCLK=CLK/(CLKDIV+1) */
	cpsw_write_4(MDIOCONTROL, (1<<30) | (1<<18) | 0xFF);

	/* Attach PHY(s) */
	error = mii_attach(dev, &sc->miibus, ifp, cpsw_ifmedia_upd,
	    cpsw_ifmedia_sts, BMSR_DEFCAPMASK, phy, MII_OFFSET_ANY, 0);
	if (error) {
		device_printf(dev, "attaching PHYs failed\n");
		cpsw_detach(dev);
		return (error);
	}
	sc->mii = device_get_softc(sc->miibus);

	/* Tell the MAC where to find the PHY so autoneg works */
	miisc = LIST_FIRST(&sc->mii->mii_phys);
	printf("miisc->mii_phy = %u MDIOUSERPHYSEL0=%x\n",
		miisc->mii_phy, cpsw_read_4(MDIOUSERPHYSEL0));
	/* Select PHY and enable interrupts */
	cpsw_write_4(MDIOUSERPHYSEL0, (1 << 6) | (miisc->mii_phy & 0x1F));

	printf("miisc->mii_phy = %u MDIOUSERPHYSEL0=%x\n",
		miisc->mii_phy, cpsw_read_4(MDIOUSERPHYSEL0));

#if 0
	MGE_WRITE(sc, MGE_REG_PHYDEV, miisc->mii_phy);
#endif

	/* Attach interrupt handlers */
	for (i = 1; i <= CPSW_INTR_COUNT; ++i) {
		error = bus_setup_intr(dev, sc->res[i],
		    INTR_TYPE_NET | INTR_MPSAFE,
		    NULL, *cpsw_intrs[i].handler,
		    sc, &sc->ih_cookie[i - 1]);
		if (error) {
			device_printf(dev, "could not setup %s\n",
			    cpsw_intrs[i].description);
			cpsw_detach(dev);
			return (error);
		}
	}

	return (0);
}

static int
cpsw_detach(device_t dev)
{
	struct cpsw_softc *sc;
	int error,i;

	sc = device_get_softc(dev);

#if 0
	/* Stop controller and free TX queue */
	if (sc->ifp)
		cpsw_shutdown(dev);
#endif

	/* Wait for stopping ticks */
        callout_drain(&sc->wd_callout);

	/* Stop and release all interrupts */
	for (i = 0; i < CPSW_INTR_COUNT; ++i) {
		if (!sc->ih_cookie[i])
			continue;

		error = bus_teardown_intr(dev, sc->res[1 + i], sc->ih_cookie[i]);
		if (error)
			device_printf(dev, "could not release %s\n",
			    cpsw_intrs[i + 1].description);
	}

	/* Detach network interface */
	if (sc->ifp) {
		ether_ifdetach(sc->ifp);
		if_free(sc->ifp);
	}

#if 0
	/* Free DMA resources */
	cpsw_free_dma(sc);
#endif
	/* Free IO memory handler */
	bus_release_resources(dev, res_spec, sc->res);

	/* Destroy mutexes */
	mtx_destroy(&sc->receive_lock);
	mtx_destroy(&sc->transmit_lock);

	return (0);
}

static int
cpsw_suspend(device_t dev)
{

	device_printf(dev, "%s\n", __FUNCTION__);
	return (0);
}

static int
cpsw_resume(device_t dev)
{

	device_printf(dev, "%s\n", __FUNCTION__);
	return (0);
}

static int
cpsw_shutdown(device_t dev)
{
	struct cpsw_softc *sc = device_get_softc(dev);
	printf("%s: unimplemented\n",__func__);
	return (0);
}

static int
cpsw_miibus_readreg(device_t dev, int phy, int reg)
{
	struct cpsw_softc *sc;
	uint32_t r;
	uint32_t retries = CPSW_MIIBUS_RETRIES;

	sc = device_get_softc(dev);
	printf("%s: phy=0x%x reg=0x%x ",__func__,phy,reg);

	/* Wait until interface is ready by watching GO bit */
	while(--retries && (cpsw_read_4(MDIOUSERACCESS0) & (1 << 31)) )
		DELAY(CPSW_MIIBUS_DELAY);
	if (!retries)
		device_printf(dev, "Timeout while waiting for MDIO.\n");

	/* Set GO, phy and reg */
	cpsw_write_4(MDIOUSERACCESS0, (1 << 31) |
		((reg & 0x1F) << 21) | ((phy & 0x1F) << 16));

	while(--retries && (cpsw_read_4(MDIOUSERACCESS0) & (1 << 31)) )
		DELAY(CPSW_MIIBUS_DELAY);
	if (!retries)
		device_printf(dev, "Timeout while waiting for MDIO.\n");

	r = cpsw_read_4(MDIOUSERACCESS0);
	/* Check for ACK */
	if(r & (1<<29)) {
		printf(" ok r=0x%08x\n", r);
		return (r & 0xFFFF);
	}
	device_printf(dev, "Failed to read from PHY.\n");
	return 0;
}

static int
cpsw_miibus_writereg(device_t dev, int phy, int reg, int value)
{
	struct cpsw_softc *sc;
	uint32_t retries = CPSW_MIIBUS_RETRIES;

	sc = device_get_softc(dev);
	printf("%s: phy=0x%x reg=0x%x value=0x%x",__func__,phy,reg,value);

	/* Wait until interface is ready by watching GO bit */
	while(--retries && (cpsw_read_4(MDIOUSERACCESS0) & (1 << 31)) )
		DELAY(CPSW_MIIBUS_DELAY);
	if (!retries)
		device_printf(dev, "Timeout while waiting for MDIO.\n");

	/* Set GO, WRITE, phy, reg and value */
	cpsw_write_4(MDIOUSERACCESS0, (value & 0xFFFF) | (3 << 30) |
		((reg & 0x1F) << 21) | ((phy & 0x1F) << 16));

	while(--retries && (cpsw_read_4(MDIOUSERACCESS0) & (1 << 31)) )
		DELAY(CPSW_MIIBUS_DELAY);
	if (!retries)
		device_printf(dev, "Timeout while waiting for MDIO.\n");

	/* Check for ACK */
	if(cpsw_read_4(MDIOUSERACCESS0) & (1<<29)) {
		printf(" ok\n");
		return 0;
	}
	device_printf(dev, "Failed to write to PHY.\n");

	return 0;
}

static void
cpsw_start(struct ifnet *ifp)
{
	struct cpsw_softc *sc = ifp->if_softc;
	printf("%s: unimplemented\n",__func__);
#if 0
	MGE_TRANSMIT_LOCK(sc);

	mge_start_locked(ifp);

	MGE_TRANSMIT_UNLOCK(sc);
#endif
}

static void
cpsw_stop(struct cpsw_softc *sc)
{
	printf("%s: unimplemented\n",__func__);
}

static int
cpsw_ioctl(struct ifnet *ifp, u_long command, caddr_t data)
{
	struct cpsw_softc *sc = ifp->if_softc;
	struct ifreq *ifr = (struct ifreq *)data;
	int mask, error;
	uint32_t flags;

	error = 0;

	switch (command) {
	case SIOCSIFFLAGS:
		CPSW_GLOBAL_LOCK(sc);
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
				flags = ifp->if_flags ^ sc->cpsw_if_flags;
				if (flags & IFF_PROMISC)
					printf("%s: SIOCSIFFLAGS "
						"IFF_PROMISC unimplemented\n",
						__func__);

				if (flags & IFF_ALLMULTI)
					printf("%s: SIOCSIFFLAGS "
						"IFF_ALLMULTI unimplemented\n",
						__func__);
			} else
				cpsw_init_locked(sc);
		}
		else if (ifp->if_drv_flags & IFF_DRV_RUNNING)
			cpsw_stop(sc);

		sc->cpsw_if_flags = ifp->if_flags;
		CPSW_GLOBAL_UNLOCK(sc);
		break;
		printf("%s: SIOCSIFFLAGS\n",__func__);
		break;
	case SIOCADDMULTI:
		printf("%s: SIOCADDMULTI\n",__func__);
		break;
	case SIOCDELMULTI:
		printf("%s: SIOCDELMULTI\n",__func__);
		break;
	case SIOCSIFCAP:
		printf("%s: SIOCSIFCAP\n",__func__);
		break;
	case SIOCGIFMEDIA: /* fall through */
		printf("%s: SIOCGIFMEDIA\n",__func__);
	case SIOCSIFMEDIA:
		printf("%s: SIOCSIFMEDIA\n",__func__);
		error = ifmedia_ioctl(ifp, ifr, &sc->mii->mii_media, command);
		break;
	default:
		printf("%s: default\n",__func__);
		error = ether_ioctl(ifp, command, data);
	}
	return (error);
}

static void
cpsw_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct cpsw_softc *sc = ifp->if_softc;
	struct mii_data *mii;

	printf("%s: unimplemented\n",__func__);
#if 0
	MGE_TRANSMIT_LOCK(sc);

	mii = sc->mii;
	mii_pollstat(mii);

	ifmr->ifm_active = mii->mii_media_active;
	ifmr->ifm_status = mii->mii_media_status;

	MGE_TRANSMIT_UNLOCK(sc);
#endif
}


static int
cpsw_ifmedia_upd(struct ifnet *ifp)
{
	struct cpsw_softc *sc = ifp->if_softc;

	printf("%s: unimplemented\n",__func__);
#if 0
	if (ifp->if_flags & IFF_UP) {
		MGE_GLOBAL_LOCK(sc);

		sc->mge_media_status = sc->mii->mii_media.ifm_media;
		mii_mediachg(sc->mii);
		mge_init_locked(sc);

		MGE_GLOBAL_UNLOCK(sc);
	}
#endif
	
	return (0);
}

static void
cpsw_intr_rx_thresh(void *arg)
{
	printf("%s: unimplemented\n",__func__);
}

static void
cpsw_intr_rx(void *arg)
{
	printf("%s: unimplemented\n",__func__);
}

static void
cpsw_intr_tx(void *arg)
{
	printf("%s: unimplemented\n",__func__);
}

static void
cpsw_intr_misc(void *arg)
{
	printf("%s: unimplemented\n",__func__);
}


static int
cpsw_ale_read_entry(struct cpsw_softc *sc, uint16_t idx, uint32_t *ale_entry)
{
	cpsw_write_4(CPSW_ALE_TBLCTL, idx & 1023);
	ale_entry[0] = cpsw_read_4(CPSW_ALE_TBLW0);
	ale_entry[1] = cpsw_read_4(CPSW_ALE_TBLW1);
	ale_entry[2] = cpsw_read_4(CPSW_ALE_TBLW2);
}

static int
cpsw_ale_write_entry(struct cpsw_softc *sc, uint16_t idx, uint32_t *ale_entry)
{
	cpsw_write_4(CPSW_ALE_TBLW0, ale_entry[0]);
	cpsw_write_4(CPSW_ALE_TBLW1, ale_entry[1]);
	cpsw_write_4(CPSW_ALE_TBLW2, ale_entry[2]);
	cpsw_write_4(CPSW_ALE_TBLCTL, (idx & 1023) | (1 << 31));
}


static inline void
cpsw_ale_configure(struct cpsw_softc *sc)
{
	uint32_t reg;
	int i;
	
	/* CLEAR_TABLE and ENABLE_ALE */
	cpsw_write_4(CPSW_ALE_CONTROL, (3<<30));

	/* Set port state to forward for ports 0, 1 and 2 */
	for(i=0;i<3;i++) {
		reg = cpsw_read_4(CPSW_ALE_PORTCTL(i));
		reg |= 0x3;
		cpsw_write_4(CPSW_ALE_PORTCTL(i), reg);
	}

#if 0
	sitaraif_ale_unicastentry_set(sitaraif, 0, (u8_t *)(sitaraif->eth_addr));

	sitaraif_ale_multicastentry_set(sitaraif,
                                PORT_0_MASK | PORT_1_MASK | PORT_2_MASK,
                                bcast_addr);
#endif
}

static void
cpsw_ale_dump_table(struct cpsw_softc *sc) {
	int i;
	uint32_t ale_entry[3];
	for(i=0; i< 1024; i++) {
		cpsw_ale_read_entry(sc, i, ale_entry);
		if (ale_entry[0] || ale_entry[1] || ale_entry[2]) {
			printf("ALE[%4u] %08x %08x %08x ", i, ale_entry[0], 
				ale_entry[1],ale_entry[2]);
			printf("mac: %02x:%02x:%02x:%02x:%02x:%02x ", 
				(ale_entry[1] >> 8) & 0xFF,
				(ale_entry[1] >> 0) & 0xFF,
				(ale_entry[0] >>24) & 0xFF,
				(ale_entry[0] >>16) & 0xFF,
				(ale_entry[0] >> 8) & 0xFF,
				(ale_entry[0] >> 0) & 0xFF);
			printf( ((ale_entry[1]>>8)&1) ? "mcast " : "ucast ");
			printf("type: %u ", (ale_entry[1]>>28)&3);
			printf("port: %u ", (ale_entry[2]>>2)&7);
			printf("\n");
		}
	}
}

static void
cpsw_init(void *arg)
{
	struct cpsw_softc *sc = arg;

	CPSW_GLOBAL_LOCK(sc);
	cpsw_init_locked(arg);
	CPSW_GLOBAL_UNLOCK(sc);
}

static void
cpsw_init_locked(void *arg)
{
	struct cpsw_softc *sc = arg;
	printf("%s: unimplemented\n",__func__);

	/* Reset SS */
	cpsw_write_4(CPSW_SS_SOFT_RESET, 1);
	while(cpsw_read_4(CPSW_SS_SOFT_RESET) & 1);

	/* Reset writer */
	cpsw_write_4(CPSW_WR_SOFT_RESET, 1);
	while(cpsw_read_4(CPSW_WR_SOFT_RESET) & 1);

	/* Reset Sliver port 0 and 1 */
	cpsw_write_4(CPSW_SL_SOFT_RESET(0), 1);
	while(cpsw_read_4(CPSW_SL_SOFT_RESET(0)) & 1);
	cpsw_write_4(CPSW_SL_SOFT_RESET(1), 1);
	while(cpsw_read_4(CPSW_SL_SOFT_RESET(1)) & 1);

	cpsw_ale_dump_table(sc);

	/* Enable statistics for ports 0, 1 and 2 */
	cpsw_write_4(CPSW_SS_STAT_PORT_EN, 7);

        /* Select MII, Internal Delay mode */
	//HWREG(SOC_CONTROL_REGS + CONTROL_GMII_SEL) = 0x00;

	/* EOI_TX_PULSE */
	cpsw_write_4(CPSW_CPDMA_CPDMA_EOI_VECTOR, 2);
	/* EOI_RX_PULSE */
	cpsw_write_4(CPSW_CPDMA_CPDMA_EOI_VECTOR, 1);

	/* Set number of free rx buffers to 0 */
	cpsw_write_4(CPSW_CPDMA_RX_FREEBUFFER(0), 0);

	/* Enable TX */
	cpsw_write_4(CPSW_CPDMA_TX_CONTROL, 1);

	/* Enable RX DMA  */
	cpsw_write_4(CPSW_CPDMA_RX_CONTROL, 1);

	/* Set MACCONTROL for ports 0,1 GMII_EN(5), IFCTL_A(15), IFCTL_B(16) */
	cpsw_write_4(CPSW_SL_MACCONTROL(0), (1<<5) | (3<<15));
	cpsw_write_4(CPSW_SL_MACCONTROL(1), (1<<5) | (3<<15));

	/* Write channel 0 RX HDP */
	// cpsw_write_4(CPSW_CPDMA_RX_HDP(0), 0);  // FIXME active_head

	/* Enable interrupts for TX Channel 0 */
	cpsw_write_4(CPSW_CPDMA_TX_INTMASK_SET, 1);
	/* Enable TX interrupt receive for core 0 */
	cpsw_write_4(CPSW_WR_C_TX_EN(0), 1);

	/* Enable interrupts for RX Channel 0 */
	cpsw_write_4(CPSW_CPDMA_RX_INTMASK_SET, 1);
	/* Enable RX interrupt receive for core 0 */
	cpsw_write_4(CPSW_WR_C_RX_EN(0), 1);

	/* Activate network interface */
	sc->ifp->if_drv_flags |= IFF_DRV_RUNNING;
	sc->ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

}
