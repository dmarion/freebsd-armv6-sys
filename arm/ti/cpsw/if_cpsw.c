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

#include <arm/ti/cpsw/if_cpswreg.h>
#include <arm/ti/cpsw/if_cpswvar.h>

#include "miibus_if.h"

static struct cpsw_softc *cpsw_sc = NULL;

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
static void cpsw_start_locked(struct ifnet *ifp);
static int cpsw_ioctl(struct ifnet *ifp, u_long command, caddr_t data);
static int cpsw_allocate_dma(struct cpsw_softc *sc);
static int cpsw_new_rxbuf(bus_dma_tag_t tag, bus_dmamap_t map,
    struct mbuf **mbufp, bus_addr_t *paddr);

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


#define DUMP_RXBD(p) cpsw_cpdma_read_rxbd(p, &bd);					\
	printf("%s: RXBD[%3u] next=0x%08x bufptr=0x%08x bufoff=0x%04x "		\
	"buflen=0x%04x pktlen=0x%04x flags=0x%04x\n", __func__, p,	\
	bd.next, bd.bufptr, bd.bufoff,bd.buflen, bd.pktlen, bd.flags)


/* Locking macros */
#define CPSW_TX_LOCK(sc) do {					\
		mtx_assert(&(sc)->rx_lock, MA_NOTOWNED);		\
		mtx_lock(&(sc)->tx_lock);				\
} while (0)

#define CPSW_TX_UNLOCK(sc)	mtx_unlock(&(sc)->tx_lock)
#define CPSW_TX_LOCK_ASSERT(sc)	mtx_assert(&(sc)->tx_lock, MA_OWNED)

#define CPSW_RX_LOCK(sc) do {					\
		mtx_assert(&(sc)->tx_lock, MA_NOTOWNED);		\
		mtx_lock(&(sc)->rx_lock);				\
} while (0)

#define CPSW_RX_UNLOCK(sc)		mtx_unlock(&(sc)->rx_lock)
#define CPSW_RX_LOCK_ASSERT(sc)	mtx_assert(&(sc)->rx_lock, MA_OWNED)

#define CPSW_GLOBAL_LOCK(sc) do {					\
		if ((mtx_owned(&(sc)->tx_lock) ? 1 : 0) !=	\
		    (mtx_owned(&(sc)->rx_lock) ? 1 : 0)) {		\
			panic("mge deadlock possibility detection!");	\
		}							\
		mtx_lock(&(sc)->tx_lock);				\
		mtx_lock(&(sc)->rx_lock);				\
} while (0)

#define CPSW_GLOBAL_UNLOCK(sc) do {					\
		CPSW_RX_UNLOCK(sc);				\
		CPSW_TX_UNLOCK(sc);				\
} while (0)

#define CPSW_GLOBAL_LOCK_ASSERT(sc) do {				\
		CPSW_TX_LOCK_ASSERT(sc);				\
		CPSW_RX_LOCK_ASSERT(sc);				\
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
	uint8_t hwaddr[ETHER_ADDR_LEN] = {0xd4,0x94,0xa1,0x38,0xb9,0x13};
	int i, error, phy;
	uint32_t reg;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->node = ofw_bus_get_node(dev);

	if (device_get_unit(dev) == 0)
		cpsw_sc = sc;

	/* Get phy address from fdt */
	if (fdt_get_phyaddr(sc->node, &phy) != 0) {
		device_printf(dev, "failed to get PHY address from FDT\n");
		return (ENXIO);
	}
	/* Initialize mutexes */
	mtx_init(&sc->tx_lock, device_get_nameunit(dev),
		"cpsw TX lock", MTX_DEF);
	mtx_init(&sc->rx_lock, device_get_nameunit(dev),
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

	/* Allocate DMA, buffers, buffer descriptors */
	error = cpsw_allocate_dma(sc);
	if (error) {
		cpsw_detach(dev);
		return (ENXIO);
	}

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

	ifp->if_snd.ifq_drv_maxlen = CPSW_MAX_TX_BUFFERS - 1;
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
		    NULL, *cpsw_intrs[i - 1].handler,
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
	mtx_destroy(&sc->rx_lock);
	mtx_destroy(&sc->tx_lock);

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
	//struct cpsw_softc *sc = device_get_softc(dev);
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
	//printf("%s: phy=0x%x reg=0x%x ",__func__,phy,reg);

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
	//	printf(" ok r=0x%08x\n", r);
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

static int
cpsw_allocate_dma(struct cpsw_softc *sc)
{
	int err;
	int i;

	/* Allocate a busdma tag and DMA safe memory for tx mbufs. */
	err = bus_dma_tag_create(
		bus_get_dma_tag(sc->dev),	/* parent */
		1, 0,				/* alignment, boundary */
		BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
		BUS_SPACE_MAXADDR,		/* highaddr */
		NULL, NULL,			/* filtfunc, filtfuncarg */
		MCLBYTES, 1,			/* maxsize, nsegments */
		MCLBYTES, 0,			/* maxsegsz, flags */
		NULL, NULL,			/* lockfunc, lockfuncarg */
		&sc->mbuf_tx_dtag);		/* dmatag */

	if (err)
		return (ENOMEM);
	for (i = 0; i < CPSW_MAX_TX_BUFFERS; i++) {
		if ( bus_dmamap_create(sc->mbuf_tx_dtag, 0, &sc->tx_dmamap[i])) {
			if_printf(sc->ifp, "failed to create dmamap for rx mbuf\n");
			return (ENOMEM);
		}
	}


	/* Allocate a busdma tag and DMA safe memory for rx mbufs. */
	err = bus_dma_tag_create(
		bus_get_dma_tag(sc->dev),	/* parent */
		1, 0,				/* alignment, boundary */
		BUS_SPACE_MAXADDR_32BIT,	/* lowaddr */
		BUS_SPACE_MAXADDR,		/* highaddr */
		NULL, NULL,			/* filtfunc, filtfuncarg */
		MCLBYTES, 1,			/* maxsize, nsegments */
		MCLBYTES, 0,			/* maxsegsz, flags */
		NULL, NULL,			/* lockfunc, lockfuncarg */
		&sc->mbuf_rx_dtag);		/* dmatag */

	for (i = 0; i < CPSW_MAX_RX_BUFFERS; i++) {
		if ( bus_dmamap_create(sc->mbuf_rx_dtag, 0, &sc->rx_dmamap[i])) {
			if_printf(sc->ifp, "failed to create dmamap for rx mbuf\n");
			return (ENOMEM);
		}
	}

	return (0);
}

static int
cpsw_new_rxbuf(bus_dma_tag_t tag, bus_dmamap_t map, struct mbuf **mbufp,
    bus_addr_t *paddr)
{
	struct mbuf *new_mbuf;
	bus_dma_segment_t seg[1];
	int error;
	int nsegs;

	KASSERT(mbufp != NULL, ("NULL mbuf pointer!"));

	new_mbuf = m_getcl(M_DONTWAIT, MT_DATA, M_PKTHDR);
	if (new_mbuf == NULL)
		return (ENOBUFS);
	new_mbuf->m_len = new_mbuf->m_pkthdr.len = new_mbuf->m_ext.ext_size;

	if (*mbufp) {
		bus_dmamap_sync(tag, map, BUS_DMASYNC_POSTREAD);
		bus_dmamap_unload(tag, map);
	}

	error = bus_dmamap_load_mbuf_sg(tag, map, new_mbuf, seg, &nsegs,
	    BUS_DMA_NOWAIT);
	KASSERT(nsegs == 1, ("Too many segments returned!"));
	if (nsegs != 1 || error)
		panic("%s: nsegs(%d), error(%d)",__func__, nsegs, error);

	bus_dmamap_sync(tag, map, BUS_DMASYNC_PREREAD);

	(*mbufp) = new_mbuf;
	(*paddr) = seg->ds_addr;
	return (0);
}


static int
cpsw_encap(struct cpsw_softc *sc, struct mbuf *m0)
{
	bus_dma_segment_t segs[512];	// FIXME
	bus_dmamap_t mapp;
	int error;
	int seg, nsegs;

	printf("%s: start\n",__func__);

	mapp = sc->tx_dmamap[0];

	/* Create mapping in DMA memory */
	error = bus_dmamap_load_mbuf_sg(sc->mbuf_tx_dtag, mapp, m0, segs, &nsegs,
	    BUS_DMA_NOWAIT);

	if (error != 0 || nsegs != 1 ) {
		bus_dmamap_unload(sc->mbuf_tx_dtag, mapp);
		return ((error != 0) ? error : -1);
	}

	bus_dmamap_sync(sc->mbuf_tx_dtag, mapp, BUS_DMASYNC_PREWRITE);
	for (seg = 0; seg < nsegs; seg++) {
		printf("%s: seg=%u mapp=%x ds_len=%u ds_addr=%x\n", __func__,
			seg, mapp, segs[seg].ds_len, segs[seg].ds_addr);
	}

	printf("%s: end\n",__func__);
	return (0);
}

static void
cpsw_start(struct ifnet *ifp)
{
	struct cpsw_softc *sc = ifp->if_softc;
	CPSW_TX_LOCK(sc);

	cpsw_start_locked(ifp);

	CPSW_TX_UNLOCK(sc);
}

static void
cpsw_start_locked(struct ifnet *ifp)
{
	struct cpsw_softc *sc = ifp->if_softc;
	struct mbuf *m0, *mtmp;
	uint32_t queued = 0;

	printf("%s: start\n",__func__);

	CPSW_TX_LOCK_ASSERT(sc);

	if ((ifp->if_drv_flags & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING)
		return;

	for (;;) {
		printf("%s: for (;;)\n",__func__);
		/* Get packet from the queue */
		IF_DEQUEUE(&ifp->if_snd, m0);
		if (m0 == NULL)
			break;

		mtmp = m_defrag(m0, M_DONTWAIT);
		if (mtmp)
			m0 = mtmp;

		if (cpsw_encap(sc, m0)) {
			IF_PREPEND(&ifp->if_snd, m0);
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
			break;
		}
		queued++;
		BPF_MTAP(ifp, m0);
	}

	if (queued) {
		/* Enable transmitter and watchdog timer */
		printf("%s: process queued\n",__func__);
		//sc->wd_timer = 5;
	}
	printf("%s: done\n",__func__);
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
	int error;
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
			} else {
				printf("%s: SIOCSIFFLAGS cpsw_init_locked", __func__);
				cpsw_init_locked(sc);
			}
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
	//struct cpsw_softc *sc = ifp->if_softc;
	//struct mii_data *mii;

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

	printf("%s: start\n",__func__);
	if (ifp->if_flags & IFF_UP) {
		CPSW_GLOBAL_LOCK(sc);

		sc->cpsw_media_status = sc->mii->mii_media.ifm_media;
		mii_mediachg(sc->mii);
		cpsw_init_locked(sc);

		CPSW_GLOBAL_UNLOCK(sc);
	}

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
	//struct cpsw_softc *sc = arg;
	printf("%s: unimplemented\n",__func__);
}

static void
cpsw_intr_tx(void *arg)
{
	struct cpsw_softc *sc = arg;
	printf("%s: unimplemented\n",__func__);
	printf("CPSW_WR_C_TX_STAT(0)=%x\n", cpsw_read_4(CPSW_WR_C_TX_STAT(0)));
	printf("CPSW_CPDMA_TX_HDP(0)=%x\n", cpsw_read_4(CPSW_CPDMA_TX_HDP(0)));
	printf("CPSW_CPDMA_TX_CP(0)=%x\n", cpsw_read_4(CPSW_CPDMA_TX_CP(0)));
	cpsw_write_4(CPSW_CPDMA_CPDMA_EOI_VECTOR, 2);
}

static void
cpsw_intr_misc(void *arg)
{
	struct cpsw_softc *sc = arg;
	/* EOI_RX_PULSE */
	printf("%s: misc_stat=%x\n", __func__,
		 cpsw_read_4(CPSW_WR_C_MISC_STAT(0)));
	cpsw_write_4(CPSW_CPDMA_CPDMA_EOI_VECTOR, 3);
}


#if 0
static void
cpsw_get_dma_addr(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	u_int32_t *paddr;

	KASSERT(nseg == 1, ("wrong number of segments, should be 1"));
	paddr = arg;

	*paddr = segs->ds_addr;
}
#endif

static void
cpsw_tick(void *msc)
{
	struct cpsw_softc *sc = msc;

	/* Check for TX timeout */
	//cpsw_watchdog(sc);

	mii_tick(sc->mii);

	/* Check for media type change */
	if(sc->cpsw_media_status != sc->mii->mii_media.ifm_media) {
		printf("%s: media type changed (ifm_media=%x)\n",__func__, 
			sc->mii->mii_media.ifm_media);
		cpsw_ifmedia_upd(sc->ifp);
	}

	/* Schedule another timeout one second from now */
	callout_reset(&sc->wd_callout, hz, cpsw_tick, sc);
}


static void
cpsw_init(void *arg)
{
	struct cpsw_softc *sc = arg;
	printf("%s: unimplemented\n",__func__);
	CPSW_GLOBAL_LOCK(sc);
	cpsw_init_locked(arg);
	CPSW_GLOBAL_UNLOCK(sc);
}

static void
cpsw_init_locked(void *arg)
{
	struct cpsw_softc *sc = arg;
	struct cpsw_cpdma_bd bd;
	int i;

	printf("%s: unimplemented\n",__func__);

	/* Reset SS */
	cpsw_write_4(CPSW_SS_SOFT_RESET, 1);
	while(cpsw_read_4(CPSW_SS_SOFT_RESET) & 1);

	/* Reset writer */
	cpsw_write_4(CPSW_WR_SOFT_RESET, 1);
	while(cpsw_read_4(CPSW_WR_SOFT_RESET) & 1);

	/* Reset DMA */
	cpsw_write_4(CPSW_CPDMA_SOFT_RESET, 1);
	while(cpsw_read_4(CPSW_CPDMA_SOFT_RESET) & 1);
        for(i = 0; i < 8; i++) {
		cpsw_write_4(CPSW_CPDMA_TX_HDP(i), 0);
		cpsw_write_4(CPSW_CPDMA_RX_HDP(i), 0);
		cpsw_write_4(CPSW_CPDMA_TX_CP(i), 0);
		cpsw_write_4(CPSW_CPDMA_RX_CP(i), 0);
        }


	/* Reset Sliver port 0 and 1 */
	cpsw_write_4(CPSW_SL_SOFT_RESET(0), 1);
	while(cpsw_read_4(CPSW_SL_SOFT_RESET(0)) & 1);
	cpsw_write_4(CPSW_SL_SOFT_RESET(1), 1);
	while(cpsw_read_4(CPSW_SL_SOFT_RESET(1)) & 1);

	//cpsw_ale_dump_table(sc);

	/* Enable statistics for ports 0, 1 and 2 */
	cpsw_write_4(CPSW_SS_STAT_PORT_EN, 7);

        /* Select MII, Internal Delay mode */
	//HWREG(SOC_CONTROL_REGS + CONTROL_GMII_SEL) = 0x00;


	/* Initialize RX Buffer Descriptors */
	i = CPSW_MAX_RX_BUFFERS;
	bd.next = NULL;
	while (i--) {
		bd.bufptr = 0;
		bd.buflen = 0;
		bd.pktlen = 0;
		bd.flags = (1<<13);
		cpsw_new_rxbuf(sc->mbuf_rx_dtag, sc->rx_dmamap[i],
			&sc->rx_mbuf[i], (bus_addr_t *) &bd.bufptr);
		cpsw_cpdma_write_rxbd(i, &bd);
		DUMP_RXBD(i);
		bd.next = cpsw_cpdma_rxbd_paddr(i);
	}

#if 0
	error = bus_dmamem_alloc(sc->buffer_tag, &sc->buffer_vaddr,
		BUS_DMA_NOWAIT, &sc->buffer_map);
	if (error) {
		if_printf(sc->ifp, "bus_dmamem_alloc failed\n");
	}

	error = bus_dmamap_load(sc->buffer_tag, sc->buffer_map,sc->buffer, 2048,
		cpsw_get_dma_addr, &(sc->buffer_paddr), BUS_DMA_NOWAIT);

	bus_dmamap_sync(sc->buffer_tag, sc->buffer_map, BUS_DMASYNC_PREREAD);
	if_printf(sc->ifp," vaddr=%x paddr=%x\n", sc->buffer_vaddr, sc->buffer_paddr);
#endif
	/* EOI_TX_PULSE */
	cpsw_write_4(CPSW_CPDMA_CPDMA_EOI_VECTOR, 2);
	/* EOI_RX_PULSE */
	cpsw_write_4(CPSW_CPDMA_CPDMA_EOI_VECTOR, 1);

	/* Set number of free rx buffers to 0 */
	cpsw_write_4(CPSW_CPDMA_RX_FREEBUFFER(0), 0);

	/* Enable TX DMA */
	cpsw_write_4(CPSW_CPDMA_TX_CONTROL, 1);

	/* Enable RX DMA  */
	cpsw_write_4(CPSW_CPDMA_RX_CONTROL, 1);

	/* Set MACCONTROL for ports 0,1 GMII_EN(5), IFCTL_A(15), IFCTL_B(16) */
	cpsw_write_4(CPSW_SL_MACCONTROL(0), (1<<5) | (3<<15));
	cpsw_write_4(CPSW_SL_MACCONTROL(1), (1<<5) | (3<<15));

	/* Write channel 0 RX HDP */
	 cpsw_write_4(CPSW_CPDMA_RX_HDP(0), 0x4a102000);  // FIXME active_head

	/* Enable interrupts for TX Channel 0 */
	cpsw_write_4(CPSW_CPDMA_TX_INTMASK_SET, 1);
	/* Enable TX interrupt receive for core 0 */
	cpsw_write_4(CPSW_WR_C_TX_EN(0), 1);

	/* Enable interrupts for RX Channel 0 */
	cpsw_write_4(CPSW_CPDMA_RX_INTMASK_SET, 1);
	/* Enable RX interrupt receive for core 0 */
	cpsw_write_4(CPSW_WR_C_RX_EN(0), 1);

	/* Initialze MDIO - ENABLE, PREAMBLE=0, FAULTENB, CLKDIV=0xFF */
	/* TODO Calculate MDCLK=CLK/(CLKDIV+1) */
	cpsw_write_4(MDIOCONTROL, (1<<30) | (1<<18) | 0xFF);

	/* Activate network interface */
	sc->ifp->if_drv_flags |= IFF_DRV_RUNNING;
	sc->ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	sc->wd_timer = 0;

	callout_reset(&sc->wd_callout, hz, cpsw_tick, sc);
}
