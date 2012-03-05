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
	cpsw_ale_uc_entry_set(sc, 0,eth_addr);
	cpsw_ale_mc_entry_set(sc, PORT_0_MASK|PORT_1_MASK|PORT_2_MASK, bcast_addr);
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


