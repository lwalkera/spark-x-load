/*
 * (C) Copyright 2009
 * PASCO scientific <http://pasco.com>
 * Written-by: Laine Walker-Avina <lwalkera@pasco.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA
 */
#include <common.h>
#include <asm/io.h>

#ifdef CONFIG_USB_EHCI_OMAP3
#include <usb.h>
#include <asm/arch/bits.h>
#include "ehci.h"
#include "ehci-core.h"

/* USB/EHCI registers */
#define OMAP3_USBTLL_BASE		0x48062000
#define OMAP3_UHH_BASE			0x48064000
#define OMAP3_EHCI_BASE			0x48064800

#define REVISION				0x0
#define SYSCONFIG				0x10
#define SYSSTATUS				0x14

#define ENAWAKEUP				(1<<2)
#define SIDLEMODE_NOIDLE		(1<<3)
#define CACTIVITY				(1<<8)
#define MIDLEMODE_NOIDLE		(1<<12)
#define AUTOIDLE				(1<<0)

#define UHH_HOSTCONFIG			0x40
#define UHH_DEBUG_CSR			0x44

#define OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN		(1 << 2)
#define OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN		(1 << 3)
#define OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN		(1 << 4)
#define OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN		(1 << 5)
#define OMAP_UHH_HOSTCONFIG_P1_CONNECT_STATUS	(1 << 8)
#define OMAP_UHH_HOSTCONFIG_P2_CONNECT_STATUS	(1 << 9)
#define OMAP_UHH_HOSTCONFIG_P3_CONNECT_STATUS	(1 << 10)
#define	OMAP_UHH_HOSTCONFIG_ULPI_P1_BYPASS		(1 << 0)
#define	OMAP_UHH_HOSTCONFIG_ULPI_P2_BYPASS		(1 << 11)
#define	OMAP_UHH_HOSTCONFIG_ULPI_P3_BYPASS		(1 << 12)

#define EHCI_INSNREG04							(0xA0)
#define EHCI_INSNREG04_DISABLE_UNSUSPEND		(1 << 5)
#define	EHCI_INSNREG05_ULPI						(0xA4)
#define	EHCI_INSNREG05_ULPI_CONTROL_SHIFT		31
#define	EHCI_INSNREG05_ULPI_PORTSEL_SHIFT		24
#define	EHCI_INSNREG05_ULPI_OPSEL_SHIFT			22
#define	EHCI_INSNREG05_ULPI_REGADD_SHIFT		16
#define	EHCI_INSNREG05_ULPI_EXTREGADD_SHIFT		8
#define	EHCI_INSNREG05_ULPI_WRDATA_SHIFT		0

/* ULPI */

#define ULPI_SET(a)				(a + 1)
#define ULPI_CLR(a)				(a + 2)

/*
 * Register Map
 */
#define ULPI_VENDOR_ID_LOW			0x00
#define ULPI_VENDOR_ID_HIGH			0x01
#define ULPI_PRODUCT_ID_LOW			0x02
#define ULPI_PRODUCT_ID_HIGH		0x03
#define ULPI_FUNC_CTRL				0x04
#define ULPI_IFC_CTRL				0x07
#define ULPI_OTG_CTRL				0x0a
#define ULPI_USB_INT_EN_RISE		0x0d
#define ULPI_USB_INT_EN_FALL		0x10
#define ULPI_USB_INT_STS			0x13
#define ULPI_USB_INT_LATCH			0x14
#define ULPI_DEBUG					0x15
#define ULPI_SCRATCH				0x16
/* Optional Carkit Registers */
#define ULPI_CARCIT_CTRL			0x19
#define ULPI_CARCIT_INT_DELAY		0x1c
#define ULPI_CARCIT_INT_EN			0x1d
#define ULPI_CARCIT_INT_STS			0x20
#define ULPI_CARCIT_INT_LATCH		0x21
#define ULPI_CARCIT_PLS_CTRL		0x22
/* Other Optional Registers */
#define ULPI_TX_POS_WIDTH			0x25
#define ULPI_TX_NEG_WIDTH			0x26
#define ULPI_POLARITY_RECOVERY		0x27
/* Access Extended Register Set */
#define ULPI_ACCESS_EXTENDED		0x2f
/* Vendor Specific */
#define ULPI_VENDOR_SPECIFIC		0x30
/* Extended Registers */
#define ULPI_EXT_VENDOR_SPECIFIC	0x80

#define ULPI_FUNC_CTRL_RESET		(1 << 5)

#ifdef DEBUG
#define OMAP3_EHCI_DEBUG(...) printf(__VA_ARGS__)
#else
#define OMAP3_EHCI_DEBUG(...) while(0)
#endif

#define HW(x) *((volatile unsigned int *)(x))

extern void board_ehci_stop(void);
extern void board_ehci_init(void);

#if 0
static void omap_ehci_soft_phy_reset(u8 port)
{
	unsigned reg = 0;

	OMAP3_EHCI_DEBUG("PHY reset\n");

	reg = ULPI_FUNC_CTRL_RESET
		/* FUNCTION_CTRL_SET register */
		| (ULPI_SET(ULPI_FUNC_CTRL) << EHCI_INSNREG05_ULPI_REGADD_SHIFT)
		/* Write */
		| (2 << EHCI_INSNREG05_ULPI_OPSEL_SHIFT)
		/* PORTn */
		| ((port + 1) << EHCI_INSNREG05_ULPI_PORTSEL_SHIFT)
		/* start ULPI access*/
		| (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT);

	HW(OMAP3_EHCI_BASE+EHCI_INSNREG05_ULPI) = reg;

	/* Wait for ULPI access completion */
	while (HW(OMAP3_EHCI_BASE+EHCI_INSNREG05_ULPI)
			& (1 << EHCI_INSNREG05_ULPI_CONTROL_SHIFT));
}
#endif

/*
 * Create the appropriate control structures to manage
 * a new EHCI host controller.
 */
int ehci_hcd_init(void)
{
	unsigned int reg;

	OMAP3_EHCI_DEBUG("Initializing OMAP3 ECHI\n");

	// USBTLL soft reset
	HW(OMAP3_USBTLL_BASE+SYSCONFIG) |= SOFTRESET;
	while(!(HW(OMAP3_USBTLL_BASE+SYSSTATUS) & RESETDONE));
	OMAP3_EHCI_DEBUG("TLL reset done\n");

	//Force UHH and TLL no-idle
	HW(OMAP3_USBTLL_BASE+SYSCONFIG) =
		ENAWAKEUP | SIDLEMODE_NOIDLE | CACTIVITY;

	reg = HW(OMAP3_UHH_BASE + SYSCONFIG);
	reg |= ENAWAKEUP | SIDLEMODE_NOIDLE | CACTIVITY | MIDLEMODE_NOIDLE;
	reg &= ~AUTOIDLE;
	HW(OMAP3_UHH_BASE + SYSCONFIG) = reg;

	//setup PHY interface
	reg = HW(OMAP3_UHH_BASE + UHH_HOSTCONFIG);
	reg |=  ( OMAP_UHH_HOSTCONFIG_INCR4_BURST_EN
			| OMAP_UHH_HOSTCONFIG_INCR8_BURST_EN
			| OMAP_UHH_HOSTCONFIG_INCR16_BURST_EN);
	reg &= ~OMAP_UHH_HOSTCONFIG_INCRX_ALIGN_EN;
	reg &= ~(OMAP_UHH_HOSTCONFIG_P1_CONNECT_STATUS |
			OMAP_UHH_HOSTCONFIG_P2_CONNECT_STATUS  |
			OMAP_UHH_HOSTCONFIG_P3_CONNECT_STATUS);
	reg &= ~OMAP_UHH_HOSTCONFIG_ULPI_P2_BYPASS;
	HW(OMAP3_UHH_BASE + UHH_HOSTCONFIG) = reg;


	/*
	 * An undocumented "feature" in the OMAP3 EHCI controller,
	 * causes suspended ports to be taken out of suspend when
	 * the USBCMD.Run/Stop bit is cleared (for example when
	 * we do ehci_bus_suspend).
	 * This breaks suspend-resume if the root-hub is allowed
	 * to suspend. Writing 1 to this undocumented register bit
	 * disables this feature and restores normal behavior.
	 */
	HW(OMAP3_EHCI_BASE+EHCI_INSNREG04) = EHCI_INSNREG04_DISABLE_UNSUSPEND;

	//omap_ehci_soft_phy_reset(1);
	board_ehci_init();

	hccr = (struct ehci_hccr *)(OMAP3_EHCI_BASE);
	hcor = (struct ehci_hcor *)(OMAP3_EHCI_BASE + 0x10);

	OMAP3_EHCI_DEBUG("OMAP3-ehci: init \n");

	return 0;
}

/*
 * Destroy the appropriate control structures corresponding
 * the the EHCI host controller.
 */
int ehci_hcd_stop(void)
{
	/* Reset OMAP modules */
	OMAP3_EHCI_DEBUG("Resetting OMAP USBH\n");
	HW(OMAP3_UHH_BASE+SYSCONFIG) = SOFTRESET;
	while (!(HW(OMAP3_UHH_BASE+SYSSTATUS) & (1 << 0)));
	OMAP3_EHCI_DEBUG("USB Host reset\n");
	while (!(HW(OMAP3_UHH_BASE+SYSSTATUS) & (1 << 1)));
	OMAP3_EHCI_DEBUG("OHCI reset\n");
	while (!(HW(OMAP3_UHH_BASE+SYSSTATUS) & (1 << 2)));
	OMAP3_EHCI_DEBUG("EHCI reset\n");

	OMAP3_EHCI_DEBUG("Resetting OMAP USBTLL\n");
	HW(OMAP3_USBTLL_BASE+SYSCONFIG) = 1<<1;
	while (!(HW(OMAP3_USBTLL_BASE+SYSSTATUS) & (1 << 0)));

	board_ehci_stop();
	return 0;
}
#endif
