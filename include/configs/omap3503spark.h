/*
 * (C) Copyright 2010
 * PASCO scientific
 * Laine Walker-Avina <lwalkera@pasco.com>
 *
 * (C) Copyright 2006
 * Texas Instruments <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * X-Loader Configuation settings for the Spark SLS HW2 board.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* serial printf facility takes about 3.5K */
#define CFG_PRINTF
//#undef CFG_PRINTF

/*
 * High Level Configuration Options
 */
#define CONFIG_ARMCORTEXA8	1	/* This is an ARM V7 CPU core */
#define CONFIG_OMAP		1	/* in a TI OMAP core */
#define CONFIG_OMAP34XX		1	/* which is a 34XX */
#define CONFIG_OMAP3503		1	/* which is in a 3503 */
#define CONFIG_OMAP3_SPARKHW2	1	/* working with SPARKHW2 */

#define CONFIG_ICACHE_OFF 1
#define CONFIG_L2_OFF 1

/* We always boot from MMC/eMMC */
#define CONFIG_MMC	1
#if defined(CONFIG_MMC)
	#define CFG_CMD_MMC		1
	#define CFG_CMD_FAT		1
#endif

#include <asm/arch/cpu.h>        /* get chip and board defs */

/* uncomment it if you need timer based udelay(). it takes about 250 bytes */
//#define CFG_UDELAY

/* Clock Defines */
#define V_OSCK	26000000  /* Clock output from T2 */

#if (V_OSCK > 19200000)
#define V_SCLK	(V_OSCK >> 1)
#else
#define V_SCLK	V_OSCK
#endif

//#define PRCM_CLK_CFG2_266MHZ	1	/* VDD2=1.15v - 133MHz DDR */
#define PRCM_CLK_CFG2_332MHZ	1	/* VDD2=1.15v - 166MHz DDR */
#define PRCM_PCLK_OPP2		1	/* ARM=381MHz - VDD1=1.20v */

/* Memory type */
#define CFG_3430SDRAM_DDR	1

/* The actual register values are defined in u-boot- mem.h */
/* SDRAM Bank Allocation method */
//#define SDRC_B_R_C		1
//#define SDRC_B1_R_B0_C	1
#define SDRC_R_B_C		1

#define OMAP34XX_GPMC_CS0_SIZE GPMC_SIZE_64M


/* Serial console configuation */

#ifdef CFG_PRINTF

#define CFG_NS16550
#define CFG_NS16550_SERIAL
#define CFG_NS16550_REG_SIZE	-4
#define CFG_NS16550_CLK		48000000
#define CFG_NS16550_COM3	OMAP34XX_UART3

/*
 * select serial console configuration
 */
#define CONFIG_SERIAL1		3	/* use UART3 */
#define CONFIG_CONS_INDEX	3

#define CONFIG_BAUDRATE		115200
#define CFG_PBSIZE		256

#endif /* CFG_PRINTF */

/*
 * Miscellaneous configurable options
 */
#define CFG_LOADADDR		0x80008000

#undef	CFG_CLKS_IN_HZ		/* everything, incl board info, in Hz */

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(128*1024) /* regular stack */

#define GPMC_CONFIG 		(OMAP34XX_GPMC_BASE+0x50)

/* Linux load info */
#include "asm/mach-types.h"

#define CONFIG_LOAD_LINUX		1
#define CONFIG_MACH_TYPE		MACH_TYPE_SPARK_SLS_HW2
#define CONFIG_ATAG_LOCATION	0x80000000


#endif /* __CONFIG_H */

