/*
 * Copyright (C) 2010 PASCO scientific
 * Laine Walker-Avina, lwalkera@pasco.com
 *
 * Copyright (C) 2005 Texas Instruments.
 *
 * (C) Copyright 2004
 * Jian Zhang, Texas Instruments, jzhang@ti.com.
 *
 * (C) Copyright 2002
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <part.h>
#include <fat.h>
#include <asm/arch/mem.h>
#include <asm/string.h>
#include <asm/arch/cpu.h>
#ifdef CONFIG_MMC
#include <mmc.h>
#endif
#ifdef CONFIG_USB_EHCI
#include <usb.h>
#endif

const char version_string[] =
	"PASCO scientific OMAPload 1.0.0 (" __DATE__ " - " __TIME__ ")";

#ifdef CONFIG_LOAD_LINUX
#define IMAGE_NAME	"zImage"
#else
#define IMAGE_NAME	"u-boot.bin"
#endif

int print_info(void)
{
#ifdef CFG_PRINTF
        printf("\n\n%s\n", version_string);
#endif
	return 0;
}

static int init_func_i2c (void)
{
#if defined(CONFIG_MMC) && defined(CONFIG_DRIVER_OMAP34XX_I2C)
	i2c_init (CFG_I2C_SPEED, CFG_I2C_SLAVE);
#endif
	return 0;
}

typedef int (init_fnc_t) (void);
#ifdef CONFIG_LOAD_LINUX
uint * board_setup_atags(char * cmdline);
#endif

init_fnc_t *init_sequence[] = {
	cpu_init,		/* basic cpu dependent setup */
	board_init,		/* basic board dependent setup */
#ifdef CONFIG_TIMER
	timer_init,
#endif
#ifdef CFG_NS16550_SERIAL
 	serial_init,		/* serial communications setup */
#endif
	print_info,
  	nand_init,		/* board specific nand init */
#ifdef CONFIG_MMC
	init_func_i2c,
#endif
  	NULL,
};

void start_armboot (void)
{
  	init_fnc_t **init_fnc_ptr;
 	int i, size;
	uchar *buf;
	int *first_instruction;
#ifdef CONFIG_LOAD_LINUX
	char *cmdline = CONFIG_ATAG_LOCATION + 0x1000;
#endif

   	for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
		if ((*init_fnc_ptr)() != 0) {
			hang ();
		}
	}

	misc_init_r();

	buf =  (uchar*) CFG_LOADADDR;

#ifdef CONFIG_MMC
	/* first try mmc */
	/* we use the "verbose" param of mmc_init to specifiy which mmc to try */
	for(i=1;i<3;i++)
	{
		if (mmc_init(i)) {
#ifdef CFG_PRINTF
			printf("Found mmc%d, looking for " IMAGE_NAME "...\n", i);
#endif
			size = file_fat_read(IMAGE_NAME, buf, 0);

			if (size > 0) {
#ifdef CFG_PRINTF
				printf("\nLoaded " IMAGE_NAME " (%d bytes) from mmc\n", size);
#endif
				buf += size;
#ifdef CONFIG_LOAD_LINUX
				size = file_fat_read("cmdline", cmdline, 0x800);
				if(size > 0) {
					cmdline[size] = 0;
#ifdef CFG_PRINTF
					printf("Loaded command line (%d bytes)\n", size);
#endif
				} else
					cmdline = NULL;
#endif
				break;
			} else
				printf(IMAGE_NAME " not found on mmc%d\n", i);
		}
	}
#endif
#ifdef CONFIG_USB_EHCI_OMAP3
	/* skip USB boot if we already got a valid image from MMC, except if
	 * sys_boot[5] was true when we reset */
	if(*((volatile int *)CONTROL_STATUS) & (1<<5) || buf == CFG_LOADADDR)
	{
		usb_stop();
		if(usb_init() >= 0 && usb_stor_scan(1) >= 0)
		{
			block_dev_desc_t * usb_blk_dev;
			usb_blk_dev = usb_stor_get_dev(0);
			fat_register_device(usb_blk_dev,1);

			printf("Found USB, looking for " IMAGE_NAME "...\n");
			buf = CFG_LOADADDR;
			size = file_fat_read(IMAGE_NAME, buf, 0);
			if(size > 0) {
				printf("\nLoaded " IMAGE_NAME " (%d bytes) from USB\n", size);
				buf += size;
#ifdef CONFIG_LOAD_LINUX
				size = file_fat_read("cmdline", cmdline, 0x800);
				if(size > 0) {
					cmdline[size] = 0;
					printf("Loaded command line (%d bytes)\n", size);
				} else
					cmdline = NULL;
#endif
			} else
				printf("couldn't read\n");
		}
		/* reset USB controller to prepare for whoever uses it next */
		usb_stop();
	}
#endif

#ifdef ONENAND_START_BLOCK
	if (buf == (uchar *)CFG_LOADADDR) {
		/* if no u-boot on mmc, try onenand or nand, depending upon sysboot */
		if (get_mem_type() == GPMC_ONENAND){
#ifdef CFG_PRINTF
       			printf("Loading u-boot.bin from onenand\n");
#endif
        		for (i = ONENAND_START_BLOCK; i < ONENAND_END_BLOCK; i++){
        			if (!onenand_read_block(buf, i))
        				buf += ONENAND_BLOCK_SIZE;
        		}
		} else if (get_mem_type() == GPMC_NAND){
#ifdef CFG_PRINTF
       			printf("Loading u-boot.bin from nand\n");
#endif
        		for (i = NAND_UBOOT_START; i < NAND_UBOOT_END; i+= NAND_BLOCK_SIZE){
        			if (!nand_read_block(buf, i))
        				buf += NAND_BLOCK_SIZE; /* advance buf ptr */
        		}
		}
	}
#endif

	/* if u-boot not found on mmc or
         * nand read result is erased data
         * then serial boot 
         */
	first_instruction = (int *)CFG_LOADADDR;
	if((buf == (uchar *)CFG_LOADADDR) || (*first_instruction == 0xffffffff)) {
		printf(IMAGE_NAME " not found or blank nand contents - attempting serial boot . . .\n");
		do_load_serial_bin(CFG_LOADADDR, 115200);
	}

#ifdef CFG_PRINTF
	printf("Booting " IMAGE_NAME "...\n");
#endif
#ifdef CONFIG_LOAD_LINUX
	void	(*theKernel)(int zero, int arch, uint * params) = 
		(void (*)(int, int, uint*))CFG_LOADADDR;

	cleanup_before_linux();

	theKernel(0, CONFIG_MACH_TYPE, board_setup_atags(cmdline));
#else
	/* go run U-Boot and never return */
	((init_fnc_t *)CFG_LOADADDR)();
#endif

	/* should never come here */
}

void hang (void)
{
	/* call board specific hang function */
	board_hang();

	/* if board_hang() returns, hang here */
#ifdef CFG_PRINTF
	printf("\nOMAPload - Data/prefetch abort\n");
#endif
	for (;;);
}
