/***********************************************************************
*
* Copyright (c) 2017-2019 Gyrfalcon Technology Inc. All rights reserved.
* See LICENSE file in the project root for full license information.
*
************************************************************************/

#ifndef __GTI2803_FPGA_H__
#define __GTI2803_FPGA_H__

#include <linux/types.h>
#include <linux/device.h>

#define REG_OFFSET_FIFO  0x0000
#define REG_OFFSET_CONFIG  0xD000
#define REG_OFFSET_CONTROL  0xE000
#define REG_OFFSET_SWITCH  0xF000
#define BUFFER_SIZE 4096

struct control_regs {
	u32 identifier[2];
	u32 timestamp;
	u32 temperature;
	u32 reset;  //bit 12 for reset, bit[0-9] reformatter channel number
	u32 reserved_1[22];	/* padding */

	u32 fifo_size;
	u32 h2c_fifo_full;
	u32 c2h_fifo_full;
	u32 in_out_selection;
	u32 sda_port_number;
	u32 dna_serial[2];
	u32 efuse_serial;
	u32 led_control;
} __packed;

struct axis_switch_regs {
	u32  command; //0x02 to commit changes
	u32  reserved_1[0xf];
	u32 sda_post;
	u32 reformmater_post;
	u32 cascade_out;
	u32 c2h_fifo;
} __packed;



typedef struct {
	struct cdev ch_dev;
	dev_t devBase;
	void *__iomem control_bar;
	int mode64;
	struct mutex 	mlock;
	int count;
	unsigned char buffer[BUFFER_SIZE];
}GTI2803_FPGA;

#endif /* __GTI2800_H__ */
