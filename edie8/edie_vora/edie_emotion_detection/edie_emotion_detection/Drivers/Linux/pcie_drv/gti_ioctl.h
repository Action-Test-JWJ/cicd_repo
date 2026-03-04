/***********************************************************************
*
* Copyright (c) 2017-2019 Gyrfalcon Technology Inc. All rights reserved.
* See LICENSE file in the project root for full license information.
*
************************************************************************/

#ifndef __GTI2800_IOCTL_H__
#define __GTI2800_IOCTL_H__

#include <asm/ioctl.h>

#define GTI2800_MAGIC 'G'

#define GTI_DATA_FORMAT_UNPACKED_16	1
#define GTI_DATA_FORMAT_UNPACKED_32	2
#define GTI_DATA_FORMAT_PACKED_32	3

#define GTI_CMD0	0
#define GTI_CMD1	1
#define GTI_CMD2	2
#define GTI_CMD3	3
#define GTI_CMD4	4
#define GTI_CMD5	5
#define GTI_CMD6	6
#define GTI_CMD7	7

#define GTI_CMD_IDLE			GTI_CMD0
#define GTI_CMD_SOFT_RESET		GTI_CMD1
#define GTI_CMD_CLOCK_DISABLE		GTI_CMD2
#define GTI_CMD_LOAD_FILTER_IMAGE	GTI_CMD3
#define GTI_CMD_LOAD_REG_0		GTI_CMD4
#define GTI_CMD_LOAD_REG_X		GTI_CMD5
#define GTI_CMD_READ_REG_X		GTI_CMD6
#define GTI_CMD_READ_STATUS		GTI_CMD7

/* Set the GTI2800 CMD mode */
#define ICHIP_RESET		_IOW(GTI2800_MAGIC, 1, int)

/* Set the GTI2800 CMD mode */
#define ISET_GTI_CMD		_IOW(GTI2800_MAGIC, 2, int)

/* Get the current GTI2800 CMD mode */
#define IGET_GTI_CMD		_IOR(GTI2800_MAGIC, 3, int)

/* Set the data write format */
#define ISET_GTI_DATA_FORMAT	_IOW(GTI2800_MAGIC, 4, int)

/* Get number of bytes available in TX FIFO */
#define IGET_AVAIL_RX_BYTES	_IOW(GTI2800_MAGIC, 5, int)

#endif /* __GTI2800_IOCTL_H__ */
