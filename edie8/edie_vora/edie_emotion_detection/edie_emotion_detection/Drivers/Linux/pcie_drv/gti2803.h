/***********************************************************************
*
* Copyright (c) 2017-2019 Gyrfalcon Technology Inc. All rights reserved.
* See LICENSE file in the project root for full license information.
*
************************************************************************/

#ifndef __GTI2803_H__
#define __GTI2803_H__


#include <linux/types.h>
#include <linux/device.h>

//#define BUFFER_SIZE 4096
#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE /* Enable both normal and verbose logging */

#define log_verbose(args...)    printk(args)
#define log_normal(args...)     printk(args)

#elif defined DEBUG_NORMAL /* Enable only normal logging */

#define log_verbose(x...)
#define log_normal(args...)     printk(args)

#else

#define log_normal(x...)
#define log_verbose(x...)

#endif /* DEBUG_VERBOSE and DEBUG_NORMAL */

#define PIO_PORTING

#ifdef PIO_PORTING

#define STATE_INIT	    0
#define STATE_IDLE	    1
#define STATE_POST	    2
#define STATE_RTN	    3

#define MAX_BARS        2       /**< Maximum number of BARs */
#define SUCCESS         0L
#define FAILURE         1L

#define NUM_FIPS	    			1
#define GYR_BUFSIZE		4096 //(224 * 224 * 3)		/* 150528 */

struct _fipData {
    int state;

    char data_out_buf[GYR_BUFSIZE];
    u32 data_out_buf_index;

    char data_in_buf[GYR_BUFSIZE];
    u32 data_in_buf_index;

    int open_count;
};
#endif

#define GYR_DATA_RTN_OFFSET(i)			(0x00)
#define GYR_DATA_POST_OFFSET(i)			(0x00)
#define MIN(a, b)	    ((a) <= (b) ? a : b)

typedef struct {
	struct cdev ch_dev;
	dev_t devBase;
	void *__iomem control_bar;
	int mode64;
	struct mutex 	mlock;
	int count;
	//unsigned char buffer[BUFFER_SIZE];
	char buffer[GYR_BUFSIZE];
	void *__iomem pio_bar;
	//int buffer_len;

	char * mapped_buffer;
#if 0 //def PIO_PORTING
	void *__iomem pio_bar;
	struct _fipData fipData[NUM_FIPS];
//#else
	u32 barMask;                    /**< Bitmask for BAR information */
	struct {
	    unsigned long basePAddr;    /**< Base address of device memory */
	    unsigned long baseLen;      /**< Length of device memory */
	    void __iomem * baseVAddr;   /**< VA - mapped address */
	} barInfo[MAX_BARS];

	u32 index;                      /**< Which interface is this */

	int userCount;                  /**< Number of registered users */

	struct _fipData fipData[NUM_FIPS];
#endif

}GTI2803;


#endif /* __GTI2800_H__ */
