/***********************************************************************
*
* Copyright (c) 2017-2019 Gyrfalcon Technology Inc. All rights reserved.
* See LICENSE file in the project root for full license information.
*
************************************************************************/

#ifndef __GTI2800_H__
#define __GTI2800_H__

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

#ifdef MAYBE_UNUSED
#elif defined(__GNUC__) && !defined(DEBUG_VERBOSE)
# define MAYBE_UNUSED(x) UNUSED_ ## x __attribute__((unused))
#elif defined(__LCLINT__)
# define MAYBE_UNUSED(x) /*@unused@*/ x
#else
# define MAYBE_UNUSED(x) x
#endif



#define DEVICE_MINOR_BASE (0)
#define GTI_CHANNEL_NUMBER (4)


/* bits of the SG DMA control register */
#define XDMA_CTRL_RUN_STOP			(1UL << 0)
#define XDMA_CTRL_IE_DESC_STOPPED		(1UL << 1)
#define XDMA_CTRL_IE_DESC_COMPLETED		(1UL << 2)
#define XDMA_CTRL_IE_DESC_ALIGN_MISMATCH	(1UL << 3)
#define XDMA_CTRL_IE_MAGIC_STOPPED		(1UL << 4)
#define XDMA_CTRL_IE_INVALID_LENGTH		(1UL << 5)
#define XDMA_CTRL_IE_IDLE_STOPPED		(1UL << 6)
#define XDMA_CTRL_IE_READ_ERROR			(0x1FUL << 9)
#define XDMA_CTRL_IE_DESC_ERROR			(0x1FUL << 19)
#define XDMA_CTRL_NON_INCR_ADDR			(1UL << 25)
#define XDMA_CTRL_POLL_MODE_WB			(1UL << 26)

/* bits of the SG DMA status register */
#define XDMA_STAT_BUSY			(1UL << 0)
#define XDMA_STAT_DESC_STOPPED		(1UL << 1)
#define XDMA_STAT_DESC_COMPLETED	(1UL << 2)
#define XDMA_STAT_ALIGN_MISMATCH	(1UL << 3)
#define XDMA_STAT_MAGIC_STOPPED		(1UL << 4)
#define XDMA_STAT_FETCH_STOPPED		(1UL << 5)
#define XDMA_STAT_IDLE_STOPPED		(1UL << 6)
#define XDMA_STAT_READ_ERROR		(0x1FUL << 9)
#define XDMA_STAT_DESC_ERROR		(0x1FUL << 19)

/* bits of the SGDMA descriptor control field */
#define XDMA_DESC_STOPPED	(1UL << 0)
#define XDMA_DESC_COMPLETED	(1UL << 1)
#define XDMA_DESC_EOP		(1UL << 4)

#define DESC_MAGIC 0xAD4B0000UL

#define PCI_DMA_H(addr) ((addr >> 16) >> 16)
#define PCI_DMA_L(addr) (addr & 0xffffffffUL)


#define DMA_BUFFER_SIZE (0x1000*64)

#define REG_C2H_OFFSET  0x1000

#define REG_OFFSET_ENGINE  0x0000
struct engine_regs {
	u32 identifier;
	u32 control;
	u32 control_w1s;
	u32 control_w1c;
	u32 reserved_1[12];	/* padding */

	u32 status;
	u32 status_rc;
	u32 completed_desc_count;
	u32 alignments;
	u32 reserved_2[14];	/* padding */

	u32 poll_mode_wb_lo;
	u32 poll_mode_wb_hi;
	u32 interrupt_enable_mask;
	u32 interrupt_enable_mask_w1s;
	u32 interrupt_enable_mask_w1c;
	u32 reserved_3[9];	/* padding */

	u32 perf_ctrl;
	u32 perf_cyc_lo;
	u32 perf_cyc_hi;
	u32 perf_dat_lo;
	u32 perf_dat_hi;
	u32 perf_pnd_lo;
	u32 perf_pnd_hi;
} __packed;

#define REG_OFFSET_INTERRUPT 0x2000
struct interrupt_regs {
	u32 identifier;
	u32 user_int_enable;
	u32 user_int_enable_w1s;
	u32 user_int_enable_w1c;
	u32 channel_int_enable;
	u32 channel_int_enable_w1s;
	u32 channel_int_enable_w1c;
	u32 reserved_1[9];	/* padding */

	u32 user_int_request;
	u32 channel_int_request;
	u32 user_int_pending;
	u32 channel_int_pending;
	u32 reserved_2[12];	/* padding */

	u32 user_msi_vector[8];
	u32 channel_msi_vector[8];
} __packed;

#define REG_OFFSET_CONFIG 0x3000
struct config_regs {
	u32 identifier;
	u32 reserved_1[4];
	u32 msi_enable;
}__packed;

#define REG_OFFSET_SGDMA		0x4000
struct engine_sgdma_regs {
	u32 identifier;
	u32 reserved_1[31];	/* padding */

	/* bus address to first descriptor in Root Complex Memory */
	u32 first_desc_lo;
	u32 first_desc_hi;
	/* number of adjacent descriptors at first_desc */
	u32 first_desc_adjacent;
	u32 credits;
} __packed;


#define REG_OFFSET_SGDMA_COMMON 0x6000
struct sgdma_common_regs {
	u32 padding[9];
	u32 credit_feature_enable;
} __packed;

/**
 * Descriptor for a single contiguous memory block transfer.
 *
 * Multiple descriptors are linked by means of the next pointer. An additional
 * extra adjacent number gives the amount of extra contiguous descriptors.
 *
 * The descriptors are in root complex memory, and the bytes in the 32-bit
 * words must be in little-endian byte ordering.
 */
struct xdma_desc {
	u32 control;
	u32 bytes;		/* transfer length in bytes */
	u32 src_addr_lo;	/* source address (low 32-bit) */
	u32 src_addr_hi;	/* source address (high 32-bit) */
	u32 dst_addr_lo;	/* destination address (low 32-bit) */
	u32 dst_addr_hi;	/* destination address (high 32-bit) */
	/*
	 * next descriptor in the single-linked list of descriptors;
	 * this is the PCIe (bus) address of the next descriptor in the
	 * root complex memory
	 */
	u32 next_lo;		/* next desc address (low 32-bit) */
	u32 next_hi;		/* next desc address (high 32-bit) */
} __packed;

struct fpga_dev;

struct gti_dev{
	struct fpga_dev * fpga;
	struct cdev chdev;
	struct device * device;
	int channel;
	int mode;
	int count;
	char * buffer;
};

struct engine{
	struct fpga_dev * fpga;
	int direction;
};


struct fpga_dev {
	GTI_DEV *dev;
	int instance;		/* instance number */
	dev_t cdevno_base;


	struct gti_dev gti[GTI_CHANNEL_NUMBER];
	/* PCIe BAR management */
	void *__iomem control_bar;
	void *__iomem dma_bar;

	/* Interrupt management */
	int irq_count;		/* interrupt counter */
	int irq_line;		/* flag if irq allocated successfully */
	struct engine *tx_engine;
	struct engine *rx_engine;
	struct mutex 	mlock;
	spinlock_t slock;
	struct semaphore  sema;

	int state;
	unsigned int last_irq;
	wait_queue_head_t wq;
	dma_addr_t dma_handle;
	dma_addr_t desc_handle;
	char * tx_descriptor;
	char * rx_descriptor;
	char * buffer;
	int buffer_len;
};



#define SUCCESS         0L
#define FAILURE         1L

#define GYR_TMP_BUFSIZE	8
#define GYR_BUFSIZE	(224 * 224 * 3)		/* 150528 */

#define GYR_FIP_SPACING	0x10000
#define GYR_RX_OFFSET	(GYR_FIP_SPACING * 8)

#define GYR_CMD_OFFSET(i)			(0xe000 + (((i)+1) << 4)) 
#define GYR_STATUS_OFFSET(i)			(0xe004 + (((i)+1) << 4))
#define GYR_TX_FIFO_FULLNESS_OFFSET(i)		(0xe008 + (((i)+1) << 4)) 
#define GYR_RX_FIFO_FULLNESS_OFFSET(i)		(0xe00c + (((i)+1) << 4)) 
#define GYR_RX_SET_FIFO_SIZE_OFFSET(i)		(0xe088 + (((i)+1) << 4)) 
#define GYR_RX_BYTE_COUNT_OFFSET(i)		(0xe08c + (((i)+1) << 4)) 

#define GYR_DATA_POST_OFFSET(i)			((((i)+1) << 12) + 0x00)
#define GYR_DATA_RTN_OFFSET(i)			((((i)+1) << 12) + GYR_FIP_SPACING + 0x00)


#define GYR_CMD(cmd)				(cmd<<8)

#define MAX_FIPS	    			8

struct gti_fip_control_regs {
    u32 control;
    u32 status;
    u32 tx_fifo_fullness;
    u32 rx_fifo_fullness;
} __packed;

struct gti_control_regs {  
    u8 xdma_slave_control[0x1000];
    u8 reserved1[0xd000];
    u8 fpga_id[16];
    struct gti_fip_control_regs fip[MAX_FIPS];
    u32 tx_fifo_service_threshold;
    u32 rx_fifo_service_threshold; 
    u32 rx_fifo_almost_full_threshold;
    u32 fifo_size;
    u32 tx_fifo_service_flags;
    u32 rx_fifo_service_flags;
    u32 reserved2;
    u32 n_devices;
} __packed;

struct fip_data {
    u32 current_cmd;
};

struct gti_device {
    struct gti_control_regs *control_regs;
    struct fip_data fipdat[MAX_FIPS];
    void *tx_buffer;	// Used as a temporary buffer for mode changes
    void *dma_tx_baseaddr;
    void *dma_rx_baseaddr;
    int minor_base;
};

#endif /* __GTI2800_H__ */
