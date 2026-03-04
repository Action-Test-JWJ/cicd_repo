/***********************************************************************
*
* Copyright (c) 2017-2019 Gyrfalcon Technology Inc. All rights reserved.
* See LICENSE file in the project root for full license information.
*
************************************************************************/

#if !defined(GTI_DRV_H_)
#define GTI_DRV_H_

#include <linux/types.h>
#include <linux/pci.h>

//#define GTI_DEBUG
#ifdef GTI_DEBUG
#define gti_debug(fmt, ...) pr_debug("%s():" fmt, \
		__func__, ##__VA_ARGS__)
#else
#define gti_debug(...)
#endif

typedef int (*gti_drv_init)(void *gti_dev);
typedef int (*gti_drv_remove)(void *gti_dev);

extern int gti2801_init(void *gti_dev);
extern int gti2803_init(void *gti_dev);
extern int gti2803_fpga_init(void *gti_dev);
extern int gti2803_dma_init(void *gti_dev);

extern int gti2801_remove(void *gti_dev);
extern int gti2803_remove(void *gti_dev);
extern int gti2803_fpga_remove(void *gti_dev);
extern int gti2803_dma_remove(void *gti_dev);

typedef struct{
	struct pci_dev *pDev;	/* pci device struct from probe() */
	struct pci_device_id id;
	struct class * pClass;
	int instance;
	void * private;
	gti_drv_init initFn;
	gti_drv_remove exitFn;
}GTI_DEV;


static inline void write_register(u32 value, void *iomem){
    gti_debug("write %px=%x\n",iomem,value);
	iowrite32(value, iomem);
}

static inline u32 read_register(void *iomem){
	u32 ret= ioread32(iomem);
    gti_debug("read %px=%x\n",iomem,ret);
    return ret;
}

static inline u32 build_u32(u32 hi, u32 lo)
{
	return ((hi & 0xFFFFUL) << 16) | (lo & 0xFFFFUL);
}
static inline u64 build_u64(u64 hi, u64 lo)
{
	return ((hi & 0xFFFFFFFULL) << 32) | (lo & 0xFFFFFFFFULL);
}




#endif

