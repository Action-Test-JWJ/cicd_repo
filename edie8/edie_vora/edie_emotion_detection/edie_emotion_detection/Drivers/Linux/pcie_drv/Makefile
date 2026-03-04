obj-m += gti_pcie_drv.o
gti_pcie_drv-objs := gti_drv.o gti2801.o gti2803.o gti2803_fpga.o gti2803_dma.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build

PWD       := $(shell pwd)

ROOT := $(dir $(M))

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

install: all
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install
	depmod -a
	install -m 644 70-gti.rules /etc/udev/rules.d

	# add the following line to force insmod gti_pcie_drv
	#modprobe gti_pcie_drv
	# add the following line to force insmod at boot up
	#if [ -z `grep gti_pcie_drv /etc/modules` ]; then sudo sed -i -e "\$agti_pcie_drv" /etc/modules; fi

clean:
	rm -rf *.o *.o.d *~ core .depend .*.cmd *.ko *.ko.unsigned *.mod.c .tmp_versions *.symvers *.ur-safe .#* *.save *.bak Modules.* modules.order Module.markers *.bin .cache.mk



