#!/bin/bash

deviceName="/dev/gti2800-0"
if [ $# -ge 1 ] ; then  deviceName=$1 ; fi


devicePath=/sys/$(udevadm info -q path ${deviceName})

devicePath=${devicePath%GTI_PCIE*}

echo $devicePath

sudo sh -c "echo '1' > $devicePath/reset"
sudo sh -c "echo '1' > $devicePath/remove"
sudo sh -c "echo '1' > /sys/bus/pci/rescan"

