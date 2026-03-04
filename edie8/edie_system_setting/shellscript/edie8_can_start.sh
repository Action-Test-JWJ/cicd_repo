#!/bin/bash
/usr/bin/slcand -o -c -s8 /dev/ttyACM0 can0
/sbin/ifconfig can0 up
/sbin/ifconfig can0 txqueuelen 1000
