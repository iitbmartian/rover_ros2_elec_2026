#!/usr/bin/env bash

BIT_RATE=500000

if ! ifconfig | grep -q "can1" ; then 
    sudo ip link set can1 type can bitrate "$BIT_RATE"
    sudo ifconfig can1 up
    sudo ifconfig can1 txqueuelen 65536
fi