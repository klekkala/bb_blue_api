#!/bin/bash

OVERLAY="RoboticsCape-00A0"
CAPENAME="RoboticsCape"
MPU="BBBMPU6050-00A0"


echo "Installing Device Tree Overlay"

dtc -O dtb -o /lib/firmware/$OVERLAY.dtbo -b 0 -@ 2016-05-01/$OVERLAY.dts


echo $MPU
dtc -O dtb -o /lib/firmware/$MPU.dtbo -b 0 -@ 2016-05-01/$MPU.dts


