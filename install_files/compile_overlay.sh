#!/bin/bash

MPU_OVERLAY="BBBMPU6500-00A0"
BMP_OVERLAY="BBBBMP280-00A0"

echo "Installing $MPU_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$MPU_OVERLAY.dtbo -b 0 -@ 2016-05-01/$MPU_OVERLAY.dts


echo "Installing $BMP_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$BMP_OVERLAY.dtbo -b 0 -@ 2016-05-01/$BMP_OVERLAY.dts


