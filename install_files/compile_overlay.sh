#!/bin/bash

MPU6050_OVERLAY="BBBMPU6050-00A0"
MPU6500_OVERLAY="BBBMPU6500-00A0"
BMP_OVERLAY="BBBBMP280-00A0"

echo "Installing $MPU6500_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$MPU6500_OVERLAY.dtbo -b 0 -@ 2016-05-01/$MPU6500_OVERLAY.dts

echo "Installing $MPU6050_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$MPU6050_OVERLAY.dtbo -b 0 -@ 2016-05-01/$MPU6050_OVERLAY.dts

echo "Installing $BMP_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$BMP_OVERLAY.dtbo -b 0 -@ 2016-05-01/$BMP_OVERLAY.dts


