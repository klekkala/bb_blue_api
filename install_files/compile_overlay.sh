#!/bin/bash

MPU6050_OVERLAY="BBB_MPU-00A0"
BMP_OVERLAY="BBB_BMP280-00A0"
BUTLED_OVERLAY="BBB_BUTLED-00A0"

echo "Installing $MPU6050_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$MPU6050_OVERLAY.dtbo -b 0 -@ 2016-05-01/$MPU6050_OVERLAY.dts

echo "Installing $BMP_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$BMP_OVERLAY.dtbo -b 0 -@ 2016-05-01/$BMP_OVERLAY.dts

echo "Installing $BUTLED_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$BUTLED_OVERLAY.dtbo -b 0 -@ 2016-05-01/$BUTLED_OVERLAY.dts

