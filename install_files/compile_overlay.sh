#!/bin/bash

MPU6050_OVERLAY="BBB_MPU-00A0"
BMP_OVERLAY="BBB_BMP280-00A0"
BUTLED_OVERLAY="BBB_BUTLED-00A0"
BUTADC_OVERLAY="BBB_ADC-00A0"
BBBPWM_OVERLAY="BBB_PWM-00A0"
BBBEQEP_OVERLAY="BBB_EQEP-00A0"
BBBSERVO_OVERLAY="BBB_SERVO-00A0"
BBBBASE_OVERLAY="BBB_BASE-00A0"


echo "Installing $MPU6050_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$MPU6050_OVERLAY.dtbo -b 0 -@ 2016-05-01/overlays/$MPU6050_OVERLAY.dts

echo "Installing $BMP_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$BMP_OVERLAY.dtbo -b 0 -@ 2016-05-01/overlays/$BMP_OVERLAY.dts

echo "Installing $BUTLED_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$BUTLED_OVERLAY.dtbo -b 0 -@ 2016-05-01/overlays/$BUTLED_OVERLAY.dts

echo "Installing $BUTADC_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$BUTADC_OVERLAY.dtbo -b 0 -@ 2016-05-01/overlays/$BUTADC_OVERLAY.dts

echo "Installing $BBBPWM_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$BBBPWM_OVERLAY.dtbo -b 0 -@ 2016-05-01/overlays/$BBBPWM_OVERLAY.dts

echo "Installing $BBBEQEP_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$BBBEQEP_OVERLAY.dtbo -b 0 -@ 2016-05-01/overlays/$BBBEQEP_OVERLAY.dts

echo "Installing $BBBSERVO_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$BBBSERVO_OVERLAY.dtbo -b 0 -@ 2016-05-01/overlays/$BBBSERVO_OVERLAY.dts

echo "Installing $BBBBASE_OVERLAY Device Tree Overlay"
dtc -O dtb -o /lib/firmware/$BBBBASE_OVERLAY.dtbo -b 0 -@ 2016-05-01/$BBBBASE_OVERLAY.dts
