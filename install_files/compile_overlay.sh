#!/bin/bash

OVERLAY="SD-101C-00A0"
CAPENAME="SD-101C"

echo "Installing Device Tree Overlay"

dtc -O dtb -o /lib/firmware/$OVERLAY.dtbo -b 0 -@ $OVERLAY.dts


