#!/bin/sh
# AIK-Linux/cleanup: reset working directory
# osm0sis @ xda-developers

../scripts/dtbToolCM -2  -v -s 2048 -o split_img/boot.img-dtb -p ../scripts/dtc/ ../arch/arm/boot/dts/
echo "DTB Image created successfully";
exit 0;

