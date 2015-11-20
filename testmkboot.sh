#!/bin/bash

                cp arch/arm/boot/zImage AIK-Linux/split_img/boot.img-zImage
                cd AIK-Linux
		./repackimg.sh
		
		mv image-new.img ../boot.img


	