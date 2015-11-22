#!/bin/bash

###############################################################################
# To all DEV around the world :)                                              #
#                                                                             #
# 1.) use the "bash"                                                          #
# chsh -s /bin/bash `whoami`                                                  #
#                                                                             #
# 2.) load the ".config"                                                      #
# ./load_config.sh                                                            #
#                                                                             #
# 3.) clean the sources                                                       #
# ./clean_kernel.sh                                                           #
#                                                                             #
# 4.) now you can build my kernel                                             #
# ./build_kernel.sh                                                           #
#                                                                             #
# Have fun and update me if something nice can be added to my source.         #
###############################################################################

# location
KERNELDIR=$(readlink -f .);
export PATH=$PATH:tools/lz4demo

export PARENT_DIR=`readlink -f ${KERNELDIR}/..`;


# kernel
export ARCH=arm;
export USE_SEC_FIPS_MODE=true;
export KERNEL_CONFIG=.config;
KERNEL_CONFIG_FILE=msm8916-perf_defconfig

# build script
export USER=`whoami`;
export HOST=`uname -n`;
export TMPFILE=`mktemp -t`;

chmod -R 777 /tmp;

# system compiler
# gcc x.x.x
# export CROSS_COMPILE=$PARENT_DIR/toolchain/bin/arm-none-eabi-;

# gcc 4.8.3 (Linaro 2013.x)
export CROSS_COMPILE=$KERNELDIR/android-toolchain/bin/arm-eabi-;


    # begin by ensuring the required directory structure is complete, and empty
	echo "Initialising................."
	rm -rf "$KERNELDIR"/READY-KERNEL/boot
	rm -f "$KERNELDIR"/READY-KERNEL/*.zip
	rm -f "$KERNELDIR"/READY-KERNEL/*.img
	mkdir -p "$KERNELDIR"/READY-KERNEL/boot

	if [ -d ../ramdisk-tmp ]; then
		rm -rf ../ramdisk-tmp/*
	else
		mkdir ../ramdisk-tmp
		chown root:root ../ramdisk-tmp
		chmod 777 ../ramdisk-tmp
	fi;

	# force regeneration of .dtb and zImage files for every compile
	rm -f arch/arm/boot/dts/*.dtb
	rm -f arch/arm/boot/*.cmd
	rm -f arch/arm/boot/zImage
	rm -f arch/arm/boot/Image


	PYTHON_CHECK=$(ls -la /usr/bin/python | grep python3 | wc -l);
	PYTHON_WAS_3=0;

	if [ "$PYTHON_CHECK" -eq "1" ] && [ -e /usr/bin/python2 ]; then
		if [ -e /usr/bin/python2 ]; then
			rm /usr/bin/python
			ln -s /usr/bin/python2 /usr/bin/python
			echo "Switched to Python2 for building kernel will switch back when done";
			PYTHON_WAS_3=1;
		else
			echo "You need Python2 to build this kernel. install and come back."
			exit 1;
		fi;
	else
		echo "Python2 is used! all good, building!";
	fi;

	# move into the kernel directory and compile the main image
	echo "Compiling Kernel.............";
	cp arch/arm/configs/"$KERNEL_CONFIG_FILE" .config

        # get version from config
        GETVER=$(grep 'Kernel-.*-V' .config |sed 's/Kernel-//g' | sed 's/.*".//g' | sed 's/-L.*//g');
	
	# copy new config
	cp "$KERNELDIR"/.config "$KERNELDIR"/arch/arm/configs/"$KERNEL_CONFIG_FILE";

	# remove all old modules before compile
	for i in $(find "$KERNELDIR"/ -name "*.ko"); do
		rm -f "$i";
	done;

	# Copy needed dtc binary to system to finish the build.
	if [ ! -e /bin/dtc ]; then
		cp -a tools/dtc-binary/dtc /bin/;
	fi;

	# Idea by savoca
	NR_CPUS=$(grep -c ^processor /proc/cpuinfo)

	if [ "$NR_CPUS" -le "2" ]; then
		NR_CPUS=4;
		echo "Building kernel with 4 CPU threads";
	else
		echo "Building kernel with $NR_CPUS CPU threads";
	fi;

	# build zImage
	time make -j ${NR_CPUS}

	cp "$KERNELDIR"/.config "$KERNELDIR"/arch/arm/configs/"$KERNEL_CONFIG_FILE";

	stat "$KERNELDIR"/arch/arm/boot/zImage || exit 1;

	# compile the modules, and depmod to create the final zImage
	#echo "Compiling Modules............"
	#time make modules -j ${NR_CPUS} || exit 1

	# move the compiled zImage and modules into the READY-KERNEL working directory
	#echo "Move compiled objects........"
	
	# copy all ROOT ramdisk files to ramdisk temp dir.
	cp -a ../ramdisk/* ../ramdisk-tmp/
	
	#for i in $(find "$KERNELDIR" -name '*.ko'); do
	#	cp -av "$i" ../ramdisk-tmp/lib/modules/;
	#done;

	#chmod 755 ../ramdisk-tmp/lib/modules/*

	# remove empty directory placeholders from tmp-initramfs
	for i in $(find ../ramdisk-tmp/ -name EMPTY_DIRECTORY); do
		rm -f "$i";
	done;

	if [ -e "$KERNELDIR"/arch/arm/boot/zImage ]; then
		#cp arch/arm/boot/zImage READY-KERNEL/boot/

		# strip not needed debugs from modules.
		#android-toolchain/bin/arm-eabi-strip --strip-unneeded ../ramdisk-tmp/lib/modules/* 2>/dev/null
		#android-toolchain/bin/arm-eabi-strip --strip-debug ../ramdisk-tmp/lib/modules/* 2>/dev/null

		# create the ramdisk and move it to the output working directory
		#echo "Create ramdisk..............."
		#scripts/mkbootfs ../ramdisk-tmp | gzip > ramdisk.gz 2>/dev/null
		#mv ramdisk.gz READY-KERNEL/boot/

		# create the dt.img from the compiled device files, necessary for msm8974 boot images
		#echo "Create dt.img................"
		#./scripts/dtbTool -v -s 2048 -o READY-KERNEL/boot/dt.img arch/arm/boot/dts/

		if [ "$PYTHON_WAS_3" -eq "1" ]; then
			rm /usr/bin/python
			ln -s /usr/bin/python3 /usr/bin/python
		fi;

		# add kernel config to kernle zip for other devs
		cp "$KERNELDIR"/.config READY-KERNEL/

		# build the final boot.img ready for inclusion in flashable zip
		echo "Build boot.img..............."
		
		
		#cp scripts/mkbootimg READY-KERNEL/boot/
		#cd READY-KERNEL/boot/
		
		#kernel="zImage"
		#ramdisk="ramdisk.gz"
		#cmdline="'androidboot.hardware=qcom user_debug=31 msm_rtb.filter=0x3F ehci-hcd.park=3 androidboot.bootdevice=7824900.sdhci'"
		#board=""
		#base="80000000"
		#pagesize="2048"
                #kerneloff="00008000"
                #ramdiskoff="01000000"
                #tagsoff="00000100"
                #secondoff=""
                #dtb="--dt ../../kk-dtb/kk.dtb"

		#./mkbootimg --kernel zImage --ramdisk ramdisk.gz --cmdline "$cmd_line" --base $base --offset $offset --tags-addr $tags_addr --pagesize 2048 --dt dt.img -o newboot.img
		
		#./mkbootimg --kernel "$kernel" --ramdisk "$ramdisk" $second --cmdline "$cmdline" --base $base --pagesize $pagesize --kernel_offset $kerneloff --ramdisk_offset $ramdiskoff $secondoff --tags_offset $tagsoff $dtb -o newboot.img;
                cp arch/arm/boot/zImage AIK-Linux/split_img/boot.img-zImage
                cd AIK-Linux
		./repackimg.sh		
		mv image-new.img ../READY-KERNEL/boot.img		
		
		cd ../READY-KERNEL/		

		# cleanup all temporary working files
		echo "Post build cleanup..........."
		#cd ..
		rm -rf boot

		# BUMP boot.img with magic key to install on JB/KK bootloader
		#cd ..
		#sh kernel_bump.sh
		#mv READY-KERNEL/boot_bumped.img READY-KERNEL/boot.img
		#echo "Kernel BUMP done!";
		#cd READY-KERNEL/

		# create the flashable zip file from the contents of the output directory
		echo "Make flashable zip..........."
		zip -r Kernel-"${GETVER}"-KK-"$(date +"[%H-%M]-[%d-%m]-A6000-PWR-CORE")".zip * >/dev/null
		stat boot.img
		mv boot.img Kernel-"${GETVER}"-KK-"$(date +"[%H-%M]-[%d-%m]-A6000-PWR-CORE")".img
		
		cp *.zip ../READY-RELEASES/;
		cp *.img ../READY-RELEASES/;
		
		rm -f ./*.img
		cd ..
	else
		if [ "$PYTHON_WAS_3" -eq "1" ]; then
			rm /usr/bin/python
			ln -s /usr/bin/python3 /usr/bin/python
		fi;

		# with red-color
		echo -e "\e[1;31mKernel STUCK in BUILD! no zImage exist\e[m"
	fi;
	