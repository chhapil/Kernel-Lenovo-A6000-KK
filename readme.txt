cd kernel/
mkdir out
make O=out ARCH=arm msm8916-perf_defconfig
make O=out ARCH=arm CROSS_COMPILE=arm-eabi-
