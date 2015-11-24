#!/tmp/busybox sh
# Detects filesystem type automatically and mounts it
# Nedded because now we can have f2fs filesystem
# Credits for that goes to Dorimanx and Blastagator
# This workaround was done by Computoncio

BB=/tmp/busybox

systemfs=$(eval $($BB blkid /dev/block/bootdevice/by-name/system | $BB awk ' { print $3 } '); $BB echo $TYPE);
datafs=$(eval $($BB blkid /dev/block/bootdevice/by-name/userdata | $BB awk ' { print $3 } '); $BB echo $TYPE);
DATA_MOUNT=$($BB mount | $BB grep /data | $BB wc -l);

$BB mount -t $systemfs /dev/block/bootdevice/by-name/system /system;

if [ "$DATA_MOUNT" -eq "0" ]; then
	$BB mount -t $datafs /dev/block/bootdevice/by-name/userdata /data;
fi;

