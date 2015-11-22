#!/sbin/busybox sh

## Multi boot animation changer ##
# you need to put multiple bootanimationX.zip/bin where X is 1 to n
# priority is /data/local/bootanimation.bin, then /data/local/bootanimation1.zip and finally /system/media/bootanimation1.zip

#Remount /system RW
/sbin/busybox mount -o remount,rw /system

# script first checks for a bootanimation1 file, if found, we start to loop through the series on each boot
if [ -f /data/local/bootanimation1.bin ] || [ -f /data/local/bootanimation1.zip ] || [ -f /system/media/bootanimation1.zip ] ;
then
   if [ -f /data/local/bootanimation-loop.txt ] ; #check if loop txt file already exists
      then nb=$(/sbin/busybox head /data/local/bootanimation-loop.txt) # we read in it $nb, the animation number to play on boot
           nbi=`/sbin/busybox expr "$nb" + 1` # we increment it by 1, $nbi will be the animation to play on next boot
   else nb="1" #on first boot, loop txt file is absent, we set file to play as number 1
        nbi="2" # we manually increment to 2 for the number of animation to play on next boot
   fi;
else nb=""  # if no bootanimation1 is found, then we do not loop animations
     nbi="" # variables are set to empty so that we check later if a single bootanimation.zip/bin exists
fi;

#if the strings are not empty, it means bootanimation1 was found in previous check
if [ -n "$nb" ] && [ -n "$nbi" ] ;
then # we verify that the animation set to play on next boot exists
    if [ -f /data/local/bootanimation$nbi.bin ] || [ -f /data/local/bootanimation$nbi.zip ] || [ -f /system/media/bootanimation$nbi.zip ] ;
       then echo "$nbi" >/data/local/bootanimation-loop.txt # if it exists, we set loop txt file to $nbi (actual animation number+1)
    else echo "1" >/data/local/bootanimation-loop.txt # if next incremented animation does not exist, we restart loop from animation 1
    fi;
    if [ -f /data/local/bootanimation$nb.bin ] ; #we copy the animation to play on actual boot to its expected name
       then cp -af /data/local/bootanimation$nb.bin /data/local/bootanimation.bin
    elif [ -f /data/local/bootanimation$nb.zip ] ;
       then cp -af /data/local/bootanimation$nb.zip /data/local/bootanimation.zip
    elif [ -f /system/media/bootanimation$nb.zip ] ;
       then cp -af /system/media/bootanimation$nb.zip /system/media/bootanimation.zip
    fi;
fi;
