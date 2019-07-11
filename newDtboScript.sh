# usr/local/bin/bash

echo 'Remove dtbo'
rm spigen-rpi3.dtbo
echo 'Compile dtso'
dtc -@ -o spigen-rpi3.dtbo spigen-rpi3.dtso
echo 'Remove dtbo from overlays'
rm /boot/dtb/overlays/spigen-rpi3.dtbo
echo 'Copy to overlays'
cp spigen-rpi3.dtbo /boot/dtb/overlays/
echo 'Operation end'

