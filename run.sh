# usr/local/bin/bash

make
rm .depend.Roboto13p.o .depend.lcd.o .depend.myFont.o myFont.o lcd.o Roboto13p.o lcd.kld
kldunload lcd
kldload ./lcd.ko
echo "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum." > /dev/lcdMattPro
