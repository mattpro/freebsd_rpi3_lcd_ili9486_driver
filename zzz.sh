# usr/local/bin/bash


make
kldunload lcd
kldload ./lcd.ko
