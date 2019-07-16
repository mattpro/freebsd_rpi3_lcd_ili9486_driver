# freebsd_rpi3_lcd_ili9486_driver
## 1. Descripction
This project presents the LCD display driver for a popular LCD TFT controller - ILI9486. 

For test I use:
* FreeBSD 12.0 downloaded from here:  [https://wiki.freebsd.org/FreeBSD/arm/Raspberry%20Pi](https://wiki.freebsd.org/FreeBSD/arm/Raspberry%20Pi)
* [3.5inch RPi LCD (A), 480x320](https://www.waveshare.com/3.5inch-rpi-lcd-a.htm)
* [Raspberry Pi 3B+](https://www.raspberrypi.org/products/raspberry-pi-3-model-b-plus/)

This driver is under development. At this moment You can use it to print text on screen from userland. For example:
```
echo "Lorem ipsum dolor sit amet, consectetur adipiscing elit." > /dev/lcdMattPro
```
shows text on lcd screen:
![2019-07-16 22 27 52](https://user-images.githubusercontent.com/31981020/61327403-1d41a180-a819-11e9-8f87-3514102161cc.jpg)

Hardware connections.

LCD from Waveshare fits perfect for RPI3. You don't need any extra connections - just put lcd on header. 

## 2. Download repo

Use git command for downloading driver:
```
%git clone https://github.com/mattpro/freebsd_rpi3_lcd_ili9486_driver
```

## 3. Update FDT
Befor use driver, You must update FDT. 
1. First step - run bash script:
```
root@generic:/usr/src/git/freebsd_rpi3_lcd_ili9486_driver # ./newDtboScript.sh
```
where /usr/src/git/freebsd_rpi3_lcd_ili9486_driver is a path to cloned git repository.

2. Secod step - add one line in file loader.conf with is located in /boot/ directory
To do this, change directory to:
```
cd /boot/
```
open loader.conf by tet editor ( for examle ee )
```
ee loader.conf
```
add to the end, line:
```
fdt_overlays="spigen-rpi3"
```
close, save file and reboot RPI3.

## 4. Compile
You can compile by Yourself doing make, or You Can use script 
```
./run.sh
```
this script run make, delet all not nessesary files and load driver in a kernel:
```
kldload ./lcd.ko
```
After that, new device "lcdMattPro" apper in /dev/ and You can use it for printing any text to the screen.
```
echo "Lorem ipsum dolor sit amet, consectetur adipiscing elit." > /dev/lcdMattPro
```

You must have kernel source to compile, if You don't have You can download it by use svn:
```
svnlite checkout https://svn.freebsd.org/base/releng/12.0/sys /usr/src/sys
```

## 5. Other
As default I use Roboto 13p Font, but You can use any font You need. Font can be converted to c files by using font converter:
![Font converter](https://downloads.riuson.com/lcd-image-converter/lcd-image-converter-20180211-beta.zip/preview)
And change font in function lcd_write to other font:
```
FONT_DrawString((uint8_t*)lcdBuffer, buff, 10, 10, &Robo13p);
```
to
```
FONT_DrawString((uint8_t*)lcdBuffer, buff, 10, 10, &myFont);
```


