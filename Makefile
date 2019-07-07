KMOD =	lcd
SRCS =	lcd.c
SRCS+= gpio_if.h
SRCS+= spibus_if.h 
SRCS+= bus_if.h
SRCS+= device_if.h

.include <bsd.kmod.mk>
