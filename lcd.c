
#include <sys/param.h>  
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/types.h>  
#include <sys/uio.h>

#include <sys/bus.h>
#include <dev/spibus/spi.h>
#include <dev/spibus/spibusvar.h>
#include <sys/gpio.h>   

#include <sys/lock.h>
#include <sys/mutex.h>

#include "spibus_if.h"
#include "gpio_if.h"

#include "lcd.h"


struct lcd_sc_t 		*lcd_sc;
static devclass_t 		lcd_devclass; 
static d_write_t 		lcd_write;


static device_method_t lcd_methods[] =
  {
    DEVMETHOD(device_probe, lcd_probe),
    DEVMETHOD(device_attach, lcd_attach),
    DEVMETHOD(device_detach, lcd_detach),
    DEVMETHOD(device_shutdown, lcd_shutdown),
    {0, 0}
  };
  
static driver_t lcd_driver =
  {
    "lcdRpiX", 					/* driver’s official name */
    lcd_methods, 			/* device method table */
    sizeof(struct  lcd_sc_t)
  };

/* Register LCD Newbus driver */
DRIVER_MODULE(lcdRpi, spibus, lcd_driver, lcd_devclass, NULL, NULL);



/* Adds LCD to SPI bus. */
static int lcd_probe(device_t dev)
{


  device_set_desc(dev, "Lcd MattPro");
  return (BUS_PROBE_SPECIFIC); /* Only I can use this device. */
}

static int lcd_attach(device_t dev)
{
    lcd_sc = device_get_softc(dev);
    lcd_sc->dev = dev;
    lcd_sc->dev_gpio = devclass_get_device(devclass_find("gpio"), 0);
    if (lcd_sc->dev_gpio == NULL)
    {
		device_printf(lcd_sc->dev, "[LCD] Error: failed to get the GPIO dev\n");
		return (1);
    }
    mtx_init(&lcd_sc->mtx, "LCD Mutex", NULL, MTX_DEF);
    lcd_init();

    return(0);
}

static int lcd_detach(device_t dev)
{
    mtx_destroy(&lcd_sc->mtx);
//    destroy_dev(lcd_sc->cdev_p);
    return(0);
}

/* Shutdown */
static int lcd_shutdown(device_t dev)
{
    return(lcd_detach(dev));
}




void lcd_send(uint8_t byte)
{
    struct spi_command spi_cmd;
    uint8_t temp;
    memset(&spi_cmd, 0, sizeof(struct spi_command));
    spi_cmd.tx_data = &byte;
    spi_cmd.rx_data = &temp;
    spi_cmd.rx_data_sz = 1;
    spi_cmd.tx_data_sz = 1;
    SPIBUS_TRANSFER(device_get_parent(lcd_sc->dev), lcd_sc->dev, &spi_cmd);
}

void lcd_reset( void )
{
	PIN_SET(LCD_RST);
	DELAY(50000);
	PIN_RESET(LCD_RST);
	DELAY(100000);
	PIN_SET(LCD_RST);
	DELAY(100);
}


void lcd_init(void)
{
	int i;
	
	GPIO_PIN_SETFLAGS(lcd_sc->dev_gpio, LED_PIN_NUMBER, GPIO_PIN_OUTPUT);
	
	GPIO_PIN_SETFLAGS( lcd_sc->dev_gpio, LCD_RS_PIN_NUMBER, GPIO_PIN_OUTPUT);
	GPIO_PIN_SETFLAGS( lcd_sc->dev_gpio, LCD_RST_PIN_NUMBER, GPIO_PIN_OUTPUT); 

	lcd_reset();



	for( i = 0 ; i < 10 ; i ++ )
	{
		GPIO_PIN_TOGGLE(lcd_sc->dev_gpio, LED_PIN_NUMBER);
		lcd_send(i);
		DELAY(100000);
	}
}

static int lcd_write(struct cdev *dev, struct uio *uio, int ioflag)
{
    return 0;
}

