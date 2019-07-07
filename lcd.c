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



struct lcd_disp_buff_t 	lcd_disp_buff;
struct lcd_sc_t 		*lcd_sc;
static devclass_t 		lcd_devclass; 
static d_write_t 		lcd_write;

static struct cdevsw lcd_cdevsw =
{
    .d_version = D_VERSION,
    .d_write   = lcd_write,
    .d_name    = "lcd"
};

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
    "lcd", 					/* driver’s official name */
    lcd_methods, 			/* device method table */
    sizeof(struct  lcd_sc_t)
  };

/* Register LCD Newbus driver */
DRIVER_MODULE(lcd, spibus, lcd_driver, lcd_devclass, NULL, NULL);



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
    lcd_sc->cdev_p = make_dev(&lcd_cdevsw,
							device_get_unit(dev),
							UID_ROOT,
							GID_WHEEL,
							0600, "lcd");
    return(0);
}

static int lcd_detach(device_t dev)
{
    mtx_destroy(&lcd_sc->mtx);
    destroy_dev(lcd_sc->cdev_p);
    return(0);
}

/* Shutdown */
static int lcd_shutdown(device_t dev)
{
    return(lcd_detach(dev));
}






void lcd_do_reset(void)
{
    GPIO_PIN_SET(lcd_sc->dev_gpio, LCD_RST, GPIO_PIN_HIGH);
    DELAY(10000); // 10 msec
    GPIO_PIN_SET(lcd_sc->dev_gpio, LCD_RST, GPIO_PIN_LOW);
    DELAY(10000); // 10 msec
    GPIO_PIN_SET(lcd_sc->dev_gpio, LCD_RST, GPIO_PIN_HIGH);
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

void lcd_send_cmd(uint8_t cmd)
{

}

void lcd_send_data(uint8_t *data, int len)
{

}

void lcd_render(void)
{

}

void lcd_set_pixel(uint8_t x, uint8_t y, uint8_t value)
{

}

void lcd_set_cursor(uint8_t x, uint8_t y)
{

}

void lcd_clear(void)
{

}

void lcd_write_char(char code, uint8_t scale)
{
   
}

void lcd_write_string(const char *str, uint8_t scale)
{

}

void lcd_init(void)
{
	int i;
	
	GPIO_PIN_SETFLAGS(lcd_sc->dev_gpio, LED_PIN_NUMBER, GPIO_PIN_OUTPUT);
	
	for ( i = 0 ; i < 250 ; i ++ )
	{
		GPIO_PIN_TOOGEL(lcd_sc->dev_gpio, LED_PIN_NUMBER);
		lcd_send(i);
		DELAY(50000);
	}
}

static int lcd_write(struct cdev *dev, struct uio *uio, int ioflag)
{
    return 0;
}

