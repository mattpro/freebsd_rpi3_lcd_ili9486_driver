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

#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include "spibus_if.h"
#include "gpio_if.h"

#include "lcd.h"
 

struct lcd_sc_t 		*lcd_sc;
static devclass_t 		lcd_devclass; 
static d_write_t 		lcd_write;

uint8_t lcdBuffer[320*480];


/* Zmienne dotyczace LCD */
volatile uint16_t LCD_HEIGHT = ILI9341_SCREEN_HEIGHT;
volatile uint16_t LCD_WIDTH	 = ILI9341_SCREEN_WIDTH;


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
    "lcdRpi", 					/* driver’s official name */
    lcd_methods, 			/* device method table */
    sizeof(struct  lcd_sc_t)
  };

static struct ofw_compat_data compat_data[] = {
	{ "mattpro,lcd", 	1 },
	{ "mattpro,touch", 	1},
	{ NULL, 		0},
};


/* Register LCD Newbus driver */
DRIVER_MODULE(lcdRpi, spibus, lcd_driver, lcd_devclass, NULL, NULL);




/* Adds LCD to SPI bus. */
static int lcd_probe(device_t dev)
{
	int rv;

	if( !ofw_bus_status_okay(dev) )
	{
		return (ENXIO);
	}

	if( ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0 )
	{ 
		return (ENXIO);
	}

	rv = BUS_PROBE_DEFAULT;

   	 uprintf("LCD Probe \n");
  	device_set_desc(dev, "Lcd MattPro");
  	return (rv); /* Only I can use this device. */
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
    ILI9341_init();

    return(0);
}

static int lcd_detach(device_t dev)
{
    mtx_destroy(&lcd_sc->mtx);
    return(0);
}

/* Shutdown */
static int lcd_shutdown(device_t dev)
{
    return(lcd_detach(dev));
}




/* LCD CONTROL */
void ILI9341_spiSendByte(uint8_t byte)
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


void ILI9341_spiSend(uint8_t* txData, uint8_t* rxData, uint32_t dataLen)
{
    struct spi_command spi_cmd;
	
    memset(&spi_cmd, 0, sizeof(struct spi_command));
    spi_cmd.tx_data = txData;
    spi_cmd.rx_data = rxData;
    spi_cmd.rx_data_sz = dataLen;
    spi_cmd.tx_data_sz = dataLen;
    SPIBUS_TRANSFER(device_get_parent(lcd_sc->dev), lcd_sc->dev, &spi_cmd);
}


/* Send command (char) to LCD  - OK */
void ILI9341_writeCommand(uint8_t command)
{
	PIN_RESET(LCD_DC);
	ILI9341_spiSendByte(command);
}

/* Send Data (char) to LCD */
void ILI9341_writeData(uint8_t Data)
{
	PIN_SET(LCD_DC);
	ILI9341_spiSendByte(Data);	
}

/* Reset LCD */
void ILI9341_reset( void )
{
	uprintf("LCD Reset \n");

	PIN_RESET(LCD_RST);
	DELAY(200000);
	PIN_SET(LCD_RST);
	DELAY(200000);
}

/*Ser rotation of the screen - changes x0 and y0*/
void ILI9341_setRotation(uint8_t rotation) 
{
	uprintf("LCD set rotation = %d \n", rotation);

	ILI9341_writeCommand(0x36);
	DELAY(100);
		
	switch(rotation) 
	{
		case SCREEN_VERTICAL_1:
			ILI9341_writeData(0x48);
			LCD_WIDTH = 240;
			LCD_HEIGHT = 320;
			break;
		case SCREEN_HORIZONTAL_1:
			ILI9341_writeData(0x28);
			LCD_WIDTH  = 320;
			LCD_HEIGHT = 240;
			break;
		case SCREEN_VERTICAL_2:
			ILI9341_writeData(0x98);
			LCD_WIDTH  = 240;
			LCD_HEIGHT = 320;
			break;
		case SCREEN_HORIZONTAL_2:
			ILI9341_writeData(0xF8);
			LCD_WIDTH  = 320;
			LCD_HEIGHT = 240;
			break;
		default:
			//EXIT IF SCREEN ROTATION NOT VALID!
			break;
	}
}


void ILI9341_drawPixel(uint16_t X,uint16_t Y,uint16_t Colour) 
{
//	uprintf("Draw pixel in x=%d Y=%d\n", X, Y);
	
	if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return;	//OUT OF BOUNDS!
	
	//ADDRESS	
	ILI9341_writeCommand(0x2A);

	//XDATA
	ILI9341_writeData( (uint8_t)(X >> 8));
	ILI9341_writeData( (uint8_t)(X));
	ILI9341_writeData( (uint8_t)((X+1) >> 8));
	ILI9341_writeData( (uint8_t)(X+1) );

	//ADDRESS
	ILI9341_writeCommand(0x2B);
					
	//YDATA
	ILI9341_writeData( (uint8_t)(Y >> 8) );
	ILI9341_writeData( (uint8_t)(Y) );
	ILI9341_writeData( (uint8_t)((Y+1) >> 8) );
	ILI9341_writeData( (uint8_t)(Y+1) );

	//ADDRESS	
	ILI9341_writeCommand(0x2C);
						
	//COLOUR	
	PIN_SET(LCD_DC);
	ILI9341_writeData( (uint8_t)(Colour >> 8) );
	ILI9341_writeData( (uint8_t)(Colour) );		
}

void LCD_fill(uint16_t color);

coid LCD_fill(uint16_t color)
{
	

}


void ILI9341_init(void)
{
	volatile int i;
	
	uprintf("LCD init start ... \n");
	GPIO_PIN_SETFLAGS( lcd_sc->dev_gpio, LED_PIN_NUMBER, GPIO_PIN_OUTPUT);	
	GPIO_PIN_SETFLAGS( lcd_sc->dev_gpio, LCD_DC_PIN_NUMBER, GPIO_PIN_OUTPUT);
	GPIO_PIN_SETFLAGS( lcd_sc->dev_gpio, LCD_RST_PIN_NUMBER, GPIO_PIN_OUTPUT); 

	
	// only for test 
	for( i = 0 ; i < 10 ; i ++ )
	{
		GPIO_PIN_TOGGLE(lcd_sc->dev_gpio, LED_PIN_NUMBER);
		//lcd_send(i);
		DELAY(50000);
	}

	ILI9341_reset();


	ILI9341_writeCommand(0x01); // Sleep out, also SW reset
        DELAY(150000);

        ILI9341_writeCommand(0x3A); // Interface Pixel Format
        ILI9341_writeData(0x55);

        ILI9341_writeCommand(0xC2); // Power Control 3 (For Normal Mode)
        ILI9341_writeData(0x44);

        ILI9341_writeCommand(0xC5); // VCOM Control
        ILI9341_writeData(0x00);
        ILI9341_writeData(0x48);
        ILI9341_writeData(0x00);
        ILI9341_writeData(0x48);

        ILI9341_writeCommand(0xE0); // PGAMCTRL(Positive Gamma Control)
        ILI9341_writeData(0x0F);
        ILI9341_writeData(0x1F);
        ILI9341_writeData(0x1C);
        ILI9341_writeData(0x0C);
        ILI9341_writeData(0x0F);
        ILI9341_writeData(0x08);
        ILI9341_writeData(0x48);
        ILI9341_writeData(0x98);
        ILI9341_writeData(0x37);
        ILI9341_writeData(0x0A);
        ILI9341_writeData(0x13);
        ILI9341_writeData(0x04);
        ILI9341_writeData(0x11);
        ILI9341_writeData(0x0D);
        ILI9341_writeData(0x00);

        ILI9341_writeCommand(0xE1); // NGAMCTRL (Negative Gamma Correction)
        ILI9341_writeData(0x0F);
        ILI9341_writeData(0x32);
        ILI9341_writeData(0x2E);
        ILI9341_writeData(0x0B);
        ILI9341_writeData(0x0D);
        ILI9341_writeData(0x05);
        ILI9341_writeData(0x47);
        ILI9341_writeData(0x75);
        ILI9341_writeData(0x37);
        ILI9341_writeData(0x06);
        ILI9341_writeData(0x10);
        ILI9341_writeData(0x03);
        ILI9341_writeData(0x24);
        ILI9341_writeData(0x20);
        ILI9341_writeData(0x00);

	ILI9341_writeCommand(0x11);

	DELAY(150000);	

        ILI9341_writeCommand(0x20); // Display Inversion OFF   RPi LCD (A)
//      ILI9341_writeCommand(0x21); // Display Inversion ON    RPi LCD (B)

        ILI9341_writeCommand(0x36); // Memory Access Control
        ILI9341_writeData(0x48);

        ILI9341_writeCommand(0x29); // Display ON
        DELAY(150000);

	ILI9341_writeCommand(0x2A);
	ILI9341_writeData(0x00);
	ILI9341_writeData(0x00);
	ILI9341_writeData(0x01);
	ILI9341_writeData(0x3F);

	ILI9341_writeCommand(0x2B);
	ILI9341_writeData(0x00);
	ILI9341_writeData(0x00);
	ILI9341_writeData(0x01);
	ILI9341_writeData(0xE0);


	DELAY(120000);

	ILI9341_writeCommand(0x2C);
	
	ILI9341_setRotation(0);

	int x;
	int y;
	
	for ( x = 0 ; x < 320; x++ )
	{	
		for ( y = 0 ; y < 480 ; y ++ )
		{
			ILI9341_drawPixel(x, y, 0xFFFF);
		}
	} 
	ILI9341_drawPixel(100, 100, 0xFFFF);
	


/*

	//SOFTWARE RESET
	ILI9341_writeCommand(0x01);
	DELAY(1000000);
		
	//POWER CONTROL A
	ILI9341_writeCommand(0xCB);
	ILI9341_writeData(0x39);
	ILI9341_writeData(0x2C);
	ILI9341_writeData(0x00);
	ILI9341_writeData(0x34);
	ILI9341_writeData(0x02);

	//POWER CONTROL B
	ILI9341_writeCommand(0xCF);
	ILI9341_writeData(0x00);
	ILI9341_writeData(0xC1);
	ILI9341_writeData(0x30);

	//DRIVER TIMING CONTROL A
	ILI9341_writeCommand(0xE8);
	ILI9341_writeData(0x85);
	ILI9341_writeData(0x00);
	ILI9341_writeData(0x78);

	//DRIVER TIMING CONTROL B
	ILI9341_writeCommand(0xEA);
	ILI9341_writeData(0x00);
	ILI9341_writeData(0x00);

	//POWER ON SEQUENCE CONTROL
	ILI9341_writeCommand(0xED);
	ILI9341_writeData(0x64);
	ILI9341_writeData(0x03);
	ILI9341_writeData(0x12);
	ILI9341_writeData(0x81);

	//PUMP RATIO CONTROL
	ILI9341_writeCommand(0xF7);
	ILI9341_writeData(0x20);

	//POWER CONTROL,VRH[5:0]
	ILI9341_writeCommand(0xC0);
	ILI9341_writeData(0x23);

	//POWER CONTROL,SAP[2:0];BT[3:0]
	ILI9341_writeCommand(0xC1);
	ILI9341_writeData(0x10);

	//VCM CONTROL
	ILI9341_writeCommand(0xC5);
	ILI9341_writeData(0x3E);
	ILI9341_writeData(0x28);

	//VCM CONTROL 2
	ILI9341_writeCommand(0xC7);
	ILI9341_writeData(0x86);

	//MEMORY ACCESS CONTROL
	ILI9341_writeCommand(0x36);
	ILI9341_writeData(0x48);

	//PIXEL FORMAT
	ILI9341_writeCommand(0x3A);
	ILI9341_writeData(0x55);

	//FRAME RATIO CONTROL, STANDARD RGB COLOR
	ILI9341_writeCommand(0xB1);
	ILI9341_writeData(0x00);
	ILI9341_writeData(0x18);

	//DISPLAY FUNCTION CONTROL
	ILI9341_writeCommand(0xB6);
	ILI9341_writeData(0x08);
	ILI9341_writeData(0x82);
	ILI9341_writeData(0x27);

	//3GAMMA FUNCTION DISABLE
	ILI9341_writeCommand(0xF2);
	ILI9341_writeData(0x00);

	//GAMMA CURVE SELECTED
	ILI9341_writeCommand(0x26);
	ILI9341_writeData(0x01);

	//POSITIVE GAMMA CORRECTION
	ILI9341_writeCommand(0xE0);
	ILI9341_writeData(0x0F);
	ILI9341_writeData(0x31);
	ILI9341_writeData(0x2B);
	ILI9341_writeData(0x0C);
	ILI9341_writeData(0x0E);
	ILI9341_writeData(0x08);
	ILI9341_writeData(0x4E);
	ILI9341_writeData(0xF1);
	ILI9341_writeData(0x37);
	ILI9341_writeData(0x07);
	ILI9341_writeData(0x10);
	ILI9341_writeData(0x03);
	ILI9341_writeData(0x0E);
	ILI9341_writeData(0x09);
	ILI9341_writeData(0x00);

	//NEGATIVE GAMMA CORRECTION
	ILI9341_writeCommand(0xE1);
	ILI9341_writeData(0x00);
	ILI9341_writeData(0x0E);
	ILI9341_writeData(0x14);
	ILI9341_writeData(0x03);
	ILI9341_writeData(0x11);
	ILI9341_writeData(0x07);
	ILI9341_writeData(0x31);
	ILI9341_writeData(0xC1);
	ILI9341_writeData(0x48);
	ILI9341_writeData(0x08);
	ILI9341_writeData(0x0F);
	ILI9341_writeData(0x0C);
	ILI9341_writeData(0x31);
	ILI9341_writeData(0x36);
	ILI9341_writeData(0x0F);

	//EXIT SLEEP
	ILI9341_writeCommand(0x11);
	DELAY(100000);

	//TURN ON DISPLAY
	ILI9341_writeCommand(0x29);

	//STARTING ROTATION
	ILI9341_setRotation(SCREEN_VERTICAL_1);
	uprintf("LCD init end ... \n");
	
	ILI9341_drawPixel(100,100, 0xFFFF);
	ILI9341_drawPixel(100,105, 0xFFFF);
	*/
	/*
	uprintf("Test Spi multiple send \n");
	uint8_t txData[10];
	uint8_t rxData[10];
	for ( i = 0 ; i < 1000 ; i ++ )
	{
		txData[0] = i;
		txData[1] = i;
		txData[2] = i;
		txData[3] = i;
		txData[4] = i;
		ILI9341_spiSend(txData, NULL, 5);
		DELAY(10000);

	}

	uprintf("Test SPI Byte \n");
	for( i = 0 ; i < 1000 ; i ++ )
	{	
		ILI9341_spiSendByte(i);
		DELAY(10000);
	}


	int k;
	uprintf("LCD DC PIN TEST \n");
	for (k = 0 ;  k < 100 ; k ++ )
	{
		PIN_SET(LCD_DC);
		DELAY(10000);
		PIN_RESET(LCD_DC);
		DELAY(10000);
	}
	uprintf("LCD RS PIN TEST  \n");
	int o;
	for ( o = 0 ; o < 100 ; o ++ )
	{	
		PIN_SET(LCD_RST);	
		DELAY(10000);
		PIN_RESET(LCD_RST);
		DELAY(10000);
	}
	*/
}

static int lcd_write(struct cdev *dev, struct uio *uio, int ioflag)
{
    return 0;
}

