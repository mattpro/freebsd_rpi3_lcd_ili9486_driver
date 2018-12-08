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

#include <sys/kthread.h>

#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include "spibus_if.h"
#include "gpio_if.h"

#include "lcd.h"

 
struct lcd_sc_t 		*lcd_sc;
static devclass_t 		lcd_devclass; 
static d_write_t 		lcd_write;

uint16_t lcdBuffer[LCD_SCREEN_WIDTH * LCD_SCREEN_HEIGHT];


/* Zmienne dotyczace LCD */
volatile uint16_t LCD_HEIGHT = LCD_SCREEN_HEIGHT;
volatile uint16_t LCD_WIDTH  = LCD_SCREEN_WIDTH;


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

static struct cdevsw lcd_cdevsw = {
	.d_version = D_VERSION,
	.d_write  = lcd_write,
	.d_name  = "MattProLCD"
};


/* Register LCD Newbus driver */
DRIVER_MODULE(lcdRpi, spibus, lcd_driver, lcd_devclass, NULL, NULL);


/* Adds LCD to SPI bus. */
static int lcd_probe(device_t dev)
{
	if( !ofw_bus_status_okay(dev) )
	{	
		uprintf("dskldkss 1 \n");
		return (ENXIO);
	}

	if( ofw_bus_is_compatible(dev, "mattpro,lcd") )
	{ 
		uprintf("Znaleziono LCD na szynie SPI \n");
		device_set_desc(dev, "LCD MattPro");
		return (BUS_PROBE_DEFAULT);
	}
	
        if( ofw_bus_is_compatible(dev, "mattpro,touch") )
        {
                uprintf("Znaleziono TOUCH na szynie SPI \n");
                device_set_desc(dev, "LCD MattPro");
                return (BUS_PROBE_DEFAULT);
     	}

  	return (ENXIO);
}

void lcd_task( void *arg);

void lcd_task( void *arg )
{
	for(;;)
	{
		LCD_fill(lcdBuffer, setColor(0,0,0) );
		DELAY(10000);
		LCD_fill(lcdBuffer, setColor(0xff,0,0) );
		DELAY(10000);
	}

} 

static void lcd_delayed_attach(void *xsc);


static int lcd_attach(device_t dev )
{ 
    uprintf("LCD attach \n");	

//    config_intrhook_oneshot( lcd_delayed_attach, sc );	
	
    lcd_sc = device_get_softc(dev);
    lcd_sc->devLcd = dev;
    lcd_sc->dev_gpio = devclass_get_device(devclass_find("gpio"), 0);
    if (lcd_sc->dev_gpio == NULL)
    {
		device_printf(lcd_sc->devLcd, "[LCD] Error: failed to get the GPIO dev\n");
		return (1);
    }
    mtx_init(&lcd_sc->mtx, device_get_nameunit(lcd_sc->devLcd), "LCD Mutex", MTX_DEF);
    LCD_init();

    lcd_sc->cdev_p = make_dev( &lcd_cdevsw, 
				device_get_unit(dev),
				UID_ROOT,
				GID_WHEEL,
				0600, "lcdMattPro");


//    kproc_create( &lcd_task, lcd_sc, &lcd_sc->p, 0, 0, "task");

    return(0);
}


static void lcd_delayed_attach( void *xsc)
{
	uprintf("Delay attach");
}


static int lcd_detach(device_t dev)
{
    mtx_destroy(&lcd_sc->mtx);
    destroy_dev(lcd_sc->cdev_p);
    return(0);
}

static int lcd_shutdown(device_t dev)
{
    return(lcd_detach(dev));
}



/* LCD CONTROL */
void LCD_spiSendByte(uint8_t byte)
{
    struct spi_command spi_cmd;

    memset(&spi_cmd, 0, sizeof(struct spi_command));
    spi_cmd.tx_data = &byte;
    spi_cmd.rx_data = NULL;
    spi_cmd.rx_data_sz = 0;
    spi_cmd.tx_data_sz = 1;
    SPIBUS_TRANSFER(device_get_parent(lcd_sc->devLcd), lcd_sc->devLcd, &spi_cmd);
}


void LCD_spiSend(uint8_t* txData, uint8_t* rxData, uint32_t dataLen)
{
    struct spi_command spi_cmd;
	
    memset(&spi_cmd, 0, sizeof(struct spi_command));
    spi_cmd.tx_data = txData;
    spi_cmd.rx_data = rxData;
    spi_cmd.rx_data_sz = dataLen;
    spi_cmd.tx_data_sz = dataLen;
    SPIBUS_TRANSFER(device_get_parent(lcd_sc->devLcd), lcd_sc->devLcd, &spi_cmd);
}


/* Send command (char) to LCD  - OK */
void LCD_writeCommand(uint8_t command)
{
	PIN_RESET(LCD_DC);
	LCD_spiSendByte(command);
	PIN_SET(LCD_DC);
}

/* Send Data (char) to LCD */
void LCD_writeData(uint8_t Data)
{
	PIN_SET(LCD_DC);
	LCD_spiSendByte(Data);	
}

/* Reset LCD */
void LCD_reset( void )
{
	uprintf("LCD Reset \n");

	PIN_RESET(LCD_RST);
	DELAY(200000);
	PIN_SET(LCD_RST);
	DELAY(200000);
}

/*Ser rotation of the screen - changes x0 and y0*/
void LCD_setRotation(uint8_t rotation) 
{
	uprintf("LCD set rotation = %d \n", rotation);

	LCD_writeCommand(0x36);
	DELAY(100);
		
	switch(rotation) 
	{
		case SCREEN_VERTICAL_1:
			LCD_writeData(0x48);
			LCD_WIDTH = 320;
			LCD_HEIGHT = 480;
			break;
		case SCREEN_HORIZONTAL_1:
			LCD_writeData(0x28);
			LCD_WIDTH  = 480;
			LCD_HEIGHT = 320;
			break;
		case SCREEN_VERTICAL_2:
			LCD_writeData(0x98);
			LCD_WIDTH  = 320;
			LCD_HEIGHT = 480;
			break;
		case SCREEN_HORIZONTAL_2:
			LCD_writeData(0xF8);
			LCD_WIDTH  = 480;
			LCD_HEIGHT = 320;
			break;
		default:
			//EXIT IF SCREEN ROTATION NOT VALID!
			break;
	}
}

void LCD_brightness(uint8_t brightness)
{
	// chyba trzeba wczesniej zainicjalizowac - rejestr 0x53
	LCD_writeCommand(0x51); // byc moze 2 bajty?
	LCD_writeData(brightness); // byc moze 2 bajty
}


uint16_t setColor(uint8_t r, uint8_t g, uint8_t b)
{
	uint16_t color;
	
	color  = MIN( (uint16_t)r, 0x1F ) << 11;
	color |= MIN( (uint16_t)g, 0x3F ) << 5;
	color |= MIN( (uint16_t)b, 0x1F );
	
	return color;
}


void LCD_drawPixel(uint16_t X,uint16_t Y,uint16_t colour) 
{
	if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return;	//OUT OF BOUNDS!
	
	LCD_writeCommand(0x2A); // Column Address Set
	LCD_writeData( (uint8_t)( X >> 8 ) );
	LCD_writeData( (uint8_t)( X ) );
	LCD_writeData( (uint8_t)( X >> 8 ) );
	LCD_writeData( (uint8_t)( X ) );

	LCD_writeCommand(0x2B); // Page Address Set 		
	LCD_writeData( (uint8_t)( Y >> 8 ) );
	LCD_writeData( (uint8_t)( Y ) );
	LCD_writeData( (uint8_t)( Y >> 8 ) );
	LCD_writeData( (uint8_t)( Y ) );

	LCD_writeCommand(0x2C); // Memory write
							
	LCD_writeData( (uint8_t)(colour >> 8) ); // maluj kolor
	LCD_writeData( (uint8_t)(colour) );		
}


void LCD_fill(uint16_t* buffer, uint16_t color)
{ 
	struct spi_command spi_cmd;
	int i;	

	for ( i = 0 ; i < LCD_WIDTH*LCD_HEIGHT ; i ++ )
	{
		lcdBuffer[i] = color;
	}

	LCD_writeCommand(0x2A);
	LCD_writeData(0x00);
	LCD_writeData(0x00);
	LCD_writeData(0x01);
	LCD_writeData(0x3F);

	LCD_writeCommand(0x2B);
	LCD_writeData(0x00);
	LCD_writeData(0x00);
	LCD_writeData(0x01);
	LCD_writeData(0xE0);
	
	LCD_writeCommand(0x2C); // Memory write?

	memset(&spi_cmd, 0, sizeof(struct spi_command) );

	for ( i = 0 ; i < 30  ; i ++ )
	{	
		spi_cmd.tx_data = (uint8_t*)&lcdBuffer[5120*i];
		spi_cmd.rx_data = NULL;
		spi_cmd.tx_data_sz = 2 * 5120;
		spi_cmd.rx_data_sz = 0;
    	SPIBUS_TRANSFER(device_get_parent(lcd_sc->devLcd), lcd_sc->devLcd, &spi_cmd);
	}
}

void LCD_showBuffer(uint16_t* buffer)
{ 
	struct spi_command spi_cmd;
	int i;	

	LCD_writeCommand(0x2A);
	LCD_writeData(0x00);
	LCD_writeData(0x00);
	LCD_writeData(0x01);
	LCD_writeData(0x3F);

	LCD_writeCommand(0x2B);
	LCD_writeData(0x00);
	LCD_writeData(0x00);
	LCD_writeData(0x01);
	LCD_writeData(0xE0);
	
	LCD_writeCommand(0x2C); // Memory write?

	memset(&spi_cmd, 0, sizeof(struct spi_command) );

	for ( i = 0 ; i < 30  ; i ++ )
	{	
		spi_cmd.tx_data = (uint8_t*)&buffer[5120*i];
		spi_cmd.rx_data = NULL;
		spi_cmd.tx_data_sz = 2 * 5120;
		spi_cmd.rx_data_sz = 0;
    	SPIBUS_TRANSFER(device_get_parent(lcd_sc->devLcd), lcd_sc->devLcd, &spi_cmd);
	}
}

void LCD_clear(uint16_t* buffer)
{ 
	LCD_fill(buffer, 0x0000);
}
	
void LCD_bufferClear(uint16_t* buffer)
{ 
	memset(buffer, 0, LCD_SCREEN_WIDTH * LCD_SCREEN_HEIGHT );
}



void LCD_init(void)
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

	LCD_reset();

//	LCD_writeCommand(0x01); // Soft Reset
//	DELAY(150000);			// 15 ms wait

	LCD_writeCommand(0x28); // Display OFF

	LCD_writeCommand(0x3A); // Interface Pixel Format
	LCD_writeData(0x55);	// 16 bit/pixel

	LCD_writeCommand(0xC2); // Power Control 3 (For Normal Mode)
	LCD_writeData(0x44);    // Cos z napieciem 

	LCD_writeCommand(0xC5); // VCOM Control
	LCD_writeData(0x00);  // const
	LCD_writeData(0x00);  // nVM ? 0x48
	LCD_writeData(0x00);  // VCOM voltage ref
	LCD_writeData(0x00);  // VCM out

	LCD_writeCommand(0xE0); // PGAMCTRL(Positive Gamma Control)
	LCD_writeData(0x0F);
	LCD_writeData(0x1F);
	LCD_writeData(0x1C);
	LCD_writeData(0x0C);
	LCD_writeData(0x0F);
	LCD_writeData(0x08);
	LCD_writeData(0x48);
	LCD_writeData(0x98);
	LCD_writeData(0x37);
	LCD_writeData(0x0A);
	LCD_writeData(0x13);
	LCD_writeData(0x04);
	LCD_writeData(0x11);
	LCD_writeData(0x0D);
	LCD_writeData(0x00);

	LCD_writeCommand(0xE1); // NGAMCTRL (Negative Gamma Correction)
	LCD_writeData(0x0F);
	LCD_writeData(0x32);
	LCD_writeData(0x2E);
	LCD_writeData(0x0B);
	LCD_writeData(0x0D);
	LCD_writeData(0x05);
	LCD_writeData(0x47);
	LCD_writeData(0x75);
	LCD_writeData(0x37);
	LCD_writeData(0x06);
	LCD_writeData(0x10);
	LCD_writeData(0x03);
	LCD_writeData(0x24);
	LCD_writeData(0x20);
	LCD_writeData(0x00);

	LCD_writeCommand(0x11);	// Exit sleep - Sleep OUT
	DELAY(120000);	 		// 120 ms wait

	LCD_writeCommand(0x20); // Display Inversion OFF   RPi LCD (A)
	//LCD_writeCommand(0x21); // Display Inversion ON    RPi LCD (B)

	LCD_writeCommand(0x36); // Memory Access Control
	LCD_writeData(0x48);

	LCD_writeCommand(0x29); // Display ON
	DELAY(150000);

	LCD_setRotation(0);
	

	int x;
	int y;
	
	uprintf("Write pixel test\n");
	for ( x = 0 ; x < 50 ; x++ )
	{	
		for ( y = 0 ; y < 50 ; y ++ )
		{
			LCD_drawPixel( x    , y, setColor( 0xFF, 0x00, 0x00 ) );
			LCD_drawPixel( x+50 , y, setColor( 0x00, 0xFF, 0x00 ) );
			LCD_drawPixel( x+100, y, setColor( 0x00, 0x00, 0xFF ) );
		}
	} 


/*
	uprintf("Writre buffer test\n");
	int u;
	int k;
	for ( u = 0 ; u < 50 ; u ++ )
	{	
		LCD_bufferClear( lcdBuffer );
		for ( k = 0 ; k < LCD_SCREEN_WIDTH*LCD_SCREEN_HEIGHT ; k++ )
		{
			lcdBuffer[k] = setColor( k%(u+9), k%(u+78), k%(u*4) );
		}
		
		LCD_showBuffer(lcdBuffer);
	}


	uprintf("Fill LCD test\n");
	int o;
	for ( o = 0 ; o < 10 ; o ++ )
	{
		LCD_fill( lcdBuffer, setColor( 0xFF, 0x00, 0x00 ) );
		DELAY(100000);
		LCD_fill( lcdBuffer, setColor( 0x00, 0xFF, 0x00 ) );
		DELAY(100000);
		LCD_fill( lcdBuffer, setColor( 0x00, 0x00, 0xFF ) );
		DELAY(100000);
	}
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
		LCD_spiSend(txData, NULL, 5);
		DELAY(10000);

	}

	uprintf("Test SPI Byte \n");
	for( i = 0 ; i < 1000 ; i ++ )
	{	
		LCD_spiSendByte(i);
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
	LCD_LOCK(lcd_sc);

	uint16_t color = 0;	
	uint8_t buff[6];

	uprintf("lcd write \n");	

	copyin(uio->uio_iov->iov_base,
			buff,
			MIN( uio->uio_iov->iov_len, 6 ) );
	
	color = buff[0] | ( (uint16_t)buff[1] << 8 );
	
	uprintf("Set color to: %d\n", color );

	LCD_fill( lcdBuffer, color  );

    	LCD_UNLOCK(lcd_sc);    
    	return 0;
}

