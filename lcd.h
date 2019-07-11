// Hardware connections
#define LED_PIN_NUMBER		26

#define LCD_DC_PIN_NUMBER	24
#define LCD_RST_PIN_NUMBER	25


#define ILI9341_SCREEN_HEIGHT 	240 
#define ILI9341_SCREEN_WIDTH 	320

#define SCREEN_VERTICAL_1			0
#define SCREEN_HORIZONTAL_1			1
#define SCREEN_VERTICAL_2			2
#define SCREEN_HORIZONTAL_2			3


struct lcd_sc_t
{
    device_t dev;
    device_t dev_gpio;
    struct mtx mtx;
};


#define PIN_SET( pin )		GPIO_PIN_SET( lcd_sc->dev, pin##_PIN_NUMBER, GPIO_PIN_HIGH)
#define PIN_RESET( pin )	GPIO_PIN_SET( lcd_sc->dev, pin##_PIN_NUMBER, GPIO_PIN_LOW)



void ILI9341_SPI_Send(unsigned char SPI_Data);
void ILI9341_writeCommand(uint8_t Command);
void ILI9341_writeData(uint8_t Data);
void ILI9341_reset(void);
void ILI9341_setRotation(uint8_t Rotation);
void ILI9341_init(void);

void ILI9341_drawPixel(uint16_t X,uint16_t Y,uint16_t Colour);


static int lcd_write(struct cdev *dev, struct uio *uio, int ioflag);
static int lcd_probe(device_t dev);
static int lcd_attach(device_t dev);
static int lcd_detach(device_t dev);
static int lcd_shutdown(device_t dev);
