// Hardware connections
#define LED_PIN_NUMBER			26

#define LCD_DC_PIN_NUMBER		24
#define LCD_RST_PIN_NUMBER		25

// Screen settings
#define LCD_SCREEN_HEIGHT 		320 
#define LCD_SCREEN_WIDTH 		480

// Other define
#define SCREEN_VERTICAL_1		0
#define SCREEN_HORIZONTAL_1		1
#define SCREEN_VERTICAL_2		2
#define SCREEN_HORIZONTAL_2		3

#define TSTATE_STOPPED 	0
#define TSTATE_STOPPING 1
#define TSTATE_RUNNING  2

#define LCD_LOCK(_sc) 				mtx_lock( &(_sc)->mtx )
#define LCD_UNLOCK(_sc) 			mtx_unlock( &(_sc)->mtx )
#define LCD_LOCK_DESTROY(_sc) 		mtx_destroy( &(_sc)->mtx )
#define LCD_ASSERT_LOCKED(_sc)  	mtx_assert( &(_sc)->mtx, MA_OWNED )
#define LCD_ASSERT_UNLOCKED(_sc)	mtx_assert( &(_sc)->mtx, MA_NOTOWNED)


#define PIN_SET( pin )		GPIO_PIN_SET( lcd_sc->dev_gpio, pin##_PIN_NUMBER, GPIO_PIN_HIGH)
#define PIN_RESET( pin )	GPIO_PIN_SET( lcd_sc->dev_gpio, pin##_PIN_NUMBER, GPIO_PIN_LOW)


struct lcd_sc_t
{
    device_t devLcd;
    device_t devTouch;
    device_t dev_gpio; 
    struct cdev* cdev_p;
    struct mtx mtx;
    struct proc *p;
};


void LCD_spiSend(uint8_t* txData, uint8_t* rxData, uint32_t dataLen);
void LCD_spiSendByte(unsigned char SPI_Data);
void LCD_writeCommand(uint8_t Command);
void LCD_writeData(uint8_t Data);
void LCD_reset(void);
void LCD_setRotation(uint8_t Rotation);
void LCD_init(void);
void LCD_brightness(uint8_t brightness);
uint16_t setColor( uint8_t r, uint8_t g, uint8_t b);

void LCD_drawPixel(uint16_t X,uint16_t Y,uint16_t colour);
void LCD_fill(uint16_t* buffer, uint16_t color);
void LCD_showBuffer(uint16_t* buffer);
void LCD_bufferClear(uint16_t* buffer);
void LCD_clear(uint16_t* buffer);

static int lcd_write(struct cdev *dev, struct uio *uio, int ioflag);
static int lcd_probe(device_t dev);
static int lcd_attach(device_t dev);
static int lcd_detach(device_t dev);
static int lcd_shutdown(device_t dev);
