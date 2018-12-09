// Hardware connections
#define LED_PIN_NUMBER		26

#define LCD_RS_PIN_NUMBER	24
#define LCD_RST_PIN_NUMBER	25


struct lcd_sc_t
{
    device_t dev;
    device_t dev_gpio;
    struct mtx mtx;
};


#define PIN_SET( pin )		GPIO_PIN_SET( lcd_sc->dev, pin##_PIN_NUMBER, GPIO_PIN_HIGH)
#define PIN_RESET( pin )	GPIO_PIN_SET( lcd_sc->dev, pin##_PIN_NUMBER, GPIO_PIN_LOW)


void lcd_send(uint8_t byte);
void lcd_init(void);
void lcd_reset( void );


static int lcd_write(struct cdev *dev, struct uio *uio, int ioflag);

static int lcd_probe(device_t dev);
static int lcd_attach(device_t dev);
static int lcd_detach(device_t dev);
static int lcd_shutdown(device_t dev);
