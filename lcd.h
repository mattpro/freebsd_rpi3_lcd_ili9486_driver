// Hardware connections
#define LED_PIN_NUMBER	 26



struct lcd_sc_t
{
    device_t dev;
    device_t dev_gpio;
    struct mtx mtx;
};



void lcd_send(uint8_t byte);
void lcd_init(void);

static int lcd_write(struct cdev *dev, struct uio *uio, int ioflag);

static int lcd_probe(device_t dev);
static int lcd_attach(device_t dev);
static int lcd_detach(device_t dev);
static int lcd_shutdown(device_t dev);
