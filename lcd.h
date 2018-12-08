// Hardware connections
#define LED_PIN_NUMBER	 26


// Local disply buffer size
#define MAX_DISPLAY_LEN  90



struct lcd_sc_t
{
    device_t dev;
    device_t dev_gpio;
//    struct cdev* cdev_p;
    struct mtx mtx;
};



void lcd_do_reset(void);
void lcd_send(uint8_t byte);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t *data, int len);
void lcd_render(void);
void lcd_set_pixel(uint8_t x, uint8_t y, uint8_t value);
void lcd_set_cursor(uint8_t x, uint8_t y);
void lcd_clear(void);
void lcd_write_char(char code, uint8_t scale);
void lcd_write_string(const char *str, uint8_t scale);
void lcd_init(void);

static int lcd_write(struct cdev *dev, struct uio *uio, int ioflag);

static int lcd_probe(device_t dev);
static int lcd_attach(device_t dev);
static int lcd_detach(device_t dev);
static int lcd_shutdown(device_t dev);
