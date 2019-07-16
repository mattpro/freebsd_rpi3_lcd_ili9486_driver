#ifndef __MYFONT
#define __MYFONT

typedef struct
{
    const unsigned char *data;
    uint16_t width;
    uint16_t height;
    uint8_t  dataSize;
} tImage;

typedef struct
{
    long int code;
    const tImage *image;
} tChar;

typedef struct
{
    int length;
    const tChar *chars;
} tFont;

int32_t FONT_TextWidth(const char *str, const tFont *font);
void FONT_DrawString(uint8_t *buffer, const char *str, uint32_t x, uint32_t y, const tFont *font);
void FONT_DrawStringHCenter(uint8_t *buffer, const char *str, uint32_t x, uint32_t y, const tFont *font);

#endif
