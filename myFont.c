#include <string.h>
#include "myFont.h"


static void Font_SetPixel(uint8_t *buffer, uint32_t x, uint32_t y, uint8_t *color)
{
	buffer[ (x + ( 480 * y ) ) * 2 ]     = *( color  + 1 );
	buffer[ (x + ( 480 * y ) ) * 2 + 1 ] = *( color ); 
}


static void Font_DrawBitmapFont(uint8_t *buffer, int x, int y, const tImage *image)
{
    uint32_t x0, y0;
    uint8_t *pdata;
	
		pdata = ( uint8_t *) image->data;
    // rows
    for (y0 = 0; y0 < image->height; y0++)
    {
        // columns
        for (x0 = 0; x0 < image->width; x0++)
        {
          // set pixel
			Font_SetPixel(buffer, x + x0, y + y0, pdata );		
			pdata = pdata + 2; // bo 2 bajty na kolor	
        }
    }
}


static const tChar *Font_FindCharByCode(uint32_t code, const tFont *font)
{
    int32_t count = font->length;
    uint32_t first = 0;
    uint32_t last = count - 1;
    uint32_t mid = 0;

    if (count > 0)
    {
        if ((code >= font->chars[0].code) && (code <= font->chars[count - 1].code))
        {
            while (last >= first)
            {
                mid = first + ((last - first) / 2);

                if (font->chars[mid].code < code)
				{
                    first = mid + 1;
				}
                else
				{
                    if (font->chars[mid].code > code)
					{
                        last = mid - 1;
					}
                    else
					{
                        break;
					}
				}
            }

            if (font->chars[mid].code == code)
			{
                return (&font->chars[mid]);
			}
        }
    }
    return (0);
}


static int Font_Utf8NextChar(const char *str, int32_t start, uint32_t *resultCode, int32_t *nextIndex)
{
    uint32_t len = 0;
    uint32_t index = 0;
    *resultCode = 0;

    while (*(str + index) != 0)
    {
        len++;
        index++;
    }

    unsigned char c;
    uint32_t code = 0;
    uint32_t result = 0;
    uint32_t skip = 0;

    *resultCode = 0;
    *nextIndex = -1;

    if (start >= 0 && start < len)
    {
        index = start;

        while (index < len)
        {
            c = *(str + index);
            index++;

            // msb
            if (skip == 0)
            {
                // if range 0x00010000-0x001fffff
                if ((c & 0xf8) == 0xf0)
                {
                    skip = 3;
                    code = c;
                }
                // if range 0x00000800-0x0000ffff
                else if ((c & 0xf0) == 0xe0)
                {
                    skip = 2;
                    code = c;
                }
                // if range 0x00000080-0x000007ff
                else if ((c & 0xe0) == 0xc0)
                {
                    skip = 1;
                    code = c;
                }
                // if range 0x00-0x7f
                else //if ((c & 0x80) == 0x00)
                {
                    skip = 0;
                    code = c;
                }
            }
            else // not msb
            {
                code = code << 8;
                code |= c;
                skip--;
            }
            if (skip == 0)
            {
                // completed
                *resultCode = code;
                *nextIndex = index;
                result = 1;
                break;
            }
        }
    }
    return (result);
}

int32_t FONT_TextWidth(const char *str, const tFont *font)
{
	int32_t len = strlen(str);
	int32_t index = 0;
	uint32_t code = 0;
	int32_t nextIndex;
	int32_t totalWidth = 0;
	
	while (index < len)
	{
		if (Font_Utf8NextChar(str, index, &code, &nextIndex) != 0)
		{
			const tChar *ch = Font_FindCharByCode(code, font);
			if (ch != 0)
			{
				totalWidth += ch->image->width + 1;
			}
		}
		index = nextIndex;
		if (nextIndex < 0)
		{
			break;
		}
	}
	return totalWidth;
}

void FONT_DrawStringHCenter(uint8_t *buffer, const char *str, uint32_t x, uint32_t y, const tFont *font)
{
	int32_t x0 = x - FONT_TextWidth(str,font)/2;
	FONT_DrawString(buffer,str,x0,y,font);
}

void FONT_DrawString(uint8_t *buffer, const char *str, uint32_t x, uint32_t y, const tFont *font)
{
    int32_t len = strlen(str);
    int32_t index = 0;
    uint32_t code = 0;
    uint32_t x1 = x;
    int32_t nextIndex;
	
    while (index < len)
    {
        if (Font_Utf8NextChar(str, index, &code, &nextIndex) != 0)
        {
            const tChar *ch = Font_FindCharByCode(code, font);
            if (ch != 0)
            {
                Font_DrawBitmapFont(buffer, x1, y, ch->image);
                x1 += ch->image->width + 1; 
            }
        }
        index = nextIndex;
        if (nextIndex < 0)
		{
            break;
		}
    }
}
