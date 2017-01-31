/**
 * DiscoTest1
 * ----------------------------------------
 *
 * MIT License
 *
 * Copyright (c) 2017 robin
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file	/DiscoTest1/src/lib/gfx/gfx.c/gfx.c
 * @author	robin
 * @date	Jan 29, 2017
 * @brief	[DESCRIPTION]
 */

#include <math.h>
#include <string.h>
#include "lib/gfx/gfx.h"

#define sign(x) ((x > 0)? 1 : ((x < 0)? -1: 0))

GFX_Point GFX_POINT(int x, int y) {
	GFX_Point p = { .x = x, .y = y };
	return p;
}

GFX_Color GFX_COLOR(unsigned char alpha, unsigned char red, unsigned char green, unsigned char blue) {
	GFX_Color c;
	c.argb.alpha = alpha;
	c.argb.red = red;
	c.argb.green = green;
	c.argb.blue = blue;
	return c;
}

GFX_Font  GFX_FONT(char *font_name)
{
	const struct mf_font_s *font = mf_find_font(font_name);
	if (!font) {
		// what to do here?
	}
	GFX_Font f = { .font = font };
	return f;
}

GFX_Rect GFX_RECT(int x, int y, int width, int height)
{
	GFX_Rect rect;
	rect.x = x;
	rect.y = y;
	rect.width = width;
	rect.height = height;
	return rect;
}

void GFX_Clear(GFX_Framebuffer *fb, GFX_Color color)
{
	unsigned int i, j, pos;
	for (i = 0; i < fb->height; i++) {
		for (j = 0; j < fb->width; j++) {
			pos = (i * fb->width) + j;
			fb->fbptr[pos] = color.uint;
		}
	}
}

void _GFX_FontPixelCallback(int16_t x, int16_t y, uint8_t count, uint8_t alpha, void *state)
{
	GFX_Framebuffer *fb = (GFX_Framebuffer*)state;
	GFX_Color color;
	color.uint = fb->font_context.color.uint;
	color.argb.alpha = alpha;

	uint8_t i = 0;
	for (; i < count; i++) {
		GFX_DrawPixel(fb, GFX_POINT(x+i, y), color);
	}
}

#define RED_OF(c)			(((c) & 0x00FF0000) >> 16 )
#define GREEN_OF(c)			(((c) & 0x0000FF00) >> 8 )
#define BLUE_OF(c)			(((c) & 0x000000FF)      )

GFX_Color GFX_BlendColors(GFX_Color colora, GFX_Color colorb) {
	GFX_Color ret;

	uint32_t rb = colorb.uint & 0x00FF00FF;
	uint32_t g  = colorb.uint & 0x0000FF00;

	rb += ((colora.uint & 0x00FF00FF) - rb) * colora.argb.alpha >> 8;
	g  += ((colora.uint & 0x0000FF00) -  g) * colora.argb.alpha >> 8;
	ret.uint = 0xFF000000 | (rb & 0x00FF00FF) | (g & 0x0000FF00);
	return ret;

//	uint16_t fg_ratio = colora.argb.alpha + 1;
//	uint16_t bg_ratio = 256 - colora.argb.alpha;
//	uint16_t r, g, b;
//
//	r = colora.argb.red * fg_ratio;
//	g = colora.argb.green * fg_ratio;
//	b = colora.argb.blue * fg_ratio;
//
//	r += colorb.argb.red * bg_ratio;
//	g += colorb.argb.green * bg_ratio;
//	b += colorb.argb.blue * bg_ratio;
//
//	r >>= 8;
//	g >>= 8;
//	b >>= 8;
//
//	return GFX_COLOR(255, r, g, b);
}

void GFX_DrawPixel(GFX_Framebuffer *fb, GFX_Point point, GFX_Color color)
{
	if (point.x < 0 || point.x >= fb->width) {
		return;
	}
	if (point.y < 0 || point.y >= fb->height) {
		return;
	}

	if (point.x < fb->width && point.y < fb->height) {
		unsigned int pos = (point.y * fb->width) + point.x;

		if (color.argb.alpha == 255) {
			fb->fbptr[pos] = color.uint;
			return;
		}

		GFX_Color color_bg;
		color_bg.uint = fb->fbptr[pos];
		GFX_Color color_px = GFX_BlendColors(color, color_bg);

		fb->fbptr[pos] = color_px.uint;
	}
}

void GFX_DrawText(GFX_Framebuffer *fb, GFX_Point pos, const char *string, GFX_Color color, GFX_Font font)
{
	fb->font_context.x = pos.x;
	fb->font_context.y = pos.y;
	fb->font_context.color.uint = color.uint;

	int w = 0;
	int i = 0;
	for (; i < strlen(string); i++) {
		w = mf_render_character(font.font,
				fb->font_context.x,
				pos.y,
				string[i],
				(mf_pixel_callback_t)_GFX_FontPixelCallback,
				(void*)fb);
		fb->font_context.x += w;
	}
}

void GFX_DrawFilledRect(GFX_Framebuffer *fb, GFX_Rect rect, GFX_Color color)
{
	int x1 = rect.x;
	int x2 = rect.x + rect.width;
	int y1 = rect.y;
	int y2 = rect.y + rect.height;

	for (; x1 < x2; x1++) {
		y1 = rect.y;
		for (; y1 < y2; y1++) {
			GFX_DrawPixel(fb, GFX_POINT(x1, y1), color);
		}
	}
}

void GFX_DrawLine(GFX_Framebuffer *fb, GFX_Point p1, GFX_Point p2, GFX_Color color)
{
	int i = 0;

	int x = p1.x;
	int y = p1.y;

	int delta_x = abs(p2.x - p1.x);
	int delta_y = abs(p2.y - p1.y);

	int s1 = sign(p2.x - p1.x);
	int s2 = sign(p2.y - p1.y);
	int swap = 0;

	if (delta_y > delta_x) {
		int temp = delta_x;
		delta_x = delta_y;
		delta_y = temp;
		swap = 1;
	}

	int D = 2*delta_y - delta_x;

	for (i = 0; i < delta_x; i++) {
		GFX_DrawPixel(fb, GFX_POINT(x,y), color);
		while (D >= 0) {
			D = D - 2 * delta_x;
			if (swap) {
				x += s1;
			}
			else {
				y += s2;
			}
		}
		D = D + 2 * delta_y;
		if (swap) {
			y += s2;
		}
		else {
			x += s1;
		}
	}
}
