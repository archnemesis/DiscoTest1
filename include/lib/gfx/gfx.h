#ifndef GFX_H
#define GFX_H

#include <stdint.h>
#include "mcufont.h"
#include "mf_font.h"

typedef union _GFX_Color {
	unsigned int uint;
	struct {
		unsigned char blue;
		unsigned char green;
		unsigned char red;
		unsigned char alpha;
	} argb;
} GFX_Color;

typedef struct _GFX_Point {
	int x;
	int y;
} GFX_Point;

typedef struct _GFX_Font {
	const struct mf_font_s *font;
} GFX_Font;

typedef struct _GFX_Rect {
	int x;
	int y;
	int height;
	int width;
} GFX_Rect;

typedef struct _GFX_FontContext {
	int x;
	int y;
	GFX_Color color;
	GFX_Font font;
} GFX_FontContext;

typedef struct _GFX_Framebuffer {
	volatile uint32_t *fbptr;
	uint32_t width;
	uint32_t height;
	GFX_FontContext font_context;
} GFX_Framebuffer;

#define GFX_COLOR_BLACK GFX_COLOR(255,0,0,0);
#define GFX_COLOR_WHITE GFX_COLOR(255,255,255,255);

GFX_Point GFX_POINT(int x, int y);
GFX_Color GFX_COLOR(unsigned char alpha, unsigned char red, unsigned char green, unsigned char blue);
GFX_Font  GFX_FONT(char *font_name);
GFX_Rect GFX_RECT(int x, int y, int width, int height);

void GFX_Clear(GFX_Framebuffer *fb, GFX_Color color);
void GFX_DrawPixel(GFX_Framebuffer *fb, GFX_Point point, GFX_Color color);
void GFX_DrawLine(GFX_Framebuffer *fb, GFX_Point p1, GFX_Point p2, GFX_Color color);
void GFX_DrawText(GFX_Framebuffer *fb, GFX_Point pos, const char *string, GFX_Color color, GFX_Font font);

#endif
