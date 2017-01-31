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
 * @file	/DiscoTest1/include/lib/gfx/gfx_ui.h/gfx_ui.h
 * @author	robin
 * @date	Jan 30, 2017
 * @brief	[DESCRIPTION]
 */
#ifndef INCLUDE_LIB_GFX_GFX_UI_H_
#define INCLUDE_LIB_GFX_GFX_UI_H_

#include "gfx_config.h"
#include "gfx.h"

typedef struct _GFXUI_Instance {
	GFX_Framebuffer *fb;
	uint32_t canvas[GFX_SCREEN_WIDTH * GFX_SCREEN_HEIGHT * 4];
	char window_title[32];
	GFX_Font window_title_font;
	GFX_Color window_title_fg;
	GFX_Color window_title_bg;
	GFX_Color window_title_border;
	GFX_Color window_background;
	uint8_t needs_redraw;
} GFXUI_Instance;

void GFXUI_Setup(GFXUI_Instance *ui, GFX_Framebuffer *fb);
void GFXUI_SetWindowBackgroundColor(GFXUI_Instance *ui, const GFX_Color color);
void GFXUI_SetWindowTitle(GFXUI_Instance *ui, const char *title);
void GFXUI_SetWindowTitleFont(GFXUI_Instance *ui, const GFX_Font font);
void GFXUI_SetWindowTitleForeground(GFXUI_Instance *ui, const GFX_Color color);
void GFXUI_SetWindowTitleBackground(GFXUI_Instance *ui, const GFX_Color color);
void GFXUI_SetWindowTitleBorderColor(GFXUI_Instance *ui, const GFX_Color color);
void GFXUI_Draw(GFXUI_Instance *ui);
void GFXUI_CopyFramebuffer(uint32_t *dest);

#endif /* INCLUDE_LIB_GFX_GFX_UI_H_ */
