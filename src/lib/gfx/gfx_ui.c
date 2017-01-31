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
 * @file	/DiscoTest1/src/lib/gfx/gfx_ui.c/gfx_ui.c
 * @author	robin
 * @date	Jan 30, 2017
 * @brief	[DESCRIPTION]
 */

#include "gfx_ui.h"
#include <string.h>

void GFXUI_Setup(GFXUI_Instance *ui, GFX_Framebuffer *fb)
{
	ui->fb = fb;

	sprintf(ui->window_title, "Welcome");
	ui->window_background = GFX_COLOR(255,255,255,255);
	ui->window_title_bg = GFX_COLOR(255,0,0,0);
	ui->window_title_fg = GFX_COLOR(255,255,255,255);
	ui->window_title_border = GFX_COLOR(255,255,255,255);
	ui->needs_redraw = 1;
}

void GFXUI_SetWindowBackgroundColor(GFXUI_Instance *ui, const GFX_Color color)
{
	ui->window_background = color;
	ui->needs_redraw = 1;
}

void GFXUI_SetWindowTitle(GFXUI_Instance *ui, const char *title)
{
	strncpy(ui->window_title, title, strlen(title));
	ui->needs_redraw = 1;
}

void GFXUI_SetWindowTitleFont(GFXUI_Instance *ui, const GFX_Font font)
{
	ui->window_title_font = font;
	ui->needs_redraw = 1;
}

void GFXUI_SetWindowTitleForeground(GFXUI_Instance *ui, const GFX_Color color)
{
	ui->window_title_fg = color;
	ui->needs_redraw = 1;
}

void GFXUI_SetWindowTitleBackground(GFXUI_Instance *ui, const GFX_Color color)
{
	ui->window_title_bg = color;
	ui->needs_redraw = 1;
}

void GFXUI_SetWindowTitleBorderColor(GFXUI_Instance *ui, const GFX_Color color)
{
	ui->window_title_border = color;
	ui->needs_redraw = 1;
}

void GFXUI_Draw(GFXUI_Instance *ui)
{
	if (ui->needs_redraw) {
		GFX_Clear(ui->fb, ui->window_background);

	}
}
void GFXUI_CopyFramebuffer(uint32_t *dest)
{

}

