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
 * @file	/DiscoTest1/include/lib/commandparser.h/commandparser.h
 * @author	robin
 * @date	Jan 26, 2017
 * @brief	[DESCRIPTION]
 */
#ifndef INCLUDE_LIB_COMMANDPARSER_H_
#define INCLUDE_LIB_COMMANDPARSER_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

void CommandParser_Init(TaskHandle_t notify_task);
void CommandParser_RxData(uint8_t *data, int length);
int CommandParser_Available();
int CommandParser_GetCommand(char *buf);

#endif /* INCLUDE_LIB_COMMANDPARSER_H_ */
