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
 * @file	/DiscoTest1/src/lib/commandparser.h/commandparser.h
 * @author	robin
 * @date	Jan 26, 2017
 * @brief	[DESCRIPTION]
 */

#include "lib/commandparser.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "Trace.h"

uint8_t CommandParser_Buffer[256];
uint8_t CommandParser_CommandBuffer[256];
int     CommandParser_BufferIndex = 0;
int		CommandParser_CommandBufferLength = 0;
TaskHandle_t CommandParser_TaskToNotify = NULL;

void CommandParser_Init(TaskHandle_t notify_task)
{
	memset(CommandParser_Buffer, 0, 256);
	CommandParser_TaskToNotify = notify_task;
}

/**
 * Called from ISR to deliver new data to the parser
 */
void CommandParser_RxData(uint8_t *data, int length)
{
	int i = 0;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	for (; i < length; i++) {
		if (CommandParser_BufferIndex == 256) {
			CommandParser_BufferIndex = 0;
		}

		if ((data[i] == '\n' || data[i] == '\r') && CommandParser_BufferIndex > 0) {
			//
			// copy the line to the command buffer
			//
			memcpy(CommandParser_CommandBuffer, CommandParser_Buffer, CommandParser_BufferIndex);
			CommandParser_CommandBufferLength = CommandParser_BufferIndex;
			CommandParser_BufferIndex = 0;
			vTaskNotifyGiveFromISR(CommandParser_TaskToNotify, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else {
			CommandParser_Buffer[CommandParser_BufferIndex++] = data[i];
		}
	}
}

int CommandParser_Available()
{
	return CommandParser_CommandBufferLength;
}

int CommandParser_GetCommand(char *buf)
{
	int copied = CommandParser_CommandBufferLength;

	if (copied > 0) {
		memcpy(buf, CommandParser_CommandBuffer, CommandParser_CommandBufferLength);
		CommandParser_CommandBufferLength = 0;
		return copied;
	}

	return 0;
}
