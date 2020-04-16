#ifndef __KEYBOARD_H_
#define __KEYBOARD_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t xHandleTaskKeyboard;

void vTaskKeyboard(void *pvParameters);

#endif
