#ifndef __KEYBOARD_H_
#define __KEYBOARD_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"



#define KEYBOARD_EVENT_UP       0x0A
#define KEYBOARD_EVENT_REPEAT   0x0B
#define KEYBOARD_EVENT_DOWN     0x0C

typedef struct
{
    uint8_t keyCode;
    uint8_t event;
}keyEvent;

extern TaskHandle_t xHandleTaskKeyboard;

void vTaskKeyboard(void *pvParameters);

#endif
