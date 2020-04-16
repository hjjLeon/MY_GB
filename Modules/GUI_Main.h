#ifndef __GUI_MAIN_H_
#define __GUI_MAIN_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t xHandleTaskGuiMain;

void GuiHalInit(void);
void vTaskGuiMain(void *pvParameters);

#endif