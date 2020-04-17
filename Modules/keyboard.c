#include "keyboard.h"
#include "oled.h"
#include "FreeRTOS.h"
#include "task.h"

/*********************************************************************************************/

#define KEYBOARD_SCAN_PERIOD        5
#define KEYBOARD_DEBOUNCING_TIME    20
#define KEYBOARD_REPEAT_START_TIME  400
#define KEYBOARD_REPEAT_PERIOD      100
#define KEYBOARD_FORCE_POWOFF_TIME  5000

#define KEYBOARD_DEBOUNCING_COUNT       (KEYBOARD_DEBOUNCING_TIME/KEYBOARD_SCAN_PERIOD)
#define KEYBOARD_REPEAT_START_COUNT     (KEYBOARD_REPEAT_START_TIME/KEYBOARD_SCAN_PERIOD - KEYBOARD_DEBOUNCING_COUNT)
#define KEYBOARD_REPEAT_PERIOD_COUNT    (KEYBOARD_REPEAT_PERIOD/KEYBOARD_SCAN_PERIOD)
#define KEYBOARD_FORCE_POWOFF_COUNT     ((KEYBOARD_FORCE_POWOFF_TIME - KEYBOARD_REPEAT_START_TIME)/KEYBOARD_REPEAT_PERIOD)

/**************************************************************************************************/

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t     pin;
    uint32_t     keyCode;
}keyLineDef;

/******************************************************************************************************/

keyLineDef keyLine[10] = {
    {KEY_UP_GPIO_Port,      KEY_UP_Pin,     0,},
    {KEY_DOWN_GPIO_Port,    KEY_DOWN_Pin,   0,},
    {KEY_LEFT_GPIO_Port,    KEY_LEFT_Pin,   0,},
    {KEY_RIGHT_GPIO_Port,   KEY_RIGHT_Pin,  0,},
    {KEY_X_GPIO_Port,       KEY_X_Pin,      0,},
    {KEY_Y_GPIO_Port,       KEY_Y_Pin,      0,},
    {KEY_A_GPIO_Port,       KEY_A_Pin,      0,},
    {KEY_B_GPIO_Port,       KEY_B_Pin,      0,},
    {KEY_SELECT_GPIO_Port,  KEY_SELECT_Pin, 0,},
    {KEY_START_GPIO_Port,   KEY_START_Pin,  0,},
};


TaskHandle_t xHandleTaskKeyboard = NULL;
static int16_t keyPressTimeCount[10] = {0};

void vTaskKeyboard(void *pvParameters)
{
    uint16_t powerOffCount = 0;
    uint16_t count = 0;
    uint8_t i;

    TickType_t xLastWakeTime = 0;

    keyEvent event;

    QueueHandle_t queueKeyboardEvent = xQueueCreate( 20, sizeof(keyEvent));
    if(queueKeyboardEvent == NULL)
    {
        while(1);
    }
    
    vTaskDelayUntil(&xLastWakeTime, 3000);
    HAL_GPIO_WritePin(POWER_SET_GPIO_Port, POWER_SET_Pin, GPIO_PIN_SET);
    OLED_BLK_Set();

    //等待电源键被松开
    while(HAL_GPIO_ReadPin(KEY_START_GPIO_Port, KEY_START_Pin) == GPIO_PIN_RESET)
    {
        vTaskDelay(10);
    }


    //键盘扫描
    while(1)
    {
        vTaskDelayUntil(&xLastWakeTime, KEYBOARD_SCAN_PERIOD);
        for(i = 0; i < 10; i++)
        {
            //读取IO并计数
            if(HAL_GPIO_ReadPin(keyLine[i].port, keyLine[i].pin) == GPIO_PIN_RESET)
            {
                keyPressTimeCount[i]++;
            }
            else
            {
                if(keyPressTimeCount[i] >= KEYBOARD_DEBOUNCING_COUNT)
                   keyPressTimeCount[i] = -1;
            }

            //判断按下时长是否足以触发时间
            if(keyPressTimeCount[i] == KEYBOARD_DEBOUNCING_COUNT)
            {
                //按键按下
                event.event = KEYBOARD_EVENT_DOWN;
            }
            else if(keyPressTimeCount[i] == KEYBOARD_REPEAT_START_COUNT)
            {
                //按键长按
                event.event = KEYBOARD_EVENT_REPEAT;

                keyPressTimeCount[i] -= KEYBOARD_REPEAT_PERIOD_COUNT;//重新计算长按重复触发时间
                if(i == 9)
                {
                    if(powerOffCount < KEYBOARD_FORCE_POWOFF_COUNT)
                    {
                        powerOffCount++;
                    }
                    else
                    {
                        //Power OFF
                        HAL_GPIO_WritePin(POWER_SET_GPIO_Port, POWER_SET_Pin, GPIO_PIN_RESET);
                        OLED_BLK_Clr();
                    }
                    
                }

                //暂不处理REPEAT事件
                continue;
            }
            else if(keyPressTimeCount[i] == -1)
            {
                //按键松开
                event.event = KEYBOARD_EVENT_UP;

                keyPressTimeCount[i] = 0;
                if(i == 9)
                {
                    powerOffCount = 0;
                }
            }
            else
            {
                //未满足触发，跳过发送事件
                continue;
            }

            //满足触发，发送按键事件
            event.keyCode = i;
            xQueueSend(queueKeyboardEvent, &event, 0);
        }
    }
}
