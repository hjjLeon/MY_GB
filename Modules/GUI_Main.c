#include "GUI_Main.h"
#include "oled.h"
#include "lvgl.h"
#include "FreeRTOS.h"
#include "task.h"


TaskHandle_t xHandleTaskGuiMain = NULL;

void my_disp_flush(struct _disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

void GuiHalInit(void)
{
    static lv_disp_buf_t disp_buf;
    static lv_color_t buf[LV_HOR_RES_MAX * 10];                     /*Declare a buffer for 10 lines*/
    lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);    /*Initialize the display buffer*/

    static lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
    disp_drv.flush_cb = my_disp_flush;    /*Set your driver function*/
    disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
    lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/
}

void vTaskGuiMain(void *pvParameters)
{
    
    lv_init();
    GuiHalInit();

    //build window
    lv_obj_t * btn = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
    lv_obj_set_size(btn, 100, 50);                          /*Set its size*/
    lv_obj_t * label = lv_label_create(btn, NULL);          /*Add a label to the button*/
    lv_label_set_text(label, "Button");                     /*Set the labels text*/

    while(1)
    {
        lv_task_handler();
        vTaskDelay(10);
    }
}

void my_disp_flush(struct _disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    int32_t x, y;
    // for(y = area->y1; y <= area->y2; y++) {
    //     for(x = area->x1; x <= area->x2; x++) {
    //         set_pixel(x, y, *color_p);  /* Put a pixel to the display.*/
    //         color_p++;
    //     }
    // }
    x = area->x2 - area->x1 + 1;
    y = area->y2 - area->y1 + 1;
	Address_set(area->x1, area->y1, area->x2, area->y2);
    LCD_WR_DATAS((uint16_t*)color_p, x*y);
    vTaskSuspend(xHandleTaskGuiMain);

    lv_disp_flush_ready(disp_drv);         /* Indicate you are ready with the flushing*/
}

