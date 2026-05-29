/*
 * UI_task.h
 *
 *  Created on: May 29, 2026
 *      Author: mason
 */

#ifndef UI_UI_TASK_H_
#define UI_UI_TASK_H_

#include "FreeRTOS.h"
#include "task.h"

#ifdef DEBUG
extern TaskHandle_t g_lvgl_task_handle;
#define ASSERT_LVGL_TASK() configASSERT(xTaskGetCurrentTaskHandle() == g_lvgl_task_handle)
#else
#define ASSERT_LVGL_TASK()
#endif

#endif /* UI_UI_TASK_H_ */
