/*
 * ui_events.c
 *
 *  Created on: May 23, 2026
 *      Author: mason
 */
#include <stdbool.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "ui_events.h"
#include "ui_toast.h"

QueueHandle_t g_ui_event_queue = NULL;

void UIEvents_Init(void) {
    g_ui_event_queue = xQueueCreate(8, sizeof(UIEvent_t));
    configASSERT(g_ui_event_queue);
}

void UI_HandleEvent(const UIEvent_t *evt) {
    switch (evt->type) {
        case UI_EVENT_SD_INSERTED:
            UIToast_Show("SD card inserted", 3000);
            break;

        case UI_EVENT_SD_REMOVED:
            UIToast_Show("SD card removed", 3000);
            break;

        case UI_EVENT_FAULT:
            UIToast_Show(evt->fault.message, 5000);
            break;

        case UI_EVENT_TEST:
            UIToast_Show(evt->test.message, 3000);
            break;

        default:
            break;
    }
}

bool UIEvents_Post(const UIEvent_t *evt) {
    if (g_ui_event_queue == NULL) return false;
    return xQueueSend(g_ui_event_queue, evt, 0) == pdPASS;
}

void UIEvents_PostSD(bool inserted) {
	//STUB implementation
}

void UIEvents_PostFault(const char *message) {
    UIEvent_t evt = { .type = UI_EVENT_FAULT };
    snprintf(evt.fault.message, sizeof(evt.fault.message), "%s", message);
    UIEvents_Post(&evt);
}

void UIEvents_PostTest(const char *message) {
    UIEvent_t evt = { .type = UI_EVENT_TEST };
    snprintf(evt.test.message, sizeof(evt.test.message), "%s", message);
    UIEvents_Post(&evt);
}
