// ui_events.h
#ifndef _UI_EVENTS_H
#define _UI_EVENTS_H

#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UI_EVENT_SD_INSERTED,
    UI_EVENT_SD_REMOVED,
    UI_EVENT_FAULT,
    UI_EVENT_TEST,
} UIEventType_t;

typedef struct {
    UIEventType_t type;
    union {
        struct { char message[48]; } fault;
        struct { char message[48]; } test;
    };
} UIEvent_t;

extern QueueHandle_t g_ui_event_queue;

void UIEvents_Init(void);
void UI_HandleEvent(const UIEvent_t *evt);
bool UIEvents_Post(const UIEvent_t *evt);
void UIEvents_PostSD(bool inserted);
void UIEvents_PostFault(const char *message);
void UIEvents_PostTest(const char *message);

#ifdef __cplusplus
}
#endif

#endif // _UI_EVENTS_H
