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

/* FreeRTOS queue handle — populated by UIEvents_Init(), used by UIEvents_Post().
 * NULL until initialised; all post functions guard against this. */
QueueHandle_t g_ui_event_queue = NULL;

/* Fixed toast message strings for SD card state changes */
const char *SD_INSERTED = "SD card inserted";
const char *SD_EJECTED  = "SD card removed";

/*
 * UIEvents_Init — creates the UI event queue.
 *
 * Must be called once before any UIEvents_Post* calls, typically at
 * RTOS startup before the scheduler is started. Asserts on allocation
 * failure so misconfigured heap sizes are caught early.
 */
void UIEvents_Init(void) {
    g_ui_event_queue = xQueueCreate(8, sizeof(UIEvent_t));
    configASSERT(g_ui_event_queue);
}


/*
 * UI_HandleEvent — dispatches a single UIEvent to the appropriate UI action.
 *
 * Called from the UI task each time an event is dequeued. Each case maps an
 * event type to a toast notification. UI_EVENT_FAULT and UI_EVENT_TEST carry
 * their own message payload; SD events use the fixed strings above.
 *
 * @param evt  Pointer to the event to handle. Must not be NULL.
 */

void UI_HandleEvent(const UIEvent_t *evt) {
    switch (evt->type) {
        case UI_EVENT_SD_INSERTED:
            UIToast_Show(SD_INSERTED, 3000);
            break;

        case UI_EVENT_SD_REMOVED:
            UIToast_Show(SD_EJECTED, 3000);
            break;

        case UI_EVENT_FAULT:
            /* Message is embedded in the event payload */
            UIToast_Show(evt->fault.message, 3000);
            break;

        case UI_EVENT_TEST:
            /* Message is embedded in the event payload */
            UIToast_Show(evt->test.message, 3000);
            break;

        default:
            break;
    }
}


/*
 * UIEvents_Post — enqueues a UI event for the UI task to process.
 *
 * Non-blocking (timeout = 0): if the queue is full the event is silently
 * dropped and false is returned. Safe to call from any task context
 * that does not require delivery guarantees.
 *
 * @param evt  Pointer to the event to enqueue. Copied by value into the queue.
 * @return     true if the event was accepted, false if the queue was full or
 *             uninitialised.
 */
bool UIEvents_Post(const UIEvent_t *evt) {
    if (g_ui_event_queue == NULL) return false;
    return xQueueSend(g_ui_event_queue, evt, 0) == pdPASS;
}

/*
 * UIEvents_Post_FROM_ISR — ISR-safe variant of UIEvents_Post().
 *
 * Uses xQueueSendFromISR() in place of xQueueSend(), which is not safe
 * to call from an interrupt context. Must be used instead of UIEvents_Post()
 * whenever posting an event from an ISR.
 *
 * Non-blocking: if the queue is full the event is dropped and false is
 * returned. No higher-priority task wakeup is requested (pxHigherPriorityTaskWoken
 * is passed as NULL), so a yield must be triggered manually if needed.
 *
 * @param evt  Pointer to the event to enqueue. Copied by value into the queue.
 * @return     true if the event was accepted, false if the queue was full or
 *             uninitialised.
 */
bool UIEvents_Post_FROM_ISR(const UIEvent_t *evt) {
    if (g_ui_event_queue == NULL) return false;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t result = xQueueSendFromISR(g_ui_event_queue, evt, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return result == pdPASS;
}
/*
 * UIEvents_PostSD — convenience wrapper for SD card insertion/removal events.
 *
 * Builds the appropriate event type and forwards it to UIEvents_Post().
 *
 * @param inserted  true  → UI_EVENT_SD_INSERTED
 *                  false → UI_EVENT_SD_REMOVED
 */

void UIEvents_PostSD(bool inserted) {
    UIEvent_t evt = {
        .type = inserted ? UI_EVENT_SD_INSERTED : UI_EVENT_SD_REMOVED
    };
    UIEvents_Post(&evt);
}


void UIEvents_PostSD(bool inserted) {
    UIEvent_t evt = {
        .type = inserted ? UI_EVENT_SD_INSERTED : UI_EVENT_SD_REMOVED
    };
    UIEvents_Post(&evt);
}


/*
 * UIEvents_PostFault — enqueues a fault notification with a custom message.
 *
 * Asserts that `length` fits within the payload buffer before copying,
 * catching oversized messages at the call site rather than silently truncating.
 *
 * @param message  Null-terminated fault description string.
 * @param length   Length of `message` excluding the null terminator.
 *                 Must be < MESSAGE_LENGTH.
 */
void UIEvents_PostFault(const char *message, size_t length) {
    configASSERT(length < MESSAGE_LENGTH);
    UIEvent_t evt = { .type = UI_EVENT_FAULT };
    snprintf(evt.fault.message, sizeof(evt.fault.message), "%s", message);
    UIEvents_Post(&evt);
}


/*
 * UIEvents_PostTest — enqueues a test notification with a custom message.
 *
 * Mirrors UIEvents_PostFault() but targets the UI_EVENT_TEST path, keeping
 * test-originated toasts distinguishable from genuine fault events.
 *
 * @param message  Null-terminated test description string.
 * @param length   Length of `message` excluding the null terminator.
 *                 Must be < MESSAGE_LENGTH.
 */
void UIEvents_PostTest(const char *message, size_t length) {
    UIEvent_t evt = { .type = UI_EVENT_TEST };
    snprintf(evt.test.message, sizeof(evt.test.message), "%s", message);
    UIEvents_Post(&evt);
}
