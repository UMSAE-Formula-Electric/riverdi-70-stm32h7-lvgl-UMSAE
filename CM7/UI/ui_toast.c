/*
 * ui_toast.c
 *
 *  Created on: May 23, 2026
 *      Author: mason
 */
#include "ui_toast.h"
#include "lvgl/lvgl.h"

/*
 * Animation completion callback — deletes the toast object once the
 * fade-out animation has finished, freeing all associated LVGL resources.
 */
#define TOAST_ANIM_MS 300


/*
 * Animation completion callback — deletes the toast object once the
 * fade-out animation has finished, freeing all associated LVGL resources.
 */
static void toast_anim_ready_cb(lv_anim_t *a) {
    lv_obj_t *toast = (lv_obj_t *)lv_anim_get_user_data(a);
    lv_obj_delete(toast);
}

/*
 * Animation executor callback — updates the opacity of the toast object
 * on each animation tick. Called repeatedly by the LVGL animation engine
 * with interpolated values between LV_OPA_COVER and LV_OPA_TRANSP.
 */
static void set_opa_cb(void *obj, int32_t v) {
    lv_obj_set_style_opa((lv_obj_t *)obj, (lv_opa_t)v, LV_PART_MAIN);
}

/*
 * UIToast_Show — creates and displays a temporary toast notification.
 *
 * A styled container is placed in the top-right corner of the active screen
 * and populated with the provided message. After `duration_ms` milliseconds,
 * a fade-out animation plays over TOAST_ANIM_MS ms, then the object is deleted.
 *
 * @param message     Null-terminated string to display inside the toast.
 * @param duration_ms How long (ms) the toast remains fully visible before fading.
 */
void UIToast_Show(const char *message, uint32_t duration_ms) {

    lv_obj_t *screen = lv_screen_active();

    /* --- Toast container --- */
    lv_obj_t *toast = lv_obj_create(screen);
    lv_obj_set_size(toast, LV_SIZE_CONTENT, LV_SIZE_CONTENT);   /* Shrink-wrap contents */
    lv_obj_align(toast, LV_ALIGN_TOP_RIGHT, -5, 10);            /* 5px left, 10px down from top-right */
    lv_obj_set_style_bg_color(toast, lv_color_hex(0x222222), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(toast, LV_OPA_90, LV_PART_MAIN);    /* Slight transparency */
    lv_obj_set_style_radius(toast, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(toast, 12, LV_PART_MAIN);
    lv_obj_set_style_border_width(toast, 0, LV_PART_MAIN);      /* No visible border */
    /* Prevent the container from intercepting scroll or click events */
    lv_obj_clear_flag(toast, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    /* --- Message label --- */
    lv_obj_t *label = lv_label_create(toast);
    lv_label_set_text(label, message);
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_18, LV_PART_MAIN);

    /* --- Fade-out animation --- */
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, toast);                                  /* Object to animate */
    lv_anim_set_user_data(&a, toast);                            /* Passed to ready callback for deletion */
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)set_opa_cb);    /* Per-tick opacity setter */
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);        /* Animate from fully opaque to fully transparent */
    lv_anim_set_duration(&a, TOAST_ANIM_MS);                     /* Fade lasts TOAST_ANIM_MS ms */
    lv_anim_set_delay(&a, duration_ms);                          /* Begin fade after the display duration */
    lv_anim_set_ready_cb(&a, toast_anim_ready_cb);               /* Delete toast once fade completes */

    lv_anim_start(&a);
}
