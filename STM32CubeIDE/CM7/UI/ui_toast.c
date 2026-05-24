/*
 * ui_toast.c
 *
 *  Created on: May 23, 2026
 *      Author: mason
 */
#include "ui_toast.h"
#include "lvgl/lvgl.h"


#define TOAST_ANIM_MS 300

static void toast_anim_ready_cb(lv_anim_t *a) {
    lv_obj_t *toast = (lv_obj_t *)lv_anim_get_user_data(a);
    lv_obj_delete(toast);
}

void UIToast_Show(const char *message, uint32_t duration_ms) {

    lv_obj_t *screen = lv_screen_active();

    // Container — bottom center of screen
    lv_obj_t *toast = lv_obj_create(screen);
    lv_obj_set_size(toast, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_align(toast, LV_ALIGN_TOP_RIGHT, -5, 10);
    lv_obj_set_style_bg_color(toast, lv_color_hex(0x222222), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(toast, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_style_radius(toast, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(toast, 12, LV_PART_MAIN);
    lv_obj_set_style_border_width(toast, 0, LV_PART_MAIN);
    lv_obj_clear_flag(toast, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *label = lv_label_create(toast);
    lv_label_set_text(label, message);
    lv_obj_set_style_text_color(label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_18, LV_PART_MAIN);

    // Fade out after duration_ms
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, toast);
    lv_anim_set_user_data(&a, toast);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_style_opa);
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_duration(&a, TOAST_ANIM_MS);
    lv_anim_set_delay(&a, duration_ms);
    lv_anim_set_ready_cb(&a, toast_anim_ready_cb);
    lv_anim_start(&a);
}
