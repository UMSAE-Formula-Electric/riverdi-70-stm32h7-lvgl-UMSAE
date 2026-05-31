/*
 * ui_theme.h
 *
 *  Created on: May 29, 2026
 *      Author: mason
 */

#ifndef _UI_THEME_H
#define _UI_THEME_H

#include "lvgl/lvgl.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Color palette — raw colors defined once, referenced by semantic roles below
// ─────────────────────────────────────────────────────────────────────────────
typedef struct {
    // Backgrounds
    lv_color_t bg_primary;       // main screen background
    lv_color_t bg_secondary;     // widget / card background
    lv_color_t bg_accent;        // highlighted widget background
    lv_color_t bg_overlay;       // toast / modal overlay

    // Text
    lv_color_t text_primary;     // main readable text
    lv_color_t text_secondary;   // labels, subtitles
    lv_color_t text_accent;      // highlighted values
    lv_color_t text_warning;     // warning state
    lv_color_t text_danger;      // fault / critical state
    lv_color_t text_success;     // ok / ready state
    lv_color_t text_on_overlay;  // text drawn on toast/modal bg

    // Borders and dividers
    lv_color_t border_primary;
    lv_color_t border_secondary;

    // Interactive elements
    lv_color_t bar_background;   // unfilled portion of a bar
    lv_color_t bar_indicator;    // filled portion of a bar
    lv_color_t bar_warning;      // bar color when value is in warning range
    lv_color_t bar_danger;       // bar color when value is critical
} UIColorPalette_t;

// ─────────────────────────────────────────────────────────────────────────────
// Typography — all fonts in one place
// ─────────────────────────────────────────────────────────────────────────────
typedef struct {
    const lv_font_t *xs;          // 14px — hints, fine print
    const lv_font_t *sm;          // 18px — units, subtitles
    const lv_font_t *md;          // 26px — widget titles / labels
    const lv_font_t *lg;          // 32–40px — section headers
    const lv_font_t *value_sm;    // 60px  — secondary stat values
    const lv_font_t *value_md;    // 80px  — standard stat values
    const lv_font_t *value_lg;    // 120px — hero value (speed)
} UITypography_t;

// ─────────────────────────────────────────────────────────────────────────────
// Geometry — spacing, sizing, shape
// ─────────────────────────────────────────────────────────────────────────────
typedef struct {
    lv_coord_t radius_sm;         // small widget corner radius
    lv_coord_t radius_md;         // standard widget corner radius
    lv_coord_t radius_lg;         // card / container corner radius

    lv_coord_t pad_xs;            // 4px
    lv_coord_t pad_sm;            // 8px
    lv_coord_t pad_md;            // 12px  — standard widget padding
    lv_coord_t pad_lg;            // 20px  — container padding

    lv_coord_t gap_row;           // vertical gap between stat rows
    lv_coord_t gap_col;           // horizontal gap between stat columns

    lv_coord_t border_width;      // widget border thickness

    lv_coord_t toast_min_width;
    lv_coord_t toast_pad;
    lv_coord_t toast_radius;
    lv_coord_t toast_y_offset;    // distance from bottom edge
} UIGeometry_t;

// ─────────────────────────────────────────────────────────────────────────────
// Opacity
// ─────────────────────────────────────────────────────────────────────────────
typedef struct {
    lv_opa_t widget_bg;           // normal widget background opacity
    lv_opa_t overlay_bg;          // toast/modal background opacity
    lv_opa_t disabled;            // opacity for hidden/inactive elements
} UIOpacity_t;

// ─────────────────────────────────────────────────────────────────────────────
// Animation timing
// ─────────────────────────────────────────────────────────────────────────────
typedef struct {
    uint32_t toast_fade_ms;       // duration of toast fade-out animation
    uint32_t screen_transition_ms;
} UIAnimation_t;

typedef struct {
    // Car State Colors
    lv_color_t state_idle;
    lv_color_t state_tractive;
    lv_color_t state_ready;
    lv_color_t state_error;
    lv_color_t state_charging;
    lv_color_t state_unknown;
} UICarStateColors_t;

// ─────────────────────────────────────────────────────────────────────────────
// Full theme — composes all sections
// ─────────────────────────────────────────────────────────────────────────────
typedef struct {
    const char       *name;
    UIColorPalette_t  colors;
    UITypography_t    fonts;
    UIGeometry_t      geometry;
    UIOpacity_t       opacity;
    UIAnimation_t     animation;
    UICarStateColors_t states;
} UITheme_t;

// ─────────────────────────────────────────────────────────────────────────────
// Built-in themes
// ─────────────────────────────────────────────────────────────────────────────
extern const UITheme_t UI_Theme_Dark;
extern const UITheme_t UI_Theme_Light;
extern const UITheme_t UI_Theme_HighContrast;

extern const UITheme_t *UI_Themes[];
extern const uint8_t    UI_Theme_Count;



// ─────────────────────────────────────────────────────────────────────────────
// API
// ─────────────────────────────────────────────────────────────────────────────
void              UITheme_Init(void);
void              UITheme_Set(const UITheme_t *theme);
void              UITheme_SetByIndex(uint8_t index);
const UITheme_t  *UITheme_Get(void);
uint8_t           UITheme_GetIndex(void);
const char       *UITheme_GetName(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif // _UI_THEME_H

