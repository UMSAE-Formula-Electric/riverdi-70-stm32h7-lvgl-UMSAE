/*
 * ui_theme.c
 *
 *  Created on: May 29, 2026
 *      Author: mason
 */


// ui_theme.c
#include "ui_theme.h"

// Forward declare fonts from SquareLine / LVGL font converter
extern const lv_font_t ui_font_Aoto_Gothic_60;
extern const lv_font_t ui_font_Aoto_Gothic_80;
extern const lv_font_t ui_font_Aoto_Gothic_120;

// ─────────────────────────────────────────────────────────────────────────────
// Dark theme
// ─────────────────────────────────────────────────────────────────────────────
const UITheme_t UI_Theme_Dark = {
    .name = "Dark",

    .colors = {
        .bg_primary        = LV_COLOR_MAKE(0x12, 0x12, 0x12),
        .bg_secondary      = LV_COLOR_MAKE(0x1E, 0x1E, 0x1E),
        .bg_accent         = LV_COLOR_MAKE(0x2A, 0x2A, 0x2A),
        .bg_overlay        = LV_COLOR_MAKE(0x22, 0x22, 0x22),

        .text_primary      = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),
        .text_secondary    = LV_COLOR_MAKE(0xAA, 0xAA, 0xAA),
        .text_accent       = LV_COLOR_MAKE(0x00, 0xD4, 0xFF),
        .text_warning      = LV_COLOR_MAKE(0xFF, 0xAA, 0x00),
        .text_danger       = LV_COLOR_MAKE(0xFF, 0x3C, 0x3C),
        .text_success      = LV_COLOR_MAKE(0x00, 0xFF, 0x88),
        .text_on_overlay   = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),

        .border_primary    = LV_COLOR_MAKE(0x33, 0x33, 0x33),
        .border_secondary  = LV_COLOR_MAKE(0x22, 0x22, 0x22),

        .bar_background    = LV_COLOR_MAKE(0x33, 0x33, 0x33),
        .bar_indicator     = LV_COLOR_MAKE(0x00, 0xD4, 0xFF),
        .bar_warning       = LV_COLOR_MAKE(0xFF, 0xAA, 0x00),
        .bar_danger        = LV_COLOR_MAKE(0xFF, 0x3C, 0x3C),
    },

    .fonts = {
        .xs          = &lv_font_montserrat_14,
        .sm          = &lv_font_montserrat_18,
        .md          = &lv_font_montserrat_26,
        .lg          = &lv_font_montserrat_40,
        .value_sm    = &lv_font_montserrat_26,
        .value_md    = &lv_font_montserrat_30,
        .value_lg    = &lv_font_montserrat_48,
    },

    .geometry = {
        .radius_sm        = 4,
        .radius_md        = 8,
        .radius_lg        = 12,

        .pad_xs           = 4,
        .pad_sm           = 8,
        .pad_md           = 12,
        .pad_lg           = 20,

        .gap_row          = 10,
        .gap_col          = 70,

        .border_width     = 1,

        .toast_min_width  = 200,
        .toast_pad        = 12,
        .toast_radius     = 8,
        .toast_y_offset   = 20,
    },

    .opacity = {
        .widget_bg   = LV_OPA_90,
        .overlay_bg  = LV_OPA_90,
        .disabled    = LV_OPA_40,
    },

    .animation = {
        .toast_fade_ms        = 300,
        .screen_transition_ms = 200,
    },

	.states = {
			// In UI_Theme_Dark colors initializer
			.state_idle      = LV_COLOR_MAKE(0x88, 0x88, 0x88),
			.state_tractive  = LV_COLOR_MAKE(0x00, 0xE5, 0x76),
			.state_ready     = LV_COLOR_MAKE(0x00, 0xD4, 0xFF),
			.state_error     = LV_COLOR_MAKE(0xFF, 0x3C, 0x3C),
			.state_charging  = LV_COLOR_MAKE(0xFF, 0xCC, 0x00),
			.state_unknown   = LV_COLOR_MAKE(0xFF, 0x3C, 0x3C),
	},
};

// ─────────────────────────────────────────────────────────────────────────────
// Light theme — same structure, different values
// ─────────────────────────────────────────────────────────────────────────────
const UITheme_t UI_Theme_Light = {
    .name = "Light",

    .colors = {
        .bg_primary        = LV_COLOR_MAKE(0xF0, 0xF0, 0xF0),
        .bg_secondary      = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),
        .bg_accent         = LV_COLOR_MAKE(0xE0, 0xE0, 0xE0),
        .bg_overlay        = LV_COLOR_MAKE(0x33, 0x33, 0x33),

        .text_primary      = LV_COLOR_MAKE(0x11, 0x11, 0x11),
        .text_secondary    = LV_COLOR_MAKE(0x55, 0x55, 0x55),
        .text_accent       = LV_COLOR_MAKE(0x00, 0x7A, 0xCC),
        .text_warning      = LV_COLOR_MAKE(0xCC, 0x77, 0x00),
        .text_danger       = LV_COLOR_MAKE(0xCC, 0x22, 0x22),
        .text_success      = LV_COLOR_MAKE(0x00, 0x99, 0x44),
        .text_on_overlay   = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),

        .border_primary    = LV_COLOR_MAKE(0xCC, 0xCC, 0xCC),
        .border_secondary  = LV_COLOR_MAKE(0xDD, 0xDD, 0xDD),

        .bar_background    = LV_COLOR_MAKE(0xCC, 0xCC, 0xCC),
        .bar_indicator     = LV_COLOR_MAKE(0x00, 0x7A, 0xCC),
        .bar_warning       = LV_COLOR_MAKE(0xCC, 0x77, 0x00),
        .bar_danger        = LV_COLOR_MAKE(0xCC, 0x22, 0x22),
    },

    .fonts = {
        .xs          = &lv_font_montserrat_14,
        .sm          = &lv_font_montserrat_18,
        .md          = &lv_font_montserrat_26,
        .lg          = &lv_font_montserrat_40,
        .value_sm    = &lv_font_montserrat_26,
        .value_md    = &lv_font_montserrat_30,
        .value_lg    = &lv_font_montserrat_48,
    },

    .geometry = {
        .radius_sm        = 4,
        .radius_md        = 8,
        .radius_lg        = 12,

        .pad_xs           = 4,
        .pad_sm           = 8,
        .pad_md           = 12,
        .pad_lg           = 20,

        .gap_row          = 10,
        .gap_col          = 70,

        .border_width     = 1,

        .toast_min_width  = 200,
        .toast_pad        = 12,
        .toast_radius     = 8,
        .toast_y_offset   = 20,
    },

    .opacity = {
        .widget_bg   = LV_OPA_COVER,
        .overlay_bg  = LV_OPA_90,
        .disabled    = LV_OPA_40,
    },

    .animation = {
        .toast_fade_ms        = 300,
        .screen_transition_ms = 200,
    },

	.states = {
			// In UI_Theme_Dark colors initializer
			.state_idle      = LV_COLOR_MAKE(0x88, 0x88, 0x88),
			.state_tractive  = LV_COLOR_MAKE(0x00, 0xE5, 0x76),
			.state_ready     = LV_COLOR_MAKE(0x00, 0xD4, 0xFF),
			.state_error     = LV_COLOR_MAKE(0xFF, 0x3C, 0x3C),
			.state_charging  = LV_COLOR_MAKE(0xFF, 0xCC, 0x00),
			.state_unknown   = LV_COLOR_MAKE(0xFF, 0x3C, 0x3C),
	},
};

// ─────────────────────────────────────────────────────────────────────────────
// High contrast — night race / low visibility conditions
// ─────────────────────────────────────────────────────────────────────────────
const UITheme_t UI_Theme_HighContrast = {
    .name = "High Contrast",

    .colors = {
        .bg_primary        = LV_COLOR_MAKE(0x00, 0x00, 0x00),
        .bg_secondary      = LV_COLOR_MAKE(0x00, 0x00, 0x00),
        .bg_accent         = LV_COLOR_MAKE(0x1A, 0x1A, 0x1A),
        .bg_overlay        = LV_COLOR_MAKE(0x00, 0x00, 0x00),

        .text_primary      = LV_COLOR_MAKE(0xFF, 0xFF, 0x00),
        .text_secondary    = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF),
        .text_accent       = LV_COLOR_MAKE(0x00, 0xFF, 0x88),
        .text_warning      = LV_COLOR_MAKE(0xFF, 0x88, 0x00),
        .text_danger       = LV_COLOR_MAKE(0xFF, 0x00, 0x00),
        .text_success      = LV_COLOR_MAKE(0x00, 0xFF, 0x00),
        .text_on_overlay   = LV_COLOR_MAKE(0xFF, 0xFF, 0x00),

        .border_primary    = LV_COLOR_MAKE(0xFF, 0xFF, 0x00),
        .border_secondary  = LV_COLOR_MAKE(0xAA, 0xAA, 0x00),

        .bar_background    = LV_COLOR_MAKE(0x33, 0x33, 0x00),
        .bar_indicator     = LV_COLOR_MAKE(0x00, 0xFF, 0x88),
        .bar_warning       = LV_COLOR_MAKE(0xFF, 0x88, 0x00),
        .bar_danger        = LV_COLOR_MAKE(0xFF, 0x00, 0x00),
    },

    .fonts = {
        .xs          = &lv_font_montserrat_14,
        .sm          = &lv_font_montserrat_18,
        .md          = &lv_font_montserrat_26,
        .lg          = &lv_font_montserrat_40,
        .value_sm    = &lv_font_montserrat_26,
        .value_md    = &lv_font_montserrat_30,
        .value_lg    = &lv_font_montserrat_48,
    },

    .geometry = {
        .radius_sm        = 2,
        .radius_md        = 4,
        .radius_lg        = 8,

        .pad_xs           = 4,
        .pad_sm           = 8,
        .pad_md           = 14,
        .pad_lg           = 20,

        .gap_row          = 10,
        .gap_col          = 70,

        .border_width     = 2,

        .toast_min_width  = 200,
        .toast_pad        = 14,
        .toast_radius     = 4,
        .toast_y_offset   = 20,
    },

    .opacity = {
        .widget_bg   = LV_OPA_COVER,
        .overlay_bg  = LV_OPA_COVER,
        .disabled    = LV_OPA_50,
    },

    .animation = {
        .toast_fade_ms        = 200,
        .screen_transition_ms = 150,
    },

	.states = {
			// In UI_Theme_Dark colors initializer
			.state_idle      = LV_COLOR_MAKE(0x88, 0x88, 0x88),
			.state_tractive  = LV_COLOR_MAKE(0x00, 0xE5, 0x76),
			.state_ready     = LV_COLOR_MAKE(0x00, 0xD4, 0xFF),
			.state_error     = LV_COLOR_MAKE(0xFF, 0x3C, 0x3C),
			.state_charging  = LV_COLOR_MAKE(0xFF, 0xCC, 0x00),
			.state_unknown   = LV_COLOR_MAKE(0xFF, 0x3C, 0x3C),
	},
};

// ─────────────────────────────────────────────────────────────────────────────
// Registry
// ─────────────────────────────────────────────────────────────────────────────
const UITheme_t *UI_Themes[] = {
    &UI_Theme_Dark,
    &UI_Theme_Light,
    &UI_Theme_HighContrast,
};
const uint8_t UI_Theme_Count = 3;

static const UITheme_t *g_active = &UI_Theme_Dark;
static uint8_t          g_active_index = 0;

void UITheme_Init(void) {
    g_active       = &UI_Theme_Dark;
    g_active_index = 0;
}

void UITheme_Set(const UITheme_t *theme) {
    g_active = theme;
    // keep index in sync
    for (uint8_t i = 0; i < UI_Theme_Count; i++) {
        if (UI_Themes[i] == theme) {
            g_active_index = i;
            break;
        }
    }
}

void UITheme_SetByIndex(uint8_t index) {
    if (index < UI_Theme_Count) {
        g_active       = UI_Themes[index];
        g_active_index = index;
    }
}

const UITheme_t *UITheme_Get(void) {
    return g_active;
}

uint8_t UITheme_GetIndex(void) {
    return g_active_index;
}

const char *UITheme_GetName(uint8_t index) {
    if (index < UI_Theme_Count) return UI_Themes[index]->name;
    return "Unknown";
}
