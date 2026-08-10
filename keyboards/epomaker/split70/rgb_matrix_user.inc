// Copyright 2024 Epomaker (@Epomaker)
// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef ENABLE_RGB_MATRIX_RGBR_PLAY
RGB_MATRIX_EFFECT(RGBR_PLAY)
#    ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS

bool RGBR_PLAY(effect_params_t *params) {
    RGB_MATRIX_USE_LIMITS(led_min, led_max);

    extern void rgbrec_play(uint8_t led_min, uint8_t led_max);
    rgbrec_play(led_min, led_max);

    return rgb_matrix_check_finished_leds(led_max);
}

#    endif // RGB_MATRIX_CUSTOM_EFFECT_IMPLS
#endif

// Static light for the strip lights only (LED indices 0-2 and 36-38 in
// keyboard.json's rgb_matrix.layout — these six slots all map to matrix
// [0,0] because they are the accent strip LEDs on each half, not tied to
// any physical key). Every other LED stays off.
RGB_MATRIX_EFFECT(STRIP_STATIC)
#ifdef RGB_MATRIX_CUSTOM_EFFECT_IMPLS

static bool strip_static_is_strip_led(uint8_t index) {
    switch (index) {
        case 0:
        case 1:
        case 2:
        case 36:
        case 37:
        case 38:
            return true;
        default:
            return false;
    }
}

bool STRIP_STATIC(effect_params_t *params) {
    RGB_MATRIX_USE_LIMITS(led_min, led_max);

    RGB rgb = rgb_matrix_hsv_to_rgb(rgb_matrix_config.hsv);

    for (uint8_t i = led_min; i < led_max; i++) {
        if (strip_static_is_strip_led(i)) {
            // Strip LEDs are wired R/G-swapped relative to the rest of the
            // matrix; see the matching compensation in split70.c's
            // strip_render().
            rgb_matrix_set_color(i, rgb.g, rgb.r, rgb.b);
        } else {
            rgb_matrix_set_color(i, 0, 0, 0);
        }
    }

    return rgb_matrix_check_finished_leds(led_max);
}

#endif // RGB_MATRIX_CUSTOM_EFFECT_IMPLS
