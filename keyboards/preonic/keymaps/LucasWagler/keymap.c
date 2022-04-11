/* Copyright 2015-2021 Jack Humbert
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "muse.h"

enum preonic_layers {
  _QWERTY,
  _COLEMAK,
  _DVORAK,
  _LOWER,
  _RAISE,
  _ADJUST
};

enum preonic_keycodes {
  QWERTY = SAFE_RANGE,
  COLEMAK,
  DVORAK,
  LOWER,
  RAISE,
  BACKLIT
};

// Home Row
#define GUI_A LGUI_T(KC_A)
#define ALT_S LALT_T(KC_S)
#define CTL_D LCTL_T(KC_D)
#define SHFT_F LSFT_T(KC_F)
#define L2_G LT(2,KC_G)
#define L2_H LT(2,KC_H)
#define SHFT_J RSFT_T(KC_J)
#define CTL_K RCTL_T(KC_K)
#define ALT_L LALT_T(KC_L)
#define GUI_SCLN RGUI_T(KC_SCLN)

// Thumbs
#define L2_ESC LT(2,KC_ESC)
#define SHFT_BSP LSFT_T(KC_BSPC)
#define SHFT_SPC RSFT_T(KC_SPC)
#define L1_ENT LT(1,KC_ENT)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
        [0] = LAYOUT_ortho_5x12(KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_BTN2, KC_MS_L, KC_MS_D, KC_MS_U, KC_MS_R, KC_BTN1, KC_Q, KC_W, KC_E, KC_R, KC_T, KC_NO, KC_NO, KC_Y, KC_U, KC_I, KC_O, KC_P, GUI_A, ALT_S, CTL_D, SHFT_F, L2_G, KC_NO, KC_NO, L2_H, SHFT_J, CTL_K, ALT_L, GUI_SCLN, KC_Z, KC_X, KC_C, KC_V, KC_B, KC_NO, KC_NO, KC_N, KC_M, KC_COMM, KC_DOT, KC_SLSH, KC_NO, KC_NO, KC_NO, L2_ESC, SHFT_BSP, KC_NO, KC_NO, SHFT_SPC, L1_ENT, KC_NO, DF(15), KC_DEL),
        [1] = LAYOUT_ortho_5x12(KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_BTN2, KC_MS_L, KC_MS_D, KC_MS_U, KC_MS_R, KC_BTN1, KC_F1, KC_F2, KC_F3, KC_F4, KC_PAUS, KC_NO, KC_NO, KC_MPRV, KC_VOLD, KC_VOLU, KC_MNXT, KC_MPLY, KC_F5, KC_F6, KC_F7, KC_F8, KC_SLCK, KC_NO, KC_NO, KC_LEFT, KC_DOWN, KC_UP, KC_RGHT, KC_MUTE, KC_F9, KC_F10, KC_F11, KC_F12, KC_INS, KC_NO, KC_NO, KC_HOME, KC_PGDN, KC_PGUP, KC_END, KC_PSCR, KC_NO, KC_NO, KC_TRNS, LT(2,KC_ESC), LSFT_T(KC_DEL), KC_TRNS, KC_TRNS, LSFT_T(KC_TAB), KC_TRNS, KC_TRNS, KC_NO, RESET),
        [2] = LAYOUT_ortho_5x12(KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_1, KC_2, KC_3, KC_4, KC_5, KC_NO, KC_NO, KC_6, KC_7, KC_8, KC_9, KC_0, LGUI_T(KC_GRV), LALT_T(KC_MINS), LCTL_T(KC_LBRC), LSFT_T(KC_RBRC), KC_PPLS, KC_NO, KC_NO, KC_PEQL, RSFT_T(KC_4), RCTL_T(KC_5), LALT_T(KC_6), RGUI_T(KC_QUOT), KC_BSLS, KC_SLSH, KC_LCBR, KC_RCBR, KC_PAST, KC_NO, KC_NO, KC_DOT, KC_1, KC_2, KC_3, KC_SLSH, KC_NO, KC_NO, KC_TRNS, KC_TRNS, LSFT_T(KC_DEL), KC_TRNS, KC_TRNS, LSFT_T(KC_TAB), LT(1,KC_ENT), KC_TRNS, KC_NO, RESET),
        [3] = LAYOUT_ortho_5x12(KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, TO(14), KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_MPLY, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_MPRV, KC_VOLD, KC_VOLU, KC_MNXT, KC_MPLY, KC_NO, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_MUTE, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TAB, KC_TRNS, TO(0), KC_TRNS, KC_TRNS),
        [4] = LAYOUT_ortho_5x12(KC_F13, KC_F14, KC_F15, KC_F16, KC_F17, KC_NO, KC_NO, KC_F20, KC_F21, KC_F22, KC_F23, KC_F24, KC_EXLM, KC_AT, KC_HASH, KC_DLR, KC_PERC, KC_NO, KC_NO, KC_CIRC, KC_AMPR, KC_ASTR, KC_LPRN, KC_RPRN, KC_TILD, KC_F1, KC_F2, KC_F3, KC_F4, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_UNDS, KC_NO, KC_F7, KC_F8, KC_F9, KC_F10, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_NO, RESET),
        [5] = LAYOUT_ortho_5x12(KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, LT(13,KC_NO), KC_TRNS, KC_NO, KC_NO, LT(7,KC_NO), KC_TRNS, TO(0), KC_NO, KC_NO),
        [6] = LAYOUT_ortho_5x12(KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_MPLY, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_MPRV, KC_VOLD, KC_VOLU, KC_MNXT, KC_MPLY, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_MUTE, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, LT(14,KC_NO), KC_TRNS, TO(0), KC_NO, KC_TRNS, LT(7,KC_NO), TO(0), KC_NO, KC_NO),
        [7] = LAYOUT_ortho_5x12(KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_NO, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, TO(0), KC_TRNS, KC_TRNS),
        [8] = LAYOUT_ortho_5x12(KC_ESC, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NO, KC_NLCK, KC_NO, KC_LPRN, KC_RPRN, KC_NO, KC_TAB, KC_NO, KC_PSCR, KC_SLCK, KC_PAUS, KC_NO, KC_NO, KC_7, KC_8, KC_9, KC_PAST, KC_PEQL, KC_LCTL, KC_NO, KC_INS, KC_HOME, KC_PGUP, KC_NO, KC_NO, KC_4, KC_5, KC_6, KC_PMNS, KC_PPLS, KC_LSFT, KC_NO, KC_DEL, KC_END, KC_PGDN, KC_NO, KC_NO, KC_1, KC_2, KC_3, KC_PSLS, KC_SFTENT, KC_LCTL, KC_LGUI, KC_LALT, KC_TRNS, LT(12,KC_NO), KC_BSPC, KC_PDOT, LT(10,KC_NO), LT(9,KC_NO), KC_0, KC_PENT, KC_NO),
        [9] = LAYOUT_ortho_5x12(KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, LT(13,KC_NO), KC_TRNS, KC_TRNS, LT(11,KC_NO), KC_TRNS, TO(0), KC_TRNS, KC_TRNS),
        [10] = LAYOUT_ortho_5x12(KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, LT(14,KC_NO), KC_TRNS, KC_TRNS, KC_TRNS, LT(11,KC_NO), TO(0), KC_TRNS, KC_TRNS),
        [11] = LAYOUT_ortho_5x12(KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_NO, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, TO(0), KC_TRNS, KC_TRNS),
        [12] = LAYOUT_ortho_5x12(KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, LT(14,KC_NO), LT(13,KC_NO), TO(0), KC_TRNS, KC_TRNS),
        [13] = LAYOUT_ortho_5x12(RGB_MOD, RGB_M_P, RGB_M_B, RGB_M_R, RGB_M_SW, RGB_M_SN, RGB_M_K, RGB_M_X, RGB_M_G, RGB_M_T, RGB_TOG, KC_TRNS, RGB_RMOD, KC_TRNS, RGB_SAI, RGB_SPI, KC_TRNS, RGB_VAI, RGB_HUI, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, RGB_SAD, RGB_SPD, KC_TRNS, RGB_VAD, RGB_HUD, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, RGB_TOG, RGB_TOG, KC_NO, KC_TRNS, TO(0), KC_TRNS, KC_TRNS),
        [14] = LAYOUT_ortho_5x12(KC_TRNS, KC_F1, KC_F2, KC_F3, KC_F4, KC_F5, KC_F6, KC_F7, KC_F8, KC_F9, KC_F10, KC_F11, KC_TRNS, RGB_M_R, KC_TRNS, KC_TRNS, RESET, KC_TRNS, KC_TRNS, TERM_ON, TERM_OFF, KC_TRNS, DEBUG, KC_F12, KC_TRNS, KC_TRNS, MU_MOD, AU_ON, AU_OFF, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_SLEP, KC_TRNS, MUV_DE, MUV_IN, MU_ON, MU_OFF, MI_ON, MI_OFF, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, KC_TRNS, TO(0), KC_TRNS, KC_TRNS),
        [15] = LAYOUT_ortho_5x12(KC_ESC, KC_1, KC_2, KC_3, KC_4, KC_5, KC_BSPC, KC_6, KC_7, KC_8, KC_9, KC_0, KC_TAB, KC_Q, KC_W, KC_E, KC_R, KC_T, KC_DEL, KC_Y, KC_U, KC_I, KC_O, KC_P, KC_HOME, KC_A, KC_S, KC_D, KC_F, KC_G, KC_QUOT, KC_H, KC_J, KC_K, KC_L, KC_SCLN, KC_LSFT, KC_Z, KC_X, KC_C, KC_V, KC_B, KC_SFTENT, KC_N, KC_M, KC_COMM, KC_DOT, KC_SLSH, KC_LCTL, KC_END, KC_LALT, KC_SPC, KC_SPC, KC_SPC, KC_NO, KC_BSPC, KC_ENT, KC_NO, DF(0), KC_GRV)
};

// bool process_record_user(uint16_t keycode, keyrecord_t *record) {
//   switch (keycode) {
//         case QWERTY:
//           if (record->event.pressed) {
//             set_single_persistent_default_layer(_QWERTY);
//           }
//           return false;
//           break;
//         case COLEMAK:
//           if (record->event.pressed) {
//             set_single_persistent_default_layer(_COLEMAK);
//           }
//           return false;
//           break;
//         case DVORAK:
//           if (record->event.pressed) {
//             set_single_persistent_default_layer(_DVORAK);
//           }
//           return false;
//           break;
//         case LOWER:
//           if (record->event.pressed) {
//             layer_on(_LOWER);
//             update_tri_layer(_LOWER, _RAISE, _ADJUST);
//           } else {
//             layer_off(_LOWER);
//             update_tri_layer(_LOWER, _RAISE, _ADJUST);
//           }
//           return false;
//           break;
//         case RAISE:
//           if (record->event.pressed) {
//             layer_on(_RAISE);
//             update_tri_layer(_LOWER, _RAISE, _ADJUST);
//           } else {
//             layer_off(_RAISE);
//             update_tri_layer(_LOWER, _RAISE, _ADJUST);
//           }
//           return false;
//           break;
//         case BACKLIT:
//           if (record->event.pressed) {
//             register_code(KC_RSFT);
//             #ifdef BACKLIGHT_ENABLE
//               backlight_step();
//             #endif
//             #ifdef RGBLIGHT_ENABLE
//               rgblight_step();
//             #endif
//             #ifdef __AVR__
//             writePinLow(E6);
//             #endif
//           } else {
//             unregister_code(KC_RSFT);
//             #ifdef __AVR__
//             writePinHigh(E6);
//             #endif
//           }
//           return false;
//           break;
//       }
//     return true;
// };

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

bool encoder_update_user(uint8_t index, bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
      register_code(KC_PGDN);
      unregister_code(KC_PGDN);
    } else {
      register_code(KC_PGUP);
      unregister_code(KC_PGUP);
    }
  }
    return true;
}

bool dip_switch_update_user(uint8_t index, bool active) {
    switch (index) {
        case 0:
            if (active) {
                layer_on(_ADJUST);
            } else {
                layer_off(_ADJUST);
            }
            break;
        case 1:
            if (active) {
                muse_mode = true;
            } else {
                muse_mode = false;
            }
    }
    return true;
}


void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    } else {
        if (muse_counter) {
            stop_all_notes();
            muse_counter = 0;
        }
    }
#endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}

// bool get_tapping_force_hold(uint16_t keycode, keyrecord_t *record) {
//     switch (keycode) {
//         case GUI_A:
//             return true;
//         case ALT_S:
//             return true;
//         case CTL_D:
//             return true;
//         case SHFT_F:
//             return true;
//         case L2_G:
//             return true;
//         case L2_H:
//             return true;
//         case SHFT_J:
//             return true;
//         case CTL_K:
//             return true;
//         case ALT_L:
//             return true;
//         case GUI_SCLN:
//             return true;
//         case L2_ESC:
//             return true;
//         case SHFT_SPC:
//             return true;
//         case L1_ENT:
//             return true;
//         default:
//             return false;
//     }
// }

bool get_tapping_force_hold(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case SHFT_BSP:
            return false;
        default:
            return true;
    }
}
