// Copyright 2025 EPOMAKER (@Epomaker)
// SPDX-License-Identifier: GPL-2.0-or-later

#include QMK_KEYBOARD_H
#include "rgb_record/rgb_record.h"
#include "quantum.h"
#include "serial_usart.h"
#ifdef WIRELESS_ENABLE
#    include "wls/wls.h"
#    include "wireless.h"
#    include "usb_main.h"
#    include "lowpower.h"
#    include "module.h"

// Add this forward declaration
void usb_mode_test_report_task(void);
extern bool im_test_rate_flag;
#endif

typedef union {
    uint32_t raw;
    struct {
        uint8_t flag : 1;
        uint8_t devs : 3;
        uint8_t record_channel : 4;
        uint8_t record_last_mode;
        uint8_t last_btdevs : 3;
        uint8_t dir_flag : 1;
        uint8_t filp : 1;
    };
} confinfo_t;
confinfo_t confinfo;

typedef struct {
    bool     active;
    uint32_t timer;
    uint32_t interval;
    uint32_t times;
    uint8_t  index;
    RGB      rgb;
    void (*blink_cb)(uint8_t);
} hs_rgb_indicator_t;

enum layers {
    _BL = 0,
    _FL,
    _MBL,
    _MFL,
};

hs_rgb_indicator_t hs_rgb_indicators[HS_RGB_INDICATOR_COUNT];
hs_rgb_indicator_t hs_rgb_bat[HS_RGB_BAT_COUNT];

void user_sync_mms_slave_handler(uint8_t in_buflen, const void *in_data, uint8_t out_buflen, void *out_data);
void rgb_blink_dir(void);
void hs_reset_settings(void);
void rgb_matrix_hs_indicator(void);
void rgb_matrix_hs_indicator_set(uint8_t index, RGB rgb, uint32_t interval, uint8_t times);
void rgb_matrix_hs_set_remain_time(uint8_t index, uint8_t remain_time);

#define keymap_is_mac_system() ((get_highest_layer(default_layer_state) == _MBL) || (get_highest_layer(default_layer_state) == _MFL))
#define keymap_is_base_layer() ((get_highest_layer(default_layer_state) == _BL) || (get_highest_layer(default_layer_state) == _FL))

uint32_t        post_init_timer       = 0x00;
bool            inqbat_flag           = false;
bool            mac_status            = false;
bool            charging_state        = false;
bool            bat_full_flag         = false;
bool            enable_bat_indicators = true;
uint32_t        bat_indicator_cnt     = 0;
static uint32_t ee_clr_timer          = 0;
bool            test_white_light_flag = false;
HSV             start_hsv;
bool            no_record_fg;
bool            lower_sleep = false;
uint8_t         pov;
static bool     im_bat_req_charging_flag = false;
uint8_t         buff[]                   = {14, 8, 2, 1, 1, 1, 1, 1, 1, 1, 0};

void usart_init(void) {
    // palSetLineMode(SERIAL_USART_TX_PIN, PAL_MODE_ALTERNATE(SERIAL_USART_TX_PAL_MODE) | PAL_OUTPUT_TYPE_OPENDRAIN);
    palSetLineMode(SERIAL_USART_TX_PIN, PAL_MODE_ALTERNATE(SERIAL_USART_TX_PAL_MODE) | PAL_OUTPUT_TYPE_PUSHPULL | PAL_OUTPUT_SPEED_HIGHEST);
    palSetLineMode(SERIAL_USART_RX_PIN, PAL_MODE_ALTERNATE(SERIAL_USART_RX_PAL_MODE) | PAL_OUTPUT_TYPE_PUSHPULL | PAL_OUTPUT_SPEED_HIGHEST);
}

void eeconfig_confinfo_update(uint32_t raw) {
    eeconfig_update_kb(raw);
}

typedef struct _master_to_slave_t {
    uint8_t cmd;
    // Widened from 4 to 12 so the strip frame sync (0xEE) can carry the right
    // half's three RGB triplets plus a mode byte. Both halves run the same
    // image, so the two ends of the RPC stay in agreement.
    uint8_t body[12];
} master_to_slave_t;

typedef struct _slave_to_master_t {
    uint8_t resp;
    uint8_t body[4];
} slave_to_master_t;

/* ------------------------------------------------------------------------
 * Companion-app control of the six accent strip LEDs.
 *
 * The strip LEDs are indices 0-2 (left half) and 36-38 (right half); they all
 * map to matrix [0,0] in keyboard.json because they are not tied to any key.
 *
 * A host application drives them over raw HID using a VIA custom channel, so
 * the same protocol works on USB, Bluetooth and 2.4G: md_raw.c bridges raw HID
 * onto the wireless module, and via.c is compiled with
 * -Draw_hid_send=replaced_hid_send (see linker/wireless/wireless.mk) so replies
 * go out over whichever link is active.
 *
 * NOTE: that -D rename applies to via.o only. Never call raw_hid_send() from
 * this file -- it is a silent no-op in wireless mode. Handling the request in
 * via_custom_value_command_kb() and letting via.c send the reply avoids this.
 * ---------------------------------------------------------------------- */

#define STRIP_LED_COUNT 6
#define STRIP_RIGHT_FIRST 3 // array index at which the right half's LEDs begin

static const uint8_t strip_led_indices[STRIP_LED_COUNT] = {0, 1, 2, 36, 37, 38};

#define STRIP_CFG_MAGIC 0x53

typedef struct {
    uint8_t  magic;       // STRIP_CFG_MAGIC once written
    uint8_t  boot_app;    // take over the strip at power-on?
    uint8_t  h;           // fallback colour, also the first frame on takeover
    uint8_t  s;
    uint8_t  v;
    uint16_t watchdog_ms; // 0 disables
    uint8_t  reserved;
} strip_cfg_t;

static strip_cfg_t strip_cfg;

static bool     strip_app_mode;
static uint8_t  strip_rgb[STRIP_LED_COUNT][3];
static uint32_t strip_last_frame;
static bool     strip_sync_dirty;
// Separate from strip_sync_dirty: the fallback colour and boot-app default
// change rarely (via `default`/`boot`/`save`), so they ride a lighter 0xEF
// command instead of riding along on every frame update.
static bool     strip_cfg_dirty;

static void strip_fill_fallback(void) {
    RGB rgb = hsv_to_rgb((HSV){.h = strip_cfg.h, .s = strip_cfg.s, .v = strip_cfg.v});
    for (uint8_t i = 0; i < STRIP_LED_COUNT; i++) {
        strip_rgb[i][0] = rgb.r;
        strip_rgb[i][1] = rgb.g;
        strip_rgb[i][2] = rgb.b;
    }
}

static void strip_cfg_load(void) {
    eeconfig_read_user_datablock(&strip_cfg, (uint32_t)STRIP_EECONFIG_OFFSET, sizeof(strip_cfg));

    if (strip_cfg.magic != STRIP_CFG_MAGIC) {
        strip_cfg.magic       = STRIP_CFG_MAGIC;
        strip_cfg.boot_app    = 0;
        strip_cfg.h           = 0;
        strip_cfg.s           = 0;
        strip_cfg.v           = RGB_MATRIX_MAXIMUM_BRIGHTNESS;
        strip_cfg.watchdog_ms = 3000;
        strip_cfg.reserved    = 0;
    }

    strip_app_mode   = strip_cfg.boot_app ? true : false;
    strip_last_frame = timer_read32();
    strip_fill_fallback();
    // The slave's own EEPROM copy is never written -- id_custom_save only
    // ever reaches the master, since raw HID only reaches whichever half is
    // plugged in. Push both dirty flags so the slave's runtime state is
    // overwritten with the master's true persisted config shortly after boot,
    // rather than running on its own stale/default copy indefinitely.
    strip_sync_dirty = true;
    strip_cfg_dirty  = true;
}

static void strip_cfg_save(void) {
    strip_cfg.magic = STRIP_CFG_MAGIC;
    eeconfig_update_user_datablock(&strip_cfg, (uint32_t)STRIP_EECONFIG_OFFSET, sizeof(strip_cfg));
}

// True while the host is actively driving the strip. Once the watchdog expires
// we hand the strip back to the normal rgb_matrix effect rather than freezing
// on the last frame received.
static bool strip_app_active(void) {
    if (!strip_app_mode) return false;

    if (strip_cfg.watchdog_ms && timer_elapsed32(strip_last_frame) > strip_cfg.watchdog_ms) {
        strip_app_mode = false;
        return false;
    }

    return true;
}

static void strip_render(uint8_t led_min, uint8_t led_max) {
    if (!strip_app_active()) return;

    for (uint8_t i = 0; i < STRIP_LED_COUNT; i++) {
        uint8_t index = strip_led_indices[i];
        if (index < led_min || index >= led_max) continue;
        // The accent strip LEDs are wired R/G-swapped relative to the rest of
        // the matrix (confirmed by dump-frame: the firmware stores the
        // intended colour correctly, but the strip renders it with red and
        // green interchanged) -- compensate here rather than in the wire
        // protocol, so strip_rgb and GetFrame readback keep the colour the
        // host actually asked for.
        rgb_matrix_set_color(index, strip_rgb[i][1], strip_rgb[i][0], strip_rgb[i][2]);
    }
}

#ifdef VIA_ENABLE
#    include "via.h"

// Channel id chosen well clear of the QMK core channels (1..5).
#    define id_epomaker_strip_channel 0x10

enum epomaker_strip_value_id {
    id_strip_mode     = 0x01, // 1 byte : 0 = firmware effects, 1 = host driven
    id_strip_frame    = 0x02, // 18 bytes: RGB x6, order {0,1,2,36,37,38}
    id_strip_default  = 0x03, // 3 bytes : fallback HSV
    id_strip_identify = 0x04, // get only
    id_strip_watchdog = 0x05, // 2 bytes : timeout ms, big endian, 0 disables
    id_strip_boot     = 0x06, // 1 byte  : enter host-driven mode at power-on
};

#    define STRIP_PROTOCOL_VERSION 1

void via_custom_value_command_kb(uint8_t *data, uint8_t length) {
    // data = [ command_id, channel_id, value_id, value_data... ]
    uint8_t *command_id   = &(data[0]);
    uint8_t *channel_id   = &(data[1]);
    uint8_t *value_id     = &(data[2]);
    uint8_t *value_data   = &(data[3]);

    if (*channel_id != id_epomaker_strip_channel) {
        *command_id = id_unhandled;
        return;
    }

    switch (*command_id) {
        case id_custom_set_value: {
            switch (*value_id) {
                case id_strip_mode: {
                    bool enable = value_data[0] ? true : false;
                    // Entering host control with no frame yet: show the
                    // fallback colour rather than whatever was left over.
                    if (enable && !strip_app_mode) strip_fill_fallback();
                    strip_app_mode   = enable;
                    strip_last_frame = timer_read32();
                    strip_sync_dirty = true;
                    break;
                }
                case id_strip_frame: {
                    for (uint8_t i = 0; i < STRIP_LED_COUNT; i++) {
                        strip_rgb[i][0] = value_data[(i * 3) + 0];
                        strip_rgb[i][1] = value_data[(i * 3) + 1];
                        strip_rgb[i][2] = value_data[(i * 3) + 2];
                    }
                    // A frame implies host control; this also feeds the watchdog.
                    strip_app_mode   = true;
                    strip_last_frame = timer_read32();
                    strip_sync_dirty = true;
                    break;
                }
                case id_strip_default: {
                    strip_cfg.h     = value_data[0];
                    strip_cfg.s     = value_data[1];
                    strip_cfg.v     = value_data[2];
                    strip_cfg_dirty = true;
                    break;
                }
                case id_strip_watchdog: {
                    strip_cfg.watchdog_ms = (uint16_t)((value_data[0] << 8) | value_data[1]);
                    strip_last_frame      = timer_read32();
                    // Rides the existing 0xEE frame sync rather than 0xEF:
                    // the slave enforces its own watchdog independently (it
                    // never sees the host), so a value that only lives on the
                    // master leaves the two halves reverting on different
                    // schedules.
                    strip_sync_dirty      = true;
                    break;
                }
                case id_strip_boot: {
                    strip_cfg.boot_app = value_data[0] ? 1 : 0;
                    strip_cfg_dirty    = true;
                    break;
                }
                default: {
                    *command_id = id_unhandled;
                    break;
                }
            }
            break;
        }

        case id_custom_get_value: {
            switch (*value_id) {
                case id_strip_mode: {
                    value_data[0] = strip_app_active() ? 1 : 0;
                    break;
                }
                case id_strip_frame: {
                    for (uint8_t i = 0; i < STRIP_LED_COUNT; i++) {
                        value_data[(i * 3) + 0] = strip_rgb[i][0];
                        value_data[(i * 3) + 1] = strip_rgb[i][1];
                        value_data[(i * 3) + 2] = strip_rgb[i][2];
                    }
                    break;
                }
                case id_strip_default: {
                    value_data[0] = strip_cfg.h;
                    value_data[1] = strip_cfg.s;
                    value_data[2] = strip_cfg.v;
                    break;
                }
                case id_strip_identify: {
                    // Magic lets the host tell this firmware apart from any
                    // other device exposing the VIA raw usage page.
                    value_data[0] = 'E';
                    value_data[1] = 'P';
                    value_data[2] = 'S';
                    value_data[3] = 'T';
                    value_data[4] = STRIP_PROTOCOL_VERSION;
                    value_data[5] = STRIP_LED_COUNT;
#    ifdef WIRELESS_ENABLE
                    // Current link, so the host can pick a sane frame rate.
                    value_data[6] = wireless_get_current_devs();
#    else
                    value_data[6] = 0;
#    endif
                    break;
                }
                case id_strip_watchdog: {
                    value_data[0] = (uint8_t)(strip_cfg.watchdog_ms >> 8);
                    value_data[1] = (uint8_t)(strip_cfg.watchdog_ms & 0xFF);
                    break;
                }
                case id_strip_boot: {
                    value_data[0] = strip_cfg.boot_app;
                    break;
                }
                default: {
                    *command_id = id_unhandled;
                    break;
                }
            }
            break;
        }

        case id_custom_save: {
            strip_cfg_save();
            break;
        }

        default: {
            *command_id = id_unhandled;
            break;
        }
    }
}
#endif // VIA_ENABLE

// The strip LEDs at indices 36-38 live on the right half, and the automatic
// PUT_RGB_MATRIX split transaction only carries rgb_config_t -- it will not
// carry this custom state. Push it over the existing user RPC instead.
static void strip_sync_push(void) {
    if (!is_keyboard_master()) return;

    if (strip_sync_dirty) {
        master_to_slave_t m2s = {0};
        slave_to_master_t s2m = {0};

        m2s.cmd     = 0xEE;
        m2s.body[0] = strip_app_mode ? 1 : 0;
        for (uint8_t i = 0; i < (STRIP_LED_COUNT - STRIP_RIGHT_FIRST); i++) {
            m2s.body[1 + (i * 3) + 0] = strip_rgb[STRIP_RIGHT_FIRST + i][0];
            m2s.body[1 + (i * 3) + 1] = strip_rgb[STRIP_RIGHT_FIRST + i][1];
            m2s.body[1 + (i * 3) + 2] = strip_rgb[STRIP_RIGHT_FIRST + i][2];
        }
        // The slave enforces its own watchdog independently -- it never sees
        // the host directly -- so its copy of the timeout has to ride along
        // here, or it reverts on a different schedule than the master.
        m2s.body[10] = (uint8_t)(strip_cfg.watchdog_ms >> 8);
        m2s.body[11] = (uint8_t)(strip_cfg.watchdog_ms & 0xFF);

        // Retry on the next pass if the split bus was busy, otherwise a
        // dropped frame would leave the two halves showing different colours.
        if (transaction_rpc_exec(USER_SYNC_MMS, sizeof(m2s), &m2s, sizeof(s2m), &s2m)) {
            strip_sync_dirty = false;
        }
    }

    if (strip_cfg_dirty) {
        master_to_slave_t m2s = {0};
        slave_to_master_t s2m = {0};

        m2s.cmd     = 0xEF;
        m2s.body[0] = strip_cfg.h;
        m2s.body[1] = strip_cfg.s;
        m2s.body[2] = strip_cfg.v;
        m2s.body[3] = strip_cfg.boot_app;

        if (transaction_rpc_exec(USER_SYNC_MMS, sizeof(m2s), &m2s, sizeof(s2m), &s2m)) {
            strip_cfg_dirty = false;
        }
    }
}

uint32_t eeconfig_confinfo_read(void) {
    return eeconfig_read_kb();
}

void eeconfig_confinfo_default(void) {
    confinfo.flag             = true;
    confinfo.record_channel   = 0;
    confinfo.record_last_mode = 0xff;
    confinfo.last_btdevs      = 1;
    confinfo.dir_flag         = 0;

    // #ifdef WIRELESS_ENABLE
    //     confinfo.devs = DEVS_USB;
    // #endif

    eeconfig_init_user_datablock();
    eeconfig_confinfo_update(confinfo.raw);

#ifdef RGBLIGHT_ENABLE
    rgblight_mode(buff[0]);
#endif
}

void master_sync_mms_slave(uint8_t last_mode, uint8_t now_mode, uint8_t reset) {
    master_to_slave_t m2s = {0};
    slave_to_master_t s2m = {0};
    m2s.cmd               = 0x55;
    m2s.body[0]           = last_mode;
    m2s.body[1]           = now_mode;
    m2s.body[2]           = reset;
    if (transaction_rpc_exec(USER_SYNC_MMS, sizeof(m2s), &m2s, sizeof(s2m), &s2m)) {
        if (s2m.resp == 0x00)
            ;
        dprintf("Slave Sleep OK1\n");
    } else {
        dprintf("Slave sync failed1!\n");
    }
}

void eeconfig_confinfo_init(void) {
    confinfo.raw = eeconfig_confinfo_read();
    if (!confinfo.raw) {
        eeconfig_confinfo_default();
    }
}

// Master detection for this wireless split. The default QMK logic picks the
// master by USB-host presence, which is fine when cabled but leaves BOTH halves
// as slave in wireless/dongle mode (no USB host) -> nothing works wirelessly.
// So: if a USB cable is inserted, this half is master; otherwise fall back to
// the handedness pin so the left half drives the wireless module.
bool is_keyboard_master(void) {
    if (gpio_read_pin(HS_BAT_CABLE_PIN)) return true;
    gpio_set_pin_input(SPLIT_HAND_PIN);
    return gpio_read_pin(SPLIT_HAND_PIN);
}

void keyboard_post_init_kb(void) {
#ifdef CONSOLE_ENABLE
    debug_enable = true;
#endif

    eeconfig_confinfo_init();

    strip_cfg_load();

    // Only master controls LED_POWER_EN_PIN/EN2_PIN directly
    // Slave uses A5/A8 controlled via RPC commands
    if (is_keyboard_master()) {
#ifdef LED_POWER_EN_PIN
        gpio_set_pin_output(LED_POWER_EN_PIN);
        if (rgb_matrix_get_val() != 0) gpio_write_pin_high(LED_POWER_EN_PIN);
#endif

#ifdef LED_POWER_EN2_PIN
        gpio_set_pin_output(LED_POWER_EN2_PIN);
        if (rgb_matrix_get_val() != 0) gpio_write_pin_high(LED_POWER_EN2_PIN);
#endif
    } else {
        // Slave: initialize A5/A8 directly (not via LED_POWER_EN defines)
        gpio_set_pin_output(A5);
        gpio_set_pin_output(A8);
        if (rgb_matrix_get_val() != 0) {
            gpio_write_pin_high(A5);
            gpio_write_pin_high(A8);
        }
    }

#ifdef HS_BT_DEF_PIN
    gpio_set_pin_input_high(HS_BT_DEF_PIN);
#endif

#ifdef HS_2G4_DEF_PIN
    gpio_set_pin_input_high(HS_2G4_DEF_PIN);
#endif

#ifdef USB_POWER_EN_PIN
    gpio_write_pin_low(USB_POWER_EN_PIN);
    gpio_set_pin_output(USB_POWER_EN_PIN);
#endif

#ifdef HS_BAT_CABLE_PIN
    gpio_set_pin_input(HS_BAT_CABLE_PIN);
#endif

#ifdef BAT_FULL_PIN
    gpio_set_pin_input_high(BAT_FULL_PIN);
#endif

#ifdef WIRELESS_ENABLE
    wireless_init();
#    if (!(defined(HS_BT_DEF_PIN) && defined(HS_2G4_DEF_PIN)))
    wireless_devs_change(!confinfo.devs, confinfo.devs, false);
#    endif
    post_init_timer = timer_read32();
#endif

    keyboard_post_init_user();

    rgbrec_init(confinfo.record_channel);

    start_hsv = rgb_matrix_get_hsv();

#ifdef WIRELESS_ENABLE
    pov = *md_getp_bat();
#endif
    // usart_init();
    transaction_register_rpc(USER_SYNC_MMS, user_sync_mms_slave_handler);
}

#ifdef WIRELESS_ENABLE

void usb_power_connect(void) {
#    ifdef USB_POWER_EN_PIN
    gpio_write_pin_low(USB_POWER_EN_PIN);
#    endif
}

void usb_power_disconnect(void) {
#    ifdef USB_POWER_EN_PIN
    gpio_write_pin_high(USB_POWER_EN_PIN);
#    endif
}

void suspend_power_down_kb(void) {
    // Only master controls LED_POWER_EN pins - slave uses A5/A8 via RPC
    if (is_keyboard_master()) {
#    ifdef LED_POWER_EN_PIN
        gpio_write_pin_low(LED_POWER_EN_PIN);
#    endif

#    ifdef LED_POWER_EN2_PIN
        gpio_write_pin_low(LED_POWER_EN2_PIN);
#    endif
    }

    suspend_power_down_user();
}

void suspend_wakeup_init_kb(void) {
    // Give split UART time to stabilize before powering LEDs
    // This prevents flickering on the slave/right half
    // wait_ms(50);

    // Clear keyboard state to prevent stuck modifiers
    // This ensures clean state after wake, especially important
    // when waking with modifier keys from the slave half
    // clear_keyboard();

    // Only master controls LED_POWER_EN pins - slave uses A5/A8 via RPC
    if (is_keyboard_master()) {
#    ifdef LED_POWER_EN_PIN
        if (rgb_matrix_get_val() != 0) gpio_write_pin_high(LED_POWER_EN_PIN);
#    endif

#    ifdef LED_POWER_EN2_PIN
        if (rgb_matrix_get_val() != 0) gpio_write_pin_high(LED_POWER_EN2_PIN);
#    endif
    }

    wireless_devs_change(wireless_get_current_devs(), wireless_get_current_devs(), false);
    suspend_wakeup_init_user();
    hs_rgb_blink_set_timer(timer_read32());
}

void suspend_wakeup_init_user(void) {
    usart_init();
    master_to_slave_t m2s = {0};
    slave_to_master_t s2m = {0};
    m2s.cmd               = 0xCC;
    if (transaction_rpc_exec(USER_SYNC_MMS, sizeof(m2s), &m2s, sizeof(s2m), &s2m)) {
        if (s2m.resp == 0x00) {}
        dprintf("Slave Sleep OK\n");
    } else {
        dprint("Slave sync failed!\n");
    }
}

void suspend_power_down_user(void) {

    master_to_slave_t m2s = {0};
    slave_to_master_t s2m = {0};
    m2s.cmd               = 0xBB;
    if (transaction_rpc_exec(USER_SYNC_MMS, sizeof(m2s), &m2s, sizeof(s2m), &s2m)) {
        if (s2m.resp == 0x00) {}
        dprintf("Slave Sleep OK\n");
    } else {
        dprint("Slave sync failed!\n");
    }

}

bool lpwr_is_allow_timeout_hook(void) {

    // In wired/USB mode neither half may enter low-power sleep. The check must
    // NOT be gated on is_keyboard_master(): the slave (right half) is not the
    // USB host, so a master-only guard lets it time out on its own after
    // ~5 min of no local keypress. On such an uncoordinated sleep it arms only
    // the cable-insert wake source, so key presses can't wake it and the half
    // appears frozen until unplugged/replugged. Gating on devs alone keeps the
    // slave awake whenever the keyboard is running over the cable.
    if (wireless_get_current_devs() == DEVS_USB) {
        return false;
    }

    // Wireless: only the master runs the idle-timeout. QMK's split activity sync
    // is slave->master, so the master's inactivity timer reflects the WHOLE
    // keyboard, but the slave's local timer never sees master keypresses. If the
    // slave were allowed to time out on its own it would sleep while you type on
    // the left half, leaving the right half unresponsive until a right-side key
    // (matrix wake) or a replug. So the slave sleeps only when the master
    // coordinates it via the 0xAA RPC, which sets manual_timeout on the slave.
    if (!is_keyboard_master() && !lpwr_get_timeout_manual()) {
        return false;
    }

    return true;
}

bool lpwr_is_allow_presleep_hook(void) {
    extern bool charging_state;

    if (is_keyboard_master()) {
        master_to_slave_t m2s = {0};
        slave_to_master_t s2m = {0};
        m2s.cmd               = 0xAA;
        m2s.body[0] = lower_sleep;
        if (transaction_rpc_exec(USER_SYNC_MMS, sizeof(m2s), &m2s, sizeof(s2m), &s2m)) {
            if (s2m.resp == 0x00) {}
            dprintf("Slave Sleep OK\n");
        } else {
            dprint("Slave sync failed!\n");
        }
    }

    if ((wireless_get_current_devs() == DEVS_USB) && (!charging_state)) {

        if (USB_DRIVER.state != USB_STOP) {
            usb_power_disconnect();
            usbDisconnectBus(&USBD1);
            usbStop(&USBD1);
        }
    }
    return true;
}

void lpwr_stop_hook_pre(void) {
    // Deferred from lpwr_is_allow_presleep_hook()/the 0xAA RPC handler:
    // switching these pins to open-drain before the 0xBB power-down RPC
    // breaks that transaction (no external pull-up to drive a clean
    // logic-high), which is why the slave/right half's LED rail never got
    // powered down in wireless mode. Do it here instead, after
    // suspend_power_down_user()'s 0xBB has already been sent/received.
    if (confinfo.devs != DEVS_USB) {
        palSetLineMode(SERIAL_USART_RX_PIN, PAL_OUTPUT_TYPE_OPENDRAIN);
        palSetLineMode(SERIAL_USART_TX_PIN, PAL_OUTPUT_TYPE_OPENDRAIN);
    }
}

void wireless_post_task(void) {
    // auto switching devs
    if (post_init_timer && timer_elapsed32(post_init_timer) >= 100) {
        md_send_devctrl(MD_SND_CMD_DEVCTRL_FW_VERSION);   // get the module fw version.
        md_send_devctrl(MD_SND_CMD_DEVCTRL_SLEEP_BT_EN);  // timeout 30min to sleep in bt mode, enable
        md_send_devctrl(MD_SND_CMD_DEVCTRL_SLEEP_2G4_EN); // timeout 30min to sleep in 2.4g mode, enable
        wireless_devs_change(!confinfo.devs, confinfo.devs, false);
        post_init_timer = 0x00;
    }
#    if defined(HS_BT_DEF_PIN) && defined(HS_2G4_DEF_PIN)
    hs_mode_scan(false, confinfo.devs, confinfo.last_btdevs);
#    endif
}

uint32_t wls_process_long_press(uint32_t trigger_time, void *cb_arg) {
    uint16_t keycode = *((uint16_t *)cb_arg);

    switch (keycode) {
        case KC_BT1: {
            uint8_t mode = confinfo.devs;
            hs_modeio_detection(true, &mode, confinfo.last_btdevs);
            if ((mode == hs_bt) || (mode == hs_wireless) || (mode == hs_none)) {
                wireless_devs_change(wireless_get_current_devs(), DEVS_BT1, true);
            }

        } break;
        case KC_BT2: {
            uint8_t mode = confinfo.devs;
            hs_modeio_detection(true, &mode, confinfo.last_btdevs);
            if ((mode == hs_bt) || (mode == hs_wireless) || (mode == hs_none)) {
                wireless_devs_change(wireless_get_current_devs(), DEVS_BT2, true);
            }
        } break;
        case KC_BT3: {
            uint8_t mode = confinfo.devs;
            hs_modeio_detection(true, &mode, confinfo.last_btdevs);
            if ((mode == hs_bt) || (mode == hs_wireless) || (mode == hs_none)) {
                wireless_devs_change(wireless_get_current_devs(), DEVS_BT3, true);
            }
        } break;
        case KC_2G4: {
            uint8_t mode = confinfo.devs;
            hs_modeio_detection(true, &mode, confinfo.last_btdevs);
            if ((mode == hs_2g4) || (mode == hs_wireless) || (mode == hs_none)) {
                wireless_devs_change(wireless_get_current_devs(), DEVS_2G4, true);
            }
        } break;
        case EE_CLR: {
        } break;
        default:
            break;
    }

    return 0;
}

bool process_record_wls(uint16_t keycode, keyrecord_t *record) {
    static uint16_t       keycode_shadow               = 0x00;
    static deferred_token wls_process_long_press_token = INVALID_DEFERRED_TOKEN;

    keycode_shadow = keycode;

#    ifndef WLS_KEYCODE_PAIR_TIME
#        define WLS_KEYCODE_PAIR_TIME 3000
#    endif

#    define WLS_KEYCODE_EXEC(wls_dev)                                                                                          \
        do {                                                                                                                   \
            if (record->event.pressed) {                                                                                       \
                if (wireless_get_current_devs() != wls_dev)                                                                    \
                    wireless_devs_change(wireless_get_current_devs(), wls_dev, false);                                         \
                if (wls_process_long_press_token == INVALID_DEFERRED_TOKEN) {                                                  \
                    wls_process_long_press_token = defer_exec(WLS_KEYCODE_PAIR_TIME, wls_process_long_press, &keycode_shadow); \
                }                                                                                                              \
            } else {                                                                                                           \
                cancel_deferred_exec(wls_process_long_press_token);                                                            \
                wls_process_long_press_token = INVALID_DEFERRED_TOKEN;                                                         \
            }                                                                                                                  \
        } while (false)

    switch (keycode) {
        case KC_BT1: {
            uint8_t mode = confinfo.devs;
            hs_modeio_detection(true, &mode, confinfo.last_btdevs);
            if ((mode == hs_bt) || (mode == hs_wireless) || (mode == hs_none)) {
                WLS_KEYCODE_EXEC(DEVS_BT1);
                hs_rgb_blink_set_timer(timer_read32());
            }

        } break;
        case KC_BT2: {
            uint8_t mode = confinfo.devs;
            hs_modeio_detection(true, &mode, confinfo.last_btdevs);
            if ((mode == hs_bt) || (mode == hs_wireless) || (mode == hs_none)) {
                WLS_KEYCODE_EXEC(DEVS_BT2);
                hs_rgb_blink_set_timer(timer_read32());
            }
        } break;
        case KC_BT3: {
            uint8_t mode = confinfo.devs;
            hs_modeio_detection(true, &mode, confinfo.last_btdevs);
            if ((mode == hs_bt) || (mode == hs_wireless) || (mode == hs_none)) {
                WLS_KEYCODE_EXEC(DEVS_BT3);
                hs_rgb_blink_set_timer(timer_read32());
            }
        } break;
        case KC_2G4: {
            uint8_t mode = confinfo.devs;
            hs_modeio_detection(true, &mode, confinfo.last_btdevs);
            if ((mode == hs_2g4) || (mode == hs_wireless) || (mode == hs_none)) {
                WLS_KEYCODE_EXEC(DEVS_2G4);
                hs_rgb_blink_set_timer(timer_read32());
            }
        } break;
        case KC_USB: {
            uint8_t mode = confinfo.devs;
            hs_modeio_detection(true, &mode, confinfo.last_btdevs);
            if ((mode == hs_usb) || (mode == hs_wireless) || (mode == hs_none)) {
                WLS_KEYCODE_EXEC(DEVS_USB);
                hs_rgb_blink_set_timer(timer_read32());
            }
        } break;

        default:
            return true;
    }

    return false;
}
#endif

bool process_record_user_split65(uint16_t keycode, keyrecord_t *record) {

    // Call the user's keymap function first
    if (!process_record_user(keycode, record)) {
        return false;
    }

    if (test_white_light_flag && record->event.pressed) {
        test_white_light_flag = false;
        rgb_matrix_set_color_all(0x00, 0x00, 0x00);
    }

#ifdef WIRELESS_ENABLE
    if (*md_getp_state() == MD_STATE_CONNECTED) {
        hs_rgb_blink_set_timer(timer_read32());
    }
#endif

    switch (keycode) {
        case MO(_FL):
        case MO(_MFL): {
            if (!record->event.pressed && rgbrec_is_started()) {
                if (no_record_fg == true) {
                    no_record_fg = false;
                    rgbrec_register_record(keycode, record);
                }
                no_record_fg = true;
            }
            break;
        }

        case RM_NEXT:
            break;
        default: {
            if (rgbrec_is_started()) {
                if (!IS_QK_MOMENTARY(keycode) && record->event.pressed) {
                    rgbrec_register_record(keycode, record);

                    return false;
                }
            }
        } break;
    }

    return true;
}

void im_rgblight_increase(void) {
    HSV            rgb;
    uint8_t        moude;
    static uint8_t mode = 0;

    moude = rgblight_get_mode();
    if (moude == 1) {
        rgb = rgblight_get_hsv();
        if (rgb.h == 0 && rgb.s != 0)
            mode = 3;
        else
            mode = 9;
        switch (rgb.h) {
            case 40: {
                mode = 4;
            } break;
            case 80: {
                mode = 5;
            } break;
            case 120: {
                mode = 6;
            } break;
            case 160: {
                mode = 7;
            } break;
            case 200: {
                mode = 8;
            } break;
            default:
                break;
        }
    }

    mode++;
    if (mode == 11) mode = 0;
    if (mode == 10) {
        rgb = rgblight_get_hsv();
        rgblight_sethsv(0, 255, rgb.v);
        rgblight_disable();
    } else {
        rgblight_enable();
        rgblight_mode(buff[mode]);
    }

    rgb = rgblight_get_hsv();
    switch (mode) {
        case 3: {
            rgblight_sethsv(0, 255, rgb.v);
        } break;
        case 4: {
            rgblight_sethsv(40, 255, rgb.v);
        } break;
        case 5: {
            rgblight_sethsv(80, 255, rgb.v);
        } break;
        case 6: {
            rgblight_sethsv(120, 255, rgb.v);
        } break;
        case 7: {
            rgblight_sethsv(160, 255, rgb.v);
        } break;
        case 8: {
            rgblight_sethsv(200, 255, rgb.v);
        } break;
        case 9: {
            rgblight_sethsv(0, 0, rgb.v);
        } break;
        case 0: {
            rgblight_set_speed(255);
        } break;
        default: {
            rgblight_set_speed(200);
        } break;
    }
}

uint32_t hs_ct_time;
RGB      rgb_test_open;
bool     process_record_kb(uint16_t keycode, keyrecord_t *record) {

    if (process_record_user_split65(keycode, record) != true) {
        return false;
    }

#ifdef WIRELESS_ENABLE
    if (process_record_wls(keycode, record) != true) {
        return false;
    }
#endif
    switch (keycode) {
        case KC_F1: {
            if (confinfo.filp) {
                if (keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code16(KC_MSEL);
                    } else {
                        unregister_code16(KC_MSEL);
                    }
                    return false;
                }
            }
            return true;
        } break;
        case KC_F2: {
            if (confinfo.filp) {
                if (keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code16(KC_VOLD);
                    } else {
                        unregister_code16(KC_VOLD);
                    }
                    return false;
                }
            }
            return true;
        } break;
        case KC_F3: {
            if (confinfo.filp) {
                if (keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code16(KC_VOLU);
                    } else {
                        unregister_code16(KC_VOLU);
                    }
                    return false;
                }
            }
            return true;
        } break;
        case KC_F4: {
            if (confinfo.filp) {
                if (keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code16(KC_MUTE);
                    } else {
                        unregister_code16(KC_MUTE);
                    }
                    return false;
                }
            }
            return true;
        } break;
        case KC_F5: {
            if (confinfo.filp) {
                if (keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code16(KC_MSTP);
                    } else {
                        unregister_code16(KC_MSTP);
                    }
                    return false;
                }
            }
            return true;
        } break;
        case KC_F6: {
            if (confinfo.filp) {
                if (keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code16(KC_MPRV);
                    } else {
                        unregister_code16(KC_MPRV);
                    }
                    return false;
                }
            }
            return true;
        } break;
        case KC_F7: {
            if (confinfo.filp) {
                if (keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code16(KC_MPLY);
                    } else {
                        unregister_code16(KC_MPLY);
                    }
                    return false;
                }
            }
            return true;
        } break;
        case KC_F8: {
            if (confinfo.filp) {
                if (keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code16(KC_MNXT);
                    } else {
                        unregister_code16(KC_MNXT);
                    }
                    return false;
                }
            }
            return true;
        } break;
        case KC_F9: {
            if (confinfo.filp) {
                if (keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code16(KC_MAIL);
                    } else {
                        unregister_code16(KC_MAIL);
                    }
                    return false;
                }
            }
            return true;
        } break;
        case KC_F10: {
            if (confinfo.filp) {
                if (keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code16(KC_WHOM);
                    } else {
                        unregister_code16(KC_WHOM);
                    }
                    return false;
                }
            }
            return true;
        } break;
        case KC_F11: {
            if (confinfo.filp) {
                if (keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code16(KC_CALC);
                    } else {
                        unregister_code16(KC_CALC);
                    }
                    return false;
                }
            }
            return true;
        } break;
        case KC_F12: {
            if (confinfo.filp) {
                if (keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code16(KC_WSCH);
                    } else {
                        unregister_code16(KC_WSCH);
                    }
                    return false;
                }
            }
            return true;
        } break;
        case KC_1: {
            if (confinfo.filp) {
                if (!keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code(KC_F1);
                    } else {
                        unregister_code(KC_F1);
                    }
                } else {
                    if (record->event.pressed) {
                        register_code(KC_BRID);
                    } else {
                        unregister_code(KC_BRID);
                    }
                }
                return false;
            }
            return true;
        } break;
        case KC_2: {
            if (confinfo.filp) {
                if (!keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code(KC_F2);
                    } else {
                        unregister_code(KC_F2);
                    }
                } else {
                    if (record->event.pressed) {
                        register_code(KC_BRIU);
                    } else {
                        unregister_code(KC_BRIU);
                    }
                }
                return false;
            }
            return true;
        } break;
        case KC_3: {
            if (confinfo.filp) {
                if (!keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code(KC_F3);
                    } else {
                        unregister_code(KC_F3);
                    }
                } else {
                    if (record->event.pressed) {
                        register_code(KC_LGUI);
                        register_code(KC_TAB);
                    } else {
                        unregister_code(KC_LGUI);
                        unregister_code(KC_TAB);
                    }
                }
                return false;
            }
            return true;
        } break;
        case KC_4: {
            if (confinfo.filp) {
                if (!keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code(KC_F4);
                    } else {
                        unregister_code(KC_F4);
                    }
                } else {
                    if (record->event.pressed) {
                        register_code(KC_LGUI);
                        register_code(KC_E);
                    } else {
                        unregister_code(KC_LGUI);
                        unregister_code(KC_E);
                    }
                }
                return false;
            }
            return true;
        } break;
        case KC_5: {
            if (confinfo.filp) {
                if (!keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code(KC_F5);
                    } else {
                        unregister_code(KC_F5);
                    }
                } else {
                    if (record->event.pressed) {
                        rgb_matrix_decrease_val();
                    }
                }
                return false;
            }
            return true;
        } break;
        case KC_6: {
            if (confinfo.filp) {
                if (!keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code(KC_F6);
                    } else {
                        unregister_code(KC_F6);
                    }
                } else {
                    if (record->event.pressed) {
                        rgb_matrix_increase_val();
                    }
                }
                return false;
            }
            return true;
        } break;
        case KC_7: {
            if (confinfo.filp) {
                if (!keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code(KC_F7);
                    } else {
                        unregister_code(KC_F7);
                    }
                } else {
                    if (record->event.pressed) {
                        register_code(KC_MPRV);
                    } else {
                        unregister_code(KC_MPRV);
                    }
                }
                return false;
            }
            return true;
        } break;
        case KC_8: {
            if (confinfo.filp) {
                if (!keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code(KC_F8);
                    } else {
                        unregister_code(KC_F8);
                    }
                } else {
                    if (record->event.pressed) {
                        register_code(KC_MPLY);
                    } else {
                        unregister_code(KC_MPLY);
                    }
                }
                return false;
            }
            return true;
        } break;
        case KC_9: {
            if (confinfo.filp) {
                if (!keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code(KC_F9);
                    } else {
                        unregister_code(KC_F9);
                    }
                } else {
                    if (record->event.pressed) {
                        register_code(KC_MNXT);
                    } else {
                        unregister_code(KC_MNXT);
                    }
                }
                return false;
            }
            return true;
        } break;
        case KC_0: {
            if (confinfo.filp) {
                if (!keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code(KC_F10);
                    } else {
                        unregister_code(KC_F10);
                    }
                } else {
                    if (record->event.pressed) {
                        register_code(KC_MUTE);
                    } else {
                        unregister_code(KC_MUTE);
                    }
                }
                return false;
            }
            return true;
        } break;
        case KC_MINS: {
            if (confinfo.filp) {
                if (!keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code(KC_F11);
                    } else {
                        unregister_code(KC_F11);
                    }
                } else {
                    if (record->event.pressed) {
                        register_code(KC_VOLD);
                    } else {
                        unregister_code(KC_VOLD);
                    }
                }
                return false;
            }
            return true;
        } break;
        case KC_EQL: {
            if (confinfo.filp) {
                if (!keymap_is_mac_system()) {
                    if (record->event.pressed) {
                        register_code(KC_F12);
                    } else {
                        unregister_code(KC_F12);
                    }
                } else {
                    if (record->event.pressed) {
                        register_code(KC_VOLU);
                    } else {
                        unregister_code(KC_VOLU);
                    }
                }
                return false;
            }
            return true;
        } break;
        case KC_FILP: {
            if (record->event.pressed) {
                confinfo.filp = !confinfo.filp;
                eeconfig_confinfo_update(confinfo.raw);
            }
            return false;
        } break;
        case KC_BATQ: {
            if (record->event.pressed) {
                im_bat_req_charging_flag = true;
            } else {
                im_bat_req_charging_flag = false;
            }
        } break;
        case QK_BOOT: {
            if (record->event.pressed) {
                dprintf("into boot!!!\r\n");
                eeconfig_disable();
                bootloader_jump();
            }
        } break;

        case NK_TOGG: {
            if (rgbrec_is_started()) {
                return false;
            }
            if (record->event.pressed) {
                rgb_matrix_hs_indicator_set(0xFF, (RGB){0x00, 0x6E, 0x00}, 250, 1);
            }
        } break;
        case RL_MOD: {
            if (rgbrec_is_started()) {
                return false;
            }
            if (record->event.pressed) {
                im_rgblight_increase();
            }

            return false;
        } break;
        case EE_CLR: {
            if (record->event.pressed) {
                ee_clr_timer = timer_read32();
            } else {
                ee_clr_timer = 0;
            }

            return false;
        } break;
        case RM_SPDU: {
            if (record->event.pressed) {
                if (rgb_matrix_get_speed() >= 215) {
                    rgb_blink_dir();
                }
            }
        } break;
        case RM_SPDD: {
            if (record->event.pressed) {
                if (rgb_matrix_get_speed() <= 95) {
                    rgb_blink_dir();
                }
            }
        } break;
        case RM_VALU: {
            if (record->event.pressed) {
                rgb_matrix_enable();
                if (is_keyboard_master()) {
                    gpio_write_pin_high(LED_POWER_EN_PIN);
                    gpio_write_pin_high(LED_POWER_EN2_PIN);
                }
                if (rgb_matrix_get_speed() >= 120) {
                    rgb_blink_dir();
                }
            }
        } break;
        case RM_VALD: {
            if (record->event.pressed) {
                if (rgb_matrix_get_val() <= RGB_MATRIX_VAL_STEP) {
                    if (is_keyboard_master()) {
                        gpio_write_pin_low(LED_POWER_EN_PIN);
                        gpio_write_pin_low(LED_POWER_EN2_PIN);
                    }
                    for (uint8_t i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
                        rgb_matrix_set_color(i, 0, 0, 0);
                    }
                }
                if (rgb_matrix_get_speed() <= 30) {
                    rgb_blink_dir();
                }
            }
        } break;

        case TO(_BL): {
            if (record->event.pressed) {
                rgb_matrix_hs_set_remain_time(HS_RGB_BLINK_INDEX_MAC, 0);
                rgb_matrix_hs_indicator_set(HS_RGB_BLINK_INDEX_WIN, (RGB){RGB_WHITE}, 250, 3);
                if (keymap_is_mac_system()) {
                    set_single_persistent_default_layer(_BL);
                    layer_move(0);
                }
            }

            return false;
        } break;
        case TO(_MBL): {
            if (record->event.pressed) {
                rgb_matrix_hs_set_remain_time(HS_RGB_BLINK_INDEX_WIN, 0);
                rgb_matrix_hs_indicator_set(HS_RGB_BLINK_INDEX_MAC, (RGB){RGB_WHITE}, 250, 3);
                if (!keymap_is_mac_system()) {
                    set_single_persistent_default_layer(_MBL);
                    layer_move(0);
                }
            }

            return false;
        } break;

        case RM_NEXT: {
            if (record->event.pressed) {
                uint8_t mode = rgb_matrix_get_mode() + 1;
                if (mode >= RGB_MATRIX_EFFECT_MAX) {
                    mode = 1;
                }
                // RGBR_PLAY is only meant to be reached via the RGB record/playback
                // feature, not the normal RM_NEXT cycle, so skip over it here.
                if (mode == RGB_MATRIX_CUSTOM_RGBR_PLAY) {
                    mode++;
                    if (mode >= RGB_MATRIX_EFFECT_MAX) {
                        mode = 1;
                    }
                }
                rgb_matrix_mode(mode);
            }
            return false;
        } break;
        case KC_LCMD: {
            if (keymap_is_mac_system()) {
                if (keymap_config.no_gui && !rgbrec_is_started()) {
                    if (record->event.pressed) {
                        register_code16(KC_LCMD);
                    } else {
                        unregister_code16(KC_LCMD);
                    }
                }
            }

            return true;
        } break;
        case KC_RCMD: {
            if (keymap_is_mac_system()) {
                if (keymap_config.no_gui && !rgbrec_is_started()) {
                    if (record->event.pressed) {
                        register_code16(KC_RCMD);
                    } else {
                        unregister_code16(KC_RCMD);
                    }
                }
            }

            return true;
        } break;
#ifdef WIRELESS_ENABLE
        case HS_BATQ: {
            extern bool rk_bat_req_flag;
            rk_bat_req_flag = (confinfo.devs != DEVS_USB) && record->event.pressed;
            return false;
        } break;
#endif

        default:
            break;
    }

    return true;
}

void housekeeping_task_kb(void) { // loop

    // Came from wireless.c
    if (wireless_get_current_devs() == DEVS_USB && im_test_rate_flag) usb_mode_test_report_task();
        wireless_task();

    // Runs outside the raw HID callback so a busy split bus cannot stall the
    // host transfer.
    strip_sync_push();

#ifdef WIRELESS_ENABLE
    uint8_t         hs_now_mode;
    static uint32_t hs_current_time;

    charging_state = gpio_read_pin(HS_BAT_CABLE_PIN);

    bat_full_flag = gpio_read_pin(BAT_FULL_PIN);

    if (charging_state && (bat_full_flag)) {
        hs_now_mode = MD_SND_CMD_DEVCTRL_CHARGING_DONE;
    } else if (charging_state) {
        hs_now_mode = MD_SND_CMD_DEVCTRL_CHARGING;
    } else {
        hs_now_mode = MD_SND_CMD_DEVCTRL_CHARGING_STOP;
    }

    if (!hs_current_time || timer_elapsed32(hs_current_time) > 1000) {
        hs_current_time = timer_read32();
        md_send_devctrl(hs_now_mode);
        md_send_devctrl(MD_SND_CMD_DEVCTRL_INQVOL);
    }
#endif

    if (is_keyboard_master()) {
        static uint32_t last_sync = 0;
        if (timer_elapsed32(last_sync) > 2000) {
            last_sync = timer_read32();
#ifdef WIRELESS_ENABLE
            pov = *md_getp_bat();
            master_sync_mms_slave(wireless_get_current_devs(), wireless_get_current_devs(), pov);
#endif
        }
    }

    // Call the user-level function at the end
    housekeeping_task_user();
}

#ifdef RGB_MATRIX_ENABLE

#    ifdef WIRELESS_ENABLE
bool     wls_rgb_indicator_reset    = false;
uint32_t wls_rgb_indicator_timer    = 0x00;
uint32_t wls_rgb_indicator_interval = 0;
uint32_t wls_rgb_indicator_times    = 0;
uint32_t wls_rgb_indicator_index    = 0;
RGB      wls_rgb_indicator_rgb      = {0};

void rgb_matrix_wls_indicator_set(uint8_t index, RGB rgb, uint32_t interval, uint8_t times) {
    wls_rgb_indicator_timer = timer_read32();

    wls_rgb_indicator_index    = index;
    wls_rgb_indicator_interval = interval;
    wls_rgb_indicator_times    = times * 2;
    wls_rgb_indicator_rgb      = rgb;
}

void wireless_devs_change_kb(uint8_t old_devs, uint8_t new_devs, bool reset) {
    wls_rgb_indicator_reset = reset;

    if (confinfo.devs != wireless_get_current_devs()) {
        confinfo.devs = wireless_get_current_devs();
        if (confinfo.devs > 0 && confinfo.devs < 4) confinfo.last_btdevs = confinfo.devs;
        eeconfig_confinfo_update(confinfo.raw);
    }

    switch (new_devs) {
        case DEVS_BT1: {
            if (reset) {
                rgb_matrix_wls_indicator_set(HS_RGB_BLINK_INDEX_BT1, (RGB){HS_LBACK_COLOR_BT1}, 200, 1);
            } else {
                rgb_matrix_wls_indicator_set(HS_RGB_BLINK_INDEX_BT1, (RGB){HS_PAIR_COLOR_BT1}, 500, 1);
            }
        } break;
        case DEVS_BT2: {
            if (reset) {
                rgb_matrix_wls_indicator_set(HS_RGB_BLINK_INDEX_BT2, (RGB){HS_LBACK_COLOR_BT2}, 200, 1);
            } else {
                rgb_matrix_wls_indicator_set(HS_RGB_BLINK_INDEX_BT2, (RGB){HS_PAIR_COLOR_BT2}, 500, 1);
            }
        } break;
        case DEVS_BT3: {
            if (reset) {
                rgb_matrix_wls_indicator_set(HS_RGB_BLINK_INDEX_BT3, (RGB){HS_LBACK_COLOR_BT3}, 200, 1);
            } else {
                rgb_matrix_wls_indicator_set(HS_RGB_BLINK_INDEX_BT3, (RGB){HS_PAIR_COLOR_BT3}, 500, 1);
            }
        } break;
        case DEVS_BT4: {
            if (reset) {
                rgb_matrix_wls_indicator_set(41, (RGB){RGB_BLUE}, 200, 1);
            } else {
                rgb_matrix_wls_indicator_set(41, (RGB){RGB_BLUE}, 500, 1);
            }
        } break;
        case DEVS_BT5: {
            if (reset) {
                rgb_matrix_wls_indicator_set(42, (RGB){RGB_BLUE}, 200, 1);
            } else {
                rgb_matrix_wls_indicator_set(42, (RGB){RGB_BLUE}, 500, 1);
            }
        } break;
        case DEVS_2G4: {
            if (reset) {
                rgb_matrix_wls_indicator_set(HS_RGB_BLINK_INDEX_2G4, (RGB){HS_LBACK_COLOR_2G4}, 200, 1);
            } else {
                rgb_matrix_wls_indicator_set(HS_RGB_BLINK_INDEX_2G4, (RGB){HS_LBACK_COLOR_2G4}, 500, 1);
            }
        } break;
        default:
            break;
    }
}

bool rgb_matrix_wls_indicator_cb(void) {
    if (*md_getp_state() != MD_STATE_CONNECTED) {
        wireless_devs_change_kb(wireless_get_current_devs(), wireless_get_current_devs(), wls_rgb_indicator_reset);
        return true;
    }

    // refresh led
    led_wakeup();

    return false;
}

void rgb_matrix_wls_indicator(void) {
    if (wls_rgb_indicator_timer) {
        if (timer_elapsed32(wls_rgb_indicator_timer) >= wls_rgb_indicator_interval) {
            wls_rgb_indicator_timer = timer_read32();

            if (wls_rgb_indicator_times) {
                wls_rgb_indicator_times--;
            }

            if (wls_rgb_indicator_times <= 0) {
                wls_rgb_indicator_timer = 0x00;
                if (rgb_matrix_wls_indicator_cb() != true) {
                    return;
                }
            }
        }

        if (wls_rgb_indicator_times % 2) {
            rgb_matrix_set_color(wls_rgb_indicator_index, wls_rgb_indicator_rgb.r, wls_rgb_indicator_rgb.g, wls_rgb_indicator_rgb.b);
        } else {
            rgb_matrix_set_color(wls_rgb_indicator_index, 0x00, 0x00, 0x00);
        }
    }
}

void rgb_matrix_hs_bat_set(uint8_t index, RGB rgb, uint32_t interval, uint8_t times) {
    for (int i = 0; i < HS_RGB_BAT_COUNT; i++) {
        if (!hs_rgb_bat[i].active) {
            hs_rgb_bat[i].active   = true;
            hs_rgb_bat[i].timer    = timer_read32();
            hs_rgb_bat[i].interval = interval;
            hs_rgb_bat[i].times    = times * 2;
            hs_rgb_bat[i].index    = index;
            hs_rgb_bat[i].rgb      = rgb;
            break;
        }
    }
}

void rgb_matrix_hs_bat(void) {
    for (int i = 0; i < HS_RGB_BAT_COUNT; i++) {
        if (hs_rgb_bat[i].active) {
            if (timer_elapsed32(hs_rgb_bat[i].timer) >= hs_rgb_bat[i].interval) {
                hs_rgb_bat[i].timer = timer_read32();

                if (hs_rgb_bat[i].times) {
                    hs_rgb_bat[i].times--;
                }

                if (hs_rgb_bat[i].times <= 0) {
                    hs_rgb_bat[i].active = false;
                    hs_rgb_bat[i].timer  = 0x00;
                }
            }

            if (hs_rgb_bat[i].times % 2) {
                rgb_matrix_set_color(hs_rgb_bat[i].index, hs_rgb_bat[i].rgb.r, hs_rgb_bat[i].rgb.g, hs_rgb_bat[i].rgb.b);
            } else {
                rgb_matrix_set_color(hs_rgb_bat[i].index, 0x00, 0x00, 0x00);
            }
        }
    }
}
bool temp,im_test_rate_flag;
void bat_indicators(void) {
    static uint32_t battery_process_time = 0;

    if (!is_keyboard_master())  {
        return;
    }

    if (charging_state && (bat_full_flag)) {
        battery_process_time = 0;
        if (im_bat_req_charging_flag) rgb_matrix_set_color(HS_MATRIX_BLINK_INDEX_BAT, 0xFF, 0x00, 0x00);
    } else if (charging_state) {
        battery_process_time = 0;
        if (im_bat_req_charging_flag) rgb_matrix_set_color(HS_MATRIX_BLINK_INDEX_BAT, 0x00, 0xFF, 0x00);
    } else if (*md_getp_bat() <= BATTERY_CAPACITY_LOW) {
        // was 250, changing to 30000 to blink every 30 seconds instead of 4 times a second
        rgb_matrix_hs_bat_set(HS_MATRIX_BLINK_INDEX_BAT, (RGB){0xFF, 0x00, 0x00}, 30000, 1);

        if (*md_getp_bat() <= BATTERY_CAPACITY_STOP) {
            if (!battery_process_time) {
                battery_process_time = timer_read32();
            }

            if (battery_process_time && timer_elapsed32(battery_process_time) > 60000) {
                battery_process_time = 0;
                lower_sleep          = true;
                lpwr_set_timeout_manual(true);
            }
        }
    } else {
        battery_process_time = 0;
    }
}

#    endif

#endif

void rgb_blink_dir(void) {
    rgb_matrix_hs_indicator_set(54, (RGB){0, 0, 0}, 250, 1);
    rgb_matrix_hs_indicator_set(66, (RGB){0, 0, 0}, 250, 1);
    rgb_matrix_hs_indicator_set(67, (RGB){0, 0, 0}, 250, 1);
    rgb_matrix_hs_indicator_set(65, (RGB){0, 0, 0}, 250, 1);
}

bool hs_reset_settings_user(void) {
    rgb_matrix_hs_indicator_set(0xFF, (RGB){0x10, 0x10, 0x10}, 250, 3);

    return true;
}

void nkr_indicators_hook(uint8_t index) {
    if ((hs_rgb_indicators[index].rgb.r == 0x6E) && (hs_rgb_indicators[index].rgb.g == 0x00) && (hs_rgb_indicators[index].rgb.b == 0x00)) {
        rgb_matrix_hs_indicator_set(0xFF, (RGB){0x6E, 0x00, 0x00}, 250, 1);

    } else if ((hs_rgb_indicators[index].rgb.r == 0x00) && (hs_rgb_indicators[index].rgb.g == 0x6E) && (hs_rgb_indicators[index].rgb.b == 0x00)) {
        rgb_matrix_hs_indicator_set(0xFF, (RGB){0x00, 0x00, 0x6F}, 250, 1);
    }
}

void rgb_matrix_hs_indicator_set(uint8_t index, RGB rgb, uint32_t interval, uint8_t times) {
    for (int i = 0; i < HS_RGB_INDICATOR_COUNT; i++) {
        if (!hs_rgb_indicators[i].active) {
            hs_rgb_indicators[i].active   = true;
            hs_rgb_indicators[i].timer    = timer_read32();
            hs_rgb_indicators[i].interval = interval;
            hs_rgb_indicators[i].times    = times * 2;
            hs_rgb_indicators[i].index    = index;
            hs_rgb_indicators[i].rgb      = rgb;
            if (index != 0xFF)
                hs_rgb_indicators[i].blink_cb = NULL;
            else {
                hs_rgb_indicators[i].blink_cb = nkr_indicators_hook;
            }
            break;
        }
    }
}

void rgb_matrix_hs_set_remain_time(uint8_t index, uint8_t remain_time) {
    for (int i = 0; i < HS_RGB_INDICATOR_COUNT; i++) {
        if (hs_rgb_indicators[i].index == index) {
            hs_rgb_indicators[i].times  = 0;
            hs_rgb_indicators[i].active = false;
            break;
        }
    }
}

void rgb_matrix_hs_indicator(void) {
    for (int i = 0; i < HS_RGB_INDICATOR_COUNT; i++) {
        if (hs_rgb_indicators[i].active) {
            if (timer_elapsed32(hs_rgb_indicators[i].timer) >= hs_rgb_indicators[i].interval) {
                hs_rgb_indicators[i].timer = timer_read32();

                if (hs_rgb_indicators[i].times) {
                    hs_rgb_indicators[i].times--;
                }

                if (hs_rgb_indicators[i].times <= 0) {
                    hs_rgb_indicators[i].active = false;
                    hs_rgb_indicators[i].timer  = 0x00;
                    if (hs_rgb_indicators[i].blink_cb != NULL) hs_rgb_indicators[i].blink_cb(i);
                    continue;
                }
            }

            if ((hs_rgb_indicators[i].times % 2)) {
                if (hs_rgb_indicators[i].index == 0xFF) {
                    rgb_matrix_set_color_all(hs_rgb_indicators[i].rgb.r, hs_rgb_indicators[i].rgb.g, hs_rgb_indicators[i].rgb.b);
                } else {
                    rgb_matrix_set_color(hs_rgb_indicators[i].index, hs_rgb_indicators[i].rgb.r, hs_rgb_indicators[i].rgb.g, hs_rgb_indicators[i].rgb.b);
                }
            } else {
                if (hs_rgb_indicators[i].index == 0xFF) {
                    rgb_matrix_set_color_all(0x00, 0x00, 0x00);
                } else {
                    rgb_matrix_set_color(hs_rgb_indicators[i].index, 0x00, 0x00, 0x00);
                }
            }
        }
    }
}

bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max) {
    if (test_white_light_flag) {
        RGB rgb_test_open = hsv_to_rgb((HSV){.h = 0, .s = 0, .v = RGB_MATRIX_VAL_STEP * 5});
        rgb_matrix_set_color_all(rgb_test_open.r, rgb_test_open.g, rgb_test_open.b);

        return false;
    }
#ifdef RGBLIGHT_ENABLE
    if (rgb_matrix_indicators_advanced_user(led_min, led_max) != true) {
        return false;
    }
#endif

    if (ee_clr_timer && timer_elapsed32(ee_clr_timer) > 3000) {
        hs_reset_settings();
        ee_clr_timer = 0;
    }

    if (host_keyboard_led_state().caps_lock) rgb_matrix_set_color(HS_RGB_INDEX_CAPS, 0x20, 0x20, 0x20);

    if (!keymap_is_mac_system() && keymap_config.no_gui) rgb_matrix_set_color(HS_RGB_INDEX_WIN_LOCK, 0x20, 0x20, 0x20);

#ifdef RGBLIGHT_ENABLE
    if (rgb_matrix_indicators_advanced_rgblight(led_min, led_max) != true) {
        return false;
    }
#endif

#ifdef WIRELESS_ENABLE
    rgb_matrix_wls_indicator();

    if (enable_bat_indicators && !inqbat_flag && !rgbrec_is_started()) {
        rgb_matrix_hs_bat();
        bat_indicators();
        bat_indicator_cnt = timer_read32();
    }

    if (!enable_bat_indicators) {
        if (timer_elapsed32(bat_indicator_cnt) > 2000) {
            enable_bat_indicators = true;
            bat_indicator_cnt     = timer_read32();
        }
    }

#endif

    rgb_matrix_hs_indicator();
    if (confinfo.filp) rgb_matrix_set_color(32, RGB_MATRIX_MAXIMUM_BRIGHTNESS, RGB_MATRIX_MAXIMUM_BRIGHTNESS, RGB_MATRIX_MAXIMUM_BRIGHTNESS);
    query();
    // Last, so host-driven strip colours win over the active effect. Running
    // here rather than in a custom effect is what lets the strip be driven
    // while the keys keep animating.
    strip_render(led_min, led_max);
    return true;
}

void hs_reset_settings(void) {
    if (is_keyboard_master()) {
        master_to_slave_t m2s = {0};
        slave_to_master_t s2m = {0};
        m2s.cmd               = 0xDD;
        if (transaction_rpc_exec(USER_SYNC_MMS, sizeof(m2s), &m2s, sizeof(s2m), &s2m)) {
            if (s2m.resp == 0x00) {
            }
            dprintf("Slave Sleep OK\n");
        } else {
            dprintf("Slave sync failed!\n");
        }
    }
    enable_bat_indicators = false;
    eeconfig_init();
    eeconfig_update_rgb_matrix_default();

#ifdef RGBLIGHT_ENABLE
    extern void rgblight_init(void);
    is_rgblight_initialized = false;
    rgblight_init();
    eeconfig_update_rgblight_default();
    rgblight_enable();
#endif

    eeconfig_read_keymap(&keymap_config);

#if defined(NKRO_ENABLE) && defined(FORCE_NKRO)
    keymap_config.nkro = 0;
    eeconfig_update_keymap(&keymap_config);
#endif

    // #if defined(WIRELESS_ENABLE)
    //     wireless_devs_change(wireless_get_current_devs(), DEVS_USB, false);
    // #endif

    if (hs_reset_settings_user() != true) {
        return;
    }
#ifdef WIRELESS_ENABLE
    hs_rgb_blink_set_timer(timer_read32());
#endif
    keyboard_post_init_kb();
}

void lpwr_wakeup_hook(void) {
#ifdef WIRELESS_ENABLE
    hs_mode_scan(false, confinfo.devs, confinfo.last_btdevs);
#endif

    // Delay to ensure split communication is ready before LED power changes
    // Prevents flickering on the slave/right half during wake from deep sleep
    wait_ms(50);

    if (is_keyboard_master()) {
        // Master controls its LED pins via LED_POWER_EN defines
        if (rgb_matrix_get_val() != 0) {
            gpio_write_pin_high(LED_POWER_EN_PIN);
            gpio_write_pin_high(LED_POWER_EN2_PIN);
        } else {
            gpio_write_pin_low(LED_POWER_EN_PIN);
            gpio_write_pin_low(LED_POWER_EN2_PIN);
        }
    } else {
        // Slave controls A5/A8 directly (not via LED_POWER_EN defines)
        if (rgb_matrix_get_val() != 0) {
            gpio_write_pin_high(A5);
            gpio_write_pin_high(A8);
        } else {
            gpio_write_pin_low(A5);
            gpio_write_pin_low(A8);
        }
    }
}

void user_sync_mms_slave_handler(uint8_t in_buflen, const void* in_data, uint8_t out_buflen, void* out_data){
    const master_to_slave_t *m2s = (const master_to_slave_t*)in_data;
    slave_to_master_t *s2m = (slave_to_master_t*)out_data;

    switch(m2s->cmd)
    {
        case 0x55:  //sync multimode
            wireless_devs_change(m2s->body[0], m2s->body[1], false);
            s2m->resp = 0x00;
        break;
        case 0xAA:
            // Open-drain switch deferred to lpwr_stop_hook_pre() so it
            // doesn't run before this half's own 0xBB power-down handling.
            s2m->resp = 0x00;
            lower_sleep = m2s->body[0];
            lpwr_set_timeout_manual(true);
            break;
        case 0xBB:

            gpio_write_pin_low(A5);
            gpio_write_pin_low(A8);
            s2m->resp = 0x00;
            break;
        case 0xCC:
            gpio_write_pin_high(A5);
            gpio_write_pin_high(A8);
            s2m->resp = 0x00;
            break;
        case 0xDD:
            hs_reset_settings();
            s2m->resp = 0x00;
            break;
        case 0xEE: // strip frame for this half's LEDs (indices 36-38)
            strip_app_mode = m2s->body[0] ? true : false;
            for (uint8_t i = 0; i < (STRIP_LED_COUNT - STRIP_RIGHT_FIRST); i++) {
                strip_rgb[STRIP_RIGHT_FIRST + i][0] = m2s->body[1 + (i * 3) + 0];
                strip_rgb[STRIP_RIGHT_FIRST + i][1] = m2s->body[1 + (i * 3) + 1];
                strip_rgb[STRIP_RIGHT_FIRST + i][2] = m2s->body[1 + (i * 3) + 2];
            }
            // Keep the slave's own watchdog check (see strip_app_active())
            // in agreement with whatever the host set on the master -- it
            // never sees the host directly, so without this it reverts on
            // its own independent default schedule.
            strip_cfg.watchdog_ms = (uint16_t)((m2s->body[10] << 8) | m2s->body[11]);
            // The slave never sees the host, so keep its watchdog fed from the
            // arrival of sync traffic instead.
            strip_last_frame = timer_read32();
            s2m->resp        = 0x00;
            break;
        case 0xEF: // strip config: fallback colour + boot-on-power default
            strip_cfg.h        = m2s->body[0];
            strip_cfg.s        = m2s->body[1];
            strip_cfg.v        = m2s->body[2];
            strip_cfg.boot_app = m2s->body[3];
            s2m->resp          = 0x00;
            break;
        default :break;
    }
}
