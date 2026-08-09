# Epomaker Split70

- Keyboard Maintainer: [Epomaker](https://github.com/Epomaker)
- Hardware Supported: Epomaker Split70
- Hardware Availability: Epomaker Split70

> ⚠️ **Personal firmware.** This directory is customized for my own Split70 and
> deviates from both the stock Epomaker firmware and upstream QMK. Most RGB
> animations are disabled on purpose and a custom strip-only lighting effect is
> the default. See [Personal customizations](#personal-customizations) below
> before flashing this onto someone else's board.

Make example for this keyboard (after setting up your build environment):

    make epomaker/split70:default

Flashing example for this keyboard:

    make epomaker/split70:default

## Flashing the Firmware

**HOW TO ENTER DFU(FIRMWARE UPDATE)MODE**
A：The Split70 is a split keyboard with two halves — the left half (with the knob) and the right half (with arrow keys).

**Each half must be entered into DFU mode separately.**

● **Left Half (with knob):**

1. Disconnect all cables from the keyboard.
2. Hold ESC and plug in USB-C.
3. "Device Connected" shows on the QMK Toolbox

● **Right Half(with arrow keys):**

1. Disconnect all cables from the keyboard.
2. Remove ALT and FN Keycaps and Flip the toggle switch between them down.
   ![Remove Keys](keys.png)
3. Remove Right Spacebar keycap and switch,short-circuit PCB holes with tweezers,then plug in USB-C.
   ![Short Circuit PCB Holes](short_pins.png)
4. "Device Connected"shows on the QMK Toolbox
5. After flashing,flip ALT/FN toggle back up.

**After flashing this firmware, you can do this by holding the "7" key while plugging the USB cable instead of shorting the holes. Flipping the switch to the down position is still required to flash and return to up position after flashing.
**

● Please reset the keyboard after flashing is completed.

‼️ Notes:
When updating or flashing the keyboard, MAKE SURE ONLY ONE KEYBOARDIS CONNECTED TO THE DEVICE!
When updating or flashing the keyboard, DON'T MOVE THE KEYBOARD or PRESS ANY KEYS!

The bootloader is `wb32-dfu`, so flashing goes through `wb32-dfu-updater_cli`
rather than `dfu-util`. That tool must be installed and on `PATH`; `make flash`
polls for a DFU device and waits until one appears.

See the [build environment setup](https://docs.qmk.fm/#/getting_started_build_tools) and the [make instructions](https://docs.qmk.fm/#/getting_started_make_guide) for more information. Brand new to QMK? Start with our [Complete Newbs Guide](https://docs.qmk.fm/#/newbs).

## Personal customizations

Everything in this section is specific to this fork. Files referenced are
relative to `keyboards/epomaker/split70/`.

### Strip-only static lighting (`STRIP_STATIC`)

`rgb_matrix_user.inc` adds a custom RGB Matrix effect named `STRIP_STATIC`. It
lights only the six accent-strip LEDs — indices `0-2` (left half) and `36-38`
(right half) in the `rgb_matrix.layout` of `keyboard.json` — in the current
RGB Matrix HSV colour, and forces every key LED off.

Those six slots all map to matrix position `[0, 0]` because they are strip LEDs
not tied to any physical key, which is why the effect selects them by LED index
rather than by matrix position.

Because the effect reads `rgb_matrix_config.hsv` on every frame, the hue,
saturation, brightness and toggle keycodes (`RM_HUEU/D`, `RM_SATU/D`,
`RM_VALU/D`, `RM_TOGG`) all work while it is active.

### Reduced animation list and new default

In `keyboard.json`, every stock RGB Matrix animation is set to `false`. What
remains available at runtime is:

| Mode | Notes |
| --- | --- |
| `SOLID_COLOR` | Always compiled in by QMK; lights the whole keyboard |
| `RGBR_PLAY` | Per-key recorded lighting; reachable only via the RGB record feature |
| `STRIP_STATIC` | Custom strip-only effect described above |

The default animation is set to `custom_strip_static`
(`rgb_matrix.default.animation` in `keyboard.json`), with `speed: 204` and
`val: 80`. Codegen turns that into `RGB_MATRIX_DEFAULT_MODE
RGB_MATRIX_CUSTOM_STRIP_STATIC`.

In practice this means **Fn + Tab** (`RM_NEXT`) toggles between full-keyboard
`SOLID_COLOR` and strip-only `STRIP_STATIC`.

### `RM_NEXT` cycling fix

`process_record_kb` in `split70.c` used to hardcode mode numbers (`if (mode ==
29) rgb_matrix_mode(31)`), which broke as soon as the enabled-animation list
changed — the RGB Matrix effect enum is generated from the `.inc` files, so
those indices are not stable. Symptom was Fn + Tab needing two presses to
change anything.

It now advances the mode by one, wraps at `RGB_MATRIX_EFFECT_MAX` back to `1`,
and skips `RGB_MATRIX_CUSTOM_RGBR_PLAY` since that mode is only meant to be
entered through the RGB record/playback feature.

### Low-power sleep in wired mode

`lpwr_is_allow_timeout_hook` in `split70.c` blocks low-power sleep for **both**
halves whenever `wireless_get_current_devs() == DEVS_USB`. The check is
deliberately *not* gated on `is_keyboard_master()`: the right half is not the
USB host, so a master-only guard let it time out on its own after ~5 minutes of
no local keypress. On such an uncoordinated sleep it arms only the cable-insert
wake source, so key presses cannot wake it and the half appears frozen until
the cable is replugged.

### Known limitation: `HS_BATQ` on USB

`HS_BATQ` (Fn + B) shows the battery level on the LEDs. Its handler sets
`rk_bat_req_flag` only when `confinfo.devs != DEVS_USB`, so it is a silent
no-op over the cable — battery data comes from the wireless module, which is
not reporting in wired mode. This is accepted behaviour, not a bug to fix.

### Keymap notes

`keymaps/default/` builds with `VIA_ENABLE` and `ENCODER_MAP_ENABLE`. Layers:

| Layer | Purpose |
| --- | --- |
| `_BL` | Base (Windows/Linux), Ctrl-based cut/copy/paste macro column |
| `_FL` | Function layer for `_BL`; RGB, Bluetooth/2.4G/USB switching, `QK_BOOT`, `EE_CLR` |
| `_MBL` | Base (macOS), Cmd-based macro column, Alt/GUI swapped |
| `_MFL` | Function layer for `_MBL` |

`TO(_MBL)` / `TO(_BL)` on the function layers switch between the Windows and
macOS bases. All RGB control keycodes live on the function layers only. The
encoder is mapped to volume down/up on every layer.
