/*
    Copyright 2026 Daniel Balsom

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the “Software”),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "tusb.h"

#if __has_include("bsp/board_api.h")
#include "bsp/board_api.h"
#else
#include "bsp/board.h"
#endif

#ifndef PCJR_FUNCTION_USB_LOGO_FALLBACK
#define PCJR_FUNCTION_USB_LOGO_FALLBACK 1
#endif

// GPIO pin to use for IR strobe, active-low.
enum {
  IR_RX_PIN = 3,
  // How many bitcells per PCjr scancode - 1 start bit + 8 scancode bits + 1 parity bit = 10 bitcells
  PCJR_BIT_CELLS = 10,
  // Approx ~440us per bitcell, or 4.4ms per scancode
  PCJR_BIT_CELL_US = 440,
  PCJR_HALF_CELL_US = PCJR_BIT_CELL_US / 2,
  PCJR_FRAME_GAP_US = 2500,
  // Start bit value is always 1
  PCJR_DATA_START_CELL = 1,
  // Parity bit is always bitcell 9 (final bitcell)
  PCJR_PARITY_CELL = 9,
  PCJR_ALT_SCANCODE = 0x38,
  PCJR_FUNCTION_SCANCODE = 0x54,
  PCJR_SCANCODE_COUNT = 0x80,

  // Maximum pulse count before we give up decoding
  IR_MAX_PULSES = 24,
  IR_MIN_PULSE_SPACING_US = 80,
  PCJR_PULSE_TIMING_TOLERANCE_US = 170,
  PCJR_PULSE_PHASE_OFFSET_US = 0,

  // How long to pulse the on-board status LED when indicating a status.
  STATUS_LED_PULSE_MS = 80,

  // Serial console layout constants.
  SERIAL_STATUS_ROWS = 7,
  SERIAL_SCROLL_TOP_ROW = SERIAL_STATUS_ROWS + 1,
  SERIAL_WRITE_TIMEOUT_MS = 250,
};

static const char ANSI_COLOR_RESET[] = "\x1b[0m";
static const char ANSI_LIGHT_BLUE[] = "\x1b[94m";

#ifndef KEYBOARD_MODIFIER_LEFTCTRL
#define KEYBOARD_MODIFIER_LEFTCTRL 0x01
#define KEYBOARD_MODIFIER_LEFTSHIFT 0x02
#define KEYBOARD_MODIFIER_LEFTALT 0x04
#define KEYBOARD_MODIFIER_LEFTGUI 0x08
#define KEYBOARD_MODIFIER_RIGHTSHIFT 0x20
#endif

enum {
  USB_MOD_CTRL = KEYBOARD_MODIFIER_LEFTCTRL,
  USB_MOD_SHIFT = KEYBOARD_MODIFIER_LEFTSHIFT,
  USB_MOD_ALT = KEYBOARD_MODIFIER_LEFTALT,
  USB_MOD_LOGO = KEYBOARD_MODIFIER_LEFTGUI,
  USB_MOD_RSHIFT = KEYBOARD_MODIFIER_RIGHTSHIFT,
};

static volatile uint32_t ir_frame_start_us = 0;
static volatile bool ir_frame_pending = false;
static volatile bool ir_receiver_armed = true;
static volatile uint16_t ir_pulse_time_us[IR_MAX_PULSES];
static volatile uint8_t ir_pulse_count = 0;
static volatile uint32_t ir_last_pulse_us = 0;
static volatile bool ir_pulse_overflow = false;
static volatile uint32_t ir_falling_edge_count = 0;

static bool serial_was_connected = false;
static uint32_t status_led_off_at_ms = 0;
static bool status_led_active = false;
static uint8_t current_usb_modifiers = 0;
static bool pcjr_function_down = false;
static bool pcjr_function_used_in_combo = false;
static bool pcjr_function_used_as_logo_modifier = false;
static bool pcjr_key_down[PCJR_SCANCODE_COUNT];
static bool hid_release_pending = false;

struct pcjr_key_mapping {
  uint8_t pcjr_scancode;
  const char *name;
  uint8_t usb_usage;
  uint8_t usb_modifier;
};

// clang-format off
static const struct pcjr_key_mapping PCJR_KEYMAP[] = {
    {0x01, "Escape",       0x29, 0},
    {0x02, "1",            0x1E, 0},
    {0x03, "2",            0x1F, 0},
    {0x04, "3",            0x20, 0},
    {0x05, "4",            0x21, 0},
    {0x06, "5",            0x22, 0},
    {0x07, "6",            0x23, 0},
    {0x08, "7",            0x24, 0},
    {0x09, "8",            0x25, 0},
    {0x0A, "9",            0x26, 0},
    {0x0B, "0",            0x27, 0},
    {0x0C, "Minus",        0x2D, 0},
    {0x0D, "Equal",        0x2E, 0},
    {0x0E, "Backspace",    0x2A, 0},
    {0x0F, "Tab",          0x2B, 0},
    {0x10, "Q",            0x14, 0},
    {0x11, "W",            0x1A, 0},
    {0x12, "E",            0x08, 0},
    {0x13, "R",            0x15, 0},
    {0x14, "T",            0x17, 0},
    {0x15, "Y",            0x1C, 0},
    {0x16, "U",            0x18, 0},
    {0x17, "I",            0x0C, 0},
    {0x18, "O",            0x12, 0},
    {0x19, "P",            0x13, 0},
    {0x1A, "BracketLeft",  0x2F, 0},
    {0x1B, "BracketRight", 0x30, 0},
    {0x1C, "Enter",        0x28, 0},
    {0x1D, "Control",      0,    USB_MOD_CTRL},
    {0x1E, "A",            0x04, 0},
    {0x1F, "S",            0x16, 0},
    {0x20, "D",            0x07, 0},
    {0x21, "F",            0x09, 0},
    {0x22, "G",            0x0A, 0},
    {0x23, "H",            0x0B, 0},
    {0x24, "J",            0x0D, 0},
    {0x25, "K",            0x0E, 0},
    {0x26, "L",            0x0F, 0},
    {0x27, "Semicolon",    0x33, 0},
    {0x28, "Quote",        0x34, 0},
    {0x2A, "ShiftLeft",    0,    USB_MOD_SHIFT},
    {0x2C, "Z",            0x1D, 0},
    {0x2D, "X",            0x1B, 0},
    {0x2E, "C",            0x06, 0},
    {0x2F, "V",            0x19, 0},
    {0x30, "B",            0x05, 0},
    {0x31, "N",            0x11, 0},
    {0x32, "M",            0x10, 0},
    {0x33, "Comma",        0x36, 0},
    {0x34, "Period",       0x37, 0},
    {0x35, "Slash",        0x38, 0},
    {0x36, "ShiftRight",   0,    USB_MOD_RSHIFT},
    {0x37, "PrintScreen",  0x46, 0},
    {0x38, "Alt",          0,    USB_MOD_ALT},
    {0x39, "Space",        0x2C, 0},
    {0x3A, "CapsLock",     0x39, 0},
    {0x3B, "F1",           0x3A, 0},
    {0x3C, "F2",           0x3B, 0},
    {0x3D, "F3",           0x3C, 0},
    {0x3E, "F4",           0x3D, 0},
    {0x3F, "F5",           0x3E, 0},
    {0x40, "F6",           0x3F, 0},
    {0x41, "F7",           0x40, 0},
    {0x42, "F8",           0x41, 0},
    {0x43, "F9",           0x42, 0},
    {0x44, "F10",          0x43, 0},
    {0x45, "Pause",        0x48, 0},
    {0x46, "ScrollLock",   0x47, 0},
    {0x48, "ArrowUp",      0x52, 0},
    {0x4B, "ArrowLeft",    0x50, 0},
    {0x4D, "ArrowRight",   0x4F, 0},
    {0x50, "ArrowDown",    0x51, 0},
    {0x52, "Insert",       0x49, 0},
    {0x53, "Delete",       0x4C, 0},
    {0x54, "Function",     0,    USB_MOD_LOGO},
};
// clang-format on

static const size_t PCJR_KEYMAP_SIZE = sizeof(PCJR_KEYMAP) / sizeof(PCJR_KEYMAP[0]);

// clang-format off
static const struct pcjr_key_mapping PCJR_FUNCTION_KEYMAP[] = {
    {0x02, "F1",          0x3A, 0},
    {0x03, "F2",          0x3B, 0},
    {0x04, "F3",          0x3C, 0},
    {0x05, "F4",          0x3D, 0},
    {0x06, "F5",          0x3E, 0},
    {0x07, "F6",          0x3F, 0},
    {0x08, "F7",          0x40, 0},
    {0x09, "F8",          0x41, 0},
    {0x0A, "F9",          0x42, 0},
    {0x0B, "F10",         0x43, 0},
    {0x10, "Pause",       0x48, 0},
    {0x19, "PrintScreen", 0x46, 0},
    {0x1F, "ScrollLock",  0x47, 0},
    {0x48, "Home",        0x4A, 0},
    {0x4B, "PageUp",      0x4B, 0},
    {0x4D, "PageDown",    0x4E, 0},
    {0x50, "End",         0x4D, 0},
};
// clang-format on

static const size_t PCJR_FUNCTION_KEYMAP_SIZE = sizeof(PCJR_FUNCTION_KEYMAP) / sizeof(PCJR_FUNCTION_KEYMAP[0]);

// clang-format off
static const struct pcjr_key_mapping PCJR_ALT_KEYMAP[] = {
    {0x1A, "Pipe",      0x31, USB_MOD_SHIFT},
    {0x1B, "Tilde",     0x35, USB_MOD_SHIFT},
    {0x28, "Grave",     0x35, 0},
    {0x35, "Backslash", 0x31, 0},
};
// clang-format on

static const size_t PCJR_ALT_KEYMAP_SIZE = sizeof(PCJR_ALT_KEYMAP) / sizeof(PCJR_ALT_KEYMAP[0]);

static const struct pcjr_key_mapping *find_pcjr_key_mapping(uint8_t base_scancode) {
  for (size_t i = 0; i < PCJR_KEYMAP_SIZE; i++) {
    if (PCJR_KEYMAP[i].pcjr_scancode == base_scancode) {
      return &PCJR_KEYMAP[i];
    }
  }

  return NULL;
}

static const struct pcjr_key_mapping *find_pcjr_function_key_mapping(uint8_t base_scancode) {
  for (size_t i = 0; i < PCJR_FUNCTION_KEYMAP_SIZE; i++) {
    if (PCJR_FUNCTION_KEYMAP[i].pcjr_scancode == base_scancode) {
      return &PCJR_FUNCTION_KEYMAP[i];
    }
  }

  return NULL;
}

static const struct pcjr_key_mapping *find_pcjr_alt_key_mapping(uint8_t base_scancode) {
  for (size_t i = 0; i < PCJR_ALT_KEYMAP_SIZE; i++) {
    if (PCJR_ALT_KEYMAP[i].pcjr_scancode == base_scancode) {
      return &PCJR_ALT_KEYMAP[i];
    }
  }

  return NULL;
}

static void set_pcjr_key_state(uint8_t base_scancode, bool down) {
  if (base_scancode < PCJR_SCANCODE_COUNT && find_pcjr_key_mapping(base_scancode) != NULL) {
    pcjr_key_down[base_scancode] = down;
  }
}

static bool serial_connected(void) { return tud_cdc_connected(); }

static bool serial_write_bytes(const char *data, size_t length) {
  if (!serial_connected()) {
    return false;
  }

  absolute_time_t deadline = make_timeout_time_ms(SERIAL_WRITE_TIMEOUT_MS);
  while (length > 0 && serial_connected()) {
    uint32_t available = tud_cdc_write_available();
    if (available > 0) {
      uint32_t chunk = length < available ? (uint32_t)length : available;
      uint32_t written = tud_cdc_write(data, chunk);
      tud_cdc_write_flush();

      data += written;
      length -= written;
      deadline = make_timeout_time_ms(SERIAL_WRITE_TIMEOUT_MS);
      continue;
    }

    tud_task();
    tud_cdc_write_flush();
    if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
      return false;
    }
    sleep_ms(1);
  }

  return length == 0;
}

static void serial_write(const char *text) { serial_write_bytes(text, strlen(text)); }

static void serial_write_char(char ch) { serial_write_bytes(&ch, 1); }

// Serial printf helper.
static void serial_printf(const char *format, ...) {
  if (!serial_connected()) {
    return;
  }

  char buffer[192];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  serial_write(buffer);
}

static void status_led_off(void) {
#if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN >= 0
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
#endif
  status_led_active = false;
}

static void pulse_status_led(void) {
#if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN >= 0
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif
  status_led_off_at_ms = to_ms_since_boot(get_absolute_time()) + STATUS_LED_PULSE_MS;
  status_led_active = true;
}

// Determine if/when to turn off the status LED pulse.
static void handle_status_led(void) {
  if (status_led_active && (int32_t)(to_ms_since_boot(get_absolute_time()) - status_led_off_at_ms) >= 0) {
    status_led_off();
  }
}

static void print_hex_byte(uint8_t value) { serial_printf("%02X", value); }

static void serial_move_cursor(uint8_t row, uint8_t column) { serial_printf("\x1b[%u;%uH", row, column); }

// Print a keycap in our cool ASCII art keyboard display.
static void print_keycap(const char *label, uint8_t scancode) {
  bool pressed = scancode < PCJR_SCANCODE_COUNT && pcjr_key_down[scancode];

  if (pressed) {
    serial_write(ANSI_LIGHT_BLUE);
  }

  serial_write_char('[');
  serial_write(label);
  serial_write_char(']');

  if (pressed) {
    serial_write(ANSI_COLOR_RESET);
  }
}

static void print_keyboard_row_1(void) {
  serial_write("   | ");
  print_keycap("Esc", 0x01);
  print_keycap("1", 0x02);
  print_keycap("2", 0x03);
  print_keycap("3", 0x04);
  print_keycap("4", 0x05);
  print_keycap("5", 0x06);
  print_keycap("6", 0x07);
  print_keycap("7", 0x08);
  print_keycap("8", 0x09);
  print_keycap("9", 0x0A);
  print_keycap("0", 0x0B);
  print_keycap("-", 0x0C);
  print_keycap("+", 0x0D);
  print_keycap("BkSp", 0x0E);
  print_keycap(" Fn ", PCJR_FUNCTION_SCANCODE);
  serial_write_char('|');
}

static void print_keyboard_row_2(void) {
  serial_write("   | ");
  print_keycap("Tab ", 0x0F);
  print_keycap("Q", 0x10);
  print_keycap("W", 0x11);
  print_keycap("E", 0x12);
  print_keycap("R", 0x13);
  print_keycap("T", 0x14);
  print_keycap("Y", 0x15);
  print_keycap("U", 0x16);
  print_keycap("I", 0x17);
  print_keycap("O", 0x18);
  print_keycap("P", 0x19);
  print_keycap(" [", 0x1A);
  print_keycap(" ]", 0x1B);
  print_keycap("  ", 0xFF);
  serial_write("     |");
}

static void print_keyboard_row_3(void) {
  serial_write("   | ");
  print_keycap("Ctrl", 0x1D);
  print_keycap("A", 0x1E);
  print_keycap("S", 0x1F);
  print_keycap("D", 0x20);
  print_keycap("F", 0x21);
  print_keycap("G", 0x22);
  print_keycap("H", 0x23);
  print_keycap("J", 0x24);
  print_keycap("K", 0x25);
  print_keycap("L", 0x26);
  print_keycap(";:", 0x27);
  print_keycap("\"'", 0x28);
  print_keycap("Enter", 0x1C);
  print_keycap("^  ", 0x48);
  serial_write_char('|');
}

static void print_keyboard_row_4(void) {
  serial_write("   | ");
  print_keycap("Shift", 0x2A);
  print_keycap("Z", 0x2C);
  print_keycap("X", 0x2D);
  print_keycap("C", 0x2E);
  print_keycap("V", 0x2F);
  print_keycap("B", 0x30);
  print_keycap("N", 0x31);
  print_keycap("M", 0x32);
  print_keycap(",<", 0x33);
  print_keycap(".>", 0x34);
  print_keycap("/?", 0x35);
  print_keycap("Shift", 0x36);
  print_keycap("<", 0x4B);
  print_keycap(">", 0x4D);
  serial_write_char('|');
}

static void print_keyboard_row_5(void) {
  serial_write("   |           ");
  print_keycap("A", 0x38);
  print_keycap("________Spacebar______", 0x39);
  print_keycap("C", 0x3A);
  print_keycap("In", 0x52);
  print_keycap("D", 0x53);
  print_keycap("  v ", 0x50);
  serial_write_char('|');
}

// Draw our cool ASCII art keyboard status display.
static void render_keyboard_status(void) {
  if (!serial_connected()) {
    return;
  }

  serial_write("\x1b[s");
  for (uint8_t row = 1; row <= SERIAL_STATUS_ROWS; row++) {
    serial_move_cursor(row, 1);
    serial_write("\x1b[2K");
  }

  serial_move_cursor(1, 1);
  serial_write("    ______________________________________________________");
  serial_move_cursor(2, 1);
  print_keyboard_row_1();
  serial_move_cursor(3, 1);
  print_keyboard_row_2();
  serial_move_cursor(4, 1);
  print_keyboard_row_3();
  serial_move_cursor(5, 1);
  print_keyboard_row_4();
  serial_move_cursor(6, 1);
  print_keyboard_row_5();
  serial_move_cursor(7, 1);
  serial_write("   |______________________________________________________|");
  serial_write("\x1b[u");
}

static void configure_serial_screen(void) {
  serial_write("\x1b[r");
  serial_write("\x1b[2J");
  serial_write("\x1b[8;24r"); // rows 8-24 scroll; rows 1-7 stay fixed
  serial_move_cursor(SERIAL_SCROLL_TOP_ROW, 1);
  render_keyboard_status();
}

static void print_bit_buffer(const char bit_buffer[]) {
  serial_write(" bits ");
  for (uint8_t i = 0; i < PCJR_BIT_CELLS; i++) {
    serial_write_char(bit_buffer[i]);
  }
}

static bool release_hid_keyboard(void) {
  if (!hid_release_pending || !tud_hid_ready()) {
    return !hid_release_pending;
  }

  uint8_t empty_keys[6] = {0};
  tud_hid_keyboard_report(0, 0, empty_keys);
  hid_release_pending = false;
  return true;
}

static bool send_keyboard_tap(uint8_t usage, uint8_t modifiers) {
  if (hid_release_pending && !release_hid_keyboard()) {
    return false;
  }

  if (!tud_hid_ready()) {
    return false;
  }

  uint8_t keys[6] = {0};
  keys[0] = usage;
  tud_hid_keyboard_report(0, modifiers, keys);
  hid_release_pending = true;
  return true;
}

// Return true if the parity calculation is odd (correct parity for scancode)
static bool has_odd_parity(uint8_t value, uint8_t parity_bit) {
  uint8_t ones = parity_bit ? 1 : 0;

  for (uint8_t i = 0; i < 8; i++) {
    if (value & (1u << i)) {
      ones++;
    }
  }
  return (ones & 1u) == 1;
}

static bool nearest_pulse_distance(const uint16_t pulse_times[], uint8_t pulse_count, uint16_t target_us,
                                   uint16_t *nearest_distance_us) {
  if (pulse_count == 0) {
    return false;
  }

  // PCjr IR frames encode each bit as a 40KHz IR chirp located in one of the two halves of each bitcell.
  // This scans every captured falling edge and finds the one closest to the expected slot time for
  // the value we are testing.
  uint16_t nearest = 0xFFFF;

  for (uint8_t i = 0; i < pulse_count; i++) {
    uint16_t distance = pulse_times[i] > target_us ? pulse_times[i] - target_us : target_us - pulse_times[i];
    if (distance < nearest) {
      nearest = distance;
    }
  }

  // A slot is considered present only if some pulse landed close enough to the expected position.
  // We can use the distance to choose between competing "1" and "0" slots when it is ambiguous.
  *nearest_distance_us = nearest;
  return nearest <= PCJR_PULSE_TIMING_TOLERANCE_US;
}

// Helper to return a shifted and clamped pulse slot time.
static uint16_t pulse_slot_us(uint16_t base_us) {
  int32_t shifted = (int32_t)base_us + PCJR_PULSE_PHASE_OFFSET_US;

  if (shifted < 0) {
    return 0;
  }

  if (shifted > 0xFFFF) {
    return 0xFFFF;
  }

  return (uint16_t)shifted;
}

// Read a PCjr keyboard IR frame (scancode sending sequence of 10 bitcells)
static bool read_pcjr_frame(const uint16_t pulse_times[], uint8_t pulse_count, bool pulse_overflow, uint8_t *scancode,
                            uint16_t *raw_bits, char bit_buffer[]) {
  uint16_t bits = 0;
  bool frame_ok = !pulse_overflow;

  for (uint8_t i = 0; i < PCJR_BIT_CELLS; i++) {
    // Initialize bitcells to unknown / phase error state
    bit_buffer[i] = '?';
  }
  bit_buffer[PCJR_BIT_CELLS] = '\0';

  bits |= 1;
  // First bitcell is always 1 as start bit always starts frame
  bit_buffer[0] = '1';

  for (uint8_t cell = 1; cell < PCJR_BIT_CELLS; cell++) {
    uint16_t cell_start_us = cell * PCJR_BIT_CELL_US;
    uint16_t one_distance_us;
    uint16_t zero_distance_us;
    // In each data/parity cell, a pulse near the cell start means 1; a pulse near the half-cell
    // point means 0. Real captures drift a bit, so compare against both slots and keep whichever
    // timing is closer.
    bool near_one_slot =
        nearest_pulse_distance(pulse_times, pulse_count, pulse_slot_us(cell_start_us), &one_distance_us);
    bool near_zero_slot = nearest_pulse_distance(pulse_times, pulse_count,
                                                 pulse_slot_us(cell_start_us + PCJR_HALF_CELL_US), &zero_distance_us);

    if (!near_one_slot && !near_zero_slot) {
      // Situation remains ambiguous, hmm. Mark this as a phase error. Your PCjr would beep at you.
      bit_buffer[cell] = '?';
      frame_ok = false;
    }
    else if (near_one_slot && (!near_zero_slot || one_distance_us <= zero_distance_us)) {
      // Assume this is a '1' bit.
      bits |= (1u << cell);
      bit_buffer[cell] = '1';
    }
    else {
      // Assume this is a '0' bit.
      bit_buffer[cell] = '0';
    }
  }

  // Assemble bits into 10-bit word.
  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (bits & (1u << (PCJR_DATA_START_CELL + i))) {
      data |= (1u << i);
    }
  }
  uint8_t start_bit = bits & 1u;
  uint8_t parity_bit = (bits >> PCJR_PARITY_CELL) & 1u;

  // Perform parity check on frame.
  if (start_bit != 1 || !has_odd_parity(data, parity_bit)) {
    frame_ok = false;
  }

  *scancode = data;
  *raw_bits = bits;
  return frame_ok;
}

// Called if we had a frame decode failure; this will print some debug info.
static void handle_decode_error(const char bit_buffer[], uint8_t scancode, const uint16_t pulse_times[],
                                uint8_t pulse_count, bool pulse_overflow) {
  pulse_status_led();
  serial_write("IR frame ignored: decode/parity error");
  print_bit_buffer(bit_buffer);
  serial_write(" scancode candidate 0x");
  print_hex_byte(scancode);
  serial_printf(" pulses %u pulse_us", pulse_count);
  for (uint8_t i = 0; i < pulse_count; i++) {
    serial_printf("%s%u", i == 0 ? " " : ",", pulse_times[i]);
  }
  if (pulse_overflow) {
    serial_write(" overflow");
  }
  serial_write("\r\n");
}

// Called when we have a successful frame decode; will print scancode info.
static void handle_decoded_scancode(uint8_t scancode, const char bit_buffer[]) {
  bool released = (scancode & 0x80) != 0;
  uint8_t base_scancode = scancode & 0x7F;
  const struct pcjr_key_mapping *physical_key = find_pcjr_key_mapping(base_scancode);
  const struct pcjr_key_mapping *key = physical_key;
  bool pcjr_function_mapping = false;
  bool pcjr_alt_mapping = false;
  char key_name[11];
  char usb_info[33];
  char usb_info_raw[33];

  if (pcjr_function_down && base_scancode != PCJR_FUNCTION_SCANCODE) {
    const struct pcjr_key_mapping *function_key = find_pcjr_function_key_mapping(base_scancode);
    if (function_key != NULL) {
      pcjr_function_used_in_combo = true;
      pcjr_function_mapping = true;
      key = function_key;
    }
  }

  if (!pcjr_function_mapping && (current_usb_modifiers & USB_MOD_ALT) != 0 && base_scancode != PCJR_ALT_SCANCODE) {
    const struct pcjr_key_mapping *alt_key = find_pcjr_alt_key_mapping(base_scancode);
    if (alt_key != NULL) {
      pcjr_alt_mapping = true;
      key = alt_key;
    }
  }

  set_pcjr_key_state(base_scancode, !released);
  render_keyboard_status();
  pulse_status_led();

  snprintf(key_name, sizeof(key_name), "%-10.10s", key == NULL ? "??" : key->name);
  snprintf(usb_info, sizeof(usb_info), "%-32.32s", "");

  serial_write(released ? "[keyup  ]: " : "[keydown]: ");

  if (key == NULL) {
    print_hex_byte(base_scancode);
    serial_printf(" '%s' %s", key_name, usb_info);
    print_bit_buffer(bit_buffer);
    serial_write("\r\n");
    return;
  }

  serial_write("Scancode: ");
  print_hex_byte(base_scancode);
  serial_write("/");
  print_hex_byte(scancode);
  serial_printf(" '%s' ", key_name);

  if (base_scancode == PCJR_FUNCTION_SCANCODE) {
    if (released) {
      if (pcjr_function_down && !pcjr_function_used_in_combo && !pcjr_function_used_as_logo_modifier) {
        if (PCJR_FUNCTION_USB_LOGO_FALLBACK) {
          bool sent = send_keyboard_tap(0, USB_MOD_LOGO);
          snprintf(usb_info_raw, sizeof(usb_info_raw), sent ? "usb logo tap 0x%02X" : "usb busy logo tap",
                   USB_MOD_LOGO);
        }
        else {
          snprintf(usb_info_raw, sizeof(usb_info_raw), "pcjr function idle");
        }
      }
      else if (pcjr_function_used_as_logo_modifier) {
        snprintf(usb_info_raw, sizeof(usb_info_raw), "usb logo modifier used");
      }
      else {
        snprintf(usb_info_raw, sizeof(usb_info_raw), "pcjr function combo");
      }
      pcjr_function_down = false;
      pcjr_function_used_in_combo = false;
      pcjr_function_used_as_logo_modifier = false;
    }
    else {
      pcjr_function_down = true;
      pcjr_function_used_in_combo = false;
      pcjr_function_used_as_logo_modifier = false;
      if (PCJR_FUNCTION_USB_LOGO_FALLBACK) {
        snprintf(usb_info_raw, sizeof(usb_info_raw), "usb logo pending 0x%02X", USB_MOD_LOGO);
      }
      else {
        snprintf(usb_info_raw, sizeof(usb_info_raw), "pcjr function pending");
      }
    }

    snprintf(usb_info, sizeof(usb_info), "%-32.32s", usb_info_raw);
    serial_write(usb_info);
    print_bit_buffer(bit_buffer);
    serial_write("\r\n");
    return;
  }

  if (key->usb_modifier != 0 && !pcjr_alt_mapping) {
    if (released) {
      current_usb_modifiers &= ~key->usb_modifier;
    }
    else {
      current_usb_modifiers |= key->usb_modifier;
    }

    snprintf(usb_info_raw, sizeof(usb_info_raw), "usb modifier 0x%02X active 0x%02X", key->usb_modifier,
             current_usb_modifiers);
    snprintf(usb_info, sizeof(usb_info), "%-32.32s", usb_info_raw);
    serial_write(usb_info);
    print_bit_buffer(bit_buffer);
    serial_write("\r\n");
    return;
  }

  uint8_t active_usb_modifiers = current_usb_modifiers;
  if (PCJR_FUNCTION_USB_LOGO_FALLBACK) {
    if (pcjr_function_down && !pcjr_function_mapping) {
      active_usb_modifiers |= USB_MOD_LOGO;
      if (!released) {
        pcjr_function_used_as_logo_modifier = true;
      }
    }
  }

  if (pcjr_alt_mapping) {
    active_usb_modifiers &= (uint8_t)~(USB_MOD_ALT | USB_MOD_SHIFT | USB_MOD_RSHIFT);
    active_usb_modifiers |= key->usb_modifier;
  }

  bool sent = true;
  if (!released && key->usb_usage != 0) {
    sent = send_keyboard_tap(key->usb_usage, active_usb_modifiers);
  }

  if (sent) {
    snprintf(usb_info_raw, sizeof(usb_info_raw), "usb usage 0x%02X mods 0x%02X", key->usb_usage, active_usb_modifiers);
  }
  else {
    snprintf(usb_info_raw, sizeof(usb_info_raw), "usb busy usage 0x%02X", key->usb_usage);
  }

  snprintf(usb_info, sizeof(usb_info), "%-32.32s", usb_info_raw);
  serial_write(usb_info);
  print_bit_buffer(bit_buffer);
  serial_write("\r\n");
}

static void gpio_irq_handler(uint gpio, uint32_t events) {
  if (gpio != IR_RX_PIN || (events & GPIO_IRQ_EDGE_FALL) == 0) {
    return;
  }

  ir_falling_edge_count++;

  if (!ir_receiver_armed) {
    return;
  }

  uint32_t now = time_us_32();

  if (!ir_frame_pending) {
    ir_frame_start_us = now;
    ir_frame_pending = true;
    ir_last_pulse_us = now;
    ir_pulse_count = 0;
    ir_pulse_overflow = false;
    return;
  }

  if ((now - ir_last_pulse_us) < IR_MIN_PULSE_SPACING_US) {
    return;
  }

  ir_last_pulse_us = now;

  if (ir_pulse_count < IR_MAX_PULSES) {
    ir_pulse_time_us[ir_pulse_count] = (uint16_t)(now - ir_frame_start_us);
    ir_pulse_count++;
  }
  else {
    ir_pulse_overflow = true;
  }
}

// Receive IR pulses from an active IR frame.
static void handle_ir_receiver(void) {
  if (!ir_frame_pending) {
    return;
  }

  // Get timestamp of frame start.
  uint32_t now = time_us_32();
  uint32_t frame_start_us = ir_frame_start_us;

  // If the frame is still within the expected timing window, wait for more pulses to arrive.
  if ((now - frame_start_us) < ((PCJR_BIT_CELLS * PCJR_BIT_CELL_US) + PCJR_FRAME_GAP_US)) {
    return;
  }

  uint16_t pulse_times[IR_MAX_PULSES];
  uint8_t pulse_count;
  bool pulse_overflow;

  uint32_t interrupts = save_and_disable_interrupts();
  pulse_count = ir_pulse_count;
  pulse_overflow = ir_pulse_overflow;
  // Make a safe local copy of volatile pulse times
  for (uint8_t i = 0; i < pulse_count; i++) {
    pulse_times[i] = ir_pulse_time_us[i];
  }
  ir_frame_pending = false;
  ir_pulse_count = 0;
  ir_pulse_overflow = false;
  restore_interrupts(interrupts);

  uint8_t scancode = 0;
  uint16_t raw_bits = 0;
  char bit_buffer[PCJR_BIT_CELLS + 1];
  // Attempt to decode the frame, and handle it if successful.
  bool decoded = read_pcjr_frame(pulse_times, pulse_count, pulse_overflow, &scancode, &raw_bits, bit_buffer);
  (void)raw_bits;

  if (decoded) {
    // If we successfully decoded a scancode, handle it.
    handle_decoded_scancode(scancode, bit_buffer);
  }
  else {
    // If we failed to decode a valid scancode, indicate an error and print debug info.
    handle_decode_error(bit_buffer, scancode, pulse_times, pulse_count, pulse_overflow);
  }
}

// Handle changes in the serial connection state.
static void handle_serial_connection(void) {
  bool connected = serial_connected();

  // If a terminal was just connected, print the ready message and status.
  if (connected && !serial_was_connected) {
    configure_serial_screen();
    serial_write("Pico 2 PCjr->USB keyboard bridge ready.\r\n");
    serial_printf("Connect active-low IR receiver to GPIO%u.\r\n", IR_RX_PIN);
  }
  serial_was_connected = connected;
}

// Initialize the status LED.
static void init_status_led(void) {
  // Some boards may not have an exposed LED, defining this as -1 or not at all,
  // so don't try to init an invalid pin #.
#if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN >= 0
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
  status_led_off();
}

int main(void) {
  board_init();
  tusb_init();

  init_status_led();

  gpio_init(IR_RX_PIN);
  gpio_set_dir(IR_RX_PIN, GPIO_IN);
  gpio_pull_up(IR_RX_PIN);
  gpio_set_irq_enabled_with_callback(IR_RX_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

  while (true) {
    tud_task();
    release_hid_keyboard();
    handle_serial_connection();
    handle_ir_receiver();
    handle_status_led();
  }
}
