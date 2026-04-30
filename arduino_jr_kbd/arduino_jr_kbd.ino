/*
    arduino_jr_kbd.ino

    Copyright 2022-2025 Daniel Balsom

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

#include "PluggableUSBHID.h"
#include "USBKeyboard.h"

USBKeyboard host_keyboard;

// GPIO pin to use for IR strobe, active-low.
const uint8_t IR_RX_PIN = 3;

// How many bitcells per PCjr scancode - 1 start bit + 8 scancode bits + 1 parity bit = 10 bitcells
const uint8_t PCJR_BIT_CELLS = 10;
// Approx ~440us per bitcell, or 4.4ms per scancode
const uint16_t PCJR_BIT_CELL_US = 440;
const uint16_t PCJR_HALF_CELL_US = PCJR_BIT_CELL_US / 2;
const uint16_t PCJR_FRAME_GAP_US = 2500;
// Start bit value is always 1
const uint8_t PCJR_DATA_START_CELL = 1;
// Parity bit is always bitcell 9 (final bitcell)
const uint8_t PCJR_PARITY_CELL = 9;
const uint8_t PCJR_FUNCTION_SCANCODE = 0x54;
// Maximum pulse count before we give up decoding
const uint8_t IR_MAX_PULSES = 24;
const uint16_t IR_MIN_PULSE_SPACING_US = 80;
const uint16_t PCJR_PULSE_TIMING_TOLERANCE_US = 170;
const int16_t PCJR_PULSE_PHASE_OFFSET_US = 0;

// How long to pulse the RGB status LED when indicating a status.
const uint16_t STATUS_LED_PULSE_MS = 80;

volatile uint32_t ir_frame_start_us = 0;
volatile bool ir_frame_pending = false;
volatile bool ir_receiver_armed = true;
volatile uint16_t ir_pulse_time_us[IR_MAX_PULSES];
volatile uint8_t ir_pulse_count = 0;
volatile uint32_t ir_last_pulse_us = 0;
volatile bool ir_pulse_overflow = false;
volatile uint32_t ir_falling_edge_count = 0;

bool serial_was_connected = false;
uint32_t status_led_off_at_ms = 0;
bool status_led_active = false;
uint8_t current_usb_modifiers = 0;
bool pcjr_function_down = false;
bool pcjr_function_used_in_combo = false;

// Table definition for mapping PCjr scancodes to key names and USB HID usage/modifier codes.
struct pcjr_key_mapping {
  uint8_t pcjr_scancode;
  const char *name;
  uint8_t usb_usage;
  uint8_t usb_modifier;
};

const pcjr_key_mapping PCJR_KEYMAP[] = {
    {0x01, "Escape", 0x29, 0},
    {0x02, "1", 0x1E, 0},
    {0x03, "2", 0x1F, 0},
    {0x04, "3", 0x20, 0},
    {0x05, "4", 0x21, 0},
    {0x06, "5", 0x22, 0},
    {0x07, "6", 0x23, 0},
    {0x08, "7", 0x24, 0},
    {0x09, "8", 0x25, 0},
    {0x0A, "9", 0x26, 0},
    {0x0B, "0", 0x27, 0},
    {0x0C, "Minus", 0x2D, 0},
    {0x0D, "Equal", 0x2E, 0},
    {0x0E, "Backspace", 0x2A, 0},
    {0x0F, "Tab", 0x2B, 0},
    {0x10, "Q", 0x14, 0},
    {0x11, "W", 0x1A, 0},
    {0x12, "E", 0x08, 0},
    {0x13, "R", 0x15, 0},
    {0x14, "T", 0x17, 0},
    {0x15, "Y", 0x1C, 0},
    {0x16, "U", 0x18, 0},
    {0x17, "I", 0x0C, 0},
    {0x18, "O", 0x12, 0},
    {0x19, "P", 0x13, 0},
    {0x1A, "BracketLeft", 0x2F, 0},
    {0x1B, "BracketRight", 0x30, 0},
    {0x1C, "Enter", 0x28, 0},
    {0x1D, "Control", 0, KEY_CTRL},
    {0x1E, "A", 0x04, 0},
    {0x1F, "S", 0x16, 0},
    {0x20, "D", 0x07, 0},
    {0x21, "F", 0x09, 0},
    {0x22, "G", 0x0A, 0},
    {0x23, "H", 0x0B, 0},
    {0x24, "J", 0x0D, 0},
    {0x25, "K", 0x0E, 0},
    {0x26, "L", 0x0F, 0},
    {0x27, "Semicolon", 0x33, 0},
    {0x28, "Quote", 0x34, 0},
    {0x2A, "ShiftLeft", 0, KEY_SHIFT},
    {0x2C, "Z", 0x1D, 0},
    {0x2D, "X", 0x1B, 0},
    {0x2E, "C", 0x06, 0},
    {0x2F, "V", 0x19, 0},
    {0x30, "B", 0x05, 0},
    {0x31, "N", 0x11, 0},
    {0x32, "M", 0x10, 0},
    {0x33, "Comma", 0x36, 0},
    {0x34, "Period", 0x37, 0},
    {0x35, "Slash", 0x38, 0},
    {0x36, "ShiftRight", 0, KEY_RSHIFT},
    {0x37, "PrintScreen", 0x46, 0},
    {0x38, "Alt", 0, KEY_ALT},
    {0x39, "Space", 0x2C, 0},
    {0x3A, "CapsLock", 0x39, 0},
    {0x3B, "F1", 0x3A, 0},
    {0x3C, "F2", 0x3B, 0},
    {0x3D, "F3", 0x3C, 0},
    {0x3E, "F4", 0x3D, 0},
    {0x3F, "F5", 0x3E, 0},
    {0x40, "F6", 0x3F, 0},
    {0x41, "F7", 0x40, 0},
    {0x42, "F8", 0x41, 0},
    {0x43, "F9", 0x42, 0},
    {0x44, "F10", 0x43, 0},
    {0x45, "Pause", 0x48, 0},
    {0x46, "ScrollLock", 0x47, 0},
    {0x48, "ArrowUp", 0x52, 0},
    {0x4B, "ArrowLeft", 0x50, 0},
    {0x4D, "ArrowRight", 0x4F, 0},
    {0x50, "ArrowDown", 0x51, 0},
    {0x52, "Insert", 0x49, 0},
    {0x53, "Delete", 0x4C, 0},
    {PCJR_FUNCTION_SCANCODE, "Function", 0, KEY_LOGO},
};

const size_t PCJR_KEYMAP_SIZE = sizeof(PCJR_KEYMAP) / sizeof(PCJR_KEYMAP[0]);

const pcjr_key_mapping PCJR_FUNCTION_KEYMAP[] = {
    {0x02, "F1", 0x3A, 0},
    {0x03, "F2", 0x3B, 0},
    {0x04, "F3", 0x3C, 0},
    {0x05, "F4", 0x3D, 0},
    {0x06, "F5", 0x3E, 0},
    {0x07, "F6", 0x3F, 0},
    {0x08, "F7", 0x40, 0},
    {0x09, "F8", 0x41, 0},
    {0x0A, "F9", 0x42, 0},
    {0x0B, "F10", 0x43, 0},
    {0x10, "Pause", 0x48, 0},
    {0x19, "PrintScreen", 0x46, 0},
    {0x1F, "ScrollLock", 0x47, 0},
};

const size_t PCJR_FUNCTION_KEYMAP_SIZE = sizeof(PCJR_FUNCTION_KEYMAP) / sizeof(PCJR_FUNCTION_KEYMAP[0]);

// Turn RGB status led completely off.
void status_led_off() {
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  status_led_active = false;
}

// Turn the specified RGB status LED pin on, scheduling it to turn off after a short delay.
void pulse_status_led(uint8_t led_pin) {
  status_led_off();
  digitalWrite(led_pin, LOW);
  status_led_off_at_ms = millis() + STATUS_LED_PULSE_MS;
  status_led_active = true;
}

// Turn the specified RGB status LED pins on, scheduling them to turn off after a short delay.
void pulse_status_led(uint8_t led_pin_a, uint8_t led_pin_b) {
  status_led_off();
  digitalWrite(led_pin_a, LOW);
  digitalWrite(led_pin_b, LOW);
  status_led_off_at_ms = millis() + STATUS_LED_PULSE_MS;
  status_led_active = true;
}

// Check if it is time to turn the status LED off after a pulse has triggered.
void handle_status_led() {
  if (status_led_active && static_cast<int32_t>(millis() - status_led_off_at_ms) >= 0) {
    status_led_off();
  }
}

// Print a byte value to the serial console as a two-digit hexadecimal value.
void print_hex_byte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

// Print the ASCII bit-buffer.
void print_bit_buffer(const char bit_buffer[]) {
  Serial.print(" bits ");
  for (uint8_t i = 0; i < PCJR_BIT_CELLS; i++) {
    Serial.print(bit_buffer[i]);
  }
}

void handle_decoded_scancode(uint8_t scancode, const char bit_buffer[]) {
  bool released = (scancode & 0x80) != 0;
  uint8_t base_scancode = scancode & 0x7F;
  const pcjr_key_mapping *key = nullptr;
  char key_name[11];
  char usb_info[33];
  char usb_info_raw[33];

  for (size_t i = 0; i < PCJR_KEYMAP_SIZE; i++) {
    if (PCJR_KEYMAP[i].pcjr_scancode == base_scancode) {
      key = &PCJR_KEYMAP[i];
      break;
    }
  }

  if (pcjr_function_down && base_scancode != PCJR_FUNCTION_SCANCODE) {
    pcjr_function_used_in_combo = true;
    for (size_t i = 0; i < PCJR_FUNCTION_KEYMAP_SIZE; i++) {
      if (PCJR_FUNCTION_KEYMAP[i].pcjr_scancode == base_scancode) {
        key = &PCJR_FUNCTION_KEYMAP[i];
        break;
      }
    }
  }

  snprintf(key_name, sizeof(key_name), "%-10.10s", key == nullptr ? "??" : key->name);
  snprintf(usb_info, sizeof(usb_info), "%-32.32s", "");

  if (released) {
    pulse_status_led(LEDR, LEDG);
  }
  else {
    pulse_status_led(LEDG);
  }

  Serial.print("[");
  Serial.print(released ? "keyup  ]: " : "keydown]: ");

  if (key == nullptr) {
    print_hex_byte(base_scancode);
    Serial.print(" '");
    Serial.print(key_name);
    Serial.print("' ");
    Serial.print(usb_info);
    return;
  }

  Serial.print("Scancode: ");
  print_hex_byte(base_scancode);
  Serial.print("/");
  print_hex_byte(scancode);
  Serial.print(" '");
  Serial.print(key_name);
  Serial.print("' ");

  if (base_scancode == PCJR_FUNCTION_SCANCODE) {
    if (released) {
      if (pcjr_function_down && !pcjr_function_used_in_combo) {
        host_keyboard.key_code_raw(0, KEY_LOGO);
        snprintf(usb_info_raw, sizeof(usb_info_raw), "usb logo tap 0x%02X", KEY_LOGO);
      }
      else {
        snprintf(usb_info_raw, sizeof(usb_info_raw), "usb logo combo suppressed");
      }
      pcjr_function_down = false;
      pcjr_function_used_in_combo = false;
    }
    else {
      pcjr_function_down = true;
      pcjr_function_used_in_combo = false;
      snprintf(usb_info_raw, sizeof(usb_info_raw), "usb logo pending 0x%02X", KEY_LOGO);
    }

    snprintf(usb_info, sizeof(usb_info), "%-32.32s", usb_info_raw);
    Serial.print(usb_info);
    print_bit_buffer(bit_buffer);
    Serial.println();
    return;
  }

  if (key->usb_modifier != 0) {
    if (released) {
      current_usb_modifiers &= ~key->usb_modifier;
    }
    else {
      current_usb_modifiers |= key->usb_modifier;
    }

    snprintf(usb_info_raw, sizeof(usb_info_raw), "usb modifier 0x%02X active 0x%02X", key->usb_modifier,
             current_usb_modifiers);
    snprintf(usb_info, sizeof(usb_info), "%-32.32s", usb_info_raw);
    Serial.print(usb_info);
    return;
  }

  snprintf(usb_info_raw, sizeof(usb_info_raw), "usb usage 0x%02X", key->usb_usage);
  snprintf(usb_info, sizeof(usb_info), "%-32.32s", usb_info_raw);
  Serial.print(usb_info);

  if (!released && key->usb_usage != 0) {
    host_keyboard.key_code_raw(key->usb_usage, current_usb_modifiers);
  }

  print_bit_buffer(bit_buffer);
  Serial.println();
}

// ISR handler for IR_RX_PIN GPIO pin falling edge.
void ir_start_isr() {
  // Update trigger count.
  ir_falling_edge_count++;

  if (!ir_receiver_armed) {
    return;
  }

  uint32_t now = micros();

  // If we weren't in an IR frame, start one now.
  if (!ir_frame_pending) {
    ir_frame_start_us = now;
    ir_frame_pending = true;
    ir_last_pulse_us = now;
    ir_pulse_count = 0;
    ir_pulse_overflow = false;
    return;
  }

  if ((now - ir_last_pulse_us) < IR_MIN_PULSE_SPACING_US) {
    // Ignore this interrupt since it is too close to the previous.
    return;
  }

  ir_last_pulse_us = now;

  // Collect timestamp for current pulse.
  if (ir_pulse_count < IR_MAX_PULSES) {
    ir_pulse_time_us[ir_pulse_count] = static_cast<uint16_t>(now - ir_frame_start_us);
    ir_pulse_count++;
  }
  else {
    // We have more than IR_MAX_PULSES, flag overflow error condition.
    ir_pulse_overflow = true;
  }
}

// Calculate odd parity.
bool has_odd_parity(uint8_t value, uint8_t parity_bit) {
  uint8_t ones = parity_bit ? 1 : 0;

  for (uint8_t i = 0; i < 8; i++) {
    if (value & (1 << i)) {
      ones++;
    }
  }
  return (ones & 1) == 1;
}

// Return the nearest pulse target distance in `nearest_distance_us`, and return true if underneath our tolerance.
bool nearest_pulse_distance(const uint16_t pulse_times[], uint8_t pulse_count, uint16_t target_us,
                            uint16_t *nearest_distance_us) {
  if (pulse_count == 0) {
    return false;
  }

  uint16_t nearest = 0xFFFF;

  for (uint8_t i = 0; i < pulse_count; i++) {
    uint16_t distance = pulse_times[i] > target_us ? pulse_times[i] - target_us : target_us - pulse_times[i];
    if (distance < nearest) {
      nearest = distance;
    }
  }

  *nearest_distance_us = nearest;
  return nearest <= PCJR_PULSE_TIMING_TOLERANCE_US;
}

// Helper to shift pulse phase with clamping.
uint16_t pulse_slot_us(uint16_t base_us) {
  int32_t shifted = static_cast<int32_t>(base_us) + PCJR_PULSE_PHASE_OFFSET_US;

  if (shifted < 0) {
    return 0;
  }

  if (shifted > 0xFFFF) {
    return 0xFFFF;
  }

  return static_cast<uint16_t>(shifted);
}

// Attempt to decode a PCjr scancode from the given pulse times and count.
bool read_pcjr_frame(const uint16_t pulse_times[], uint8_t pulse_count, bool pulse_overflow, uint8_t *scancode,
                     uint16_t *raw_bits, char bit_buffer[]) {
  uint16_t bits = 0;
  bool frame_ok = !pulse_overflow;

  for (uint8_t i = 0; i < PCJR_BIT_CELLS; i++) {
    bit_buffer[i] = '?';
  }
  bit_buffer[PCJR_BIT_CELLS] = '\0';

  // The falling edge that triggered the receive is the active-low start bit.
  bits |= 1;
  bit_buffer[0] = '1';

  for (uint8_t cell = 1; cell < PCJR_BIT_CELLS; cell++) {
    uint16_t cell_start_us = cell * PCJR_BIT_CELL_US;
    uint16_t one_distance_us;
    uint16_t zero_distance_us;
    bool near_one_slot =
        nearest_pulse_distance(pulse_times, pulse_count, pulse_slot_us(cell_start_us), &one_distance_us);
    bool near_zero_slot = nearest_pulse_distance(pulse_times, pulse_count,
                                                 pulse_slot_us(cell_start_us + PCJR_HALF_CELL_US), &zero_distance_us);

    if (!near_one_slot && !near_zero_slot) {
      bit_buffer[cell] = '?';
      frame_ok = false;
    }
    else if (near_one_slot && (!near_zero_slot || one_distance_us <= zero_distance_us)) {
      bits |= (1 << cell);
      bit_buffer[cell] = '1';
    }
    else {
      bit_buffer[cell] = '0';
    }
  }

  uint8_t data = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (bits & (1 << (PCJR_DATA_START_CELL + i))) {
      data |= (1 << i);
    }
  }

  uint8_t start_bit = bits & 1;
  uint8_t parity_bit = (bits >> PCJR_PARITY_CELL) & 1;

  if (start_bit != 1 || !has_odd_parity(data, parity_bit)) {
    frame_ok = false;
  }

  *scancode = data;
  *raw_bits = bits;
  return frame_ok;
}

void handle_decode_error(const char bit_buffer[], uint8_t scancode, const uint16_t pulse_times[], uint8_t pulse_count,
                         bool pulse_overflow) {
  pulse_status_led(LEDR);
  Serial.print("IR frame ignored: decode/parity error");
  print_bit_buffer(bit_buffer);
  Serial.print(" scancode candidate 0x");
  print_hex_byte(scancode);
  Serial.print(" pulses ");
  Serial.print(pulse_count);
  Serial.print(" pulse_us");
  for (uint8_t i = 0; i < pulse_count; i++) {
    Serial.print(i == 0 ? " " : ",");
    Serial.print(pulse_times[i]);
  }
  if (pulse_overflow) {
    Serial.print(" overflow");
  }
  Serial.println();
}

// Receive IR pulses from an active IR frame.
void handle_ir_receiver() {
  if (!ir_frame_pending) {
    // Nothing to do
    return;
  }

  // Get timestamp of frame start.
  uint32_t now = micros();
  uint32_t frame_start_us = ir_frame_start_us;

  // If the frame is still within the expected timing window, wait for more pulses to arrive.
  if ((now - frame_start_us) < ((PCJR_BIT_CELLS * PCJR_BIT_CELL_US) + PCJR_FRAME_GAP_US)) {
    return;
  }

  uint16_t pulse_times[IR_MAX_PULSES];
  uint8_t pulse_count;
  bool pulse_overflow;

  noInterrupts();
  pulse_count = ir_pulse_count;
  pulse_overflow = ir_pulse_overflow;
  // make a safe local copy of volatile pulse times
  for (uint8_t i = 0; i < pulse_count; i++) {
    pulse_times[i] = ir_pulse_time_us[i];
  }
  ir_frame_pending = false;
  ir_pulse_count = 0;
  ir_pulse_overflow = false;
  interrupts();

  uint8_t scancode = 0;
  uint16_t raw_bits = 0;
  char bit_buffer[PCJR_BIT_CELLS + 1];
  // Attempt to decode the frame, and handle it if successful.
  bool decoded = read_pcjr_frame(pulse_times, pulse_count, pulse_overflow, &scancode, &raw_bits, bit_buffer);

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
void handle_serial_connection() {
  // Serial becomes true when a connection is made.
  // It becomes false again when the terminal is closed.
  // So we can respond to the user opening and closing the serial terminal.
  bool serial_connected = Serial;

  if (serial_connected && !serial_was_connected) {
    // If a terminal was just connected, print the ready message and status.
    Serial.println("Arduino Giga PCjr->USB keyboard bridge ready.");
    Serial.print("Connect active-low IR receiver to D");
    Serial.print(IR_RX_PIN);
    Serial.println(".");
  }
  serial_was_connected = serial_connected;
}

void setup() {
  // Set up RGB LED pins as outputs, then turn the status LED off.
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  status_led_off();

  // Set up the IR receiver pin as an input, with an interrupt on falling edge
  pinMode(IR_RX_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_RX_PIN), ir_start_isr, FALLING);

  Serial.begin(115200);
}

void loop() {
  handle_serial_connection();
  handle_ir_receiver();
  handle_status_led();
}
