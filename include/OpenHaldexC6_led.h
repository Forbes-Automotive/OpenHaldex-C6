#pragma once
#include <Arduino.h>

// Single on-board WS2812, driven by the core's rgbLedWriteOrdered().
// Drop-in for the subset of Freenove_ESP32_WS2812 this firmware used (begin /
// setBrightness / setLedColorData / show). The Freenove class hard-codes a
// 256-LED RMT buffer (24 KB of static RAM for our one LED); on the C6 with
// WiFi + ESP-NOW + BLE that 24 KB was the difference between BLE starting
// (and the web UI serving) or not. The core keeps a 24-entry buffer on the
// caller's stack for the duration of the write instead.
class OneLed
{
public:
  OneLed(uint8_t pin, rgb_led_color_order_t order) : pin_(pin), order_(order) {}

  bool begin() { return true; } // RMT channel is set up by the first write

  void setBrightness(uint8_t brightness) { br_ = brightness; }

  // index ignored: there is only one LED
  void setLedColorData(int /*index*/, uint8_t r, uint8_t g, uint8_t b)
  {
    r_ = r * br_ / 255;
    g_ = g * br_ / 255;
    b_ = b * br_ / 255;
  }

  void show() { rgbLedWriteOrdered(pin_, order_, r_, g_, b_); }

private:
  uint8_t pin_;
  rgb_led_color_order_t order_;
  uint8_t br_ = 255;
  uint8_t r_ = 0, g_ = 0, b_ = 0;
};
