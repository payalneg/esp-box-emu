#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/ledc.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_types.h>

#include <freertos/FreeRTOS.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

#include "base_component.hpp"
#include "display.hpp"
#include "gt911.hpp"
#include "i2c.hpp"
#include "task.hpp"
#include "touchpad_input.hpp"

/// BSP wrapper for Waveshare ESP32-P4-WIFI6-Touch-LCD-4.3 board.
/// Implements the same API surface as espp::EspBox so that BoxEmu can
/// use it as a drop-in replacement via `using Bsp = WaveshareP4Bsp`.
class WaveshareP4Bsp : public espp::BaseComponent {
public:
  using Pixel = lv_color16_t;
  using DisplayDriver = void *; // not used on this platform
  using TouchpadData = espp::TouchpadData;
  using touch_callback_t = std::function<void(const TouchpadData &)>;
  using button_callback_t = std::function<void(bool)>;

  /// Singleton access
  static WaveshareP4Bsp &get() {
    static WaveshareP4Bsp instance;
    return instance;
  }

  WaveshareP4Bsp(const WaveshareP4Bsp &) = delete;
  WaveshareP4Bsp &operator=(const WaveshareP4Bsp &) = delete;
  WaveshareP4Bsp(WaveshareP4Bsp &&) = delete;
  WaveshareP4Bsp &operator=(WaveshareP4Bsp &&) = delete;

  /////////////////////////////////////////////////////////////////////////////
  // Display constants — native portrait 480x800
  /////////////////////////////////////////////////////////////////////////////

  static constexpr size_t lcd_width() { return 800; }
  static constexpr size_t lcd_height() { return 480; }

  /////////////////////////////////////////////////////////////////////////////
  // Pins — no mute button on this board
  /////////////////////////////////////////////////////////////////////////////

  static constexpr gpio_num_t get_mute_pin() { return GPIO_NUM_NC; }

  /////////////////////////////////////////////////////////////////////////////
  // I2C
  /////////////////////////////////////////////////////////////////////////////

  espp::I2c &internal_i2c();

  /////////////////////////////////////////////////////////////////////////////
  // Display
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the MIPI-DSI LCD panel (ST7701)
  bool initialize_lcd();

  /// Initialize the LVGL display (creates espp::Display, allocates VRAM)
  bool initialize_display(size_t pixel_buffer_size);

  /// Initialize the GT911 touchpad
  bool initialize_touch(const touch_callback_t &callback = nullptr);

  std::shared_ptr<espp::Display<Pixel>> display() const;

  void brightness(float brightness);
  float brightness() const;

  Pixel *vram0() const;
  Pixel *vram1() const;
  uint8_t *frame_buffer0() const;
  uint8_t *frame_buffer1() const;

  /// Write a rectangle of pixel data to the LCD
  void write_lcd_frame(const uint16_t x, const uint16_t y, const uint16_t width,
                       const uint16_t height, uint8_t *data);

  /////////////////////////////////////////////////////////////////////////////
  // Touch
  /////////////////////////////////////////////////////////////////////////////

  TouchpadData touchpad_data() const;
  std::shared_ptr<espp::TouchpadInput> touchpad_input() const;
  void touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y, uint8_t *btn_state);

  /////////////////////////////////////////////////////////////////////////////
  // Audio
  /////////////////////////////////////////////////////////////////////////////

  bool initialize_sound(uint32_t default_audio_rate = 48000,
                        const espp::Task::BaseConfig &task_config = {.name = "audio",
                                                                     .stack_size_bytes = 4096,
                                                                     .priority = 19,
                                                                     .core_id = 1});
  void enable_sound(bool enable);
  uint32_t audio_sample_rate() const;
  void audio_sample_rate(uint32_t sample_rate);
  void mute(bool mute);
  bool is_muted() const;
  void volume(float volume);
  float volume() const;
  void play_audio(const std::vector<uint8_t> &data);
  void play_audio(const uint8_t *data, uint32_t num_bytes);

protected:
  WaveshareP4Bsp();

  bool initialize_backlight();
  bool initialize_dsi_panel();
  bool initialize_codec();
  bool initialize_i2s(uint32_t default_audio_rate);
  bool update_touch();
  void update_volume_output();
  bool audio_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  /////////////////////////////////////////////////////////////////////////////
  // Pin definitions
  /////////////////////////////////////////////////////////////////////////////

  // I2C (touch + audio codec)
  static constexpr auto internal_i2c_port = I2C_NUM_1;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_7;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_8;

  // Display
  static constexpr gpio_num_t lcd_backlight_io = GPIO_NUM_26;
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_27;
  static constexpr gpio_num_t lcd_touch_reset_io = GPIO_NUM_23;

  // Audio I2S
  static constexpr auto i2s_port = I2S_NUM_1;
  static constexpr gpio_num_t i2s_mck_io = GPIO_NUM_13;
  static constexpr gpio_num_t i2s_bck_io = GPIO_NUM_12;
  static constexpr gpio_num_t i2s_ws_io = GPIO_NUM_10;
  static constexpr gpio_num_t i2s_do_io = GPIO_NUM_9;
  static constexpr gpio_num_t i2s_di_io = GPIO_NUM_11;
  static constexpr gpio_num_t power_amp_io = GPIO_NUM_53;

  // Audio constants
  static constexpr int NUM_CHANNELS = 2;
  static constexpr int NUM_BYTES_PER_CHANNEL = 2;
  static constexpr int UPDATE_FREQUENCY = 60;

  static constexpr int calc_audio_buffer_size(int sample_rate) {
    return sample_rate * NUM_CHANNELS * NUM_BYTES_PER_CHANNEL / UPDATE_FREQUENCY;
  }

  /////////////////////////////////////////////////////////////////////////////
  // State
  /////////////////////////////////////////////////////////////////////////////

  espp::I2c internal_i2c_{{.port = internal_i2c_port,
                           .sda_io_num = internal_i2c_sda,
                           .scl_io_num = internal_i2c_scl,
                           .sda_pullup_en = GPIO_PULLUP_ENABLE,
                           .scl_pullup_en = GPIO_PULLUP_ENABLE}};

  // Display
  esp_lcd_panel_handle_t panel_handle_{nullptr};
  std::shared_ptr<espp::Display<Pixel>> display_;
  Pixel *vram_0_{nullptr};
  Pixel *vram_1_{nullptr};
  Pixel *rot_buf_{nullptr}; // scratch buffer for 90° rotation
  uint8_t *frame_buffer0_{nullptr};
  uint8_t *frame_buffer1_{nullptr};
  std::atomic<float> brightness_{0.0f};

  // Touch
  std::shared_ptr<espp::Gt911> gt911_;
  std::shared_ptr<espp::TouchpadInput> touchpad_input_;
  std::recursive_mutex touchpad_data_mutex_;
  TouchpadData touchpad_data_;
  touch_callback_t touch_callback_{nullptr};

  // Audio
  std::atomic<bool> sound_initialized_{false};
  std::atomic<float> volume_{50.0f};
  std::atomic<bool> mute_{false};
  std::unique_ptr<espp::Task> audio_task_{nullptr};
  i2s_chan_handle_t audio_tx_handle_{nullptr};
  i2s_chan_handle_t audio_rx_handle_{nullptr};
  std::vector<uint8_t> audio_tx_buffer_;
  StreamBufferHandle_t audio_tx_stream_{nullptr};
  i2s_std_config_t audio_std_cfg_;
};
