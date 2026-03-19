#pragma once

#include <sdkconfig.h>

#include <memory>
#include <string>
#include <vector>

#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/spi_master.h>
#include <hal/spi_ll.h>
#include <hal/spi_types.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>

#include "base_component.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "st7789.hpp"
#include "touchpad_input.hpp"

#ifdef CONFIG_ESP_BOX_BOARD_WAVESHARE_28
#include "cst328.hpp"
#else
#include "es7210.hpp"
#include "es8311.hpp"
#include "gt911.hpp"
#include "icm42607.hpp"
#include "tt21100.hpp"
#endif

namespace espp {
/// The EspBox class provides an interface to the ESP32-S3-BOX,
/// ESP32-S3-BOX-3, and Waveshare ESP32-S3-Touch-LCD-2.8 development boards.
class EspBox : public BaseComponent {
public:
  using button_callback_t = espp::Interrupt::event_callback_fn;
  using Pixel = lv_color16_t;
  using DisplayDriver = espp::St7789;
  using TouchpadData = espp::TouchpadData;
  using touch_callback_t = std::function<void(const TouchpadData &)>;

#ifndef CONFIG_ESP_BOX_BOARD_WAVESHARE_28
  using Imu = espp::Icm42607<icm42607::Interface::I2C>;
#endif

  static constexpr size_t SPI_MAX_TRANSFER_BYTES = SPI_LL_DMA_MAX_BIT_LEN / 8;

  enum class BoxType {
    UNKNOWN,
    BOX,
    BOX3,
    WAVESHARE,
  };

  static EspBox &get() {
    static EspBox instance;
    return instance;
  }

  EspBox(const EspBox &) = delete;
  EspBox &operator=(const EspBox &) = delete;
  EspBox(EspBox &&) = delete;
  EspBox &operator=(EspBox &&) = delete;

  BoxType box_type() const { return box_type_; }

  I2c &internal_i2c() { return internal_i2c_; }

  espp::Interrupt &interrupts() { return interrupts_; }

  /////////////////////////////////////////////////////////////////////////////
  // Touchpad
  /////////////////////////////////////////////////////////////////////////////

  bool initialize_touch(const touch_callback_t &callback = nullptr);
  std::shared_ptr<TouchpadInput> touchpad_input() const { return touchpad_input_; }
  TouchpadData touchpad_data() const { return touchpad_data_; }
  void touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y, uint8_t *btn_state);
  TouchpadData touchpad_convert(const TouchpadData &data) const;

  /////////////////////////////////////////////////////////////////////////////
  // Display
  /////////////////////////////////////////////////////////////////////////////

  bool initialize_lcd();
  bool initialize_display(size_t pixel_buffer_size);

  static constexpr size_t lcd_width() { return lcd_width_; }
  static constexpr size_t lcd_height() { return lcd_height_; }
  static constexpr auto get_lcd_dc_gpio() { return lcd_dc_io; }
  static constexpr size_t display_width() { return lcd_width_; }
  static constexpr size_t display_height() { return lcd_height_; }

  size_t rotated_display_width() const;
  size_t rotated_display_height() const;

  std::shared_ptr<Display<Pixel>> display() const { return display_; }

  void brightness(float brightness);
  float brightness() const;

  Pixel *vram0() const;
  Pixel *vram1() const;
  uint8_t *frame_buffer0() const;
  uint8_t *frame_buffer1() const;

  void write_command(uint8_t command, std::span<const uint8_t> parameters, uint32_t user_data);
  void write_lcd_frame(const uint16_t x, const uint16_t y, const uint16_t width,
                       const uint16_t height, uint8_t *data);
  void write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data, uint32_t user_data);

  /////////////////////////////////////////////////////////////////////////////
  // Button
  /////////////////////////////////////////////////////////////////////////////

  bool initialize_boot_button(const button_callback_t &callback = nullptr);
  bool boot_button_state() const;
  bool initialize_mute_button(const button_callback_t &callback = nullptr);
  bool mute_button_state() const;

  /////////////////////////////////////////////////////////////////////////////
  // Audio
  /////////////////////////////////////////////////////////////////////////////

  static constexpr auto get_mute_pin() { return mute_pin; }

  bool
  initialize_sound(uint32_t default_audio_rate = 48000,
                   const espp::Task::BaseConfig &task_config = {
                       .name = "audio", .stack_size_bytes = 4096, .priority = 19, .core_id = 1});
  void enable_sound(bool enable);
  uint32_t audio_sample_rate() const;
  void audio_sample_rate(uint32_t sample_rate);
  size_t audio_buffer_size() const;
  void mute(bool mute);
  bool is_muted() const;
  void volume(float volume);
  float volume() const;
  void play_audio(const std::vector<uint8_t> &data);
  void play_audio(const uint8_t *data, uint32_t num_bytes);

  /////////////////////////////////////////////////////////////////////////////
  // IMU
  /////////////////////////////////////////////////////////////////////////////

#ifndef CONFIG_ESP_BOX_BOARD_WAVESHARE_28
  bool initialize_imu(const Imu::filter_fn &orientation_filter = nullptr,
                      const Imu::ImuConfig &imu_config = {
                          .accelerometer_range = Imu::AccelerometerRange::RANGE_2G,
                          .accelerometer_odr = Imu::AccelerometerODR::ODR_400_HZ,
                          .gyroscope_range = Imu::GyroscopeRange::RANGE_2000DPS,
                          .gyroscope_odr = Imu::GyroscopeODR::ODR_400_HZ});
  std::shared_ptr<Imu> imu() const;
#endif

  bool update_touch();

protected:
  EspBox();
  void detect();
  bool initialize_codec();
  bool initialize_i2s(uint32_t default_audio_rate);
  void update_volume_output();
  bool audio_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified);
  void lcd_wait_lines();

#ifdef CONFIG_ESP_BOX_BOARD_WAVESHARE_28
  bool update_cst328();
#else
  bool update_gt911();
  bool update_tt21100();
#endif

  // =========================================================================
  // Board-specific pin definitions
  // =========================================================================

#ifdef CONFIG_ESP_BOX_BOARD_WAVESHARE_28

  // --- Waveshare ESP32-S3-Touch-LCD-2.8 ---

  // Waveshare has a single configuration (no BOX vs BOX3 runtime detection)
  struct waveshare {
    static constexpr gpio_num_t backlight_io = GPIO_NUM_5;
    static constexpr bool reset_value = false; // active-low reset for ST7789
    static constexpr gpio_num_t i2s_ws_io = GPIO_NUM_38;
    static constexpr bool touch_invert_x = false; // TODO: verify on hardware
    static constexpr auto touch_interrupt_level = espp::Interrupt::ActiveLevel::LOW;
    static constexpr auto touch_interrupt_type = espp::Interrupt::Type::FALLING_EDGE;
    static constexpr auto touch_interrupt_pullup_enabled = true;
  };

  // button
  static constexpr gpio_num_t boot_button_io = GPIO_NUM_6; // PWR button on Waveshare

  // internal i2c (touchscreen)
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_1;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_3;

  // LCD
  static constexpr size_t lcd_width_ = 320;
  static constexpr size_t lcd_height_ = 240;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr size_t frame_buffer_size = (((lcd_width_)*lcd_bytes_per_pixel) * lcd_height_);
  static constexpr int lcd_clock_speed = 60 * 1000 * 1000;
  static constexpr auto lcd_spi_num = SPI2_HOST;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_42;
  static constexpr gpio_num_t lcd_mosi_io = GPIO_NUM_45;
  static constexpr gpio_num_t lcd_sclk_io = GPIO_NUM_40;
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_39;
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_41;
  static constexpr bool backlight_value = true;
  static constexpr bool invert_colors = false;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool mirror_x = false;  // TODO: verify orientation on hardware
  static constexpr bool mirror_y = false;  // TODO: verify orientation on hardware
  static constexpr bool swap_xy = false;
  static constexpr bool swap_color_order = false; // TODO: verify on hardware

  // touch
  static constexpr bool touch_swap_xy = false;
  static constexpr bool touch_invert_y = false;
  static constexpr gpio_num_t touch_interrupt = GPIO_NUM_4;
  static constexpr gpio_num_t touch_reset_io = GPIO_NUM_2;

  // sound (PCM5101 — pure I2S DAC, no codec registers, no power/mute pins)
  static constexpr gpio_num_t sound_power_pin = GPIO_NUM_NC;
  static constexpr auto i2s_port = I2S_NUM_0;
  static constexpr gpio_num_t i2s_mck_io = GPIO_NUM_NC;
  static constexpr gpio_num_t i2s_bck_io = GPIO_NUM_48;
  static constexpr gpio_num_t i2s_do_io = GPIO_NUM_47;
  static constexpr gpio_num_t i2s_di_io = GPIO_NUM_NC;
  static constexpr gpio_num_t mute_pin = GPIO_NUM_NC;

#else // CONFIG_ESP_BOX_BOARD_ESPBOX (default)

  // --- ESP32-S3-BOX / BOX-3 ---

  // box 3:
  struct box3 {
    static constexpr gpio_num_t backlight_io = GPIO_NUM_47;
    static constexpr bool reset_value = true;
    static constexpr gpio_num_t i2s_ws_io = GPIO_NUM_45;
    static constexpr bool touch_invert_x = false;
    static constexpr auto touch_interrupt_level = espp::Interrupt::ActiveLevel::HIGH;
    static constexpr auto touch_interrupt_type = espp::Interrupt::Type::RISING_EDGE;
    static constexpr auto touch_interrupt_pullup_enabled = false;
  };

  // box:
  struct box {
    static constexpr gpio_num_t backlight_io = GPIO_NUM_45;
    static constexpr bool reset_value = false;
    static constexpr gpio_num_t i2s_ws_io = GPIO_NUM_47;
    static constexpr bool touch_invert_x = true;
    static constexpr auto touch_interrupt_level = espp::Interrupt::ActiveLevel::LOW;
    static constexpr auto touch_interrupt_type = espp::Interrupt::Type::FALLING_EDGE;
    static constexpr auto touch_interrupt_pullup_enabled = true;
  };

  // button
  static constexpr gpio_num_t boot_button_io = GPIO_NUM_0;

  // internal i2c (touchscreen, audio codec)
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_8;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_18;

  // LCD
  static constexpr size_t lcd_width_ = 320;
  static constexpr size_t lcd_height_ = 240;
  static constexpr size_t lcd_bytes_per_pixel = 2;
  static constexpr size_t frame_buffer_size = (((lcd_width_)*lcd_bytes_per_pixel) * lcd_height_);
  static constexpr int lcd_clock_speed = 60 * 1000 * 1000;
  static constexpr auto lcd_spi_num = SPI2_HOST;
  static constexpr gpio_num_t lcd_cs_io = GPIO_NUM_5;
  static constexpr gpio_num_t lcd_mosi_io = GPIO_NUM_6;
  static constexpr gpio_num_t lcd_sclk_io = GPIO_NUM_7;
  static constexpr gpio_num_t lcd_reset_io = GPIO_NUM_48;
  static constexpr gpio_num_t lcd_dc_io = GPIO_NUM_4;
  static constexpr bool backlight_value = true;
  static constexpr bool invert_colors = true;
  static constexpr auto rotation = espp::DisplayRotation::LANDSCAPE;
  static constexpr bool mirror_x = true;
  static constexpr bool mirror_y = true;
  static constexpr bool swap_xy = false;
  static constexpr bool swap_color_order = true;

  // touch
  static constexpr bool touch_swap_xy = false;
  static constexpr bool touch_invert_y = false;
  static constexpr gpio_num_t touch_interrupt = GPIO_NUM_3;

  // sound
  static constexpr gpio_num_t sound_power_pin = GPIO_NUM_46;
  static constexpr auto i2s_port = I2S_NUM_0;
  static constexpr gpio_num_t i2s_mck_io = GPIO_NUM_2;
  static constexpr gpio_num_t i2s_bck_io = GPIO_NUM_17;
  static constexpr gpio_num_t i2s_do_io = GPIO_NUM_15;
  static constexpr gpio_num_t i2s_di_io = GPIO_NUM_16;
  static constexpr gpio_num_t mute_pin = GPIO_NUM_1;

#endif // CONFIG_ESP_BOX_BOARD_WAVESHARE_28

  // =========================================================================
  // Common constants
  // =========================================================================

  static constexpr int NUM_CHANNELS = 2;
  static constexpr int NUM_BYTES_PER_CHANNEL = 2;
  static constexpr int UPDATE_FREQUENCY = 60;

  static constexpr int calc_audio_buffer_size(int sample_rate) {
    return sample_rate * NUM_CHANNELS * NUM_BYTES_PER_CHANNEL / UPDATE_FREQUENCY;
  }

  // =========================================================================
  // Runtime-configured members (set by detect())
  // =========================================================================

  gpio_num_t backlight_io;
  bool reset_value;
  gpio_num_t i2s_ws_io;
  bool touch_invert_x;
  espp::Interrupt::ActiveLevel touch_interrupt_level =
#ifdef CONFIG_ESP_BOX_BOARD_WAVESHARE_28
      waveshare::touch_interrupt_level;
#else
      box::touch_interrupt_level;
#endif
  espp::Interrupt::Type touch_interrupt_type;
  bool touch_interrupt_pullup_enabled;

  BoxType box_type_{BoxType::UNKNOWN};

  I2c internal_i2c_{{.port = internal_i2c_port,
                     .sda_io_num = internal_i2c_sda,
                     .scl_io_num = internal_i2c_scl,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE}};

  espp::Interrupt::PinConfig boot_button_interrupt_pin_{
      .gpio_num = boot_button_io,
      .callback =
          [this](const auto &event) {
            if (boot_button_callback_) {
              boot_button_callback_(event);
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
      .pullup_enabled = true};

#ifndef CONFIG_ESP_BOX_BOARD_WAVESHARE_28
  espp::Interrupt::PinConfig mute_button_interrupt_pin_{
      .gpio_num = mute_pin,
      .callback =
          [this](const auto &event) {
            mute(event.active);
            if (mute_button_callback_) {
              mute_button_callback_(event);
            }
          },
      .active_level = espp::Interrupt::ActiveLevel::LOW,
      .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
      .pullup_enabled = true};
#endif

  espp::Interrupt::PinConfig touch_interrupt_pin_{
      .gpio_num = touch_interrupt,
      .callback =
          [this](const auto &event) {
            if (update_touch()) {
              if (touch_callback_) {
                touch_callback_(touchpad_data());
              }
            }
          },
      .active_level = touch_interrupt_level,
      .filter_type = espp::Interrupt::FilterType::PIN_GLITCH_FILTER,
  };

  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "esp-box interrupts",
                       .stack_size_bytes = CONFIG_ESP_BOX_INTERRUPT_STACK_SIZE}}};

  // button
  std::atomic<bool> boot_button_initialized_{false};
  button_callback_t boot_button_callback_{nullptr};
#ifndef CONFIG_ESP_BOX_BOARD_WAVESHARE_28
  std::atomic<bool> mute_button_initialized_{false};
  button_callback_t mute_button_callback_{nullptr};
#endif

  // touch
#ifdef CONFIG_ESP_BOX_BOARD_WAVESHARE_28
  std::shared_ptr<Cst328> cst328_;
#else
  std::shared_ptr<Gt911> gt911_;
  std::shared_ptr<Tt21100> tt21100_;
#endif
  std::shared_ptr<TouchpadInput> touchpad_input_;
  std::recursive_mutex touchpad_data_mutex_;
  TouchpadData touchpad_data_;
  touch_callback_t touch_callback_{nullptr};

  // display
  std::vector<Led::ChannelConfig> backlight_channel_configs_;
  std::shared_ptr<Led> backlight_;
  std::shared_ptr<Display<Pixel>> display_;
  spi_bus_config_t lcd_spi_bus_config_;
  spi_device_interface_config_t lcd_config_;
  spi_device_handle_t lcd_handle_{nullptr};
  static constexpr int spi_queue_size = 6;
  spi_transaction_t trans[spi_queue_size];
  std::atomic<int> num_queued_trans = 0;
  uint8_t *frame_buffer0_{nullptr};
  uint8_t *frame_buffer1_{nullptr};

  // sound
  std::atomic<bool> sound_initialized_{false};
  std::atomic<float> volume_{50.0f};
  std::atomic<bool> mute_{false};
  std::unique_ptr<espp::Task> audio_task_{nullptr};
  i2s_chan_handle_t audio_tx_handle{nullptr};
  i2s_chan_handle_t audio_rx_handle{nullptr};
  std::vector<uint8_t> audio_tx_buffer;
  StreamBufferHandle_t audio_tx_stream;
  i2s_std_config_t audio_std_cfg;

  // IMU
#ifndef CONFIG_ESP_BOX_BOARD_WAVESHARE_28
  std::shared_ptr<Imu> imu_;
#endif
}; // class EspBox
} // namespace espp

// for easy printing of BoxType using libfmt
template <> struct fmt::formatter<espp::EspBox::BoxType> : fmt::formatter<std::string> {
  template <typename FormatContext> auto format(espp::EspBox::BoxType c, FormatContext &ctx) const {
    std::string name;
    switch (c) {
    case espp::EspBox::BoxType::UNKNOWN:
      name = "UNKNOWN";
      break;
    case espp::EspBox::BoxType::BOX:
      name = "BOX";
      break;
    case espp::EspBox::BoxType::BOX3:
      name = "BOX3";
      break;
    case espp::EspBox::BoxType::WAVESHARE:
      name = "WAVESHARE";
      break;
    }
    return formatter<std::string>::format(name, ctx);
  }
};
