#include "waveshare-p4-bsp.hpp"

#include <cstring>

#include <driver/ledc.h>
#include <esp_check.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_st7701.h>
#include <esp_ldo_regulator.h>
#include <esp_log.h>

#include "es8311.hpp"

static const char *TAG = "WaveshareP4Bsp";

// ─── ST7701 vendor-specific init commands ────────────────────────────────────
static const st7701_lcd_init_cmd_t vendor_specific_init_default[] = {
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xEF, (uint8_t[]){0x08}, 1, 0},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, (uint8_t[]){0x63, 0x00}, 2, 0},
    {0xC1, (uint8_t[]){0x0D, 0x02}, 2, 0},
    {0xC2, (uint8_t[]){0x17, 0x08}, 2, 0},
    {0xCC, (uint8_t[]){0x10}, 1, 0},
    {0xB0,
     (uint8_t[]){0x40, 0xC9, 0x94, 0x0E, 0x10, 0x05, 0x0B, 0x09, 0x08, 0x26, 0x04, 0x52, 0x10,
                 0x69, 0x6B, 0x69},
     16, 0},
    {0xB1,
     (uint8_t[]){0x40, 0xD2, 0x98, 0x0C, 0x92, 0x07, 0x09, 0x08, 0x07, 0x25, 0x02, 0x0E, 0x0C,
                 0x6E, 0x78, 0x55},
     16, 0},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    {0xB0, (uint8_t[]){0x5D}, 1, 0},
    {0xB1, (uint8_t[]){0x4E}, 1, 0},
    {0xB2, (uint8_t[]){0x87}, 1, 0},
    {0xB3, (uint8_t[]){0x80}, 1, 0},
    {0xB5, (uint8_t[]){0x4E}, 1, 0},
    {0xB7, (uint8_t[]){0x85}, 1, 0},
    {0xB8, (uint8_t[]){0x21}, 1, 0},
    {0xB9, (uint8_t[]){0x10, 0x1F}, 2, 0},
    {0xBB, (uint8_t[]){0x03}, 1, 0},
    {0xBC, (uint8_t[]){0x00}, 1, 0},
    {0xC1, (uint8_t[]){0x78}, 1, 0},
    {0xC2, (uint8_t[]){0x78}, 1, 0},
    {0xD0, (uint8_t[]){0x88}, 1, 0},
    {0xE0, (uint8_t[]){0x00, 0x3A, 0x02}, 3, 0},
    {0xE1,
     (uint8_t[]){0x04, 0xA0, 0x00, 0xA0, 0x05, 0xA0, 0x00, 0xA0, 0x00, 0x40, 0x40}, 11, 0},
    {0xE2,
     (uint8_t[]){0x30, 0x00, 0x40, 0x40, 0x32, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00},
     13, 0},
    {0xE3, (uint8_t[]){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE4, (uint8_t[]){0x44, 0x44}, 2, 0},
    {0xE5,
     (uint8_t[]){0x09, 0x2E, 0xA0, 0xA0, 0x0B, 0x30, 0xA0, 0xA0, 0x05, 0x2A, 0xA0, 0xA0, 0x07,
                 0x2C, 0xA0, 0xA0},
     16, 0},
    {0xE6, (uint8_t[]){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE7, (uint8_t[]){0x44, 0x44}, 2, 0},
    {0xE8,
     (uint8_t[]){0x08, 0x2D, 0xA0, 0xA0, 0x0A, 0x2F, 0xA0, 0xA0, 0x04, 0x29, 0xA0, 0xA0, 0x06,
                 0x2B, 0xA0, 0xA0},
     16, 0},
    {0xEB, (uint8_t[]){0x00, 0x00, 0x4E, 0x4E, 0x00, 0x00, 0x00}, 7, 0},
    {0xEC, (uint8_t[]){0x08, 0x01}, 2, 0},
    {0xED,
     (uint8_t[]){0xB0, 0x2B, 0x98, 0xA4, 0x56, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0x65, 0x4A,
                 0x89, 0xB2, 0x0B},
     16, 0},
    {0xEF, (uint8_t[]){0x08, 0x08, 0x08, 0x45, 0x3F, 0x54}, 6, 0},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
    {0x11, (uint8_t[]){0x00}, 0, 120},
    {0x29, (uint8_t[]){0x00}, 0, 0},
};

// ─── Panel native dimensions (portrait) ─────────────────────────────────────
static constexpr int PANEL_H_RES = 480;  // native horizontal
static constexpr int PANEL_V_RES = 800;  // native vertical

// ─── MIPI-DSI constants ──────────────────────────────────────────────────────
static constexpr int DSI_LANE_NUM = 2;
static constexpr int DSI_LANE_BITRATE_MBPS = 500;
static constexpr int DSI_PHY_LDO_CHAN = 3;
static constexpr int DSI_PHY_LDO_VOLTAGE_MV = 2500;
static constexpr int DPI_CLOCK_MHZ = 30;
static constexpr int LCD_LEDC_CH = CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH;

// ─── Constructor ─────────────────────────────────────────────────────────────

WaveshareP4Bsp::WaveshareP4Bsp() : espp::BaseComponent("WaveshareP4Bsp") {
  logger_.info("Waveshare ESP32-P4 4.3\" BSP created");
}

espp::I2c &WaveshareP4Bsp::internal_i2c() { return internal_i2c_; }

// ─── Backlight ───────────────────────────────────────────────────────────────

bool WaveshareP4Bsp::initialize_backlight() {
  const ledc_timer_config_t backlight_timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                               .duty_resolution = LEDC_TIMER_10_BIT,
                                               .timer_num = LEDC_TIMER_1,
                                               .freq_hz = 5000,
                                               .clk_cfg = LEDC_AUTO_CLK};

  const ledc_channel_config_t backlight_channel = {.gpio_num = lcd_backlight_io,
                                                   .speed_mode = LEDC_LOW_SPEED_MODE,
                                                   .channel = (ledc_channel_t)LCD_LEDC_CH,
                                                   .intr_type = LEDC_INTR_DISABLE,
                                                   .timer_sel = LEDC_TIMER_1,
                                                   .duty = 0,
                                                   .hpoint = 0,
                                                   .flags = {.output_invert = 1}};

  auto err = ledc_timer_config(&backlight_timer);
  if (err != ESP_OK) {
    logger_.error("LEDC timer config failed: {}", esp_err_to_name(err));
    return false;
  }
  err = ledc_channel_config(&backlight_channel);
  if (err != ESP_OK) {
    logger_.error("LEDC channel config failed: {}", esp_err_to_name(err));
    return false;
  }
  return true;
}

void WaveshareP4Bsp::brightness(float brightness_pct) {
  if (brightness_pct > 100.0f)
    brightness_pct = 100.0f;
  if (brightness_pct < 0.0f)
    brightness_pct = 0.0f;
  brightness_.store(brightness_pct);
  uint32_t duty = (1023 * (int)brightness_pct) / 100;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LCD_LEDC_CH, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)LCD_LEDC_CH);
}

float WaveshareP4Bsp::brightness() const { return brightness_.load(); }

// ─── MIPI-DSI Display ────────────────────────────────────────────────────────

bool WaveshareP4Bsp::initialize_dsi_panel() {
  // Power on MIPI DSI PHY via LDO
  esp_ldo_channel_handle_t phy_pwr_chan = nullptr;
  esp_ldo_channel_config_t ldo_cfg = {
      .chan_id = DSI_PHY_LDO_CHAN,
      .voltage_mv = DSI_PHY_LDO_VOLTAGE_MV,
  };
  auto err = esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan);
  if (err != ESP_OK) {
    logger_.error("Failed to acquire LDO channel for DSI PHY: {}", esp_err_to_name(err));
    return false;
  }
  logger_.info("MIPI DSI PHY powered on");

  // Create DSI bus
  esp_lcd_dsi_bus_handle_t mipi_dsi_bus = nullptr;
  esp_lcd_dsi_bus_config_t bus_config = {
      .bus_id = 0,
      .num_data_lanes = DSI_LANE_NUM,
      .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
      .lane_bit_rate_mbps = DSI_LANE_BITRATE_MBPS,
  };
  err = esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus);
  if (err != ESP_OK) {
    logger_.error("Failed to create DSI bus: {}", esp_err_to_name(err));
    return false;
  }

  // Create DBI panel IO
  esp_lcd_panel_io_handle_t io = nullptr;
  esp_lcd_dbi_io_config_t dbi_config = {
      .virtual_channel = 0,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
  };
  err = esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io);
  if (err != ESP_OK) {
    logger_.error("Failed to create DBI panel IO: {}", esp_err_to_name(err));
    return false;
  }

  // Configure DPI panel — native portrait 480x800
  esp_lcd_dpi_panel_config_t dpi_config = {};
  dpi_config.virtual_channel = 0;
  dpi_config.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
  dpi_config.dpi_clock_freq_mhz = DPI_CLOCK_MHZ;
  dpi_config.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
  dpi_config.num_fbs = CONFIG_BSP_LCD_DPI_BUFFER_NUMS;
  // Native portrait 480x800 — software rotation to landscape
  dpi_config.video_timing.h_size = 480;
  dpi_config.video_timing.v_size = 800;
  dpi_config.video_timing.hsync_back_porch = 42;
  dpi_config.video_timing.hsync_pulse_width = 12;
  dpi_config.video_timing.hsync_front_porch = 42;
  dpi_config.video_timing.vsync_back_porch = 2;
  dpi_config.video_timing.vsync_pulse_width = 8;
  dpi_config.video_timing.vsync_front_porch = 60;
  dpi_config.flags.use_dma2d = true;

  st7701_vendor_config_t vendor_config = {};
  vendor_config.init_cmds = vendor_specific_init_default;
  vendor_config.init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(st7701_lcd_init_cmd_t);
  vendor_config.mipi_config.dsi_bus = mipi_dsi_bus;
  vendor_config.mipi_config.dpi_config = &dpi_config;
  vendor_config.flags.use_mipi_interface = 1;

  esp_lcd_panel_dev_config_t lcd_dev_config = {};
  lcd_dev_config.reset_gpio_num = lcd_reset_io;
  lcd_dev_config.bits_per_pixel = 16;
  lcd_dev_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
  lcd_dev_config.vendor_config = &vendor_config;

  err = esp_lcd_new_panel_st7701(io, &lcd_dev_config, &panel_handle_);
  if (err != ESP_OK) {
    logger_.error("Failed to create ST7701 panel: {}", esp_err_to_name(err));
    return false;
  }
  logger_.info("ST7701 panel created, resetting...");
  vTaskDelay(pdMS_TO_TICKS(10));

  err = esp_lcd_panel_reset(panel_handle_);
  if (err != ESP_OK) {
    logger_.error("Panel reset failed: {}", esp_err_to_name(err));
    return false;
  }
  logger_.info("Panel reset done, initializing...");
  vTaskDelay(pdMS_TO_TICKS(10));

  err = esp_lcd_panel_init(panel_handle_);
  if (err != ESP_OK) {
    logger_.error("Panel init failed: {}", esp_err_to_name(err));
    return false;
  }

  logger_.info("MIPI-DSI display initialized (480x800 portrait)");
  return true;
}

bool WaveshareP4Bsp::initialize_lcd() {
  if (!initialize_backlight()) {
    logger_.error("Backlight init failed");
    return false;
  }
  if (!initialize_dsi_panel()) {
    logger_.error("DSI panel init failed");
    return false;
  }
  // Turn on display + backlight
  esp_lcd_panel_disp_on_off(panel_handle_, true);
  brightness(100.0f);
  return true;
}

bool WaveshareP4Bsp::initialize_display(size_t pixel_buffer_size) {
  if (display_) {
    logger_.warn("Display already initialized");
    return true;
  }

  logger_.info("Initializing display with pixel buffer size {}", pixel_buffer_size);

  // Allocate VRAM in PSRAM for LVGL rendering buffers
  size_t vram_bytes = pixel_buffer_size * sizeof(Pixel);
  vram_0_ = (Pixel *)heap_caps_malloc(vram_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  vram_1_ = (Pixel *)heap_caps_malloc(vram_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  if (!vram_0_ || !vram_1_) {
    logger_.error("Failed to allocate VRAM (2x {} bytes)", vram_bytes);
    return false;
  }
  memset(vram_0_, 0, vram_bytes);
  memset(vram_1_, 0, vram_bytes);

  // Allocate application frame buffers in PSRAM
  size_t fb_size = lcd_width() * lcd_height();
  frame_buffer0_ = (uint8_t *)heap_caps_malloc(fb_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  frame_buffer1_ = (uint8_t *)heap_caps_malloc(fb_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  if (!frame_buffer0_ || !frame_buffer1_) {
    logger_.error("Failed to allocate frame buffers (2x {} bytes)", fb_size);
    return false;
  }
  memset(frame_buffer0_, 0, fb_size);
  memset(frame_buffer1_, 0, fb_size);

  // Allocate rotation scratch buffer in internal SRAM for fast writes
  size_t rot_buf_size = pixel_buffer_size * sizeof(Pixel);
  auto *rot_buf = (Pixel *)heap_caps_malloc(rot_buf_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!rot_buf) {
    // Fallback to PSRAM if SRAM not available
    rot_buf = (Pixel *)heap_caps_malloc(rot_buf_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  }
  assert(rot_buf && "Failed to allocate rotation buffer");
  rot_buf_ = rot_buf;

  // Flush callback: rotate 90° CW from landscape (800x480) to portrait (480x800)
  // Uses tile-based rotation for cache-friendly access patterns.
  auto panel = panel_handle_;
  auto flush_fn = [panel, rot_buf](lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
    int x1 = area->x1;
    int y1 = area->y1;
    int w = area->x2 - x1 + 1;
    int h = area->y2 - y1 + 1;
    auto *src = (Pixel *)color_map;

    // Rotate 90° CW: landscape (lx, ly) → portrait (PANEL_H_RES-1-ly, lx)
    int px_start = PANEL_H_RES - 1 - area->y2;
    int py_start = x1;

    // Tile-based rotation: process in 8x8 tiles for cache locality
    constexpr int TILE = 8;
    for (int ty = 0; ty < h; ty += TILE) {
      int th = (ty + TILE <= h) ? TILE : h - ty;
      for (int tx = 0; tx < w; tx += TILE) {
        int tw = (tx + TILE <= w) ? TILE : w - tx;
        for (int ly = 0; ly < th; ly++) {
          for (int lx = 0; lx < tw; lx++) {
            rot_buf[(tx + lx) * h + (h - 1 - (ty + ly))] = src[(ty + ly) * w + (tx + lx)];
          }
        }
      }
    }
    esp_lcd_panel_draw_bitmap(panel, px_start, py_start, px_start + h, py_start + w, rot_buf);
    lv_display_flush_ready(disp);
  };

  auto set_brightness_fn = [this](float b) { brightness(b * 100.0f); };
  auto get_brightness_fn = [this]() -> float { return brightness() / 100.0f; };

  display_ = std::make_shared<espp::Display<Pixel>>(
      espp::Display<Pixel>::LvglConfig{
          .width = lcd_width(),
          .height = lcd_height(),
          .flush_callback = flush_fn,
          .rotation = espp::DisplayRotation::LANDSCAPE,
      },
      espp::Display<Pixel>::OledConfig{
          .set_brightness_callback = set_brightness_fn,
          .get_brightness_callback = get_brightness_fn,
      },
      espp::Display<Pixel>::StaticMemoryConfig{
          .pixel_buffer_size = pixel_buffer_size,
          .vram0 = vram_0_,
          .vram1 = vram_1_,
      });

  logger_.info("Display initialized");
  return true;
}

std::shared_ptr<espp::Display<WaveshareP4Bsp::Pixel>> WaveshareP4Bsp::display() const {
  return display_;
}

WaveshareP4Bsp::Pixel *WaveshareP4Bsp::vram0() const { return vram_0_; }
WaveshareP4Bsp::Pixel *WaveshareP4Bsp::vram1() const { return vram_1_; }
uint8_t *WaveshareP4Bsp::frame_buffer0() const { return frame_buffer0_; }
uint8_t *WaveshareP4Bsp::frame_buffer1() const { return frame_buffer1_; }

void WaveshareP4Bsp::write_lcd_frame(const uint16_t x, const uint16_t y, const uint16_t width,
                                      const uint16_t height, uint8_t *data) {
  if (!panel_handle_ || !data || !rot_buf_) {
    return;
  }
  auto *src = (Pixel *)data;
  // Rotate 90° CW: landscape (lx, ly) → portrait (PANEL_H_RES-1-ly, lx)
  // Input strip: landscape coords (x, y, width, height)
  // Output rect in portrait: (PANEL_H_RES-1-(y+height-1), x, height, width)
  int px_start = PANEL_H_RES - 1 - (y + height - 1);
  int py_start = x;
  for (int ly = 0; ly < height; ly++) {
    for (int lx = 0; lx < width; lx++) {
      int dst_x = (height - 1 - ly);
      int dst_y = lx;
      rot_buf_[dst_y * height + dst_x] = src[ly * width + lx];
    }
  }
  esp_lcd_panel_draw_bitmap(panel_handle_, px_start, py_start,
                            px_start + height, py_start + width, rot_buf_);
}

void WaveshareP4Bsp::write_lcd_full_frame(const uint16_t x, const uint16_t y,
                                           const uint16_t width, const uint16_t height,
                                           const uint16_t *data) {
  if (!panel_handle_ || !data) {
    return;
  }
  // Lazy-allocate PSRAM rotation buffer (height × width because 90° CW swaps dims)
  size_t need = (size_t)width * height * sizeof(Pixel);
  if (!full_rot_buf_ || full_rot_buf_size_ < need) {
    if (full_rot_buf_) heap_caps_free(full_rot_buf_);
    full_rot_buf_ = (Pixel *)heap_caps_malloc(need, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    full_rot_buf_size_ = full_rot_buf_ ? need : 0;
    if (!full_rot_buf_) {
      // Fallback to strip-based write
      write_lcd_frame(x, y, width, height, (uint8_t *)data);
      return;
    }
  }

  auto *src = (const Pixel *)data;
  auto *dst = full_rot_buf_;

  // Tile-based 90° CW rotation: landscape (lx, ly) → portrait (height-1-ly, lx)
  // Output buffer layout: dst[py * height + px] where py=lx, px=height-1-ly
  constexpr int TILE = 8;
  for (int ty = 0; ty < height; ty += TILE) {
    int th = (ty + TILE <= height) ? TILE : height - ty;
    for (int tx = 0; tx < width; tx += TILE) {
      int tw = (tx + TILE <= width) ? TILE : width - tx;
      for (int ly = 0; ly < th; ly++) {
        for (int lx = 0; lx < tw; lx++) {
          dst[(tx + lx) * height + (height - 1 - (ty + ly))] =
              src[(ty + ly) * width + (tx + lx)];
        }
      }
    }
  }

  int px_start = PANEL_H_RES - 1 - (y + height - 1);
  int py_start = x;
  esp_lcd_panel_draw_bitmap(panel_handle_, px_start, py_start,
                            px_start + height, py_start + width, dst);
}

// ─── Touch ───────────────────────────────────────────────────────────────────

bool WaveshareP4Bsp::initialize_touch(const touch_callback_t &callback) {
  if (gt911_) {
    logger_.warn("Touch already initialized");
    return true;
  }

  touch_callback_ = callback;

  logger_.info("Initializing GT911 touch controller");

  // Probe GT911 address
  uint8_t touch_addr = espp::Gt911::DEFAULT_ADDRESS_1; // 0x5D
  if (!internal_i2c_.probe_device(touch_addr)) {
    touch_addr = espp::Gt911::DEFAULT_ADDRESS_2; // 0x14
    if (!internal_i2c_.probe_device(touch_addr)) {
      logger_.error("GT911 not found at 0x5D or 0x14");
      return false;
    }
  }
  logger_.info("GT911 found at 0x{:02x}", touch_addr);

  gt911_ = std::make_shared<espp::Gt911>(espp::Gt911::Config{
      .write = std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                         std::placeholders::_2, std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, &internal_i2c_, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3),
      .address = touch_addr,
      .log_level = espp::Logger::Verbosity::WARN,
  });

  touchpad_input_ = std::make_shared<espp::TouchpadInput>(espp::TouchpadInput::Config{
      .touchpad_read =
          std::bind(&WaveshareP4Bsp::touchpad_read, this, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      .log_level = espp::Logger::Verbosity::WARN,
  });

  logger_.info("Touch initialized");
  return true;
}

bool WaveshareP4Bsp::update_touch() {
  if (!gt911_) {
    return false;
  }
  std::error_code ec;
  bool new_data = gt911_->update(ec);
  if (ec) {
    return false;
  }
  if (!new_data) {
    return false;
  }

  uint8_t num_touch_points = 0;
  uint16_t tx = 0, ty = 0;
  gt911_->get_touch_point(&num_touch_points, &tx, &ty);

  // Rotate touch from portrait (480x800) to landscape (800x480)
  // Same 90° CW as display: landscape_x = ty, landscape_y = (PANEL_H_RES-1) - tx
  TouchpadData data;
  data.x = ty;
  data.y = (PANEL_H_RES - 1) - tx;
  data.btn_state = num_touch_points > 0 ? 1 : 0;
  data.num_touch_points = num_touch_points;

  {
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = data;
  }

  if (touch_callback_) {
    touch_callback_(data);
  }

  return true;
}

WaveshareP4Bsp::TouchpadData WaveshareP4Bsp::touchpad_data() const {
  // NOTE: caller should know this is a snapshot
  return touchpad_data_;
}

std::shared_ptr<espp::TouchpadInput> WaveshareP4Bsp::touchpad_input() const {
  return touchpad_input_;
}

void WaveshareP4Bsp::touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y,
                                    uint8_t *btn_state) {
  update_touch();
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  *num_touch_points = touchpad_data_.num_touch_points;
  *x = touchpad_data_.x;
  *y = touchpad_data_.y;
  *btn_state = touchpad_data_.btn_state;
}

// ─── Audio ───────────────────────────────────────────────────────────────────

bool WaveshareP4Bsp::initialize_codec() {
  logger_.info("Initializing ES8311 codec");

  set_es8311_write(std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3));
  set_es8311_read(std::bind(&espp::I2c::read_at_register, &internal_i2c_, std::placeholders::_1,
                            std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  esp_err_t ret_val = ESP_OK;
  audio_hal_codec_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.codec_mode = AUDIO_HAL_CODEC_MODE_DECODE;
  cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_ALL;
  cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  cfg.i2s_iface.samples = AUDIO_HAL_48K_SAMPLES;

  ret_val |= es8311_codec_init(&cfg);
  ret_val |= es8311_config_fmt(ES_I2S_NORMAL);
  ret_val |= es8311_set_bits_per_sample(AUDIO_HAL_BIT_LENGTH_16BITS);
  ret_val |= es8311_codec_set_voice_volume(volume_);
  ret_val |= es8311_codec_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START);

  if (ret_val != ESP_OK) {
    logger_.error("ES8311 codec initialization failed");
    return false;
  }

  logger_.info("ES8311 codec initialized");
  return true;
}

bool WaveshareP4Bsp::initialize_i2s(uint32_t default_audio_rate) {
  logger_.info("Initializing I2S driver");

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_port, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &audio_tx_handle_, &audio_rx_handle_));

  audio_std_cfg_ = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(default_audio_rate),
      .slot_cfg =
          I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {.mclk = i2s_mck_io,
                   .bclk = i2s_bck_io,
                   .ws = i2s_ws_io,
                   .dout = i2s_do_io,
                   .din = i2s_di_io,
                   .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
  };

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_tx_handle_, &audio_std_cfg_));

  auto buffer_size = calc_audio_buffer_size(default_audio_rate);
  audio_tx_buffer_.resize(buffer_size);

  audio_tx_stream_ = xStreamBufferCreate(buffer_size * 4, 0);
  xStreamBufferReset(audio_tx_stream_);

  ESP_ERROR_CHECK(i2s_channel_enable(audio_tx_handle_));

  return true;
}

bool WaveshareP4Bsp::initialize_sound(uint32_t default_audio_rate,
                                       const espp::Task::BaseConfig &task_config) {
  if (sound_initialized_) {
    logger_.warn("Sound already initialized");
    return true;
  }

  if (!initialize_i2s(default_audio_rate)) {
    logger_.error("Could not initialize I2S driver");
    return false;
  }
  if (!initialize_codec()) {
    logger_.error("Could not initialize codec");
    return false;
  }

  // Power amplifier control
  gpio_set_direction(power_amp_io, GPIO_MODE_OUTPUT);
  enable_sound(true);

  using namespace std::placeholders;
  audio_task_ = espp::Task::make_unique({
      .callback = std::bind(&WaveshareP4Bsp::audio_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });

  sound_initialized_ = true;
  return audio_task_->start();
}

void WaveshareP4Bsp::enable_sound(bool enable) { gpio_set_level(power_amp_io, enable); }

bool WaveshareP4Bsp::audio_task_callback(std::mutex &m, std::condition_variable &cv,
                                          bool &task_notified) {
  uint16_t available = xStreamBufferBytesAvailable(audio_tx_stream_);
  int buffer_size = audio_tx_buffer_.size();
  available = std::min<uint16_t>(available, buffer_size);
  uint8_t *buffer = &audio_tx_buffer_[0];
  memset(buffer, 0, buffer_size);

  if (available == 0) {
    i2s_channel_write(audio_tx_handle_, buffer, buffer_size, NULL, portMAX_DELAY);
  } else {
    xStreamBufferReceive(audio_tx_stream_, buffer, available, 0);
    i2s_channel_write(audio_tx_handle_, buffer, buffer_size, NULL, portMAX_DELAY);
  }
  return false;
}

void WaveshareP4Bsp::update_volume_output() {
  if (!sound_initialized_) {
    return;
  }
  if (mute_) {
    es8311_codec_set_voice_volume(0);
  } else {
    es8311_codec_set_voice_volume(volume_);
  }
}

void WaveshareP4Bsp::mute(bool m) {
  mute_ = m;
  update_volume_output();
}

bool WaveshareP4Bsp::is_muted() const { return mute_; }

void WaveshareP4Bsp::volume(float v) {
  volume_ = v;
  update_volume_output();
}

float WaveshareP4Bsp::volume() const { return volume_; }

uint32_t WaveshareP4Bsp::audio_sample_rate() const { return audio_std_cfg_.clk_cfg.sample_rate_hz; }

void WaveshareP4Bsp::audio_sample_rate(uint32_t sample_rate) {
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  i2s_channel_disable(audio_tx_handle_);
  es8311_codec_set_sample_rate(sample_rate);
  audio_std_cfg_.clk_cfg.sample_rate_hz = sample_rate;
  i2s_channel_reconfig_std_clock(audio_tx_handle_, &audio_std_cfg_.clk_cfg);
  xStreamBufferReset(audio_tx_stream_);
  i2s_channel_enable(audio_tx_handle_);
}

void WaveshareP4Bsp::play_audio(const std::vector<uint8_t> &data) {
  play_audio(data.data(), data.size());
}

void WaveshareP4Bsp::play_audio(const uint8_t *data, uint32_t num_bytes) {
  xStreamBufferSendFromISR(audio_tx_stream_, data, num_bytes, NULL);
}
