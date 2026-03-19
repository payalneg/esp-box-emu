#include "esp-box.hpp"

using namespace espp;

////////////////////////
// Touchpad Functions //
////////////////////////

bool EspBox::initialize_touch(const EspBox::touch_callback_t &callback) {
#ifdef CONFIG_ESP_BOX_BOARD_WAVESHARE_28
  if (cst328_) {
    logger_.warn("Touch already initialized, not initializing again!");
    return false;
  }

  fmt::print("[CST328] Initializing CST328 touch controller\n");
  fmt::print("[CST328] I2C: SDA={}, SCL={}\n", (int)internal_i2c_sda, (int)internal_i2c_scl);

  // Reset the touch controller via GPIO
  fmt::print("[CST328] Resetting via GPIO {}\n", (int)touch_reset_io);
  gpio_set_direction(touch_reset_io, GPIO_MODE_OUTPUT);
  gpio_set_level(touch_reset_io, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level(touch_reset_io, 1);
  vTaskDelay(pdMS_TO_TICKS(50));

  // Probe for CST328 on I2C
  bool found = internal_i2c_.probe_device(espp::Cst328::DEFAULT_ADDRESS);
  fmt::print("[CST328] Probe at 0x{:02x}: {}\n", espp::Cst328::DEFAULT_ADDRESS, found ? "FOUND" : "NOT FOUND");

  if (!found) {
    // Try alternate addresses
    for (uint8_t addr : {0x15, 0x5A, 0x1A}) {
      found = internal_i2c_.probe_device(addr);
      fmt::print("[CST328] Probe at 0x{:02x}: {}\n", addr, found ? "FOUND" : "NOT FOUND");
      if (found) break;
    }
  }

  cst328_ = std::make_shared<espp::Cst328>(espp::Cst328::Config{
      .write = std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                         std::placeholders::_2, std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, &internal_i2c_, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3),
      .log_level = espp::Logger::Verbosity::INFO});

#else
  if (gt911_ || tt21100_) {
    logger_.warn("Touch already initialized, not initializing again!");
    return false;
  }

  switch (box_type_) {
  case BoxType::BOX3:
    logger_.info("Initializing GT911");
    gt911_ = std::make_unique<espp::Gt911>(espp::Gt911::Config{
        .write = std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                           std::placeholders::_2, std::placeholders::_3),
        .read = std::bind(&espp::I2c::read, &internal_i2c_, std::placeholders::_1,
                          std::placeholders::_2, std::placeholders::_3),
        .log_level = espp::Logger::Verbosity::WARN});
    break;
  case BoxType::BOX:
    logger_.info("Initializing TT21100");
    tt21100_ = std::make_unique<espp::Tt21100>(espp::Tt21100::Config{
        .write = std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                           std::placeholders::_2, std::placeholders::_3),
        .read = std::bind(&espp::I2c::read, &internal_i2c_, std::placeholders::_1,
                          std::placeholders::_2, std::placeholders::_3),
        .log_level = espp::Logger::Verbosity::WARN});
    break;
  default:
    return false;
  }
#endif

  // store the callback
  touch_callback_ = callback;

  // add the touch interrupt pin
  interrupts_.add_interrupt(touch_interrupt_pin_);

  return true;
}

#ifdef CONFIG_ESP_BOX_BOARD_WAVESHARE_28

bool EspBox::update_cst328() {
  if (!cst328_) {
    return false;
  }
  static int zero_count = 0;
  std::error_code ec;
  bool new_data = cst328_->update(ec);
  if (ec) {
    static int err_count = 0;
    if (err_count++ < 5) {
      logger_.error("could not update cst328: {}", ec.message());
    }
    return false;
  }
  if (new_data) {
    zero_count = 0;
    TouchpadData temp_data;
    cst328_->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y);
    temp_data.btn_state = cst328_->get_home_button_state();
    if (temp_data.num_touch_points > 0) {
      fmt::print("[TOUCH] x={}, y={}, points={}\n", temp_data.x, temp_data.y, temp_data.num_touch_points);
    }
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = temp_data;
    return true;
  }
  // No data — only clear after several consecutive zero reads
  // (CST328 needs time to re-sample after acknowledge)
  if (++zero_count >= 3) {
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
  }
  return false;
}

#else

bool EspBox::update_gt911() {
  if (!gt911_) {
    return false;
  }
  std::error_code ec;
  bool new_data = gt911_->update(ec);
  if (ec) {
    logger_.error("could not update gt911: {}\n", ec.message());
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
    return false;
  }
  if (!new_data) {
    return false;
  }
  TouchpadData temp_data;
  gt911_->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y);
  temp_data.btn_state = gt911_->get_home_button_state();
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  touchpad_data_ = temp_data;
  return true;
}

bool EspBox::update_tt21100() {
  if (!tt21100_) {
    return false;
  }
  std::error_code ec;
  bool new_data = tt21100_->update(ec);
  if (ec) {
    logger_.error("could not update tt21100: {}\n", ec.message());
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
    return false;
  }
  if (!new_data) {
    return false;
  }
  TouchpadData temp_data;
  tt21100_->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y);
  temp_data.btn_state = tt21100_->get_home_button_state();
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  touchpad_data_ = temp_data;
  return true;
}

#endif

bool EspBox::update_touch() {
#ifdef CONFIG_ESP_BOX_BOARD_WAVESHARE_28
  return update_cst328();
#else
  switch (box_type_) {
  case BoxType::BOX3:
    return update_gt911();
  case BoxType::BOX:
    return update_tt21100();
  default:
    return false;
  }
#endif
}

void EspBox::touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y,
                           uint8_t *btn_state) {
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  *num_touch_points = touchpad_data_.num_touch_points;
  *x = touchpad_data_.x;
  *y = touchpad_data_.y;
  *btn_state = touchpad_data_.btn_state;
}

EspBox::TouchpadData EspBox::touchpad_convert(const EspBox::TouchpadData &data) const {
  TouchpadData temp_data;
  temp_data.num_touch_points = data.num_touch_points;
  temp_data.x = data.x;
  temp_data.y = data.y;
  temp_data.btn_state = data.btn_state;
  if (temp_data.num_touch_points == 0) {
    return temp_data;
  }
  if (touch_swap_xy) {
    std::swap(temp_data.x, temp_data.y);
  }
  if (touch_invert_x) {
    temp_data.x = lcd_width_ - (temp_data.x + 1);
  }
  if (touch_invert_y) {
    temp_data.y = lcd_height_ - (temp_data.y + 1);
  }
  // get the orientation of the display
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  switch (rotation) {
  case LV_DISPLAY_ROTATION_0:
    break;
  case LV_DISPLAY_ROTATION_90:
    temp_data.y = lcd_height_ - (temp_data.y + 1);
    std::swap(temp_data.x, temp_data.y);
    break;
  case LV_DISPLAY_ROTATION_180:
    temp_data.x = lcd_width_ - (temp_data.x + 1);
    temp_data.y = lcd_height_ - (temp_data.y + 1);
    break;
  case LV_DISPLAY_ROTATION_270:
    temp_data.x = lcd_width_ - (temp_data.x + 1);
    std::swap(temp_data.x, temp_data.y);
    break;
  default:
    break;
  }
  return temp_data;
}
