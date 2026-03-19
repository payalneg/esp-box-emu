#pragma once

#include <atomic>
#include <functional>

#include "base_peripheral.hpp"

namespace espp {
/// @brief Driver for the CST328 capacitive touch controller
/// Used on Waveshare ESP32-S3-Touch-LCD-2.8
/// Based on official Waveshare CST328.c driver
class Cst328 : public BasePeripheral<std::uint16_t> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x1A;

  struct Config {
    BasePeripheral::write_fn write;
    BasePeripheral::read_fn read;
    uint8_t address = DEFAULT_ADDRESS;
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN};
  };

  explicit Cst328(const Config &config)
      : BasePeripheral({.address = config.address, .write = config.write, .read = config.read},
                       "Cst328", config.log_level) {}

  /// @brief Update the state of the CST328 driver
  /// @param ec Error code to set if an error occurs
  /// @return True if the CST328 has new data, false otherwise
  bool update(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    static const uint8_t clear = 0x00;

    // Step 1: Read finger count from 0xD005 (1 byte)
    uint8_t num_reg = 0;
    read_many_from_register((uint16_t)Registers::NUM_POINTS, &num_reg, 1, ec);
    if (ec) {
      return false;
    }

    uint8_t num = num_reg & 0x0F;
    if (num == 0 || num > MAX_CONTACTS) {
      // No touch or invalid — clear and return
      num_touch_points_ = 0;
      write_many_to_register((uint16_t)Registers::NUM_POINTS, &clear, 1, ec);
      return false;
    }

    // Step 2: Read touch data from 0xD000 (27 bytes for up to 5 fingers)
    // Layout per finger (5 bytes): [id+state, x_hi, y_hi, x_lo4+y_lo4, pressure]
    // Finger 1: buf[0..4], then buf[5]=num_points, buf[6]=0xAB sync
    // Finger 2: buf[7..11], etc.
    static uint8_t buf[27];
    read_many_from_register((uint16_t)Registers::TOUCH_DATA, buf, 27, ec);
    if (ec) {
      write_many_to_register((uint16_t)Registers::NUM_POINTS, &clear, 1, ec);
      return false;
    }

    // Step 3: Clear touch data
    write_many_to_register((uint16_t)Registers::NUM_POINTS, &clear, 1, ec);

    // Step 4: Parse first finger coordinates
    // Waveshare format: x = (buf[1] << 4) | ((buf[3] >> 4) & 0x0F)
    //                   y = (buf[2] << 4) | (buf[3] & 0x0F)
    num_touch_points_ = 1;
    y_ = 240 - (((uint16_t)buf[1] << 4) | ((buf[3] >> 4) & 0x0F));
    x_ = ((uint16_t)buf[2] << 4) | (buf[3] & 0x0F);

    logger_.debug("Touch ({}, {}), points={}", x_.load(), y_.load(), num);

    return true;
  }

  /// @brief Get the number of touch points
  uint8_t get_num_touch_points() const { return num_touch_points_; }

  /// @brief Get the touch point data
  void get_touch_point(uint8_t *num_touch_points, uint16_t *x, uint16_t *y) const {
    *num_touch_points = get_num_touch_points();
    if (*num_touch_points != 0) {
      *x = x_;
      *y = y_;
    }
  }

  /// @brief CST328 has no home button
  bool get_home_button_state() const { return false; }

protected:
  static constexpr int MAX_CONTACTS = 5;

  enum class Registers : uint16_t {
    TOUCH_DATA = 0xD000,
    NUM_POINTS = 0xD005,
    NORMAL_MODE = 0xD109,
  };

  std::atomic<uint8_t> num_touch_points_{0};
  std::atomic<uint16_t> x_{0};
  std::atomic<uint16_t> y_{0};
};
} // namespace espp
