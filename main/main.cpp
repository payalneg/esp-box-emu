// #define TOUCH_TEST_ENABLED
#include <sdkconfig.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include <stdio.h>

#include "logger.hpp"
#include "task_monitor.hpp"
#include "timer.hpp"

#include "box-emu.hpp"
#include "carts.hpp"
#include "gui.hpp"
#include "heap_utils.hpp"
#include "rom_info.hpp"
#include "statistics.hpp"

#include "lvgl.h"
#include "esp_heap_caps.h"

using namespace std::chrono_literals;

#ifdef TOUCH_TEST_ENABLED
static void touch_test() {
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_white(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  fmt::print("[TOUCH_TEST] Touch the screen. Test runs for 30 seconds.\n");
  auto &box = BoxEmu::Bsp::get();
  int dot_count = 0;
  int16_t last_x = -10, last_y = -10;
  auto start = xTaskGetTickCount();
  while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(30000)) {
    box.update_touch();
    uint8_t num_points;
    uint16_t x, y;
    uint8_t btn;
    box.touchpad_read(&num_points, &x, &y, &btn);
    if (num_points > 0 && dot_count < 500) {
      // Only draw if moved at least 3 pixels from last dot
      int dx = (int)x - last_x;
      int dy = (int)y - last_y;
      if (dx * dx + dy * dy >= 9) {
        lv_obj_t *dot = lv_obj_create(scr);
        lv_obj_remove_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(dot, 6, 6);
        lv_obj_set_pos(dot, x - 3, y - 3);
        lv_obj_set_style_bg_color(dot, lv_color_black(), 0);
        lv_obj_set_style_radius(dot, 3, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        lv_obj_set_style_pad_all(dot, 0, 0);
        last_x = x;
        last_y = y;
        dot_count++;
      }
    }
    lv_task_handler();
    vTaskDelay(pdMS_TO_TICKS(16));
  }

  lv_obj_clean(scr);
  fmt::print("[TOUCH_TEST] Done.\n");
}
#endif

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "esp-box-emu", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup");

  // initialize the hardware abstraction layer
  BoxEmu &emu = BoxEmu::get();
  logger.info("Box Emu version: {}", emu.version());

  // initialize
  if (!emu.initialize_box()) {
    logger.error("Failed to initialize box!");
    return;
  }

  if (!emu.initialize_sdcard()) {
    logger.warn("Failed to initialize SD card!");
    logger.warn("This may happen if the SD card is not inserted.");
  }

  if (!emu.initialize_memory()) {
    logger.error("Failed to initialize memory!");
    return;
  }

  if (!emu.initialize_gamepad()) {
    logger.warn("Failed to initialize gamepad!");
    logger.warn("This may happen if the gamepad is not connected.");
  }

  if (!emu.initialize_battery()) {
    logger.warn("Failed to initialize battery!");
    logger.warn("This may happen if the battery is not connected.");
  }

  if (!emu.initialize_video()) {
    logger.error("Failed to initialize video!");
    return;
  }

  if (!emu.initialize_haptics()) {
    logger.warn("Failed to initialize haptics!");
    logger.warn("This may happen if the gamepad is not connected.");
  }

#ifdef TOUCH_TEST_ENABLED
  touch_test();
#endif

  logger.info("initializing gui...");

  auto display = BoxEmu::Bsp::get().display();

  // initialize the gui
  Gui gui({
      .play_haptic = [&emu]() { emu.play_haptic_effect(); },
      .set_waveform = [&emu](uint8_t waveform) { emu.set_haptic_effect(waveform); },
      .log_level = espp::Logger::Verbosity::WARN
    });

  print_heap_state();

  // set the task priority (for main) to high
  vTaskPrioritySet(nullptr, 20);

  // main loop
  while (true) {
    // reset gui ready to play and user_quit
    gui.ready_to_play(false);
    while (!gui.ready_to_play()) {
      std::this_thread::sleep_for(50ms);
    }

    // have broken out of the loop, let the user know we're processing...
    emu.set_haptic_effect(gui.get_haptic_waveform());
    emu.play_haptic_effect();

    gui.pause();

    auto maybe_selected_rom = gui.get_selected_rom();
    if (maybe_selected_rom.has_value()) {
      auto selected_rom = maybe_selected_rom.value();
      logger.info("Selected rom:\n\t{}", selected_rom);

      print_heap_state();

      // Cart handles platform specific code, state management, etc.
      {
        std::unique_ptr<Cart> cart(make_cart(selected_rom, display));
        if (cart) {
          while (cart->run());
        } else {
          logger.error("Failed to create cart!");
        }
      }
    } else {
      logger.error("Invalid rom selected!");
    }

    // print the frame statistics from the previous run
    print_statistics();

    logger.info("Done playing, resuming gui...");

    logger.debug("Task table:\n{}", espp::TaskMonitor::get_latest_info_table());

    gui.resume();
    display->force_refresh();
  }
}
