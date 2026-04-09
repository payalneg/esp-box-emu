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
#include "sd_ota.hpp"
#include "statistics.hpp"

using namespace std::chrono_literals;

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

  if (emu.initialize_sdcard()) {
    // Check for firmware update on SD card (reboots if found)
    check_sd_ota(BoxEmu::mount_point);
  } else {
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

#ifndef BOARD_WAVESHARE_P4
  if (!emu.initialize_battery()) {
    logger.warn("Failed to initialize battery!");
    logger.warn("This may happen if the battery is not connected.");
  }
#endif

  if (!emu.initialize_video()) {
    logger.error("Failed to initialize video!");
    return;
  }

  if (!emu.initialize_haptics()) {
    logger.warn("Failed to initialize haptics!");
    logger.warn("This may happen if the gamepad is not connected.");
  }

  if (!emu.initialize_usb()) {
    logger.warn("Failed to initialize USB MSC!");
  }

  logger.info("initializing gui...");

  auto display = BoxEmu::Bsp::get().display();

  // initialize the gui
  Gui gui({
      .play_haptic = [&emu]() { emu.play_haptic_effect(); },
      .set_waveform = [&emu](uint8_t waveform) { emu.set_haptic_effect(waveform); },
      .log_level = espp::Logger::Verbosity::WARN
    });

  // Confirm OTA rollback — this app booted successfully
  esp_ota_mark_app_valid_cancel_rollback();

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

    // Clear screen to black before launching game
    {
      size_t buf_size = BoxEmu::lcd_width() * 30 * sizeof(BoxEmu::Pixel);
      auto *black_buf = (uint8_t *)heap_caps_calloc(1, buf_size, MALLOC_CAP_SPIRAM);
      if (black_buf) {
        auto &bsp = BoxEmu::Bsp::get();
        for (int y = 0; y < (int)BoxEmu::lcd_height(); y += 30) {
          int h = std::min(30, (int)BoxEmu::lcd_height() - y);
          bsp.write_lcd_frame(0, y, BoxEmu::lcd_width(), h, black_buf);
        }
        free(black_buf);
      }
    }

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
