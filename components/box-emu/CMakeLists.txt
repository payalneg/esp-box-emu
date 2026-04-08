# IDF_TARGET is set as a CMake variable before component processing
if(IDF_TARGET STREQUAL "esp32p4")
  set(BSP_REQUIRES "waveshare-p4-bsp")
  set(S3_ONLY_REQUIRES "")
elseif(IDF_TARGET STREQUAL "esp32s3")
  set(BSP_REQUIRES "esp-box")
  set(S3_ONLY_REQUIRES
    "aw9523"
    "mcp23x17"
    "drv2605"
    "max1704x"
    "tt21100"
    "gt911"
    "button"
    "adc"
    "display_drivers"
    "usb"
    "esp_tinyusb"
  )
else()
  # Fallback: try P4 path
  set(BSP_REQUIRES "waveshare-p4-bsp")
  set(S3_ONLY_REQUIRES "")
endif()

message(STATUS "box-emu: IDF_TARGET=${IDF_TARGET}, BSP_REQUIRES=${BSP_REQUIRES}")

idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES
  "driver"
  "heap"
  "fatfs"
  "esp_lcd"
  "esp_psram"
  "hal"
  "lvgl"
  "codec"
  "input_drivers"
  "display"
  "event_manager"
  "events"
  "task"
  "timer"
  "serialization"
  "statistics"
  "pool_allocator"
  ${BSP_REQUIRES}
  ${S3_ONLY_REQUIRES}
  )
