idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES
  "driver"
  "esp_lcd"
  "esp_psram"
  "hal"
  "lvgl"
  "base_component"
  "display"
  "i2c"
  "input_drivers"
  "gt911"
  "task"
  "codec"
  PRIV_REQUIRES
  "esp_timer"
)
