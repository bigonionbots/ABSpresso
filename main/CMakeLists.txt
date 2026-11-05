idf_component_register(SRC_DIRS "." "hardware" "network" "app" "ui" "ui/widgets" "ui/pages" "ui/fonts" "module"
                       INCLUDE_DIRS "." "hardware" "network" "app" "ui" "ui/widgets" "module"
                       # lv_mem_psram.c provides functions LVGL calls; keep them despite link order.
                       WHOLE_ARCHIVE)
