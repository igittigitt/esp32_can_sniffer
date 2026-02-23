cmake_minimum_required(VERSION 3.16)
# Include local components directory
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32_can_sniffer)
#nvs_create_partition_image(nvs nvs.csv FLASH_IN_PROJECT)
