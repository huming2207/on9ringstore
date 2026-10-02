cmake_minimum_required(VERSION 3.22)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/..")
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Linux TickType_t is 64-bit; on9rstore takes uint32_t timeouts like the ESP targets
idf_build_set_property(COMPILE_OPTIONS "-Wno-overflow" APPEND)
project(on9rstore_test)
