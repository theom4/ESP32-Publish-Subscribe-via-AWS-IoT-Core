# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/components/common/posix_compat)
project(aws_2)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "tmp/root_cert_auth.crt" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "tmp/client.crt" TEXT)
target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "tmp/client.key" TEXT)

