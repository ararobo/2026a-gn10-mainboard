cmake_minimum_required(VERSION 3.22)

add_library(gn10_mainboard STATIC
    src/app.cpp
    src/robomas_can.cpp
    ${CMAKE_SOURCE_DIR}/gn10_can/drivers/stm32_fdcan/driver_stm32_fdcan.cpp
)

target_include_directories(gn10_mainboard PUBLIC
    include
    ${CMAKE_SOURCE_DIR}/gn10_can
)
target_compile_features(gn10_mainboard PUBLIC cxx_std_17)

target_link_libraries(gn10_mainboard PUBLIC gn10_can)
target_link_libraries(gn10_mainboard PRIVATE stm32cubemx)
