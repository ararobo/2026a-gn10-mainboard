cmake_minimum_required(VERSION 3.22)

add_library(robomas_can STATIC
    src/c620_can.cpp
    src/c610_can.cpp
    src/robomas_can.cpp
    ${CMAKE_SOURCE_DIR}/gn10_can/drivers/stm32_fdcan/driver_stm32_fdcan.cpp

)

target_include_directories(robomas_can PUBLIC
    include
    ${CMAKE_SOURCE_DIR}/gn10_can
)

target_compile_features(robomas_can PUBLIC cxx_std_17)

target_link_libraries(robomas_can PUBLIC gn10_can)
target_link_libraries(robomas_can PRIVATE stm32cubemx)