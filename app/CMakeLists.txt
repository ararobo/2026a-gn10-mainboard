cmake_minimum_required(VERSION 3.22)

add_library(app STATIC
    src/app.cpp
    src/serial_printf.cpp
    src/robot_ethernet.cpp
    src/conversion_command.cpp
    src/bucket_arm_controller.cpp
    src/belt_launcher_controller.cpp
)

target_include_directories(app PUBLIC
    include
)
target_compile_features(app PUBLIC cxx_std_17)

target_link_libraries(app PUBLIC gn10_can)
target_link_libraries(app PRIVATE wiznet_ether)
target_link_libraries(app PRIVATE stm32cubemx)
target_link_libraries(app PRIVATE gn10_stm32_fdcan_driver)