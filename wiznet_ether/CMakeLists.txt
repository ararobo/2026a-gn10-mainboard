cmake_minimum_required(VERSION 3.22)

add_library(wiznet_ether STATIC
    src/robot_ethernet.cpp
    src/socket.cpp
    src/w5500_spi.cpp
    src/w5500.cpp
    src/wizchip_conf.cpp
)

target_include_directories(wiznet_ether PUBLIC
    include
)
target_compile_features(wiznet_ether PUBLIC cxx_std_17)

target_link_libraries(wiznet_ether PRIVATE stm32cubemx)
