cmake_minimum_required(VERSION 3.5)
project(mirte_master_usb_switch)

add_compile_options(-lgpiodcxx)

add_executable(${PROJECT_NAME} usb_switch.cpp)
target_link_libraries(${PROJECT_NAME} gpiodcxx)
