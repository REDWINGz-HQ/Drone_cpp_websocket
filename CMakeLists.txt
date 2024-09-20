cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(drone_client_action)

add_executable(drone_client_action
    drone_client_action.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(drone_client_action
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(drone_client PRIVATE -Wall -Wextra)
else()
    add_compile_options(drone_client PRIVATE -WX -W2)
endif()
