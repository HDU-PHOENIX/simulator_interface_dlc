cmake_minimum_required(VERSION 3.16)
project(talos_ipc_gimbal_cmd_sender LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(talos_ipc_gimbal_cmd_sender main.cpp)

target_include_directories(talos_ipc_gimbal_cmd_sender PRIVATE /usr/include)
