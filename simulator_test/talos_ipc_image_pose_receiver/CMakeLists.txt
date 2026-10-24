cmake_minimum_required(VERSION 3.16)
project(talos_ipc_image_pose_receiver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(talos_ipc_image_pose_receiver main.cpp)

target_link_libraries(talos_ipc_image_pose_receiver PRIVATE ${OpenCV_LIBS})
