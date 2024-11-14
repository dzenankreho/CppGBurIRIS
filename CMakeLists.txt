cmake_minimum_required(VERSION 3.22)

project(gburvcc)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


find_package(drake CONFIG REQUIRED PATHS /opt/drake)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy REQUIRED)

include_directories(include)

add_executable(gburvcc src/visualization.cpp src/robot.cpp src/planar_arm.cpp src/main.cpp)

target_link_libraries(gburvcc drake::drake Python3::Python Python3::Module Python3::NumPy)
