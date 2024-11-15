cmake_minimum_required(VERSION 3.22)

project(CppGBurIRIS)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


find_package(drake CONFIG REQUIRED PATHS /opt/drake)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy REQUIRED)

include_directories(include)

file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")

add_executable(CppGBurIRIS ${SOURCES})

target_link_libraries(CppGBurIRIS drake::drake Python3::Python Python3::Module Python3::NumPy)
