cmake_minimum_required(VERSION 3.16)

project(libpca9685_agnostic VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(pca9685_agnostic STATIC
    src/pca9685.cpp
    platform/linux/linux_delay.cpp
    platform/linux/linux_i2c_bus.cpp
)

target_include_directories(pca9685_agnostic
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Build examples (optional)
option(PCA9685_BUILD_EXAMPLES "Build examples for PCA9685" OFF)
if(PCA9685_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()