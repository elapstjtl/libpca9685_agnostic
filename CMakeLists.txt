cmake_minimum_required(VERSION 3.16)

project(libpca9685_agnostic VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 创建静态库
add_library(pca9685_agnostic STATIC
    src/pca9685.cpp
    platform/linux/linux_delay.cpp
    platform/linux/linux_i2c_bus.cpp
)

# 设置头文件包含路径（PUBLIC 确保使用此库的项目也能访问头文件）
target_include_directories(pca9685_agnostic
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)

# 设置库的别名，方便其他项目使用
add_library(pca9685_agnostic::pca9685_agnostic ALIAS pca9685_agnostic)

# 链接系统库（Linux I2C库）
if(UNIX AND NOT APPLE)
    target_link_libraries(pca9685_agnostic PUBLIC i2c)
endif()

# Build examples (optional)
option(PCA9685_BUILD_EXAMPLES "Build examples for PCA9685" OFF)
if(PCA9685_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()