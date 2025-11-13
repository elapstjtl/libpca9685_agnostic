
# 设置树莓派交叉编译的sysroot路径
set(RPI_SYSROOT "/home/supernova/jtl/rpi/sysroot")

# 设置交叉编译的库搜索路径
set(CMAKE_FIND_ROOT_PATH ${RPI_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

add_executable(SCD4XPlus_example_raspberry_pi
    main.cpp
    ${PROJECT_SOURCE_DIR}/platform/linux/linux_i2c_bus.cpp
    ${PROJECT_SOURCE_DIR}/platform/linux/linux_delay.cpp
)

target_include_directories(SCD4XPlus_example_raspberry_pi PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${RPI_SYSROOT}/usr/usr/include
)

# 设置库搜索路径 - 修正为正确的路径
target_link_directories(SCD4XPlus_example_raspberry_pi PRIVATE
    ${RPI_SYSROOT}/lib/aarch64-linux-gnu
    ${RPI_SYSROOT}/usr/usr/lib/aarch64-linux-gnu
)

target_link_libraries(SCD4XPlus_example_raspberry_pi PRIVATE
    SCD4XPlus
    i2c
)