add_executable(pca9685_example_raspberry_pi
    main.cpp
)

target_include_directories(pca9685_example_raspberry_pi PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}
)

target_link_directories(pca9685_example_raspberry_pi PRIVATE
    ${CMAKE_SYSROOT}/lib/aarch64-linux-gnu
    ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu
)

target_link_libraries(pca9685_example_raspberry_pi PRIVATE
    pca9685_agnostic
    i2c
)