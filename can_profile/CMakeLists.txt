add_library(foc_lib OBJECT)

target_sources(foc_lib PRIVATE

    # Can odrive
    can_odrive/can_odrive.c

    # Can wrapper
    can_wrapper/can_wrapper.c

    # Commander
    commander/commander.c

    # Slave driver
    slave_driver/slave_driver.c
)

target_include_directories(foc_lib PUBLIC
    can_odrive
    can_wrapper
    commander
    slave_driver
)

target_link_libraries(foc_lib PUBLIC
    stm32cubemx
)