# Standardowa biblioteka statyczna
add_library(stm32_can_lib STATIC)

target_sources(stm32_can_lib PRIVATE
    can_profile/can_interface/can_interface.c
    can_profile/can_interface/can_slave.c
    can_profile/can_interface/can_master.c
    can_profile/can_wrapper/can_wrapper.c
    can_profile/commander/commander.c
    can_profile/slave_driver/slave_driver.c
)

target_include_directories(stm32_can_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/can_profile/can_interface
    ${CMAKE_CURRENT_SOURCE_DIR}/can_profile/can_wrapper
    ${CMAKE_CURRENT_SOURCE_DIR}/can_profile/commander
    ${CMAKE_CURRENT_SOURCE_DIR}/can_profile/slave_driver
)

target_link_libraries(stm32_can_lib PUBLIC
    stm32cubemx
)
