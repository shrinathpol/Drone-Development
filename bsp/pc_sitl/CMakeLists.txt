cmake_minimum_required(VERSION 3.16)
project(pan_brain_sitl C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")

add_executable(pan_brain_sitl
    src/sitl_main.c
    src/bsp_pc_sitl.c
    src/sitl_physics.c
    ${ROOT_DIR}/app/src/main_brain.c
    ${ROOT_DIR}/core/src/telemetry_proto.c
    ${ROOT_DIR}/core/src/crypto_auth.c
    ${ROOT_DIR}/core/src/offline_queue.c
    ${ROOT_DIR}/core/src/dsp_filters.c
    ${ROOT_DIR}/core/src/ahrs_fusion.c
    ${ROOT_DIR}/core/src/ekf_nav.c
    ${ROOT_DIR}/core/src/flight_pid.c
    ${ROOT_DIR}/core/src/mission_engine.c
    ${ROOT_DIR}/core/src/failsafe_engine.c
    ${ROOT_DIR}/core/src/link_router.c
    ${ROOT_DIR}/core/src/ota_manager.c
    ${ROOT_DIR}/core/src/config_manager.c
    ${ROOT_DIR}/drivers/src/modem_sim7000g.c
    ${ROOT_DIR}/drivers/src/imu_universal.c
    ${ROOT_DIR}/drivers/src/env_sensors.c
    ${ROOT_DIR}/drivers/src/gps_ublox.c
    ${ROOT_DIR}/drivers/src/optflow_pmw3901.c
    ${ROOT_DIR}/drivers/src/motor_esc.c
    ${ROOT_DIR}/drivers/src/rc_radio.c
    ${ROOT_DIR}/drivers/src/camera_ctrl.c
    ${ROOT_DIR}/drivers/src/led_status.c
)

target_include_directories(pan_brain_sitl PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ROOT_DIR}/bsp/include
    ${ROOT_DIR}/drivers/include
    ${ROOT_DIR}/core/include
    ${ROOT_DIR}/app/include
)

if(WIN32)
    target_link_libraries(pan_brain_sitl PRIVATE ws2_32)
else()
    target_link_libraries(pan_brain_sitl PRIVATE m)
endif()

