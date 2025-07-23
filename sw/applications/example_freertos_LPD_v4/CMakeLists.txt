cmake_minimum_required(VERSION 3.15)

project(example_freertos_LPD_v4 C)

set(SOURCES
        main.c
        buffers/circular_buffer.c 
        buffers/linear_buffer.c 
        algorithm/fft.c
        communication/uwb_api.c
        communication/uwb_core.c
        communication/uwb_commands.c
        drivers/spi_interface.c
        drivers/gpio.c
        algorithm/model.c
        algorithm/z_math.c
        tasks/task_control.c
        sync/sync.c
        tasks/task_data_acquisition.c
        tasks/task_algorithm_execution.c
        tasks/handlers_tasks.c
        lib/hal/spi_host.c
        config/radar_config.c
        testing_data/input_data_generated.c
        testing_data/cir_input_sequence_EXE_EEE_3.c
)

INCLUDE_DIRECTORIES(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/buffers
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers
        ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
        ${CMAKE_CURRENT_SOURCE_DIR}/communication
        ${CMAKE_CURRENT_SOURCE_DIR}/tasks
        ${CMAKE_CURRENT_SOURCE_DIR}/sync
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/hal
        ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${CMAKE_CURRENT_SOURCE_DIR}/testing_data
)