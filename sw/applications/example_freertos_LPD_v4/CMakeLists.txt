cmake_minimum_required(VERSION 3.15)

project(example_freertos_LPD_v4 C)

set(SOURCES
        main.c
        algorithm/fft.c
        algorithm/model.c
        algorithm/z_math.c
        buffers/circular_buffer.c
        buffers/linear_buffer.c
        communication/uwb_api.c
        communication/uwb_core.c
        communication/uwb_commands.c
        config/radar_config.c
        drivers/spi_interface.c
        drivers/gpio_interface.c
        sync/sync.c
        tasks/task_control.c
        tasks/task_data_acquisition.c
        tasks/task_algorithm_execution.c
        tasks/handlers_tasks.c
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
        ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${CMAKE_CURRENT_SOURCE_DIR}/testing_data
)