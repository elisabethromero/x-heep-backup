#include "gpio.h"
#include "gpio_interface.h"
#include "task_control.h"
#include "uwb_api.h"
#include "uwb_core.h"
#include "rv_plic.h"
#include "rv_plic_regs.h"
#include "pad_control.h"
#include "fast_intr_ctrl.h"
#include "spi_host.h"
#include "spi_sdk.h"
#include "csr.h"
#include "hart.h"
#include "handler.h"
#include "core_v_mini_mcu.h"
#include "rv_timer.h"
#include "Config.h"

QueueHandle_t gpio_evt_queue = NULL;
QueueHandle_t int_evt_queue = NULL;
QueueHandle_t rdy_evt_queue = NULL;

volatile uint8_t gpio_intr_flag_int = 0;
volatile uint8_t gpio_intr_flag_rdy = 0;

// Función de inicialización de GPIOs y colas de eventos para interrupciones
void gpio_init(){
    printf("\n>> Initializing GPIO interrupt system...\n");
    
    gpio_result_t gpio_res;
    gpio_cfg_t int_conf = {0};
    gpio_cfg_t rdy_conf = {0};
    gpio_cfg_t cs_conf = {0};
    gpio_cfg_t rst_conf = {0};
    gpio_cfg_t prueba_conf = {0};

    // Configurar GPIOs de entrada
    int_conf.pin = GPIO_INT_IO;
    int_conf.mode = GpioModeIn;
    int_conf.en_input_sampling = true;
    int_conf.en_intr = true;
    int_conf.intr_type = GpioIntrEdgeRisingFalling;
    // gpio_config(&io_conf);
    gpio_res = gpio_config(int_conf);
    if (gpio_res != GpioOk) {
        printf("Failed to configure GPIO at pin %d\n", GPIO_INT_IO);
        return -1;
    }

    // Configurar GPIOs de entrada
    rdy_conf.pin = GPIO_RDY_IO;
    rdy_conf.mode = GpioModeIn;
    rdy_conf.en_input_sampling = true;
    rdy_conf.en_intr = true;
    rdy_conf.intr_type = GpioIntrEdgeRisingFalling;
    // gpio_config(&io_conf);
    gpio_res = gpio_config(rdy_conf);
    if (gpio_res != GpioOk) {
        printf("Failed to configure GPIO at pin %d\n", GPIO_RDY_IO);
        return -1;
    }

    // Configurar GPIOs de salida
    cs_conf.pin = GPIO_CS;
    //cs_conf.mode = GpioModeOutPushPull;
    cs_conf.mode = GpioModeoutOpenDrain1;
    // gpio_config(&io_conf);
    gpio_res = gpio_config(cs_conf);
    if (gpio_res != GpioOk) {
        printf("Failed to configure GPIO at pin %d\n", GPIO_CS);
        return -1;
    }
    gpio_write(GPIO_CS, true); // Desactivar CS al inicio    

    // Configurar pin RST como salida y establecerlo en alto por defecto
    rst_conf.pin = GPIO_RST_IO;
    rst_conf.mode = GpioModeOutPushPull;
    gpio_res = gpio_config(rst_conf);
    if (gpio_res != GpioOk) {
        printf("Failed to configure GPIO at pin %d\n", GPIO_RST_IO);
        return -1;
    }

    gpio_write(GPIO_RST_IO, true); // Establecer nivel alto por defecto

    // Configurar pin RST como salida y establecerlo en alto por defecto
    prueba_conf.pin = GPIO_PRUEBA;
    prueba_conf.mode = GpioModeOutPushPull;
    gpio_res = gpio_config(prueba_conf);
    if (gpio_res != GpioOk) {
        printf("Failed to configure GPIO at pin %d\n", GPIO_PRUEBA);
        return -1;
    }

    // Crear cola de eventos
    if(!gpio_evt_queue) {
        gpio_evt_queue = xQueueCreate(25, sizeof(uint32_t));
    }
    if(!int_evt_queue) { 
        int_evt_queue = xQueueCreate(25, sizeof(uint32_t)); 
    } 
    // if(!rdy_evt_queue) { //     rdy_evt_queue = xQueueCreate(10, sizeof(uint32_t)); // }

    //gpio_write(GPIO_CS, false);

    gpio_assign_irq_handler(GPIO_INTR_INT, &gpio_isr_handler_int);
    gpio_assign_irq_handler(GPIO_INTR_RDY, &gpio_isr_handler_rdy);

    plic_assign_external_irq_handler(GPIO_INTR_INT, &gpio_isr_handler_int);
    plic_assign_external_irq_handler(GPIO_INTR_RDY, &gpio_isr_handler_rdy);

    print_gpio_states();

    printf("GPIO interrupt system initialized successfully.\n");

    //vTaskDelay(500 / portTICK_PERIOD_MS);
}

// Manejador de interrupciones
void gpio_isr_handler_int() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t io_num = GPIO_INT_IO;
    gpio_intr_flag_int = 1;
    if(acquisition_active || conf_uwb_active || interrupt_processing_enabled) {
        if (gpio_evt_queue != NULL) {
        xQueueSendFromISR(gpio_evt_queue, &io_num, &xHigherPriorityTaskWoken);
        }
    }
    //printf("Interrupt received\n"); // Totalmente prohibido imprimir en una ISR

    // Para hacer un cambio de contexto inmediato si es necesario
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Manejador de interrupciones
void gpio_isr_handler_rdy() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t io_num = GPIO_RDY_IO;
    gpio_intr_flag_rdy = 1;
    if(acquisition_active || conf_uwb_active || interrupt_processing_enabled) {
        if (gpio_evt_queue != NULL) {
        xQueueSendFromISR(gpio_evt_queue, &io_num, &xHigherPriorityTaskWoken);
        }
    }
    //printf("Interrupt received\n"); // Totalmente prohibido imprimir en una ISR

    // Para hacer un cambio de contexto inmediato si es necesario
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}