#include "gpio.h"
#include "gpio_interface.h"
#include "task_control.h"
#include "uwb_api.h"
#include "uwb_core.h"

QueueHandle_t gpio_evt_queue = NULL;
QueueHandle_t int_evt_queue = NULL;
QueueHandle_t rdy_evt_queue = NULL;


// Función de inicialización de GPIOs y colas de eventos para interrupciones
void gpio_init(){
    printf("\n>> Initializing GPIO interrupt system...\n");
    
    // INT y RDY: flanco de bajada //ambos flancos
    gpio_config_t input_conf = {
        .intr_type = GPIO_INTR_ANYEDGE, // Interrupción por ambos flancos
        //.intr_type = GPIO_INTR_NEGEDGE, // Interrupción por flanco de bajada
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en = 1 // en alto, por defecto
    };
    gpio_config(&input_conf);

    // Configurar pin CS como salida
    gpio_config_t output_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_CS_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en = 1 // en alto, por defecto
    };
    gpio_config(&output_conf);
    // gpio_set_level(GPIO_CS, 0);    

    // Configurar pin RST como salida y establecerlo en bajo por defecto
    gpio_config_t rst_conf = {
        .intr_type = GPIO_INTR_ANYEDGE, // Interrupción por ambos flancos
        // .intr_type = GPIO_INTR_NEGEDGE, // Interrupción por flanco de bajada
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_RST_PIN_SEL,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE // en bajo, por defecto
    };
    gpio_config(&rst_conf);
    gpio_set_level(GPIO_RST_IO, 1); // Establecer nivel alto por defecto

    // Crear cola de eventos
    if(!gpio_evt_queue) {
        gpio_evt_queue = xQueueCreate(25, sizeof(uint32_t));
    }
    if(!int_evt_queue) { int_evt_queue = xQueueCreate(25, sizeof(uint32_t)); } 
    // if(!rdy_evt_queue) { //     rdy_evt_queue = xQueueCreate(10, sizeof(uint32_t)); // }

    gpio_set_level(GPIO_CS, 0);
    // Instalar servicio de interrupciones (ISR) y registrar los dos pines
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    gpio_isr_handler_add(GPIO_RDY_IO, gpio_isr_handler, (void*) GPIO_RDY_IO);
    gpio_isr_handler_add(GPIO_INT_IO, gpio_isr_handler, (void*) GPIO_INT_IO);
    gpio_isr_handler_add(GPIO_CS, gpio_isr_handler, (void*) GPIO_CS);
    gpio_isr_handler_add(GPIO_RST_IO, gpio_isr_handler, (void*) GPIO_RST_IO);

    printf("GPIO interrupt system initialized successfully.\n");

    vTaskDelay(500 / portTICK_PERIOD_MS);
}

// Manejador de interrupciones
void IRAM_ATTR gpio_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t gpio_num = (uint32_t) arg;
    if(acquisition_active || conf_uwb_active || interrupt_processing_enabled) {
    
        if (gpio_num == GPIO_CS || gpio_num == GPIO_RDY_IO || gpio_num == GPIO_INT_IO || gpio_num == GPIO_RST_IO) {
            // Enviar a la cola general de eventos (RDY, CS, etc.)
            xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);
        }
    }
    //printf("Interrupt received\n"); // Totalmente prohibido imprimir en una ISR

    // Para hacer un cambio de contexto inmediato si es necesario
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}