/* c stdlib */
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"

/* Librerías de X-HEEP */
#include "csr.h"
#include "hart.h"
#include "handler.h"
#include "core_v_mini_mcu.h"
#include "rv_timer.h"
#include "soc_ctrl.h"
#include "gpio.h"
#include "x-heep.h"
#include "fast_intr_ctrl.h"
#include "rv_plic.h"
#include "rv_plic_regs.h"
#include "pad_control.h"


#define GPIO_CS 10 //6 // Chip select del SPI
#define GPIO_RDY_IO  8
#define GPIO_INT_IO  9

#define GPIO_INTR_INT GPIO_INTR_9 // GPIO para interrupciones
#define GPIO_INTR_RDY GPIO_INTR_8 // GPIO para RDY

#define RDY_TIMEOUT_MS 1000 // 1 segundo máximo
#define INT_TIMEOUT_MS 1000 // 1 segundo máximo

#define FIC_SPI_HOST_MEIE  20               // SPI Host 1 fast interrupt bit enable
#define CSR_INTR_EN        0x08             // CPU Global interrupt enable
#define FIC_FLASH_MEIE     21               // SPI Flash fast interrupt bit enable

volatile uint8_t gpio_intr_flag_int = 0;
volatile uint8_t gpio_intr_flag_rdy = 0;

QueueHandle_t gpio_evt_queue = NULL;
QueueHandle_t int_evt_queue = NULL;

// Definición de semáforos y cola
SemaphoreHandle_t data_sampling_ready_sem = NULL;
SemaphoreHandle_t data_fft_ready_sem = NULL;
QueueHandle_t commandQueue = NULL;
QueueHandle_t commandQueue_UWB = NULL;
QueueHandle_t cir_queue = NULL; // Cola para almacenar CIRs

TickType_t start_time;
TickType_t start_acquisition_time;

TaskHandle_t uwb_task_handle = NULL;

/****************************************************************************/
/**                                                                        **/
/*                          VARIABLES GLOBALES                              */
/**                                                                        **/
/****************************************************************************/

#define _MAIN_C_SRC

/* Allocate heap to special section. Note that we have no references in the
 * whole program to this variable (since its just here to allocate space in the
 * section for our heap), so when using LTO it will be removed. We force it to
 * stay with the "used" attribute
 */
__attribute__((section(".heap"), used)) uint8_t ucHeap[configTOTAL_HEAP_SIZE];

/* Timer 0 AO Domain as Tick Counter */
static rv_timer_t timer_0_1;

//[EMRL]
/****************************************************************************/
/**                                                                        **/
/*          FUNCIONES PROPIAS DEL FREERTOS PARA RISC-V                      */
/**                                                                        **/
/****************************************************************************/
/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file.  See https://www.freertos.org/a00016.html */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

/*-----------------------------------------------------------*/
void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	printf( "error: application malloc failed\n\r" );
	__asm volatile( "ebreak" );
	for( ;; );
}
/*-----------------------------------------------------------*/

void freertos_risc_v_application_exception_handler(uint32_t mcause)
{
	//printf("App mcause:%d\r\n", mcause);
    uint32_t mepc, mtval;
    // Leer el program counter donde ocurrió la excepción
    __asm volatile ("csrr %0, mepc" : "=r"(mepc));
    // Leer el valor relacionado con la excepción (dirección de acceso, etc.)
    __asm volatile ("csrr %0, mtval" : "=r"(mtval));

    printf("App mcause: %d\r\n", mcause);
    printf("Exception at PC: 0x%08lx\r\n", (unsigned long)mepc);
    printf("MTVAL (address/value): 0x%08lx\r\n", (unsigned long)mtval);

    // Opcional: puedes agregar una descripción del tipo de excepción
    switch (mcause) {
        case 2: printf("Illegal instruction\n"); break;
        case 3: printf("Store/AMO access fault\n"); break;
        case 5: printf("Load access fault\n"); break;
        case 7: printf("Store/AMO page fault\n"); break;
        default: printf("Other exception\n"); break;
    }
}
/*-----------------------------------------------------------*/

void freertos_risc_v_application_interrupt_handler(void)
{
    uint32_t irq_id;
    printf("INTERRUPTION GLOBAL HANDLER TRIGGERED!\n");
    
    if (plic_irq_claim(&irq_id) != kPlicOk) {
        printf("Error reclamando interrupción: irq_id = %lu\n", irq_id);
        return; // Fallo al reclamar la interrupción
    }

    switch (irq_id) {
        case GPIO_INTR_9:
            gpio_isr_handler_int();
            break;
        case GPIO_INTR_8:
            gpio_isr_handler_rdy();
            break;
        default:
            printf("Unhandled IRQ ID: %d\n", irq_id);
            break;
    }

    // Marca la interrupción como completada
    plic_irq_complete(&irq_id);
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
	taskENTER_CRITICAL();
	//printf("I\r\n");
	taskEXIT_CRITICAL();
	
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	__asm volatile( "ebreak" );
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
    //vacío
}
/*-----------------------------------------------------------*/

/* Prepare hardware to run the demo. */

static void SetupHardware( void )
{
	/* Init board hardware. */
	system_init();
}

/*-----------------------------------------------------------*/
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, 
    StackType_t **ppxIdleTaskStackBuffer, 
    uint32_t *pulIdleTaskStackSize) {
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
*ppxIdleTaskStackBuffer = uxIdleTaskStack;
*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

#if configUSE_TIMERS == 1
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, 
                                   StackType_t **ppxTimerTaskStackBuffer, 
                                   uint32_t *pulTimerTaskStackSize) {
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
    
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif
/*-----------------------------------------------------------*/

/**
 * Board init code. Always call this before anything else.
 */
void system_init(void)
{
// Get current Frequency
    soc_ctrl_t soc_ctrl;
    soc_ctrl.base_addr = mmio_region_from_addr((uintptr_t)SOC_CTRL_START_ADDRESS);
    uint32_t freq_hz = soc_ctrl_get_frequency(&soc_ctrl);

    // Setup rv_timer_0_1
    mmio_region_t timer_0_1_reg = mmio_region_from_addr(RV_TIMER_AO_START_ADDRESS);
    rv_timer_init(timer_0_1_reg, (rv_timer_config_t){.hart_count = 2, .comparator_count = 1}, &timer_0_1);
	
    plic_result_t plic_res;
    plic_res = plic_Init();
    if (plic_res != kPlicOk) {
        printf("Init PLIC failed\n\r;");
    }

    plic_res = plic_irq_set_trigger(GPIO_INTR_INT, kPlicIrqTriggerEdge);
    if (plic_res != kPlicOk) {
        printf("Failed\n\r;");
    }

    plic_res = plic_irq_set_trigger(GPIO_INTR_RDY, kPlicIrqTriggerEdge);
    if (plic_res != kPlicOk) {
        printf("Failed\n\r;");
    }

    plic_res = plic_irq_set_priority(GPIO_INTR_INT, 1);
    if (plic_res != kPlicOk) {
        printf("Failed\n\r;");
    }

    plic_res = plic_irq_set_priority(GPIO_INTR_RDY, 1);
    if (plic_res != kPlicOk) {
        printf("Failed\n\r;");
    }
    

    plic_res = plic_irq_set_enabled(GPIO_INTR_INT, kPlicToggleEnabled);
    if (plic_res != kPlicOk) {
        printf("Failed\n\r;");
    }

    plic_res = plic_irq_set_enabled(GPIO_INTR_RDY, kPlicToggleEnabled);
    if (plic_res != kPlicOk) {
        printf("Failed\n\r;");
    }

	// Just in case you are playing with Tick freq.
    //rv_timer_approximate_tick_params(freq_hz, kTickFreqHz, &tick_params);

    // Enable interrupt on processor side
    // Enable global interrupt for machine-level interrupts
    CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);

    // Set mie.MEIE bit to one to enable machine-level external interrupts
    //uint32_t mask = 1 << 7;
    uint32_t mask = 1 << 11;
    CSR_SET_BITS(CSR_REG_MIE, mask);

    configASSERT(rv_timer_irq_enable(&timer_0_1, 0, 0, kRvTimerEnabled) == kRvTimerOk);
	configASSERT(rv_timer_counter_set_enabled(&timer_0_1, 0, kRvTimerEnabled) == kRvTimerOk);
}

/*****************************************************************************
*****************************************************************************/

// Manejador de interrupciones
void gpio_isr_handler_int() {
    printf("GPIO interrupt handler INT called.\n");
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t io_num = GPIO_INT_IO;
    gpio_intr_flag_int = 1;
    //xQueueSendFromISR(gpio_evt_queue, &io_num, &xHigherPriorityTaskWoken);
    if (xQueueSendFromISR(gpio_evt_queue, &io_num, &xHigherPriorityTaskWoken) != pdPASS) {
        printf("ERROR: gpio_evt_queue FULL in INT handler!\n");
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Manejador de interrupciones
void gpio_isr_handler_rdy() {
    printf("GPIO interrupt handler RDY called.\n");
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t io_num = GPIO_RDY_IO;
    gpio_intr_flag_rdy = 1;
    //xQueueSendFromISR(gpio_evt_queue, &io_num, &xHigherPriorityTaskWoken);
    if (xQueueSendFromISR(gpio_evt_queue, &io_num, &xHigherPriorityTaskWoken) != pdPASS) {
        printf("ERROR: gpio_evt_queue FULL in RDY handler!\n");
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Función de inicialización de GPIOs y colas de eventos para interrupciones
void gpio_init(){
    printf("\n>> Initializing GPIO interrupt system...\n");
    
    gpio_result_t gpio_res;
    gpio_cfg_t int_conf = {0};
    gpio_cfg_t rdy_conf = {0};
    gpio_cfg_t cs_conf = {0};

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
        return EXIT_FAILURE;
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
        return EXIT_FAILURE;
    }

    // Configurar GPIOs de salida
    cs_conf.pin = GPIO_CS;
    cs_conf.mode = GpioModeOutPushPull;
    // gpio_config(&io_conf);
    gpio_res = gpio_config(cs_conf);
    if (gpio_res != GpioOk) {
        printf("Failed to configure GPIO at pin %d\n", GPIO_CS);
        return EXIT_FAILURE;
    }
    gpio_write(GPIO_CS, true); // Desactivar CS al inicio

    // Crear cola de eventos
    if(!gpio_evt_queue) {
        gpio_evt_queue = xQueueCreate(25, sizeof(uint32_t));
    }
    if(!int_evt_queue) {
        int_evt_queue = xQueueCreate(25, sizeof(uint32_t));
    }

    gpio_assign_irq_handler(GPIO_INTR_INT, &gpio_isr_handler_int);
    gpio_assign_irq_handler(GPIO_INTR_RDY, &gpio_isr_handler_rdy);

    plic_assign_external_irq_handler(GPIO_INTR_INT, &gpio_isr_handler_int);
    plic_assign_external_irq_handler(GPIO_INTR_RDY, &gpio_isr_handler_rdy);

    printf("GPIO interrupt system initialized successfully.\n");
}

/*
void print_task(void* arg) {
    for (;;) {
        printf("[PRINT_TASK] Hola, tarea activa!\n");
        bool level1, level2, level3;
        printf("\033[0;32m\t\tChecando estado de los pines GPIO...\033[0m\n");
        gpio_read(GPIO_INT_IO, &level1);
        printf("\033[0;32m\t\tNivel del gpio INT: %d\033[0m\n", level1);
        gpio_read(GPIO_RDY_IO, &level2);
        printf("\033[0;32m\t\tNivel del gpio RDY: %d\033[0m\n", level2);
        gpio_read(GPIO_CS, &level3);
        printf("\033[0;32m\t\tNivel del gpio CS: %d\033[0m\n", level3);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
*/

void gpio_monitor_task(void* arg) {
    uint32_t io_num;

    for (;;) {
        printf("\n[GPIO MONITOR] Estamos dentro de la tarea\n");
        if (xQueueReceive(gpio_evt_queue, &io_num, 0)==pdPASS) {
            printf("[GPIO MONITOR] Interrupción en GPIO_%ld\n", io_num);
            if (io_num == GPIO_INT_IO){
                bool level;
                gpio_read(io_num, &level);
                printf("[GPIO_TASK] INT interrupt! Pin %d is %s\n", io_num, level ? "HIGH" : "LOW");
                gpio_intr_flag_int = 0;
            }
            if (io_num == GPIO_RDY_IO){
                bool level;
                gpio_read(io_num, &level);
                printf("[GPIO_TASK] RDY interrupt! Pin %d is %s\n", io_num, level ? "HIGH" : "LOW");
                gpio_intr_flag_rdy = 0;
            }
        } else {
            printf("[GPIO_MONITOR] No hay eventos en la cola\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
}

void main()
{
    SetupHardware(); // Configuración de hardware

    gpio_init();     // Pines e interrupciones

    //xTaskCreate(print_task, "print_task", 1024, NULL, 1, NULL);
    xTaskCreate(gpio_monitor_task, "gpio_monitor_task", 4096, NULL, 3, NULL);

    printf("Free heap after task creation: %d bytes\n", xPortGetFreeHeapSize());

    vTaskStartScheduler();

    // Mantener app_main() en loop infinito para evitar que termine
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    return; // Nunca se debe llegar aquí

}