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
#include "spi_host.h"
#include "spi_sdk.h"
#include "rv_plic.h"
#include "rv_plic_regs.h"
#include "pad_control.h"

#include "radar_config.h"
#include "linear_buffer.h"
#include "circular_buffer.h"
#include "task_control.h"
#include "task_data_acquisition.h"
#include "task_algorithm_execution.h"
#include "uwb_api.h"
#include "uwb_core.h"
#include "spi_interface.h"
#include "gpio.h"
#include "sync.h"
#include "handlers_tasks.h"
#include "uwb_commands.h"
#include "config.h"
#include "Config.h"

//Definición de Buffers
CircularBuffer circ_buffer;
LinearBuffer lin_buffer;

TickType_t start_time;

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
    uint32_t mtvec;
    __asm volatile ("csrr %0, mtvec" : "=r"(mtvec));
    printf("mtvec = 0x%08lx\r\n", (unsigned long)mtvec);

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
    //printf("INTERRUPTION GLOBAL HANDLER TRIGGERED!\n");
    
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
    printf("Stack remaining: %d\n", uxTaskGetStackHighWaterMark(NULL));
    printf("¡Stack overflow en tarea: %s!\n", pcTaskName);
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
    CSR_SET_BITS(CSR_REG_MSTATUS, 0x8); //CSR_SET_BITS(CSR_REG_MSTATUS, 0x8);

    // Set mie.MEIE bit to one to enable machine-level external interrupts
    uint32_t mask_timer = 1 << 7; // Enable timer interrupt
    uint32_t mask_machine = 1 << 11; // Enable machine external interrupts
    //uint32_t mask_ext_int = 1 << 31; // Enable fast interrupt - spi
    uint32_t mask_spi_host = 1 << 20; // fast interrupt - spi
    uint32_t mask_spi_flash = 1 << 21; // fast interrupt - spi flash
    CSR_SET_BITS(CSR_REG_MIE, mask_timer);
    CSR_SET_BITS(CSR_REG_MIE, mask_machine);
    CSR_SET_BITS(CSR_REG_MIE, mask_spi_host);

    // Enable the external interrupt at fast interrupt controller level
    enable_fast_interrupt(4, true);

    configASSERT(rv_timer_irq_enable(&timer_0_1, 0, 0, kRvTimerEnabled) == kRvTimerOk);
	configASSERT(rv_timer_counter_set_enabled(&timer_0_1, 0, kRvTimerEnabled) == kRvTimerOk);
}

/*****************************************************************************
*****************************************************************************/

void printSplashScreen(void) {
    printf("\n");
    printf("=============================================================\n");
    printf("                     UWB MONITORING SYSTEM                   \n");
    printf("                     RISCCOM Project - LPD                   \n");
    printf("=============================================================\n\n");
    printf(">> Starting system initialization...\n\n");
}


// En esta función principal se inicializan los semáforos, las colas y los buffers y se crean las tareas
void main()
{   
    start_time = xTaskGetTickCount();  // Tiempo de arranque en ticks

    printSplashScreen();

    SetupHardware(); // Configuración de hardware
    
    printf("FreeRTOS version: %s\n", tskKERNEL_VERSION_NUMBER);

    // Inicialización de periféricos y drivers
    sync_init();        // Semáforos, colas...
    gpio_init();        // Pines e interrupciones
    spi_initialization();         // SPI
    
    // Inicializar buffers
    initializeCircularBuffer(&circ_buffer);
    initializeLinearBuffer(&lin_buffer);
    
    // // Creación tareas
    TaskHandle_t control_task_handle = NULL;
    TaskHandle_t acquisition_task_handle = NULL;
    TaskHandle_t algorithm_task_handle = NULL;
    TaskHandle_t uwb_task_handle = NULL;
     
    if (SOURCE_DATA != 3) {
        xTaskCreate(task_control, "Control Task", 4096, NULL, 3, &control_task_handle);
        if (SOURCE_DATA == 2) {
            printf("\n[INFO] Training mode active. Starting tasks...\n\n");
        } else if (SOURCE_DATA == 1) {
            printf("\n[INFO] Simulation mode active. Starting tasks...\n\n");
        }
    }

    // Si se ha seleccionado el módulo UWB, crear la tarea para recibir comandos y gestionar el UWB
    if(SOURCE_DATA == 3){
        printf("\n[INFO] UWB module active. Starting tasks...\n\n");
        //xTaskCreate(task_uwb_menu, "UWB Task", 8192, NULL, 2, &uwb_task_handle);
        xTaskCreate(gpio_monitor_task, "gpio_monitor_task", 4096, NULL, 3, NULL);
        xTaskCreate(task_uwb, "Communication UWB", 4096, NULL, 3, NULL);
    }

    xTaskCreate(task_data_acquisition, "Data Acquisition", 4096, NULL, 2, &acquisition_task_handle);
    xTaskCreate(task_algorithm_execution, "Algorithm Execution", 8192, NULL, 2, &algorithm_task_handle);

    printf("Free heap after task creation: %d bytes\n", xPortGetFreeHeapSize());

    //En FreeRTOS para PYNQ-Z2 se deberá iniciar manualmente el scheduler
    vTaskStartScheduler();


    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    return; // Nunca se debe llegar aquí
}