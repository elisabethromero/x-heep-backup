#include <string.h>
#include <stdio.h>

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
#include "rv_plic.h"
#include "rv_plic_regs.h"
#include "pad_control.h"

#include "spi_risccom.h"

#define RX_LEN 16

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
    //CSR_SET_BITS(CSR_REG_MIE, mask_spi_host);

    // Enable the external interrupt at fast interrupt controller level
    //enable_fast_interrupt(4, true);

    configASSERT(rv_timer_irq_enable(&timer_0_1, 0, 0, kRvTimerEnabled) == kRvTimerOk);
	configASSERT(rv_timer_counter_set_enabled(&timer_0_1, 0, kRvTimerEnabled) == kRvTimerOk);
}

/*****************************************************************************
*****************************************************************************/



/****************************************************************************//*
 * Public code
 ******************************************************************************/

void vSpiTask(void *pvParameter)
{
        uint8_t tx_buf[16] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00 };
        uint8_t rx_buf[RX_LEN] = {0};
        bool ret;
        
        //Init board
        printf("RISCOM Program started.\r\n");
        ret = riscom_platform_init();
        if (ret)
        {
            printf("Platform initialization correct.\r\n");
        }
        else
        {
            printf("Platform initialization error.\r\n");
            vTaskDelete(NULL);
        }

        /****************************************************************************//*
        * TRANSMIT SPI DATA
        ******************************************************************************/
        /*if (riscom_platform_spi_transmit(tx_buf, sizeof(tx_buf))) {
            printf("Código enviado por maestro SPI: ");
            for (int i = 0; i < sizeof(tx_buf); i++) {
                printf("%02X ", tx_buf[i]);
            }
            printf("\n");
        } else {
            printf("Error de transmisión.\n");
        }*/

        /****************************************************************************//*
        * RECEIVE SPI DATA
        ******************************************************************************/
    /*
        printf("Esperando datos desde el esclavo SPI...\n");

        if (riscom_platform_spi_receive(rx_buf, RX_LEN)) {
            printf("Datos recibidos del esclavo SPI: ");
            for (int i = 0; i < RX_LEN; i++) {
                printf("%02X ", rx_buf[i]);
            }
            printf("\n");
        } else {
            printf("Error de recepción.\n");
        }
    */
        /****************************************************************************//*
        * TRANSCEIVE SPI DATA
        ******************************************************************************/
    for (;;){
        printf("Inside the task\r\n");
        if (riscom_platform_spi_transceive(tx_buf, rx_buf, 16)) {
            printf("Datos recibidos del esclavo SPI: ");
            for (int i = 0; i < RX_LEN; ++i) {
                printf("%02X ", rx_buf[i]);
            }
            printf("\n");
            printf("Código enviado por maestro SPI: ");
            for (int i = 0; i < sizeof(tx_buf); i++) {
                printf("%02X ", tx_buf[i]);
            }
            printf("\n");
        } else {
            printf("Fallo en la transacción.\n");
        }
       vTaskDelay(pdMS_TO_TICKS(1000)); // Termina la tarea
    }
}

void main()
{

    SetupHardware(); // Configuración de hardware
    
    printf("FreeRTOS version: %s\n", tskKERNEL_VERSION_NUMBER);

    xTaskCreate(vSpiTask, "SPI Transfer Task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    return; // Nunca se debe llegar aquí
}
/******************************** End of file *********************************/