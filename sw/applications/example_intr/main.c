/* c stdlib */
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>

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


#define GPIO_CS 10 // Chip select del SPI
#define GPIO_RDY_IO  8
#define GPIO_INT_IO  9

#define GPIO_INTR_INT GPIO_INTR_9 // GPIO para interrupciones
#define GPIO_INTR_RDY GPIO_INTR_8 // GPIO para RDY

#define FIC_SPI_HOST_MEIE  20               // SPI Host 1 fast interrupt bit enable
#define CSR_INTR_EN        0x08             // CPU Global interrupt enable
#define FIC_FLASH_MEIE     21               // SPI Flash fast interrupt bit enable

volatile uint8_t gpio_intr_flag_int = 0;
volatile uint8_t gpio_intr_flag_rdy = 0;


/* Prepare hardware to run the demo. */

static void SetupHardware( void )
{
	/* Init board hardware. */
	system_init();
}

/**
 * Board init code. Always call this before anything else.
 */
void system_init(void)
{
// Get current Frequency
    soc_ctrl_t soc_ctrl;
    soc_ctrl.base_addr = mmio_region_from_addr((uintptr_t)SOC_CTRL_START_ADDRESS);
    uint32_t freq_hz = soc_ctrl_get_frequency(&soc_ctrl);

    plic_result_t plic_res;
    plic_res = plic_Init();
    if (plic_res != kPlicOk) {
        printf("Init PLIC failed\n\r;");
        return -1;
    }

    
    plic_res = plic_irq_set_priority(GPIO_INTR_INT, 1);
    if (plic_res != kPlicOk) {
        printf("Failed\n\r;");
        return -1;
    }

    plic_res = plic_irq_set_priority(GPIO_INTR_RDY, 1);
    if (plic_res != kPlicOk) {
        printf("Failed\n\r;");
        return -1;
    }
    

    plic_res = plic_irq_set_enabled(GPIO_INTR_INT, kPlicToggleEnabled);
    if (plic_res != kPlicOk) {
        printf("Failed\n\r;");
        return -1;
    }

    plic_res = plic_irq_set_enabled(GPIO_INTR_RDY, kPlicToggleEnabled);
    if (plic_res != kPlicOk) {
        printf("Failed\n\r;");
        return -1;
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
}

// Manejador de interrupciones
void gpio_isr_handler_int() { 
    uint32_t gpio_num = GPIO_INT_IO;
    printf("\n>>[ISR] Handler INT\n");

    gpio_intr_flag_int = 1;
}

// Manejador de interrupciones
void gpio_isr_handler_rdy() { 
    uint32_t gpio_num = GPIO_RDY_IO;
    printf("\n>>[ISR] Handler RDY\n");

    gpio_intr_flag_rdy = 1;
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

    gpio_assign_irq_handler(GPIO_INTR_INT, &gpio_isr_handler_int);
    gpio_assign_irq_handler(GPIO_INTR_RDY, &gpio_isr_handler_rdy);

    printf("GPIO interrupt system initialized successfully.\n");
}

void main()
{
    SetupHardware(); // Configuración de hardware

    gpio_init();     // Pines e interrupciones

    while(1){
        if (gpio_intr_flag_int) {
            gpio_intr_flag_int = 0;
            printf("Interrupción detectada en INT (pin %d)\n", GPIO_INT_IO);
        }
        else if (gpio_intr_flag_rdy) {
            gpio_intr_flag_rdy = 0;
            printf("Interrupción detectada en RDY (pin %d)\n", GPIO_RDY_IO);
        }
        else{
            // No hay interrupciones pendientes, puedes realizar otras tareas
            printf("No hay interrupciones pendientes.\n");
        }
    }

    return 0;

}