/*
 * FileName : Config.h
 * Brief    : place global typedefs and macros here
 */

#ifndef _CONFIG_H
#define _CONFIG_H

#include <stdio.h>
#include <math.h>

/* type redefinition */
/*
typedef enum
{
   FALSE = 0,
   TRUE  = !FALSE
} BOOL;
*/

//Definiciones de SPI
#define IDNEO_SPI         SPI_IDX_HOST //SPI_IDX_HOST_2
#define IDNEO_SPI_SPEED   1000000 //He puesto 100kbits/s (100000) lo mismo que PREMO. Antes habia 1000000, que es lo que pone Miguel Ángel

// Definiciones de pines
#define GPIO_RST_IO 10 
#define GPIO_RDY_IO  8
#define GPIO_INT_IO  9
#define GPIO_CS 6 // Chip select del SPI

#define GPIO_INTR_RDY GPIO_INTR_8 // GPIO para RDY
#define GPIO_INTR_INT GPIO_INTR_9 // GPIO para interrupciones
#define GPIO_INTR_RST GPIO_INTR_10 // GPIO para RST

#define FIC_SPI_HOST_MEIE  20               // SPI Host 1 fast interrupt bit enable
#define CSR_INTR_EN        0x08             // CPU Global interrupt enable
#define FIC_FLASH_MEIE     21               // SPI Flash fast interrupt bit enable


typedef unsigned int                      UINT16;
typedef unsigned long                     UINT32;
typedef unsigned char                     UINT8;
typedef int                               INT16;
typedef long                              INT32;
typedef char                              INT8;

typedef float                             FP32;
typedef double                            FP64;

typedef long long                         int64_t;

typedef struct {
    float real;
	float imag;
} COMPLEX;

/* Get a byte or word in specified address */
#define MEM_B(x)                          ( *( (UINT8 *)(x) ) )
#define MEM_W(x)                          ( *( (UINT32 *)(x) ) )

#define UPCASE( c )                       ( ((c) >= ''a'' && (c) <= ''z'') ?  \
                                          ( (c) - 0x20) : (c) )

/* Check wether a case is a decimal */
#define DECCHK( c )                       ((c) >= ''0'' && (c) <= ''9'')

/* Check wether a case is a hex */
#define HEXCHK( c )                       ( ((c) >= ''0'' && (c) <= ''9'') ||  \
                                          ( (c) >= ''A'' && (c) <= ''F'') ||  \
                                          ( (c) >= ''a'' && (c) <= ''f'') )

/* ABS/MAX/MIN */

#define ABS(x)                            ( ((x) > 0) ? (x) : (-x) ) 

#ifndef MIN
#define MIN(a, b)                         ( ((a) < (b))?(a):(b) )
#endif

#ifndef MAX
#define MAX(a, b)                         ( ((a) > (b))?(a):(b) ) 
#endif

/* Constants */
#define PI                                (3.1416f)
#define PI_DIV2                           (1.5708f)

/* Function Define */
#ifndef ClrBit
#define ClrBit(reg, bit)                  reg &= ~(1 << bit)
#endif

#ifndef SetBit
#define SetBit(reg, bit)                  reg |= (1 << bit)
#endif 

/* Constants */
#define ZX_SUCCESS                        (1)
#define ZX_FAILURE                        (0)

#define LEVEL_HIGH                        (1)
#define LEVEL_LOW                         (0)

/* ASSERT */
// uncomment line below to 
#define USE_FULL_ASSERT
#ifdef USE_FULL_ASSERT
/**
 * @brief  The assert_param macro is used for function's parameters check.
 * @param  expr: If expr is false, it calls assert_failed function
 *   which reports the name of the source file and the source
 *   line number of the call that failed. 
 *   If expr is true, it returns no value.
 * @retval None
 */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
  extern void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif

#endif
