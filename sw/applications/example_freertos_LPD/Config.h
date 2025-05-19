#ifndef _CONFIG_H
#define _CONFIG_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>


typedef unsigned int                      UINT16;
typedef unsigned long                     UINT32;
typedef unsigned char                     UINT8;
typedef int                               INT16;
typedef long                              INT32;
typedef char                              INT8;

typedef float                             FP32;
typedef double                            FP64;

//[EMRL] LED RGB LD5
#define GPIO_LD5_R  11
#define GPIO_LD5_B  12
#define GPIO_LD5_G  13

//[EMRL] Datos necesarios SPI PYNQ
#define DATA_CHUNK_W    1
#define DATA_CHUNK_B    1
#define SPI_BUFFER_SIZE 200 //Esta definición es de ERTIS
#define CHUNKS_NW       (CIR_TAPS/(DATA_CHUNK_W*4)) + ((CIR_TAPS%(DATA_CHUNK_W*4))!=0)
#define CHUNKS_NB       (CIR_TAPS/DATA_CHUNK_B)

#define GPIO_SYNQ 10

#define DUMMY_CYCLES  32

#define SYNC_LOGIC(x) x


//[EMRL] En nuestro sistema ya hay un archivo stdint.h que define los tipos de datos
/*
#if !defined(__stdint_h) && !defined(_STDINT_H)
typedef unsigned long                     uint40_t;
typedef unsigned int                      uint32_t;
typedef unsigned short                    uint16_t;
typedef unsigned char                     uint8_t;
typedef long                              int40_t;
typedef int                               int32_t;
typedef short                             int16_t;
//typedef char                              int8_t;

typedef float                             fp32_t;
typedef double                            fp64_t;
#else
#include <stdint.h>
#endif  // #ifndef __stdint_h
*/

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
