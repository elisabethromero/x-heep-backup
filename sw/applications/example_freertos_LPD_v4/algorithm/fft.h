 #ifndef FFT_H
 #define FFT_H
 
 #include <stdint.h>
 #include <math.h>
 
 #define FFT_SIZE 32  // Debe ser potencia de 2
 
 
 #include "Config.h"
 
 #define TYPE_FFT_E     float    // Type is the same with COMPLEX member    
 
 #ifndef PI
 #define PI             (3.14159265f)
 #endif
 
 typedef COMPLEX TYPE_FFT;  // Define COMPLEX in Config.h
 
 extern int fft(TYPE_FFT *x, uint32_t N);
 extern int fft_real(TYPE_FFT *x, uint32_t N);
 extern int ifft(TYPE_FFT *x, uint32_t N);
 extern int ifft_real(TYPE_FFT *x, uint32_t N);
 
 //void compute_fft(float *input, float *output, int size);
 

 #endif // FFT_H
 