#include "fft.h"
#include <z_math.h>
#include <stdio.h>

/*
 * FFT.c
 * Implementación de la Transformada Rápida de Fourier (FFT) e IFFT
 * utilizando el algoritmo Cooley-Tukey (Radix-2).
 *
 * - Convierte una señal en el dominio del tiempo al dominio de la frecuencia.
 * - Requiere que el tamaño de la FFT (N) sea una potencia de 2.
 * - La implementación incluye ordenamiento bit-reversal y operaciones butterfly.
 */

 // Tablas precomputadas de valores de seno y coseno para optimizar cálculos
const float sin_tb[] = {  //(PI PI/2 PI/4 PI/8 PI/16 ... PI/(2^k))
0.000000, 1.000000, 0.707107, 0.382683, 0.195090, 0.098017, 
0.049068, 0.024541, 0.012272, 0.006136, 0.003068, 0.001534, 
0.000767, 0.000383, 0.000192, 0.000096, 0.000048, 0.000024, 
0.000012, 0.000006, 0.000003 
};


const float cos_tb[] = {  //(PI PI/2 PI/4 PI/8 PI/16 ... PI/(2^k))
-1.000000, 0.000000, 0.707107, 0.923880, 0.980785, 0.995185, 
0.998795, 0.999699, 0.999925, 0.999981, 0.999995, 0.999999, 
1.000000, 1.000000, 1.000000, 1.000000 , 1.000000, 1.000000, 
1.000000, 1.000000, 1.000000
};


/*
 * FFT (Fast Fourier Transform)
 * === Parámetros ===
 * x : Array de números complejos de entrada.
 * N : Tamaño de la FFT (debe ser potencia de 2).
 * === Salida ===
 * x : Contiene la FFT de la señal de entrada.
 */

int fft(TYPE_FFT *x, uint32_t N)
{
	int i,j,l,k,ip;
	static uint32_t M = 0;
	static int le,le2;
	static TYPE_FFT_E sR,sI,tR,tI,uR,uI;

	// Calcular log2(N) para determinar el número de etapas de la FFT
	M = floor_log2_32(N);

    /*
     * PASO 1: Reordenamiento de bits (Bit-Reversal Sorting)
     * - Rearregla los datos de entrada para mejorar la eficiencia del algoritmo.
     * - Es necesario porque la FFT se ejecuta en niveles de potencias de 2.
     */
	l = N >> 1;
	j = l;
    ip = N-2;
    for (i=1; i<=ip; i++) {
        if (i < j) {
            tR = x[j].real;
			tI = x[j].imag;
            x[j].real = x[i].real;
			x[j].imag = x[i].imag;
            x[i].real = tR;
			x[i].imag = tI;
		}
		k = l;
		while (k <= j) {
            j = j - k;
			k = k >> 1;
		}
		j = j + k;
	}

	/*
     * PASO 2: Aplicación del Algoritmo FFT
     * - Se realizan operaciones butterfly para combinar coeficientes.
     * - Cada iteración reduce la cantidad de puntos a la mitad.
     */
	for (l=1; l<=M; l++) {   // Bucle principal, iterando sobre niveles de la FFT
		//le = (int)pow(2,l);
		le  = (int)(1 << l);	// Número de puntos en la etapa actual
		le2 = (int)(le >> 1); // Mitad del grupo
		uR = 1;
		uI = 0;

		// Selección de Twiddle Factors usando tablas precomputadas
        k = floor_log2_32(le2);
        sR = cos_tb[k]; //cos(PI / le2);
        sI = -sin_tb[k];  // -sin(PI / le2)

		for (j=1; j<=le2; j++) {   // loop for each sub DFT 
			//jm1 = j - 1;
			for (i=j-1; i<N; i+=le) {  // loop for each butterfly 
				ip = i + le2;
				//  Multiplicación compleja usando Twiddle Factors
				tR = x[ip].real * uR - x[ip].imag * uI;
				tI = x[ip].real * uI + x[ip].imag * uR;
				
				// Operaciones butterfly
				x[ip].real = x[i].real - tR;
				x[ip].imag = x[i].imag - tI;
				x[i].real += tR;
				x[i].imag += tI;
			}  /* Next i */

			// Actualización de Twiddle Factors para la siguiente iteración
			tR = uR;
			uR = tR * sR - uI * sI;
			uI = tR * sI + uI *sR;
		} // Next j 
	} // Next l

	return 0;
}

/*
 * FFT Algorithm para señales reales
 * === Inputs ===
 * x : complex numbers
 * N : nodes of FFT. @N should be power of 2, that is 2^(*)
 * === Output ===
 * - Separa los datos en partes reales e imaginarias antes de ejecutar la FFT estándar.
 */
int fft_real(TYPE_FFT *x, uint32_t N)
{
	int i,j,l,k;
	static uint32_t M = 0;
    static uint32_t ND4 = 0;
	static TYPE_FFT_E sR,sI,tR,tI,uR,uI;


    // Separación de datos en pares (real, imaginario)
    M = N >> 1;
    for (i=0; i<M; i++) {
        x[i].real = x[i<<1].real;
        x[i].imag = x[(i<<1)+1].real;
    }

	// Aplicar FFT en los datos reorganizados (N/2 points FFT)
    fft(x, M); 

	// Transformación de dominio de frecuencia (Even/Odd frequency domain decomposition)
    ND4 = N >> 2;
    for (i=1; i<ND4; i++) {
        j = M - i;
        k = i + M;
        l = j + M;
        x[k].real = (x[i].imag + x[j].imag) / 2;
        x[l].real = x[k].real;
        x[k].imag = -(x[i].real - x[j].real) / 2; 
        x[l].imag = -x[k].imag;
        x[i].real = (x[i].real + x[j].real) / 2;
        x[j].real = x[i].real;
        x[i].imag = (x[i].imag - x[j].imag) / 2;
        x[j].imag = -x[i].imag;
    }
    x[N-ND4].real = x[ND4].imag;
    x[M].real = x[0].imag;
    x[N-ND4].imag = 0;
    x[M].imag = 0;
    x[ND4].imag = 0;
    x[0].imag = 0;

    /* Complete last stage FFT */
    uR = 1;
    uI = 0;
    k = floor_log2_32(M);
    sR = cos_tb[k]; //cos(PI / M);
    sI = -sin_tb[k];  // -sin(PI / M)
    //sR = cos(PI / M);
    //sI = -sin(PI / M);

    for (i=0; i<M; i++) {   /* loop for each sub DFT */
        k = i + M;
        tR = x[k].real * uR - x[k].imag * uI;
        tI = x[k].real * uI + x[k].imag * uR;
        x[k].real = x[i].real - tR;
        x[k].imag = x[i].imag - tI;
        x[i].real += tR;
        x[i].imag += tI;

        tR = uR;
        uR = tR * sR - uI * sI;
        uI = tR * sI + uI *sR;
    } /* Next i */

	return 0;
}

/*
 *  Inversa de FFT (IFFT)
 * - Convierte la señal de vuelta al dominio del tiempo.
 * - Se invierte la parte imaginaria antes de aplicar la FFT.
 * - Proceso similar, pero normalizando los valores
 */
int ifft(TYPE_FFT *x, uint32_t N)
{
	int k = 0;

	// Invertir la parte imaginaria de cada coeficiente
	for (k=0; k<=N-1; k++) {
		x[k].imag = -x[k].imag;
	}

	// Aplicar FFT normal
	fft(x, N);    /* using FFT */

	// Normalización dividiendo cada componente entre N
	for (k=0; k<=N-1; k++) {
		x[k].real = x[k].real / N;
		x[k].imag = -x[k].imag / N;
	}

	return 0;
}

/*
 * IFFT para señales reales
 * === Inputs ===
 * x : complex numbers
 * N : nodes of FFT. @N should be power of 2, that is 2^(*)
 * === Output ===
 * the @x contains the result of FFT algorithm, so the original data
 * in @x is destroyed, please store them before using FFT.
 *  - Permite recuperar la señal original a partir de su espectro en frecuencia.
 */
int ifft_real(TYPE_FFT *x, uint32_t N)
{
	int k = 0;

	// Hacer que el dominio de frecuencia sea simétrico
	for (k=(N>>1)+1; k<N; k++) {
        x[k].real = x[N-k].real;
		x[k].imag = -x[N-k].imag;
	}

    // Sumar las partes reales e imaginarias
    for (k=0; k<N; k++) {
        x[k].real += x[k].imag;
    }

	fft_real(x, N);    /* using FFT */

	for (k=0; k<N; k++) {
		x[k].real = (x[k].real + x[k].imag) / N;
		x[k].imag = 0;
	}

	return 0;
}
