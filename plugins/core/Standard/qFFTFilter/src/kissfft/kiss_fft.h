/* kiss_fft.h - Simplified KissFFT header
 * This is a minimal FFT implementation used as fallback when FFTW is not available
 * Original: https://github.com/mborgerding/kissfft
 */

#ifndef KISS_FFT_H
#define KISS_FFT_H

#include <stdlib.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Complex number structure */
typedef struct {
    float r;  /* real part */
    float i;  /* imaginary part */
} kiss_fft_cpx;

/* Opaque FFT configuration structure */
typedef struct kiss_fft_state* kiss_fft_cfg;

/* Allocate FFT configuration
 * nfft: FFT size (number of points)
 * inverse_fft: 0 for forward FFT, 1 for inverse FFT
 * mem: optional memory buffer (pass NULL for automatic allocation)
 * lenmem: length of mem buffer (pass NULL if mem is NULL)
 */
kiss_fft_cfg kiss_fft_alloc(int nfft, int inverse_fft, void* mem, size_t* lenmem);

/* Execute FFT
 * cfg: FFT configuration from kiss_fft_alloc
 * fin: input array of complex numbers (size nfft)
 * fout: output array of complex numbers (size nfft)
 */
void kiss_fft(kiss_fft_cfg cfg, const kiss_fft_cpx* fin, kiss_fft_cpx* fout);

/* Free FFT configuration */
void kiss_fft_free(kiss_fft_cfg cfg);

#ifdef __cplusplus
}
#endif

#endif /* KISS_FFT_H */