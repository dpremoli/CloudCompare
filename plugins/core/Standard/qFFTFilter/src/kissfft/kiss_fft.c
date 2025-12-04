/* kiss_fft.c - Simplified KissFFT implementation
 * This is a minimal, self-contained FFT for educational purposes
 * Based on the Cooley-Tukey FFT algorithm
 */

#include "kiss_fft.h"
#include <string.h>

#define MAXFACTORS 32
#define kiss_fft_scalar float

/* FFT configuration structure */
struct kiss_fft_state {
    int nfft;              /* FFT size */
    int inverse;           /* 0=forward, 1=inverse */
    int factors[2*MAXFACTORS];
    kiss_fft_cpx twiddles[1];  /* Variable length array */
};

/* Complex multiplication: c = a * b */
static void C_MUL(kiss_fft_cpx* c, kiss_fft_cpx a, kiss_fft_cpx b)
{
    c->r = a.r * b.r - a.i * b.i;
    c->i = a.r * b.i + a.i * b.r;
}

/* Complex addition: c = a + b */
static void C_ADD(kiss_fft_cpx* c, kiss_fft_cpx a, kiss_fft_cpx b)
{
    c->r = a.r + b.r;
    c->i = a.i + b.i;
}

/* Complex subtraction: c = a - b */
static void C_SUB(kiss_fft_cpx* c, kiss_fft_cpx a, kiss_fft_cpx b)
{
    c->r = a.r - b.r;
    c->i = a.i - b.i;
}

/* Factor n into small primes */
static void kf_factor(int n, int* facbuf)
{
    int p = 4;
    int floor_sqrt = (int)floor(sqrt((double)n));
    int idx = 0;
    
    /* Factor out powers of 4, 2, 3, 5, and other primes */
    do {
        while (n % p) {
            switch (p) {
                case 4: p = 2; break;
                case 2: p = 3; break;
                default: p += 2; break;
            }
            if (p > floor_sqrt)
                p = n;
        }
        n /= p;
        facbuf[idx++] = p;
        facbuf[idx++] = n;
    } while (n > 1);
}

/* Butterfly operation for FFT */
static void kf_bfly2(kiss_fft_cpx* Fout, const size_t fstride, 
                     const kiss_fft_cfg st, int m)
{
    kiss_fft_cpx* Fout2;
    kiss_fft_cpx* tw1 = st->twiddles;
    kiss_fft_cpx t;
    
    Fout2 = Fout + m;
    
    do {
        C_MUL(&t, *Fout2, *tw1);
        tw1 += fstride;
        C_SUB(Fout2, *Fout, t);
        C_ADD(Fout, *Fout, t);
        ++Fout2;
        ++Fout;
    } while (--m);
}

/* Generic butterfly for arbitrary factor */
static void kf_bfly_generic(kiss_fft_cpx* Fout, const size_t fstride,
                            const kiss_fft_cfg st, int m, int p)
{
    int u, k, q1, q;
    kiss_fft_cpx t;
    int Norig = st->nfft;
    kiss_fft_cpx* scratch = (kiss_fft_cpx*)malloc(sizeof(kiss_fft_cpx) * p);
    
    for (u = 0; u < m; ++u) {
        k = u;
        for (q1 = 0; q1 < p; ++q1) {
            scratch[q1] = Fout[k];
            k += m;
        }
        
        k = u;
        for (q1 = 0; q1 < p; ++q1) {
            int twidx = 0;
            Fout[k] = scratch[0];
            for (q = 1; q < p; ++q) {
                twidx += fstride * k;
                if (twidx >= Norig) twidx -= Norig;
                C_MUL(&t, scratch[q], st->twiddles[twidx]);
                C_ADD(&Fout[k], Fout[k], t);
            }
            k += m;
        }
    }
    
    free(scratch);
}

/* Recursive FFT work function */
static void kf_work(kiss_fft_cpx* Fout, const kiss_fft_cpx* f,
                   const size_t fstride, int in_stride,
                   int* factors, const kiss_fft_cfg st)
{
    kiss_fft_cpx* Fout_beg = Fout;
    const int p = *factors++;  /* radix */
    const int m = *factors++;  /* stage's FFT length / p */
    const kiss_fft_cpx* Fout_end = Fout + p * m;
    
    if (m == 1) {
        /* Copy input to output */
        do {
            *Fout = *f;
            f += fstride * in_stride;
        } while (++Fout != Fout_end);
    } else {
        do {
            kf_work(Fout, f, fstride * p, in_stride, factors, st);
            f += fstride * in_stride;
        } while ((Fout += m) != Fout_end);
    }
    
    Fout = Fout_beg;
    
    /* Apply butterfly */
    switch (p) {
        case 2: kf_bfly2(Fout, fstride, st, m); break;
        default: kf_bfly_generic(Fout, fstride, st, m, p); break;
    }
}

/* Allocate FFT configuration */
kiss_fft_cfg kiss_fft_alloc(int nfft, int inverse_fft, void* mem, size_t* lenmem)
{
    kiss_fft_cfg st = NULL;
    size_t memneeded = sizeof(struct kiss_fft_state) + 
                       sizeof(kiss_fft_cpx) * (nfft - 1);
    
    if (lenmem == NULL) {
        st = (kiss_fft_cfg)malloc(memneeded);
    } else {
        if (*lenmem >= memneeded)
            st = (kiss_fft_cfg)mem;
        *lenmem = memneeded;
    }
    
    if (st) {
        st->nfft = nfft;
        st->inverse = inverse_fft;
        
        /* Compute twiddle factors */
        for (int i = 0; i < nfft; ++i) {
            const double pi = 3.141592653589793238462643383279502884197;
            double phase = -2 * pi * i / nfft;
            if (st->inverse)
                phase *= -1;
            st->twiddles[i].r = (kiss_fft_scalar)cos(phase);
            st->twiddles[i].i = (kiss_fft_scalar)sin(phase);
        }
        
        kf_factor(nfft, st->factors);
    }
    
    return st;
}

/* Execute FFT */
void kiss_fft(kiss_fft_cfg st, const kiss_fft_cpx* fin, kiss_fft_cpx* fout)
{
    kf_work(fout, fin, 1, 1, st->factors, st);
}

/* Free FFT configuration */
void kiss_fft_free(kiss_fft_cfg cfg)
{
    free(cfg);
}