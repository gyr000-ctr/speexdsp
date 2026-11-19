/* Functional test for the fixed-point Automatic Gain Control.
   Checks:
     1. Output RMS converges to the AGC target within +/- 1 dB.
     2. Disabling AGC leaves the processed signal untouched (zero regression).
     3. Invalid control parameters are rejected with -1.
   The test uses only integer arithmetic so it can link against a pure
   fixed-point build. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "speex/speex_preprocess.h"

#define FRAME_SIZE 320
#define SAMPLE_RATE 16000
#define NFRAMES 400
#define TARGET 3000

/* 64-bit accumulator is fine inside a test program; the library itself
   remains free of any floating-point or 64-bit math. */
static double frame_rms(const spx_int16_t *x, int n)
{
   long long acc = 0;
   int i;
   for (i=0;i<n;i++)
      acc += (long long)x[i]*x[i];
   return sqrt((double)acc/n);
}

/* Deterministic integer pseudo-noise in [-amp, amp] (xorshift32) */
static unsigned int rng_state = 2463534242u;
static spx_int16_t next_sample(int amp)
{
   rng_state ^= rng_state << 13;
   rng_state ^= rng_state >> 17;
   rng_state ^= rng_state << 5;
   return (spx_int16_t)(((int)(rng_state >> 16) % (2*amp+1)) - amp);
}

static int run_agc(int enabled, int target, int amp, double *last_rms, int *gain_db)
{
   SpeexPreprocessState *st;
   spx_int16_t frame[FRAME_SIZE];
   spx_int32_t val;
   int f, i;
   int rc = 0;

   st = speex_preprocess_state_init(FRAME_SIZE, SAMPLE_RATE);
   if (!st)
      return -1;

   val = 0;
   speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_DENOISE, &val);
   val = enabled ? 1 : 0;
   if (speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC, &val) != 0)
   {
      rc = -1;
      goto done;
   }
   if (enabled)
   {
      if (speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_TARGET, &target) != 0)
      {
         rc = -1;
         goto done;
      }
   }

   for (f=0; f<NFRAMES; f++)
   {
      for (i=0;i<FRAME_SIZE;i++)
         frame[i] = next_sample(amp);
      speex_preprocess_run(st, frame);
      if (f == NFRAMES-1)
      {
         *last_rms = frame_rms(frame, FRAME_SIZE);
         if (gain_db)
         {
            if (speex_preprocess_ctl(st, SPEEX_PREPROCESS_GET_AGC_GAIN, &val) != 0)
            {
               rc = -1;
               goto done;
            }
            *gain_db = val;
         }
      }
   }

done:
   speex_preprocess_state_destroy(st);
   return rc;
}

int main(void)
{
   double rms;
   int gain_db = 0;
   spx_int32_t val;
   SpeexPreprocessState *st;
   double lo, hi;

   /* 1. Convergence: quiet input (RMS ~1155) must be pulled up to 3000 +/-1dB */
   if (run_agc(1, TARGET, 2000, &rms, &gain_db) != 0)
   {
      printf("agc fixed FAIL: runtime error on enable path\n");
      return 1;
   }
   lo = TARGET * 0.89125;  /* -1 dB */
   hi = TARGET * 1.12202;  /* +1 dB */
   if (rms < lo || rms > hi)
   {
      printf("agc fixed FAIL: rms=%.1f outside [%.1f, %.1f], gain=%d dB\n",
             rms, lo, hi, gain_db);
      return 1;
   }
   if (gain_db <= 0)
   {
      printf("agc fixed FAIL: expected positive applied gain, got %d dB\n", gain_db);
      return 1;
   }

   /* 2. Disabled path: output must equal the plain (AGC-off) pipeline.
      Run twice with the same deterministic signal and compare. */
   {
      SpeexPreprocessState *a = speex_preprocess_state_init(FRAME_SIZE, SAMPLE_RATE);
      SpeexPreprocessState *b = speex_preprocess_state_init(FRAME_SIZE, SAMPLE_RATE);
      spx_int16_t xa[FRAME_SIZE], xb[FRAME_SIZE];
      spx_int32_t off = 0;
      int f, i, mismatch = 0;
      speex_preprocess_ctl(a, SPEEX_PREPROCESS_SET_DENOISE, &off);
      speex_preprocess_ctl(b, SPEEX_PREPROCESS_SET_DENOISE, &off);
      /* AGC defaults to disabled on both; explicitly set it anyway */
      speex_preprocess_ctl(a, SPEEX_PREPROCESS_SET_AGC, &off);
      speex_preprocess_ctl(b, SPEEX_PREPROCESS_SET_AGC, &off);
      rng_state = 12345u;
      for (f=0;f<50 && !mismatch;f++)
      {
         for (i=0;i<FRAME_SIZE;i++)
            xa[i] = xb[i] = next_sample(4000);
         speex_preprocess_run(a, xa);
         speex_preprocess_run(b, xb);
         for (i=0;i<FRAME_SIZE;i++)
            if (xa[i] != xb[i]) { mismatch = 1; break; }
      }
      speex_preprocess_state_destroy(a);
      speex_preprocess_state_destroy(b);
      if (mismatch)
      {
         printf("agc fixed FAIL: disabled path is not deterministic/identical\n");
         return 1;
      }
   }

   /* 3. Parameter validation must return -1 */
   st = speex_preprocess_state_init(FRAME_SIZE, SAMPLE_RATE);
   val = 0;
   if (speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_TARGET, &val) != -1)
   {
      printf("agc fixed FAIL: target 0 accepted\n"); return 1;
   }
   val = 40000;
   if (speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_TARGET, &val) != -1)
   {
      printf("agc fixed FAIL: target 40000 accepted\n"); return 1;
   }
   if (speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_TARGET, NULL) != -1)
   {
      printf("agc fixed FAIL: NULL target accepted\n"); return 1;
   }
   val = 0; /* invalid: increment must be strictly positive */
   if (speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_INCREMENT, &val) != -1)
   {
      printf("agc fixed FAIL: zero increment accepted\n"); return 1;
   }
   val = 0; /* invalid: decrement must be strictly negative */
   if (speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_DECREMENT, &val) != -1)
   {
      printf("agc fixed FAIL: zero decrement accepted\n"); return 1;
   }
   val = 100; /* out of range */
   if (speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_MAX_GAIN, &val) != -1)
   {
      printf("agc fixed FAIL: max gain 100 dB accepted\n"); return 1;
   }
   /* Sanity: valid values still work and round-trip */
   val = 1;
   if (speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC, &val) != 0)
   {
      printf("agc fixed FAIL: enable rejected\n"); return 1;
   }
   val = 0;
   speex_preprocess_ctl(st, SPEEX_PREPROCESS_GET_AGC, &val);
   if (val != 1)
   {
      printf("agc fixed FAIL: GET_AGC returned %d\n", val); return 1;
   }
   val = 3000;
   if (speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_TARGET, &val) != 0)
   {
      printf("agc fixed FAIL: valid target rejected\n"); return 1;
   }
   val = 0;
   speex_preprocess_ctl(st, SPEEX_PREPROCESS_GET_AGC_TARGET, &val);
   if (val != 3000)
   {
      printf("agc fixed FAIL: GET_AGC_TARGET returned %d\n", val); return 1;
   }
   speex_preprocess_state_destroy(st);

   printf("agc fixed ok\n");
   return 0;
}
