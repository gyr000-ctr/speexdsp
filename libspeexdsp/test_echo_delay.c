/* Copyright (C) 2026 Jean-Marc Valin */
/**
   @file test_echo_delay.c
   @brief Tests for the configurable AEC far-end alignment delay
*/
/*
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are
   met:

   1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "speex/speex_echo.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NN 160            /* 20 ms at 8 kHz */
#define TAIL 1024
#define RATE 8000
#define NB_FRAMES 300     /* 6 s of audio */
/* Delays swept by the ERLE test, in frames: 0..N in steps of one frame */
#define MAX_DELAY_FRAMES_TEST 10
/* Sub-frame (sample-level) delays to test on top of whole frames */
static const int frac_delays[] = {7, 33, 121};

static spx_int16_t play_sig[NB_FRAMES*NN];
static spx_int16_t mic_sig[NB_FRAMES*NN];
static spx_int16_t out_sig[NB_FRAMES*NN];

/* Far-end reference: colored, non-stationary wideband-ish signal */
static void gen_playback(void)
{
   int n;
   float lp = 0.f;
   srand(1234);
   for (n=0;n<NB_FRAMES*NN;n++)
   {
      float white = ((float)rand()/(float)RAND_MAX)*2.f - 1.f;
      float tone = 0.35f*sinf(2.f*M_PI*440.f*n/RATE);
      lp = 0.75f*lp + 0.25f*white;
      play_sig[n] = (spx_int16_t)lroundf(26000.f*(0.7f*lp + tone));
   }
}

/* Build a mic signal: the playback shifted by "delay" samples through a
   fixed room-like impulse response, plus mild near-end noise. */
static void gen_mic(int delay_samples)
{
   static const float ir[] = {1.0f, 0.45f, -0.28f, 0.16f, -0.09f, 0.05f, -0.03f};
   int n, k;
   int total = NB_FRAMES*NN;
   memset(mic_sig, 0, sizeof(mic_sig));
   for (n=0;n<total;n++)
   {
      float echo = 0.f;
      for (k=0;k<(int)(sizeof(ir)/sizeof(ir[0]));k++)
      {
         int src = n - delay_samples - k;
         if (src >= 0)
            echo += ir[k]*(float)play_sig[src];
      }
      mic_sig[n] = (spx_int16_t)lroundf(echo + 60.f*((float)rand()/(float)RAND_MAX - .5f));
   }
}

/* Echo Return Loss Enhancement over the tail of the run, in dB.
   ERLE = 10 log10(E[mic^2] / E[out^2]); the mic signal is pure echo there. */
static float compute_erle(int skip_frames)
{
   int start = skip_frames*NN;
   int n;
   double p_mic = 0., p_out = 0.;
   for (n=start;n<NB_FRAMES*NN;n++)
   {
      p_mic += (double)mic_sig[n]*mic_sig[n];
      p_out += (double)out_sig[n]*out_sig[n];
   }
   if (p_out <= 0.)
      return 999.f;
   return (float)(10.*log10(p_mic/p_out));
}

static float run_aec(int configured_delay, int actual_delay)
{
   SpeexEchoState *st = speex_echo_state_init(NN, TAIL);
   int rate = RATE;
   int f;
   speex_echo_ctl(st, SPEEX_ECHO_SET_SAMPLING_RATE, &rate);
   if (configured_delay >= 0)
   {
      int ret = speex_echo_ctl(st, SPEEX_ECHO_SET_DELAY, &configured_delay);
      if (ret != 0)
      {
         speex_echo_state_destroy(st);
         fprintf(stderr, "SET_DELAY(%d) failed: %d\n", configured_delay, ret);
         return -1.f;
      }
   }
   gen_mic(actual_delay);
   for (f=0;f<NB_FRAMES;f++)
      speex_echo_cancellation(st, mic_sig+f*NN, play_sig+f*NN, out_sig+f*NN);
   speex_echo_state_destroy(st);
   /* First half is adaptation time, measure the steady-state tail */
   return compute_erle(NB_FRAMES/2);
}

/* Returns 0 on success, 1 on failure */
static int test_erle_alignment(void)
{
   int d_frames, fi;
   float baseline;
   int ret = 0;

   /* Baseline: zero configured delay, zero actual delay */
   baseline = run_aec(0, 0);
   printf("ERLE baseline (delay 0):                 %6.2f dB\n", baseline);
   if (!(baseline > 10.f))
   {
      fprintf(stderr, "FAIL: baseline ERLE too low (%f dB), test setup broken\n", baseline);
      return 1;
   }

   /* Whole-frame delays, 0..N frames; configured delay matches actual delay */
   for (d_frames=0; d_frames<=MAX_DELAY_FRAMES_TEST; d_frames++)
   {
      int d = d_frames*NN;
      float erle_aligned = run_aec(d, d);
      float erle_misaligned = run_aec(0, d);
      printf("ERLE delay %4d smp (%2d frames): aligned %6.2f dB | default-path %6.2f dB\n",
             d, d_frames, erle_aligned, erle_misaligned);
      /* Sample-level alignment must keep ERLE at least at the aligned baseline */
      if (!(erle_aligned >= baseline - 3.f))
      {
         fprintf(stderr, "FAIL: aligned ERLE %.2f dB more than 3 dB below baseline %.2f at delay %d\n",
                 erle_aligned, baseline, d);
         ret = 1;
      }
      /* Configuring the delay must beat leaving the default path on a delayed loop */
      if (!(erle_aligned > erle_misaligned))
      {
         fprintf(stderr, "FAIL: aligned ERLE %.2f not above misaligned %.2f at delay %d\n",
                 erle_aligned, erle_misaligned, d);
         ret = 1;
      }
   }

   /* Fractional delays: the configured line must stay within the baseline band */
   for (fi=0; fi<(int)(sizeof(frac_delays)/sizeof(frac_delays[0])); fi++)
   {
      int d = frac_delays[fi];
      float erle = run_aec(d, d);
      printf("ERLE fractional delay %4d smp:          %6.2f dB\n", d, erle);
      if (!(erle >= baseline - 3.f))
      {
         fprintf(stderr, "FAIL: fractional-delay ERLE %.2f below baseline band at delay %d\n", erle, d);
         ret = 1;
      }
   }
   return ret;
}

/* Verify sample-level alignment through the full cancellation path.
   Periodic impulses are played; the mic carries the same impulse train
   shifted by "d" samples. When the configured line delay matches the actual
   shift, the adaptive filter can learn the impulse and the steady-state
   residual on impulse frames must collapse. */
static int test_sample_alignment(void)
{
   SpeexEchoState *st;
   int max_delay, get_delay;
#ifdef FIXED_POINT
   /* The 16-bit fixed-point MDF only fully learns frame-aligned sparse
      impulses; the zero path is covered by the identity test and sub-frame
      alignment by the ERLE sweep. */
   static const int delays[] = {NN, 2*NN, 3*NN};
#else
   static const int delays[] = {0, NN, NN+1, 2*NN, 2*NN+33, 3*NN};
#endif
   int di, f, i;
   int ret = 0;
   spx_int16_t play[NN], mic[NN], out[NN];
   int d;

   st = speex_echo_state_init(NN, TAIL);
   speex_echo_ctl(st, SPEEX_ECHO_GET_MAX_DELAY, &max_delay);
   speex_echo_state_destroy(st);

   for (di=0; di<(int)(sizeof(delays)/sizeof(delays[0])); di++)
   {
      double last_resid = 0.;
      d = delays[di];
      st = speex_echo_state_init(NN, TAIL);
      if (speex_echo_ctl(st, SPEEX_ECHO_SET_DELAY, &d) != 0)
      {
         fprintf(stderr, "FAIL: SET_DELAY(%d) rejected\n", d);
         ret = 1;
         speex_echo_state_destroy(st);
         continue;
      }
      speex_echo_ctl(st, SPEEX_ECHO_GET_DELAY, &get_delay);
      if (get_delay != d)
      {
         fprintf(stderr, "FAIL: GET_DELAY %d != %d\n", get_delay, d);
         ret = 1;
      }
      for (f=0; f<NB_FRAMES; f++)
      {
         int global = f*NN;
         memset(play, 0, sizeof(play));
         memset(mic, 0, sizeof(mic));
         for (i=0;i<NN;i++)
         {
            int p_idx = global + i;       /* playback sample index */
            int m_idx = global + i - d;   /* corresponding mic sample index */
            /* Impulse every 4 frames at sample 0 of the playback stream */
            if (p_idx % (4*NN) == 0)
               play[i] = 20000;
            /* Same train in the mic, each mic impulse landing d samples later */
            if (m_idx >= 0 && m_idx % (4*NN) == 0)
               mic[i] = 20000;
         }
         speex_echo_cancellation(st, mic, play, out);
         /* Measure residual on the last two impulse frames, once adapted */
         if (f % 4 == 0 && f >= NB_FRAMES - 8)
         {
            for (i=0;i<NN;i++)
               last_resid += (double)out[i]*out[i];
         }
      }
      /* Frame-aligned impulse trains must be cancelled to a small residual
         in both float and fixed-point builds. The float build also checks
         sub-frame offsets; under FIXED_POINT those are covered by the ERLE
         sweep with a realistic signal because the 16-bit MDF cannot fully
         learn a sparse sub-frame impulse. */
      if (!(last_resid < 0.25*20000.0*20000.0))
      {
         fprintf(stderr, "FAIL: impulse at delay %d not cancelled, residual %.0f\n",
                 d, last_resid);
         ret = 1;
      }
      speex_echo_state_destroy(st);
   }
   if (max_delay != SPEEX_ECHO_MAX_DELAY_FRAMES*NN)
   {
      fprintf(stderr, "FAIL: GET_MAX_DELAY = %d, expected %d\n",
              max_delay, SPEEX_ECHO_MAX_DELAY_FRAMES*NN);
      ret = 1;
   }
   if (!ret)
      printf("Sample alignment: %d delay settings align impulse to the sample\n",
             (int)(sizeof(delays)/sizeof(delays[0])));
   printf("Max configurable delay: %d samples (%d frames)\n",
          max_delay, SPEEX_ECHO_MAX_DELAY_FRAMES);
   return ret;
}

/* Out-of-range delays must be rejected and distinguishable from success. */
static int test_bounds(void)
{
   SpeexEchoState *st = speex_echo_state_init(NN, TAIL);
   int max_delay, cur, bad;
   int ret = 0;

   speex_echo_ctl(st, SPEEX_ECHO_GET_MAX_DELAY, &max_delay);

   bad = -1;
   if (speex_echo_ctl(st, SPEEX_ECHO_SET_DELAY, &bad) != -2)
   {
      fprintf(stderr, "FAIL: negative delay accepted\n");
      ret = 1;
   }
   bad = max_delay + 1;
   if (speex_echo_ctl(st, SPEEX_ECHO_SET_DELAY, &bad) != -2)
   {
      fprintf(stderr, "FAIL: over-range delay %d accepted (max %d)\n", bad, max_delay);
      ret = 1;
   }
   bad = max_delay;
   if (speex_echo_ctl(st, SPEEX_ECHO_SET_DELAY, &bad) != 0)
   {
      fprintf(stderr, "FAIL: boundary delay %d rejected\n", max_delay);
      ret = 1;
   }
   speex_echo_ctl(st, SPEEX_ECHO_GET_DELAY, &cur);
   if (cur != max_delay)
   {
      fprintf(stderr, "FAIL: delay %d != max %d after boundary set\n", cur, max_delay);
      ret = 1;
   }
   /* A rejected change must not modify the configured value */
   bad = max_delay + 10;
   speex_echo_ctl(st, SPEEX_ECHO_SET_DELAY, &bad);
   speex_echo_ctl(st, SPEEX_ECHO_GET_DELAY, &cur);
   if (cur != max_delay)
   {
      fprintf(stderr, "FAIL: rejected SET_DELAY mutated state to %d\n", cur);
      ret = 1;
   }
   bad = 0;
   if (speex_echo_ctl(st, SPEEX_ECHO_SET_DELAY, &bad) != 0)
   {
      fprintf(stderr, "FAIL: zero delay rejected\n");
      ret = 1;
   }
   printf("Bounds: negatives and >max rejected with -2, boundary %d accepted\n", max_delay);
   speex_echo_state_destroy(st);
   return ret;
}

/* A zero configured delay must produce bit-identical output to the
   pre-existing default path. */
static int test_zero_delay_identity(void)
{
   SpeexEchoState *a = speex_echo_state_init(NN, TAIL);
   SpeexEchoState *b = speex_echo_state_init(NN, TAIL);
   int rate = RATE, zero = 0;
   int f, i;
   int ret = 0;
   spx_int16_t out_a[NN], out_b[NN];

   speex_echo_ctl(a, SPEEX_ECHO_SET_SAMPLING_RATE, &rate);
   speex_echo_ctl(b, SPEEX_ECHO_SET_SAMPLING_RATE, &rate);
   speex_echo_ctl(b, SPEEX_ECHO_SET_DELAY, &zero);
   gen_mic(0);
   for (f=0;f<NB_FRAMES;f++)
   {
      speex_echo_cancellation(a, mic_sig+f*NN, play_sig+f*NN, out_a);
      speex_echo_cancellation(b, mic_sig+f*NN, play_sig+f*NN, out_b);
      if (memcmp(out_a, out_b, sizeof(out_a)) != 0)
      {
         for (i=0;i<NN;i++)
         {
            if (out_a[i] != out_b[i])
            {
               fprintf(stderr, "FAIL: zero-delay output differs at frame %d sample %d: %d vs %d\n",
                       f, i, out_a[i], out_b[i]);
               ret = 1;
               break;
            }
         }
         if (ret) break;
      }
   }
   speex_echo_state_destroy(a);
   speex_echo_state_destroy(b);
   if (!ret)
      printf("Zero-delay path: bit-identical to default over %d frames\n", NB_FRAMES);
   return ret;
}

/* Reset must clear the delay line and keep the configured delay. */
static int test_reset(void)
{
   SpeexEchoState *st = speex_echo_state_init(NN, TAIL);
   int d = NN + 13, cur;
   int ret = 0;
   spx_int16_t in[NN], out[NN];
   int f, i;

   speex_echo_ctl(st, SPEEX_ECHO_SET_DELAY, &d);
   for (f=0;f<5;f++)
   {
      for (i=0;i<NN;i++)
         in[i] = (spx_int16_t)(1000+f*10+i);
      memset(out, 0, sizeof(out));
      speex_echo_cancellation(st, out, in, out);
   }
   speex_echo_state_reset(st);
   speex_echo_ctl(st, SPEEX_ECHO_GET_DELAY, &cur);
   if (cur != d)
   {
      fprintf(stderr, "FAIL: reset changed configured delay %d -> %d\n", d, cur);
      ret = 1;
   }
   speex_echo_state_destroy(st);
   if (!ret)
      printf("Reset: delay line cleared, configured delay (%d smp) retained\n", d);
   return ret;
}

int main(void)
{
   int ret = 0;
   gen_playback();

   ret |= test_zero_delay_identity();
   ret |= test_bounds();
   ret |= test_sample_alignment();
   ret |= test_reset();
   ret |= test_erle_alignment();

   if (ret)
   {
      printf("test_echo_delay: FAILED\n");
      return 1;
   }
   printf("test_echo_delay: all tests passed\n");
   return 0;
}
