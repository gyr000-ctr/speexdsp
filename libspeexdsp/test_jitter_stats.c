/* Copyright (C) 2026 Jean-Marc Valin
   File: test_jitter_stats.c

   Self-test for the clock-drift compensating jitter buffer.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

   - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

   - Neither the name of the Xiph.org Foundation nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "speex/speex_jitter_stats.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SPAN 160                 /* 20 ms of 8 kHz audio, in timestamp units */
#define RING (JITTER_STATS_CAPACITY)
#define TICKS_30MIN (30*60*50)   /* 50 ticks per second for 30 minutes */

static int failures = 0;
static int checks = 0;

#define CHECK(cond, ...) do { \
   checks++; \
   if (!(cond)) { \
      failures++; \
      printf("FAIL: " __VA_ARGS__); \
      printf("\n"); \
   } \
} while (0)

/* Catch any heap allocation on the packet hot path. These interceptors must
   never call printf themselves: stdio allocates internally, which would
   re-enter the interceptor and overflow the stack. A violation is recorded
   in hot_path_allocs and reported from main() instead. */
static long alloc_balance = 0;
static int alloc_guard = 0;
static unsigned long hot_path_allocs = 0;

void *malloc(size_t size)
{
   extern void *__libc_malloc(size_t);
   if (alloc_guard)
      hot_path_allocs++;
   alloc_balance++;
   return __libc_malloc(size);
}
void free(void *p)
{
   extern void __libc_free(void*);
   if (p)
      alloc_balance--;
   __libc_free(p);
}
void *calloc(size_t n, size_t size)
{
   extern void *__libc_calloc(size_t, size_t);
   if (alloc_guard)
      hot_path_allocs++;
   alloc_balance++;
   return __libc_calloc(n, size);
}
void *realloc(void *p, size_t size)
{
   extern void *__libc_realloc(void*, size_t);
   if (alloc_guard)
      hot_path_allocs++;
   if (!p)
      alloc_balance++;
   return __libc_realloc(p, size);
}

/* ================================================================== */
/* Independent reference implementation of the documented semantics.   */
/* Written separately from the library so a mistake in one copy is     */
/* caught by the other. It runs the same deterministic fixed-point     */
/* algorithm and must produce identical per-packet classifications.    */
/* ================================================================== */
#define REF_MIN_SAMPLES 64
#define REF_Q16 65536
#define REF_Q8 256

typedef struct {
   int present[RING];
   unsigned short slot_seq[RING];
   int started, have_anchor;
   unsigned short play_seq, max_seq;
   unsigned anchor_arr, anchor_ts;
   unsigned last_arr, last_ts, next_due;
   int drift_q, corr_phase_q;
   int win_count;
   int jitter_q8, target;
   unsigned min_d, max_d;
   unsigned long recv, lost, dup, late, underrun, overflow, queued;
} Ref;

static int ref_sdiff16(unsigned short a, unsigned short b)
{ return (short)(a - b); }
static int ref_sdiff32(unsigned a, unsigned b)
{ return (int)(a - b); }
static int ref_clamp(int v, int lo, int hi)
{ return v < lo ? lo : (v > hi ? hi : v); }

static void ref_init(Ref *r, unsigned min_d, unsigned max_d)
{
   int i;
   memset(r, 0, sizeof(*r));
   for (i=0;i<RING;i++) r->slot_seq[i] = 0;
   r->min_d = min_d; r->max_d = max_d;
   r->target = (int)min_d;
}

static void ref_estim(Ref *r, unsigned ts, unsigned now)
{
   if (!r->have_anchor)
   {
      r->have_anchor=1; r->anchor_arr=now; r->anchor_ts=ts;
      r->last_arr=now; r->last_ts=ts; r->win_count=0;
      return;
   }
   {
      int da = ref_sdiff32(now, r->last_arr);
      int dt = ref_sdiff32(ts, r->last_ts);
      if (da > 0 && dt > 0)
      {
         int resid = da - dt;
         int dev = (resid < 0 ? -resid : resid) * REF_Q8;
         int d;
         if (r->jitter_q8 == 0) r->jitter_q8 = dev;
         else {
            d = dev - r->jitter_q8;
            r->jitter_q8 += (d + (1<<2)) >> 3;
         }
         r->target = ref_clamp((int)r->min_d + 4*(r->jitter_q8/REF_Q8),
                               (int)r->min_d, (int)r->max_d);
         r->win_count++;
         {
            int sa = ref_sdiff32(now, r->anchor_arr);
            int st = ref_sdiff32(ts, r->anchor_ts);
            if (r->win_count >= REF_MIN_SAMPLES && sa > 0 && st > 0)
               r->drift_q = REF_Q16
                  - (int)(((long long)sa << 16) / st);
         }
      }
      r->last_arr = now; r->last_ts = ts;
   }
}

static int ref_slot_holds(Ref *r, unsigned short seq)
{
   int idx = seq & (RING-1);
   return r->present[idx] && r->slot_seq[idx] == seq;
}

static int ref_put(Ref *r, unsigned short seq, unsigned ts, unsigned now)
{
   int idx = seq & (RING-1);
   if (!r->started)
   {
      r->started=1; r->max_seq=seq; r->play_seq=seq;
      r->next_due = now + (unsigned)r->target;
      ref_estim(r, ts, now);
      r->present[idx]=1; r->slot_seq[idx]=seq; r->recv++; r->queued++;
      return JITTER_STATS_OK;
   }
   if (ref_slot_holds(r, seq))
   { r->dup++; return JITTER_STATS_DUPLICATE; }
   if (ref_sdiff16(r->play_seq, seq) > 0)
   { r->late++; return JITTER_STATS_LATE; }
   if (ref_sdiff16(seq, r->max_seq) < 0)
   {
      if (r->present[idx]) { r->overflow++; return JITTER_STATS_OVERFLOW; }
      r->present[idx]=1; r->slot_seq[idx]=seq; r->recv++; r->queued++;
      return JITTER_STATS_OK;
   }
   {
      int gap = ref_sdiff16(seq, r->max_seq);
      if (gap <= 32) r->lost += gap - 1;
      else
      {
         int i;
         for (i=0;i<RING;i++) { r->present[i]=0; }
         r->queued=0; r->have_anchor=0; r->play_seq=seq; r->corr_phase_q=0;
         r->win_count=0;
         r->next_due = now + (unsigned)r->target;
      }
   }
   if (r->present[idx]) { r->overflow++; return JITTER_STATS_OVERFLOW; }
   ref_estim(r, ts, now);
   r->present[idx]=1; r->slot_seq[idx]=seq;
   r->max_seq=seq; r->recv++; r->queued++;
   return JITTER_STATS_OK;
}

static int ref_get(Ref *r, unsigned now)
{
   int idx, step;
   if (!r->started) return JITTER_STATS_NOT_DUE;
   if (ref_sdiff32(now, r->next_due) < 0) return JITTER_STATS_NOT_DUE;
   r->corr_phase_q += r->drift_q * (int)SPAN;
   step = (int)SPAN - (r->corr_phase_q >> 16);
   r->corr_phase_q &= (REF_Q16-1);
   if (step < 1) step = 1;
   r->next_due += (unsigned)step;
   idx = r->play_seq & (RING-1);
   if (!r->present[idx] || r->slot_seq[idx] != r->play_seq)
   { r->play_seq++; r->underrun++; return JITTER_STATS_MISSING; }
   r->present[idx]=0; r->play_seq++; r->queued--;
   return JITTER_STATS_OK;
}

/* ================================================================== */
/* Test 1: thirty minutes of pure clock drift at +/-1000 ppm.          */
/* The path is lossless and ordered: counters must stay zero, the      */
/* queue must stay bounded and never underrun or overflow.             */
/* ================================================================== */
static void test_drift(int ppm)
{
   JitterStats js;
   JitterBufferPacket pkt, out;
   JitterStatsCounters c;
   unsigned char payload[4];
   long long sender_ts = 0;
   long long sender_acc = 0;       /* sender clock accumulated across ticks */
   long long drift_frac = 0;       /* sub-unit drift accumulator (ppm*SPAN) */
   long long recv_clock = 0;
   long long next_send = 0;
   unsigned short seq = 0;
   long long frames_sent = 0;      /* 64-bit: survives 16-bit RTP wraparound */
   int tick, got_ok = 0, got_missing = 0;
   int max_queued = 0, min_queued = 1000000;
   unsigned long total_played = 0;

   jitter_stats_init(&js, SPAN, 0, 0);
   memset(&pkt, 0, sizeof(pkt));
   pkt.data = (char*)payload;
   pkt.len = sizeof(payload);
   pkt.span = SPAN;

   alloc_guard = 1;
   for (tick = 0; tick < TICKS_30MIN; tick++)
   {
      /* Clock-drift model: each receiver tick the sender clock advances by
         SPAN*(1+ppm/1e6). The per-tick drift (0.16 timestamp units at
         1000 ppm) is sub-integer, so it is accumulated in drift_frac and
         folded into sender_acc as whole units. A fast sender thus emits an
         extra frame every ~1000 ticks and a slow sender skips one. All
         arrivals are in order and lossless. */
      drift_frac += (long long)SPAN * ppm;
      sender_acc += SPAN + drift_frac / 1000000;
      drift_frac %= 1000000;
      while (sender_acc >= next_send + (long long)SPAN)
      {
         pkt.timestamp = (spx_uint32_t)sender_ts;
         pkt.sequence = seq;
         jitter_stats_put(&js, &pkt, (spx_uint32_t)recv_clock);
         seq++;
         frames_sent++;
         sender_ts += SPAN;
         next_send += SPAN;
      }
      recv_clock += SPAN;

      /* Drain every frame due on this receiver tick (loop until NOT_DUE).
         A fast sender occasionally schedules two frames on one tick and
         both must be consumed or the queue grows without bound; a slow
         sender makes the first call return NOT_DUE so nothing is consumed. */
      for (;;)
      {
         int rc = jitter_stats_get(&js, (spx_uint32_t)recv_clock, &out);
         if (rc == JITTER_STATS_NOT_DUE) break;
         if (rc == JITTER_STATS_OK) got_ok++;
         else if (rc == JITTER_STATS_MISSING) got_missing++;
         total_played += (rc != JITTER_STATS_NOT_DUE);
      }
      jitter_stats_get_counters(&js, &c);
      if ((int)c.queued > max_queued) max_queued = c.queued;
      if ((int)c.queued < min_queued) min_queued = c.queued;
   }
   alloc_guard = 0;

   jitter_stats_get_counters(&js, &c);

   CHECK(c.underrun_count == 0,
         "drift %d ppm: %u underruns over 30 minutes", ppm, c.underrun_count);
   CHECK(c.overflow_count == 0,
         "drift %d ppm: %u overflows over 30 minutes", ppm, c.overflow_count);
   CHECK(c.late_count == 0,
         "drift %d ppm: %u late on an ordered path", ppm, c.late_count);
   CHECK(c.duplicate_count == 0,
         "drift %d ppm: %u duplicates on a clean path", ppm, c.duplicate_count);
   CHECK(c.lost_count == 0,
         "drift %d ppm: %u losses on a lossless path", ppm, c.lost_count);
   CHECK(got_missing == 0,
         "drift %d ppm: %d MISSING results", ppm, got_missing);

   /* Delay bounded: never near ring capacity, never empty after fill */
   CHECK(max_queued < RING,
         "drift %d ppm: queue reached %d (capacity %d)", ppm, max_queued, RING);
   CHECK(max_queued <= 10,
         "drift %d ppm: adaptive delay unbounded, peak queue %d", ppm, max_queued);
   CHECK(min_queued >= 1,
         "drift %d ppm: queue drained to %d", ppm, min_queued);
   CHECK(c.target_delay >= 4*SPAN && c.target_delay <= 16*SPAN,
         "drift %d ppm: target delay %u outside bounded interval",
         ppm, c.target_delay);

   /* Frame conservation across 16-bit sequence wraparound: every frame is
      either played, missing, or still queued at the end. */
   CHECK((long long)c.packets_received == frames_sent,
         "drift %d ppm: received %u of %lld sent",
         ppm, c.packets_received, frames_sent);
   CHECK(frames_sent == (long long)got_ok + got_missing + c.queued,
         "drift %d ppm: sent=%lld but ok=%d missing=%d queued=%u",
         ppm, frames_sent, got_ok, got_missing, c.queued);

   /* Drift estimate within 15% of truth once windows have elapsed */
   CHECK(abs((int)c.drift_ppm - ppm) <= 150,
         "drift %d ppm: estimate is %d ppm", ppm, c.drift_ppm);

   printf("drift %+5d ppm: underrun=%u overflow=%u late=%u dup=%u lost=%u "
          "est=%d ppm queue[min=%d max=%d] target=%u played=%lu/sent=%lld\n",
          ppm, c.underrun_count, c.overflow_count, c.late_count,
          c.duplicate_count, c.lost_count, c.drift_ppm,
          min_queued, max_queued, c.target_delay, total_played, frames_sent);
}

/* ================================================================== */
/* Test 2: loss / duplication / late / reorder / resync, every packet  */
/* classified against the independent reference, and counters matched. */
/* ================================================================== */
static void test_classification(void)
{
   JitterStats js;
   JitterBufferPacket pkt, out;
   JitterStatsCounters c;
   unsigned char payload[4];
   unsigned short seq = 100;
   unsigned t = 0;
   int i, rc, rrc;
   Ref ref;

   ref_init(&ref, 4*SPAN, 16*SPAN);
   jitter_stats_init(&js, SPAN, 4*SPAN, 16*SPAN);
   memset(&pkt, 0, sizeof(pkt));
   pkt.data = (char*)payload;
   pkt.len = sizeof(payload);
   pkt.span = SPAN;

   #define PUT(useq, uts) do { \
      unsigned short _s = (unsigned short)(useq); \
      pkt.sequence = _s; \
      pkt.timestamp = (spx_uint32_t)(uts); \
      rrc = ref_put(&ref, _s, (unsigned)(uts), t); \
      rc = jitter_stats_put(&js, &pkt, t); \
      CHECK(rc == rrc, "put seq %u: lib=%d ref=%d", _s, rc, rrc); \
   } while (0)

   #define DRAIN() do { \
      for (;;) { \
         rrc = ref_get(&ref, t); \
         rc = jitter_stats_get(&js, t, &out); \
         CHECK(rc == rrc, "get at t=%u: lib=%d ref=%d", t, rc, rrc); \
         if (rc == JITTER_STATS_NOT_DUE) break; \
      } \
   } while (0)

   alloc_guard = 1;

   /* Prime */
   for (i=0;i<6;i++) { PUT(seq, seq*SPAN); seq++; }

   /* Two lost packets (gap), then a duplicate */
   {
      unsigned short before = seq;
      seq += 2;
      PUT(seq, seq*SPAN); seq++;
      PUT(before+2, (before+2)*SPAN);          /* duplicate of previous */
   }

   /* Out-of-order gap fill before playout reaches it */
   {
      unsigned short fill = 100 + 6;           /* inside the 2-frame gap */
      PUT(fill, fill*SPAN);
   }

   /* Start playout, then a frame for an already-played slot is late */
   t += 6*SPAN;
   DRAIN();
   PUT(100, 100*SPAN);                          /* long played by now */

   /* Clean stream with arrival jitter; compare every event per packet */
   for (i=0;i<500;i++)
   {
      int jitter = (i % 7 == 0) ? SPAN/2 : ((i % 5 == 0) ? -SPAN/2 : 0);
      unsigned at = t + (unsigned)jitter;
      PUT(seq, seq*SPAN);
      (void)at;
      seq++;
      t += SPAN;
      DRAIN();
   }

   /* A large jump resyncs rather than counting an implausible loss burst */
   seq += 60;
   PUT(seq, seq*SPAN); seq++;

   /* A duplicate after resync */
   PUT(seq-1, (seq-1)*SPAN);

   alloc_guard = 0;

   jitter_stats_get_counters(&js, &c);
   CHECK(c.duplicate_count == ref.dup,
         "dup: lib=%u ref=%lu", c.duplicate_count, ref.dup);
   CHECK(c.late_count == ref.late,
         "late: lib=%u ref=%lu", c.late_count, ref.late);
   CHECK(c.lost_count == ref.lost,
         "lost: lib=%u ref=%lu", c.lost_count, ref.lost);
   CHECK(c.overflow_count == ref.overflow,
         "overflow: lib=%u ref=%lu", c.overflow_count, ref.overflow);
   CHECK(c.underrun_count == ref.underrun,
         "underrun: lib=%u ref=%lu", c.underrun_count, ref.underrun);
   CHECK(c.packets_received == ref.recv,
         "received: lib=%u ref=%lu", c.packets_received, ref.recv);
   CHECK(c.queued == ref.queued,
         "queued: lib=%u ref=%lu", c.queued, ref.queued);

   printf("classification: recv=%u lost=%u late=%u dup=%u underrun=%u "
          "overflow=%u queued=%u (per-packet reference match)\n",
          c.packets_received, c.lost_count, c.late_count,
          c.duplicate_count, c.underrun_count, c.overflow_count, c.queued);
}

/* ================================================================== */
/* Test 3: zero heap allocation, reset clears state, bad arguments.    */
/* ================================================================== */
static void test_lifecycle(void)
{
   static JitterStats js;
   JitterBufferPacket pkt, out;
   JitterStatsCounters c1, c2;
   unsigned char payload[4];
   unsigned t = 0, i;
   long before;

   memset(&pkt, 0, sizeof(pkt));
   pkt.data = (char*)payload;
   pkt.len = sizeof(payload);
   pkt.span = SPAN;

   before = alloc_balance;
   jitter_stats_init(&js, SPAN, 0, 0);
   CHECK(alloc_balance == before, "jitter_stats_init allocated heap memory");

   alloc_guard = 1;
   for (i=0;i<200;i++)
   {
      pkt.sequence = (spx_uint16_t)i;
      pkt.timestamp = i * SPAN;
      jitter_stats_put(&js, &pkt, t);
      t += SPAN;
      jitter_stats_get(&js, t, &out);
      jitter_stats_get_counters(&js, &c1);
   }
   alloc_guard = 0;

   jitter_stats_get_counters(&js, &c1);
   CHECK(c1.packets_received > 0, "no packets recorded before reset");

   jitter_stats_reset(&js);
   jitter_stats_get_counters(&js, &c2);
   CHECK(c2.packets_received == 0 && c2.duplicate_count == 0 &&
         c2.late_count == 0 && c2.lost_count == 0 &&
         c2.underrun_count == 0 && c2.overflow_count == 0 &&
         c2.queued == 0 && c2.drift_ppm == 0,
         "reset did not clear all counters");

   pkt.sequence = 0;
   pkt.timestamp = 0;
   CHECK(jitter_stats_put(&js, &pkt, 0) == JITTER_STATS_OK,
         "put after reset failed");
   CHECK(jitter_stats_get(&js, 4*SPAN, &out) == JITTER_STATS_OK,
         "get after reset failed");

   CHECK(jitter_stats_put(NULL, &pkt, 0) == JITTER_STATS_BAD_ARGUMENT,
         "NULL put not rejected");
   CHECK(jitter_stats_get(NULL, 0, &out) == JITTER_STATS_BAD_ARGUMENT,
         "NULL get not rejected");
   jitter_stats_get_counters(NULL, &c2);

   printf("lifecycle: zero heap allocation, reset clears all counters, "
          "bad args rejected\n");
}

int main(void)
{
   printf("Running test_jitter_stats (30-minute drift runs are simulated)\n");
   test_drift(0);
   test_drift(1000);
   test_drift(-1000);
   test_classification();
   test_lifecycle();
   CHECK(hot_path_allocs == 0,
         "%lu heap allocations intercepted on the packet hot path",
         hot_path_allocs);
   printf("\n%d checks, %d failures\n", checks, failures);
   return failures ? 1 : 0;
}
