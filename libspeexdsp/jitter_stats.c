/* Copyright (C) 2026 Jean-Marc Valin
   File: jitter_stats.c

   Clock-drift compensating RTP jitter buffer with deterministic statistics.
   All state is embedded in the JitterStats structure supplied by the caller;
   the per-packet code path never allocates memory.

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

#include "arch.h"
#include "speex/speex_jitter_stats.h"

#ifndef NULL
#define NULL 0
#endif

/* 64-bit intermediates keep the fixed-point divisions exact; C99 guarantees
   long long is at least 64 bits. */
typedef long long js_int64;

/* Signed 16-bit modular difference: a - b on the RTP sequence number circle */
#define SDIFF16(a,b) ((spx_int32_t)((spx_int16_t)((a)-(b))))
/* Signed 32-bit modular difference: a - b on the RTP timestamp circle */
#define SDIFF32(a,b) ((spx_int32_t)((spx_uint32_t)(a)-(spx_uint32_t)(b)))

#define Q16_ONE 65536
#define Q8_ONE  256

/* Drift slope is read directly from the long-term anchor once this many
   in-order packets have been seen: the accumulated span is then large enough
   that sub-unit-per-frame drift is not rounded away. */
#define DRIFT_MIN_SAMPLES 64
/* Jitter EWMA time constant, expressed as a power-of-two shift value */
#define JITTER_SHIFT 3       /* Jitter filter: new sample weight 1/8 */

/* A sequence jump larger than this many frames resyncs the estimators instead
   of being counted as an (implausibly large) loss burst */
#define RESYNC_GAP 32

/* Clamp helper for 32-bit signed values */
static spx_int32_t clampi(spx_int32_t v, spx_int32_t lo, spx_int32_t hi)
{
   if (v < lo)
      return lo;
   if (v > hi)
      return hi;
   return v;
}

/** Initialise jitter buffer. The state is caller-owned; nothing is allocated. */
EXPORT void jitter_stats_init(JitterStats *js, spx_uint32_t frame_span,
                              spx_uint32_t min_delay, spx_uint32_t max_delay)
{
   int i;
   if (!js || frame_span == 0)
      return;

   for (i=0;i<JITTER_STATS_CAPACITY;i++)
   {
      js->slots[i].used = 0;
      js->slots[i].data = NULL;
   }

   js->frame_span = frame_span;
   js->min_delay = min_delay ? min_delay : 4*frame_span;
   js->max_delay = max_delay ? max_delay : 16*frame_span;
   if (js->max_delay < js->min_delay)
      js->max_delay = js->min_delay;

   jitter_stats_reset(js);
}

/** Restore the jitter buffer to its initial state */
EXPORT void jitter_stats_reset(JitterStats *js)
{
   int i;
   if (!js)
      return;
   for (i=0;i<JITTER_STATS_CAPACITY;i++)
   {
      js->slots[i].used = 0;
      js->slots[i].data = NULL;
   }
   js->started = 0;
   js->play_seq = 0;
   js->max_seq = 0;
   js->have_anchor = 0;
   js->anchor_arr = 0;
   js->anchor_ts = 0;
   js->last_arr = 0;
   js->last_ts = 0;
   js->drift_q = 0;
   js->corr_phase_q = 0;
   js->next_due = 0;
   js->window_count = 0;
   js->jitter_dev_q8 = 0;
   js->target_delay = (spx_int32_t)js->min_delay;

   js->counters.packets_received = 0;
   js->counters.duplicate_count = 0;
   js->counters.late_count = 0;
   js->counters.lost_count = 0;
   js->counters.underrun_count = 0;
   js->counters.overflow_count = 0;
   js->counters.drift_ppm = 0;
   js->counters.target_delay = js->target_delay;
   js->counters.current_delay = 0;
   js->counters.queued = 0;
}

/** Update the drift and jitter estimators from one accepted, in-order packet.
    drift > 0 means the sender timestamp clock advances faster than the
    receiver clock (the receiver then has to consume frames a little faster
    than one per tick to keep up).

    Drift is the slope of receiver time against sender RTP time measured from
    a long-lived anchor. Per-frame drift is below one timestamp unit, so a
    packet-to-packet difference rounds it to zero; the accumulated span from
    the anchor grows without bound, making the slope estimate progressively
    more precise. The sign is flipped so positive means "sender fast". */
static void update_estimators(JitterStats *js, spx_uint32_t rtp_ts,
                              spx_uint32_t recv_now)
{
   if (!js->have_anchor)
   {
      js->have_anchor = 1;
      js->anchor_arr = recv_now;
      js->anchor_ts = rtp_ts;
      js->last_arr = recv_now;
      js->last_ts = rtp_ts;
      js->window_count = 0;
      return;
   }
   else
   {
      spx_int32_t d_arr = SDIFF32(recv_now, js->last_arr);
      spx_int32_t d_ts  = SDIFF32(rtp_ts, js->last_ts);
      if (d_arr > 0 && d_ts > 0)
      {
         spx_int32_t residual, dev_q8, delta;
         spx_int32_t span_arr, span_ts;

         /* One-way-delay variation between consecutive packets: zero mean on
            a clean constant-drift path, non-zero only where network jitter
            strikes (the constant clock skew cancels in the difference). */
         residual = d_arr - d_ts;
         dev_q8 = (residual < 0 ? -residual : residual) * Q8_ONE;
         if (js->jitter_dev_q8 == 0)
            js->jitter_dev_q8 = dev_q8;
         else
         {
            delta = dev_q8 - js->jitter_dev_q8;
            js->jitter_dev_q8 += (delta + (1 << (JITTER_SHIFT-1))) >> JITTER_SHIFT;
         }

         /* Adaptive target delay: standing margin plus a four-deviation
            jitter headroom, always within the configured bounded interval */
         js->target_delay = clampi((spx_int32_t)js->min_delay
                                   + 4*(js->jitter_dev_q8 / Q8_ONE),
                                   (spx_int32_t)js->min_delay,
                                   (spx_int32_t)js->max_delay);

         /* Long-term slope from the fixed anchor. */
         js->window_count++;
         span_arr = SDIFF32(recv_now, js->anchor_arr);
         span_ts  = SDIFF32(rtp_ts, js->anchor_ts);
         if (js->window_count >= DRIFT_MIN_SAMPLES && span_arr > 0 && span_ts > 0)
         {
            spx_int32_t raw_q =
               Q16_ONE - (spx_int32_t)(((js_int64)span_arr << 16) / span_ts);
            /* Track directly: with a growing anchor span the new slope is
               already well averaged, so no extra low-pass lag is needed. */
            js->drift_q = raw_q;
         }
      }
      js->last_arr = recv_now;
      js->last_ts = rtp_ts;
   }
}

/** Store a packet in its sequence-indexed ring slot */
static void store_packet(JitterStats *js, spx_uint16_t seq,
                         const JitterBufferPacket *packet)
{
   int idx = seq & (JITTER_STATS_CAPACITY-1);
   js->slots[idx].used = 1;
   js->slots[idx].timestamp = packet->timestamp;
   js->slots[idx].len = packet->len;
   js->slots[idx].user_data = packet->user_data;
   js->slots[idx].sequence = seq;
   js->slots[idx].data = packet->data;
   js->counters.packets_received++;
   js->counters.queued++;
}

/* True when the ring slot for "seq" still holds that exact sequence number.
   Matching on the full 16-bit sequence (not just the used flag) is what keeps
   a wraparound or an overrun from aliasing two different sequence numbers
   that happen to map to the same slot. */
static int slot_holds(JitterStats *js, spx_uint16_t seq)
{
   int idx = seq & (JITTER_STATS_CAPACITY-1);
   return js->slots[idx].used && js->slots[idx].sequence == seq;
}

/** Insert one packet */
EXPORT int jitter_stats_put(JitterStats *js, const JitterBufferPacket *packet,
                            spx_uint32_t recv_now)
{
   spx_int16_t seq;
   spx_int32_t gap;
   int idx;

   if (!js || !packet)
      return JITTER_STATS_BAD_ARGUMENT;

   seq = (spx_int16_t)packet->sequence;

   /* First packet ever: establish the timeline */
   if (!js->started)
   {
      js->started = 1;
      js->max_seq = seq;
      js->play_seq = seq;
      js->next_due = recv_now + (spx_uint32_t)js->target_delay;
      update_estimators(js, packet->timestamp, recv_now);
      store_packet(js, seq, packet);
      js->counters.target_delay = js->target_delay;
      js->counters.current_delay = js->counters.queued * js->frame_span;
      return JITTER_STATS_OK;
   }

   gap = SDIFF16(seq, js->max_seq);
   idx = seq & (JITTER_STATS_CAPACITY-1);

   /* Sequence number already seen and still held: duplicate */
   if (slot_holds(js, seq))
   {
      js->counters.duplicate_count++;
      return JITTER_STATS_DUPLICATE;
   }

   /* Older than the playout cursor: hopelessly late. A slot that merely
      still carries a different, already-superseded sequence number is not a
      duplicate of this packet. */
   if (SDIFF16(js->play_seq, seq) > 0)
   {
      js->counters.late_count++;
      return JITTER_STATS_LATE;
   }

   /* Out-of-order packet filling a known gap (play_seq <= seq < max_seq and
      its own slot is free): accept it but do not feed the ordered
      estimators. If its slot is taken by another live sequence number the
      ring has overrun. */
   if (gap < 0)
   {
      if (js->slots[idx].used)
      {
         js->counters.overflow_count++;
         return JITTER_STATS_OVERFLOW;
      }
      store_packet(js, seq, packet);
      js->counters.current_delay = js->counters.queued * js->frame_span;
      return JITTER_STATS_OK;
   }

   /* Forward packet: count the sequence gap as losses unless it is so large
      that it must be a sender restart / long outage */
   if (gap <= RESYNC_GAP)
   {
      js->counters.lost_count += (spx_uint32_t)(gap - 1);
   } else {
      int i;
      /* Resync: drop everything still queued and restart the estimators */
      for (i=0;i<JITTER_STATS_CAPACITY;i++)
      {
         js->slots[i].used = 0;
         js->slots[i].data = NULL;
      }
      js->counters.queued = 0;
      js->have_anchor = 0;
      js->play_seq = seq;
      js->corr_phase_q = 0;
      js->window_count = 0;
      js->next_due = recv_now + (spx_uint32_t)js->target_delay;
   }

   if (js->slots[idx].used)
   {
      /* The ring only holds JITTER_STATS_CAPACITY frames; an occupied slot
         with a different sequence means playout has fallen impossibly far
         behind (true overflow), not a normal 16-bit wraparound. */
      js->counters.overflow_count++;
      return JITTER_STATS_OVERFLOW;
   }

   update_estimators(js, packet->timestamp, recv_now);
   store_packet(js, seq, packet);
   js->max_seq = seq;

   js->counters.target_delay = js->target_delay;
   js->counters.current_delay = js->counters.queued * js->frame_span;
   return JITTER_STATS_OK;
}

/** Fetch the packet currently due for playout */
EXPORT int jitter_stats_get(JitterStats *js, spx_uint32_t recv_now,
                            JitterBufferPacket *out)
{
   int idx;
   spx_int32_t step_q;

   if (!js)
      return JITTER_STATS_BAD_ARGUMENT;
   if (!js->started)
      return JITTER_STATS_NOT_DUE;

   /* Nothing due yet on the drift-compensated playout timeline */
   if (SDIFF32(recv_now, js->next_due) < 0)
      return JITTER_STATS_NOT_DUE;

   /* Drift correction on the playout clock. Positive drift means the sender
      is fast, so the average playout step is shortened by drift*span. The
      per-frame correction (0.16 timestamp units at 1000 ppm) is sub-integer,
      so it is accumulated as a Q16 phase in corr_phase_q: the integer part
      quantises the step while the fractional part survives across frames
      instead of being discarded. A slow sender makes the step longer than a
      span, so some receiver ticks do not reach the next due time (NOT_DUE). */
   js->corr_phase_q += js->drift_q * (spx_int32_t)js->frame_span;
   step_q = (spx_int32_t)js->frame_span - (js->corr_phase_q >> 16);
   /* Keep only the fractional phase for the next frame */
   js->corr_phase_q &= (Q16_ONE-1);
   if (step_q < 1)
      step_q = 1;
   js->next_due += (spx_uint32_t)step_q;

   idx = js->play_seq & (JITTER_STATS_CAPACITY-1);

   /* The slot must hold exactly the sequence due now. A used slot carrying
      a different sequence is stale aliasing after a wraparound/resync and is
      treated as a missing frame, never as valid data. */
   if (!js->slots[idx].used || js->slots[idx].sequence != js->play_seq)
   {
      /* Underrun: the playout slot passed without the packet */
      if (out)
      {
         out->data = NULL;
         out->len = 0;
         out->timestamp = 0;
         out->span = js->frame_span;
         out->sequence = js->play_seq;
         out->user_data = 0;
      }
      js->play_seq++;
      js->counters.underrun_count++;
      js->counters.current_delay = js->counters.queued * js->frame_span;
      return JITTER_STATS_MISSING;
   }

   if (out)
   {
      out->data = js->slots[idx].data;
      out->len = js->slots[idx].len;
      out->timestamp = js->slots[idx].timestamp;
      out->span = js->frame_span;
      out->sequence = js->slots[idx].sequence;
      out->user_data = js->slots[idx].user_data;
   }
   js->slots[idx].used = 0;
   js->slots[idx].data = NULL;
   js->play_seq++;
   if (js->counters.queued > 0)
      js->counters.queued--;
   js->counters.current_delay = js->counters.queued * js->frame_span;
   return JITTER_STATS_OK;
}

/** Copy out the current statistics */
EXPORT void jitter_stats_get_counters(const JitterStats *js,
                                      JitterStatsCounters *out)
{
   if (!js || !out)
      return;
   *out = js->counters;
   out->drift_ppm = (spx_int32_t)(((js_int64)js->drift_q * 1000000) >> 16);
   out->target_delay = js->target_delay;
   out->current_delay = (spx_int32_t)(js->counters.queued * js->frame_span);
   out->queued = js->counters.queued;
}
