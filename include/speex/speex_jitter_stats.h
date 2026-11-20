/* Copyright (C) 2026 Jean-Marc Valin */
/**
   @file speex_jitter_stats.h
   @brief Zero-allocation RTP jitter buffer with clock drift compensation and statistics
*/
/*
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

#ifndef SPEEX_JITTER_STATS_H
#define SPEEX_JITTER_STATS_H
/** @defgroup JitterStats JitterStats: Clock-drift compensating jitter buffer with statistics
 *  This jitter buffer estimates the sender/receiver clock drift from RTP
 *  timestamps and sequence numbers, compensates it on the playout timeline,
 *  adapts the target buffering delay to the observed network jitter, and
 *  keeps deterministic per-packet statistics (late/lost/duplicate/underrun/
 *  overflow). All storage is embedded in the JitterStats structure and the
 *  per-packet code path performs no heap allocation.
 *  @{
 */

#include "speex/speex_jitter.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Number of packet slots in the reorder ring (power of two) */
#define JITTER_STATS_CAPACITY 64

/** Packet accepted into the buffer */
#define JITTER_STATS_OK 0
/** Playout time for the next sequence number has not been reached yet */
#define JITTER_STATS_NOT_DUE 1
/** Playout is due but the packet is not available (underrun) */
#define JITTER_STATS_MISSING 2
/** A packet with this sequence number was already received */
#define JITTER_STATS_DUPLICATE 3
/** Packet arrived after its playout slot had already been skipped */
#define JITTER_STATS_LATE 4
/** Buffer ring is full, packet had to be dropped */
#define JITTER_STATS_OVERFLOW 5
/** Invalid argument */
#define JITTER_STATS_BAD_ARGUMENT -1

/** Statistics reported by the jitter buffer */
typedef struct JitterStatsCounters_ {
   spx_uint32_t packets_received;  /**< Unique packets accepted since reset */
   spx_uint32_t duplicate_count;   /**< Duplicate sequence numbers received */
   spx_uint32_t late_count;        /**< Packets that arrived past their playout slot */
   spx_uint32_t lost_count;        /**< Sequence gaps observed on arrival */
   spx_uint32_t underrun_count;    /**< Playout ticks with no packet available */
   spx_uint32_t overflow_count;    /**< Arrivals dropped because the ring was full */
   spx_int32_t drift_ppm;          /**< Estimated drift; positive means the sender
                                        clock runs faster than the receiver clock */
   spx_int32_t target_delay;       /**< Adaptive target delay (timestamp units) */
   spx_int32_t current_delay;      /**< Delay represented by the queued packets (timestamp units) */
   spx_uint32_t queued;            /**< Packets currently held in the ring */
} JitterStatsCounters;

/** One ring slot holding a single packet */
typedef struct JitterStatsSlot_ {
   spx_uint32_t timestamp;         /**< RTP timestamp of the stored packet */
   spx_uint32_t len;               /**< Packet length in bytes */
   spx_uint32_t user_data;         /**< Opaque user data copied from the packet */
   spx_uint16_t sequence;          /**< Full RTP sequence number stored here */
   void *data;                     /**< Zero-copy data pointer (caller owns the memory) */
   int used;                       /**< Non-zero when the slot holds a packet */
} JitterStatsSlot;

/** Clock-drift compensating jitter buffer state.
 *  The structure contains no pointers to owned memory: it can be allocated on
 *  the stack or statically and used without any heap allocation. */
struct JitterStats_ {
   JitterStatsSlot slots[JITTER_STATS_CAPACITY]; /**< Packet ring, indexed by sequence */

   spx_uint32_t frame_span;        /**< Nominal duration of one frame (timestamp units) */
   spx_uint32_t min_delay;         /**< Lower bound for the adaptive target delay */
   spx_uint32_t max_delay;         /**< Upper bound for the adaptive target delay */

   int started;                    /**< Playout has begun */
   spx_uint16_t play_seq;          /**< Sequence number due next */
   spx_uint16_t max_seq;           /**< Highest sequence number accepted */
   int have_anchor;                /**< Drift/transit estimators are initialised */

   spx_uint32_t anchor_arr;        /**< Receiver time at the long-term estimator anchor */
   spx_uint32_t anchor_ts;         /**< RTP timestamp at the long-term estimator anchor */
   spx_uint32_t last_arr;          /**< Receiver time of the previous sample */
   spx_uint32_t last_ts;           /**< RTP timestamp of the previous sample */
   spx_int32_t drift_q;            /**< Estimated drift, Q16 (positive = sender fast) */
   spx_int32_t corr_phase_q;       /**< Sub-unit drift correction phase, Q16 */
   spx_uint32_t next_due;          /**< Receiver time at which play_seq becomes due */

   int window_count;               /**< In-order packets seen since the anchor */
   spx_int32_t jitter_dev_q8;      /**< Smoothed |one-way-delay variation|, Q8 */
   spx_int32_t target_delay;       /**< Adaptive target delay, timestamp units */

   JitterStatsCounters counters;   /**< Public statistics */
};

/** Generic jitter buffer state */
typedef struct JitterStats_ JitterStats;

/** Initialise a jitter buffer (no heap allocation is performed).
 *
 * @param js State storage provided by the caller
 * @param frame_span Nominal duration of one frame in RTP timestamp units
 * @param min_delay Lower bound of the adaptive delay; 0 selects 4 frames
 * @param max_delay Upper bound of the adaptive delay; 0 selects 16 frames
 */
void jitter_stats_init(JitterStats *js, spx_uint32_t frame_span,
                       spx_uint32_t min_delay, spx_uint32_t max_delay);

/** Restore the jitter buffer to its initial state (does not free anything) */
void jitter_stats_reset(JitterStats *js);

/** Insert one packet.
 *
 * Updates the drift and jitter estimators and classifies the packet as
 * JITTER_STATS_OK, JITTER_STATS_DUPLICATE, JITTER_STATS_LATE or
 * JITTER_STATS_OVERFLOW. Sequence gaps ahead of the highest seen sequence
 * number are accounted as lost packets.
 *
 * @param js Jitter buffer state
 * @param packet Incoming packet (the data pointer is retained, not copied)
 * @param recv_now Current receiver time in RTP timestamp units
 */
int jitter_stats_put(JitterStats *js, const JitterBufferPacket *packet,
                     spx_uint32_t recv_now);

/** Fetch the packet currently due for playout, if any.
 *
 * Call this once per receiver tick, and call it again whenever it returns
 * JITTER_STATS_OK or JITTER_STATS_MISSING, until it returns
 * JITTER_STATS_NOT_DUE: a fast sender occasionally makes two frames due on
 * the same receiver tick, and draining them both is what keeps the queue from
 * growing without bound. The drift-corrected playout step is shortened for a
 * fast sender and lengthened for a slow one. When the sender clock runs slow,
 * some ticks have no frame due and the first call returns NOT_DUE without
 * consuming anything (the timeline is left untouched so a later tick catches
 * up), which prevents underruns.
 *
 * Returns JITTER_STATS_NOT_DUE when no frame is due on the corrected
 * timeline, JITTER_STATS_MISSING on an underrun (the due slot has no packet),
 * or JITTER_STATS_OK with the packet filled in otherwise.
 *
 * @param js Jitter buffer state
 * @param recv_now Current receiver time in RTP timestamp units
 * @param out Returned packet (zero-copy reference to the stored data)
 */
int jitter_stats_get(JitterStats *js, spx_uint32_t recv_now,
                     JitterBufferPacket *out);

/** Copy out the current statistics */
void jitter_stats_get_counters(const JitterStats *js, JitterStatsCounters *out);

/* @} */

#ifdef __cplusplus
}
#endif

#endif
