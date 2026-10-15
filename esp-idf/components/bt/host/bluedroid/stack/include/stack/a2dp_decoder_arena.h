/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef A2DP_DECODER_ARENA_H
#define A2DP_DECODER_ARENA_H

#include <stddef.h>

/*
 * Shared static arena for A2DP sink decoder state.
 *
 * Only one A2DP decoder is active at a time, so every codec's state (LDAC,
 * aptX, Opus, AAC) is carved from the same fixed buffer instead of the heap.
 * Codec switches then never fragment the internal heap.
 *
 * The arena is a stack allocator: blocks are released when they and every
 * block above them have been freed, which matches how the decoders tear
 * down. Requests that do not fit fall back to the internal heap with a
 * warning. Must only be used from the A2DP sink task.
 *
 * Sized for the largest decoder: Opus stereo 48 kHz (~26.5 KB).
 */
#define A2DP_DEC_ARENA_SIZE (27 * 1024)

void *a2dp_dec_arena_malloc(size_t size);
void *a2dp_dec_arena_calloc(size_t nmemb, size_t size);
void a2dp_dec_arena_free(void *ptr);

/* Release everything. Called by the sink after a decoder is cleaned up. */
void a2dp_dec_arena_reset(void);

/* Bytes of arena a single allocation of `size` consumes, header included. */
size_t a2dp_dec_arena_block_size(size_t size);

size_t a2dp_dec_arena_used(void);
size_t a2dp_dec_arena_peak(void);

#endif // A2DP_DECODER_ARENA_H
