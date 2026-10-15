/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "stack/a2dp_decoder_arena.h"

#define TAG "A2D_ARENA"

#define ARENA_ALIGN     8u
#define ARENA_NONE      UINT32_MAX
#define ARENA_ROUND(n)  (((n) + (ARENA_ALIGN - 1u)) & ~(ARENA_ALIGN - 1u))

/* Header placed in front of every block; 8 bytes keeps payloads aligned. */
typedef struct {
    uint32_t prev;   /* offset of the previous block's header, or ARENA_NONE */
    uint32_t freed;
} arena_hdr_t;

static uint8_t s_arena[A2DP_DEC_ARENA_SIZE] __attribute__((aligned(ARENA_ALIGN)));
static uint32_t s_top;               /* first free byte */
static uint32_t s_last = ARENA_NONE; /* header offset of the topmost block */
static uint32_t s_peak;

static inline arena_hdr_t *arena_hdr(uint32_t off)
{
    return (arena_hdr_t *)(s_arena + off);
}

static inline int arena_owns(const void *ptr)
{
    const uint8_t *p = (const uint8_t *)ptr;
    return p >= s_arena && p < s_arena + sizeof(s_arena);
}

void *a2dp_dec_arena_malloc(size_t size)
{
    size_t need = a2dp_dec_arena_block_size(size);

    if (need > sizeof(s_arena) - s_top) {
        ESP_LOGW(TAG, "%u bytes do not fit (used %u of %u), using heap",
                 (unsigned)size, (unsigned)s_top, (unsigned)sizeof(s_arena));
        return heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }

    arena_hdr_t *hdr = arena_hdr(s_top);
    hdr->prev = s_last;
    hdr->freed = 0;
    s_last = s_top;
    s_top += need;
    if (s_top > s_peak) {
        s_peak = s_top;
    }
    return hdr + 1;
}

void *a2dp_dec_arena_calloc(size_t nmemb, size_t size)
{
    if (size && nmemb > SIZE_MAX / size) {
        return NULL;
    }
    void *p = a2dp_dec_arena_malloc(nmemb * size);
    if (p) {
        memset(p, 0, nmemb * size);
    }
    return p;
}

void a2dp_dec_arena_free(void *ptr)
{
    if (!ptr) {
        return;
    }
    if (!arena_owns(ptr)) {
        heap_caps_free(ptr);
        return;
    }

    ((arena_hdr_t *)ptr - 1)->freed = 1;

    /* Pop every freed block off the top of the stack. */
    while (s_last != ARENA_NONE && arena_hdr(s_last)->freed) {
        s_top = s_last;
        s_last = arena_hdr(s_last)->prev;
    }
}

void a2dp_dec_arena_reset(void)
{
    if (s_top) {
        ESP_LOGW(TAG, "reset with %u bytes still allocated", (unsigned)s_top);
    }
    s_top = 0;
    s_last = ARENA_NONE;
}

size_t a2dp_dec_arena_block_size(size_t size)
{
    return sizeof(arena_hdr_t) + ARENA_ROUND(size ? size : 1);
}

size_t a2dp_dec_arena_used(void)
{
    return s_top;
}

size_t a2dp_dec_arena_peak(void)
{
    return s_peak;
}
