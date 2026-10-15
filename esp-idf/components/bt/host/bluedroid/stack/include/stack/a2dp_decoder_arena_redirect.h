/*
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Force-included into third-party decoder libraries (see CMakeLists.txt) so
 * their malloc/calloc/free use the shared A2DP decoder arena without
 * patching the library sources. The system headers are included first so
 * their prototypes are not rewritten by the macros below.
 */

#ifndef A2DP_DECODER_ARENA_REDIRECT_H
#define A2DP_DECODER_ARENA_REDIRECT_H

#include <stdlib.h>
#include <malloc.h>
#include "stack/a2dp_decoder_arena.h"

#define malloc(size)        a2dp_dec_arena_malloc(size)
#define calloc(nmemb, size) a2dp_dec_arena_calloc(nmemb, size)
#define free(ptr)           a2dp_dec_arena_free(ptr)

#endif // A2DP_DECODER_ARENA_REDIRECT_H
