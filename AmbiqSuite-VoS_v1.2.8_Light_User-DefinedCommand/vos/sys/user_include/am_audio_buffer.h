
#ifndef AM_AUDIO_BUFFER_H
#define AM_AUDIO_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#include "am_app_utils_ring_buffer.h"

//*****************************************************************************
//
// helper functions
//
//*****************************************************************************

//
// Read/Head combo enum name
//
typedef enum{
    AM_AUDIO_BUFFER_STEREO,             // raw audio data from 2 channels of mics
    AM_AUDIO_BUFFER_MONO,               // audio data filtered by dspc signal chain
    AM_AUDIO_BUFFER_ENCODED,            // compressed audio data
//    AM_AUDIO_BUFFER_COMM,
    AM_AUDIO_BUFFER_MAX
}enum_audio_buffer_t;

typedef struct{
    uint8_t * head;
    uint8_t * tail;
}audio_buffer_ptr_t;

#if (configUSE_Sensory_THF || configUSE_RetuneDSP_VS) && configUSE_PREROLL
#define BYTES_UNIVERSAL_BUFFER_SIZE (96 * 1024)
#elif configUSE_RetuneDSP_VS
#define BYTES_UNIVERSAL_BUFFER_SIZE (16 * 1024)
#elif configUSE_Cyberon_Spotter
#define BYTES_UNIVERSAL_BUFFER_SIZE (6 * 1024)
#else
#define BYTES_UNIVERSAL_BUFFER_SIZE (6 * 1024)    // 4KB for mSBC, 6KB for OPUS as a minimal buffer size.
#endif // (configUSE_Sensory_THF || configUSE_RetuneDSP_VS) && configUSE_PREROLL, configUSE_RetuneDSP_VS

extern void am_audio_buffer_init(void);

extern bool am_audio_buffer_push(enum_audio_buffer_t buffer_type, void *pvSource, uint32_t ui32Bytes);
extern uint32_t am_audio_buffer_pop(enum_audio_buffer_t buffer_type, void *pvDest, uint32_t ui32Bytes);

extern void am_audio_buffer_rewind (enum_audio_buffer_t buffer_type, uint32_t offset_back);

void am_audio_buffer_nested_push(enum_audio_buffer_t buffer_push_to, void *pvSource, uint32_t ui32Bytes);
void am_audio_buffer_nested_pop(enum_audio_buffer_t buffer_pop_from, void *pvDest, uint32_t ui32Bytes);

bool am_audio_universal_buffer_status_check(am_app_utils_ring_buffer_t* nested_buff_list);

#endif // AM_AUDIO_BUFFER_H
