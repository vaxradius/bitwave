
//*****************************************************************************
//
// this is the source file of stereo audio application RAM optimization.
// In this application, there are 3 nested buffers:BUFFER_STEREO, BUFFER_MONO and BUFFER_ENCODED.
// Data flowing path as following:
//
// PDM_ISR -> BUFFER_STEREO ->BUFFER_MONO -> BUFFER_ENCODED -> BLE
//
// 3 BUFFERs share the same physical buffer: g_ringUniversalBuffer. Each buffer use its own read_head and write_tail.
//
// Align with the above data flowing chart:
//
// 0: AM_AUDIO_BUFFER_STEREO
// 1: AM_AUDIO_BUFFER_MONO
// 2: AM_AUDIO_BUFFER_ENCODED
//
// buffer2 nested to buffer1 nested to buffer0
//
// data will be pop from 0, after processing, push to 1;
//
// pop from 1, after encoded, push to 2
//
// pop from 2, then pushed to BLE.
//
// We will move ptrs' positions when we push the data into ring buffer and some conditions meet (the nested buffer is empty)
//
//*****************************************************************************
#include "am_vos_sys_config.h"

#include "am_mcu_apollo.h"
#include "am_app_utils.h"
#include "am_vos_task.h"

#include "am_audio_buffer.h"

//*****************************************************************************
//
// fundamental functions: init, push, pop, rewind.
//
//*****************************************************************************
void am_audio_buffer_init(void)
{

    for(uint8_t i = 0; i < AM_AUDIO_BUFFER_MAX; i++)
    {
        g_sAmUtil.sRingBuf[i].ui32BufferHead_read = 0;
        g_sAmUtil.sRingBuf[i].ui32BufferTail_write = 0;
        g_sAmUtil.sRingBuf[i].ui32OverWriting = 0;
        g_sAmUtil.sRingBuf[i].ui32Capacity = BYTES_UNIVERSAL_BUFFER_SIZE;
        g_sAmUtil.sRingBuf[i].pui8Data = g_sAmUtil.pui8RingBuffer;
    }
}

// returns true if a push results in overwrite
bool am_audio_buffer_push(enum_audio_buffer_t buffer_type, void *pvSource, uint32_t ui32Bytes)
{
    if(buffer_type > AM_AUDIO_BUFFER_MAX)
    {
        // type error

    }
    else
    {
//        AM_CRITICAL_BEGIN_VOS;
        am_app_utils_ring_buffer_push(&(g_sAmUtil.sRingBuf[buffer_type]), pvSource, ui32Bytes, false);
//        AM_CRITICAL_END_VOS;
    }

    return(g_sAmUtil.sRingBuf[buffer_type].ui32OverWriting);
}


// returns the actual data bytes popped
uint32_t
am_audio_buffer_pop(enum_audio_buffer_t buffer_type, void *pvDest, uint32_t ui32Bytes)
{
    if(buffer_type > AM_AUDIO_BUFFER_MAX)
    {
        // type error
        return 0;
    }
    uint32_t ui32PopLen = 0;

//    AM_CRITICAL_BEGIN_VOS;
    ui32PopLen = am_app_utils_ring_buffer_pop(&(g_sAmUtil.sRingBuf[buffer_type]), pvDest, ui32Bytes);
//    AM_CRITICAL_END_VOS;

    return ui32PopLen;
}

bool am_audio_buffer_empty(enum_audio_buffer_t buffer_type)
{
    bool bRetValue = false;
    AM_CRITICAL_BEGIN_VOS;
    bRetValue = am_app_utils_ring_buffer_empty(&(g_sAmUtil.sRingBuf[buffer_type]));
    AM_CRITICAL_END_VOS;
    return bRetValue;
}

// rewind the index pointers of an audio buffer to a position in the past
// test: only for mono buffer
// enum_audio_buffer_t buffer_type,
void am_audio_buffer_rewind(enum_audio_buffer_t buffer_type, uint32_t offset_back)
{
    int8_t buff_indx = 0;

    if(buffer_type > AM_AUDIO_BUFFER_MAX)
    {
        // type error
        return;
    }

    AM_CRITICAL_BEGIN_VOS;

    g_sAmUtil.sRingBuf[buffer_type].ui32OverWriting = 0; // clear overwrite flag
    g_sAmUtil.sRingBuf[buffer_type].ui32BufferHead_read =
                        (g_sAmUtil.sRingBuf[buffer_type].ui32BufferTail_write + BYTES_UNIVERSAL_BUFFER_SIZE - offset_back) % BYTES_UNIVERSAL_BUFFER_SIZE;

    //AM_APP_LOG_DEBUG("-- Buffer rewind: offset_back = %d, head = %d, tail = %d\n", offset_back, g_sAmUtil.sRingBuf[buffer_type].ui32BufferHead_read, g_sAmUtil.sRingBuf[buffer_type].ui32BufferTail_write);

    for(buff_indx=buffer_type+1; buff_indx<AM_AUDIO_BUFFER_MAX; buff_indx++)
    {
        am_app_utils_flush_ring_buffer(&(g_sAmUtil.sRingBuf[buff_indx]));
        g_sAmUtil.sRingBuf[buff_indx].ui32BufferHead_read = g_sAmUtil.sRingBuf[buffer_type].ui32BufferHead_read;
        g_sAmUtil.sRingBuf[buff_indx].ui32BufferTail_write = g_sAmUtil.sRingBuf[buffer_type].ui32BufferHead_read;
    }

    AM_CRITICAL_END_VOS;

}

//
// Set audio buffer index into a specific position.
// It's effective only when the buffer is empty.
// Return:
//      true  --the head and tail are successfully changed.
//      false --the head and tail are unchanged.
//

bool am_audio_buffer_index_set(enum_audio_buffer_t buffer_type, uint32_t index)
{
    if(buffer_type > AM_AUDIO_BUFFER_MAX)
    {
        // type error
        return false;
    }


    if(am_audio_buffer_empty(buffer_type))
    {
//        AM_CRITICAL_BEGIN_VOS;
        g_sAmUtil.sRingBuf[buffer_type].ui32BufferHead_read = index;//(index + 160) % BYTES_UNIVERSAL_BUFFER_SIZE;
        g_sAmUtil.sRingBuf[buffer_type].ui32BufferTail_write = index;//(index + 160) % BYTES_UNIVERSAL_BUFFER_SIZE;
//        AM_CRITICAL_END_VOS;
        return true;
    }
    else
        return false;
}

//*****************************************************************************
//
// specific-optimized functions: nested_push
// parameters:
//          buffer_push_to: the target ring buffer where we push data to
//          buffer_origin: consider it as parent buffer, where pvSource data comes from
//          pvSource: data source
//          ui32Bytes: how many bytes needs to be pushed
//
// Notice:
//      the difference between nested_push and push is: nested_push will move the
//      parent buffer head and tail to the tail of sub-buffer if parent buffer is empty.
//
//*****************************************************************************
void am_audio_buffer_nested_push(enum_audio_buffer_t buffer_push_to, void *pvSource, uint32_t ui32Bytes)
{
//    bool bReturnVal = true;
    int8_t buff_indx = 0;
    AM_CRITICAL_BEGIN_VOS;
    am_audio_buffer_push(buffer_push_to, pvSource, ui32Bytes);
    for(buff_indx=buffer_push_to-1; buff_indx >= 0; buff_indx--)
    {
        if(!am_audio_buffer_empty((enum_audio_buffer_t)buff_indx))
            break;
//        bReturnVal = am_audio_buffer_index_set((enum_audio_buffer_t)buff_indx, g_sAmUtil.sRingBuf[buffer_push_to].ui32BufferTail_write);
        am_audio_buffer_index_set((enum_audio_buffer_t)buff_indx, g_sAmUtil.sRingBuf[buffer_push_to].ui32BufferTail_write);
//        if(buff_indx == AM_AUDIO_BUFFER_STEREO)
//        {
//            AM_APP_LOG_DEBUG("--\n");
//        }
    }
    AM_CRITICAL_END_VOS;
}

//*****************************************************************************
//
// specific-optimized functions: nested_pop
// parameters:
//          buffer_pop_from: the source ring buffer where we pop data from
//          buffer_dest: consider it as sub buffer, where the data go to finally
//          pvDest: buffer of destination
//          ui32Bytes: how many bytes needs to be pushed
//
// We don't move the ptrs while poping data
//
//*****************************************************************************
void am_audio_buffer_nested_pop(enum_audio_buffer_t buffer_pop_from, void *pvDest, uint32_t ui32Bytes)
{
    AM_CRITICAL_BEGIN_VOS;
    //
    // We don't move ptrs while poping
    //
    am_audio_buffer_pop(buffer_pop_from, pvDest, ui32Bytes);
    AM_CRITICAL_END_VOS;

}

//
// buffer overwritten check function(buff_1, buff_2)
// Usage:
//       check whether buff1 and buff2 overwritten with each other
//
// Return:
//       True:                  there is no over-write between 2 buffers
//       False:                 over-write happened between these 2 buffers!
bool am_audio_buffer_overwrite_check(am_app_utils_ring_buffer_t* p_nested_buff_1, am_app_utils_ring_buffer_t* p_nested_buff_2)
{
    //
    // check both 2 buffers empty or not.
    // if empty, there is no over-write could happen.
    //
    if(am_app_utils_ring_buffer_empty(p_nested_buff_1) || am_app_utils_ring_buffer_empty(p_nested_buff_2))
        return true;
    //
    // check whether nested_buff_1 tail is between head and tail of nested_buff_2
    //
    if(p_nested_buff_2->ui32BufferTail_write > p_nested_buff_2->ui32BufferHead_read)
    {
        if((p_nested_buff_1->ui32BufferTail_write > p_nested_buff_2->ui32BufferHead_read) &&
            (p_nested_buff_1->ui32BufferTail_write <= p_nested_buff_2->ui32BufferTail_write))
            return false;
    }
    if(p_nested_buff_2->ui32BufferTail_write < p_nested_buff_2->ui32BufferHead_read)
    {
        if((p_nested_buff_1->ui32BufferTail_write >p_nested_buff_2->ui32BufferHead_read) ||
            (p_nested_buff_1->ui32BufferTail_write <= p_nested_buff_2->ui32BufferTail_write))
            return false;
    }

    //
    // check whether nested_buff_2 tail is between head and tail of nested_buff_1
    //
    if(p_nested_buff_1->ui32BufferTail_write > p_nested_buff_1->ui32BufferHead_read)
    {
        if((p_nested_buff_2->ui32BufferTail_write > p_nested_buff_1->ui32BufferHead_read) &&
            (p_nested_buff_2->ui32BufferTail_write <= p_nested_buff_1->ui32BufferTail_write))
            return false;
    }
    if(p_nested_buff_1->ui32BufferTail_write < p_nested_buff_1->ui32BufferHead_read)
    {
        if((p_nested_buff_2->ui32BufferTail_write > p_nested_buff_1->ui32BufferHead_read) ||
            (p_nested_buff_2->ui32BufferTail_write <= p_nested_buff_1->ui32BufferTail_write))
            return false;
    }

    return true;
}

//
// buffer overwritten check function
// parameters:
//          nested_buff_list: the list of all paris of ptrs need to be checked.
//
//
bool am_audio_universal_buffer_status_check(am_app_utils_ring_buffer_t* nested_buff_list)
{
    uint32_t index = 0;
    uint32_t buff_indx_2 =0;
    for(index=0; index<AM_AUDIO_BUFFER_MAX; index++)
    {
        buff_indx_2 = index +1;

        while(buff_indx_2 < AM_AUDIO_BUFFER_MAX)
        {
            if(am_audio_buffer_overwrite_check(&nested_buff_list[index], &nested_buff_list[buff_indx_2]) == false)
                return false;
            buff_indx_2++;
        }

    }
    return true;

}

