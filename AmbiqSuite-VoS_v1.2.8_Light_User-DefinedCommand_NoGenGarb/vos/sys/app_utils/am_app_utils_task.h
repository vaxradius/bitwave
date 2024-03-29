//*****************************************************************************
//
//! @file am_app_utils_task.h
//!
//! @brief RTOS task factory / encapsulation which supports corresponding queues
//! with tasks.
//!
//! This module allows the creation of multiple tasks with corresponding queues.
//! The encapsulation offered in this module greatly simplifies the main application 
//!
//
//*****************************************************************************

#ifndef AM_APP_UTILS_TASK_H
#define AM_APP_UTILS_TASK_H

#include "am_vos_sys_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

typedef struct
{
    TaskHandle_t task;
    QueueHandle_t queue;
//  am_app_utils_task_enum_t parent;
} am_app_utils_task_t;

//*****************************************************************************
//
// App task list structure typedefs
//
//*****************************************************************************
// this enum list contains all app related application tasks between AM_TASK_NONE and AM_MAX_TASK
// where each index corresponds to a TASK or ISR.
// Naming convention for a task: AM_TASK_xxx
// Naming convention for an ISR: AM_ISR_xxx
typedef enum
{
    AM_APP_TASK_NONE = 0, // The enum must begin with this value as named.
    AM_APP_TASK_LED,

#if configUSE_LOG_UART0 || configUSE_PRINTF_UART0
    AM_APP_TASK_UART0,
#endif // configUSE_LOG_UART0 || configUSE_PRINTF_UART0

#if configUSE_STDIO_PRINTF
    AM_APP_TASK_STDIO,
#endif // configUSE_STDIO_PRINTF

    AM_APP_TASK_AUD_PROCESSING,

#if configUSE_MODEL_INFERENCE
    AM_APP_TASK_MODEL_INFERENCE,
#endif // configUSE_MODEL_INFERENCE

    AM_APP_TASK_AWE_TICK,

#if configUSE_BLE
    AM_APP_BLE,
#endif // configUSE_BLE

#if configUSE_AUDIO_CODEC
    AM_APP_TASK_CODEC,
#endif // configUSE_AUDIO_CODEC

#if configUSE_RTT_RECORDER || configUSE_AMU2S_RECORDER
    AM_APP_TASK_RTT_SWITCH,
#endif // configUSE_RTT_RECORDER || configUSE_AMU2S_RECORDER

#if USE_MAYA
#if configUSE_BUZZER
    AM_APP_TASK_BUZZER,
#endif // configUSE_BUZZER
    AM_APP_TASK_LOGIC,
#endif // USE_MAYA

#if configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP
    AM_APP_TASK_GSENSOR,
#endif // configUSE_GSENSOR_MOTION || configUSE_OVVP_DOUBLE_TAP

    AM_APP_ISR_GPIO,
    AM_APP_ISR_UART0,
    
#if USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
    AM_APP_ISR_PDM,
#endif // USE_DMIC_MB3 || USE_MAYA || USE_DMIC_MB3_VM3011
    
#if USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
    AM_APP_ISR_ADC,
#endif // USE_AMIC_MB3 || USE_AMIC_MB2 || USE_AMIC_MB3_VMB
    
    AM_SLEEP,
    AM_APP_MAX_TASK // The enum must end with this value as named.
} am_app_utils_task_enum_t;

//*****************************************************************************
//
// App timer list structure typedefs
//
//*****************************************************************************
typedef enum
{
    AM_APP_TIMER_NONE = 0,      // The enum must begin with this value as named.
    AM_APP_TIMER_HEART_BEAT,
    AM_APP_TIMER_GSENSOR_PERIOD,
    AM_APP_TIMER_LONG_PRESS,    // Timer for long press
    AM_APP_MAX_TIMER            // The enum must end with this value as named.
} am_app_utils_timer_enum_t;

typedef struct
{
    //Task Setup
    am_app_utils_task_enum_t indx; //Specify this task's index.
    
    TaskFunction_t pxTaskCode; //FreeRTOS function pointer to task
    const char* const pcName; // FreeRTOS name
    const uint16_t usStackDepth; // Stack Size
    void * const pvParameters; // FreeRTOS task parameter mechanism
    UBaseType_t uxPriority; // FreeRTOS Task Priority
    
    //Queue Setup
    const UBaseType_t uxQueueLength;
    //const UBaseType_t uxItemSize; //sizeof (queue_element)
    
    //Parent Task
//    am_app_utils_task_enum_t parent; //Specify parent task.
}am_app_utils_task_setup_t;

//******************************************************************
// System timer setup structure
//
//******************************************************************

typedef struct
{
    //Task Setup
    am_app_utils_timer_enum_t indx; //Specify this task's index.
    const char* const pcTimerName; // FreeRTOS name
    TickType_t xTimerPeriodInTicks;
    UBaseType_t uxAutoReload; 
    TimerCallbackFunction_t pxTaskCode; //FreeRTOS function pointer to task
}am_app_utils_timer_setup_t;


//
// QUEUE entry for all corresponding queues
// Structure definition of the message element in queue
//
typedef struct
{
    am_app_utils_task_enum_t Source; // The sender lets the receiver know the source.
    uint32_t ui32MessageType; // May be redefined per task to index different uses of *pData.
    union{
        uint32_t ui32Note;      // short message for simple communication between tasks
        uint32_t ui32Length;    // data length for long message
        uint32_t ui32Indx;      // index for printf string buffer
    }info;
    am_app_utils_ring_buffer_t* pDataBuffer; 

}am_app_utils_task_queue_element_t;

extern am_app_utils_task_t am_KWD_tasks[AM_APP_MAX_TASK];
extern TimerHandle_t am_KWD_timers[AM_APP_MAX_TIMER];
extern void am_app_utils_task_init(void);
extern void am_app_utils_task_create(am_app_utils_task_setup_t setup);
extern bool am_app_utils_task_send(am_app_utils_task_enum_t Source, am_app_utils_task_enum_t Dest, uint32_t ui32MessageType, uint32_t ui32Notice, am_app_utils_ring_buffer_t *pData);
extern bool am_app_utils_task_send_fromISR(am_app_utils_task_enum_t Source, am_app_utils_task_enum_t Dest, uint32_t ui32MessageType, uint32_t ui32Notice, am_app_utils_ring_buffer_t *pData);
//extern bool am_app_utils_task_send_up(am_app_utils_task_enum_t Source, uint32_t MessageType, void *pData);
//extern bool am_app_utils_task_send_up_fromISR(am_app_utils_task_enum_t Source, uint32_t MessageType, void *pData);
extern bool am_app_utils_task_read(am_app_utils_task_enum_t indx, am_app_utils_task_queue_element_t *Element);
extern TaskHandle_t am_app_utils_task_get_task_handle(am_app_utils_task_enum_t indx);
extern void am_app_utils_task_create_all_tasks(const am_app_utils_task_setup_t *task_array, uint8_t task_count);
extern void am_app_utils_timer_create_all_timers(const am_app_utils_timer_setup_t *setup_array, uint8_t timer_count);
extern void am_app_utils_task_suspend(am_app_utils_task_enum_t indx);
extern void am_app_utils_task_resume(am_app_utils_task_enum_t indx);

#endif // AM_APP_UTILS_TASK_H
