/**
 * @file shift_register.h
 * @brief Public interface for the Shift Register module.
 *
 * Provides function declarations, macros, and data structures
 * that are exposed to other parts of the program.
 */

#ifndef __SHIFT_REGISTER_H
#define __SHIFT_REGISTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------- */
/*  Public type definitions                                                     */
/* ---------------------------------------------------------------------------- */
typedef enum
{
    SR_GPIO_LOW = 0,
    SR_GPIO_HIGH = 1
} Sr_GpioLevel_t;

typedef enum
{
    SR_LSBFIRST = 0,
    SR_MSBFIRST = 1
} Sr_BitOrder_t;

typedef struct
{
    uint8_t pin;
    uint16_t frequency;
    uint8_t dutyCycle;
} Sr_PwmChannel_t;

typedef struct
{
    uint8_t dataPin;
    uint8_t clockPin;
    uint8_t latchPin;
    uint8_t pwmChannelsCount;
    uint16_t maxOutputs;
    uint16_t latchHoldTimeUs;
    uint16_t resetPinHoldTimeUs;
    uint16_t shiftOutHoldTimeUs;
    bool initGpio;
} Sr_Config_t;

typedef struct Sr_Handler Sr_Handle_t;

/* ---------------------------------------------------------------------------- */
/*  Public function declarations                                                */
/* ---------------------------------------------------------------------------- */
Sr_Handle_t * Sr_Create(const Sr_Config_t * as_config);
bool Sr_Init(const Sr_Handle_t * as_handler);
bool Sr_SetNumberOfPwmChannels(const Sr_Handle_t * as_handler, const uint8_t au8_numberOfPwmChannels);
bool Sr_SetNumberOfOutputs(const Sr_Handle_t * as_handler, const uint16_t au16_numberOfOutputs);
void Sr_SetData(const Sr_Handle_t * as_handler, const uint8_t * au8_dataArr, const uint8_t au8_size);
void Sr_SetBitRangeInDataArray(const Sr_Handle_t * as_handler, const uint32_t au32_startBit, const uint32_t au32_bitLength, const uint32_t au32_data);
void Sr_CopyBitRange(const Sr_Handle_t * as_handler, const uint8_t * const au8_src, const uint32_t au32_bitStart, const uint32_t au32_bitEnd);
void Sr_SendByte(const Sr_Handle_t * as_handler, const uint8_t au8_data);
void Sr_ResetDataArray(const Sr_Handle_t * as_handler);
void Sr_SendData(const Sr_Handle_t * as_handler, const uint8_t * au8_data, const uint8_t au8_size);
void Sr_SetOutput(const Sr_Handle_t * as_handler, const uint16_t au16_output, const uint8_t au8_data);
void Sr_DoLatch(const Sr_Handle_t * as_handler);
void Sr_EnableOutputs(const Sr_Handle_t * as_handler);
void Sr_DisableOutputs(const Sr_Handle_t * as_handler);
void Sr_UpdateOutputs(const Sr_Handle_t * as_handler);

/* ---------------------------------------------------------------------------- */
/* Functions implemented by the application */
/* ---------------------------------------------------------------------------- */
bool Sr_cbk_InitGpio(const uint8_t au8_pin, const Sr_GpioLevel_t ae_gpioLevel);
bool Sr_cbk_SetGpio(const uint8_t au8_pin, const Sr_GpioLevel_t ae_gpioLevel);
void Sr_cbk_DelayUs(const uint64_t au64_timeUs);

/* ---------------------------------------------------------------------------- */
/*  Inline functions                                                            */
/* ---------------------------------------------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif /* __STP_LEDDRIVER_H */