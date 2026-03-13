/**
 * @file shift_register.cpp
 * @brief C++ class implementation
 */

#include "shift_register.hpp"

/* Include C header for C facade */
extern "C" {
#include "shift_register.h"
}

namespace m_shiftregister {

/**
 * @brief Default constructor.
 */
ShiftRegister::ShiftRegister() noexcept
    : _initialized{false},
      _number_of_outputs{0U}
{
};

/**
 * @brief Parameterized constructor.
 * @param dataPin GPIO pin for data.
 * @param clockPin GPIO pin for clock.
 * @param latchPin GPIO pin for latch.
 * @param enPin GPIO pin for enable.
 * @param number_of_outputs Number of outputs to manage.
 * @param latch_hold_time_us Latch hold time in microseconds.
 * @param reset_pin_hold_time_us Reset pin hold time in microseconds.
 * @param data_pin_hold_time_us Data pin hold time in microseconds.
 * @return ShiftRegister instance.
 */
ShiftRegister::ShiftRegister(
    uint8_t dataPin, 
    uint8_t clockPin, 
    uint8_t latchPin,
    uint8_t enPin,
    uint16_t number_of_outputs,
    uint16_t latch_hold_time_us,
    uint16_t reset_pin_hold_time_us,
    uint16_t data_pin_hold_time_us,
    uint16_t clock_pin_hold_time_us,
    bool initGpio
) noexcept
    : _initialized{false},
      _data_pin{dataPin},
      _clock_pin{clockPin},
      _latch_pin{latchPin},
      _en_pin{enPin},
      _number_of_outputs{number_of_outputs},
      _latch_hold_time_us{latch_hold_time_us},
      _reset_pin_hold_time_us{reset_pin_hold_time_us},
      _clock_pin_hold_time_us{clock_pin_hold_time_us},
      _initGpio{initGpio}
{
    init();
}

/**
 * @brief Destructor.
 */
ShiftRegister::~ShiftRegister() noexcept
{

}

/**
 * @brief Initializes the Shift Register.
 */
bool ShiftRegister::init() noexcept
{
    if (_initialized)
    {
        return true; // Already initialized
    }

    if (_initGpio)
    {
        Sr_cbk_InitGpio(_data_pin, Sr_GpioLevel_t::SR_GPIO_LOW);
        Sr_cbk_InitGpio(_clock_pin, Sr_GpioLevel_t::SR_GPIO_LOW);
        Sr_cbk_InitGpio(_latch_pin, Sr_GpioLevel_t::SR_GPIO_LOW);
    }
    _initialized = true;
    return true;
}

/**
 * @brief Sets the number of outputs managed by the driver.
 * @param in_number_of_outputs The desired number of outputs.
 */
bool ShiftRegister::setNumberOfOutputs(const uint16_t in_number_of_outputs) noexcept
{
    if (!isInitialized())
    {
        return false;
    }
    /* set everything to zero before resizing */
    _arrayReset();
    updateOutputs();
    disableOutputs();

    _number_of_outputs = in_number_of_outputs;
    // _output_states.resize(_number_of_outputs);
    
    return true;
}

/**
 * @brief Sets the output state array from an external data array.
 * @param data_arr Pointer to the data array.
 * @param size Size of the data array.
 */
void ShiftRegister::setData(const uint8_t* data_arr, const uint16_t size) noexcept
{
    if (!isInitialized() || !data_arr)
    {
        return;
    }
    _arrayReset();
    for (uint16_t i = 0; i < size && i < _number_of_outputs; i++)
    {
        _output_states[i] = data_arr[i];
    }
}

/**
 * @brief Sets a range of bits in the output state array.
 * @param start_bit The starting bit index.
 * @param bit_length The length of bits to set.
 * @param data The data to set in the specified bit range.
 */
void ShiftRegister::setBitRangeInDataArray(const uint32_t start_bit, const uint32_t bit_length, const uint32_t data) noexcept
{
    if (!isInitialized())
    {
        return;
    }
    const uint32_t total_byte_length { (start_bit + bit_length) / (kBitsPerByte + 1U) }; // Total number of bytes to cover the bit range
    
    if (total_byte_length >= _number_of_outputs)
    {
        /* Incorrect bit range */
        return;
    }
    
    const uint32_t start_byte { start_bit / kBitsPerByte }; // Starting byte index
    const uint32_t start_offset { start_bit % kBitsPerByte }; // Bit position in the starting byte
    const uint32_t end_bit { start_bit + bit_length - 1U }; // Ending bit index
    const uint32_t end_byte { end_bit / kBitsPerByte }; // Ending byte index
    
    uint32_t dataTemp { data };
    
    for (uint32_t byte_index = start_byte; byte_index <= end_byte; byte_index++)
    {
        const uint32_t bit_start_in_byte { (byte_index == start_byte) ? start_offset : 0U };
        const uint32_t bit_end_in_byte { (byte_index == end_byte) ? (end_bit % kBitsPerByte) : 7U };
        const uint32_t num_bits { (bit_end_in_byte - bit_start_in_byte) + 1U };
        const uint8_t mask { static_cast<uint8_t>(((1U << num_bits) - 1U) << bit_start_in_byte) };
        
        _output_states[byte_index] &= ~mask;
        _output_states[byte_index] |= static_cast<uint8_t>(((dataTemp & ((1U << num_bits) - 1U)) << bit_start_in_byte) & mask);
        
        dataTemp >>= num_bits;
    }
}

/**
 * @brief Copies a range of bits from a source array to the output state array.
 * @param src Pointer to the source data array.
 * @param bit_start The starting bit index.
 * @param bit_end The ending bit index.
 */
void ShiftRegister::copyBitRange(const uint8_t* src, const uint32_t bit_start, const uint32_t bit_end) noexcept
{
    if (!isInitialized() || src == nullptr)
    {
        return;
    }
    _arrayReset();
    
    const uint32_t num_bits { bit_end - bit_start + 1U };
    const uint32_t start_byte { bit_start / kBitsPerByte };
    const uint32_t start_bit_offset { bit_start % kBitsPerByte };
    const uint32_t end_byte { bit_end / kBitsPerByte };
    const uint32_t total_bytes { end_byte - start_byte + 1U };
    
    if (total_bytes > _number_of_outputs) 
    {
        /* Exceeded output array size */
        return;
    }
    
    /* Create mask for bits within a single byte */
    const uint32_t mask { ((1UL << num_bits) - 1UL) << start_bit_offset };

    /* Extract relevant bits from the source array */
    uint32_t extracted { 0U };
    
    for (uint32_t i = start_byte; i <= end_byte; i++)
    {
        extracted |= static_cast<uint32_t>(src[i]) << (kBitsPerByte * (i - start_byte));
    }
    
    extracted = (extracted & mask) >> start_bit_offset;
    
    /* Clear target bits in the destination */
    for (uint32_t i = start_byte; i <= end_byte; i++)
    {
        _output_states[i] &= ~static_cast<uint8_t>(mask >> (kBitsPerByte * (i - start_byte)));
    }
    
    /* Insert the extracted bits into the destination array */
    for (uint32_t i = start_byte; i <= end_byte; i++)
    {
        _output_states[i] |= static_cast<uint8_t>((extracted << start_bit_offset) >> (kBitsPerByte * (i - start_byte)));
    }
}

/**
 * @brief Resets the output state array to zero.
 */
void ShiftRegister::resetDataArray() noexcept
{
    _arrayReset();
}

/**
 * @brief Shifts out a single byte of data to the STP LED Driver.
 * @param data The byte of data to shift out.
 */
void ShiftRegister::sendByte(const uint8_t data) noexcept
{
    _addToArray(data);
    _shiftOutData(data);
}

/**
 * @brief Shifts out an array of data to the STP LED Driver.
 * @param data Pointer to the data array.
 * @param size Size of the data array.
 */
void ShiftRegister::sendData(const uint8_t* data, const uint16_t size) noexcept
{
    if (isInitialized() == false || data == nullptr)
    {
        return;
    }
    
    const uint16_t count { (size < _number_of_outputs) ? size : _number_of_outputs };
    for (uint16_t i = 0U; i < count; i++)
    {
        _addToArray(data[i]);
        _shiftOutData(data[i]);
    }
}

/**
 * @brief Sets the output data for a specific panel.
 * @param au16_output The panel index.
 * @param au8_data The data to set for the panel.
 */
void ShiftRegister::setOutput(const uint16_t output, const uint8_t data) noexcept
{
    if (output >= _number_of_outputs || !isInitialized())
    {
        return;
    }
    _output_states[output] = data;
}

/**
 * @brief Performs the latch operation to update outputs.
 */
void ShiftRegister::doLatch() noexcept
{
    disableOutputs();
    setGpio(_latch_pin, _kLatchHighLevel);
    delayUs(_latch_hold_time_us);
    setGpio(_latch_pin, _kLatchLowLevel);
    delayUs(_reset_pin_hold_time_us);
    enableOutputs();
}

/**
 * @brief Enables the outputs by setting the PWM duty cycles.
 */
void ShiftRegister::enableOutputs() noexcept
{
    setGpio(_en_pin, GpioLevel::kHigh);
}

/**
 * @brief Disables the outputs by setting the PWM duty cycles to disabled.
 */
void ShiftRegister::disableOutputs() noexcept
{
    setGpio(_en_pin, GpioLevel::kLow);
}

/**
 * @brief Updates the outputs by flipping the internal data array to the outputs and latching.
 */
void ShiftRegister::updateOutputs() noexcept
{
    _flipArrayToOutputs();
    doLatch();
}

/**
 * @brief Sets the GPIO pin to the specified level.
 * @param au8_pin The GPIO pin identifier.
 * @param ae_gpioLevel The desired GPIO level.
 */
void ShiftRegister::setGpio(const uint8_t gpio_id, const GpioLevel gpio_state) noexcept
{
    Sr_cbk_SetGpio(gpio_id, static_cast<Sr_GpioLevel_t>(gpio_state));
}

/**
 * @brief Delays execution for a specified number of microseconds.
 * @param au64_timeUs The delay time in microseconds.
 */
void ShiftRegister::delayUs(uint64_t time_us) const noexcept
{
    
    Sr_cbk_DelayUs(time_us);
}

/**
 * @brief Sets the initial state of GPIO pins.
 * @note This function assumes that the GPIO pins are already configured as outputs.
 */
void ShiftRegister::_arrayReset() noexcept
{
    if (!isInitialized())
    {
        /* array is not initialized */
        return;
    }
    std::fill(_output_states.begin(), _output_states.end(), 0U);
}

/**
 * @brief Shifts out a byte of data using specified data and clock pins.
 * @param data_pin The GPIO pin used for data.
 * @param clock_pin The GPIO pin used for the clock.
 * @param bit_order The order of bits to shift out (LSB first or MSB first).
 * @param val The byte value to shift out.
 */
void ShiftRegister::_shiftOut(const uint8_t data_pin, const uint8_t clock_pin, const BitOrder bit_order, const uint8_t val) noexcept
{
    if (!isInitialized())
    {
        return;
    }
    // Shift out 8 bits - each bit at a time
    for (uint8_t i = 0U; i < 8; i++)
    {
        GpioLevel bit_value;
        
        if (bit_order == BitOrder::kLsbfirst)
        {
            bit_value = ((val & (1U << i)) != 0U) ? GpioLevel::kHigh : GpioLevel::kLow;
        }
        else
        {
            bit_value = ((val & (1U << (7U - i))) != 0U) ? GpioLevel::kHigh : GpioLevel::kLow;
        }
        
        /* Set data pin */
        setGpio(data_pin, bit_value);
        delayUs(_data_pin_hold_time_us);
        /* Clock has to come when data pin is stable */
        /* Pulse clock pin */
        setGpio(clock_pin, GpioLevel::kHigh);
        delayUs(_clock_pin_hold_time_us);
        /* Data bit is record when clock changes from L to H */
        /* Clear clock pin */
        setGpio(clock_pin, GpioLevel::kLow);
        //delayUs(_data_pin_hold_time_us);
        /* Clear data pin */
        //setGpio(data_pin, GpioLevel::kLow);
    }
}

/**
 * @brief Shifts out a byte of data using the configured data and clock pins.
 * @param data The byte of data to shift out.
 */
void ShiftRegister::_shiftOutData(const uint8_t data) noexcept
{
    _shiftOut(_data_pin, _clock_pin, BitOrder::kMsbfirst, data);
}

/**
 * @brief Adds a byte of data to the internal data array.
 * @param data The byte of data to add.
 */
void ShiftRegister::_addToArray(const uint8_t data) noexcept
{
    if (!isInitialized())
    {
        /* array is not initialized */
        return;
    }
    // 1. Shift everything to the right by 1 position.
    // This effectively "drops" the last element and frees up index 0.
    // Syntax: (source_start, source_end, destination_end)
    std::copy_backward(_output_states.begin(), _output_states.end() - 1, _output_states.end());

    // 2. Insert the new data at the now-"empty" first position.
    _output_states[0] = data;
}

/**
 * @brief Flips the internal data array to the outputs by shifting out the data.
 */
void ShiftRegister::_flipArrayToOutputs() noexcept
{
    if (!isInitialized())
    {
        return;
    }
    
    // go thru whole array but limit it on number of outputs, start from end
    for (uint16_t i = 0U; i < _number_of_outputs; i++)
    {
        _shiftOutData(_output_states[_number_of_outputs - 1U - i]);
    }
}

} // namespace m_shiftregister

/* ---------------- C bridge ---------------- */
struct Sr_Handler { m_shiftregister::ShiftRegister* instance; };

/** C facade */
extern "C" {

/**
 * @brief Creates and initializes an instance of the STP LED Driver.
 * @param as_config Pointer to the configuration structure.
 * @return Pointer to the created handler, or nullptr on failure.
 */
Sr_Handle_t * Sr_Create(const Sr_Config_t * as_config)
{
    if (as_config == nullptr)
    {
        return nullptr;
    }
    
    Sr_Handle_t * handler = new Sr_Handle_t;
    if (handler == nullptr)
    {
        return nullptr;
    }
    
    handler->instance = new m_shiftregister::ShiftRegister(
        as_config->dataPin,
        as_config->clockPin,
        as_config->latchPin,
        as_config->maxOutputs,
        as_config->latchHoldTimeUs,
        as_config->resetPinHoldTimeUs,
        as_config->shiftOutHoldTimeUs
    );
    
    if (handler->instance == nullptr || !handler->instance->isInitialized())
    {
        delete handler->instance;
        delete handler;
        return nullptr;
    }
    
    return handler;
}

/**
 * @brief Initializes the STP LED Driver instance.
 * @param as_handler Pointer to the handler.
 * @return true if initialization was successful, false otherwise.
 */
bool Sr_Init(const Sr_Handle_t * as_handler)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return false;
    }
    return as_handler->instance->init();
}

/**
 * @brief Sets the number of outputs.
 * @param as_handler Pointer to the handler.
 * @param au16_numberOfOutputs The desired number of outputs.
 * @return true if the number of outputs was set successfully, false otherwise.
 */
bool Sr_SetNumberOfOutputs(const Sr_Handle_t * as_handler, const uint16_t au16_numberOfOutputs)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return false;
    }
    return as_handler->instance->setNumberOfOutputs(au16_numberOfOutputs);
}

/**
 * @brief Sets the data array for the STP LED Driver.
 * @param as_handler Pointer to the handler.
 * @param au8_dataArr Pointer to the data array.
 * @param au8_size Size of the data array.
 */
void Sr_SetData(const Sr_Handle_t * as_handler, const uint8_t * au8_dataArr, const uint8_t au8_size)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return;
    }
    as_handler->instance->setData(au8_dataArr, au8_size);
}

/**
 * @brief Sets a range of bits in the data array.
 * @param as_handler Pointer to the handler.
 * @param au32_startBit The starting bit index.
 * @param au32_bitLength The length of the bit range.
 * @param au32_data The data to set in the specified bit range.
 */
void Sr_SetBitRangeInDataArray(const Sr_Handle_t * as_handler, const uint32_t au32_startBit, const uint32_t au32_bitLength, const uint32_t au32_data)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return;
    }
    as_handler->instance->setBitRangeInDataArray(au32_startBit, au32_bitLength, au32_data);
}

/**
 * @brief Copies a range of bits from a source array to the driver's data array.
 * @param as_handler Pointer to the handler.
 * @param au8_src Pointer to the source data array.
 * @param au32_bitStart The starting bit index in the source array.
 * @param au32_bitEnd The ending bit index in the source array.
 */
void Sr_CopyBitRange(const Sr_Handle_t * as_handler, const uint8_t * const au8_src, const uint32_t au32_bitStart, const uint32_t au32_bitEnd)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return;
    }
    as_handler->instance->copyBitRange(au8_src, au32_bitStart, au32_bitEnd);
}

/**
 * @brief Sends a single byte of data to the STP LED Driver.
 * @param as_handler Pointer to the handler.
 * @param au8_data The byte of data to send.
 */
void Sr_SendByte(const Sr_Handle_t * as_handler, const uint8_t au8_data)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return;
    }
    as_handler->instance->sendByte(au8_data);
}

/**
 * @brief Resets the data array of the STP LED Driver.
 * @param as_handler Pointer to the handler.
 */
void Sr_ResetDataArray(const Sr_Handle_t * as_handler)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return;
    }
    as_handler->instance->resetDataArray();
}

/**
 * @brief Sends a data array to the STP LED Driver.
 * @param as_handler Pointer to the handler.
 * @param au8_data Pointer to the data array.
 * @param au8_size Size of the data array.
 */
void Sr_SendData(const Sr_Handle_t * as_handler, const uint8_t * au8_data, const uint8_t au8_size)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return;
    }
    as_handler->instance->sendData(au8_data, au8_size);
}

/**
 * @brief Sets the output data for a specific panel.
 * @param as_handler Pointer to the handler.
 * @param au16_output The panel index.
 * @param au8_data The data to set for the panel.
 */
void Sr_SetOutput(const Sr_Handle_t * as_handler, const uint16_t au16_output, const uint8_t au8_data)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return;
    }
    as_handler->instance->setOutput(au16_output, au8_data);
}

/**
 * @brief Performs the latch operation to update outputs.
 * @param as_handler Pointer to the handler.
 */
void Sr_DoLatch(const Sr_Handle_t * as_handler)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return;
    }
    as_handler->instance->doLatch();
}

/**
 * @brief Enables the outputs.
 * @param as_handler Pointer to the handler.
 */
void Sr_EnableOutputs(const Sr_Handle_t * as_handler)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return;
    }
    as_handler->instance->enableOutputs();
}

/**
 * @brief Disables the outputs.
 * @param as_handler Pointer to the handler.
 */
void Sr_DisableOutputs(const Sr_Handle_t * as_handler)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return;
    }
    as_handler->instance->disableOutputs();
}

/**
 * @brief Updates the outputs by sending the current data and latching it.
 * @param as_handler Pointer to the handler.
 */
void Sr_UpdateOutputs(const Sr_Handle_t * as_handler)
{
    if (as_handler == nullptr || as_handler->instance == nullptr)
    {
        return;
    }
    as_handler->instance->updateOutputs();
}

} // extern "C"