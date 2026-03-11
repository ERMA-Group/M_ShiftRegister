/**
 * @file shift_register.hpp
 * @brief C++ class definition for Shift Register module.
 *
 * This header defines the internal C++ class used by the module.
 * It is not exposed to C projects directly — only via the C facade.
 */

#pragma once
#include <cstdint>
#include <cstring>
#include <memory>
#include <array>

// Include C header for C facade
#include "shift_register.h"

namespace m_shiftregister {

enum class GpioLevel : uint8_t { kLow = 0, kHigh = 1 };

enum class BitOrder  : uint8_t { kLsbfirst = 0, kMsbfirst = 1 };

/**
 * @class ShiftRegister
 * @brief C++ implementation of the Shift Register.
 * 
 * Manages shift register-based LED drivers with PWM control.
 * Provides methods for data manipulation, output control, and PWM management.
 */
class ShiftRegister {
public:
    static constexpr uint8_t kMaxPwmChannels { 8U };
    static constexpr uint16_t kMaxOutputs { 256U };

    /** @brief PWM Channel structure */
    struct PwmChannel
    {
        uint8_t channel;
        uint8_t duty_cycle;
        uint16_t frequency;
    };

    const uint8_t  _kPwmDutyEnDisabled{0U};
    const uint16_t  _kPwmDefaultFrequency{25000U};

    /** @brief Constructor */
    ShiftRegister() noexcept;

    ShiftRegister(
        uint8_t dataPin, 
        uint8_t clockPin, 
        uint8_t latchPin, 
        uint16_t number_of_outputs,
        uint8_t number_of_pwm_channels,
        uint16_t latch_hold_time_us = 1,
        uint16_t reset_pin_hold_time_us = 1,
        uint16_t data_pin_hold_time_us = 1,
        uint16_t clock_pin_hold_time_us = 1,
        uint16_t wait_for_pwm_to_disable = 1,
        bool initGpio = false,
        bool pwm_output_inverted = true
    ) noexcept;
    
    /** @brief Destructor */
    ~ShiftRegister() noexcept;
    
    /** @brief Disable copy constructor */
    ShiftRegister(const ShiftRegister&) = delete;
    
    /** @brief Disable copy assignment */
    ShiftRegister& operator=(const ShiftRegister&) = delete;
    
    /** @brief Enable move constructor */
    ShiftRegister(ShiftRegister&&) noexcept = default;
    
    /** @brief Enable move assignment */
    ShiftRegister& operator=(ShiftRegister&&) noexcept = default;

    // Initialization and configuration
    bool init() noexcept;
    bool setNumberOfPwmChannels(const uint8_t number_of_pwm_channels) noexcept;
    bool setNumberOfOutputs(const uint16_t number_of_outputs) noexcept;
    
    // Data manipulation
    void setData(const uint8_t* dataArr, const uint16_t size) noexcept;
    void setBitRangeInDataArray(const uint32_t start_bit, const uint32_t bit_length, const uint32_t data) noexcept;
    void copyBitRange(const uint8_t* src, const uint32_t bit_start, const uint32_t bit_end) noexcept;
    void resetDataArray() noexcept;
    
    // Data transmission
    void sendByte(uint8_t data) noexcept;
    void sendData(const uint8_t* data, const uint16_t size) noexcept;
    
    // Output control
    void setOutput(const uint16_t output, const uint8_t data) noexcept;
    void setPeriod(const uint16_t output, const bool period) noexcept;
    void set8SegOutput(const uint16_t output, const uint8_t data) noexcept;
    void set16SegOutput(const uint16_t output, const uint16_t data) noexcept;
    
    // Hardware control
    void doLatch() noexcept;
    void enableOutputs() noexcept;
    void disableOutputs() noexcept;
    void updateOutputs() noexcept;
    
    // PWM control
    void setPwmDuty(const uint8_t channel, const uint8_t duty) noexcept;
    void setBrightness(const uint8_t channel, const uint8_t brightness) noexcept;
    uint8_t getBrightness(const uint8_t channel) const noexcept;
    uint8_t getPwmDuty(const uint8_t channel) const noexcept;
    
    // Utility functions
    static uint8_t convertDutyToSldCompatible(const uint8_t duty) noexcept;
    
    // Status query
    bool isInitialized() const noexcept { return _initialized; }
    uint16_t getNumberOfOutputs() const noexcept { return _number_of_outputs; }
    uint8_t getNumberOfPwmChannels() const noexcept { return _number_of_pwm_channels; }
    PwmChannel getPwmChannel(const uint8_t channel) const noexcept;

    // Brigdges to C callback functions
    void setGpio(uint8_t pin, GpioLevel level) noexcept;
    void changePwmOutput(uint8_t channel, uint16_t duty_cycle, uint16_t frequency) noexcept;
    void delayUs(uint64_t time_us) const noexcept;

    void setWaitTimes(
        uint16_t latch_hold_time_us, 
        uint16_t reset_pin_hold_time_us, 
        uint16_t data_pin_hold_time_us, 
        uint16_t clock_pin_hold_time_us, 
        uint16_t wait_for_pwm_to_disable) noexcept
    {
        _latch_hold_time_us = latch_hold_time_us;
        _reset_pin_hold_time_us = reset_pin_hold_time_us;
        _data_pin_hold_time_us = data_pin_hold_time_us;
        _clock_pin_hold_time_us = clock_pin_hold_time_us;
        _wait_for_pwm_to_disable = wait_for_pwm_to_disable;
    }

//private:
    // Constants
    
    // Member variables
    bool _initialized{false};
    bool _pwm_enabled{false};
    
    bool _pwm_output_inverted{true}; // Flag to indicate if PWM output is inverted (100% duty = LEDs off)
    uint8_t _data_pin{0};
    uint8_t _clock_pin{0};
    uint8_t _latch_pin{0};
    uint16_t _number_of_outputs{0U};
    uint8_t _number_of_pwm_channels{0U};

    uint16_t _wait_for_pwm_to_disable{1};
    uint16_t _latch_hold_time_us{1};
    uint16_t _reset_pin_hold_time_us{1};
    uint16_t _data_pin_hold_time_us{1};
    uint16_t _clock_pin_hold_time_us{1};

    const GpioLevel _kLatchHighLevel{GpioLevel::kHigh};
    const GpioLevel _kLatchLowLevel{GpioLevel::kLow};
    constexpr static uint8_t kBitsPerByte{8U};
    constexpr static uint8_t kPwmDutyMaximum{100U};
    
    std::array<uint8_t, kMaxOutputs> _output_states;
    std::array<PwmChannel, kMaxPwmChannels> _pwm_channels;
    bool _initGpio{false}; // Flag to indicate if GPIOs should be initialized on SLD init
    
    // Private helper methods
    void _setGpiosInitState(void) noexcept;
    void _arrayReset() noexcept;
    void _shiftOut(uint8_t data_pin, uint8_t clock_pin, BitOrder bit_order, uint8_t val) noexcept;
    void _shiftOutData(uint8_t data) noexcept;
    void _addToArray(uint8_t data) noexcept;
    void _flipArrayToOutputs() noexcept;
};

} // namespace m_shiftregister