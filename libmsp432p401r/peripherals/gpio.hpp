#pragma once

#include <cstdint>
#include <libcore/peripherals/gpio.hpp>
#include <libcore/peripherals/interrupt.hpp>
#include <libcore/utility/error_handling.hpp>
#include <libcore/utility/math/bit.hpp>
#include <libmsp432p401r/platform/msp432p401r.hpp>

namespace sjsu::msp432p401r
{
/// An abstract interface for General Purpose I/O
class Gpio : public sjsu::Gpio
{
 public:
  /// Array containing the port structures that map P1 to P10. Each port
  /// structure contains an odd and even port. For example, PA contains P1 and
  /// P2, PB contains P3 and P4, and so on. The only exception is Port J which
  /// is its own port and only has pins 1-5.
  inline static std::array<DIO_PORT_Interruptable_Type *, 6> ports = {
    PA,  // P1 and P2
    PB,  // P3 and P4
    PC,  // P5 and P6
    PD,  // P7 and P8
    PE,  // P9 and P10
    reinterpret_cast<DIO_PORT_Interruptable_Type *>(PJ)
  };

  /// @param port The port number. The capitol letter 'J' should be used is the
  ///             desired port is port J.
  /// @param pin  The pin number.
  constexpr Gpio(uint8_t port, uint8_t pin)
      : port_number_(port), pin_number_(pin)
  {
  }

  void ModuleInitialize() override
  {
    ConfigureFunction();
    ConfigurePullResistor();
    ConfigureOpenDrain();
  }

  void SetDirection(sjsu::Gpio::Direction direction) override
  {
    constexpr auto kDirectionBit = bit::MaskFromRange(2);

    settings.function =
        bit::Value(0).Insert(Value(direction), kDirectionBit).To<uint8_t>();
    ModuleInitialize();
  }

  void Set(State output) override
  {
    volatile uint8_t * out_register = RegisterAddress(&Port()->OUT);
    if (output == State::kHigh)
    {
      *out_register = bit::Set(*out_register, pin_number_);
    }
    else
    {
      *out_register = bit::Clear(*out_register, pin_number_);
    }
  }

  void Toggle() override
  {
    volatile uint8_t * out_register = RegisterAddress(&Port()->OUT);
    *out_register                   = bit::Toggle(*out_register, pin_number_);
  }

  bool Read() override
  {
    volatile uint8_t * in_register = RegisterAddress(&Port()->IN);
    return bit::Read(*in_register, pin_number_);
  }

  void AttachInterrupt(InterruptCallback, Edge) override
  {
    throw sjsu::Exception(std::errc::operation_not_supported, "");
  }

  void DetachInterrupt() override
  {
    throw sjsu::Exception(std::errc::operation_not_supported, "");
  }

 private:
  void ConfigureOpenDrain()
  {
    if (settings.open_drain)
    {
      throw sjsu::Exception(std::errc::operation_not_supported,
                            "MSP does not support open drain pins");
    }
  }

  /// Configures the pin's function mode based on the specified 3-bit function
  /// code. Where the most significant bit determines the pin direction.
  ///
  /// @see Port Function Tables in 6.12 Input/Output Diagrams
  ///      http://www.ti.com/lit/ds/symlink/msp432p401r.pdf#page=138
  void ConfigureFunction()
  {
    if (settings.function > 0b111)
    {
      throw sjsu::Exception(
          std::errc::invalid_argument,
          "The function code must be a 3-bit value between 0b000 and 0b111.");
    }

    constexpr auto kDirectionBit = bit::MaskFromRange(2);
    constexpr auto kSel1Bit      = bit::MaskFromRange(1);
    constexpr auto kSel0Bit      = bit::MaskFromRange(0);

    volatile uint8_t * select1_register = RegisterAddress(&Port()->SEL1);
    volatile uint8_t * select0_register = RegisterAddress(&Port()->SEL0);
    *select1_register                   = static_cast<uint8_t>(
        bit::Insert(static_cast<uint32_t>(*select1_register),
                    bit::Read(settings.function, kSel1Bit),
                    pin_number_));
    *select0_register = static_cast<uint8_t>(
        bit::Insert(static_cast<uint32_t>(*select0_register),
                    bit::Read(settings.function, kSel0Bit),
                    pin_number_));

    SetPinDirection(
        sjsu::Gpio::Direction(bit::Read(settings.function, kDirectionBit)));
  }

  /// Sets the pin's resistor pull either as pull up, pull down, or none.
  ///
  /// @see 12.2.4 Pullup or Pulldown Resistor Enable Registers (PxREN)
  ///      https://www.ti.com/lit/ug/slau356i/slau356i.pdf#page=678
  void ConfigurePullResistor()
  {
    volatile uint8_t * resistor_enable = RegisterAddress(&Port()->REN);
    // The output register (OUT) is used to select the pull down or pull up
    // resistor when resistor is enabled.
    volatile uint8_t * resistor_select = RegisterAddress(&Port()->OUT);

    switch (settings.resistor)
    {
      case PinSettings_t::Resistor::kNone:
        *resistor_enable = bit::Clear(*resistor_enable, pin_number_);
        break;
      case PinSettings_t::Resistor::kPullDown:
        *resistor_enable = bit::Set(*resistor_enable, pin_number_);
        *resistor_select = bit::Clear(*resistor_select, pin_number_);
        break;
      case PinSettings_t::Resistor::kPullUp:
        *resistor_enable = bit::Set(*resistor_enable, pin_number_);
        *resistor_select = bit::Set(*resistor_select, pin_number_);
        break;
    }
  }

  /// Sets the pin direction as output or input.
  ///
  /// @note The pin direction can determine the function of the pin when the
  ///       pin's function is not in general purpose I/O mode.
  ///
  /// @param direction The direction to set.
  void SetPinDirection(sjsu::Gpio::Direction direction) const
  {
    volatile uint8_t * direction_register = RegisterAddress(&Port()->DIR);
    if (direction == sjsu::Gpio::Direction::kOutput)
    {
      *direction_register = bit::Set(*direction_register, pin_number_);
    }
    else
    {
      *direction_register = bit::Clear(*direction_register, pin_number_);
    }
  }

  /// Helper function to calculate the register address depending on whether the
  /// port number is odd or even. For example, PA->DIR consist of PA->DIR_L
  /// (used for P1) and PA->DIR_H (used for P2).
  ///
  /// @param address The address of the 16-bit register that holds the odd and
  ///                even 8-bit registers.
  /// @returns The address of a desired 8-bit register.
  volatile uint8_t * RegisterAddress(volatile uint16_t * address) const
  {
    // If the port number is odd or is 'J', use the low register. Otherwise. if
    // the port number is even, use the high register.
    uint32_t offset = 1;
    if (static_cast<bool>(port_number_ & 0b1) || (port_number_ == 'J'))
    {
      offset = 0;
    }
    return reinterpret_cast<volatile uint8_t *>(address) + offset;
  }

  /// @returns The reference to the port structure base on the port number.
  DIO_PORT_Interruptable_Type * Port() const
  {
    if (port_number_ == 'J')
    {
      return ports[5];
    }
    else if (!static_cast<bool>(port_number_ & 0b1))  // Even port number.
    {
      return ports[(port_number_ - 1) / 2];
    }
    // Odd port number
    return ports[(port_number_ / 2)];
  }

  uint8_t port_number_;
  uint8_t pin_number_;
};

template <int port, int pin_number>
inline Gpio & GetGpio()
{
  // NOTE: port can only be 1-10 or 'J'
  static_assert((1 <= port && port <= 10) || (port == 'J'),
                "Port must be 1-10 or J");
  static_assert(pin_number <= 7, "Pin must be between 0 and 7");

  static Gpio gpio(port, pin_number);
  return gpio;
}
}  // namespace sjsu::msp432p401r
