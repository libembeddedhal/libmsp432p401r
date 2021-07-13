#pragma once

#include <libarmcortex/peripherals/interrupt.hpp>
#include <libarmcortex/peripherals/system_timer.hpp>
#include <libcore/platform/syscall.hpp>
#include <libcore/utility/time/time.hpp>
#include <libmsp432p401r/peripherals/system_controller.hpp>
#include <libmsp432p401r/platform/msp432p401r.hpp>

namespace sjsu::msp432p401r
{
void InitializePlatform()
{
  // Default initialized clock configuration object for use in the system
  // controller.
  static sjsu::msp432p401r::SystemController::ClockConfiguration
      clock_configuration;

  // Create stm32f10x system controller to be used by low level initialization.
  static sjsu::msp432p401r::SystemController system_controller(
      clock_configuration);

  // System timer is used to count milliseconds of time and to run the RTOS
  // scheduler.
  static sjsu::cortex::SystemTimer system_timer(
      sjsu::msp432p401r::SystemController::Modules::kMasterClock);

  // Cortex NVIC interrupt controller used to setup interrupt service routines
  static sjsu::cortex::InterruptController<sjsu::msp432p401r::kNumberOfIrqs,
                                           __NVIC_PRIO_BITS>
      interrupt_controller;

  sjsu::AddSysCallSymbols();

  // Set the platform's interrupt controller.
  // This will be used by other libraries to enable and disable interrupts.
  sjsu::InterruptController::SetPlatformController(&interrupt_controller);
  sjsu::SystemController::SetPlatformController(&system_controller);

  system_controller.Initialize();
  interrupt_controller.Initialize();
  system_timer.Initialize();

  sjsu::SetUptimeFunction(sjsu::cortex::SystemTimer::GetCount);
}
}  // namespace sjsu::msp432p401r
