/* vim:ts=4
 *
 * Copyleft 2017  Michał Gawron
 * Marduk Unix Labs, http://mulabs.org/
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Visit http://www.gnu.org/licenses/gpl-3.0.html for more information on licensing.
 */

// TODO sizeof...(args) + if constexpr() zamiast rekurencyjności

// AVR:
#include <math.h>
#include <stdio.h>

// Mulabs:
#include <mulabs_avr/mcu/atxmega128-a1u.h>
#include <mulabs_avr/avr/interrupts_lock.h>
#include <mulabs_avr/support/protocols/usb/setup_packet.h> // XXX
#include <mulabs_avr/support/protocols/usb/setup_conductor.h>
#include <mulabs_avr/utility/atomic.h>
#include <mulabs_avr/std/initializer_list.h>//XXX
#include <avr/interrupt.h>

// Local:
#include "exposure_request.h"
#include "pid_control.h"

// TEST XXX
#include <mulabs_avr/support/protocols/usb/device_definition.h>


using namespace mulabs::avr;

class LDI: public ATXMega128A1U
{
  private:
	// TODO yellow light on, when not exposing.
	// TODO separate green LED indicating synchronization is acquired.

	static constexpr char		kSignatureLine[]				= "Laser Digital Imaging <https://github.com/mcvsama/ldi> 2017";
	static constexpr char		kAuthorLine[]					= "Michal Gawron <mcvsama@gmail.com>";

	static constexpr bool		kDebugPortEnabled				= true;
	static constexpr uint32_t	kDebugPortBaudRate				= 115200;
	static constexpr uint8_t	kPhotodiodeCaptureEventBus		= 0;

	static constexpr uint32_t	kPreconceivedSystemClockHz		= 32000000;
	static constexpr float		kActualSystemFrequencyFactor	= 1.0;
	static constexpr uint32_t	kSystemClockHz					= kPreconceivedSystemClockHz * kActualSystemFrequencyFactor;

	static constexpr uint32_t	kLaserClockDenominator			= 6;//TODO fix for other values, like 5 or 4 TODO should be 3
	static constexpr uint32_t	kLaserClockHz					= kSystemClockHz / kLaserClockDenominator;
	static constexpr uint32_t	kScanFrequencyHz				= 1000;
	static constexpr uint32_t	kPointsInScanline				= kLaserClockHz / kScanFrequencyHz;
	static constexpr float		kProperScanlinePercentage		= 0.55; // TODO is it correct?
	static constexpr float		kProperScanlineWidthInches		= 8.0;
	static constexpr uint32_t	kPointsInProperScanline			= kProperScanlinePercentage * kPointsInScanline;
	static constexpr uint32_t	kDPI							= kPointsInProperScanline / kProperScanlineWidthInches;
	static constexpr uint16_t	kSingleSweepPulses				= kSystemClockHz / kScanFrequencyHz;
	static constexpr size_t		kScanlineBeginPulse				= 0.1 * kSingleSweepPulses;
	static constexpr size_t		kPrepareForPhotodiodeSignal		= 0.8 * kSingleSweepPulses;

	// These IDs are provided by Objective Development. The device discrimination is by its textual name.
	static constexpr uint16_t	kUSBVendorID					= 0x16c0;
	static constexpr uint16_t	kUSBProductID					= 0xdc05;

	static constexpr uint32_t	kMotorPWMPeriod					= 0xffff;
	static constexpr float		kMotorInitialDutyCycle			= 0.16; // Determined experimentally.
	static constexpr float		kMaxMotorFrequencyDeltaHz		= 0.25;
	// Lock condition gets true if for this many pulses the difference between kScanFrequencyHz and PD frequency is
	// within kMaxMotorFrequencyDeltaHz:
	static constexpr uint16_t	kRequiredLockedPulses			= 250;

	static constexpr uint32_t	kStepsPerRevolution				= 200;
	static constexpr uint32_t	kMillimetersPerRevolution		= 5;
	static constexpr uint32_t	kLaserCalibrationHeadPosition	= 0;
	static constexpr uint32_t	kLine0Position					= 2.5 * kStepsPerRevolution / kMillimetersPerRevolution;
	static constexpr uint32_t	kFastMoveStepDelayMs			= 3;
	static constexpr uint32_t	kExposingMoveStepDelayMs		= 15;
	static constexpr uint32_t	kSlowMoveStepDelayMs			= 30;
	static constexpr uint16_t	kTopLinePosition				= 300 * kStepsPerRevolution / kMillimetersPerRevolution;

	// TODO check if all are used
	enum class BuzzerSignal
	{
		LaserSyncAcquired,
		LaserSyncLost,
		ExposingFinished,
		MovementWarning,
	};

	enum class HeadDirection: bool
	{
		Up		= true,
		Down	= false,
	};

	enum class HeadSpeed
	{
		Slow,
		ExposingSpeed,
		Fast,
	};

  public:
	enum class LaserPower: uint8_t
	{
		Power12Pct,
		Power25Pct,
		Power50Pct,
		Power100Pct,
	};

  public:
	static void
	initialize();

	static void
	loop();

  private:
	static void
	configure_mcu();

	static void
	setup_clocks();

	static void
	configure_ports();

	static void
	setup_test_values();

	/**
	 * N fast beeps.
	 */
	static void
	fast_beep (uint8_t num = 1);

	/**
	 * N fast beeps.
	 */
	static void
	slow_beep (uint8_t num = 1);

	/**
	 * Execute given signal.
	 */
	static void
	signal (BuzzerSignal);

	static void
	setup_uart();

	static void
	setup_motor_pwm();

	static void
	set_motor_pulse_width (uint16_t pulse_width);

	static void
	setup_motor_reference_clock();

	static void
	setup_motor_frequency_meter();

	static void
	setup_laser_clock();

	static void
	setup_scanline_timer();

	static void
	setup_usb();

	/**
	 * Offset the default scaline begin pulse by given factor.
	 * 1.0 is the normal value.
	 */
	static void
	set_scanline_begin_offset_factor (float);

	/**
	 * Sets laser word for laser-module registers.
	 * The laser will expose bits starting with least significant bit.
	 */
	static void
	set_laser_word (uint8_t msb, uint8_t lsb);

	/**
	 * Equivalent to set_laser_word (0xff, 0xff)
	 * and pushing bits to the laser module registers.
	 */
	static void
	push_laser_enabled_word();

	/**
	 * Equivalent to set_laser_word (0, 0)
	 * and pushing bits to the laser module registers.
	 */
	static void
	push_laser_disabled_word();

	/**
	 * Push laser word to the laser-module registers.
	 * Normally the _laser_clock_timer controls the _laser_clk line, so make sure not to call this
	 * when _laser_clock_timer is running.
	 */
	static void
	push_laser_word();

	/**
	 * Start laser CLK timer.
	 * Takes control over the _laser_clk line until stop_laser_clock() called.
	 */
	static void
	start_laser_clock();

	/**
	 * Stop laser CLK timer and reset its counter to 0.
	 * Releases control over the _laser_clk line.
	 */
	static void
	stop_laser_clock();

	/**
	 * Set laser power.
	 */
	static void
	set (LaserPower);

	/**
	 * This needs interrupts enabled.
	 */
	static void
	start_motor();

	/**
	 * Stops PLL, motor and laser.
	 */
	static void
	stop_motor();

	/**
	 * Move head down until limit switch is tripped.
	 * Then move head to position 0.
	 */
	static void
	calibrate_head();

	/**
	 * Move head to given position.
	 * Position 0 is at the "bottom" and is one line away from the tripping of the limit switch.
	 * Position 0 is also reserved for calibration of the laser.
	 *
	 * Return true if movement was fully or false if limit was reached during operation.
	 */
	static bool
	move_head_to (uint32_t position, HeadSpeed);

	/**
	 * Perform n steps of movement of the head in given direction.
	 * If limit switch is being on, no operation is interrupted.
	 * The head doesn't have to be calibrated to perform the movement.
	 *
	 * Return true if movement was performed or false if limit was reached.
	 */
	static bool
	move_head (HeadDirection, uint32_t steps, HeadSpeed);

	/**
	 * Return true if the limit switch for given head movement direction
	 * is tripped.
	 */
	static bool
	is_limit_reached (HeadDirection);

	/**
	 * Send character to the UART port.
	 */
	static int
	uart_putchar (char c, FILE* stream);

	/**
	 * Send formatted string to the UART port.
	 */
	static void
	debug (char const* format, ...);

	/**
	 * Non-formatting (faster) debug printer.
	 */
	static void
	debug_raw (char const* string);

	/**
	 * Execute given code and send debug information about it to the debug port.
	 */
	template<class Function>
		static void
		task (char const* name, Function);

	/**
	 * Return a string describing the HeadDirection value.
	 */
	static char const*
	to_string (HeadDirection);

	/**
	 * Return a string describing the HeadSpeed value.
	 */
	static char const*
	to_string (HeadSpeed);

  public:
	/**
	 * Called when photodiode signal comes in.
	 */
	static void
	handle_photodiode_signal();

	/**
	 * Called when laser is at the point where scanline begins.
	 */
	static void
	handle_scanline_begin();

	/**
	 * Called when laser is about to hit photodiode so it needs to be enabled.
	 */
	static void
	handle_prepare_laser_for_photodiode_sync();

	/**
	 * Called on photodiode event through motor phase measure counter.
	 */
	static void
	handle_motor_frequency_capture (uint16_t actual_period);

	/**
	 * Called on USB bus events.
	 */
	static void
	handle_usb_event();

	/**
	 * Called when USB transaction completes.
	 */
	static void
	handle_usb_transaction_complete();

  private:
	static constexpr size_t kUSBMaxPacketSize = 16;

	// Structure of the USB device:
	// TODO provide means for USB to check if given usb::Device can be even supported by the hardware.
	static constexpr usb::Device _usb_device_definition {
		usb::USBVersion::_1_0,
		usb::VendorID (kUSBVendorID),
		usb::ProductID (kUSBProductID),
		usb::ReleaseID (0x00),
		usb::DeviceClass::VendorSpecified,
		usb::DeviceSubClass (0x00),
		usb::DeviceProtocol (0x00),
		usb::Manufacturer (u"mulabs.org"),
		usb::Product (u"laser direct imaging"),
		usb::Serial (u"00000000"),
		usb::MaxPacketSize0 (kUSBMaxPacketSize),
		{
			usb::Configuration {
				usb::Index (1),
				u"default config",
				usb::SelfPowered (true),
				usb::RemoteWakeup (true),
				usb::MaxPowerMilliAmps (100),
				{
					usb::Interface {
						usb::Index (0),
						usb::AlternateIndex (0),
						usb::DeviceClass::VendorSpecified,
						usb::DeviceSubClass (0x00),
						usb::DeviceProtocol (0x00),
						u"LDI",
						{
							usb::Endpoint {
								usb::Index (1),
								usb::Direction::Out,
								usb::TransferType::Interrupt,
								usb::SyncType::NoSync,
								usb::UsageType::Data,
								usb::MaxPacketSize (kUSBMaxPacketSize),
								usb::Interval (1)
							},
							usb::Endpoint {
								usb::Index (1),
								usb::Direction::In,
								usb::TransferType::Interrupt,
								usb::SyncType::NoSync,
								usb::UsageType::Data,
								usb::MaxPacketSize (kUSBMaxPacketSize),
								usb::Interval (1)
							}
						}
					}
				}
			}
		}
	};

	static constexpr usb::DeviceStrings<_usb_device_definition> _usb_device_strings { };

	/*
	 * External ports
	 */

	// Stepper motor control:
	static constexpr Pin					_head_direction				{ port_b.pin (7) };	// (out) connector STEPPER_MOTOR pin 1
	static constexpr Pin					_head_step					{ port_a.pin (1) };	// (out) connector STEPPER_MOTOR pin 2
	static constexpr Pin					_head_energize				{ port_b.pin (6) };	// (out) connector STEPPER_MOTOR pin 3
	// Limit switches:
	static constexpr Pin					_limits_top					{ port_a.pin (7) };	// (in)  connector LIMITS/ESTOP pin 1
	static constexpr Pin					_limits_bottom				{ port_a.pin (6) };	// (in)  connector LIMITS/ESTOP pin 2
	static constexpr Pin					_limits_estop				{ port_a.pin (5) };	// (in)  connector LIMITS/ESTOP pin 3
	// USB connector:
	static constexpr Pin					_usb_vbus					{ port_d.pin (5) };	// (in)  connector USB pin 1
	static constexpr Pin					_usb_dm						{ port_d.pin (6) };	// (?)   connector USB pin 2
	static constexpr Pin					_usb_dp						{ port_d.pin (7) };	// (?)   connector USB pin 3
	// UART:
	static constexpr Pin					_uart_rx					{ port_d.pin (2) }; // (in)
	static constexpr Pin					_uart_tx					{ port_d.pin (3) }; // (out)
	// Other pins:
	static constexpr Pin					_yellow_light				{ port_f.pin (7) };	// (out) connector YELLOW_LIGHT pin 2
	static constexpr Pin					_green_led					{ port_k.pin (7) }; // (out)
	static constexpr Pin					_buzzer						{ port_k.pin (6) }; // (out)

	/*
	 * Laser-driver connector
	 */

	// Bits are reversed (MSB is LSB and vice versa) in these two laser_data ports:
	static constexpr Port					_laser_data_rev_lsb			{ port_c };			// (out)
	static constexpr Port					_laser_data_rev_msb			{ port_e };			// (out)
	// Used to isolate latch/power/enable lines from laser module, now unused since the voltage-translator
	// is also used for other stuff:
	static constexpr Pin					_laser_translator_disable	{ port_b.pin (2) };	// (out)
	// Latch data in laser registers on rising edge:
	static constexpr Pin					_laser_latch				{ port_b.pin (3) };	// (out)
	// Power bits set to high inhibit power-output:
	static constexpr Pin					_laser_power_bit_0			{ port_b.pin (4) };	// (out)
	static constexpr Pin					_laser_power_bit_1			{ port_b.pin (5) };	// (out)
	// Power the laser (doesn't mean it's on):
	static constexpr Pin					_laser_enabled				{ port_d.pin (1) };	// (out)
	// Set high (in reality low) for resetting 4-bit counter to 0 in the laser module:
	static constexpr Pin					_laser_sync					{ port_b.pin (0) };	// (out, inverted)
	// Signalled when laser module is ready to take another data into registers:
	static constexpr Pin					_laser_prefetch				{ port_b.pin (1) };	// (in)
	static constexpr Pin					_laser_clk					{ port_d.pin (0) };	// (out)

	/*
	 * Communication ports
	 */

	static constexpr USART					_uart						{ usart_d0 };
	static constexpr USB					_usb						{ usb };

	/*
	 * Mirror motor control
	 */

	static constexpr Pin					_motor_photodiode			{ port_d.pin (4) }; // (in, edge sensing) Counter capture input pin.
	static constexpr Pin					_motor_pwm					{ port_f.pin (4) };	// (out, timer-controlled)
	static constexpr Pin					_motor_reference_clock		{ port_f.pin (0) };	// (out, timer-controlled)

	/*
	 * Timers
	 */

	// TCC0 controls timing of scanling-begin, scanline-end, etc events.
	static constexpr Timer01				_scanline_timer				{ timer_c0 };
	// TCD0 (pin output) generates laser clock.
	static constexpr Timer01				_laser_clock_timer			{ timer_d0 };
	// TCD1 measures phase between mirror motor reference clock and photodiode signals (software PLL).
	static constexpr Timer01				_motor_frequency_timer		{ timer_d1 };
	// TCF0 (pin output) generates mirror motor reference clock.
	static constexpr Timer01				_motor_reference_timer		{ timer_f0 };
	// TCF1 (pin output) generates PWM waveform for controlling the speed of laser mirror motor.
	static constexpr Timer01				_motor_pwm_timer			{ timer_f1 };

	/*
	 * USB stuff
	 */

	static inline USB::EndpointsTable<_usb_device_definition.maximum_endpoint_id() + 1>
											_usb_endpoints				{ _usb_device_definition };
	static inline USB::OutputEndpoint		_usb_control_out			{ _usb_endpoints.nth_output (0) };
	static inline USB::InputEndpoint		_usb_control_in				{ _usb_endpoints.nth_input (0) };
	static inline Array<uint8_t, kUSBMaxPacketSize>
											_usb_control_buffer_out;
	static inline Array<uint8_t, kUSBMaxPacketSize>
											_usb_control_buffer_in;
	static inline usb::SetupConductor		_usb_setup_conductor		{ _usb_device_definition, _usb_device_strings, _usb, _usb_control_in, _usb_control_buffer_in, _usb_control_out, _usb_control_buffer_out };

	/*
	 * Other
	 */

	// Size must be divisible by 2, since we send 16-bit words out to the laser module ("/ 2 * 2"):
	static inline Array<uint8_t, kPointsInProperScanline / 8 / 2 * 2>
											_scanline_buffer;
	static inline LaserPower				_exposing_laser_power		{ LaserPower::Power100Pct }; // TODO exposing power is useless, remove
	static inline ExposureRequest			_exposure_request;
	static inline uint16_t					_motor_pulse_width			{ static_cast<uint16_t> (kMotorInitialDutyCycle * LDI::kMotorPWMPeriod) };
	// TODO move Array to std as std::array
	static inline Array<uint16_t, 4>		_last_photodiode_periods;
	static inline Atomic<bool>				_motor_speed_locked			{ false };
	static inline Atomic<bool>				_motor_enabled				{ false };
	static inline PIDControl<float>			_motor_pid					{ { 1.0, 0.5, 0.1 }, kScanFrequencyHz };
	static inline bool						_head_calibrated			{ false };
	static inline uint32_t					_head_position				{ 0 };
	static inline FILE						_uart_stream;

	static_assert (_scanline_buffer.size() % 2 == 0, "The size of scanline buffer must be divisible by 2.");
};


void
LDI::initialize()
{
	configure_mcu();

	setup_uart();

	debug ("\n\n>>>>> %s\n", kSignatureLine);
	debug (">>>>> %s\n\n\n", kAuthorLine);

	task ("machine initialization", [&] {
		setup_motor_pwm();
		setup_motor_reference_clock();
		setup_motor_frequency_meter();
		setup_laser_clock();
		setup_scanline_timer();
		setup_usb();
	});

	_scanline_buffer.fill (0);
}


void
LDI::loop()
{
	sei();

#if 1
	bool prev_vbus = true;

	while (true)
	{
		// TODO FIXME doesn't work:
		if (prev_vbus && !_usb_vbus.get())
		{
			setup_usb();
			prev_vbus = _usb_vbus.get();
		}

		if (!prev_vbus && _usb_vbus.get())
			prev_vbus = _usb_vbus.get();
	}
#endif
	start_motor();
	move_head_to (kLine0Position, HeadSpeed::ExposingSpeed);
	setup_test_values();

	// TODO remember to clear scanline buffer after completing exposure request
	while (true)
	{
		_exposure_request.set_exposures_number (500);
		_exposure_request.start();

		_exposure_request.wait_for_finish();
		move_head (HeadDirection::Up, 1, HeadSpeed::Fast);

		// Check for sync errors:
		if (!_motor_speed_locked.load())
		{
			// TODO
			// Set the error flag, wait for host to read it and reset it.
			// Break the current command.
			// Host is to decide what to do (resynchronize laser and continue where left?)
		}
	}
}


void
LDI::configure_mcu()
{
	JTAG::disable();
	setup_clocks();
	configure_ports();
	// Low priority for UART and USB,
	// Medium for frequency capture of the motor,
	// High for proper scanline begin/end signals.
	InterruptSystem::enable (InterruptSystem::Level::Low, InterruptSystem::Level::Medium, InterruptSystem::Level::High);
}


void
LDI::setup_clocks()
{
	// Configure XOSC:
	Clock::set (Clock::XOSCFrequency::Range12To16MHz);
	Clock::set (Clock::XOSCType::XTAL16kCLK);
	Clock::set_high_power_16M_xosc (true);

	// Configure PLL (using 16 MHz quartz crystal for 32 MHz CPU clock):
	Clock::set (Clock::PLLSource::XOSC);
	Clock::set_pll_multiplication_factor<2>();

	// Enable and wait for clocks:
	Clock::enable (Clock::Oscillator::PLL, Clock::Oscillator::XOSC, Clock::Oscillator::RC32MHz);
	Clock::wait_for (Clock::Oscillator::PLL, Clock::Oscillator::XOSC, Clock::Oscillator::RC32MHz);

	// Set CPU clock:
	Clock::set_cpu_clock (Clock::Oscillator::PLL);
	Clock::lock_system_clock();

	// Configure internal RC32M clock to run at 48 MHz using DFLL:
	Clock::set_dfll_coarse_calibration_value (Clock::DFLL::RC32MHz, MCU::read (MCU::SignatureRegister::RCOSC48M));
	Clock::set_dfll_multiplication_factor (Clock::DFLL::RC32MHz, 48'000'000 / 1024);
	Clock::set (Clock::DFLLCalibrationReference32M::USBSOF);
	Clock::set_dfll_enabled (Clock::DFLL::RC32MHz, true);

	// Set USB clock to RC32M (running now at 48 MHz):
	Clock::set (Clock::USBClockSource::RC32MHz);
	Clock::set (Clock::USBClockPrescaler::Div1);
	Clock::set_usb_enabled (true);
}


void
LDI::configure_ports()
{
	_head_direction.configure_as_output();
	_head_step.configure_as_output();
	_head_energize.configure_as_output();
	_head_energize.set_inverted_io (true);

	_limits_top.configure_as_input();
	_limits_top.set (Pin::Configuration::PullUp);
	_limits_bottom.configure_as_input();
	_limits_bottom.set (Pin::Configuration::PullUp);
	_limits_estop.configure_as_input();

	_usb_vbus.configure_as_input(); // TODO on falling edge setup_usb()
	_usb_dm.configure_as_input();
	_usb_dp.configure_as_input();

	_yellow_light.configure_as_output();
	_green_led.configure_as_output();
	_buzzer.configure_as_output();
	_buzzer.set_inverted_io (true);

	_laser_data_rev_lsb.configure_as_outputs (0xff);
	_laser_data_rev_msb.configure_as_outputs (0xff);
	_laser_translator_disable.configure_as_output();
	_laser_latch.configure_as_output();
	_laser_power_bit_0.configure_as_output();
	_laser_power_bit_0.set_inverted_io (true);
	_laser_power_bit_1.configure_as_output();
	_laser_power_bit_1.set_inverted_io (true);
	_laser_enabled.configure_as_output();
	_laser_sync.configure_as_output();
	_laser_sync.set_inverted_io (true);
	_laser_prefetch.configure_as_input();
	_laser_clk.configure_as_output();

	_motor_photodiode.configure_as_input();
	_motor_photodiode.set (Pin::Configuration::PullDown);
	_motor_photodiode.set (Pin::SenseConfiguration::RisingEdge);
	// Configure the pin D4 (_motor_photodiode) as event source for event channel 0:
	EventSystem::set_event_source_for_bus<kPhotodiodeCaptureEventBus> (EventSystem::EventSource::PortDPin4);
	// Configure interrupt 0:
	_motor_photodiode.port().set_interrupt<0> (InterruptSystem::Level::High);
	_motor_photodiode.set_selected_for_interrupt<0> (true);

	_motor_pwm.configure_as_output();
	_motor_reference_clock.configure_as_output();

	// Set initial values:
	_head_energize = false;

	_laser_translator_disable = false;
	_laser_latch = false;
	_laser_sync = false;

	_laser_data_rev_lsb = 0x00;
	_laser_data_rev_msb = 0x00;
	_laser_power_bit_0 = false;
	_laser_power_bit_1 = false;
	_laser_enabled = false;
	_laser_latch.signal();
	_laser_sync.signal();

	_yellow_light = false;
	_buzzer = false;
	_green_led = false;
}


void
LDI::setup_test_values()
{
	for (size_t k = 0; k < _scanline_buffer.size(); k += 2)
	{
		_scanline_buffer[k + 0] = 0x0f;
		_scanline_buffer[k + 1] = 0x00;
	}
}


void
LDI::fast_beep (uint8_t num)
{
	for (uint8_t i = 0; i < num; ++i)
	{
		_buzzer = true;
		sleep_ms<5>();
		_buzzer = false;

		if (num > 1)
			sleep_ms<50>();
	}
}


void
LDI::slow_beep (uint8_t num)
{
	for (uint8_t i = 0; i < num; ++i)
	{
		for (uint8_t j = 0; j < 5; ++j)
		{
			_buzzer = true;
			sleep_ms<20>();
			_buzzer = false;
			sleep_ms<20>();
		}

		if (num > 1)
			sleep_ms<100>();
	}
}


void
LDI::signal (BuzzerSignal sig)
{
	switch (sig)
	{
		case BuzzerSignal::LaserSyncAcquired:
			fast_beep (3);
			break;

		case BuzzerSignal::LaserSyncLost:
			slow_beep (2);
			break;

		case BuzzerSignal::ExposingFinished:
			fast_beep (4);
			break;

		case BuzzerSignal::MovementWarning:
			slow_beep (3);
			break;
	}
}


void
LDI::setup_uart()
{
	fdev_setup_stream (&_uart_stream, LDI::uart_putchar, nullptr, _FDEV_SETUP_WRITE);

	_uart_rx.configure_as_input();
	_uart_tx = true;
	_uart_tx.configure_as_output();

	InterruptsLock lock;

	_uart.set_baud_rate<USART::Mode::Asynchronous> (kDebugPortBaudRate, kSystemClockHz);
	_uart.set_data_bits<8>();
	_uart.set_stop_bits<1>();
	_uart.set (USART::Parity::None);
	_uart.set (USART::Mode::Asynchronous);
	_uart.set_tx_enabled (true);
}


void
LDI::setup_motor_pwm()
{
	_motor_pid.set_integral_limit (-1.0, +1.0);

	_motor_pwm_timer.set (Timer01::Mode::SingleSlopePWM);
	_motor_pwm_timer.set (Timer01::ByteMode::Normal16bit);
	_motor_pwm_timer.set (Timer01::Direction::Up);
	_motor_pwm_timer.enable (Timer01::CompareCaptureChannel::A);
	_motor_pwm_timer.set_cc_value (Timer01::CompareCaptureChannel::A, 0);
	_motor_pwm_timer.set_period (kMotorPWMPeriod);
	_motor_pwm_timer.set (Timer01::ClockSource::Div1);
}


void
LDI::set_motor_pulse_width (uint16_t pulse_width)
{
	_motor_pwm_timer.set_cc_value_buffered (Timer01::CompareCaptureChannel::A, pulse_width);
}


void
LDI::setup_motor_reference_clock()
{
	// Duty cycle of the PD is about 13%, let's have similar duty cycle, because why not?
	constexpr float kDutyCycle = 0.13;
	// For n Hz, divide system clock by n.
	constexpr uint16_t kPeriod = kSystemClockHz / kScanFrequencyHz;

	_motor_reference_timer.set (Timer01::Mode::SingleSlopePWM);
	_motor_reference_timer.set (Timer01::ByteMode::Normal16bit);
	_motor_reference_timer.set (Timer01::Direction::Up);
	_motor_reference_timer.enable (Timer01::CompareCaptureChannel::A);
	_motor_reference_timer.set_cc_value (Timer01::CompareCaptureChannel::A, kDutyCycle * kPeriod);
	_motor_reference_timer.set_period (kPeriod - 1);
	_motor_reference_timer.set (Timer01::ClockSource::Div1);
}


void
LDI::setup_motor_frequency_meter()
{
	_last_photodiode_periods.fill (0);

	_motor_frequency_timer.set (Timer01::Mode::Normal);
	_motor_frequency_timer.set (Timer01::ByteMode::Normal16bit);
	_motor_frequency_timer.set (Timer01::Direction::Up);
	_motor_frequency_timer.enable (Timer01::CompareCaptureChannel::A);
	_motor_frequency_timer.set (Timer01::EventAction::FrequencyCapture);
	_motor_frequency_timer.set_event_source_bus<kPhotodiodeCaptureEventBus>();
	_motor_frequency_timer.set (Timer01::CompareCaptureChannel::A, InterruptSystem::Level::Medium);
	_motor_frequency_timer.set_cc_value (Timer01::CompareCaptureChannel::A, 0);
	_motor_frequency_timer.set_period (0xffff);
	_motor_frequency_timer.mark_buffers_invalid (Timer01::CompareCaptureChannel::A);
	_motor_frequency_timer.set (Timer01::ClockSource::Div1);
}


void
LDI::setup_laser_clock()
{
	// Method start_laser_clock() sets mode to PWM-generation as required.
	// Otherwise set this counter to Mode::Normal, so that manual control of _laser_clk is possible.
	_laser_clock_timer.set (Timer01::Mode::Normal);
	_laser_clock_timer.set (Timer01::ByteMode::Normal16bit);
	_laser_clock_timer.set (Timer01::Direction::Up);
	_laser_clock_timer.enable (Timer01::CompareCaptureChannel::A);
	// This gives kLaserClockHz frequency:
	_laser_clock_timer.set_cc_value (Timer01::CompareCaptureChannel::A, 1);
	_laser_clock_timer.set_period (kLaserClockDenominator - 1);
	// Laser clock is enabled/disabled on demand when scanline is exposed.
}


void
LDI::setup_scanline_timer()
{
	_scanline_timer.set (Timer01::Mode::Normal);
	_scanline_timer.set (Timer01::ByteMode::Normal16bit);
	_scanline_timer.set (Timer01::Direction::Up);
	_scanline_timer.enable (Timer01::CompareCaptureChannel::A, Timer01::CompareCaptureChannel::B);
	_scanline_timer.set (Timer01::CompareCaptureChannel::A, InterruptSystem::Level::High);
	_scanline_timer.set (Timer01::CompareCaptureChannel::B, InterruptSystem::Level::High);
	_scanline_timer.set_cc_value (Timer01::CompareCaptureChannel::A, kScanlineBeginPulse);
	_scanline_timer.set_cc_value (Timer01::CompareCaptureChannel::B, kPrepareForPhotodiodeSignal);
	_scanline_timer.set_period (0xffff);
}


void
LDI::setup_usb()
{
	_usb_control_buffer_out.fill (0);
	_usb_control_buffer_in.fill (0);

	// TODO this should be in USB::EndpointsTable ctor that takes usb::Device
	_usb_control_out.initialize();
	_usb_control_out.set (USB::Endpoint::Type::Control);
	_usb_control_out.set_buffer (_usb_control_buffer_out.data(), USB::Endpoint::ControlBulkBufferSize::_64);
	_usb_control_out.set_ready();

	_usb_control_in.initialize();
	_usb_control_in.set (USB::Endpoint::Type::Control);
	_usb_control_in.set_buffer (_usb_control_buffer_in.data(), USB::Endpoint::ControlBulkBufferSize::_64);
	_usb_control_in.set_nack_all (USB::Endpoint::Buffer::_0, true);
	_usb_control_in.set_azlp_enabled (false);

	_usb.calibrate();
	_usb.set_address (0);
	_usb.set (USB::Speed::Full);
	_usb.set_endpoints_table (_usb_endpoints);
	_usb.set (InterruptSystem::Level::Low);
	//_usb.set_store_framenum_enabled (true); // FIXME doesn't work?
	_usb.enable (USB::Interrupt::BusEvent, // XXX
				 USB::Interrupt::Stall,
				 USB::Interrupt::TransactionComplete,
				 USB::Interrupt::SetupTransactionComplete);
	_usb.set_enabled (true);
	_usb.set_attached (true);

	_usb_setup_conductor.reset();
}


inline void
LDI::set_scanline_begin_offset_factor (float factor)
{
	_scanline_timer.set_cc_value (Timer01::CompareCaptureChannel::A, factor * kScanlineBeginPulse);
	_scanline_timer.set_cc_value (Timer01::CompareCaptureChannel::B, factor * kPrepareForPhotodiodeSignal);
}


inline void
LDI::set_laser_word (uint8_t msb, uint8_t lsb)
{
	_laser_data_rev_lsb = lsb;
	_laser_data_rev_msb = msb;
	_laser_latch.signal();
}


inline void
LDI::push_laser_enabled_word()
{
	set_laser_word (0xff, 0xff);
	push_laser_word();
}


inline void
LDI::push_laser_disabled_word()
{
	set_laser_word (0, 0);
	push_laser_word();
}


inline void
LDI::push_laser_word()
{
	// Push laser word to the laser module registers:
	_laser_sync.signal();
	_laser_clk.signal (16);
}


inline void
LDI::start_laser_clock()
{
	_laser_clock_timer.set_value (0);
	// This takes control over the _laser_clk pin:
	_laser_clock_timer.set (Timer01::Mode::SingleSlopePWM);
	_laser_clock_timer.set (Timer01::ClockSource::Div1);
}


inline void
LDI::stop_laser_clock()
{
	// Disable override on the _laser_clk pin by _laser_clock_timer:
	_laser_clock_timer.set (Timer01::ClockSource::None);
	_laser_clock_timer.set (Timer01::Mode::Normal);
	// Make sure the _laser_clk line is low for consistency:
	_laser_clk = false;
}


inline void
LDI::set (LaserPower power)
{
	uint8_t power_int = static_cast<uint8_t> (power);

	_laser_power_bit_0 = power_int & (1 << 0);
	_laser_power_bit_1 = power_int & (1 << 1);
}


void
LDI::start_motor()
{
	_laser_enabled = false;
	_motor_speed_locked.store (false);
	move_head_to (kLaserCalibrationHeadPosition, HeadSpeed::Fast);

	task ("starting motor", [&] {
		_motor_enabled = true;
		// Fast initial spin-up, otherwise the motor might not even start:
		set_motor_pulse_width (kMotorPWMPeriod);
		sleep_ms<200>();
		// Target speed:
		set_motor_pulse_width (_motor_pulse_width);
		sleep_ms<500>();
	});

	// Begin synchronization with PLL:
	set (LaserPower::Power100Pct);
	push_laser_enabled_word();
	_laser_enabled = true;

	task ("waiting for frequency lock", [&] {
		// TODO if not acquired in some required time, try again, or report error to host
		while (!_motor_speed_locked.load())
			continue;
	});

	signal (BuzzerSignal::LaserSyncAcquired);
}


void
LDI::stop_motor()
{
	task ("stopping motor", [&] {
		push_laser_disabled_word();
		_laser_enabled = false;
		set_motor_pulse_width (0);
	});
}


void
LDI::calibrate_head()
{
	task ("calibrating head", [&] {
		signal (BuzzerSignal::MovementWarning);
		sleep_ms<500>();

		_head_energize = true;

		// TODO if more than some MAX steps were performed, and still no limit signal,
		// stop and signal error.

		while (move_head (HeadDirection::Down, 1, HeadSpeed::Fast))
			continue;

		while (move_head (HeadDirection::Up, 1, HeadSpeed::Slow) && is_limit_reached (HeadDirection::Down))
			continue;

		_head_position = 0;
		_head_calibrated = true;
	});
}


bool
LDI::move_head_to (uint32_t position, HeadSpeed speed)
{
	if (!_head_calibrated)
		calibrate_head();

	while (position > _head_position && move_head (HeadDirection::Up, 1, speed))
		continue;

	while (position < _head_position && move_head (HeadDirection::Down, 1, speed))
		continue;

	return is_limit_reached (HeadDirection::Up) || is_limit_reached (HeadDirection::Down);
}


inline bool
LDI::move_head (HeadDirection direction, uint32_t steps, HeadSpeed speed)
{
	_head_direction = static_cast<bool> (direction);

	for (uint32_t s = 0; s < steps; ++s)
	{
		if (is_limit_reached (direction))
			return false;
		else
		{
			_head_step.signal();

			switch (direction)
			{
				case HeadDirection::Up:
					_head_position += 1;
					break;

				case HeadDirection::Down:
					_head_position -= 1;
					break;
			}

			switch (speed)
			{
				case HeadSpeed::Slow:
					sleep_ms<kSlowMoveStepDelayMs>();
					break;

				case HeadSpeed::ExposingSpeed:
					sleep_ms<kExposingMoveStepDelayMs>();
					break;

				case HeadSpeed::Fast:
					sleep_ms<kFastMoveStepDelayMs>();
					break;
			}
		}
	}

	return true;
}


inline bool
LDI::is_limit_reached (HeadDirection direction)
{
	switch (direction)
	{
		case HeadDirection::Up:
			return _limits_top.get();

		case HeadDirection::Down:
			return _limits_bottom.get();
	}
}


inline int
LDI::uart_putchar (char c, FILE*)
{
	if (c == '\n')
		_uart.write_blocking ('\r');
	_uart.write_blocking (c);
	// This function is supposed to return the written character, but something is fucked up in AVR stdio.h, so
	// it has to return 0 instead.
	return 0;
}


void
LDI::debug (char const* format, ...)
{
	if (kDebugPortEnabled)
	{
		va_list args;
		va_start (args, format);
		vfprintf (&_uart_stream, format, args);
		va_end (args);
	}
}


void
LDI::debug_raw (char const* string)
{
	for (char const* c = string; *c != '\0'; ++c)
		uart_putchar (*c, nullptr);
}


template<class Function>
	inline void
	LDI::task (char const* name, Function function)
	{
		debug ("* %s...", name);
		function();
		debug (" done\n");
	}


inline char const*
LDI::to_string (HeadDirection direction)
{
	switch (direction)
	{
		case HeadDirection::Up:
			return "up";

		case HeadDirection::Down:
			return "down";
	}
}


char const*
LDI::to_string (HeadSpeed speed)
{
	switch (speed)
	{
		case HeadSpeed::Slow:
			return "slow";

		case HeadSpeed::ExposingSpeed:
			return "exposing speed";

		case HeadSpeed::Fast:
			return "fast";

		default:
			return "<?>";
	}
}


inline void
LDI::handle_photodiode_signal()
{
	// Start scanline timer:
	if (_motor_speed_locked.load())
	{
		_scanline_timer.set (Timer01::ClockSource::Div1);
		push_laser_disabled_word();
		set (_exposing_laser_power);
	}
}


inline void
LDI::handle_scanline_begin()
{
	if (_exposure_request.running())
	{
		push_laser_disabled_word();
		start_laser_clock();

		static_assert (_scanline_buffer.size() % 2 == 0, "Size of scanline_buffer must be divisible by 2.");

		for (size_t x = 0; x < _scanline_buffer.size(); x += 2)
		{
			_laser_prefetch.wait_for<true>();
			set_laser_word (_scanline_buffer[x + 1], _scanline_buffer[x]);
			_laser_prefetch.wait_for<false>(); // maybe not needed?
		}

		stop_laser_clock();
		_exposure_request.decrease();
	}
}


inline void
LDI::handle_prepare_laser_for_photodiode_sync()
{
	push_laser_enabled_word();
	set (LaserPower::Power100Pct);
	// Disable scanline timer and wait for next PD signal:
	_scanline_timer.set (Timer01::ClockSource::None);
	_scanline_timer.set_value (0);
}


inline void
LDI::handle_motor_frequency_capture (uint16_t actual_period)
{
	if (_motor_enabled.load())
	{
		static uint16_t locked_pulses = 0;
		static uint8_t i = 0;

		++i;
		uint8_t const i_mod = i % _last_photodiode_periods.size();
		auto& last = _last_photodiode_periods;

		for (size_t i = 0; i < last.size() - 1; ++i)
			last[i] = last[i + 1];

		last[last.size() - 1] = actual_period + 1;

		if (i_mod == 0)
		{
			float sum = 0.0;

			for (size_t i = 0; i < last.size(); ++i)
				sum += last[i];

			float const avg_period = sum / last.size();
			float const avg_frequency = 1.0f * kSystemClockHz / avg_period;

			float control_delta = _motor_pid (avg_frequency, 1.0f / kSystemClockHz * 4 * avg_period);
			float const frequency_ratio = _motor_pid.setpoint() / avg_frequency;

			// Make a correction to the position of proper-scanline-begin point in the scanline timer.
			// It should be OK to do it here, since handle_motor_frequency_capture() is called just after PD signal.
			set_scanline_begin_offset_factor (frequency_ratio);

			bool const locked = abs (_motor_pid.error()) < kMaxMotorFrequencyDeltaHz;
			constexpr int32_t kControlDeltaLimit = kMotorPWMPeriod / 1000;

			// TODO make mulabs_avr::clamp()
			if (control_delta > +kControlDeltaLimit)
				control_delta = +kControlDeltaLimit;

			if (control_delta < -kControlDeltaLimit)
				control_delta = -kControlDeltaLimit;

			if (locked)
			{
				if (locked_pulses < kRequiredLockedPulses)
					++locked_pulses;
			}
			else
				locked_pulses = 0;

			_motor_pulse_width += control_delta * (1.0f * kMotorPWMPeriod / 0xffff);
			set_motor_pulse_width (_motor_pulse_width);
			_motor_speed_locked.store (locked_pulses == kRequiredLockedPulses);
		}
	}
	else
		set_motor_pulse_width (0);
}


inline void
LDI::handle_usb_event()
{
	_usb.on (USB::InterruptFlag::Suspend, [&] {
		debug ("USB: Suspend\n");
	});

	_usb.on (USB::InterruptFlag::Resume, [&] {
		debug ("USB: Resume\n");
	});

	_usb.on (USB::InterruptFlag::Reset, [&] {
		debug ("USB: Reset\n");
	});

	_usb.on (USB::InterruptFlag::IsochronousCRCError, [&] {
		debug ("USB: Isochronous CRC error\n");
	});

	_usb.on (USB::InterruptFlag::Underflow, [&] {
		debug ("USB: Underflow\n");
	});

	_usb.on (USB::InterruptFlag::Overflow, [&] {
		debug ("USB: Overflow\n");
	});

	_usb.on (USB::InterruptFlag::Stall, [&] {
		debug ("USB: Stall\n");
	});
}


inline void
LDI::handle_usb_transaction_complete()
{
	auto show_endpoint = [&](auto const& ep) {
		if (ep.is_stalled())
			debug_raw (" [stalled]");
		else
			debug_raw (" ---------");

		if (ep.is_crc_error())
			debug_raw (" [crc error]");
		else
			debug_raw (" -----------");

		if (ep.transaction_complete())
			debug_raw (" [transaction complete]");
		else
			debug_raw (" ----------------------");

		if (ep.setup_transaction_complete())
			debug_raw (" [SETUP transaction complete]");
		else
			debug_raw (" ----------------------------");

		if (ep.is_nack_all (USB::Endpoint::Buffer::_0))
			debug_raw (" [NACK0]");
		else
			debug_raw (" -------");

		if (ep.is_nack_all (USB::Endpoint::Buffer::_1))
			debug_raw (" [NACK1]");
		else
			debug_raw (" -------");

		if (ep.next_data_packet() == USB::Endpoint::Data::_1)
			debug_raw (" [next: DATA1]");
		else
			debug_raw (" [next: DATA0]");
	};

	[[maybe_unused]] auto show_endpoint_out = [&] {
		auto const& ep = _usb_control_out;

		debug ("  OUT: %4u bytes", (unsigned int)_usb_control_out.transaction_size());
		show_endpoint (ep);
		if (ep.is_overflow())
			debug_raw (" [OVERFLOW]");
		else
			debug_raw (" -----------");
		debug_raw ("\n");
	};

	[[maybe_unused]] auto show_endpoint_in = [&] {
		auto const& ep = _usb_control_in;

		debug_raw ("  IN             ");
		show_endpoint (ep);
		if (ep.is_underflow())
			debug_raw (" [UNDERFLOW]");
		else
			debug_raw (" -----------");
		debug_raw ("\n");
	};

	[[maybe_unused]] auto show_endpoints = [&] {
		//show_endpoint_out();
		//show_endpoint_in();
	};

	debug_raw ("\n\nUSB INT ___________\n");
//	debug ("\n\nUSB: [ ");
//	for (size_t i = 0; i < _usb_control_out.transaction_size(); ++i)
//		debug ("%2x ", _usb_control_buffer_out[i]);
//	debug ("]\n");

	//show_endpoints();

	[[maybe_unused]] auto no_debug = [](char const*, ...) { };

	_usb.on (USB::InterruptFlag::TransactionComplete, [&] {
		_usb_setup_conductor.handle_interrupt (debug);
	});

	_usb.on (USB::InterruptFlag::SetupTransactionComplete, [&] {
		_usb_setup_conductor.handle_interrupt (debug);
	});
}


ISR(TCC0_CCA_vect)
{
	LDI::handle_scanline_begin();
}


ISR(TCC0_CCB_vect)
{
	LDI::handle_prepare_laser_for_photodiode_sync();
}


ISR(TCD1_CCA_vect)
{
	// Must read CCA to automatically clear the CCAIF flag to prevent interrupt being called right after return:
	LDI::handle_motor_frequency_capture (TCD1.CCA);
}


ISR(PORTD_INT0_vect)
{
	LDI::handle_photodiode_signal();
}


ISR(USB_BUSEVENT_vect)
{
	LDI::handle_usb_event();
}


ISR(USB_TRNCOMPL_vect)
{
	LDI::handle_usb_transaction_complete();
}


int
main()
{
	LDI::initialize();
	LDI::loop();
}

