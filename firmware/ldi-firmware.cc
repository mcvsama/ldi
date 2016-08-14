// Mulabs:
#include <mulabs_avr/mcu/atxmega128-a1u.h>

#include "event_channel_multiplexer.h"


using namespace mulabs::avr;


class MCU: public ATXMega128A1U
{
  public:
	static constexpr Pin	stepper_motor_direction	{ port_b.pin (6) };	// connector STEPPER_MOTOR pin 1
	static constexpr Pin	stepper_motor_pulse		{ port_b.pin (7) };	// connector STEPPER_MOTOR pin 2
	static constexpr Pin	stepper_motor_energize	{ port_a.pin (1) };	// connector STEPPER_MOTOR pin 3

	static constexpr Pin	mirror_motor_enable		{ port_a.pin (4) };	// connector MIRROR_MOTOR pin 2

	static constexpr Pin	limits_front			{ port_a.pin (7) };	// connector LIMITS/ESTOP pin 1
	static constexpr Pin	limits_back				{ port_a.pin (6) };	// connector LIMITS/ESTOP pin 2
	static constexpr Pin	limits_estop			{ port_a.pin (5) };	// connector LIMITS/ESTOP pin 3

	static constexpr Pin	uart_tx					{ port_d.pin (3) };	// connector UART pin 1
	static constexpr Pin	uart_rx					{ port_d.pin (2) };	// connector UART pin 2

	static constexpr Pin	usb_vusb				{ port_d.pin (5) };	// connector USB pin 1
	static constexpr Pin	usb_rxd					{ port_d.pin (6) };	// connector USB pin 2
	static constexpr Pin	usb_txd					{ port_d.pin (7) };	// connector USB pin 3

	static constexpr Pin	yellow_light_enable		{ port_f.pin (6) };	// connector YELLOW_LIGHT pin 2

	static constexpr Pin	i2c_sda					{ port_f.pin (0) };	// connector I2C pin 2
	static constexpr Pin	i2c_scl					{ port_f.pin (1) };	// connector I2C pin 3

	// data lines etc
};


int
main()
{
	MCU::Clock::enable_clock (MCU::Clock::RC32MHz);
	MCU::Clock::select_clock_for_cpu (MCU::Clock::ClockSource::RC32MHz);
	MCU::Clock::lock_system_clock();

	MCU::mirror_motor_enable.configure_as_output();
	MCU::yellow_light_enable.configure_as_output();

	MCU::sleep_ms (1000);
	MCU::mirror_motor_enable = true;
	MCU::sleep_ms (3000);
	MCU::mirror_motor_enable = false;

	while (true)
	{
		MCU::sleep_ms<500>();
		MCU::yellow_light_enable.toggle();
	}
}

