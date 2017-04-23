// Mulabs:
#include <mulabs_avr/mcu/atxmega128-a1u.h>
#include <avr/interrupt.h>


using namespace mulabs::avr;

constexpr bool using_evout_pd4 = false;

/**
 * Event buses:
 *  0 - PLL clock input (for divider)
 *  1 - Timer/Counter 0 underflow
 */
class MCU: public ATXMega128A1U
{
  public:
	/*
	 * External ports
	 */

	static constexpr Pin	stepper_motor_direction	{ port_b.pin (6) };	// (out) connector STEPPER_MOTOR pin 1
	static constexpr Pin	stepper_motor_pulse		{ port_b.pin (7) };	// (out) connector STEPPER_MOTOR pin 2
	static constexpr Pin	stepper_motor_energize	{ port_a.pin (1) };	// (out) connector STEPPER_MOTOR pin 3

	static constexpr Pin	mirror_motor_enable		{ port_a.pin (4) };	// (out) connector MIRROR_MOTOR pin 2

	static constexpr Pin	limits_front			{ port_a.pin (7) };	// (in)  connector LIMITS/ESTOP pin 1
	static constexpr Pin	limits_back				{ port_a.pin (6) };	// (in)  connector LIMITS/ESTOP pin 2
	static constexpr Pin	limits_estop			{ port_a.pin (5) };	// (in)  connector LIMITS/ESTOP pin 3

	static constexpr Pin	uart_tx					{ port_d.pin (3) };	// (out) connector UART pin 1
	static constexpr Pin	uart_rx					{ port_d.pin (2) };	// (in)  connector UART pin 2

	static constexpr Pin	usb_vusb				{ port_d.pin (5) };	// (in)  connector USB pin 1
	static constexpr Pin	usb_rxd					{ port_d.pin (6) };	// (in)  connector USB pin 2
	static constexpr Pin	usb_txd					{ port_d.pin (7) };	// (out) connector USB pin 3

	static constexpr Pin	yellow_light_enable		{ port_f.pin (6) };	// (out) connector YELLOW_LIGHT pin 2

	static constexpr Pin	i2c_sda					{ port_f.pin (0) };	// (in/out) connector I2C pin 2
	static constexpr Pin	i2c_scl					{ port_f.pin (1) };	// (in/out) connector I2C pin 3

	/*
	 * Laser-driver connector
	 */

	// Bits are reversed (MSB is LSB and vice versa) in these two laser_data ports:
	static constexpr Port	laser_data_rev_lsb		{ port_e };			// (out)
	static constexpr Port	laser_data_rev_msb		{ port_c };			// (out)
	// Used to isolate latch/power/enable lines from laser module:
	static constexpr Pin	laser_isolate			{ port_b.pin (2) };	// (out)
	static constexpr Pin	laser_latch				{ port_b.pin (3) };	// (out)
	// Power bits set to high inhibit power-output:
	static constexpr Pin	laser_power_bit_0		{ port_b.pin (4) };	// (out)
	static constexpr Pin	laser_power_bit_1		{ port_b.pin (5) };	// (out)
	static constexpr Pin	laser_enable			{ port_d.pin (3) };	// (out)
	static constexpr Pin	laser_sync				{ port_b.pin (0) };	// (out)
	static constexpr Pin	laser_prefetch			{ port_b.pin (1) };	// (in)

	/*
	 * Counter/divider
	 */

	static constexpr Pin	divider_in				{ port_d.pin (0) };	// (in)

	static constexpr Port	divider_lsb				{ port_h };			// (out)
	static constexpr Port	divider_msb				{ port_j };			// (out)
	static constexpr Pin	divider_ka				{ port_k.pin (0) };	// (out)
	static constexpr Pin	divider_kb				{ port_k.pin (1) };	// (out)
	static constexpr Pin	divider_kc				{ port_k.pin (2) };	// (out)
	static constexpr Pin	divider_latch_enable	{ port_k.pin (3) };	// (out)
	static constexpr Pin	divider_out				{ using_evout_pd4 ? port_d.pin (4) : port_f.pin (5) };	// (out)

	/*
	 * Other controls
	 */

	static constexpr Pin	pll_inhibit				{ port_f.pin (2) };	// (out)
	static constexpr Pin	pll_lock_acquired		{ port_f.pin (3) };	// (in)
	static constexpr Pin	pll_comp_input_enable	{ port_f.pin (4) };	// (out)
};


constexpr MCU::Pin MCU::stepper_motor_direction;
constexpr MCU::Pin MCU::stepper_motor_pulse;
constexpr MCU::Pin MCU::stepper_motor_energize;

constexpr MCU::Pin MCU::mirror_motor_enable;

constexpr MCU::Pin MCU::limits_front;
constexpr MCU::Pin MCU::limits_back;
constexpr MCU::Pin MCU::limits_estop;

constexpr MCU::Pin MCU::uart_tx;
constexpr MCU::Pin MCU::uart_rx;

constexpr MCU::Pin MCU::usb_vusb;
constexpr MCU::Pin MCU::usb_rxd;
constexpr MCU::Pin MCU::usb_txd;

constexpr MCU::Pin MCU::yellow_light_enable;

constexpr MCU::Pin MCU::i2c_sda;
constexpr MCU::Pin MCU::i2c_scl;

constexpr MCU::Port MCU::laser_data_rev_lsb;
constexpr MCU::Port MCU::laser_data_rev_msb;
constexpr MCU::Pin MCU::laser_isolate;
constexpr MCU::Pin MCU::laser_latch;
constexpr MCU::Pin MCU::laser_power_bit_0;
constexpr MCU::Pin MCU::laser_power_bit_1;
constexpr MCU::Pin MCU::laser_enable;
constexpr MCU::Pin MCU::laser_sync;
constexpr MCU::Pin MCU::laser_prefetch;

constexpr MCU::Port MCU::divider_lsb;
constexpr MCU::Port MCU::divider_msb;
constexpr MCU::Pin MCU::divider_ka;
constexpr MCU::Pin MCU::divider_kb;
constexpr MCU::Pin MCU::divider_kc;
constexpr MCU::Pin MCU::divider_latch_enable;
constexpr MCU::Pin MCU::divider_out;

constexpr MCU::Pin MCU::pll_inhibit;
constexpr MCU::Pin MCU::pll_lock_acquired;
constexpr MCU::Pin MCU::pll_comp_input_enable;

Array<uint8_t, 1024> scanline_buffer;


volatile bool tcc0_overflow = false;
constexpr size_t kTCC0_PERIOD = 65535;
constexpr size_t kTCC0_A = kTCC0_PERIOD / 4;
constexpr size_t kTCC0_B = kTCC0_PERIOD / 4 * 3;


/**
 * Counter 0 overflow handler.
 */
ISR(TCC0_OVF_vect)
{
	// When TC0 underflows/overflows event 1 is generated, which
	// generates signal on MCU::divider_out. No need to do this manually
	// here.

	// TODO instead of manually signalling, route event 1 out of MCU::divider_out: see SECTION_AB
	MCU::divider_out.signal();

	// TODO should auto restart TCC0.CNT = TCC0_VALUE;
	tcc0_overflow = true;
}


ISR(TCC0_CCA_vect)
{
	MCU::sleep_us<1>();

	MCU::laser_enable = false;
	MCU::sleep_us<1>();
	MCU::laser_enable = false;
	MCU::sleep_us<1>();
	MCU::laser_enable = true;
	MCU::sleep_us<1>();
	MCU::laser_enable = false;
	MCU::sleep_us<1>();
	MCU::laser_enable = true;
	MCU::sleep_us<1>();
	MCU::laser_enable = false;
	MCU::sleep_us<1>();
	MCU::laser_enable = true;
}


ISR(TCC0_CCB_vect)
{
//	MCU::laser_enable = true;
	// TODO
}


int
main()
{
	MCU::JTAG::disable();
	MCU::Clock::enable_clock (MCU::Clock::RC32MHz);
	MCU::Clock::select_clock_for_cpu (MCU::Clock::ClockSource::RC32MHz);
	MCU::Clock::lock_system_clock();

	scanline_buffer.fill (0);

	// TODO make configures_as_outputs() that takes list of pins from a single Port.

	// Outputs:
	MCU::stepper_motor_direction.configure_as_output();
	MCU::stepper_motor_pulse.configure_as_output();
	MCU::stepper_motor_energize.configure_as_output();
	MCU::mirror_motor_enable.configure_as_output();
	MCU::uart_tx.configure_as_output();
	MCU::usb_txd.configure_as_output();
	MCU::yellow_light_enable.configure_as_output();
	MCU::laser_data_rev_lsb.configure_as_outputs (0xff);
	MCU::laser_data_rev_msb.configure_as_outputs (0xff);
	MCU::laser_isolate.configure_as_output();
	MCU::laser_latch.configure_as_output();
	MCU::laser_power_bit_0.configure_as_output();
	MCU::laser_power_bit_1.configure_as_output();
	MCU::laser_enable.configure_as_output();
	MCU::laser_sync.configure_as_output(); // TODO should be output; on PCB it's configured as input, but it's an output actually - fix PCB
	MCU::divider_lsb.configure_as_outputs (0xff);
	MCU::divider_msb.configure_as_outputs (0xff);
	MCU::divider_ka.configure_as_output();
	MCU::divider_kb.configure_as_output();
	MCU::divider_kc.configure_as_output();
	MCU::divider_latch_enable.configure_as_output();
	MCU::divider_out.configure_as_output();
	MCU::pll_inhibit.configure_as_output();
	MCU::pll_comp_input_enable.configure_as_output();

	// Inputs:
	MCU::limits_front.configure_as_input();
	MCU::limits_back.configure_as_input();
	MCU::limits_estop.configure_as_input();
	MCU::uart_rx.configure_as_input();
	MCU::usb_vusb.configure_as_input();
	MCU::usb_rxd.configure_as_input();
	MCU::laser_prefetch.configure_as_input();
	MCU::pll_lock_acquired.configure_as_input();
	MCU::divider_in.configure_as_input();

	// TODO configure I2C here.

	// Set initial values:
	MCU::laser_isolate = false;
	MCU::laser_latch = false;
	MCU::laser_sync = false;
	MCU::pll_inhibit = true;
	MCU::pll_comp_input_enable = false;
	MCU::mirror_motor_enable = false;

	MCU::pll_inhibit = false;
	MCU::pll_comp_input_enable = false;

	MCU::laser_data_rev_lsb = 0xff;
	MCU::laser_data_rev_msb = 0xff;
	MCU::laser_power_bit_0 = false;
	MCU::laser_power_bit_1 = false;
	MCU::laser_enable = false;
	MCU::laser_latch.signal();
	MCU::laser_sync.signal();

	//XXX chyba pomaga na niewłączający się laser, tzn. sync jest by default up, a signal=temporary low
	MCU::laser_latch = true;
	MCU::sleep_us<2>();
	MCU::laser_latch = false;
	MCU::laser_sync = false;
	MCU::sleep_us<2>();
	MCU::laser_sync = true;

	size_t m = scanline_buffer.size() / 4;
	for (size_t i = 0; i < m; ++i)
		scanline_buffer[i] = 0x00;
	for (size_t i = m; i < m * 2; ++i)
		scanline_buffer[i] = 0xff;

	size_t index = 0;

	MCU::pll_comp_input_enable = true;

	MCU::laser_data_rev_lsb = 0xff;
	MCU::laser_data_rev_msb = 0xff;
	MCU::laser_latch.signal();
	MCU::laser_sync.signal();
	MCU::laser_enable = true;

	// TODO class EventSystem
	// TODO class TimerCounter0
	// Configure the divider_in pin as event source for event channel 0:
	MCU::EventSystem::set_event_source_for_bus<0> (MCU::EventSystem::EventSource::PortDPin0);

	// TODO remove: EVSYS.CH0MUX = static_cast<uint8_t> (EventSource::PortDPin0);

	// Enable CCB and CCA (bits 5 and 4) for inter-scanline interrupts (scanline begin, scanline end):
	TCC0.CTRLB = 0b0011'0000;
	TCC0.CTRLC = 0;
	TCC0.CTRLD = 0;
	TCC0.CTRLE = 0b0000'0000; // Normal mode.
	// Configure interrupt for overflow/underflow (high priority):
	TCC0.INTCTRLA = 0b0000'0011; // 0b11 is high level interrupt, 0b00 means ints are disabled
	// Configure interrupts for CCA and CCB (high priority):
	TCC0.INTCTRLB = 0b0000'1111;
	TCC0.CTRLFCLR = 0b1; // Make counter count up to PER.
	// Compare/capture channels:
	TCC0.CCABUF = kTCC0_A; // TODO value is now random
	TCC0.CCBBUF = kTCC0_B; // TODO value is now random
	TCC0.CCCBUF = 0;
	TCC0.CCDBUF = 0;
	TCC0.PERBUF = kTCC0_PERIOD;
	// Signal that PERBUF, CCABUF and CCBBUF are valid and should be used:
	TCC0.CTRLGSET = 0b0000'0111;

	// Signal that PERBUF is valid and should be used: //XXX
	//TCC0.CTRLGSET = 0b0000'0000; // XXX

	// TODO check if it works withouth the following
	TCC0.CCA = TCC0.CCABUF;
	TCC0.CCB = TCC0.CCBBUF;
	TCC0.PER = TCC0.PERBUF;
	// TODO end of check
	// Enable high-level interrupts: TODO should be in InterruptSystem
	PMIC.CTRL = 0b0000'0100;

	MCU::divider_out = false;

	if (using_evout_pd4)
	{
		// Configure event system so that OVF interrupt for TCC0 also generate event on bus 1:
		EVSYS_CH1MUX = static_cast<uint8_t> (MCU::EventSystem::event_source_for_timer (MCU::EventSystem::TimerCounter::C0, MCU::EventSystem::TimerEventType::OverOrUnderflow));
		// TODO see EVCTRL
		PORTCFG_CLKEVOUT = 0;
		// Route event bus 1 to MCU::divider_out (port port D's EVOUT):
		PORTCFG_CLKEVOUT |= 0b10 << 4;
		// Use alternate pin 4 instead of pin 7:
		PORTCFG_CLKEVOUT |= 0b1 << 7;
		PORTCFG_EVCTRL = 0b001; // Output event channel 1 to the pin.
	}

	// Enable TCC0: use event channel 0 for clock source for TC0, thus enabling TCC0:
	TCC0.CTRLA = 0b0000'1000;
	// Set counter value:
	TCC0.CNT = 0;

	sei();

	MCU::yellow_light_enable = true;

	MCU::sleep_ms<700>();
	MCU::mirror_motor_enable = true;

	MCU::laser_enable = true;
	// Wait for motor to spin up:
	MCU::sleep_ms<2000>();

	// Wait for lock:
//	while (!MCU::pll_lock_acquired.get())
//		continue;

	while (true)
	{
		// Wait for signal from counter:
		while (!tcc0_overflow)
			continue;
		tcc0_overflow = false;

		MCU::yellow_light_enable = !MCU::pll_lock_acquired.get();
	}

#if 0
	uint16_t p = 0;
	uint16_t k = 6400;
	while (true)
	{
		p++;
		MCU::laser_enable = p > k;
		if (p > 2 * k)
			p = 0;
	}
#endif

//	size_t p = 0;
//	while (true)
//	{
//		p++;
//		MCU::laser_power_bit_0 = p & 1;
//		MCU::laser_power_bit_1 = (p << 1) & 1;
//
//		//MCU::laser_enable = false;
//		MCU::sleep_ms<1000>();
//		MCU::laser_enable = true;
//	}

	while (true)
	{
		// Wait for signal from divider:
		MCU::divider_out.wait_for<true>(); // XXX wtf, usuń

		index += 2;
		if (index >= 1024)
			index = 0;

		while (!MCU::laser_prefetch.get())
			continue;
		while (MCU::laser_prefetch.get())
			continue;

		//MCU::laser_data_rev_lsb = scanline_buffer[index + 0];
		//MCU::laser_data_rev_msb = scanline_buffer[index + 1];
		MCU::sleep_ms<1>();

		MCU::laser_enable = index > 512;
		MCU::laser_latch = true;
		//MCU::laser_sync = true;
		//MCU::sleep_us<1>();
		MCU::laser_latch = false;
		//MCU::laser_sync = false;
	}
}

