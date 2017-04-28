/*
 * Machine states:
 *
 * <IDLE>
 *      Not interruptible.
 * <CALIBRATING HEAD>
 *      Just likemoving head. Actually this command issues <MOVING HEAD> commands
 *      for moving the head.
 * <SYNCING PLL>
 *      When interrupted, leaves PLL calibration, laser off, etc.
 * <MOVING HEAD>
 *      When interrupted, saves current head position and goes <IDLE>.
 * <EXPOSING SCANLINE>
 *      Not interruptible.
 */

// TODO class EventSystem
// TODO class TimerCounter0

// Mulabs:
#include <mulabs_avr/mcu/atxmega128-a1u.h>
#include <avr/interrupt.h>


using namespace mulabs::avr;

constexpr bool using_evout_pd4 = false; // XXX


/**
 * This class assumes that 8-bit boolean read/write is atomic.
 */
class AtomicFlag
{
  public:
	// Ctor
	AtomicFlag() = default;

	// Ctor
	AtomicFlag (bool initial_value);

	AtomicFlag const&
	operator= (bool value);

	operator bool() const;

  private:
	volatile bool _flag { false };
};


class ExposureRequest
{
  public:
	/**
	 * Set new exposures number.
	 */
	void
	set_exposures_number (uint8_t);

	/**
	 * Start exposing
	 */
	void
	start();

	/**
	 * Return true if exposure request is started and there are still
	 * exposures to be done.
	 */
	bool
	running() const;

	/**
	 * Decrease counter.
	 * Only allowed to execute when interrupts are disabled.
	 */
	void
	decrease(); // TODO pass an InterruptsDiabledToken

	/**
	 * Return true if exposing is finished.
	 * When it happens, internal exposure counter is 0, and the object
	 * is ready to start again.
	 */
	bool
	finished();

  private:
	uint8_t			_num_exposures	{ 0 };
	AtomicFlag		_executing		{ false };
};


/**
 * Event buses:
 *  0 - PLL clock input (for divider)
 *  1 - Timer/Counter 0 underflow
 */
class LDI: public ATXMega128A1U
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
	// Latch data in laser registers on rising edge:
	static constexpr Pin	laser_latch				{ port_b.pin (3) };	// (out)
	// Power bits set to high inhibit power-output:
	static constexpr Pin	laser_power_bit_0		{ port_b.pin (4) };	// (out)
	static constexpr Pin	laser_power_bit_1		{ port_b.pin (5) };	// (out)
	// Power the laser (doesn't mean it's on):
	static constexpr Pin	laser_enable			{ port_d.pin (3) };	// (out)
	// Set high (in reality low) for resetting 4-bit counter to 0 in the laser module:
	static constexpr Pin	laser_sync				{ port_b.pin (0) };	// (out, inverted)
	// Signalled when laser module is ready to take another data into registers:
	static constexpr Pin	laser_prefetch			{ port_b.pin (1) };	// (in)

	/*
	 * Counter/divider
	 */

	static constexpr Pin	clk_in					{ port_d.pin (0) };	// (in)

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

	static constexpr Pin	pll_enable				{ port_f.pin (2) };	// (out, inversed)
	static constexpr Pin	pll_lock_acquired		{ port_f.pin (3) };	// (in)
	static constexpr Pin	pll_comp_input_enable	{ port_f.pin (4) };	// (out) XXX this output and the AND gate might not be needed at all.

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
	configure_ports();

	static void
	fix_laser();

	static void
	setup_test_values();

	static void
	setup_counter();

	static void
	set_laser_word (uint8_t msb, uint8_t lsb);

	static void
	set_laser_power (LaserPower);

	static void
	check_for_errors();

	/**
	 * This needs interrupts enabled.
	 */
	static void
	execute_synchronize_laser();

	/**
	 * Stops PLL, motor and laser.
	 */
	static void
	execute_desynchronize_laser();

  public:
	/**
	 * Called at the beginning of each period to emit signal for PLL.
	 */
	static void
	handle_counter_overflow();

	/**
	 * Called when laser has hit the photodiode for syncing and now can be
	 * turned off.
	 */
	static void
	handle_photodiode_sync_done();

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

  private:
	// The size must be divisible by 2:
	static Array<uint8_t, 2 * 512>	_scanline_buffer;
	static LaserPower				_exposing_laser_power;
	static ExposureRequest			_exposure_request;
	// TODO static int16_t _head_position
};


inline
AtomicFlag::AtomicFlag (bool initial_value):
	_flag (initial_value)
{ }


inline AtomicFlag const&
AtomicFlag::operator= (bool value)
{
	_flag = value;
	return *this;
}


inline
AtomicFlag::operator bool() const
{
	return _flag;
}


inline void
ExposureRequest::set_exposures_number (uint8_t number)
{
	_num_exposures = number;
}


inline void
ExposureRequest::start()
{
	_executing = true;
}


inline bool
ExposureRequest::running() const
{
	return _executing;
}


inline void
ExposureRequest::decrease()
{
	if (_num_exposures > 0)
		--_num_exposures;

	if (_num_exposures == 0)
		_executing = false;
}


inline bool
ExposureRequest::finished()
{
	return !_executing;
}


// XXX In C++17 the constexpr definitions are not needed:
constexpr LDI::Pin		LDI::stepper_motor_direction;
constexpr LDI::Pin		LDI::stepper_motor_pulse;
constexpr LDI::Pin		LDI::stepper_motor_energize;

constexpr LDI::Pin		LDI::mirror_motor_enable;

constexpr LDI::Pin		LDI::limits_front;
constexpr LDI::Pin		LDI::limits_back;
constexpr LDI::Pin		LDI::limits_estop;

constexpr LDI::Pin		LDI::uart_tx;
constexpr LDI::Pin		LDI::uart_rx;

constexpr LDI::Pin		LDI::usb_vusb;
constexpr LDI::Pin		LDI::usb_rxd;
constexpr LDI::Pin		LDI::usb_txd;

constexpr LDI::Pin		LDI::yellow_light_enable;

constexpr LDI::Pin		LDI::i2c_sda;
constexpr LDI::Pin		LDI::i2c_scl;

constexpr LDI::Port		LDI::laser_data_rev_lsb;
constexpr LDI::Port		LDI::laser_data_rev_msb;
constexpr LDI::Pin		LDI::laser_isolate;
constexpr LDI::Pin		LDI::laser_latch;
constexpr LDI::Pin		LDI::laser_power_bit_0;
constexpr LDI::Pin		LDI::laser_power_bit_1;
constexpr LDI::Pin		LDI::laser_enable;
constexpr LDI::Pin		LDI::laser_sync;
constexpr LDI::Pin		LDI::laser_prefetch;

constexpr LDI::Port		LDI::divider_lsb;
constexpr LDI::Port		LDI::divider_msb;
constexpr LDI::Pin		LDI::divider_ka;
constexpr LDI::Pin		LDI::divider_kb;
constexpr LDI::Pin		LDI::divider_kc;
constexpr LDI::Pin		LDI::divider_latch_enable;
constexpr LDI::Pin		LDI::divider_out;

constexpr LDI::Pin		LDI::pll_enable;
constexpr LDI::Pin		LDI::pll_lock_acquired;
constexpr LDI::Pin		LDI::pll_comp_input_enable;

Array<uint8_t, 2 * 512>	LDI::_scanline_buffer;
LDI::LaserPower			LDI::_exposing_laser_power	{ LaserPower::Power100Pct }; // TODO change default to lowest
ExposureRequest			LDI::_exposure_request;


/**
 * Swap MSB-LSB bits in a byte.
 */
uint8_t
reverse_bits (uint8_t byte)
{
    byte = ((byte >> 1) & 0x55) | ((byte << 1) & 0xaa);
    byte = ((byte >> 2) & 0x33) | ((byte << 2) & 0xcc);
    byte = ((byte >> 4) & 0x0f) | ((byte << 4) & 0xf0);
    return byte;
}


void
LDI::initialize()
{
	configure_mcu();

	// Enable high-level interrupts: TODO should be in InterruptSystem
	PMIC.CTRL = 0b0000'0100;

	configure_ports();
	setup_counter();
	_scanline_buffer.fill (0);
	fix_laser();

	setup_test_values();
}


void
LDI::loop()
{
	sei();

	execute_synchronize_laser();

	while (true)
	{
//		if (!pll_lock_acquired.get())
//			execute_synchronize_laser();

		check_for_errors();

		//sleep_ms<1000>();
		//execute_desynchronize_laser();
		//sleep_ms<200>();
	}
}


void
LDI::configure_mcu()
{
	JTAG::disable();
	Clock::enable_clock (Clock::RC32MHz);
	Clock::select_clock_for_cpu (Clock::ClockSource::RC32MHz);
	Clock::lock_system_clock();
}


void
LDI::configure_ports()
{
	// TODO make configure_as_outputs() that takes list of pins from a single Port.

	// Outputs:
	stepper_motor_direction.configure_as_output();
	stepper_motor_pulse.configure_as_output();
	stepper_motor_energize.configure_as_output();
	mirror_motor_enable.configure_as_output();
	uart_tx.configure_as_output();
	usb_txd.configure_as_output();
	yellow_light_enable.configure_as_output();
	laser_data_rev_lsb.configure_as_outputs (0xff);
	laser_data_rev_msb.configure_as_outputs (0xff);
	laser_isolate.configure_as_output();
	laser_latch.configure_as_output();
	laser_power_bit_0.configure_as_output();
	laser_power_bit_1.configure_as_output();
	laser_enable.configure_as_output();
	laser_sync.configure_as_output(); // TODO should be output; on PCB it's configured as input, but it's an output actually - fix PCB
	divider_lsb.configure_as_outputs (0xff);
	divider_msb.configure_as_outputs (0xff);
	divider_ka.configure_as_output();
	divider_kb.configure_as_output();
	divider_kc.configure_as_output();
	divider_latch_enable.configure_as_output();
	divider_out.configure_as_output();
	pll_enable.configure_as_output();
	pll_comp_input_enable.configure_as_output();

	// Inputs:
	limits_front.configure_as_input();
	limits_back.configure_as_input();
	limits_estop.configure_as_input();
	uart_rx.configure_as_input();
	usb_vusb.configure_as_input();
	usb_rxd.configure_as_input();
	laser_prefetch.configure_as_input();
	pll_lock_acquired.configure_as_input();
	clk_in.configure_as_input();

	// TODO configure I2C here.

	// Set initial values:
	laser_isolate = false;
	laser_latch = false;
	laser_sync.set_inverted_io (true);
	laser_sync = false;
	pll_enable.set_inverted_io (true);
	pll_enable = false;
	pll_comp_input_enable = false;
	mirror_motor_enable = false;

	laser_data_rev_lsb = 0xff;
	laser_data_rev_msb = 0xff;
	laser_power_bit_0.set_inverted_io (true);
	laser_power_bit_0 = false;
	laser_power_bit_1.set_inverted_io (true);
	laser_power_bit_1 = false;
	laser_enable = false;
	laser_latch.signal();
	laser_sync.signal();
}


void
LDI::fix_laser()
{
	//XXX chyba pomaga na niewłączający się laser, tzn. sync jest by default up, a signal=temporary low
	laser_latch = true;
	sleep_us<2>();
	laser_latch = false;
	laser_sync = true;
	sleep_us<2>();
	laser_sync = false;
}


void
LDI::setup_test_values()
{
	constexpr size_t s = 16;
	constexpr size_t m = _scanline_buffer.size() / s;

	for (size_t k = 0; k < s; ++k)
	{
		for (size_t i = k * m; i < (k + 1) * m; ++i)
			_scanline_buffer[i] = (k % 2 == 0) ? 0xff : 0x00;
	}
}


void
LDI::setup_counter()
{
	// Use counter 0 and its comparators A and B.
	// Using high-priority interrupts.

	constexpr size_t kTotalLinePulses = 1000; // Number of pulses per whole laser line, not just the scanline.
	constexpr size_t kSyncDonePulse = kTotalLinePulses / 8 * 1; // TODO just a guess
	constexpr size_t kScanlineBeginPulse = kTotalLinePulses / 8 * 2; // TODO this is just an estimation
	constexpr size_t kPhotodiodeLaserEnablePulse = kTotalLinePulses / 8 * 7; // TODO this is just an estimation

	//TODO uncomment:
	//static_assert (8 * _scanline_buffer.size() < 0.9 * kTotalLinePulses);

	// Enable compare/capture channels CCC, CCB and CCA (bits 6, 5 and 4) for inter-scanline interrupts (scanline begin, scanline end):
	TCC0.CTRLB = 0b0111'0000;

	// Other control bits to 0:
	TCC0.CTRLC = 0;
	TCC0.CTRLD = 0;

	// Counter works in normal, 16-bit mode:
	TCC0.CTRLE = 0b0000'0000;

	// Configure overflow/underflow interrupt to have high priority:
	TCC0.INTCTRLA = 0b0000'0011; // 0b11 is high level interrupt (0b00 means interruptss are disabled)

	// Configure interrupts for CCA, CCB, CCC to have high priority:
	TCC0.INTCTRLB = 0b0011'1111;

	// Make counter count up to PER.
	TCC0.CTRLFCLR = 0b1;

	// Compare/capture channels and period value into 16-bit buffers:
	TCC0.CCABUF = kSyncDonePulse;
	TCC0.CCBBUF = kScanlineBeginPulse;
	TCC0.CCCBUF = kPhotodiodeLaserEnablePulse;
	TCC0.CCDBUF = 0;
	TCC0.PERBUF = kTotalLinePulses;

	// Signal that PERBUF, CCABUF and CCBBUF are valid and should be used:
	TCC0.CTRLGSET = 0b0000'1111;

	// Actually set counter values:
	// TODO check if it works withouth the following, since the above CTRLGSET should make the counter autoupdate
	// counters from the BUFfered values.
	//TCC0.CCA = TCC0.CCABUF;
	//TCC0.CCB = TCC0.CCBBUF;
	//TCC0.CCC = TCC0.CCCBUF;
	//TCC0.CCD = TCC0.CCDBUF;
	//TCC0.PER = TCC0.PERBUF;
	//TCC0.CNT = 0;
	// TODO end of check

	if (using_evout_pd4)
	{
		// Configure event system so that OVF interrupt for TCC0 also generate event on bus 1:
		EVSYS_CH1MUX = static_cast<uint8_t> (EventSystem::event_source_for_timer (EventSystem::TimerCounter::C0, EventSystem::TimerEventType::OverOrUnderflow));
		// TODO see EVCTRL
		PORTCFG_CLKEVOUT = 0;
		// Route event bus 1 to divider_out (port port D's EVOUT):
		PORTCFG_CLKEVOUT |= 0b10 << 4;
		// Use alternate pin 4 instead of pin 7:
		PORTCFG_CLKEVOUT |= 0b1 << 7;
		PORTCFG_EVCTRL = 0b001; // Output event channel 1 to the pin.
	}
}


inline void
LDI::set_laser_word (uint8_t msb, uint8_t lsb)
{
	// TODO experiment with no reverse, with MSbyte<->LSbyte… to check if it's correct:
	laser_data_rev_lsb = reverse_bits (lsb);
	laser_data_rev_msb = reverse_bits (msb);
	laser_latch.signal();
}


inline void
LDI::set_laser_power (LaserPower power)
{
	uint8_t power_int = static_cast<uint8_t> (power);

	laser_power_bit_0 = power_int & (1 << 0);
	laser_power_bit_1 = power_int & (1 << 1);
}


void
LDI::check_for_errors()
{
	// Verify that PLL is still synchronized.
	if (!pll_lock_acquired.get())
	{
		// TODO
		// Set the error flag, wait for host to read it and reset it.
		// Break the current command.
	}
}


void
LDI::execute_synchronize_laser()
{
	pll_enable = false;
	mirror_motor_enable = false;
	sleep_ms<300>();

	pll_enable = true;

	// Move head to the laser-safe area.
	// TODO

	// Wait for motor to spin up:
	mirror_motor_enable = true;
	sleep_ms<2000>();

	// Begin synchronization with PLL:
	set_laser_power (LaserPower::Power100Pct);
	set_laser_word (0xff, 0xff);
	laser_sync.signal();
	laser_enable = true;

	// Initially set divider level to low:
	divider_out = false;

	// Enable comparator-input in PLL:
	pll_comp_input_enable = true;

	// Configure the clk_in pin as event source for event channel 0:
	EventSystem::set_event_source_for_bus<0> (EventSystem::EventSource::PortDPin0);

	// Enable clocking of the counter: use event channel 0 for clock source for TC0, thus enabling TCC0:
	TCC0.CTRLA = 0b0000'1000;

	// Wait for lock:
	while (!pll_lock_acquired.get())
		continue;

	// Still wait a bit to ensure PLL is stable:
	sleep_ms<100>();
}


// TODO yellow light on, when not exposing.
// TODO separate green LED indicating synchronization is acquired.
// TODO separate big RED indicating that some command is being executed.


void
LDI::execute_desynchronize_laser()
{
	laser_enable = false;
	set_laser_word (0x00, 0x00);
	mirror_motor_enable = false;
	pll_enable = false;
}


void
LDI::handle_counter_overflow()
{
	// When TC0 underflows/overflows event 1 is generated, which
	// generates signal on divider_out. No need to do this manually
	// here.

	// TODO instead of manually signalling, route event 1 out of divider_out: see SECTION_AB
	divider_out.signal();

	// TODO should auto restart TCC0.CNT = TCC0_VALUE;
	//TCC0.CNT = 1000; // XXX needed?

	yellow_light_enable = !pll_lock_acquired.get(); // XXX
}


inline void
LDI::handle_photodiode_sync_done()
{
	if (pll_lock_acquired.get())
	{
#if 0
	laser_enable = false;
	set_laser_word (0x00, 0x00);
	set_laser_power (_exposing_laser_power);
#endif
	}
}


inline void
LDI::handle_scanline_begin()
{
	bool debug_true = false; // XXX
	if (debug_true || _exposure_request.running())
	{
		set_laser_word (0x00, 0x00);
		laser_sync.signal();
		laser_enable = true;

		static_assert (_scanline_buffer.size() % 2 == 0, "Size of scanline_buffer must be divisible by 2.");

		for (size_t x = 0; x < _scanline_buffer.size(); x += 2)
		{
			laser_prefetch.wait_for<true>();
			laser_prefetch.wait_for<false>();
			//set_laser_word (0xff, 0xff);
			set_laser_word (_scanline_buffer[x + 1], _scanline_buffer[x]);
		}

		laser_enable = false;
		set_laser_word (0x00, 0x00);

		if (!debug_true)
			_exposure_request.decrease();
	}
}


inline void
LDI::handle_prepare_laser_for_photodiode_sync()
{
	if (true || pll_lock_acquired.get())
	{
		set_laser_word (0xff, 0xff);
		set_laser_power (LaserPower::Power100Pct);
		laser_enable = true;
	}
}


ISR(TCC0_OVF_vect)
{
	LDI::handle_counter_overflow();
}


ISR(TCC0_CCA_vect)
{
	LDI::handle_photodiode_sync_done();
}


ISR(TCC0_CCB_vect)
{
	LDI::handle_scanline_begin();
}


ISR(TCC0_CCC_vect)
{
	LDI::handle_prepare_laser_for_photodiode_sync();
}


int
main()
{
	LDI::initialize();
	LDI::loop();
}

