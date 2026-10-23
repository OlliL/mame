// license:BSD-3-Clause
// copyright-holders:Oliver Lehmann
/***************************************************************************

    P8000 console terminal

    Emulation of the terminal of the P8000 (firmware ZFT/KEAW 5.0).  The
    firmware runs unchanged on a U882 (Z8) microcontroller, so ADM-31 and
    VT100 mode both come from the firmware itself; the power-on mode is
    ADM-31.

    Hardware, as far as it is known from the firmware sources:
      * U882 (Z8612 with its own ROM bus) at 7.3728 MHz, firmware ROM 4 KB
        (2732) in program memory 0000-0FFF
      * refresh memory (24 x 80 characters + row address table) at
        1000-17FF of the data memory, i8275 CRTC at 1C00/1C01
      * the row addresses are loaded into a 12 bit DMA address counter by
        accessing the refresh memory; the DMA logic feeds the i8275 while
        P35 = 0 and DRQ is active (P31 is the inverted DRQ)
      * ZG0/ZG1/bell/keyboard reset are decoded from A13-A15 of write
        accesses to program memory (4000, 2000, 8000, C000) by a 74138;
        ZG0/ZG1 are a set/reset latch that enables one of two 2716 ROMs
      * IBM XT keyboard (scancode set 1) shifted into port 2, P33 interrupt
      * P30/P37: serial in/out of the Z8, 9600 baud

***************************************************************************/

#include "emu.h"
#include "p8k_term.h"

#include "cpu/z8/z8.h"
#include "sound/beep.h"
#include "video/i8275.h"

#include "screen.h"
#include "speaker.h"

#include <algorithm>
#include <iterator>


namespace {

class p8k_terminal_device : public device_t, public device_rs232_port_interface
{
public:
	p8k_terminal_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void input_txd(int state) override;

	void key_changed(ioport_field &field, uint32_t param, ioport_value oldval, ioport_value newval);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	void mcu_program_map(address_map &map) ATTR_COLD;
	void mcu_data_map(address_map &map) ATTR_COLD;

	uint8_t bram_r(offs_t offset);
	void bram_w(offs_t offset, uint8_t data);
	void zg0_w(uint8_t data) { m_zg1 = false; }
	void zg1_w(uint8_t data) { m_zg1 = true; }
	void bell_w(uint8_t data);
	void kbd_reset_w(uint8_t data);

	uint8_t p2_r() { return m_p2; }
	void p3_w(uint8_t data);
	void drq_w(int state);
	void vrtc_w(int state);
	void dma_service();

	void kbd_queue(uint8_t data);
	void kbd_deliver();

	I8275_DRAW_CHARACTER_MEMBER(display_pixels);
	TIMER_CALLBACK_MEMBER(bell_off);
	TIMER_CALLBACK_MEMBER(kbd_timer);

	required_device<z8_device> m_mcu;
	required_device<i8275_device> m_crtc;
	required_device<beep_device> m_beeper;
	required_region_ptr<uint8_t> m_chargen;
	required_ioport m_conf;

	emu_timer *m_bell_timer;
	emu_timer *m_kbd_timer;

	uint8_t m_kbd_fifo[64];
	uint8_t m_kbd_head;
	uint8_t m_kbd_count;
	uint8_t m_bram[0x800];
	uint16_t m_dma_addr;
	uint8_t m_p2;
	bool m_zg1;
	bool m_dma_enable;
	bool m_drq;
	bool m_kbd_full;
	bool m_kbd_ready;
};

p8k_terminal_device::p8k_terminal_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, P8K_TERMINAL, tag, owner, clock)
	, device_rs232_port_interface(mconfig, *this)
	, m_mcu(*this, "mcu")
	, m_crtc(*this, "crtc")
	, m_beeper(*this, "beeper")
	, m_chargen(*this, "chargen")
	, m_conf(*this, "CONF")
{
}


ROM_START(p8k_terminal)
	// the firmware sits in the region of the Z8, as for the internal ROM of the type
	ROM_REGION(0x1000, "mcu", 0)
	ROM_LOAD("p8t_1_5.0", 0x0000, 0x1000, CRC(1bd93470) SHA1(436a9441971c2c88fe10791807742b7e727bb4ee))

	// 128 characters of 16 bytes each; ZG0 is the ADM-31 ASCII set, ZG1 the one with umlauts
	ROM_REGION(0x1000, "chargen", 0)
	ROM_LOAD("p8tezs", 0x0000, 0x0800, CRC(f9321251) SHA1(a6a796b58d50ec4a416f2accc34bd76bc83f18ea))
	ROM_LOAD("p8tdzs", 0x0800, 0x0800, CRC(32736503) SHA1(6a1d7c55dddc64a7d601dfdbf917ce1afaefbb0a))
ROM_END

const tiny_rom_entry *p8k_terminal_device::device_rom_region() const
{
	return ROM_NAME(p8k_terminal);
}


#define KEY(bit, name, code, param) \
	PORT_BIT(bit, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME(name) PORT_CODE(code) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(p8k_terminal_device::key_changed), param)

// the parameter is the XT scancode (set 1), bit 8 marks the E0 prefix; the firmware knows only
// the left shift key (2A), so the right shift key sends the same code
static INPUT_PORTS_START(p8k_terminal)
	PORT_START("CONF")
	PORT_CONFNAME(0x03, 0x00, "Color")
	PORT_CONFSETTING(   0x00, "Green")
	PORT_CONFSETTING(   0x01, "Amber")
	PORT_CONFSETTING(   0x02, "White")

	PORT_START("KEYS0")
	KEY(0x00000001, "Esc", KEYCODE_ESC, 0x001)
	KEY(0x00000002, "1", KEYCODE_1, 0x002)
	KEY(0x00000004, "2", KEYCODE_2, 0x003)
	KEY(0x00000008, "3", KEYCODE_3, 0x004)
	KEY(0x00000010, "4", KEYCODE_4, 0x005)
	KEY(0x00000020, "5", KEYCODE_5, 0x006)
	KEY(0x00000040, "6", KEYCODE_6, 0x007)
	KEY(0x00000080, "7", KEYCODE_7, 0x008)
	KEY(0x00000100, "8", KEYCODE_8, 0x009)
	KEY(0x00000200, "9", KEYCODE_9, 0x00a)
	KEY(0x00000400, "0", KEYCODE_0, 0x00b)
	KEY(0x00000800, "- (sz)", KEYCODE_MINUS, 0x00c)
	KEY(0x00001000, "= (')", KEYCODE_EQUALS, 0x00d)
	KEY(0x00002000, "Backspace", KEYCODE_BACKSPACE, 0x029)
	KEY(0x00004000, "Tab", KEYCODE_TAB, 0x00f)
	KEY(0x00008000, "Q", KEYCODE_Q, 0x010)
	KEY(0x00010000, "W", KEYCODE_W, 0x011)
	KEY(0x00020000, "E", KEYCODE_E, 0x012)
	KEY(0x00040000, "R", KEYCODE_R, 0x013)
	KEY(0x00080000, "T", KEYCODE_T, 0x014)
	KEY(0x00100000, "Y", KEYCODE_Y, 0x015)
	KEY(0x00200000, "U", KEYCODE_U, 0x016)
	KEY(0x00400000, "I", KEYCODE_I, 0x017)
	KEY(0x00800000, "O", KEYCODE_O, 0x018)
	KEY(0x01000000, "P", KEYCODE_P, 0x019)
	KEY(0x02000000, "[", KEYCODE_OPENBRACE, 0x01a)
	KEY(0x04000000, "]", KEYCODE_CLOSEBRACE, 0x01b)
	KEY(0x08000000, "Enter", KEYCODE_ENTER, 0x01c)
	KEY(0x10000000, "Ctrl", KEYCODE_LCONTROL, 0x01d)
	KEY(0x20000000, "A", KEYCODE_A, 0x01e)
	KEY(0x40000000, "S", KEYCODE_S, 0x01f)
	KEY(0x80000000, "D", KEYCODE_D, 0x020)
	PORT_START("KEYS1")
	KEY(0x00000001, "F", KEYCODE_F, 0x021)
	KEY(0x00000002, "G", KEYCODE_G, 0x022)
	KEY(0x00000004, "H", KEYCODE_H, 0x023)
	KEY(0x00000008, "J", KEYCODE_J, 0x024)
	KEY(0x00000010, "K", KEYCODE_K, 0x025)
	KEY(0x00000020, "L", KEYCODE_L, 0x026)
	KEY(0x00000040, ";", KEYCODE_COLON, 0x027)
	KEY(0x00000080, "'", KEYCODE_QUOTE, 0x028)
	KEY(0x00000100, "`", KEYCODE_TILDE, 0x00e)
	KEY(0x00000200, "Left Shift", KEYCODE_LSHIFT, 0x02a)
	KEY(0x00000400, "\\", KEYCODE_BACKSLASH, 0x02b)
	KEY(0x00000800, "Z", KEYCODE_Z, 0x02c)
	KEY(0x00001000, "X", KEYCODE_X, 0x02d)
	KEY(0x00002000, "C", KEYCODE_C, 0x02e)
	KEY(0x00004000, "V", KEYCODE_V, 0x02f)
	KEY(0x00008000, "B", KEYCODE_B, 0x030)
	KEY(0x00010000, "N", KEYCODE_N, 0x031)
	KEY(0x00020000, "M", KEYCODE_M, 0x032)
	KEY(0x00040000, ",", KEYCODE_COMMA, 0x033)
	KEY(0x00080000, ".", KEYCODE_STOP, 0x034)
	KEY(0x00100000, "/", KEYCODE_SLASH, 0x035)
	KEY(0x00200000, "Right Shift", KEYCODE_RSHIFT, 0x02a)
	KEY(0x00400000, "Keypad *", KEYCODE_ASTERISK, 0x037)
	KEY(0x00800000, "Alt", KEYCODE_LALT, 0x038)
	KEY(0x01000000, "Space", KEYCODE_SPACE, 0x039)
	KEY(0x02000000, "Caps Lock", KEYCODE_CAPSLOCK, 0x03a)
	KEY(0x04000000, "F1", KEYCODE_F1, 0x03b)
	KEY(0x08000000, "F2", KEYCODE_F2, 0x03c)
	KEY(0x10000000, "F3", KEYCODE_F3, 0x03d)
	KEY(0x20000000, "F4", KEYCODE_F4, 0x03e)
	KEY(0x40000000, "F5", KEYCODE_F5, 0x03f)
	KEY(0x80000000, "F6", KEYCODE_F6, 0x040)
	PORT_START("KEYS2")
	KEY(0x00000001, "F7", KEYCODE_F7, 0x041)
	KEY(0x00000002, "F8", KEYCODE_F8, 0x042)
	KEY(0x00000004, "F9", KEYCODE_F9, 0x043)
	KEY(0x00000008, "F10", KEYCODE_F10, 0x044)
	KEY(0x00000010, "Num Lock", KEYCODE_NUMLOCK, 0x045)
	KEY(0x00000020, "Scroll Lock", KEYCODE_SCRLOCK, 0x046)
	KEY(0x00000040, "Keypad 7", KEYCODE_7_PAD, 0x047)
	KEY(0x00000080, "Keypad 8", KEYCODE_8_PAD, 0x048)
	KEY(0x00000100, "Keypad 9", KEYCODE_9_PAD, 0x049)
	KEY(0x00000200, "Keypad -", KEYCODE_MINUS_PAD, 0x04a)
	KEY(0x00000400, "Keypad 4", KEYCODE_4_PAD, 0x04b)
	KEY(0x00000800, "Keypad 5", KEYCODE_5_PAD, 0x04c)
	KEY(0x00001000, "Keypad 6", KEYCODE_6_PAD, 0x04d)
	KEY(0x00002000, "Keypad +", KEYCODE_PLUS_PAD, 0x04e)
	KEY(0x00004000, "Keypad 1", KEYCODE_1_PAD, 0x04f)
	KEY(0x00008000, "Keypad 2", KEYCODE_2_PAD, 0x050)
	KEY(0x00010000, "Keypad 3", KEYCODE_3_PAD, 0x051)
	KEY(0x00020000, "Keypad 0", KEYCODE_0_PAD, 0x052)
	KEY(0x00040000, "Keypad .", KEYCODE_DEL_PAD, 0x053)
	KEY(0x00080000, "< >", KEYCODE_BACKSLASH2, 0x056)
	KEY(0x00100000, "F11", KEYCODE_F11, 0x057)
	KEY(0x00200000, "F12", KEYCODE_F12, 0x058)
	KEY(0x00400000, "Keypad Enter", KEYCODE_ENTER_PAD, 0x11c)
	KEY(0x00800000, "Keypad /", KEYCODE_SLASH_PAD, 0x135)
	KEY(0x01000000, "Home", KEYCODE_HOME, 0x147)
	KEY(0x02000000, "Up", KEYCODE_UP, 0x148)
	KEY(0x04000000, "Page Up", KEYCODE_PGUP, 0x149)
	KEY(0x08000000, "Left", KEYCODE_LEFT, 0x14b)
	KEY(0x10000000, "Right", KEYCODE_RIGHT, 0x14d)
	KEY(0x20000000, "End", KEYCODE_END, 0x14f)
	KEY(0x40000000, "Down", KEYCODE_DOWN, 0x150)
	KEY(0x80000000, "Page Down", KEYCODE_PGDN, 0x151)
	PORT_START("KEYS3")
	KEY(0x00000001, "Insert", KEYCODE_INSERT, 0x152)
	KEY(0x00000002, "Delete", KEYCODE_DEL, 0x153)
INPUT_PORTS_END

#undef KEY

ioport_constructor p8k_terminal_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(p8k_terminal);
}


void p8k_terminal_device::mcu_program_map(address_map &map)
{
	map(0x0000, 0x0fff).rom().region("mcu", 0);
	map(0x2000, 0x3fff).w(FUNC(p8k_terminal_device::zg1_w));
	map(0x4000, 0x5fff).w(FUNC(p8k_terminal_device::zg0_w));
	map(0x8000, 0x9fff).w(FUNC(p8k_terminal_device::bell_w));
	map(0xc000, 0xdfff).w(FUNC(p8k_terminal_device::kbd_reset_w));
}

void p8k_terminal_device::mcu_data_map(address_map &map)
{
	map(0x1000, 0x17ff).rw(FUNC(p8k_terminal_device::bram_r), FUNC(p8k_terminal_device::bram_w));
	map(0x1c00, 0x1c01).rw(m_crtc, FUNC(i8275_device::read), FUNC(i8275_device::write));
}

void p8k_terminal_device::device_add_mconfig(machine_config &config)
{
	UB8830D(config, m_mcu, 7.3728_MHz_XTAL);
	m_mcu->set_addrmap(AS_PROGRAM, &p8k_terminal_device::mcu_program_map);
	m_mcu->set_addrmap(AS_DATA, &p8k_terminal_device::mcu_data_map);
	m_mcu->p2_in_cb().set(FUNC(p8k_terminal_device::p2_r));
	m_mcu->p3_out_cb().set(FUNC(p8k_terminal_device::p3_w));

	screen_device &screen(SCREEN(config, "screen"));
	screen.set_raw(12_MHz_XTAL, 102 * 8, 0, 80 * 8, 338, 0, 24 * 13);
	screen.set_screen_update(m_crtc, FUNC(i8275_device::screen_update));

	I8275(config, m_crtc, 12_MHz_XTAL / 8);
	m_crtc->set_character_width(8);
	m_crtc->set_display_callback(FUNC(p8k_terminal_device::display_pixels));
	m_crtc->drq_wr_callback().set(FUNC(p8k_terminal_device::drq_w));
	m_crtc->vrtc_wr_callback().set(FUNC(p8k_terminal_device::vrtc_w));
	m_crtc->set_screen("screen");

	SPEAKER(config, "bell").front_center();
	BEEP(config, m_beeper, 2'000);
	m_beeper->add_route(ALL_OUTPUTS, "bell", 0.25);
}

void p8k_terminal_device::device_start()
{
	m_bell_timer = timer_alloc(FUNC(p8k_terminal_device::bell_off), this);
	m_kbd_timer = timer_alloc(FUNC(p8k_terminal_device::kbd_timer), this);

	std::fill(std::begin(m_bram), std::end(m_bram), 0);
	m_dma_addr = 0;

	save_item(NAME(m_kbd_fifo));
	save_item(NAME(m_kbd_head));
	save_item(NAME(m_kbd_count));
	save_item(NAME(m_bram));
	save_item(NAME(m_dma_addr));
	save_item(NAME(m_p2));
	save_item(NAME(m_zg1));
	save_item(NAME(m_dma_enable));
	save_item(NAME(m_drq));
	save_item(NAME(m_kbd_full));
	save_item(NAME(m_kbd_ready));
}

void p8k_terminal_device::device_reset()
{
	m_zg1 = false;
	m_dma_enable = false;
	m_drq = false;
	m_kbd_full = false;
	m_kbd_ready = false;
	m_p2 = 0;
	m_beeper->set_state(0);

	output_rxd(1);
	output_dcd(0);
	output_dsr(0);
	output_ri(0);
	output_cts(0);

	// the keyboard reports its self test result after power-on
	m_kbd_head = 0;
	m_kbd_count = 0;
	kbd_queue(0xaa);
	m_kbd_timer->adjust(attotime::from_msec(100));
}


uint8_t p8k_terminal_device::bram_r(offs_t offset)
{
	// the refresh memory is addressed through the DMA address counter, so every access loads it
	if (!machine().side_effects_disabled())
		m_dma_addr = offset;
	return m_bram[offset];
}

void p8k_terminal_device::bram_w(offs_t offset, uint8_t data)
{
	if (!machine().side_effects_disabled())
		m_dma_addr = offset;
	m_bram[offset] = data;
}

void p8k_terminal_device::bell_w(uint8_t data)
{
	m_beeper->set_state(1);
	m_bell_timer->adjust(attotime::from_msec(150));
}

TIMER_CALLBACK_MEMBER(p8k_terminal_device::bell_off)
{
	m_beeper->set_state(0);
}


void p8k_terminal_device::input_txd(int state)
{
	// P30 is the serial input of the Z8, an asserted line is a low level
	m_mcu->set_input_line(INPUT_LINE_IRQ3, state ? CLEAR_LINE : ASSERT_LINE);
}

void p8k_terminal_device::p3_w(uint8_t data)
{
	output_rxd(BIT(data, 7));

	bool const dma_enable = !BIT(data, 5);
	if (dma_enable != m_dma_enable)
	{
		m_dma_enable = dma_enable;
		if (m_dma_enable && m_drq)
			dma_service();
	}
}

void p8k_terminal_device::drq_w(int state)
{
	m_drq = bool(state);
	if (m_drq)
	{
		// every request is also an interrupt of the Z8 (P31)
		m_mcu->pulse_input_line(INPUT_LINE_IRQ2, m_mcu->minimum_quantum_time());
		if (m_dma_enable)
			dma_service();
	}
}

void p8k_terminal_device::dma_service()
{
	m_crtc->dack_w(m_bram[m_dma_addr++ & 0x7ff]);
}

void p8k_terminal_device::vrtc_w(int state)
{
	if (state)
		m_mcu->pulse_input_line(INPUT_LINE_IRQ0, m_mcu->minimum_quantum_time());
}


void p8k_terminal_device::kbd_queue(uint8_t data)
{
	if (m_kbd_count < std::size(m_kbd_fifo))
		m_kbd_fifo[(m_kbd_head + m_kbd_count++) % std::size(m_kbd_fifo)] = data;
}

void p8k_terminal_device::kbd_deliver()
{
	if (m_kbd_full || m_kbd_count == 0)
		return;

	// the shift register holds the scancode rotated left by one bit
	uint8_t const data = m_kbd_fifo[m_kbd_head];
	m_kbd_head = (m_kbd_head + 1) % std::size(m_kbd_fifo);
	m_kbd_count--;
	m_p2 = uint8_t((data << 1) | (data >> 7));
	m_kbd_full = true;
	m_mcu->pulse_input_line(INPUT_LINE_IRQ1, m_mcu->minimum_quantum_time());
}

void p8k_terminal_device::kbd_reset_w(uint8_t data)
{
	m_kbd_full = false;
	m_p2 = 0;
	if (m_kbd_ready && m_kbd_count != 0)
		m_kbd_timer->adjust(attotime::from_msec(1));
}

TIMER_CALLBACK_MEMBER(p8k_terminal_device::kbd_timer)
{
	m_kbd_ready = true;
	kbd_deliver();
}

void p8k_terminal_device::key_changed(ioport_field &field, uint32_t param, ioport_value oldval, ioport_value newval)
{
	bool const pressed = newval != 0;
	if (BIT(param, 8))
		kbd_queue(0xe0);
	kbd_queue((param & 0x7f) | (pressed ? 0x00 : 0x80));

	if (m_kbd_ready && !m_kbd_full && !m_kbd_timer->enabled())
		m_kbd_timer->adjust(attotime::from_msec(1));
}


I8275_DRAW_CHARACTER_MEMBER(p8k_terminal_device::display_pixels)
{
	using namespace i8275_attributes;

	uint8_t attr = attrcode & 0xff;
	uint8_t bits = m_chargen[(m_zg1 ? 0x800 : 0x000) | ((charcode & 0x7f) << 4) | (linecount & 0x0f)];

	if (BIT(attr, VSP))
		bits = 0x00;
	else if (BIT(attr, RVV))
		bits = ~bits;

	// the underline is on the last scan line, which the 8275 also suppresses
	if (BIT(attr, LTEN))
		bits = 0xff;

	static constexpr rgb_t colors[3][2] = {
		{ rgb_t(0x00, 0xa0, 0x00), rgb_t(0x40, 0xff, 0x40) },
		{ rgb_t(0xc0, 0x80, 0x00), rgb_t(0xff, 0xc0, 0x20) },
		{ rgb_t(0xa0, 0xa0, 0xa0), rgb_t(0xff, 0xff, 0xff) },
	};
	rgb_t const fg = colors[std::min<unsigned>(m_conf->read() & 0x03, 2)][BIT(attr, HLGT)];

	uint32_t *const dest = &bitmap.pix(y, x);
	for (int i = 0; i < 8; i++)
		dest[i] = BIT(bits, 7 - i) ? fg : rgb_t::black();
}

} // anonymous namespace


DEFINE_DEVICE_TYPE_PRIVATE(P8K_TERMINAL, device_rs232_port_interface, p8k_terminal_device, "p8k_terminal", "P8000 Terminal")
