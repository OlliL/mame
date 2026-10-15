// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***************************************************************************

    P8000

    09/2009 Skeleton driver based on Matt Knoth's emulator.
    P8000emu (http://knothusa.net/Home.php) has been a great source of info.
    Other info from
      * http://www.pofo.de/P8000/notes/books/
      * http://www.pofo.de/P8000/

    P8000 memory layout
      * divided into 3 banks of 64k
      * bank A is for roms, only 0000-1FFF is populated
      * bank B is for static ram, only 2000-2FFF exists
      * bank C is for dynamic ram, all 64k is available.
      * selection is done with OUT(c), code
      * code = 0 for do nothing; 1 = bank A; 2 = bank B; 4 = bank C.
      * Reg C = 0; Reg B = start address of memory that is being switched,
        for example B = 20 indicates "bank2" in memory map, and also the
        corresponding address in bank A/B/C.

    P8000 monitor commands
      * B : ?
      * D : display and modify memory
      * F : fill memory
      * G : go to
      * M : move (copy) memory
      * N : dump registers
      * O : boot from floppy
      * P : ?
      * Q : ?
      * R : dump registers
      * S : boot from floppy
      * T : jump to ROM at CEF0
      * X : jump to ROM at DB00
      * return : boot from floppy disk

    P8000_16 : All input must be in uppercase.

    TODO:
      * Find out how to enter hardware check on 16 bit board

****************************************************************************/

#include "emu.h"
#include "cpu/z80/z80.h"
#include "machine/z80daisy.h"
#include "cpu/z8000/z8000.h"
#include "imagedev/floppy.h"
#include "machine/upd765.h"
#include "machine/z80ctc.h"
#include "machine/z80sio.h"
#include "machine/z80dma.h"
#include "imagedev/harddriv.h"
#include "machine/z80pio.h"
#include "machine/z8010.h"
#include "sound/beep.h"
#include "speaker.h"
#include "machine/clock.h"
#include "bus/rs232/rs232.h"

#include "p8k_u130.h"
#include "p8k_wdc.h"

//#define VERBOSE 1
#include "logmacro.h"

class p8k_16_daisy_device : public device_t, public z80_daisy_chain_interface
{
public:
	p8k_16_daisy_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);
	uint16_t viack_r() {
		device_z80daisy_interface *intf = daisy_get_irq_device();
		return intf ? intf->z80daisy_irq_ack() : 0;
	}
	int update_irq_state() { return daisy_update_irq_state(); }
	auto int_cb() { return m_int_cb.bind(); }
	void reti_w(uint8_t data)
	{
		if (data == 0x4d)
		{
			// writes of ED,4D emulate Z80 RETI, which changes the daisy chain
			// state and may expose a lower priority request: update the VI line
			daisy_call_reti_device();
			m_int_cb(CLEAR_LINE);
		}
	}
protected:
	void device_start() override {}
private:
	devcb_write_line m_int_cb;
};

DEFINE_DEVICE_TYPE(P8K_16_DAISY, p8k_16_daisy_device, "p8k_16_daisy", "P8000 16-bit daisy chain device")

p8k_16_daisy_device::p8k_16_daisy_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, P8K_16_DAISY, tag, owner, clock)
	, z80_daisy_chain_interface(mconfig, *this)
	, m_int_cb(*this) {}


// The DMA controller of the 8 bit board reads back one less than the number of
// bytes transferred (needed by the monitor's hardware self-test).
class p8k_dma_device : public ua858d_device
{
public:
	p8k_dma_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

protected:
	virtual u16 byte_counter() const override { return m_byte_counter - 1; }
};

DEFINE_DEVICE_TYPE(P8K_DMA, p8k_dma_device, "p8k_dma", "P8000 DMA controller")

p8k_dma_device::p8k_dma_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ua858d_device(mconfig, P8K_DMA, tag, owner, clock)
{
}

namespace {

class p8k_state : public driver_device
{
public:
	p8k_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_cpu16(*this, "cpu16")
		, m_daisy(*this, "p8k_16_daisy")
		, m_pio0(*this, "pio0")
		, m_pio2(*this, "pio2")
		, m_ctc(*this, "u_ctc%u", 0U)
		, m_sio(*this, "u_sio%u", 0U)
		, m_u_pio0(*this, "u_pio0")
		, m_u_pio1(*this, "u_pio1")
		, m_u_pio2(*this, "u_pio2")
		, m_wdc(*this, "wdc")
		, m_u130(*this, "u130")
		, m_i8272(*this, "i8272")
		, m_ram(*this, "ram", 0x100000, ENDIANNESS_BIG)
		, m_local_ram(*this, "local_ram", 0x4000, ENDIANNESS_BIG)
		, m_view_code(*this, "view_code")
		, m_view_data(*this, "view_data")
		, m_view_stck(*this, "view_stck")
		, m_mmu_code(*this, "mmu_code")
		, m_mmu_data(*this, "mmu_data")
		, m_mmu_stck(*this, "mmu_stck")
	{ }

	void p8k(machine_config &config);
	void p8k_16(machine_config &config);

	void init_p8k();

	DECLARE_INPUT_CHANGED_MEMBER(nmi_button);

protected:
	virtual void machine_start() override ATTR_COLD;

private:
	uint8_t port0_r(offs_t offset);
	void port0_w(offs_t offset, uint8_t data);
	DECLARE_MACHINE_RESET(p8k);

	[[maybe_unused]] void fdc_irq(int state);
	void p8k_daisy_interrupt(int state);
	void p8k_dma_irq_w(int state);
	void p8k_16_daisy_interrupt(int state);
	uint8_t pio2_r(offs_t offset);
	void pio2_w(offs_t offset, uint8_t data);

	// coupling of the two boards
	uint8_t pio0_r(offs_t offset);
	void pio0_w(offs_t offset, uint8_t data);
	uint8_t u_pio0_r(offs_t offset);
	void u_pio0_w(offs_t offset, uint8_t data);
	void p16a_w(uint8_t data);
	void p16b_w(uint8_t data);
	uint8_t coupling_pio0_a_r();
	uint8_t coupling_pio0_b_r();
	uint8_t coupling_u_pio0_b_r();
	void coupling_u_pio0_b_w(uint8_t data);
	uint8_t coupling_u_pio1_a_r();
	uint8_t coupling_u_pio1_b_r();
	void coupling_ardy0_w(int state) { coupling_ardy0_set(state, false); }
	void coupling_ardy0_set(int state, bool data_written);
	void coupling_u_pio0_a_w(uint8_t data);   // 16 bit PIO0 A ready: data for the 8 bit board
	void coupling_ardy1_w(int state);   // 16 bit PIO1 A ready: 16 bit side can take data
	void coupling_ardy8_w(int state);   // 8 bit PIO0 A ready: 8 bit side took the data
	void coupling_update_8bit();
	void u8000_reset(bool held);
	uint8_t memory_read_byte(offs_t offset);
	void memory_write_byte(offs_t offset, uint8_t data);
	uint8_t io_read_byte(offs_t offset);
	void io_write_byte(offs_t offset, uint8_t data);

	// 16-bit board, a copy of the System 8000 CPU-A board (see s8k_cpu.h)
	DECLARE_MACHINE_RESET(p8k_16);

	void addrmap_program(address_map &map) ATTR_COLD { map.unmap_value_low(); }
	void addrmap_data(address_map &map) ATTR_COLD { map.unmap_value_low(); }
	void addrmap_stack(address_map &map) ATTR_COLD { map.unmap_value_low(); }
	void addrmap_io(address_map &map) ATTR_COLD;
	void addrmap_sio(address_map &map) ATTR_COLD;

	z8010_device *select_code_mmu(offs_t offset);
	z8010_device *select_data_mmu(offs_t offset, uint8_t nbr);

	bool translate_addr(int spacenum, bool write, offs_t &offset);
	uint16_t ram16_r(offs_t offset, uint16_t mask);
	void ram16_w(offs_t offset, uint16_t data, uint16_t mask);
	uint16_t ram_r(address_space &space, offs_t offset, uint16_t mask);
	void ram_w(address_space &space, offs_t offset, uint16_t data, uint16_t mask);

	uint16_t nmiack_r();
	uint16_t segtack_r();
	uint16_t viack_r();
	uint16_t nviack_r();
	void mmu_instruction_end();

	void segt_interrupt(int state);

	void soft_reset_w(uint8_t dummy);

	void install_memory(offs_t lrom_end, offs_t lram_start, offs_t lmem_end);

	uint8_t reg_trpl_r();
	uint8_t reg_if1l_r();

	void observe_bus_cycle(offs_t offset, bool if1);

	//helpers
	void out_ns_cb(int state);

	uint8_t reg_scr_r();
	void reg_scr_w(uint8_t data);
	uint8_t reg_sbr_r();
	void reg_sbr_w(uint8_t data);
	uint8_t reg_nbr_r();
	void reg_nbr_w(uint8_t data);

	uint8_t mmu_cmd_r(offs_t offset);
	void mmu_cmd_w(offs_t offset, uint8_t data);

	void board16_reset();

	void p8k_iomap(address_map &map) ATTR_COLD;
	void p8k_memmap(address_map &map) ATTR_COLD;

	required_device<z80_device> m_maincpu;        // 8 bit board
	optional_device<z8001_device> m_cpu16;        // 16 bit board
	optional_device<p8k_16_daisy_device> m_daisy;
	required_device<z80pio_device> m_pio0;        // 8 bit board: coupling PIO
	optional_device<z80pio_device> m_pio2;        // 8 bit board
	optional_device_array<z80ctc_device, 2> m_ctc;
	optional_device_array<z80sio_device, 2> m_sio;
	optional_device<z80pio_device> m_u_pio0;      // 16 bit board: coupling PIO (to 8 bit)
	optional_device<z80pio_device> m_u_pio1;      // 16 bit board: coupling PIO (from 8 bit)
	optional_device<z80pio_device> m_u_pio2;      // 16 bit board: WDC
	optional_device<p8k_wdc_device> m_wdc;
	optional_device<p8k_u130_device> m_u130;
	optional_device<i8272a_device> m_i8272;

	// 16-bit board
	memory_share_creator<uint16_t> m_ram;         // external memory, 1 MiB
	memory_share_creator<uint16_t> m_local_ram;   // on-board RAM at 4000-7FFF
	memory_view m_view_code;
	memory_view m_view_data;
	memory_view m_view_stck;
	optional_device<z8010_device> m_mmu_code;
	optional_device<z8010_device> m_mmu_data;
	optional_device<z8010_device> m_mmu_stck;
	// coupling state
	uint8_t m_p16a = 0;               // 8 -> 16 bit data latch
	uint8_t m_p16b = 0;               // 8 -> 16 bit status latch (bit 7 = STB, bit 0 = INT16)
	bool m_ardy0 = false;
	bool m_ardy1 = false;
	bool m_u_pio0_writing = false;
	uint8_t m_u_pio0_a_bus = 0;
	bool m_u8000_held = true;

	// Board registers
	uint8_t m_reg_scr   = 0xf0; // System Configuration Register (bits 4-7: switches)
	uint8_t m_reg_sbr   = 0; // System Break Register
	uint8_t m_reg_nbr   = 0; // Normal Break Register
	uint8_t m_reg_trpl  = 0; // Segment trap memory address low-byte
	uint8_t m_reg_if1l  = 0; // Segment trap instruction low-byte

	// Running bus-side latches and the SEGT- line state
	uint8_t m_lad_low   = 0; // low address byte of the current memory cycle
	uint8_t m_if1_low   = 0; // low address byte of the last IFETCH1 cycle
	bool m_segt_state   = false;

	bool m_is_seg_os = true;    // the P8000 operating system is always segmented
	bool m_is_seg_user = false;

	int m_normal_mode = 0;
	int m_dma_on = 0;
};

/***************************************************************************

    P8000 8bit

****************************************************************************/

void p8k_state::p8k_memmap(address_map &map)
{
	map(0x0000, 0x0FFF).bankrw("bank0");
	map(0x1000, 0x1FFF).bankrw("bank1");
	map(0x2000, 0x2FFF).bankrw("bank2");
	map(0x3000, 0x3FFF).bankrw("bank3");
	map(0x4000, 0x4FFF).bankrw("bank4");
	map(0x5000, 0x5FFF).bankrw("bank5");
	map(0x6000, 0x6FFF).bankrw("bank6");
	map(0x7000, 0x7FFF).bankrw("bank7");
	map(0x8000, 0x8FFF).bankrw("bank8");
	map(0x9000, 0x9FFF).bankrw("bank9");
	map(0xA000, 0xAFFF).bankrw("bank10");
	map(0xB000, 0xBFFF).bankrw("bank11");
	map(0xC000, 0xCFFF).bankrw("bank12");
	map(0xD000, 0xDFFF).bankrw("bank13");
	map(0xE000, 0xEFFF).bankrw("bank14");
	map(0xF000, 0xFFFF).bankrw("bank15");
}

void p8k_state::p8k_iomap(address_map &map)
{
	map.global_mask(0xff);
	map(0x00, 0x07).rw(FUNC(p8k_state::port0_r), FUNC(p8k_state::port0_w)); // MH7489
	map(0x08, 0x0b).rw("ctc0", FUNC(z80ctc_device::read), FUNC(z80ctc_device::write));
	map(0x0c, 0x0f).rw(FUNC(p8k_state::pio0_r), FUNC(p8k_state::pio0_w));
	map(0x10, 0x10).w(FUNC(p8k_state::p16a_w));   // data to the 16 bit board
	map(0x14, 0x14).w(FUNC(p8k_state::p16b_w));   // status/strobe/INT to the 16 bit board
	map(0x18, 0x1b).rw("pio1", FUNC(z80pio_device::read_alt), FUNC(z80pio_device::write_alt));
	map(0x1c, 0x1f).rw(m_pio2, FUNC(z80pio_device::read_alt), FUNC(z80pio_device::write_alt));
	map(0x20, 0x21).m(m_i8272, FUNC(i8272a_device::map));
	map(0x24, 0x27).rw("sio", FUNC(z80sio_device::ba_cd_r), FUNC(z80sio_device::ba_cd_w));
	map(0x28, 0x2b).rw("sio1", FUNC(z80sio_device::ba_cd_r), FUNC(z80sio_device::ba_cd_w));
	map(0x2c, 0x2f).rw("ctc1", FUNC(z80ctc_device::read), FUNC(z80ctc_device::write));
	map(0x3c, 0x3c).rw("dma", FUNC(z80dma_device::read), FUNC(z80dma_device::write));
}



uint8_t p8k_state::port0_r(offs_t offset)
{
	return 0;
}

// see memory explanation above
void p8k_state::port0_w(offs_t offset, uint8_t data)
{
	uint8_t breg = m_maincpu->state_int(Z80_B) >> 4;
	if ((data==1) || (data==2) || (data==4))
	{
		char banknum[8];
		sprintf(banknum,"bank%d", breg);

		offset = 0;
		if (data==2)
			offset = 16;
		else
		if (data==4)
			offset = 32;

		offset += breg;

		membank(banknum)->set_entry(offset);
	}
	else
	if (data)
		printf("Invalid data %X for bank %d\n",data,breg);
}


/***************************************************************************

    P8000 8bit Peripherals

****************************************************************************/

void p8k_state::p8k_daisy_interrupt(int state)
{
	m_maincpu->set_input_line(0, state);
}

/* Z80 DMA */

void p8k_state::p8k_dma_irq_w(int state)
{
	m_i8272->tc_w(state);
	p8k_daisy_interrupt(state);
}

uint8_t p8k_state::memory_read_byte(offs_t offset)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM);
	return prog_space.read_byte(offset);
}

void p8k_state::memory_write_byte(offs_t offset, uint8_t data)
{
	address_space& prog_space = m_maincpu->space(AS_PROGRAM);
	prog_space.write_byte(offset, data);
}

uint8_t p8k_state::io_read_byte(offs_t offset)
{
	address_space& prog_space = m_maincpu->space(AS_IO);
	return prog_space.read_byte(offset);
}

void p8k_state::io_write_byte(offs_t offset, uint8_t data)
{
	address_space& prog_space = m_maincpu->space(AS_IO);
	prog_space.write_byte(offset, data);
}


/* Z80 Daisy Chain */

static const z80_daisy_config p8k_daisy_chain[] =
{
	{ "dma" },   /* FDC related */
	{ "pio2" },
	{ "ctc0" },
	{ "sio" },
	{ "sio1" },
	{ "pio0" },
	{ "pio1" },
	{ "ctc1" },
	{ nullptr }
};

/* Intel 8272 Interface */

void p8k_state::fdc_irq(int state)
{
	m_pio2->port_b_write(state ? 0x10 : 0x00);
}

static void p8k_floppies(device_slot_interface &device)
{
	device.option_add("525hd", FLOPPY_525_HD);
}

/* Input ports */
static INPUT_PORTS_START( p8k )
	PORT_START("DSW")
	PORT_BIT( 0x7f, 0x7f, IPT_UNUSED )
	PORT_DIPNAME( 0x80, 0x00, "Hardware Test")
	PORT_DIPSETTING(    0x00, DEF_STR(Off))
	PORT_DIPSETTING(    0x80, DEF_STR(On))

	PORT_START("NMI")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("NMI") PORT_CODE(KEYCODE_F12) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(p8k_state::nmi_button), 0)
INPUT_PORTS_END

INPUT_CHANGED_MEMBER(p8k_state::nmi_button)
{
	// with a 16 bit board the NMI key goes to the board that is currently released
	// (PIO0 B7 of the 8 bit board = /NMI_UM)
	if (m_cpu16 && !m_u8000_held)
		m_cpu16->set_input_line(INPUT_LINE_NMI, newval ? ASSERT_LINE : CLEAR_LINE);
	else
		m_maincpu->set_input_line(INPUT_LINE_NMI, newval ? ASSERT_LINE : CLEAR_LINE);
}


MACHINE_RESET_MEMBER(p8k_state,p8k)
{
	if (m_cpu16)
		board16_reset();

	membank("bank0")->set_entry(0);
	membank("bank1")->set_entry(0);
	membank("bank2")->set_entry(0);
	membank("bank3")->set_entry(0);
	membank("bank4")->set_entry(0);
	membank("bank5")->set_entry(0);
	membank("bank6")->set_entry(0);
	membank("bank7")->set_entry(0);
	membank("bank8")->set_entry(0);
	membank("bank9")->set_entry(0);
	membank("bank10")->set_entry(0);
	membank("bank11")->set_entry(0);
	membank("bank12")->set_entry(0);
	membank("bank13")->set_entry(0);
	membank("bank14")->set_entry(0);
	membank("bank15")->set_entry(0);
}

void p8k_state::init_p8k()
{
	uint8_t *RAM = memregion("maincpu")->base();

	// the two 4K monitor ROMs are not fully decoded and appear repeatedly in bank A
	// (the monitor copies the coupling code from 0DB00h)
	for (int i = 2; i < 16; i++)
		memcpy(&RAM[i * 0x1000], &RAM[(i & 1) * 0x1000], 0x1000);

	membank("bank0")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank1")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank2")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank3")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank4")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank5")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank6")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank7")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank8")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank9")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank10")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank11")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank12")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank13")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank14")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
	membank("bank15")->configure_entries(0, 48, &RAM[0x0000], 0x1000);
}


/***************************************************************************

    P8000 16bit

    The 16 bit board is a copy of the System 8000 CPU-A board
    (src/devices/bus/zbi/s8k_cpu.cpp) with minimal changes: there is no ZBI
    bus, the memory sits on the board, and the peripherals are two SIOs,
    two CTCs and three PIOs. The names and the order of the functions follow
    that file.

***************************************************************************/

#define CLK_CPU     XTAL(4'000'000)
#define CLK_CTC     (2.4576_MHz_XTAL / 2)   // oscillator of the real-time clock
#define CLK_BAUD    (2.4576_MHz_XTAL / 4)   // baud rate oscillator: ZC pulses of the CTC / 32 = 19200 / time constant

// System Configuration Register control bits 0-3
#define SCR_BD_MEMON    0x01    // Use off-board memory (disable monitor ROM and local RAM)
#define SCR_MMU_ONH     0x02    // Enable MMUs (disable linear memory)
#define SCR_SEG_USER    0x04    // Segmented user space
#define SCR_CLR_PARITY  0x08    // Enable parity check (not emulated)

void p8k_state::machine_start()
{
	if (m_mmu_code)
	{
		install_memory(0x3fff, 0x4000, 0x7fff);

		save_item(NAME(m_reg_scr));
		save_item(NAME(m_reg_sbr));
		save_item(NAME(m_reg_nbr));
		save_item(NAME(m_reg_trpl));
		save_item(NAME(m_reg_if1l));
		save_item(NAME(m_lad_low));
		save_item(NAME(m_if1_low));
		save_item(NAME(m_segt_state));
		save_item(NAME(m_normal_mode));
	}
}

//**************************************************************************
//  INTERRUPT HANDLING
//**************************************************************************

void p8k_state::segt_interrupt(int state)
{
	if (state && !m_segt_state)
	{
		// The address low bytes -- which the Z8010s cannot record --
		// are captured at the moment the segment trap is raised.  A trap
		// handler combines them with the high bytes latched inside the MMUs.
		m_reg_trpl = m_lad_low;
		m_reg_if1l = m_if1_low;
	}
	m_segt_state = (state != 0);
	m_cpu16->set_input_line(z8001_device::SEGT_LINE, state ? ASSERT_LINE : CLEAR_LINE);
}

void p8k_state::observe_bus_cycle(offs_t offset, bool if1)
{
	m_lad_low = uint8_t(offset);
	if (if1)
	{
		m_if1_low = uint8_t(offset);
		// Every MMU sits on the CPU's address/data bus and snoops each
		// IFETCH1 cycle, whether or not it translates it.
		m_mmu_code->ifetch1_observed(offset);
		m_mmu_data->ifetch1_observed(offset);
		m_mmu_stck->ifetch1_observed(offset);
	}
}

uint16_t p8k_state::segtack_r()
{
	uint16_t code = (m_mmu_code->segtack_r() |
					 m_mmu_data->segtack_r() |
					 m_mmu_stck->segtack_r());

	// Locally triggered?
	if (code == 0)
	{
		segt_interrupt(0);
	}

	LOG("%s SEGTRAP: %d\n", machine().describe_context(), code);

	return code;
}

void p8k_state::mmu_instruction_end()
{
	m_mmu_code->instruction_end();
	m_mmu_data->instruction_end();
	m_mmu_stck->instruction_end();
}

uint16_t p8k_state::viack_r()
{
	mmu_instruction_end();
	uint16_t vec = m_daisy->viack_r();
	// The ack changed the acked device's daisy state (INT -> IEO), so the VI
	// request line must be re-evaluated
	p8k_16_daisy_interrupt(CLEAR_LINE);
	return vec;
}

uint16_t p8k_state::nviack_r()
{
	mmu_instruction_end();
	return 0xffff;
}

uint16_t p8k_state::nmiack_r()
{
	mmu_instruction_end();
	return 0xffff;
}

void p8k_state::soft_reset_w(uint8_t dummy)
{
	// resets the 16 bit board only, the 8 bit board keeps running
	if (m_cpu16)
	{
		board16_reset();
		u8000_reset(false);
	}
}

/*
    Memory access of the 16-bit board.

    The on-board ROM (0000-3FFF) and RAM (4000-7FFF) of segment 0 are decoded
    on the CPU bus (switched by SCR bit 0).  Everything else goes to the memory
    bus, whose address is either the CPU address or, with SCR bit 1 set, the
    output of one of the three Z8010 MMUs.  The selection of the MMU follows
    the "special logic" described in the hardware manual (index 4 board):

      system mode               : code MMU for everything
      normal mode, SEG USR = 1  : segments 0-63 -> data MMU, 64-127 -> stack MMU
      normal mode, SEG USR = 0  : code fetch -> code MMU, data/stack split by
                                  comparing address bits 8-15 with the NBR
*/

z8010_device *p8k_state::select_code_mmu(offs_t offset)
{
	z8010_device *mmu = m_mmu_code;
	uint8_t seg = (uint8_t)(offset >> 16);

	LOG("%s CODE MMU SELECT, offset: %06x\n", machine().describe_context(), offset);

	if (m_is_seg_os)
	{
		if (m_is_seg_user && m_normal_mode)
		{
			if (seg < 64)
				mmu = m_mmu_data;
			else
				mmu = m_mmu_stck;
		}
	}

	return mmu;
}

z8010_device *p8k_state::select_data_mmu(offs_t offset, uint8_t nbr)
{
	z8010_device *mmu = m_mmu_stck;
	uint8_t seg = (uint8_t)(offset >> 16);
	uint8_t seg_offs = (uint8_t)(offset >> 8);

	LOG("%s DATA MMU SELECT, offset: %06x\n", machine().describe_context(), offset);

	if (m_is_seg_os)
	{
		if (!m_normal_mode) // System mode
		{
			mmu = m_mmu_code;
		}
		else if (m_is_seg_user)
		{
			if (seg < 64)
			{
				mmu = m_mmu_data;
			}
		}
		else if (seg_offs < nbr)
		{
			mmu = m_mmu_data;
		}
	}

	return mmu;
}

uint16_t p8k_state::ram16_r(offs_t offset, uint16_t mask)
{
	offs_t const word = offset >> 1;
	return (word < m_ram.length()) ? m_ram[word] : 0xffff;
}

void p8k_state::ram16_w(offs_t offset, uint16_t data, uint16_t mask)
{
	offs_t const word = offset >> 1;
	if (word < m_ram.length())
		m_ram[word] = (m_ram[word] & ~mask) | (data & mask);
}

uint16_t p8k_state::ram_r(address_space &space, offs_t offset, uint16_t mask)
{
	uint16_t data = 0;
	if (translate_addr(space.spacenum(), false, offset))
		data = ram16_r(offset, mask);
	else if (space.spacenum() == AS_PROGRAM)
		data = 0x8d07;  // NOP instruction

	return data;
}

void p8k_state::ram_w(address_space &space, offs_t offset, uint16_t data, uint16_t mask)
{
	if (translate_addr(space.spacenum(), true, offset))
		ram16_w(offset, data, mask);
}

void p8k_state::install_memory(offs_t lrom_end, offs_t lram_start, offs_t lmem_end)
{
	void *LROM, *LRAM;

	address_space& pspace = m_cpu16->space(AS_PROGRAM);
	address_space& dspace = m_cpu16->space(AS_DATA);
	address_space& sspace = m_cpu16->space(z8001_device::AS_STACK);

	// Install handlers over entire address space
	pspace.install_readwrite_handler(0x00'0000, 0x7f'ffff,
		read16_delegate(*this, FUNC(p8k_state::ram_r)), write16_delegate(*this, FUNC(p8k_state::ram_w)));
	dspace.install_readwrite_handler(0x00'0000, 0x7f'ffff,
		read16_delegate(*this, FUNC(p8k_state::ram_r)), write16_delegate(*this, FUNC(p8k_state::ram_w)));
	sspace.install_readwrite_handler(0x00'0000, 0x7f'ffff,
		read16_delegate(*this, FUNC(p8k_state::ram_r)), write16_delegate(*this, FUNC(p8k_state::ram_w)));

	pspace.install_view(0x0000, lmem_end, m_view_code);
	dspace.install_view(0x0000, lmem_end, m_view_data);
	sspace.install_view(0x0000, lmem_end, m_view_stck);

	LROM = memregion("cpu16")->base();
	LRAM = memshare("local_ram")->ptr();
	m_view_code[0].install_rom(0x0000, lrom_end, LROM);
	m_view_code[0].install_ram(lram_start, lmem_end, LRAM);
	m_view_data[0].install_rom(0x0000, lrom_end, LROM);
	m_view_data[0].install_ram(lram_start, lmem_end, LRAM);
	m_view_stck[0].install_ram(lram_start, lmem_end, LRAM);
}

uint8_t p8k_state::reg_trpl_r()
{
	return m_reg_trpl;
}

uint8_t p8k_state::reg_if1l_r()
{
	return m_reg_if1l;
}

void p8k_state::out_ns_cb(int state)
{
	LOG("%s NORMAL/SYSTEM MODE CHANGE: %d\n", machine().describe_context(), state);

	m_normal_mode = state;
}

//**************************************************************************
//  ADDRESS MAPS
//**************************************************************************

uint8_t p8k_state::reg_scr_r()
{
	return m_reg_scr;
}

void p8k_state::reg_scr_w(uint8_t data)
{
	uint8_t diff = data ^ m_reg_scr;

	LOG("%s reg_scr_w: %02x\n", machine().describe_context(), data);

	if (diff & SCR_BD_MEMON)
	{
		if (data & SCR_BD_MEMON)
		{
			m_view_code.disable();
			m_view_data.disable();
			m_view_stck.disable();
		}
		else
		{
			m_view_code.select(0);
			m_view_data.select(0);
			m_view_stck.select(0);
		}
	}

	if (diff & SCR_MMU_ONH)
	{
		m_cpu16->space(AS_PROGRAM).invalidate_caches(read_or_write::READWRITE);
		m_cpu16->space(AS_DATA).invalidate_caches(read_or_write::READWRITE);
		m_cpu16->space(z8001_device::AS_STACK).invalidate_caches(read_or_write::READWRITE);
	}

	if (diff & SCR_SEG_USER)
	{
		m_is_seg_user = !!(data & SCR_SEG_USER);
	}

	m_reg_scr = (m_reg_scr & 0xf0) | ( data & 0x0f );   // Mask off read-only nibble
}

uint8_t p8k_state::reg_sbr_r()
{
	return m_reg_sbr;
}

void p8k_state::reg_sbr_w(uint8_t data)
{
	LOG("%s reg_SBR_w: %02x\n", machine().describe_context(), data);
	m_reg_sbr = data;
}

uint8_t p8k_state::reg_nbr_r()
{
	return m_reg_nbr;
}

void p8k_state::reg_nbr_w(uint8_t data)
{
	LOG("%s reg_NBR_w: %02x\n", machine().describe_context(), data);
	m_reg_nbr = data;
}

// MMU commands in the special I/O space; address bits 1-3 select the MMU (active low),
// bits 8-15 the MMU register
uint8_t p8k_state::mmu_cmd_r(offs_t offset)
{
	if (!BIT(offset, 0))
	{
		uint8_t reg = (uint8_t)(offset >> 8);

		if (!BIT(offset, 1)) return m_mmu_code->read(reg);
		if (!BIT(offset, 2)) return m_mmu_data->read(reg);
		if (!BIT(offset, 3)) return m_mmu_stck->read(reg);
	}

	return 0xff;
}

void p8k_state::mmu_cmd_w(offs_t offset, uint8_t data)
{
	if (!BIT(offset, 0))
	{
		uint8_t reg = (uint8_t)(offset >> 8);

		if (!BIT(offset, 1)) m_mmu_code->write(reg, data);
		if (!BIT(offset, 2)) m_mmu_data->write(reg, data);
		if (!BIT(offset, 3)) m_mmu_stck->write(reg, data);

		m_cpu16->space(AS_PROGRAM).invalidate_caches(read_or_write::READWRITE);
		m_cpu16->space(AS_DATA).invalidate_caches(read_or_write::READWRITE);
		m_cpu16->space(z8001_device::AS_STACK).invalidate_caches(read_or_write::READWRITE);
	}
}

void p8k_state::addrmap_sio(address_map &map)
{
	map.unmap_value_high();
	map(0x00f0, 0x20fc).rw(FUNC(p8k_state::mmu_cmd_r), FUNC(p8k_state::mmu_cmd_w));
}

bool p8k_state::translate_addr(int spacenum, bool write, offs_t &offset)
{
	bool const stack_access = (spacenum == z8001_device::AS_STACK);
	bool const side_effects = !machine().side_effects_disabled();

	offset <<= 1;

	bool const code_access = (spacenum == AS_PROGRAM);
	int const st = code_access ?
				(m_cpu16->is_ifetch1() ?
					z8002_device::ST_IFETCH_1 :
					z8002_device::ST_IFETCH_N) :
				(stack_access ?
					z8002_device::ST_REQ_STACK :
					z8002_device::ST_REQ_DATA);

	if (side_effects)
		observe_bus_cycle(offset, !m_dma_on && st == z8002_device::ST_IFETCH_1);
	// SUP is shared by all three MMUs, not just the selected address driver.
	if (side_effects && !m_dma_on &&
		(m_mmu_code->cpu_suppressed() || m_mmu_data->cpu_suppressed() || m_mmu_stck->cpu_suppressed()))
		return false;

	z8010_device *const mmu = code_access ?
						select_code_mmu(offset) : select_data_mmu(offset, m_reg_nbr);

	if (mmu)
	{
		LOG("%s MMU MEM REQ (space %d): %06x\n", machine().describe_context(), spacenum, offset);

		auto const result = mmu->translate(offset & 0x3f'ffff, write, true, m_dma_on, st);
		if (result.suppress || ((m_reg_scr & SCR_MMU_ONH) && !result.address_driven))
			return false;
		if (m_reg_scr & SCR_MMU_ONH)
			offset = result.address;
	}

	return true;
}

/***************************************************************************

    Coupling of the 8 bit and the 16 bit board

    16 -> 8 : 16 bit PIO0 port A (output, mode 0) -> 8 bit PIO0 port A (input, mode 1)
              The ready line of each PIO strobes the other one.
              PIO0 B bit 0 = INT8, bits 1-4 = status
    8 -> 16 : two output latches on the 8 bit board (P16A = data, P16B = status)
              feed 16 bit PIO1 (A: input mode 1, B: bit control).
              P16B bit 7 = strobe, bit 0 = INT16, bits 1-6 = status
    The ready lines are visible to the other side as bits 5 and 6 of PIO0 B:
              8 bit : bit 5 = data from the 16 bit side pending (ARDY0),
                      bit 6 = 16 bit side can take data (ARDY1)
              16 bit: bit 5 = ARDY1, bit 6 = ARDY0
    8 bit PIO0 B bit 7 holds the 16 bit board in reset.

***************************************************************************/

uint8_t p8k_state::pio0_r(offs_t offset)
{
	return m_pio0->read_alt(offset);
}

void p8k_state::pio0_w(offs_t offset, uint8_t data)
{
	m_pio0->write_alt(offset, data);

	// port B bit 7 controls /RESET_U8000; a latch change is not reported when only the mode changes
	if (m_cpu16 && (offset == 2 || offset == 3))
		u8000_reset(BIT(m_pio0->port_b_read(), 7));
}

void p8k_state::board16_reset()
{
	m_is_seg_user = false;

	m_reg_scr = 0xf0;
	m_reg_sbr = 0;
	m_reg_nbr = 0;
	m_reg_trpl = 0;
	m_reg_if1l = 0;
	m_lad_low = 0;
	m_if1_low = 0;
	m_normal_mode = 0;
	m_segt_state = false;
	m_p16a = m_p16b = 0;
	m_u_pio0_a_bus = 0;
	m_ardy0 = m_ardy1 = false;
	m_view_code.select(0);
	m_view_data.select(0);
	m_view_stck.select(0);
	m_ctc[0]->reset();
	m_ctc[1]->reset();
	m_sio[0]->reset();
	m_sio[1]->reset();
	m_u_pio0->reset();
	m_u_pio1->reset();
	m_u_pio2->reset();
	m_u8000_held = false;   // force the transition in u8000_reset()
	u8000_reset(true);
}

void p8k_state::u8000_reset(bool held)
{
	bool const was_held = m_u8000_held;
	m_u8000_held = held;

	if (held)
	{
		m_cpu16->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
		if (!was_held)
		{
			// the coupling logic is reset together with the 16 bit board
			m_u_pio0->reset();
			m_u_pio1->reset();
			m_p16a = m_p16b = 0;
			m_ardy0 = m_ardy1 = false;
			coupling_update_8bit();
		}
	}
	else
	{
		m_cpu16->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
	}
}

uint8_t p8k_state::u_pio0_r(offs_t offset)
{
	return m_u_pio0->read(offset);
}

void p8k_state::u_pio0_w(offs_t offset, uint8_t data)
{
	m_u_pio0_writing = true;
	m_u_pio0->write(offset, data);
	m_u_pio0_writing = false;

	if (offset == 0)
	{
		// data for the 8 bit board: present it, then strobe its PIO (ARDY0 -> ASTB-)
		m_ardy0 = true;
		m_pio0->strobe_a(0);
		m_pio0->strobe_a(1);
		coupling_update_8bit();
	}
	else if (offset == 1 || offset == 3)
	{
		// PIO0 B bits 0-4 are visible on the 8 bit board
		coupling_update_8bit();
	}
}

void p8k_state::coupling_update_8bit()
{
	// tell the 8 bit PIO that its port B inputs may have changed (interrupt on INT8)
	uint8_t const b = (m_u_pio0->port_b_read() & 0x1f) | (m_ardy0 ? 0x20 : 0) | (m_ardy1 ? 0x40 : 0);
	m_pio0->port_b_write(b);
}

void p8k_state::p16a_w(uint8_t data)
{
	m_p16a = data;
}

void p8k_state::p16b_w(uint8_t data)
{
	m_p16b = data;
	if (!m_cpu16 || m_u8000_held)
		return;

	m_u_pio1->port_b_write(data & 0x7f);
	m_u_pio1->strobe_a(BIT(data, 7));
}

uint8_t p8k_state::coupling_pio0_a_r()
{
	return m_u_pio0_a_bus;
}

void p8k_state::coupling_u_pio0_a_w(uint8_t data)
{
	m_u_pio0_a_bus = data;
}

uint8_t p8k_state::coupling_pio0_b_r()
{
	return (m_u_pio0->port_b_read() & 0x1f) | (m_ardy0 ? 0x20 : 0) | (m_ardy1 ? 0x40 : 0);
}

uint8_t p8k_state::coupling_u_pio0_b_r()
{
	return (m_ardy1 ? 0x20 : 0) | (m_ardy0 ? 0x40 : 0);
}

void p8k_state::coupling_u_pio0_b_w(uint8_t data)
{
	coupling_update_8bit();
}

uint8_t p8k_state::coupling_u_pio1_a_r()
{
	return m_p16a;
}

uint8_t p8k_state::coupling_u_pio1_b_r()
{
	return m_p16b & 0x7f;
}

void p8k_state::coupling_ardy0_set(int state, bool data_written)
{
	// the PIO pulses RDY while data is written (before the output latch is valid);
	// the data transfer to the 8 bit board is done in u_pio0_w. Here only the
	// acknowledge (RDY low after the 8 bit side took the data) is of interest.
	if (m_u_pio0_writing || state)
		return;

	m_ardy0 = false;
	coupling_update_8bit();
}

void p8k_state::coupling_ardy1_w(int state)
{
	m_ardy1 = state;
	coupling_update_8bit();
}

void p8k_state::coupling_ardy8_w(int state)
{
	m_u_pio0->strobe_a(state);
}

uint8_t p8k_state::pio2_r(offs_t offset)
{
	return m_u_pio2->read(offset);
}

void p8k_state::pio2_w(offs_t offset, uint8_t data)
{
	m_u_pio2->write(offset, data);

	// PIO2 port A data register: the WDC sees only real data writes
	if (offset == 0)
		m_wdc->data_w(data);

	// port B drives RESET and TE: the PIO does not report latch changes when its mode changes
	if (offset == 1 || offset == 3)
		m_wdc->control_w(m_u_pio2->port_b_read());
}

void p8k_state::addrmap_io(address_map &map)
{
	map(0x0fef0, 0x0fefe + 1).rw(m_u130, FUNC(p8k_u130_device::read), FUNC(p8k_u130_device::write)).umask16(0x00ff);   // U130 clock module
	map(0x0ff80, 0x0ff87).rw(m_sio[0], FUNC(z80sio_device::cd_ba_r), FUNC(z80sio_device::cd_ba_w)).umask16(0x00ff);
	map(0x0ff88, 0x0ff8f).rw(m_sio[1], FUNC(z80sio_device::cd_ba_r), FUNC(z80sio_device::cd_ba_w)).umask16(0x00ff);
	map(0x0ff90, 0x0ff97).rw(FUNC(p8k_state::u_pio0_r), FUNC(p8k_state::u_pio0_w)).umask16(0x00ff);
	map(0x0ff98, 0x0ff9f).rw(m_u_pio1, FUNC(z80pio_device::read), FUNC(z80pio_device::write)).umask16(0x00ff);
	map(0x0ffa0, 0x0ffa7).rw(FUNC(p8k_state::pio2_r), FUNC(p8k_state::pio2_w)).umask16(0x00ff);
	map(0x0ffa8, 0x0ffaf).rw(m_ctc[0], FUNC(z80ctc_device::read), FUNC(z80ctc_device::write)).umask16(0x00ff);
	map(0x0ffb0, 0x0ffb7).rw(m_ctc[1], FUNC(z80ctc_device::read), FUNC(z80ctc_device::write)).umask16(0x00ff);
	map(0x0ffc0, 0x0ffc1).rw(FUNC(p8k_state::reg_scr_r), FUNC(p8k_state::reg_scr_w)).umask16(0x00ff);
	map(0x0ffc8, 0x0ffc9).rw(FUNC(p8k_state::reg_sbr_r), FUNC(p8k_state::reg_sbr_w)).umask16(0x00ff);
	map(0x0ffd0, 0x0ffd1).rw(FUNC(p8k_state::reg_nbr_r), FUNC(p8k_state::reg_nbr_w)).umask16(0x00ff);
	map(0x0ffe0, 0x0ffe1).w(m_daisy, FUNC(p8k_16_daisy_device::reti_w)).umask16(0x00ff);
	map(0x0ffe8, 0x0ffe9).w(FUNC(p8k_state::soft_reset_w)).umask16(0x00ff);
	map(0x0fff0, 0x0fff1).r(FUNC(p8k_state::reg_trpl_r)).umask16(0x00ff);
	map(0x0fff8, 0x0fff9).r(FUNC(p8k_state::reg_if1l_r)).umask16(0x00ff);
}


/***************************************************************************

    P8000 16bit Peripherals

****************************************************************************/

void p8k_state::p8k_16_daisy_interrupt(int state)
{
	// Ignore the sent device state and forward the daisy chain state instead
	m_cpu16->set_input_line(z8001_device::VI_LINE, m_daisy->update_irq_state() ? ASSERT_LINE : CLEAR_LINE);
}


/* Z80 Daisy Chain */

static const z80_daisy_config p8k_16_daisy_chain[] =
{
	{ "u_ctc0" },
	{ "u_ctc1" },
	{ "u_sio0" },
	{ "u_sio1" },
	{ "u_pio0" },
	{ "u_pio1" },
	{ "u_pio2" },
	{ nullptr }
};



/***************************************************************************

    Machine Drivers

****************************************************************************/

void p8k_state::p8k(machine_config &config)
{
	/* basic machine hardware */
	z80_device& maincpu(Z80(config, "maincpu", 16_MHz_XTAL / 4));
	maincpu.set_daisy_config(p8k_daisy_chain);
	maincpu.set_addrmap(AS_PROGRAM, &p8k_state::p8k_memmap);
	maincpu.set_addrmap(AS_IO, &p8k_state::p8k_iomap);

	MCFG_MACHINE_RESET_OVERRIDE(p8k_state,p8k)

	/* peripheral hardware */
	z80dma_device& dma(P8K_DMA(config, "dma", 16_MHz_XTAL / 4));
	// DMA BUSREQ drives the Z80 BUSREQ, Z80 BUSACK returns to the DMA's BAI
	dma.out_busreq_callback().set_inputline(m_maincpu, Z80_INPUT_LINE_BUSREQ);
	maincpu.busack_cb().set(dma, FUNC(z80dma_device::bai_w));
	dma.out_int_callback().set(FUNC(p8k_state::p8k_dma_irq_w));
	dma.in_mreq_callback().set(FUNC(p8k_state::memory_read_byte));
	dma.out_mreq_callback().set(FUNC(p8k_state::memory_write_byte));
	dma.in_iorq_callback().set(FUNC(p8k_state::io_read_byte));
	dma.out_iorq_callback().set(FUNC(p8k_state::io_write_byte));

	z80ctc_device& ctc0(Z80CTC(config, "ctc0", 1229000));    /* 1.22MHz clock */
	// Baud rate generators: the baud rate oscillator is the trigger input of the CTC
	// channels, their zero count outputs clock the SIO channels (WEGA sio.h):
	//   CTC1 ch0 = SIO0 A (tty0), ch1 = SIO0 B (tty1, console), ch2 = SIO1 A (tty2)
	//   CTC0 ch0 = SIO1 B (tty3); CTC0 also has the FDC and the system channel
	ctc0.intr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	ctc0.set_reload_on_reset(true); // required by the monitor's hardware self-test
	ctc0.set_clk<0>(CLK_BAUD);
	ctc0.zc_callback<0>().set("sio1", FUNC(z80sio_device::rxcb_w));
	ctc0.zc_callback<0>().append("sio1", FUNC(z80sio_device::txcb_w));

	z80ctc_device& ctc1(Z80CTC(config, "ctc1", 1229000));    /* 1.22MHz clock */
	ctc1.intr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	ctc1.set_reload_on_reset(true);
	ctc1.set_clk<0>(CLK_BAUD);
	ctc1.set_clk<1>(CLK_BAUD);
	ctc1.set_clk<2>(CLK_BAUD);
	ctc1.zc_callback<0>().set("sio", FUNC(z80sio_device::rxca_w));
	ctc1.zc_callback<0>().append("sio", FUNC(z80sio_device::txca_w));
	ctc1.zc_callback<1>().set("sio", FUNC(z80sio_device::rxcb_w));
	ctc1.zc_callback<1>().append("sio", FUNC(z80sio_device::txcb_w));
	ctc1.zc_callback<2>().set("sio1", FUNC(z80sio_device::rxca_w));
	ctc1.zc_callback<2>().append("sio1", FUNC(z80sio_device::txca_w));

	z80sio_device& sio(Z80SIO(config, "sio", 16_MHz_XTAL / 4));
	sio.out_txda_callback().set("tty0", FUNC(rs232_port_device::write_txd));
	sio.out_dtra_callback().set("tty0", FUNC(rs232_port_device::write_dtr));
	sio.out_rtsa_callback().set("tty0", FUNC(rs232_port_device::write_rts));
	sio.out_txdb_callback().set("rs232", FUNC(rs232_port_device::write_txd));
	sio.out_dtrb_callback().set("rs232", FUNC(rs232_port_device::write_dtr));
	sio.out_rtsb_callback().set("rs232", FUNC(rs232_port_device::write_rts));
	sio.out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	rs232_port_device &tty0(RS232_PORT(config, "tty0", default_rs232_devices, nullptr));
	tty0.rxd_handler().set("sio", FUNC(z80sio_device::rxa_w));
	tty0.cts_handler().set("sio", FUNC(z80sio_device::ctsa_w));
	tty0.dcd_handler().set("sio", FUNC(z80sio_device::dcda_w));

	// tty1 is the system console
	rs232_port_device &rs232(RS232_PORT(config, "rs232", default_rs232_devices, "terminal"));
	rs232.rxd_handler().set("sio", FUNC(z80sio_device::rxb_w));
	rs232.cts_handler().set("sio", FUNC(z80sio_device::ctsb_w));

	z80sio_device& sio1(Z80SIO(config, "sio1", 16_MHz_XTAL / 4));
	sio1.out_txda_callback().set("tty2", FUNC(rs232_port_device::write_txd));
	sio1.out_dtra_callback().set("tty2", FUNC(rs232_port_device::write_dtr));
	sio1.out_rtsa_callback().set("tty2", FUNC(rs232_port_device::write_rts));
	sio1.out_txdb_callback().set("tty3", FUNC(rs232_port_device::write_txd));
	sio1.out_dtrb_callback().set("tty3", FUNC(rs232_port_device::write_dtr));
	sio1.out_rtsb_callback().set("tty3", FUNC(rs232_port_device::write_rts));
	sio1.out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	rs232_port_device &tty2(RS232_PORT(config, "tty2", default_rs232_devices, nullptr));
	tty2.rxd_handler().set("sio1", FUNC(z80sio_device::rxa_w));
	tty2.cts_handler().set("sio1", FUNC(z80sio_device::ctsa_w));
	tty2.dcd_handler().set("sio1", FUNC(z80sio_device::dcda_w));
	rs232_port_device &tty3(RS232_PORT(config, "tty3", default_rs232_devices, nullptr));
	tty3.rxd_handler().set("sio1", FUNC(z80sio_device::rxb_w));
	tty3.cts_handler().set("sio1", FUNC(z80sio_device::ctsb_w));
	tty3.dcd_handler().set("sio1", FUNC(z80sio_device::dcdb_w));

	Z80PIO(config, m_pio0, 1229000);
	m_pio0->out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	z80pio_device& pio1(Z80PIO(config, "pio1", 1229000));
	pio1.out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	Z80PIO(config, m_pio2, 1229000);
	m_pio2->out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	m_pio2->in_pa_callback().set_ioport("DSW");

	I8272A(config, m_i8272, 16_MHz_XTAL / 2, true);
	m_i8272->drq_wr_callback().set("dma", FUNC(z80dma_device::rdy_w));
	FLOPPY_CONNECTOR(config, "i8272:0", p8k_floppies, "525hd", floppy_image_device::default_mfm_floppy_formats);
	FLOPPY_CONNECTOR(config, "i8272:1", p8k_floppies, "525hd", floppy_image_device::default_mfm_floppy_formats);

	/* sound hardware */
	SPEAKER(config, "mono").front_center();

	BEEP(config, "beeper", 3250).add_route(ALL_OUTPUTS, "mono", 0.5);
}

void p8k_state::p8k_16(machine_config &config)
{
	// the 16 bit system always consists of an 8 bit board and a 16 bit board
	// that are coupled through two PIOs
	p8k(config);

	/* 16 bit board: CPU and memory management */
	Z8001(config, m_cpu16, CLK_CPU);
	m_cpu16->set_addrmap(AS_PROGRAM, &p8k_state::addrmap_program);
	m_cpu16->set_addrmap(AS_DATA, &p8k_state::addrmap_data);
	m_cpu16->set_addrmap(z8001_device::AS_STACK, &p8k_state::addrmap_stack);
	m_cpu16->set_addrmap(AS_IO, &p8k_state::addrmap_io);
	m_cpu16->set_addrmap(z8001_device::AS_SIO, &p8k_state::addrmap_sio);
	m_cpu16->viack().set(FUNC(p8k_state::viack_r));
	m_cpu16->nviack().set(FUNC(p8k_state::nviack_r));
	m_cpu16->segtack().set(FUNC(p8k_state::segtack_r));
	m_cpu16->nmiack().set(FUNC(p8k_state::nmiack_r));
	m_cpu16->ns().set(FUNC(p8k_state::out_ns_cb));

	Z8010(config, m_mmu_code, CLK_CPU);
	m_mmu_code->out_segt_cb().set(FUNC(p8k_state::segt_interrupt));
	Z8010(config, m_mmu_data, CLK_CPU);
	m_mmu_data->out_segt_cb().set(FUNC(p8k_state::segt_interrupt));
	Z8010(config, m_mmu_stck, CLK_CPU);
	m_mmu_stck->out_segt_cb().set(FUNC(p8k_state::segt_interrupt));

	P8K_16_DAISY(config, m_daisy);
	m_daisy->set_daisy_config(p8k_16_daisy_chain);
	m_daisy->int_cb().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

	/* peripheral hardware */
	// Baud rate generators (as on the System 8000 CPU board): the baud rate
	// oscillator is the trigger input of the CTC channels, their zero count
	// outputs clock the SIO channels.
	//   CTC0 ch0 = SIO0 A (tty4), ch1 = SIO0 B (tty5), ch2 = SIO1 A (tty6), CTC1 ch0 = SIO1 B (tty7)
	Z80CTC(config, m_ctc[0], CLK_CPU).intr_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));
	m_ctc[0]->set_clk<0>(CLK_BAUD);
	m_ctc[0]->set_clk<1>(CLK_BAUD);
	m_ctc[0]->set_clk<2>(CLK_BAUD);
	m_ctc[0]->zc_callback<0>().set(m_sio[0], FUNC(z80sio_device::rxca_w));
	m_ctc[0]->zc_callback<0>().append(m_sio[0], FUNC(z80sio_device::txca_w));
	m_ctc[0]->zc_callback<1>().set(m_sio[0], FUNC(z80sio_device::rxcb_w));
	m_ctc[0]->zc_callback<1>().append(m_sio[0], FUNC(z80sio_device::txcb_w));
	m_ctc[0]->zc_callback<2>().set(m_sio[1], FUNC(z80sio_device::rxca_w));
	m_ctc[0]->zc_callback<2>().append(m_sio[1], FUNC(z80sio_device::txca_w));

	Z80CTC(config, m_ctc[1], CLK_CPU).intr_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));
	m_ctc[1]->set_clk<0>(CLK_BAUD);
	m_ctc[1]->zc_callback<0>().set(m_sio[1], FUNC(z80sio_device::rxcb_w));
	m_ctc[1]->zc_callback<0>().append(m_sio[1], FUNC(z80sio_device::txcb_w));
	m_ctc[1]->zc_callback<2>().set(m_ctc[1], FUNC(z80ctc_device::trg3));

	// Real-time (line) clock: CTC 1 channels 2 & 3 are cascaded and run in
	// COUNTER mode, counting the oscillator fed to the CTC trigger input.
	// WEGA programs ch2 /256 and ch3 /80 (event.s) -> a 60 Hz jiffy interrupt.
	// Feed ch2 only through trg2: configuring both its internal clock and this
	// external trigger would count the oscillator twice.
	clock_device &rtc_clk(CLOCK(config, "rtc_clk", CLK_CTC));
	rtc_clk.signal_handler().set(m_ctc[1], FUNC(z80ctc_device::trg2));

	Z80SIO(config, m_sio[0], CLK_CPU);
	m_sio[0]->out_txda_callback().set("tty4", FUNC(rs232_port_device::write_txd));
	m_sio[0]->out_dtra_callback().set("tty4", FUNC(rs232_port_device::write_dtr));
	m_sio[0]->out_rtsa_callback().set("tty4", FUNC(rs232_port_device::write_rts));
	m_sio[0]->out_txdb_callback().set("u_rs232", FUNC(rs232_port_device::write_txd));
	m_sio[0]->out_dtrb_callback().set("u_rs232", FUNC(rs232_port_device::write_dtr));
	m_sio[0]->out_rtsb_callback().set("u_rs232", FUNC(rs232_port_device::write_rts));
	m_sio[0]->out_int_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

	rs232_port_device &rs232(RS232_PORT(config, "u_rs232", default_rs232_devices, nullptr));
	rs232.rxd_handler().set(m_sio[0], FUNC(z80sio_device::rxb_w));
	rs232.cts_handler().set(m_sio[0], FUNC(z80sio_device::ctsb_w));

	rs232_port_device &tty4(RS232_PORT(config, "tty4", default_rs232_devices, nullptr));
	tty4.rxd_handler().set(m_sio[0], FUNC(z80sio_device::rxa_w));
	tty4.cts_handler().set(m_sio[0], FUNC(z80sio_device::ctsa_w));
	tty4.dcd_handler().set(m_sio[0], FUNC(z80sio_device::dcda_w));

	// tty5 (SIO0 B) is the terminal of the 16 bit monitor when the 8 bit part is inactive
	Z80SIO(config, m_sio[1], CLK_CPU);
	m_sio[1]->out_txda_callback().set("tty6", FUNC(rs232_port_device::write_txd));
	m_sio[1]->out_dtra_callback().set("tty6", FUNC(rs232_port_device::write_dtr));
	m_sio[1]->out_rtsa_callback().set("tty6", FUNC(rs232_port_device::write_rts));
	m_sio[1]->out_txdb_callback().set("tty7", FUNC(rs232_port_device::write_txd));
	m_sio[1]->out_dtrb_callback().set("tty7", FUNC(rs232_port_device::write_dtr));
	m_sio[1]->out_rtsb_callback().set("tty7", FUNC(rs232_port_device::write_rts));
	m_sio[1]->out_int_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

	rs232_port_device &tty6(RS232_PORT(config, "tty6", default_rs232_devices, nullptr));
	tty6.rxd_handler().set(m_sio[1], FUNC(z80sio_device::rxa_w));
	tty6.cts_handler().set(m_sio[1], FUNC(z80sio_device::ctsa_w));
	tty6.dcd_handler().set(m_sio[1], FUNC(z80sio_device::dcda_w));
	rs232_port_device &tty7(RS232_PORT(config, "tty7", default_rs232_devices, nullptr));
	tty7.rxd_handler().set(m_sio[1], FUNC(z80sio_device::rxb_w));
	tty7.cts_handler().set(m_sio[1], FUNC(z80sio_device::ctsb_w));
	tty7.dcd_handler().set(m_sio[1], FUNC(z80sio_device::dcdb_w));

	// coupling PIOs: PIO0 sends to the 8 bit board, PIO1 receives from it
	Z80PIO(config, m_u_pio0, CLK_CPU);
	m_u_pio0->out_int_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));
	m_u_pio0->out_pb_callback().set(FUNC(p8k_state::coupling_u_pio0_b_w));
	m_u_pio0->in_pb_callback().set(FUNC(p8k_state::coupling_u_pio0_b_r));
	m_u_pio0->out_ardy_callback().set(FUNC(p8k_state::coupling_ardy0_w));
	m_u_pio0->out_pa_callback().set(FUNC(p8k_state::coupling_u_pio0_a_w));

	Z80PIO(config, m_u_pio1, CLK_CPU);
	m_u_pio1->out_int_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));
	m_u_pio1->in_pa_callback().set(FUNC(p8k_state::coupling_u_pio1_a_r));
	m_u_pio1->in_pb_callback().set(FUNC(p8k_state::coupling_u_pio1_b_r));
	m_u_pio1->out_ardy_callback().set(FUNC(p8k_state::coupling_ardy1_w));

	Z80PIO(config, m_u_pio2, CLK_CPU);
	m_u_pio2->out_int_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

	// Winchester controller (software emulation) on PIO2
	P8K_U130(config, m_u130, 0);
	P8K_WDC(config, m_wdc, 0);
	m_u_pio2->in_pa_callback().set(m_wdc, FUNC(p8k_wdc_device::data_r));
	m_u_pio2->in_pb_callback().set(m_wdc, FUNC(p8k_wdc_device::status_r));
	m_u_pio2->out_pb_callback().set(m_wdc, FUNC(p8k_wdc_device::control_w));
	m_wdc->status_cb().set(m_u_pio2, FUNC(z80pio_device::port_b_write));   // /ST0 = interrupt request

	// 8 bit side of the coupling
	m_pio0->in_pa_callback().set(FUNC(p8k_state::coupling_pio0_a_r));
	m_pio0->in_pb_callback().set(FUNC(p8k_state::coupling_pio0_b_r));
	m_pio0->out_ardy_callback().set(FUNC(p8k_state::coupling_ardy8_w));
}

/* ROM definition */
ROM_START( p8000_8 )
	ROM_REGION( 0x30000, "maincpu", 0 )
	ROM_LOAD("mon8_1_3.1",  0x0000, 0x1000, CRC(ad1bb118) SHA1(2332963acd74d5d1a009d9bce8a2b108de01d2a5))
	ROM_LOAD("mon8_2_3.1",  0x1000, 0x1000, CRC(daced7c2) SHA1(f1f778e72568961b448020fc543ed6e81bbe81b1))
ROM_END

ROM_START( p8000_16 )
	ROM_REGION( 0x30000, "maincpu", 0 )
	ROM_LOAD("mon8_1_3.1",  0x0000, 0x1000, CRC(ad1bb118) SHA1(2332963acd74d5d1a009d9bce8a2b108de01d2a5))
	ROM_LOAD("mon8_2_3.1",  0x1000, 0x1000, CRC(daced7c2) SHA1(f1f778e72568961b448020fc543ed6e81bbe81b1))

	ROM_REGION16_BE( 0x4000, "cpu16", 0 )
	ROM_LOAD16_BYTE("mon16_1h_3.1_udos",   0x0000, 0x1000, CRC(0c3c28da) SHA1(0cd35444c615b404ebb9cf80da788593e573ddb5))
	ROM_LOAD16_BYTE("mon16_1l_3.1_udos",   0x0001, 0x1000, CRC(e8857bdc) SHA1(f89c65cbc479101130c71806fd3ddc28e6383f12))
	ROM_LOAD16_BYTE("mon16_2h_3.1_udos",   0x2000, 0x1000, CRC(cddf58d5) SHA1(588bad8df75b99580459c7a8e898a3396907e3a4))
	ROM_LOAD16_BYTE("mon16_2l_3.1_udos",   0x2001, 0x1000, CRC(395ee7aa) SHA1(d72fadb1608cd0915cd5ce6440897303ac5a12a6))
ROM_END

} // anonymous namespace


/* Driver */

//    YEAR  NAME      PARENT  COMPAT  MACHINE  INPUT  CLASS      INIT        COMPANY                   FULLNAME               FLAGS
COMP( 1989, p8000_8,  0,      0,      p8k,     p8k,   p8k_state, init_p8k,   "EAW electronic Treptow", "P8000 (8bit Board)",  MACHINE_NOT_WORKING)
COMP( 1989, p8000_16, p8000_8, 0,     p8k_16,  p8k,   p8k_state, init_p8k,   "EAW electronic Treptow", "P8000 (8bit and 16bit Board)", MACHINE_NOT_WORKING)
