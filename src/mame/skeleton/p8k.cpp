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
      * properly implement Z80 daisy chain in 16 bit board
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

class p8k_16_daisy_device : public device_t, public z80_daisy_chain_interface
{
public:
	p8k_16_daisy_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);
	uint16_t viack_r() {
		device_z80daisy_interface *intf = daisy_get_irq_device();
		return intf ? intf->z80daisy_irq_ack() : 0;
	}
	void reti_w(uint8_t data) { if(data == 0x4d) daisy_call_reti_device(); }
protected:
	void device_start() override {}
};

DEFINE_DEVICE_TYPE(P8K_16_DAISY, p8k_16_daisy_device, "p8k_16_daisy", "P8000 16-bit daisy chain device")

p8k_16_daisy_device::p8k_16_daisy_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, P8K_16_DAISY, tag, owner, clock)
	, z80_daisy_chain_interface(mconfig, *this) {}


class p8k_wdc_device;
DECLARE_DEVICE_TYPE(P8K_WDC, p8k_wdc_device)

/***************************************************************************

    P8000 WDC (Winchester disk controller), software emulation

    The real WDC is a separate Z80 board that talks to PIO2 of the 16-bit
    board.  Only the protocol visible on that interface is emulated here:

      PIO2 port A : 8 bit data
      PIO2 port B : bit 0-2 /ST0-/ST2 (inputs, active low status code)
                    bit 5   RESET (output)
                    bit 6   TE / direction (output, 1 = host reads)
                    bit 7   TR (input)

    Status codes (as seen by the host after inversion of bits 0-2):
      0 busy / idle between phases   1 ready for command (9 bytes)
      2 controller wants data        3 controller offers data
      7 error, one error code byte follows

    The command set, parameter table (PAR), bad track table (BTT) and the
    error codes follow the behaviour of the P8000 WDC emulator by Oliver
    Lehmann (BSD 2-clause), which was derived from signal captures of a real
    WDC 4.2.

***************************************************************************/

class p8k_wdc_device : public device_t
{
public:
	p8k_wdc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0)
		: device_t(mconfig, P8K_WDC, tag, owner, clock)
		, m_hdd(*this, "hdd")
	{ }

	// host side (PIO2)
	uint8_t data_r();
	void data_w(uint8_t data);
	uint8_t status_r();
	void control_w(uint8_t data);

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	enum : uint8_t
	{
		CMD_RD_SECTOR     = 0x01,
		CMD_WR_SECTOR     = 0x02,
		CMD_RD_WDC_RAM    = 0x08,
		CMD_FMTRD_TRACK   = 0x14,
		CMD_WR_WDC_RAM    = 0x18,
		CMD_RD_BLOCK      = 0x21,
		CMD_WR_BLOCK      = 0x22,
		CMD_FMTBTT_TRACK  = 0x24,
		CMD_RD_PARTAB     = 0x28,
		CMD_RD_WDCERR     = 0x38,
		CMD_VER_TRACK     = 0x44,
		CMD_DL_BTTAB      = 0x48,
		CMD_RD_BTTAB      = 0x58,
		CMD_WR_BTTAB      = 0x68,
		CMD_WR_PARTAB     = 0x78,
		CMD_ST_PARBTT     = 0xc2
	};

	enum : uint8_t
	{
		ERR_STANDARD          = 0x01,
		ERR_DRIVE_NOT_READY   = 0x03,
		ERR_PAR_BLOCK_TO_HIGH = 0x05,
		ERR_NO_MARK_FOUND     = 0x0b,
		ERR_DUR_TIM_CONST_CLC = 0x15,
		ERR_BTT_FULL          = 0x16,
		ERR_BTT_INVALID       = 0x18,
		ERR_NO_DRIVE_READY    = 0x27,
		ERR_CYL0_NOT_READABLE = 0x31
	};

	enum { ST_IDLE = 0, ST_GCMD = 1, ST_RDATA = 2, ST_WDATA = 3, ST_ERROR = 7 };
	enum { PH_CMD, PH_BUSY, PH_RX, PH_TX, PH_HELD };
	enum { TM_EXEC, TM_ENTER_CMD, TM_ERROR, TM_RX_DONE };
	enum { RX_NONE, RX_WR_BLOCK, RX_WR_SECTOR, RX_WR_WDC_RAM, RX_WR_PARTAB, RX_WR_BTTAB };

	static constexpr unsigned BLOCKLEN = 512;
	static constexpr unsigned BUFSIZE = 4096;
	static constexpr unsigned RAMSIZE = 4096;     // WDC S-RAM, mapped at 0x2000 on the WDC
	static constexpr unsigned POS_PAR_WDC_PAR = 8;
	static constexpr unsigned POS_PAR_WDC_BTT = 0x88;

	TIMER_CALLBACK_MEMBER(timer_cb);

	void wdc_reset();
	void enter_cmd();
	void set_status(uint8_t status) { m_status = status; }
	void begin_busy(int tm, const attotime &delay);
	void start_tx(unsigned len);
	void start_rx(unsigned len, int cont);
	void fail(uint8_t code);
	void execute();
	void rx_done();

	// disk and parameter table
	unsigned num_drives() const { return (m_hdd->exists() && m_hdd->get_info().sectorbytes == BLOCKLEN) ? 1 : 0; }
	bool read_sector(uint32_t lba, uint8_t *buf);
	bool write_sector(uint32_t lba, const uint8_t *buf);
	bool read_blocks(uint32_t lba, unsigned count);
	bool write_blocks(uint32_t lba, unsigned count);
	uint8_t hdd_sectors() const { return m_par[18 + POS_PAR_WDC_PAR]; }
	uint8_t hdd_heads() const { return m_par[17 + POS_PAR_WDC_PAR]; }
	uint16_t hdd_cylinders() const { return (m_par[16 + POS_PAR_WDC_PAR] << 8) | m_par[15 + POS_PAR_WDC_PAR]; }
	uint32_t sector2block(uint16_t cyl, uint8_t head, uint8_t sector) const;
	uint32_t p8kblock2block(uint32_t block) const { return block + uint32_t(hdd_heads()) * hdd_sectors(); }
	uint8_t validate_block(uint32_t block) const;
	uint8_t validate_cylhead(uint16_t cyl, uint8_t head, uint32_t block) const;
	uint8_t add_btt_entry(uint16_t cyl, uint8_t head);
	void set_disk_valid();
	void set_disk_invalid();
	void set_no_disk();
	void init_disk();

	required_device<harddisk_image_device> m_hdd;
	emu_timer *m_timer = nullptr;

	// bus state
	uint8_t m_status = ST_GCMD;
	bool m_tr = false;
	bool m_te = false;
	bool m_dummy_pending = false;
	bool m_in_reset = true;      // RESET line (PIO2 B5) held by the host / pulled up while the PIO is an input
	int m_phase = PH_CMD;

	// transfer state
	uint8_t m_cmd[9] = { };
	unsigned m_cmd_idx = 0;
	std::unique_ptr<uint8_t []> m_buf;
	unsigned m_len = 0;
	unsigned m_pos = 0;
	int m_rx_cont = RX_NONE;
	uint8_t m_err_code = 0;

	// controller state
	std::unique_ptr<uint8_t []> m_ram;
	uint8_t m_par[BLOCKLEN] = { };
	bool m_disk_valid = true;
	bool m_initialized = false;
	bool m_btt_cleared = false;
	uint32_t m_blocks = 0;
};

DEFINE_DEVICE_TYPE(P8K_WDC, p8k_wdc_device, "p8k_wdc", "P8000 WDC (software emulation)")

void p8k_wdc_device::device_add_mconfig(machine_config &config)
{
	HARDDISK(config, m_hdd);
}

void p8k_wdc_device::device_start()
{
	m_buf = std::make_unique<uint8_t []>(BUFSIZE);
	m_ram = std::make_unique<uint8_t []>(RAMSIZE);
	std::fill_n(m_ram.get(), RAMSIZE, 0);
	m_timer = timer_alloc(FUNC(p8k_wdc_device::timer_cb), this);

	save_item(NAME(m_status));
	save_item(NAME(m_tr));
	save_item(NAME(m_te));
	save_item(NAME(m_dummy_pending));
	save_item(NAME(m_in_reset));
	save_item(NAME(m_phase));
	save_item(NAME(m_cmd));
	save_item(NAME(m_cmd_idx));
	save_item(NAME(m_len));
	save_item(NAME(m_pos));
	save_item(NAME(m_rx_cont));
	save_item(NAME(m_err_code));
	save_item(NAME(m_par));
	save_item(NAME(m_disk_valid));
	save_item(NAME(m_initialized));
	save_item(NAME(m_btt_cleared));
	save_item(NAME(m_blocks));
	save_pointer(NAME(m_buf), BUFSIZE);
	save_pointer(NAME(m_ram), RAMSIZE);
}

void p8k_wdc_device::device_reset()
{
	// the PIO outputs are floating after a reset, the RESET line is pulled up
	m_timer->adjust(attotime::never);
	m_in_reset = true;
	m_te = false;
	m_tr = false;
	m_phase = PH_HELD;
	set_status(ST_IDLE);
}

void p8k_wdc_device::wdc_reset()
{
	m_timer->adjust(attotime::never);
	m_te = false;
	m_tr = false;
	m_dummy_pending = false;
	m_rx_cont = RX_NONE;
	m_btt_cleared = false;
	init_disk();
	m_initialized = true;
	enter_cmd();
}

void p8k_wdc_device::enter_cmd()
{
	m_phase = PH_CMD;
	m_cmd_idx = 0;
	m_tr = false;
	set_status(ST_GCMD);
}

void p8k_wdc_device::begin_busy(int tm, const attotime &delay)
{
	m_phase = PH_BUSY;
	m_tr = false;
	set_status(ST_IDLE);
	m_timer->adjust(delay, tm);
}

TIMER_CALLBACK_MEMBER(p8k_wdc_device::timer_cb)
{
	switch (param)
	{
	case TM_EXEC:      execute(); break;
	case TM_ENTER_CMD: enter_cmd(); break;
	case TM_RX_DONE:   rx_done(); break;
	case TM_ERROR:
		m_buf[0] = m_err_code;
		start_tx(1);
		set_status(ST_ERROR);
		break;
	}
}

void p8k_wdc_device::start_tx(unsigned len)
{
	m_phase = PH_TX;
	m_len = len;
	m_pos = 0;
	m_tr = true;
	m_dummy_pending = m_te;
	set_status(ST_WDATA);
}

void p8k_wdc_device::start_rx(unsigned len, int cont)
{
	m_rx_cont = cont;
	m_len = len;
	m_pos = 0;
	if (len == 0)
	{
		begin_busy(TM_RX_DONE, attotime::from_usec(100));
		return;
	}
	m_phase = PH_RX;
	set_status(ST_RDATA);
}

void p8k_wdc_device::fail(uint8_t code)
{
	// the real WDC takes a while before it reports an error; reporting it
	// too early makes the host lose the handshake
	m_err_code = code;
	begin_busy(TM_ERROR, attotime::from_msec(70));
}

uint8_t p8k_wdc_device::status_r()
{
	// bits 5/6 only matter while the PIO port is an input: they show the pin levels
	return ((~m_status) & 0x07) | (m_tr ? 0x80 : 0x00) | (m_in_reset ? 0x20 : 0x00) | (m_te ? 0x40 : 0x00);
}

void p8k_wdc_device::control_w(uint8_t data)
{
	const bool te = BIT(data, 6);
	if (te && !m_te)
		m_dummy_pending = true; // the host's dummy read before the first byte

	m_te = te;

	if (BIT(data, 5))
	{
		// controller held in reset: it ignores everything the host does
		if (!m_in_reset)
		{
			m_in_reset = true;
			m_timer->adjust(attotime::never);
			m_tr = false;
			m_phase = PH_HELD;
			set_status(ST_IDLE);
		}
	}
	else if (m_in_reset)
	{
		m_in_reset = false;
		wdc_reset();
	}
}

uint8_t p8k_wdc_device::data_r()
{
	if (m_phase != PH_TX || !m_te)
		return 0xff;

	if (m_dummy_pending)
	{
		m_dummy_pending = false;
		return 0xff;
	}

	const uint8_t data = m_buf[m_pos++];
	if (m_pos >= m_len)
		begin_busy(TM_ENTER_CMD, attotime::from_usec(100));
	return data;
}

void p8k_wdc_device::data_w(uint8_t data)
{
	switch (m_phase)
	{
	case PH_CMD:
		m_cmd[m_cmd_idx++] = data;
		if (m_cmd_idx >= sizeof(m_cmd))
			begin_busy(TM_EXEC, attotime::from_usec(200));
		break;

	case PH_RX:
		m_buf[m_pos++] = data;
		if (m_pos >= m_len)
			begin_busy(TM_RX_DONE, attotime::from_usec(100));
		break;

	default:
		break;
	}
}

//-------------------------------------------------
//  disk access and parameter table
//-------------------------------------------------

bool p8k_wdc_device::read_sector(uint32_t lba, uint8_t *buf)
{
	return num_drives() && m_hdd->read(lba, buf);
}

bool p8k_wdc_device::write_sector(uint32_t lba, const uint8_t *buf)
{
	return num_drives() && m_hdd->write(lba, buf);
}

bool p8k_wdc_device::read_blocks(uint32_t lba, unsigned count)
{
	for (unsigned i = 0; i < count; i++)
		if (!read_sector(lba + i, &m_buf[i * BLOCKLEN]))
			return false;
	return true;
}

bool p8k_wdc_device::write_blocks(uint32_t lba, unsigned count)
{
	for (unsigned i = 0; i < count; i++)
		if (!write_sector(lba + i, &m_buf[i * BLOCKLEN]))
			return false;
	return true;
}

uint32_t p8k_wdc_device::sector2block(uint16_t cyl, uint8_t head, uint8_t sector) const
{
	return uint32_t(cyl) * hdd_heads() * hdd_sectors() + uint32_t(head) * hdd_sectors() + (sector - 1);
}

uint8_t p8k_wdc_device::validate_block(uint32_t block) const
{
	return (m_disk_valid && block > m_blocks) ? ERR_PAR_BLOCK_TO_HIGH : 0;
}

uint8_t p8k_wdc_device::validate_cylhead(uint16_t cyl, uint8_t head, uint32_t block) const
{
	if (m_disk_valid && (cyl > hdd_cylinders() || head > hdd_heads()))
		return ERR_PAR_BLOCK_TO_HIGH;
	return validate_block(block);
}

uint8_t p8k_wdc_device::add_btt_entry(uint16_t cyl, uint8_t head)
{
	uint8_t *btt = &m_par[POS_PAR_WDC_BTT];
	unsigned count = btt[0] | (btt[1] << 8);
	if (count >= 120)
		return ERR_BTT_FULL;

	btt[count++ + 2] = cyl >> 8;
	btt[count++ + 2] = uint8_t(cyl);
	btt[count++ + 2] = head;
	btt[0] = uint8_t(count);
	btt[1] = count >> 8;
	return 0;
}

// Parameter table of an unformatted / absent disk (all unused bytes are 0x81)
static const uint8_t wdc_par_unformatted[128] =
{
	'W', 'D', 'C', '_', '4', '.', '2', 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
	0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x64, 0x00, 0x02, 0x12, 0x64, 0x00, 0x01, 0x81, 0x81,
	0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xeb, 0x0d, 0x00, 0x00, 0x41, 0x00,
	0x01, 0x01, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
	0x81, 0x81, 0x81, 0x81, 0x81, 0xeb, 0x0d, 0x00, 0x00, 0x24, 0x00, 0x00, 0x81, 0x81, 0x81, 0x81,
	0x00, 0x81, 0x00, 0x80, 0xfe, 0x00, 0x00, 0x00, 0x01, 0x81, 0x81, 0x73, 0x00, 0x12, 0x81, 0x01,
	0x00, 0x02, 0x00, 0x00, 0x81, 0x81, 0x81, 0x00, 0x41, 0xd7, 0x28, 0x81, 0x81, 0x81, 0x81, 0x81,
	0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81
};

static const uint8_t wdc_par_no_disk[128] =
{
	'W', 'D', 'C', '_', '4', '.', '2', 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
	0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x64, 0x00, 0x02, 0x12, 0x64, 0x00, 0x01, 0x81, 0x81,
	0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x40, 0x00,
	0x00, 0x00, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
	0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
	0x00, 0x81, 0x00, 0x80, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x33, 0x00, 0x12, 0x81, 0x00,
	0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x00, 0x40, 0xd7, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81,
	0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81
};

void p8k_wdc_device::set_disk_invalid()
{
	m_disk_valid = false;
	std::copy_n(wdc_par_unformatted, 128, m_par);
}

void p8k_wdc_device::set_no_disk()
{
	m_disk_valid = false;
	std::copy_n(wdc_par_no_disk, 128, m_par);
}

void p8k_wdc_device::set_disk_valid()
{
	m_blocks = uint32_t(hdd_sectors()) * hdd_heads() * hdd_cylinders();
	const uint32_t useful = m_blocks - uint32_t(hdd_sectors()) * hdd_heads();
	const uint16_t zw0 = useful / 65536;
	const uint16_t zw1 = (useful % 65536) - 1;
	m_disk_valid = true;
	m_par[42] = uint8_t(zw1);
	m_par[43] = zw1 >> 8;
	m_par[44] = uint8_t(zw0);
	m_par[45] = zw0 >> 8;
	m_par[46] = 0x01;
	m_par[47] = 0x09;
	m_par[48] = 0x00;
	m_par[69] = 0x83;
	m_par[70] = 0x59;
	m_par[71] = 0x01;
	m_par[73] = 0x6c;
	m_par[91] = 0x33;
	m_par[95] = 0x00;
}

void p8k_wdc_device::init_disk()
{
	std::fill(std::begin(m_par), std::end(m_par), 0x81);
	m_disk_valid = true;
	m_blocks = 0;

	if (!num_drives())
	{
		set_no_disk();
		return;
	}

	// the parameter table lives in sector 0 of the drive
	uint8_t sector[BLOCKLEN];
	if (!read_sector(0, sector))
	{
		set_no_disk();
		return;
	}

	if (memcmp(sector, "WDC_4.2", 7) == 0)
	{
		memcpy(m_par, sector, BLOCKLEN);
		set_disk_valid();
	}
	else
	{
		// disk found but not initialised: the host has to run sa.format
		set_disk_invalid();
	}
}

//-------------------------------------------------
//  command execution
//-------------------------------------------------

void p8k_wdc_device::execute()
{
	const uint8_t cmd = m_cmd[0];
	const unsigned count = m_cmd[6] | (m_cmd[7] << 8);

	if (m_initialized)
	{
		// drive problems are reported with the first command only
		m_initialized = false;
		if (num_drives() == 0)
		{
			fail(ERR_NO_DRIVE_READY);
			return;
		}
		else if (!m_disk_valid)
		{
			fail(ERR_CYL0_NOT_READABLE);
			return;
		}
	}

	const bool block_cmd = cmd == CMD_RD_BLOCK || cmd == CMD_WR_BLOCK || cmd == CMD_RD_SECTOR || cmd == CMD_WR_SECTOR;

	// without a valid disk only maintenance commands are allowed
	if (num_drives() > 0 && !m_disk_valid
			&& cmd != CMD_RD_WDC_RAM && cmd != CMD_FMTRD_TRACK && cmd != CMD_WR_WDC_RAM
			&& cmd != CMD_FMTBTT_TRACK && cmd != CMD_RD_PARTAB && cmd != CMD_RD_WDCERR
			&& cmd != CMD_DL_BTTAB && !(cmd == CMD_RD_BTTAB && m_btt_cleared)
			&& cmd != CMD_WR_BTTAB && cmd != CMD_WR_PARTAB && cmd != CMD_ST_PARBTT)
	{
		if (block_cmd)
			fail(ERR_NO_MARK_FOUND);
		else if (cmd == CMD_RD_BTTAB)
			fail(ERR_BTT_INVALID);
		else if (cmd == CMD_VER_TRACK)
			fail(ERR_DUR_TIM_CONST_CLC);
		else
			fail(ERR_STANDARD);
		return;
	}

	// commands addressing a drive that does not exist
	if (m_cmd[1] >= num_drives()
			&& (block_cmd || cmd == CMD_FMTRD_TRACK || cmd == CMD_FMTBTT_TRACK || cmd == CMD_VER_TRACK
				|| (cmd == CMD_RD_BTTAB && !m_btt_cleared)))
	{
		fail(cmd == CMD_RD_BTTAB ? ERR_BTT_INVALID : ERR_DRIVE_NOT_READY);
		return;
	}

	if (count > BUFSIZE)
	{
		fail(ERR_STANDARD);
		return;
	}

	const uint32_t p8kblock = uint32_t(m_cmd[2]) | (uint32_t(m_cmd[3]) << 8) | (uint32_t(m_cmd[4]) << 16) | (uint32_t(m_cmd[5]) << 24);
	const uint16_t cyl = m_cmd[2] | (m_cmd[3] << 8);
	uint8_t err;

	switch (cmd)
	{
	case CMD_WR_BLOCK:
		start_rx(count, RX_WR_BLOCK);
		break;

	case CMD_RD_BLOCK:
		{
			const uint32_t block = p8kblock2block(p8kblock);
			err = validate_block(block);
			if (err)
				fail(err);
			else if (!read_blocks(block, count / BLOCKLEN))
				fail(ERR_STANDARD);
			else
				start_tx(count);
		}
		break;

	case CMD_WR_WDC_RAM:
		start_rx(count, RX_WR_WDC_RAM);
		break;

	case CMD_RD_WDC_RAM:
		{
			const unsigned addr = ((m_cmd[2] << 8) | m_cmd[1]) - 0x2000;
			for (unsigned i = 0; i < count; i++)
				m_buf[i] = m_ram[(addr + i) % RAMSIZE];
			m_initialized = false;
			start_tx(count);
		}
		break;

	case CMD_RD_PARTAB:
		for (unsigned i = 0; i < count && i < BLOCKLEN; i++)
			m_buf[i] = m_par[i];
		start_tx(count);
		break;

	case CMD_WR_PARTAB:
		start_rx(count, RX_WR_PARTAB);
		break;

	case CMD_DL_BTTAB:
		std::fill_n(&m_par[POS_PAR_WDC_BTT], 125, 0x00);
		m_par[122 + POS_PAR_WDC_BTT] = 0xff;
		m_btt_cleared = true;
		begin_busy(TM_ENTER_CMD, attotime::from_usec(100));
		break;

	case CMD_RD_BTTAB:
		for (unsigned i = 0; i < count; i++)
			m_buf[i] = m_par[(POS_PAR_WDC_BTT + i) % BLOCKLEN];
		start_tx(count);
		break;

	case CMD_WR_BTTAB:
		start_rx(count, RX_WR_BTTAB);
		break;

	case CMD_FMTRD_TRACK:
	case CMD_FMTBTT_TRACK:
		// no cylinder 0 formatting for CMD_FMTBTT_TRACK
		if (cmd == CMD_FMTBTT_TRACK && cyl == 0)
		{
			begin_busy(TM_ENTER_CMD, attotime::from_usec(100));
			break;
		}
		{
			uint32_t block = sector2block(cyl, m_cmd[4], m_cmd[5]);
			err = validate_cylhead(cyl, m_cmd[4], block);
			if (err)
			{
				fail(err);
				break;
			}

			uint8_t zero[BLOCKLEN] = { };
			bool ok = true;
			for (unsigned i = 0; i < hdd_sectors() && ok; i++)
				ok = write_sector(block++, zero);

			if (ok)
			{
				begin_busy(TM_ENTER_CMD, attotime::from_usec(100));
			}
			else if (cmd == CMD_FMTRD_TRACK)
			{
				fail(ERR_STANDARD);
			}
			else
			{
				err = add_btt_entry(cyl, m_cmd[4]);
				if (err)
				{
					fail(err);
				}
				else
				{
					m_buf[0] = m_cmd[2];
					m_buf[1] = m_cmd[3];
					m_buf[2] = m_cmd[4];
					start_tx(3);
				}
			}
		}
		break;

	case CMD_ST_PARBTT:
		set_disk_valid();
		if (!write_sector(0, m_par))
			fail(ERR_STANDARD);
		else
			begin_busy(TM_ENTER_CMD, attotime::from_usec(100));
		break;

	case CMD_RD_WDCERR:
		std::fill_n(m_buf.get(), count, 0x00);
		start_tx(count);
		break;

	case CMD_VER_TRACK:
		{
			uint32_t block = sector2block(cyl, m_cmd[4], m_cmd[5]);
			err = validate_cylhead(cyl, m_cmd[4], block);
			if (err)
			{
				fail(err);
				break;
			}

			bool ok = true;
			for (unsigned i = 0; i < hdd_sectors() && ok; i++)
				ok = read_sector(block++, m_buf.get());

			if (ok)
				begin_busy(TM_ENTER_CMD, attotime::from_usec(100));
			else
				fail(ERR_STANDARD);
		}
		break;

	case CMD_RD_SECTOR:
		{
			const uint32_t block = sector2block(cyl, m_cmd[4], m_cmd[5]);
			err = validate_cylhead(cyl, m_cmd[4], block);
			if (err)
				fail(err);
			else if (!read_blocks(block, count / BLOCKLEN))
				fail(ERR_STANDARD);
			else
				start_tx(count);
		}
		break;

	case CMD_WR_SECTOR:
		{
			const uint32_t block = sector2block(cyl, m_cmd[4], m_cmd[5]);
			err = validate_cylhead(cyl, m_cmd[4], block);
			if (err)
				fail(err);
			else
				start_rx(count, RX_WR_SECTOR);
		}
		break;

	default:
		logerror("unknown WDC command %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
				m_cmd[0], m_cmd[1], m_cmd[2], m_cmd[3], m_cmd[4], m_cmd[5], m_cmd[6], m_cmd[7], m_cmd[8]);
		fail(ERR_STANDARD);
		break;
	}
}

// all data of a host-to-controller transfer has arrived
void p8k_wdc_device::rx_done()
{
	const unsigned count = m_len;
	const uint32_t p8kblock = uint32_t(m_cmd[2]) | (uint32_t(m_cmd[3]) << 8) | (uint32_t(m_cmd[4]) << 16) | (uint32_t(m_cmd[5]) << 24);
	const uint16_t cyl = m_cmd[2] | (m_cmd[3] << 8);
	const int cont = m_rx_cont;
	m_rx_cont = RX_NONE;
	bool ok = true;

	switch (cont)
	{
	case RX_WR_BLOCK:
		{
			const uint32_t block = p8kblock2block(p8kblock);
			const uint8_t err = validate_block(block);
			if (err)
			{
				fail(err);
				return;
			}
			ok = write_blocks(block, count / BLOCKLEN);
		}
		break;

	case RX_WR_SECTOR:
		ok = write_blocks(sector2block(cyl, m_cmd[4], m_cmd[5]), count / BLOCKLEN);
		break;

	case RX_WR_WDC_RAM:
		{
			const unsigned addr = ((m_cmd[2] << 8) | m_cmd[1]) - 0x2000;
			for (unsigned i = 0; i < count; i++)
				m_ram[(addr + i) % RAMSIZE] = m_buf[i];
		}
		break;

	case RX_WR_PARTAB:
		for (unsigned i = 0; i < count && (i + POS_PAR_WDC_PAR) < BLOCKLEN; i++)
			m_par[i + POS_PAR_WDC_PAR] = m_buf[i];
		break;

	case RX_WR_BTTAB:
		for (unsigned i = 0; i < count && (i + POS_PAR_WDC_BTT) < BLOCKLEN; i++)
			m_par[i + POS_PAR_WDC_BTT] = m_buf[i];
		break;

	default:
		break;
	}

	if (ok)
		enter_cmd();
	else
		fail(ERR_STANDARD);
}


namespace {

class p8k_state : public driver_device
{
public:
	p8k_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_daisy(*this, "p8k_16_daisy")
		, m_pio2(*this, "pio2")
		, m_wdc(*this, "wdc")
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
	uint8_t memory_read_byte(offs_t offset);
	void memory_write_byte(offs_t offset, uint8_t data);
	uint8_t io_read_byte(offs_t offset);
	void io_write_byte(offs_t offset, uint8_t data);

	// 16-bit board: memory, special logic and MMUs
	DECLARE_MACHINE_RESET(p8k_16);
	uint16_t ram16_r(address_space &space, offs_t offset, uint16_t mem_mask);
	void ram16_w(address_space &space, offs_t offset, uint16_t data, uint16_t mem_mask);
	bool translate16(int spacenum, bool write, offs_t &offset);
	z8010_device *select_code_mmu(offs_t offset);
	z8010_device *select_data_mmu(offs_t offset);
	void segt_interrupt(int state);
	void ns16_w(int state);
	uint16_t segtack_r();
	uint16_t nmiack16_r();
	uint16_t viack16_r();
	void mmu_instruction_end();
	void install_memory16();
	uint8_t scr_r() { return 0xf0 | m_scr; }
	void scr_w(uint8_t data);
	uint8_t nbr_r() { return m_nbr; }
	void nbr_w(uint8_t data) { m_nbr = data; }
	uint8_t trpl_r() { return m_reg_trpl; }
	uint8_t if1l_r() { return m_reg_if1l; }
	void softreset_w(uint8_t) { machine().schedule_soft_reset(); }
	uint8_t mmu_cmd_r(offs_t offset);
	void mmu_cmd_w(offs_t offset, uint8_t data);
	void invalidate16();

	void p8k_16_sio_map(address_map &map) ATTR_COLD;
	void p8k_16_datamap(address_map &map) ATTR_COLD;
	void p8k_16_iomap(address_map &map) ATTR_COLD;
	void p8k_16_memmap(address_map &map) ATTR_COLD;
	void p8k_iomap(address_map &map) ATTR_COLD;
	void p8k_memmap(address_map &map) ATTR_COLD;

	required_device<cpu_device> m_maincpu; // Z80 or Z8001 depending on the machine
	optional_device<p8k_16_daisy_device> m_daisy;
	optional_device<z80pio_device> m_pio2;
	optional_device<p8k_wdc_device> m_wdc;
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
	uint8_t m_scr = 0;                // system configuration register (4 bit)
	uint8_t m_nbr = 0;                // normal break register
	uint8_t m_reg_trpl = 0;           // trap low byte register
	uint8_t m_reg_if1l = 0;           // instruction fetch first word low byte
	uint8_t m_lad_low = 0;
	uint8_t m_if1_low = 0;
	bool m_normal_mode = false;
	bool m_segt_state = false;
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
	map(0x0c, 0x0f).rw("pio0", FUNC(z80pio_device::read_alt), FUNC(z80pio_device::write_alt));
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
	m_maincpu->set_input_line(INPUT_LINE_NMI, newval ? ASSERT_LINE : CLEAR_LINE);
}


MACHINE_RESET_MEMBER(p8k_state,p8k)
{
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

****************************************************************************/

void p8k_state::machine_start()
{
	if (m_mmu_code)
	{
		install_memory16();

		save_item(NAME(m_scr));
		save_item(NAME(m_nbr));
		save_item(NAME(m_reg_trpl));
		save_item(NAME(m_reg_if1l));
		save_item(NAME(m_lad_low));
		save_item(NAME(m_if1_low));
		save_item(NAME(m_normal_mode));
		save_item(NAME(m_segt_state));
	}
}

void p8k_state::p8k_16_memmap(address_map &map)
{
	// the real work is done in install_memory16(): all accesses pass through the
	// MMU logic of the board
}

void p8k_state::p8k_16_datamap(address_map &map)
{
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
	if (m_normal_mode && (m_scr & 0x04))
		return (uint8_t(offset >> 16) < 64) ? m_mmu_data.target() : m_mmu_stck.target();
	return m_mmu_code.target();
}

z8010_device *p8k_state::select_data_mmu(offs_t offset)
{
	uint8_t const seg = uint8_t(offset >> 16);
	uint8_t const seg_offs = uint8_t(offset >> 8);

	if (!m_normal_mode)
		return m_mmu_code;
	if (m_scr & 0x04)
		return (seg < 64) ? m_mmu_data.target() : m_mmu_stck.target();
	return (seg_offs < m_nbr) ? m_mmu_data.target() : m_mmu_stck.target();
}

bool p8k_state::translate16(int spacenum, bool write, offs_t &offset)
{
	bool const side_effects = !machine().side_effects_disabled();
	bool const code_access = (spacenum == AS_PROGRAM);
	bool const stack_access = (spacenum == z8001_device::AS_STACK);
	z8001_device &cpu = *static_cast<z8001_device *>(m_maincpu.target());

	offset <<= 1;

	int const st = code_access ?
			(cpu.is_ifetch1() ? z8002_device::ST_IFETCH_1 : z8002_device::ST_IFETCH_N) :
			(stack_access ? z8002_device::ST_REQ_STACK : z8002_device::ST_REQ_DATA);

	if (side_effects)
	{
		m_lad_low = uint8_t(offset);
		if (st == z8002_device::ST_IFETCH_1)
		{
			m_if1_low = uint8_t(offset);
			// every MMU snoops each IFETCH1 cycle
			m_mmu_code->ifetch1_observed(offset);
			m_mmu_data->ifetch1_observed(offset);
			m_mmu_stck->ifetch1_observed(offset);
		}

		// SUP is shared by all three MMUs
		if (m_mmu_code->cpu_suppressed() || m_mmu_data->cpu_suppressed() || m_mmu_stck->cpu_suppressed())
			return false;
	}

	z8010_device *const mmu = code_access ? select_code_mmu(offset) : select_data_mmu(offset);
	auto const result = mmu->translate(offset & 0x3f'ffff, write, true, false, st);
	if (result.suppress)
		return false;
	if (m_scr & 0x02)
	{
		if (!result.address_driven)
			return false;
		offset = result.address;
	}
	return true;
}

uint16_t p8k_state::ram16_r(address_space &space, offs_t offset, uint16_t mem_mask)
{
	if (!translate16(space.spacenum(), false, offset))
		return (space.spacenum() == AS_PROGRAM) ? 0x8d07 : 0xffff; // NOP on suppressed fetch

	offs_t const word = offset >> 1;
	return (word < m_ram.length()) ? m_ram[word] : 0xffff;
}

void p8k_state::ram16_w(address_space &space, offs_t offset, uint16_t data, uint16_t mem_mask)
{
	if (!translate16(space.spacenum(), true, offset))
		return;

	offs_t const word = offset >> 1;
	if (word < m_ram.length())
		COMBINE_DATA(&m_ram[word]);
}

void p8k_state::install_memory16()
{
	address_space &pspace = m_maincpu->space(AS_PROGRAM);
	address_space &dspace = m_maincpu->space(AS_DATA);
	address_space &sspace = m_maincpu->space(z8001_device::AS_STACK);

	for (address_space *space : { &pspace, &dspace, &sspace })
		space->install_readwrite_handler(0x00'0000, 0x7f'ffff,
				read16_delegate(*this, FUNC(p8k_state::ram16_r)), write16_delegate(*this, FUNC(p8k_state::ram16_w)));

	pspace.install_view(0x0000, 0x7fff, m_view_code);
	dspace.install_view(0x0000, 0x7fff, m_view_data);
	sspace.install_view(0x0000, 0x7fff, m_view_stck);

	void *const rom = memregion("maincpu")->base();
	m_view_code[0].install_rom(0x0000, 0x3fff, rom);
	m_view_code[0].install_ram(0x4000, 0x7fff, &m_local_ram[0]);
	m_view_data[0].install_rom(0x0000, 0x3fff, rom);
	m_view_data[0].install_ram(0x4000, 0x7fff, &m_local_ram[0]);
	m_view_stck[0].install_ram(0x4000, 0x7fff, &m_local_ram[0]);
}

void p8k_state::invalidate16()
{
	m_maincpu->space(AS_PROGRAM).invalidate_caches(read_or_write::READWRITE);
	m_maincpu->space(AS_DATA).invalidate_caches(read_or_write::READWRITE);
	m_maincpu->space(z8001_device::AS_STACK).invalidate_caches(read_or_write::READWRITE);
}

void p8k_state::scr_w(uint8_t data)
{
	uint8_t const diff = (data ^ m_scr) & 0x0f;
	m_scr = data & 0x0f;

	if (diff & 0x01)
	{
		if (m_scr & 0x01)
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

	if (diff & 0x06)
		invalidate16();
}

MACHINE_RESET_MEMBER(p8k_state, p8k_16)
{
	m_scr = 0;
	m_nbr = 0;
	m_reg_trpl = 0;
	m_reg_if1l = 0;
	m_normal_mode = false;
	m_segt_state = false;
	m_view_code.select(0);
	m_view_data.select(0);
	m_view_stck.select(0);
}

// segment trap: the address low bytes are latched at the moment the trap is raised
void p8k_state::segt_interrupt(int state)
{
	if (state && !m_segt_state)
	{
		m_reg_trpl = m_lad_low;
		m_reg_if1l = m_if1_low;
	}
	m_segt_state = (state != 0);
	m_maincpu->set_input_line(z8001_device::SEGT_LINE, state ? ASSERT_LINE : CLEAR_LINE);
}

void p8k_state::ns16_w(int state)
{
	m_normal_mode = bool(state);
}

uint16_t p8k_state::segtack_r()
{
	return m_mmu_code->segtack_r() | m_mmu_data->segtack_r() | m_mmu_stck->segtack_r();
}

void p8k_state::mmu_instruction_end()
{
	m_mmu_code->instruction_end();
	m_mmu_data->instruction_end();
	m_mmu_stck->instruction_end();
}

uint16_t p8k_state::nmiack16_r()
{
	mmu_instruction_end();
	return 0xffff;
}

uint16_t p8k_state::viack16_r()
{
	mmu_instruction_end();
	return m_daisy->viack_r();
}

// MMU commands in the special I/O space; address bits 1-3 select the MMU (active low),
// bits 8-15 the MMU register
uint8_t p8k_state::mmu_cmd_r(offs_t offset)
{
	if (!BIT(offset, 0))
	{
		uint8_t const reg = uint8_t(offset >> 8);
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
		uint8_t const reg = uint8_t(offset >> 8);
		if (!BIT(offset, 1)) m_mmu_code->write(reg, data);
		if (!BIT(offset, 2)) m_mmu_data->write(reg, data);
		if (!BIT(offset, 3)) m_mmu_stck->write(reg, data);
		invalidate16();
	}
}

void p8k_state::p8k_16_sio_map(address_map &map)
{
	map.unmap_value_high();
	map(0x00f0, 0x20fc).rw(FUNC(p8k_state::mmu_cmd_r), FUNC(p8k_state::mmu_cmd_w));
}


uint8_t p8k_state::pio2_r(offs_t offset)
{
	return m_pio2->read(offset);
}

void p8k_state::pio2_w(offs_t offset, uint8_t data)
{
	m_pio2->write(offset, data);

	// PIO2 port A data register: the WDC sees only real data writes
	if (offset == 0)
		m_wdc->data_w(data);

	// port B drives RESET and TE: the PIO does not report latch changes when its mode changes
	if (offset == 1 || offset == 3)
		m_wdc->control_w(m_pio2->port_b_read());
}

void p8k_state::p8k_16_iomap(address_map &map)
{
//  map(0x0fef0, 0x0feff) // clock
	map(0x0ff80, 0x0ff87).rw("sio", FUNC(z80sio_device::cd_ba_r), FUNC(z80sio_device::cd_ba_w)).umask16(0x00ff);
	map(0x0ff88, 0x0ff8f).rw("sio1", FUNC(z80sio_device::cd_ba_r), FUNC(z80sio_device::cd_ba_w)).umask16(0x00ff);
	map(0x0ff90, 0x0ff97).rw("pio0", FUNC(z80pio_device::read), FUNC(z80pio_device::write)).umask16(0x00ff);
	map(0x0ff98, 0x0ff9f).rw("pio1", FUNC(z80pio_device::read), FUNC(z80pio_device::write)).umask16(0x00ff);
	map(0x0ffa0, 0x0ffa7).rw(FUNC(p8k_state::pio2_r), FUNC(p8k_state::pio2_w)).umask16(0x00ff);
	map(0x0ffa8, 0x0ffaf).rw("ctc0", FUNC(z80ctc_device::read), FUNC(z80ctc_device::write)).umask16(0x00ff);
	map(0x0ffb0, 0x0ffb7).rw("ctc1", FUNC(z80ctc_device::read), FUNC(z80ctc_device::write)).umask16(0x00ff);
	map(0x0ffc0, 0x0ffc1).rw(FUNC(p8k_state::scr_r), FUNC(p8k_state::scr_w)).umask16(0x00ff);
	map(0x0ffd0, 0x0ffd1).rw(FUNC(p8k_state::nbr_r), FUNC(p8k_state::nbr_w)).umask16(0x00ff);
	map(0x0ffe0, 0x0ffe1).w(m_daisy, FUNC(p8k_16_daisy_device::reti_w)).umask16(0x00ff);
	map(0x0ffe8, 0x0ffe9).w(FUNC(p8k_state::softreset_w)).umask16(0x00ff);
	map(0x0fff0, 0x0fff1).r(FUNC(p8k_state::trpl_r)).umask16(0x00ff);
	map(0x0fff8, 0x0fff9).r(FUNC(p8k_state::if1l_r)).umask16(0x00ff);
}


/***************************************************************************

    P8000 16bit Peripherals

****************************************************************************/

void p8k_state::p8k_16_daisy_interrupt(int state)
{
	m_maincpu->set_input_line(z8001_device::VI_LINE, state ? ASSERT_LINE : CLEAR_LINE);
}


/* Z80 Daisy Chain */

static const z80_daisy_config p8k_16_daisy_chain[] =
{
	{ "ctc0" },
	{ "ctc1" },
	{ "sio" },
	{ "sio1" },
	{ "pio0" },
	{ "pio1" },
	{ "pio2" },
	{ nullptr }
};



#if 0
/* F4 Character Displayer */
static const gfx_layout p8k_charlayout =
{
	8, 12,                  /* 8 x 12 characters */
	256,                    /* 256 characters */
	1,                  /* 1 bits per pixel */
	{ 0 },                  /* no bitplanes */
	/* x offsets */
	{ 0, 1, 2, 3, 4, 5, 6, 7 },
	/* y offsets */
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8, 8*8, 9*8, 10*8, 11*8 },
	8*16                    /* every char takes 16 bytes */
};

static GFXDECODE_START( p8k )
	GFXDECODE_ENTRY( "chargen", 0x0000, p8k_charlayout, 0, 1 )
GFXDECODE_END
#endif


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
	z80dma_device& dma(UA858D(config, "dma", 16_MHz_XTAL / 4));
	// DMA BUSREQ drives the Z80 BUSREQ, Z80 BUSACK returns to the DMA's BAI
	dma.out_busreq_callback().set_inputline(m_maincpu, Z80_INPUT_LINE_BUSREQ);
	maincpu.busack_cb().set(dma, FUNC(z80dma_device::bai_w));
	dma.out_int_callback().set(FUNC(p8k_state::p8k_dma_irq_w));
	dma.in_mreq_callback().set(FUNC(p8k_state::memory_read_byte));
	dma.out_mreq_callback().set(FUNC(p8k_state::memory_write_byte));
	dma.in_iorq_callback().set(FUNC(p8k_state::io_read_byte));
	dma.out_iorq_callback().set(FUNC(p8k_state::io_write_byte));

	clock_device &uart_clock(CLOCK(config, "uart_clock", 307200));
	uart_clock.signal_handler().set("sio", FUNC(z80sio_device::txcb_w));
	uart_clock.signal_handler().append("sio", FUNC(z80sio_device::rxcb_w));

	z80ctc_device& ctc0(Z80CTC(config, "ctc0", 1229000));    /* 1.22MHz clock */
	// to implement: callbacks!
	// manual states the callbacks should go to
	// Baud Gen 3, FDC, System-Kanal
	ctc0.intr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	ctc0.set_reload_on_reset(true); // required by the monitor's hardware self-test

	z80ctc_device& ctc1(Z80CTC(config, "ctc1", 1229000));    /* 1.22MHz clock */
	// to implement: callbacks!
	// manual states the callbacks should go to
	// Baud Gen 0, Baud Gen 1, Baud Gen 2,
	ctc1.intr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	ctc1.set_reload_on_reset(true);

	z80sio_device& sio(Z80SIO(config, "sio", 16_MHz_XTAL / 4));
	sio.out_txdb_callback().set("rs232", FUNC(rs232_port_device::write_txd));
	sio.out_dtrb_callback().set("rs232", FUNC(rs232_port_device::write_dtr));
	sio.out_rtsb_callback().set("rs232", FUNC(rs232_port_device::write_rts));
	sio.out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	rs232_port_device &rs232(RS232_PORT(config, "rs232", default_rs232_devices, "terminal"));
	rs232.rxd_handler().set("sio", FUNC(z80sio_device::rxb_w));
	rs232.cts_handler().set("sio", FUNC(z80sio_device::ctsb_w));

	z80sio_device& sio1(Z80SIO(config, "sio1", 16_MHz_XTAL / 4));
	sio1.out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	z80pio_device& pio0(Z80PIO(config, "pio0", 1229000));
	pio0.out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

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
	/* basic machine hardware */
	z8001_device &maincpu(Z8001(config, m_maincpu, XTAL(4'000'000)));
	maincpu.set_addrmap(AS_PROGRAM, &p8k_state::p8k_16_memmap);
	maincpu.set_addrmap(AS_DATA, &p8k_state::p8k_16_datamap);
	maincpu.set_addrmap(z8001_device::AS_STACK, &p8k_state::p8k_16_datamap);
	maincpu.set_addrmap(AS_IO, &p8k_state::p8k_16_iomap);
	maincpu.set_addrmap(z8001_device::AS_SIO, &p8k_state::p8k_16_sio_map);
	maincpu.viack().set(FUNC(p8k_state::viack16_r));
	maincpu.nmiack().set(FUNC(p8k_state::nmiack16_r));
	maincpu.segtack().set(FUNC(p8k_state::segtack_r));
	maincpu.ns().set(FUNC(p8k_state::ns16_w));

	Z8010(config, m_mmu_code, 4'000'000).out_segt_cb().set(FUNC(p8k_state::segt_interrupt));
	Z8010(config, m_mmu_data, 4'000'000).out_segt_cb().set(FUNC(p8k_state::segt_interrupt));
	Z8010(config, m_mmu_stck, 4'000'000).out_segt_cb().set(FUNC(p8k_state::segt_interrupt));

	MCFG_MACHINE_RESET_OVERRIDE(p8k_state, p8k_16)

	P8K_16_DAISY(config, m_daisy);
	m_daisy->set_daisy_config(p8k_16_daisy_chain);

	clock_device &uart_clock(CLOCK(config, "uart_clock", 307200));
	uart_clock.signal_handler().set("sio", FUNC(z80sio_device::txcb_w));
	uart_clock.signal_handler().append("sio", FUNC(z80sio_device::rxcb_w));

	/* peripheral hardware */
	z80ctc_device& ctc0(Z80CTC(config, "ctc0", XTAL(4'000'000)));
	ctc0.intr_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

	z80ctc_device& ctc1(Z80CTC(config, "ctc1", XTAL(4'000'000)));
	ctc1.intr_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

	z80sio_device& sio(Z80SIO(config, "sio", XTAL(4'000'000)));
	sio.out_txdb_callback().set("rs232", FUNC(rs232_port_device::write_txd));
	sio.out_dtrb_callback().set("rs232", FUNC(rs232_port_device::write_dtr));
	sio.out_rtsb_callback().set("rs232", FUNC(rs232_port_device::write_rts));
	sio.out_int_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

	rs232_port_device &rs232(RS232_PORT(config, "rs232", default_rs232_devices, "terminal"));
	rs232.rxd_handler().set("sio", FUNC(z80sio_device::rxb_w));
	rs232.cts_handler().set("sio", FUNC(z80sio_device::ctsb_w));

	z80sio_device& sio1(Z80SIO(config, "sio1", XTAL(4'000'000)));
	sio1.out_int_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

	z80pio_device& pio0(Z80PIO(config, "pio0", XTAL(4'000'000)));
	pio0.out_int_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

	z80pio_device& pio1(Z80PIO(config, "pio1", XTAL(4'000'000)));
	pio1.out_int_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

	Z80PIO(config, m_pio2, XTAL(4'000'000));
	m_pio2->out_int_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

	// Winchester controller (software emulation) on PIO2
	P8K_WDC(config, m_wdc, 0);
	m_pio2->in_pa_callback().set(m_wdc, FUNC(p8k_wdc_device::data_r));
	m_pio2->in_pb_callback().set(m_wdc, FUNC(p8k_wdc_device::status_r));
	m_pio2->out_pb_callback().set(m_wdc, FUNC(p8k_wdc_device::control_w));

	/* sound hardware */
	SPEAKER(config, "mono").front_center();
	BEEP(config, "beeper", 3250).add_route(ALL_OUTPUTS, "mono", 0.5);
}

/* ROM definition */
ROM_START( p8000 )
	ROM_REGION( 0x30000, "maincpu", 0 )
	ROM_LOAD("mon8_1_3.1",  0x0000, 0x1000, CRC(ad1bb118) SHA1(2332963acd74d5d1a009d9bce8a2b108de01d2a5))
	ROM_LOAD("mon8_2_3.1",  0x1000, 0x1000, CRC(daced7c2) SHA1(f1f778e72568961b448020fc543ed6e81bbe81b1))

	// this is for the p8000's terminal, not the p8000 itself
	ROM_REGION( 0x1000, "chargen", 0 )
	ROM_LOAD("p8t_zs",    0x0000, 0x0800, CRC(f9321251) SHA1(a6a796b58d50ec4a416f2accc34bd76bc83f18ea))
	ROM_LOAD("p8tdzs.2",  0x0800, 0x0800, CRC(32736503) SHA1(6a1d7c55dddc64a7d601dfdbf917ce1afaefbb0a))

	ROM_REGION( 0x2000, "user1", 0 )
	ROM_LOAD( "wdc4.2_1-2c43.bin", 0x0000, 0x1000, CRC(2646f1ee) SHA1(f62574ad57a2c8ac55c5df89256a707c0cafc0eb) )
	ROM_LOAD( "wdc4.2_2-5d66.bin", 0x1000, 0x1000, CRC(5d496b65) SHA1(42166d7ec51fce086c65ea829d8a3d63088815ca) )
ROM_END

ROM_START( p8000_16 )
	ROM_REGION16_BE( 0x4000, "maincpu", 0 )
	ROM_LOAD16_BYTE("mon16_1h_3.1_udos",   0x0000, 0x1000, CRC(0c3c28da) SHA1(0cd35444c615b404ebb9cf80da788593e573ddb5))
	ROM_LOAD16_BYTE("mon16_1l_3.1_udos",   0x0001, 0x1000, CRC(e8857bdc) SHA1(f89c65cbc479101130c71806fd3ddc28e6383f12))
	ROM_LOAD16_BYTE("mon16_2h_3.1_udos",   0x2000, 0x1000, CRC(cddf58d5) SHA1(588bad8df75b99580459c7a8e898a3396907e3a4))
	ROM_LOAD16_BYTE("mon16_2l_3.1_udos",   0x2001, 0x1000, CRC(395ee7aa) SHA1(d72fadb1608cd0915cd5ce6440897303ac5a12a6))

	// this is for the p8000's terminal, not the p8000 itself
	ROM_REGION( 0x1000, "chargen", 0 )
	ROM_LOAD("p8t_zs",    0x0000, 0x0800, CRC(f9321251) SHA1(a6a796b58d50ec4a416f2accc34bd76bc83f18ea))
	ROM_LOAD("p8tdzs.2",  0x0800, 0x0800, CRC(32736503) SHA1(6a1d7c55dddc64a7d601dfdbf917ce1afaefbb0a))
ROM_END

} // anonymous namespace


/* Driver */

//    YEAR  NAME      PARENT  COMPAT  MACHINE  INPUT  CLASS      INIT        COMPANY                   FULLNAME               FLAGS
COMP( 1989, p8000,    0,      0,      p8k,     p8k,   p8k_state, init_p8k,   "EAW electronic Treptow", "P8000 (8bit Board)",  MACHINE_NOT_WORKING)
COMP( 1989, p8000_16, p8000,  0,      p8k_16,  p8k,   p8k_state, empty_init, "EAW electronic Treptow", "P8000 (16bit Board)", MACHINE_NOT_WORKING)
