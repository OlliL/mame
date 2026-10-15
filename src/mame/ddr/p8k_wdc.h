// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***************************************************************************

    P8000 WDC (Winchester disk controller), software emulation

***************************************************************************/
#ifndef MAME_DDR_P8K_WDC_H
#define MAME_DDR_P8K_WDC_H

#pragma once

#include "imagedev/harddriv.h"


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
		, m_status_cb(*this)
	{ }

	// the status lines (ST0..ST2, TR) of the PIO port B inputs changed
	auto status_cb() { return m_status_cb.bind(); }

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
	void set_status(uint8_t status) { m_status = status; m_status_cb(status_r()); }
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
	devcb_write8 m_status_cb;
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


#endif // MAME_DDR_P8K_WDC_H
