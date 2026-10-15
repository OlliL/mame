// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/***************************************************************************

    P8000 WDC (Winchester disk controller), software emulation

    See p8k_wdc.h for a description of the interface.

***************************************************************************/

#include "emu.h"
#include "p8k_wdc.h"

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
