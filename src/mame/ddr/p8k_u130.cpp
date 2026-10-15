// license:BSD-3-Clause
// copyright-holders:Oliver Lehmann
/***************************************************************************

    P8000 U130 clock module

    Author: Oliver Lehmann <lehmann@ans-netz.de>

    The module is a quartz clock with a six digit seven segment LCD and
    three keys. The 16 bit board reads the display digits and presses the keys
    (see timer.c of the kernel and sa.timer.c):

      FEF1 read   0xBF = module present
      FEF1 write  bit 0 = VT (recessed key), bit 1 = HT (main key),
                  bit 2 = ST (stop watch key, not used by the software and
                  not emulated)
      FEF5..FEFF  digits 1-6 (FEF5 = rightmost). Bits 0-4 = segments, bit 5 =
                  COM (see below), bit 7 of the sixth digit = clock pulse, bit 3
                  of the sixth digit is set while the date is shown (the module
                  shows hh:mm:ss or mm.dd.yy)

    HT toggles between time and date (data sheet: HT shows the date, pressed
    twice the seconds). Setting the clock (sa.timer) is done by counting up
    one field after the other with VT/HT until the software sees the value it
    wants; it is then taken over with VT.

    Hardware (schematic "EAW P8000 Echtzeituhr V2" drawn by Oliver Lehmann;
    the drawing may contain errors)

      * U15 is the CMOS watch circuit U 130 X (metal gate CMOS, 32.768 kHz
        quartz, LCD driven directly, battery supply of about 1.55 V; data
        sheet: radio fernsehen elektronik 7/1984 pp. 456-458).  It has the
        segment outputs a1..g6 of six digits, the date symbol DAT, a COM
        output for the back plane of the LCD, seven weekday symbol outputs
        (y1..y7, not read out by the board) and the key inputs HT (main key),
        VT (recessed key) and ST (stop key), which are active when connected
        to Udd.
      * The LCD is driven with a 32 Hz square wave: a segment that is visible
        gets the inverted back plane signal, a segment that is not visible
        gets the same phase as COM.  The segment drive outputs are therefore
        what the board reads, together with COM: when bit 5 (COM) is set the
        software inverts the value (invalue() in timer.c).  The emulation
        returns the already decoded segments with COM = 0.
      * The six digits are read through six 4000 series CMOS tri-state hex
        buffers (U2-U7, 40098).  Each puts the segment outputs of one digit
        and COM on the data bus bits 0-5 (via the 74245 bus transceiver U1
        with 4.7k pull-ups).  They are selected by a 74138 (U12) from A1-A3:
        FEFF = hours tens (HH1), FEFD = hours units (HH2), FEFB = minutes
        tens (MI1), FEF9 = minutes units (MI2), FEF7 = seconds tens (SS1),
        FEF5 = seconds units (SS2).  The outputs Q6/Q7 of the 74138 (FEF3, FEF1)
        have no buffer behind them.  The complete address decoding (only odd
        addresses FEF1-FEFF of the I/O space) is made by a 74688
        comparator (U10, high byte FE, I/O- enable) and a 7430 (U9, A0 and
        A4-A7).  The drawing does not explain why FEF1 reads 0xBF (bit 6 low)
        which the software uses as the "module present" signature.
      * The keys are not wired to switches.  A CMOS 4035 4 bit register (U8,
        parallel load) takes data bus bits 0-2 on the write strobe (DS-)
        and drives the key inputs: Q1 = VT, Q2 = HT, Q3 = ST (Q4 is not
        used); MRESET- clears it.  A key is "pressed" for as long as its bit
        is 1.  The software waits a while after every write (outvalue() in
        timer.c) because the watch circuit debounces its key inputs.

    In this emulation the module starts with the UTC time of the host. Setting
    the time or the date changes only the module's own clock (an offset to the
    host time, the host clock is never touched). The offset is kept in the
    NVRAM file of the device, so a reset, or a restart of the emulation, doesn't
    lose the setting, like the battery of the real module. The time keeps
    running while the module is stopped for a correction only in the sense that
    the new time is taken over the moment the clock is started again.

***************************************************************************/

#include "emu.h"
#include "p8k_u130.h"

#include <algorithm>


DEFINE_DEVICE_TYPE(P8K_U130, p8k_u130_device, "p8k_u130", "P8000 U130 clock module")

namespace {

// days since 1970-01-01 of a date in the proleptic Gregorian calendar
int64_t days_from_civil(int64_t y, unsigned m, unsigned d)
{
	y -= (m <= 2) ? 1 : 0;
	int64_t const era = (y >= 0 ? y : y - 399) / 400;
	unsigned const yoe = unsigned(y - era * 400);
	unsigned const doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
	unsigned const doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
	return era * 146097 + int64_t(doe) - 719468;
}

int days_in_month(int64_t year, int month) // month 1-12
{
	static const int days[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	bool const leap = ((year % 4) == 0 && (year % 100) != 0) || (year % 400) == 0;
	return days[month - 1] + ((month == 2 && leap) ? 1 : 0);
}

} // anonymous namespace

p8k_u130_device::p8k_u130_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, P8K_U130, tag, owner, clock)
	, device_nvram_interface(mconfig, *this)
{
}

int64_t p8k_u130_device::host_time() const
{
	system_time st;
	machine().current_datetime(st);
	return st.time;
}

// the module's own time
system_time::full_time p8k_u130_device::now() const
{
	return system_time(time_t(host_time() + m_offset)).utc_time;
}

void p8k_u130_device::nvram_default()
{
	m_offset = 0;
}

bool p8k_u130_device::nvram_read(util::read_stream &file)
{
	uint8_t buf[8];
	auto const [err, actual] = util::read(file, buf, sizeof(buf));
	if (err || actual != sizeof(buf))
		return false;

	uint64_t v = 0;
	for (int i = 7; i >= 0; i--)
		v = (v << 8) | buf[i];
	m_offset = int64_t(v);
	return true;
}

bool p8k_u130_device::nvram_write(util::write_stream &file)
{
	uint8_t buf[8];
	uint64_t const v = uint64_t(m_offset);
	for (int i = 0; i < 8; i++)
		buf[i] = uint8_t(v >> (i * 8));
	auto const [err, actual] = util::write(file, buf, sizeof(buf));
	return !err && actual == sizeof(buf);
}

void p8k_u130_device::device_start()
{
	save_item(NAME(m_offset));
	save_item(NAME(m_state));
	save_item(NAME(m_date));
	save_item(NAME(m_buttons));
	save_item(NAME(m_step));
	save_item(NAME(m_live));
	save_item(NAME(m_start));
	save_item(NAME(m_stamp));
	save_item(NAME(m_v));
}

void p8k_u130_device::device_reset()
{
	m_state = STATE_RUN;
	m_date = false;
	m_buttons = 0;
	m_step = 0;
	m_live = -1;
}

// the field that counts up while it is being corrected
void p8k_u130_device::sync()
{
	if (m_live < 0)
		return;

	static const int date_range[5] = { 12, 4, 10, 10, 10 };
	static const int date_base[5] = { 1, 0, 0, 0, 0 };
	static const int time_range[3] = { 24, 6, 10 };

	int const range = (m_state == STATE_DATE_CORR) ? date_range[m_live] : time_range[m_live];
	int const base = (m_state == STATE_DATE_CORR) ? date_base[m_live] : 0;
	// the field counts up four times per second
	int64_t const n = int64_t((machine().time() - m_stamp).as_double() * 4.0);
	m_v[m_live] = base + int((m_start - base + n) % range);
}

void p8k_u130_device::start_field(int index)
{
	m_live = index;
	m_start = m_v[index];
	m_stamp = machine().time();
}

void p8k_u130_device::display(uint8_t *digits, bool &date)
{
	sync();

	int d[6]; // digit values, rightmost first
	system_time::full_time const t = now();

	if (m_state == STATE_DATE_CORR)
	{
		date = true;
		d[0] = m_v[4];
		d[1] = m_v[3];
		d[2] = m_v[2];
		d[3] = m_v[1];
		d[4] = m_v[0] % 10;
		d[5] = m_v[0] / 10;
	}
	else if (m_state == STATE_TIME_SET)
	{
		date = false;
		d[0] = m_v[3] % 10;
		d[1] = m_v[3] / 10;
		d[2] = m_v[2];
		d[3] = m_v[1];
		d[4] = m_v[0] % 10;
		d[5] = m_v[0] / 10;
	}
	else if (m_date)
	{
		date = true;
		int const year = t.year % 100;
		d[0] = year % 10;
		d[1] = year / 10;
		d[2] = t.mday % 10;
		d[3] = t.mday / 10;
		d[4] = (t.month + 1) % 10;
		d[5] = (t.month + 1) / 10;
	}
	else
	{
		date = false;
		d[0] = t.second % 10;
		d[1] = t.second / 10;
		d[2] = t.minute % 10;
		d[3] = t.minute / 10;
		d[4] = t.hour % 10;
		d[5] = t.hour / 10;
	}

	static const uint8_t seg7[10] = { 0x0f, 0x02, 0x17, 0x13, 0x1a, 0x19, 0x1d, 0x03, 0x1f, 0x1b };
	for (int i = 0; i < 5; i++)
		digits[i] = seg7[d[i]];

	// the sixth digit is only 0-2: blank for 0, and only three segment bits are connected
	static const uint8_t seg7_6[3] = { 0x00, 0x02, 0x05 };
	digits[5] = seg7_6[d[5] % 3] | 0x80 | (date ? 0x08 : 0x00);
}

uint8_t p8k_u130_device::read(offs_t offset)
{
	if (offset == 0)
		return 0xbf;
	if (offset < 2)
		return 0xff;

	uint8_t digits[6];
	bool date;
	display(digits, date);
	return digits[offset - 2];
}

void p8k_u130_device::write(offs_t offset, uint8_t data)
{
	if (offset != 0)
		return;

	data &= 3;
	uint8_t const rising = data & ~m_buttons;
	m_buttons = data;

	if (BIT(rising, 1))
	{
		// HT (key 2)
		switch (m_state)
		{
		case STATE_RUN:
			m_date = !m_date;
			break;

		case STATE_DATE_CORR:
			// end of the correction: take over the new date, the time of day stays
			{
				sync();
				system_time::full_time const t = now();
				int64_t const cur = host_time() + m_offset;
				int64_t const tod = ((cur % 86400) + 86400) % 86400;
				int const month = std::clamp(int(m_v[0]), 1, 12);
				int64_t const year = (int64_t(t.year) / 100) * 100 + m_v[3] * 10 + m_v[4];
				int const day = std::clamp(m_v[1] * 10 + m_v[2], 1, days_in_month(year, month));
				m_offset = days_from_civil(year, month, day) * 86400 + tod - host_time();
			}
			m_state = STATE_RUN;
			m_date = false;
			m_live = -1;
			break;

		case STATE_TIME_SET:
			m_v[3] = 0; // seconds to zero
			break;
		}
	}

	if (BIT(rising, 0))
	{
		// VT (key 1)
		system_time::full_time const t = now();

		switch (m_state)
		{
		case STATE_RUN:
			if (m_date && BIT(data, 1))
			{
				// both keys: correct the date, starting with the month
				m_state = STATE_DATE_CORR;
				m_step = 0;
				m_v[0] = t.month + 1;
				m_v[1] = t.mday / 10;
				m_v[2] = t.mday % 10;
				m_v[3] = (t.year % 100) / 10;
				m_v[4] = t.year % 10;
				start_field(0);
			}
			else if (!m_date && !BIT(data, 1))
			{
				// stop the clock
				m_state = STATE_TIME_SET;
				m_step = 0;
				m_live = -1;
				m_v[0] = t.hour;
				m_v[1] = t.minute / 10;
				m_v[2] = t.minute % 10;
				m_v[3] = t.second;
			}
			break;

		case STATE_DATE_CORR:
			// next field: day tens, day units, year tens, year units
			sync();
			m_live = -1;
			if (++m_step < 5)
				start_field(m_step);
			break;

		case STATE_TIME_SET:
			// 1: -, 2: hours, 3: minutes tens, 4: minutes units, 5: stop, 6: start the clock
			sync();
			m_live = -1;
			m_step++;
			if (m_step >= 2 && m_step <= 4)
				start_field(m_step - 2);
			else if (m_step >= 6)
			{
				// start the clock with the new time, the date stays
				int64_t const cur = host_time() + m_offset;
				int64_t const day_start = cur - (((cur % 86400) + 86400) % 86400);
				int const hour = std::clamp(int(m_v[0]), 0, 23);
				int const minute = std::clamp(m_v[1] * 10 + m_v[2], 0, 59);
				int const second = std::clamp(int(m_v[3]), 0, 59);
				m_offset = day_start + hour * 3600 + minute * 60 + second - host_time();
				m_state = STATE_RUN;
				m_date = false;
			}
			break;
		}
	}
}
