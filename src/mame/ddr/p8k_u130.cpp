// license:BSD-3-Clause
// copyright-holders:Oliver Lehmann
/***************************************************************************

    P8000 U130 clock module

    Author: Oliver Lehmann <lehmann@ans-netz.de>

    The module is a quartz clock with a six digit seven segment display and
    two keys. The 16 bit board reads the display digits and presses the keys
    (see timer.c of the kernel and sa.timer.c):

      FEF1 read   0xBF = module present
      FEF1 write  bit 0 = key 1, bit 1 = key 2
      FEF5..FEFF  digits 1-6 (FEF5 = rightmost). Bits 0-4 = segments, bit 7 of
                  the sixth digit = clock pulse, bit 3 of the sixth digit is
                  set while the date is shown (the module shows hh:mm:ss or
                  mm.dd.yy)

    Key 2 toggles between time and date. Setting the clock (sa.timer) is done
    by counting up one field after the other until the software sees the
    value it wants; it is then taken over with key 1.

    In this emulation the display shows the UTC time of the host. The
    correction sequence is accepted and shows the counting fields, but the
    result is discarded when the correction ends (setting the clock is a nop).

***************************************************************************/

#include "emu.h"
#include "p8k_u130.h"


DEFINE_DEVICE_TYPE(P8K_U130, p8k_u130_device, "p8k_u130", "P8000 U130 clock module")

p8k_u130_device::p8k_u130_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, P8K_U130, tag, owner, clock)
{
}

void p8k_u130_device::device_start()
{
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
	system_time st;
	machine().current_datetime(st);
	system_time::full_time const &t = st.utc_time;

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
		// key 2
		switch (m_state)
		{
		case STATE_RUN:
			m_date = !m_date;
			break;

		case STATE_DATE_CORR:
			// end of the correction, the new date is dropped
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
		// key 1
		system_time st;
		machine().current_datetime(st);
		system_time::full_time const &t = st.utc_time;

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
				m_state = STATE_RUN;
				m_date = false;
			}
			break;
		}
	}
}
