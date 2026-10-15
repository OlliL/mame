// license:BSD-3-Clause
// copyright-holders:Oliver Lehmann
/***************************************************************************

    P8000 U130 clock module

    Author: Oliver Lehmann <lehmann@ans-netz.de>

***************************************************************************/
#ifndef MAME_DDR_P8K_U130_H
#define MAME_DDR_P8K_U130_H

#pragma once


// Quartz clock module U130 of the P8000 16 bit board (ports FEF1-FEFF)
class p8k_u130_device : public device_t, public device_nvram_interface
{
public:
	p8k_u130_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// offset 0 = FEF1 (keys, presence), 2..7 = FEF5..FEFF (digits)
	uint8_t read(offs_t offset);
	void write(offs_t offset, uint8_t data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// the clock runs on a battery: the offset to the host time is kept
	virtual void nvram_default() override ATTR_COLD;
	virtual bool nvram_read(util::read_stream &file) override ATTR_COLD;
	virtual bool nvram_write(util::write_stream &file) override ATTR_COLD;

private:
	system_time::full_time now() const;
	int64_t host_time() const;
	void sync();
	void start_field(int index);
	void display(uint8_t *digits, bool &date);

	enum { STATE_RUN, STATE_DATE_CORR, STATE_TIME_SET };
	int64_t m_offset = 0;             // seconds that the module's time is ahead of the host's UTC time
	uint8_t m_state = STATE_RUN;
	bool m_date = false;              // display shows the date
	uint8_t m_buttons = 0;            // last value written to FEF1 (bit 0 = VT, bit 1 = HT)
	int m_step = 0;                   // number of VT presses during a correction
	int m_live = -1;                  // field that is currently counting up
	int m_start = 0;                  // value of that field when it started
	attotime m_stamp;                 // time at which it started
	int m_v[5] = { 0, 0, 0, 0, 0 };   // values shown during a correction
};

DECLARE_DEVICE_TYPE(P8K_U130, p8k_u130_device)

#endif // MAME_DDR_P8K_U130_H
