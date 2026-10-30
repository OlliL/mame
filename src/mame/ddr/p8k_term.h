// license:BSD-3-Clause
// copyright-holders:Oliver Lehmann
/***************************************************************************

    P8000 console terminal

    The terminal firmware (U882, i8275, ADM-31 after power on) is emulated
    by running it.  The device is meant for the console slot of the P8000
    drivers only and is not part of the default RS-232 device list.

***************************************************************************/
#ifndef MAME_DDR_P8K_TERM_H
#define MAME_DDR_P8K_TERM_H

#pragma once

#include "bus/rs232/rs232.h"


DECLARE_DEVICE_TYPE(P8K_TERMINAL, device_rs232_port_interface)

#endif // MAME_DDR_P8K_TERM_H
