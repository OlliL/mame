// license:BSD-3-Clause
// copyright-holders:Fabio Priuli, Oliver Lehmann
/***************************************************************************

    P8000 (VEB Elektro-Apparate-Werke Berlin-Treptow, GDR)

    Author of the current driver: Oliver Lehmann <lehmann@ans-netz.de>
    Original skeleton: 09/2009 Fabio Priuli, based on Matt Knoth's emulator.
    P8000emu (http://knothusa.net/Home.php) has been a great source of info.
    Other info from
      * http://www.pofo.de/P8000/notes/books/ (hardware manual "Hardwarehandbuch")
      * http://www.pofo.de/P8000/ (firmware, UDOS and WEGA sources)
      * http://www.pofo.de/P8000/notes/plaene/8-Bit-Rechner/ (schematics of the
        8 bit board P8000/8.1, Ms 889 254-2, with parts list and layout)

    Machines
      * p8000  : 8 bit board (U880 = Z80) with monitor MON8 3.1 "NUR8BIT"
                   (separate EPROMs, no attempt to start a 16 bit board)
      * p8000_16 : 8 bit board plus 16 bit board (U8001 = Z8001, three
                   U8010 = Z8010 MMUs, monitor MON16).  The boards are coupled
                   through PIOs (see the "coupling" section below); the Winchester
                   disk controller (p8k_wdc) hangs on PIO2 of the 16 bit board and
                   the quartz clock module U130 (p8k_u130) sits in the 16 bit
                   board's I/O space.
                   Operating systems: OS/M (a CP/M clone) and UDOS (a Z80-RIO
                   clone) run on the 8 bit board; WEGA (a Zilog ZEUS clone,
                   System III UNIX) runs on the 16 bit board only.

    Emulated board revisions (P8000 Hardwarehandbuch, version marking "V: ...")
      * 8 bit board:  Index 3 ("V: x3xx", monitor MON8 3.1). Chapter 4 of the
                      hardware manual.  The memory control (chapter 3.3 / 4.3)
                      is identical on Index 1 ("V: x1xx").
      * 16 bit board: Index 4 ("V: 4xxx", monitor MON16 3.1). Chapter 7 of the
                      hardware manual.  An 8 bit board Index >= 3 may only be
                      coupled with a 16 bit board Index >= 4.
      The hardware manual (chapter 6) lists the SBR register (FFC9) only for
      the Index 0 (development, "GLE") board.  It is nevertheless kept as a
      plain read/write register: MON16 3.1 tests it unconditionally in its
      hardware test (error 84 if the read back differs, no check of the board
      index), later test steps load it, and WEGA writes it.

    P8000 8 bit board memory layout (hardware manual 3.3 / 4.3)
      * three memory banks, each switched in steps of 4K:
          EPROM (8 KiB, 2 x 2732, repeating), SRAM (2 KiB, A11 not decoded so
          it appears twice in the selected 4K window), DRAM (64 KiB).
      * the selection is stored in the write-only address port ADP
        (16 words x 4 bit, addressed by A12..A15), written with OUT (C),r
        to I/O 00H; register B holds the address of the 4K slice and r the
        control word: 0 = nothing selected, 1 = EPROM, 2 = SRAM, 4 = DRAM.
      * after reset the reset flip-flop RFF is set: only the EPROM is active
        and the ADP is blocked.  A dummy IN/OUT to RES_RFF (I/O 04H) releases
        the ADP (once after reset).

    8 bit board I/O decoding and clocks (schematic sheet 1, two 74138)
      * 00-1F: ADP (00), RES_RFF (04), CTC0 (08), PIO0 (0C), latch L1 (10) and
               L2 (14) towards the 16 bit board, PIO1 (18), PIO2 (1C)
      * 20-3F: FDC (20), SIO0 (24), SIO1 (28), CTC1 (2C), three spare enables
               RES1-3 (30, 34, 38), DMA (3C)
      * Clocks: 16 MHz quartz (CPU 4 MHz, FDC), 9.832 MHz quartz (parts list
        "MQ 02-9832,0k") divided by eight (7493, manual 3.3 baud rate generator)
        = 1.229 MHz trigger input of the baud rate CTC channels.
        CTC1 channels 0-2 and CTC0 channel 0 each drive a divide by two
        flip-flop that clocks one SIO channel (SIO0 A/B, SIO1 A, SIO1 B).
      * Interrupt priority chain: DMA, PIO2, CTC0, SIO0, SIO1, PIO0 (coupling),
        PIO1 (X3 parallel port), CTC1.
      * Floppy controller (i8272a, external data separator, DMA driven),
        PIO2 pin assignment as used by the firmware (FDINIT.S):
        - port A 0-3 motor on /LCK0-3 (drives 0 and 1 are modelled)
        - A4 clock T: 1 = 4 MHz / 250 kbit/s (5.25"), 0 = 8 MHz / 500 kbit/s (8")
        - A5 head load (no effect, the drives have no head load solenoid)
        - A6 TC, ORed with the inverted DMA interrupt (gates 1D2/7D4)
        - A7 input RESI: high after power-on reset, low after operator reset
          (manual 3.2.2); the monitor takes the warm start path on low.  The
          "Hardware Test" DIP switch stands in for it.
        - port B 0-3 select /DS0-3 (the FDC US lines decide here, the drive
          select is not modelled separately), B4 input FDC INT (also reaches
          the CPU through the DMA chain buffer 2D3), B5/B6 write
          precompensation (not modelled), B7 reset FDC (RFDC)
        Drives 2 and 3 and 8" drives are not present.

    16 bit board memory: 16 KiB EPROM (0000-3FFF) and 2 KiB static RAM
    (4000-47FF, mirrored up to 5FFF) of segment 0 while the on-board memory is
    active, translated by the three MMUs (code, data, stack); the main memory
    (1 MiB by default, up to 8 MiB with -ram) starts at 8000.

    Console and boot
      * The console is serial channel tty1 (SIO0 channel B, labelled CONSOLE on
        the 8 bit board).  Slot "rs232" holds the ADM-31 terminal by default
        ("-rs232 terminal" selects the generic terminal, "-rs232 null_modem"
        with "-bitb" socket connects a host program).  tty0 is the modem channel;
        tty2 and tty3 are further serial ports (slots "tty0", "tty2", "tty3").
        The 16 bit board adds tty4-tty7 (slots "tty4", "u_rs232" = tty5,
        "tty6", "tty7").
      * Power on: the 8 bit monitor MON8 runs its self test (only after a
        power-on reset: PIO2 port A bit 7 = 1, see the DSW input), prints
        "Press RETURN" and then the prompt ">".  RETURN boots the floppy disk
        (on p8000_16 the boot floppy starts the 16 bit side; on p8000 a UDOS
        system floppy runs on the Z80 alone).
      * p8000_16 : the 16 bit software monitor asks "Press NMI"; the NMI key
        (Pause) starts the 16 bit hardware check, after which the system boots
        (WEGA from the Winchester disk).  All input must be in uppercase.
      * The terminal takes every key while the keyboard is active (F3 is its
        VIDEO key, not the emulator reset); the RESET button of the computer is
        mapped to Ctrl+Pause.

    Memory size (16 bit board)
      * MAME's standard RAM option sets the main memory of the memory cards:
        "-ram 1m" (default), 2m, 4m or 8m.  The monitor reports it as
        "MAXSEG=<xx>", the number of the highest 64 KiB segment: 1m = 0F,
        4m = 3F, 8m = 7E.  WEGA reports the rest as "user memory size".
      * 8 MiB is the maximum.  The segmented Z8001 names 128 segments (7 bit
        segment number) of 64 KiB, i.e. 8 MiB, and the Z8010 MMUs deliver 24
        physical address bits (16 MiB).  16 MiB was tried: the WEGA kernel hangs
        in the memory sizing loop of main() (it writes 0x19 to each 256 byte
        click and stops at the first one that does not read back).
      * The last segment (127, physical 7F0000-7FFFFF) is not populated, also
        with 8 MiB.  The 16 MiB memory card leaves it out too ("8 MB minus
        64 KB can be used").  Test step 70 of MON16 (p.test.s, "Test auf
        externen Speicher") walks the segments upwards from 0, writes the
        segment number to offsets 8000 and FFFE of each one, reads it back, and
        stops at the first segment where neither address reads back; the
        segment before it is the highest (MAX_SEGNR).  If every segment up to
        the end of the address range reads back, the segment number wraps
        around to the start and the scan never ends: the firmware hangs in its
        memory test.  The hole also ends the WEGA sizing loop.

    Installing WEGA 3.1 on an empty hard disk (software list "p8000_flop")
      The steps follow a run on the real machine and were repeated with this
      driver; the final "df -t" of the install log matches it block for block.
      Console input needs 7E1: every character typed at the WEGA login and in
      the shell must carry even parity.
      1. Create an empty image of at least 47185920 bytes (1024 cyl x 5 heads x
         18 sectors x 512 bytes), e.g. a CHD with
           chdman createhd -o wega.chd -chs 1024,5,18 -ss 512
         (a raw file made with "truncate -s 48M wega.hd" works as well).  The
         geometry is not read from the image; it is stored on the disk by
         sa.format.  Start "mame p8000_16 -flop1 w31start.cqm -hard wega.chd
         -rs232 null_modem -bitb socket.127.0.0.1:5620" (the floppy images are
         in the software list p8000_flop).  The WEGA start floppy always stays
         in the first drive, the installation floppies go into the second.
      2. Return at the 8 bit monitor ("Press RETURN", then ">" and Return) boots
         the floppy and the 16 bit monitor asks "Press NMI".  Do not press NMI:
         the hardware test would stop with error 52, because the first command
         to the unformatted disk fails.  Type " O U" (any character stops the
         autoboot, O U boots the UDOS floppy).  At the loader prompt ">" enter
         "boot" and at "Boot :" the program, e.g. "ud(0,0)sa.format".
      3. sa.format: disk type 4 (ROB K5504.50: 1024 cylinders, 5 heads, 18
         sectors), then answers 4, y, n, a, y, y, y.
      4. sa.verify: begin cylinder 0, end cylinder 1023 (it waits for both).
      5. sa.mkfs: "13000" for md(0,0) (/usr) and "7000" for md(0,16000) (root).
      6. sa.install, started twice, always from the start floppy in drive 0.
         Date 05/30/89.  Insert the installation floppies in drive 1 only after
         sa.install is running (a foreign disk in drive 1 at boot time makes
         UDOS stop with error C4).  Input fd(1,0) (block 0 of floppy 1).
         First run: output md(0,16000), disks root1-root5.  Second run: output
         md(0,0), disks usr1-usr9.  Answer A at the directory prompts, change
         the disk, and answer y to "next input disk" for every further disk and
         n after the last one.
      7. Boot the kernel by hand once ("O U", "boot", "md(0,16000)wega"), because
         the disk has no boot program yet.  WEGA starts in single-user mode.
         Run /etc/new.install: answer j, 4000 for /dev/tmp, 60732 for /dev/z
         (it builds lost+found and the manual entries and runs for a long time).
         Then "init 2"; datem asks for the date (05/30/89) and time.  The
         message "Last known date" shows the U130 clock module's time (the
         module is always present, see p8k_u130.cpp).
      8. Log in as wega (password root), "cd /usr/sys/conf" and run "sysgen":
         n (no new constants), y (new layout), 13000 /usr, 3000 /swap, 7000
         root, 4000 /tmp, 65016 /z, Return for md5, n (no second disk), n (no
         new version parameters), n (no new driver).  Then
           mv /usr/sys/conf/wega /wega
           mv /usr/sys/conf/boot /usr/boot
           chmod 400 /wega /usr/boot
           halt
         and "sync;sync" in the single-user shell that follows.
         The new boot program has "md(0,16000)wega" as its default.
      9. Reset the machine and boot as usual: Return at the monitor, then only
         NMI.  After the hardware test the boot program starts the kernel from
         the disk.  After the file system checks and a login: "umount /dev/z",
         "mkfs /dev/rz 65016", "/etc/labelit /dev/rz /z 1", "mount /dev/z /z",
         "/etc/makelost+found /z", "rm -f /etc/new.install" and "df -t".

    8 bit monitor MON8 commands (U880SM.S, upper/lower case)
      * D : display and modify memory       * F : fill memory
      * G : go to / get                     * M : move (copy) memory
      * P : read/write an I/O port          * R : display and modify registers
      * B : set a break point               * N : execute the next instruction(s)
      * Q : quit (return to the program)    * O, S, return : boot from floppy
      * X : start the 16 bit board: copies the coupling code from ROM
            address 0DB00h to F000h, switches in RAM, releases the U8000 reset
            through the coupling PIO and continues in the monitor ("XR" only
            copies and runs it)
      * T : 8 bit self test ("Eigentest", U880EIG.S), jump to 0CEF0h

****************************************************************************/

#include "emu.h"

#include "p8k_term.h"
#include "p8k_u130.h"
#include "p8k_wdc.h"

#include "bus/rs232/rs232.h"
#include "cpu/z8/z8.h"
#include "cpu/z80/z80.h"
#include "cpu/z8000/z8000.h"
#include "imagedev/floppy.h"
#include "imagedev/harddriv.h"
#include "machine/clock.h"
#include "machine/ram.h"
#include "machine/upd765.h"
#include "machine/z8010.h"
#include "machine/z80ctc.h"
#include "machine/z80daisy.h"
#include "machine/z80dma.h"
#include "machine/z80pio.h"
#include "machine/z80sio.h"
#include "sound/beep.h"
#include "video/i8275.h"

#include "input.h"
#include "speaker.h"


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

namespace {

// The DMA controller of the 8 bit board reads back one less than the number of
// bytes transferred (needed by the monitor's hardware self-test).
class p8k_dma_device : public ua858d_device
{
public:
	p8k_dma_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

protected:
	virtual u16 byte_counter() const override { return m_byte_counter - 1; }
};

} // anonymous namespace

DEFINE_DEVICE_TYPE_PRIVATE(P8K_DMA, ua858d_device, p8k_dma_device, "p8k_dma", "P8000 DMA controller")

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
		, m_sio8_0(*this, "sio")
		, m_sio8_1(*this, "sio1")
		, m_ctc8(*this, "ctc%u", 0U)
		, m_pio1(*this, "pio1")
		, m_dma(*this, "dma")
		, m_u_pio0(*this, "u_pio0")
		, m_u_pio1(*this, "u_pio1")
		, m_u_pio2(*this, "u_pio2")
		, m_wdc(*this, "wdc")
		, m_u130(*this, "u130")
		, m_i8272(*this, "i8272")
		, m_ramdev(*this, RAM_TAG)
		, m_local_ram(*this, "local_ram", 0x800, ENDIANNESS_BIG)
		, m_adp_view{{*this, "adp0"}, {*this, "adp1"}, {*this, "adp2"}, {*this, "adp3"}, {*this, "adp4"}, {*this, "adp5"}, {*this, "adp6"}, {*this, "adp7"}, {*this, "adp8"}, {*this, "adp9"}, {*this, "adp10"}, {*this, "adp11"}, {*this, "adp12"}, {*this, "adp13"}, {*this, "adp14"}, {*this, "adp15"}}
		, m_dram8(*this, "dram8", 0x10000, ENDIANNESS_LITTLE)
		, m_view_code(*this, "view_code")
		, m_view_data(*this, "view_data")
		, m_view_stck(*this, "view_stck")
		, m_mmu_code(*this, "mmu_code")
		, m_mmu_data(*this, "mmu_data")
		, m_mmu_stck(*this, "mmu_stck")
		, m_run_led(*this, "run16")
	{ }

	void p8k(machine_config &config);
	void p8k_16(machine_config &config);

	DECLARE_INPUT_CHANGED_MEMBER(nmi_button);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	uint8_t port0_r(offs_t offset);
	void port0_w(offs_t offset, uint8_t data);
	void adp_update(int slice);
	void install_memory8();

	void fdc_irq(int state);
	void fdc_motor_w(uint8_t data);
	void fdc_reset_w(uint8_t data);
	void p8k_daisy_interrupt(int state);
	void p8k_dma_irq_w(int state);
	void p8k_16_daisy_interrupt(int state);
	void vi_deliver();
	TIMER_CALLBACK_MEMBER(vi_timer_cb);
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

	void addrmap_program(address_map &map) ATTR_COLD { map.unmap_value_low(); }
	void addrmap_data(address_map &map) ATTR_COLD { map.unmap_value_low(); }
	void addrmap_stack(address_map &map) ATTR_COLD { map.unmap_value_low(); }
	void addrmap_io(address_map &map) ATTR_COLD;
	void addrmap_sio(address_map &map) ATTR_COLD;

	z8010_device *select_code_mmu(offs_t offset);
	z8010_device *select_data_mmu(offs_t offset, uint8_t nbr);

	bool translate_addr(int spacenum, bool write, offs_t &offset);
	bool ram_hole(offs_t offset) const;
	uint16_t ram16_r(offs_t offset, uint16_t mask);
	void ram16_w(offs_t offset, uint16_t data, uint16_t mask);
	uint16_t ram_r(address_space &space, offs_t offset, uint16_t mask);
	void ram_w(address_space &space, offs_t offset, uint16_t data, uint16_t mask);

	uint16_t nmiack_r();
	void run_led_off_w(uint8_t data) { m_run_led = 0; }
	void run_led_on_w(uint8_t data) { m_run_led = 1; }
	uint16_t segtack_r();
	uint16_t viack_r();
	uint16_t nviack_r();
	void mmu_instruction_end();

	void segt_interrupt(int state);

	void soft_reset_w(uint8_t dummy);

	void install_memory(offs_t lrom_end, offs_t lram_start, offs_t lmem_end);
	void baud_w(unsigned channel, int state);
	uint16_t stck_lram_r(offs_t offset, uint16_t mem_mask);
	void stck_lram_w(offs_t offset, uint16_t data, uint16_t mem_mask);

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
	required_device<z80sio_device> m_sio8_0;      // 8 bit board SIOs
	required_device<z80sio_device> m_sio8_1;
	required_device_array<z80ctc_device, 2> m_ctc8;
	required_device<z80pio_device> m_pio1;
	required_device<z80dma_device> m_dma;
	emu_timer *m_vi_timer = nullptr;              // fallback for the delayed VI request
	bool m_vi_pending = false;                    // VI request waits for the next instruction fetch
	bool m_vi_asserted = false;                   // VI line state seen by the CPU
	bool m_baud_ff[8] = { };                      // divide by two flip-flops behind the CTC zero count outputs
	optional_device<z80pio_device> m_u_pio0;      // 16 bit board: coupling PIO (to 8 bit)
	optional_device<z80pio_device> m_u_pio1;      // 16 bit board: coupling PIO (from 8 bit)
	optional_device<z80pio_device> m_u_pio2;      // 16 bit board: WDC
	optional_device<p8k_wdc_device> m_wdc;
	optional_device<p8k_u130_device> m_u130;
	optional_device<i8272a_device> m_i8272;
	bool m_dma_int = false;
	bool m_fdc_tc = false;

	// 16-bit board
	optional_device<ram_device> m_ramdev;         // external memory (default 1 MiB, "-ram" up to 8 MiB)
	memory_share_creator<uint16_t> m_local_ram;   // on-board static RAM, 2 KiB at 4000-47FF (mirrored up to 5FFF)
	// 8 bit board memory control (ADP + reset flip-flop), one view per 4K slice
	memory_view m_adp_view[16];
	memory_share_creator<uint8_t> m_dram8; // 64 KiB dynamic RAM
	uint8_t m_sram8[0x800];               // 2 KiB static RAM
	uint8_t m_adp[16];                    // address port: 16 words of 4 bit
	bool m_rff;                           // reset flip-flop
	memory_view m_view_code;
	memory_view m_view_data;
	memory_view m_view_stck;
	optional_device<z8010_device> m_mmu_code;
	optional_device<z8010_device> m_mmu_data;
	optional_device<z8010_device> m_mmu_stck;
	output_finder<> m_run_led;                    // RUN lamp of the 16 bit board
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
	uint8_t m_reg_sbr   = 0; // System Break Register (not used by the MMU selection: the OS is always segmented)
	uint8_t m_reg_nbr   = 0; // Normal Break Register
	uint8_t m_reg_trpl  = 0; // Segment trap memory address low-byte
	uint8_t m_reg_if1l  = 0; // Segment trap instruction low-byte

	// Running bus-side latches and the SEGT- line state
	uint8_t m_lad_low   = 0; // low address byte of the current memory cycle
	uint8_t m_if1_low   = 0; // low address byte of the last IFETCH1 cycle
	bool m_segt_state   = false;

	bool m_is_seg_os = true;    // the P8000 operating system is always segmented
	bool m_is_seg_user = false;

	bool m_normal_mode = false;
	bool m_dma_on = false;
};

/***************************************************************************

    P8000 8bit

****************************************************************************/

void p8k_state::p8k_memmap(address_map &map)
{
	// the 8 bit board memory is installed in install_memory8() (16 views of 4K)
}

void p8k_state::p8k_iomap(address_map &map)
{
	map.global_mask(0xff);
	map(0x00, 0x07).rw(FUNC(p8k_state::port0_r), FUNC(p8k_state::port0_w)); // MH7489
	map(0x08, 0x0b).rw(m_ctc8[0], FUNC(z80ctc_device::read), FUNC(z80ctc_device::write));
	map(0x0c, 0x0f).rw(FUNC(p8k_state::pio0_r), FUNC(p8k_state::pio0_w));
	map(0x10, 0x10).w(FUNC(p8k_state::p16a_w));   // data to the 16 bit board
	map(0x14, 0x14).w(FUNC(p8k_state::p16b_w));   // status/strobe/INT to the 16 bit board
	map(0x18, 0x1b).rw(m_pio1, FUNC(z80pio_device::read_alt), FUNC(z80pio_device::write_alt));
	map(0x1c, 0x1f).rw(m_pio2, FUNC(z80pio_device::read_alt), FUNC(z80pio_device::write_alt));
	map(0x20, 0x21).m(m_i8272, FUNC(i8272a_device::map));
	map(0x24, 0x27).rw(m_sio8_0, FUNC(z80sio_device::ba_cd_r), FUNC(z80sio_device::ba_cd_w));
	map(0x28, 0x2b).rw(m_sio8_1, FUNC(z80sio_device::ba_cd_r), FUNC(z80sio_device::ba_cd_w));
	map(0x2c, 0x2f).rw(m_ctc8[1], FUNC(z80ctc_device::read), FUNC(z80ctc_device::write));
	map(0x3c, 0x3c).rw(m_dma, FUNC(z80dma_device::read), FUNC(z80dma_device::write));
}



uint8_t p8k_state::port0_r(offs_t offset)
{
	// ADP is write-only; any access to RES_RFF (04H) resets the reset flip-flop
	if (!machine().side_effects_disabled() && (offset & 4) && m_rff)
	{
		m_rff = false;
		for (int i = 0; i < 16; i++)
			adp_update(i);
	}
	return 0;
}

// see memory explanation above
void p8k_state::port0_w(offs_t offset, uint8_t data)
{
	if (offset & 4)
	{
		// RES_RFF: a dummy output releases the ADP (only possible once after reset)
		port0_r(offset);
		return;
	}

	// WEADP: the slice is selected by the high address byte (register B) of OUT (C),r
	const int slice = m_maincpu->state_int(Z80_B) >> 4;
	data &= 7;
	if (data > 4 || data == 3)
		logerror("Invalid ADP control word %X for slice %d\n", data, slice);
	m_adp[slice] = data;
	adp_update(slice);
}

void p8k_state::adp_update(int slice)
{
	// 0 = nothing selected, 1 = EPROM, 2 = SRAM, 4 = DRAM
	// while the reset flip-flop is set, the ADP is blocked and only the EPROM is active
	int sel = 0;
	if (m_rff || m_adp[slice] == 1)
		sel = 1;
	else if (m_adp[slice] == 2)
		sel = 2;
	else if (m_adp[slice] == 4)
		sel = 3;
	m_adp_view[slice].select(sel);
}

void p8k_state::install_memory8()
{
	address_space &space = m_maincpu->space(AS_PROGRAM);
	uint8_t *rom = memregion("maincpu")->base();

	std::fill(std::begin(m_sram8), std::end(m_sram8), 0);
	std::fill(std::begin(m_adp), std::end(m_adp), 0);
	m_rff = true;

	for (int i = 0; i < 16; i++)
	{
		const offs_t start = i * 0x1000;
		const offs_t end = start + 0xfff;

		space.install_view(start, end, m_adp_view[i]);

		// no select signal active: nothing responds
		m_adp_view[i][0].nop_readwrite(start, end);
		// EPROM: 8 KiB (2 x 2732), A12 only decoded, repeating through the address space
		// (the monitor copies the coupling code from 0DB00h)
		m_adp_view[i][1].install_rom(start, end, &rom[(i & 1) * 0x1000]);
		// SRAM: 2 KiB, A11 not decoded, so it appears twice in the 4K window
		m_adp_view[i][2].install_ram(start, start + 0x7ff, 0x800, m_sram8);
		// DRAM: 64 KiB
		m_adp_view[i][3].install_ram(start, end, &m_dram8[start]);

		m_adp_view[i].select(1);
	}

	save_item(NAME(m_sram8));
	save_item(NAME(m_adp));
	save_item(NAME(m_rff));
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
	m_dma_int = state;
	m_i8272->tc_w(m_dma_int || m_fdc_tc);
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
	// the DMA reads the FDC data register with DACK, which is not the CPU access to the FIFO
	if (offset == 0x21)
		return m_i8272->dma_r();
	address_space& prog_space = m_maincpu->space(AS_IO);
	return prog_space.read_byte(offset);
}

void p8k_state::io_write_byte(offs_t offset, uint8_t data)
{
	if (offset == 0x21)
	{
		m_i8272->dma_w(data);
		return;
	}
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

// PIO2 port B bit 7: RFDC, resets the FDC while high
void p8k_state::fdc_reset_w(uint8_t data)
{
	m_i8272->reset_w(BIT(data, 7));
}

void p8k_state::fdc_irq(int state)
{
	m_pio2->port_b_write(state ? 0x10 : 0x00);
}

// PIO2 port A bits 0-3: /LCK0../LCK3, motor of drive 0-3 (low = on)
// bit 4: clock/data rate (1 = 4 MHz, 5.25 inch, 250 kbit/s; 0 = 8 MHz, 8 inch, 500 kbit/s)
// bit 6: TC, ORed with the inverted DMA interrupt (gates 1D2/7D4)
void p8k_state::fdc_motor_w(uint8_t data)
{
	m_i8272->set_rate(BIT(data, 4) ? 250000 : 500000);
	m_fdc_tc = BIT(data, 6);
	m_i8272->tc_w(m_dma_int || m_fdc_tc);
	for (int i = 0; i < 2; i++)
	{
		floppy_connector *const conn = m_i8272->subdevice<floppy_connector>(util::string_format("%d", i));
		if (conn && conn->get_device())
			conn->get_device()->mon_w(BIT(data, i));
	}
}

void p8k_floppies(device_slot_interface &device)
{
	device.option_add("525qd", FLOPPY_525_QD);   // 5.25 inch, double sided, double density, 96 tpi (hardware manual 3.1)
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
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("NMI / RESET (with Ctrl)") PORT_CODE(KEYCODE_PAUSE) PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(p8k_state::nmi_button), 0)
INPUT_PORTS_END

INPUT_CHANGED_MEMBER(p8k_state::nmi_button)
{
	// Ctrl+Pause is the RESET button; the keyboard of the console terminal takes all keys,
	// so the reset key of the emulator (F3) does not reach MAME
	input_manager &input = machine().input();
	if (newval && (input.code_pressed(KEYCODE_LCONTROL) || input.code_pressed(KEYCODE_RCONTROL)))
	{
		machine().schedule_soft_reset();
		return;
	}

	// with a 16 bit board the NMI key goes to the board that is currently released
	// (PIO0 B7 of the 8 bit board = /NMI_UM)
	if (m_cpu16 && !m_u8000_held)
		m_cpu16->set_input_line(INPUT_LINE_NMI, newval ? ASSERT_LINE : CLEAR_LINE);
	else
		m_maincpu->set_input_line(INPUT_LINE_NMI, newval ? ASSERT_LINE : CLEAR_LINE);
}


void p8k_state::machine_reset()
{
	if (m_cpu16)
		board16_reset();

	m_rff = true;
	std::fill(std::begin(m_adp), std::end(m_adp), 0);
	for (int i = 0; i < 16; i++)
		adp_update(i);
}

/***************************************************************************

    P8000 16bit

    The 16 bit board is a copy of the System 8000 CPU-A board
    (src/devices/bus/zbi/s8k_cpu.cpp) with minimal changes: there is no ZBI
    bus, the memory sits on the board, and the peripherals are two SIOs,
    two CTCs and three PIOs. The names and the order of the functions follow
    that file.

***************************************************************************/

constexpr XTAL CLK_CPU = XTAL(4'000'000);
constexpr XTAL CLK_CTC = XTAL(9'832'000) / 8;   // 1.229 MHz: CTC trigger input (baud rate generators, real-time clock)

// System Configuration Register control bits 0-3
constexpr uint8_t SCR_BD_MEMON   = 0x01;    // Use off-board memory (disable monitor ROM and local RAM)
constexpr uint8_t SCR_MMU_ONH    = 0x02;    // Enable MMUs (disable linear memory)
constexpr uint8_t SCR_SEG_USER   = 0x04;    // Segmented user space
// SCR bit 3 (0x08) enables the NMI from parity or ECC errors (not emulated)

// The zero count output of a baud rate CTC channel drives a divide by two
// flip-flop whose output is the SIO's transmit and receive clock (hardware
// manual, "Teilerkette zur Baud-Raten Generierung").
// 0-3: 16 bit board SIO0 A/B, SIO1 A/B; 4-7: 8 bit board SIO A/B, SIO1 A/B
void p8k_state::baud_w(unsigned channel, int state)
{
	if (!state)
		return;

	m_baud_ff[channel] = !m_baud_ff[channel];
	int const level = m_baud_ff[channel] ? 1 : 0;

	z80sio_device &sio = (channel < 4) ? *m_sio[channel >> 1] : ((channel < 6) ? *m_sio8_0 : *m_sio8_1);
	if (BIT(channel, 0))
	{
		sio.rxcb_w(level);
		sio.txcb_w(level);
	}
	else
	{
		sio.rxca_w(level);
		sio.txca_w(level);
	}
}

void p8k_state::machine_start()
{
	install_memory8();
	save_item(NAME(m_baud_ff));
	save_item(NAME(m_dma_int));
	save_item(NAME(m_fdc_tc));

	if (m_mmu_code)
	{
		install_memory(0x3fff, 0x4000, 0x7fff);

		m_vi_timer = timer_alloc(FUNC(p8k_state::vi_timer_cb), this);
		save_item(NAME(m_vi_pending));
		save_item(NAME(m_vi_asserted));
		save_item(NAME(m_reg_scr));
		save_item(NAME(m_reg_sbr));
		save_item(NAME(m_reg_nbr));
		save_item(NAME(m_reg_trpl));
		save_item(NAME(m_reg_if1l));
		save_item(NAME(m_lad_low));
		save_item(NAME(m_if1_low));
		save_item(NAME(m_segt_state));
		save_item(NAME(m_normal_mode));
		save_item(NAME(m_is_seg_user));
		save_item(NAME(m_p16a));
		save_item(NAME(m_p16b));
		save_item(NAME(m_ardy0));
		save_item(NAME(m_ardy1));
		save_item(NAME(m_u_pio0_writing));
		save_item(NAME(m_u_pio0_a_bus));
		save_item(NAME(m_u8000_held));
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
	m_vi_asserted = false;
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
	// NMI identifier (low nibble of the low byte, hardware manual table 3.7-4):
	// the NMI key is the only source that is emulated, no parity errors or power fail
	return 0x0001;
}

void p8k_state::soft_reset_w(uint8_t dummy)
{
	// resets the 16 bit board only, the 8 bit board keeps running
	if (m_cpu16)
	{
		// the devices are not reset by the machine here
		m_ctc[0]->reset();
		m_ctc[1]->reset();
		m_sio[0]->reset();
		m_sio[1]->reset();
		m_u_pio0->reset();
		m_u_pio1->reset();
		m_u_pio2->reset();
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
	uint8_t seg = uint8_t(offset >> 16);

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
	uint8_t seg = uint8_t(offset >> 16);
	uint8_t seg_offs = uint8_t(offset >> 8);

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

// Segment 127, the last 64 KiB block (7F0000-7FFFFF) of the 8 MiB a segmented CPU can address,
// has no memory.  The 16 MiB memory card of the P8000 leaves it out as well (its address
// decoder gates 7FH, "8 MB minus 64 KB can be used"), so that the memory scan of MON16 (test
// step 70) finds the end of the memory instead of running around the segment numbers forever,
// and so that the WEGA kernel's sizing loop (main.c) stops there.
bool p8k_state::ram_hole(offs_t offset) const
{
	return offset >= 0x7f'0000 && offset < 0x80'0000;
}

uint16_t p8k_state::ram16_r(offs_t offset, uint16_t mask)
{
	// offset is the byte address of the memory bus
	if (offset >= m_ramdev->size() || ram_hole(offset))
		return 0xffff;
	uint8_t const *const p = m_ramdev->pointer() + (offset & ~1);
	return (p[0] << 8) | p[1];
}

void p8k_state::ram16_w(offs_t offset, uint16_t data, uint16_t mask)
{
	if (offset >= m_ramdev->size() || ram_hole(offset))
		return;
	uint8_t *const p = m_ramdev->pointer() + (offset & ~1);
	uint16_t const old = (p[0] << 8) | p[1];
	uint16_t const val = (old & ~mask) | (data & mask);
	p[0] = val >> 8;
	p[1] = val & 0xff;
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
	LRAM = m_local_ram.target();
	// The static RAM is built from four 2114 (1K x 4), i.e. 2 KiB; the chip select
	// decodes 8 KiB, so the RAM repeats up to 5FFF and nothing answers at 6000-7FFF.
	offs_t const lram_end = lram_start + 0x7ff;
	offs_t const lram_mirror = 0x1800;
	m_view_code[0].install_rom(0x0000, lrom_end, LROM);
	m_view_code[0].install_ram(lram_start, lram_end, lram_mirror, LRAM);
	m_view_data[0].install_rom(0x0000, lrom_end, LROM);
	m_view_data[0].install_ram(lram_start, lram_end, lram_mirror, LRAM);
	// every stack memory request, in the local RAM too, is counted by CTC0
	m_view_stck[0].install_readwrite_handler(lram_start, lram_end, 0, lram_mirror, 0,
		read16s_delegate(*this, FUNC(p8k_state::stck_lram_r)), write16s_delegate(*this, FUNC(p8k_state::stck_lram_w)));

	// with the on-board memory active the main memory starts at 8000: the rest
	// of the on-board window is empty
	offs_t const empty_start = lram_start + 0x2000;
	m_view_code[0].nop_readwrite(empty_start, lmem_end);
	m_view_data[0].nop_readwrite(empty_start, lmem_end);
	m_view_stck[0].nop_readwrite(empty_start, lmem_end);
}

uint16_t p8k_state::stck_lram_r(offs_t offset, uint16_t mem_mask)
{
	if (!machine().side_effects_disabled())
	{
		m_ctc[0]->trg3(1);
		m_ctc[0]->trg3(0);
	}
	return m_local_ram[offset];
}

void p8k_state::stck_lram_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	m_ctc[0]->trg3(1);
	m_ctc[0]->trg3(0);
	COMBINE_DATA(&m_local_ram[offset]);
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

	m_normal_mode = bool(state);
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
		uint8_t reg = uint8_t(offset >> 8);

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
		uint8_t reg = uint8_t(offset >> 8);

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

	// every stack memory request with data strobe pulses CLK/TRG3 of CTC0
	// (IDS- OR STACKMEMREQ-); WEGA counts these for single stepping
	if (side_effects && stack_access)
	{
		m_ctc[0]->trg3(1);
		m_ctc[0]->trg3(0);
	}

	bool const code_access = (spacenum == AS_PROGRAM);
	if (side_effects && code_access && m_vi_pending && m_cpu16->is_ifetch1())
		vi_deliver();

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
	m_normal_mode = false;
	m_segt_state = false;
	m_p16a = m_p16b = 0;
	m_u_pio0_a_bus = 0;
	m_ardy0 = m_ardy1 = false;
	m_view_code.select(0);
	m_view_data.select(0);
	m_view_stck.select(0);
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
	map(0x0ffb8, 0x0ffb9).w(FUNC(p8k_state::run_led_off_w)).umask16(0x00ff);   // LEDAUS: RUN lamp off
	map(0x0ffc0, 0x0ffc1).rw(FUNC(p8k_state::reg_scr_r), FUNC(p8k_state::reg_scr_w)).umask16(0x00ff);
	map(0x0ffc8, 0x0ffc9).rw(FUNC(p8k_state::reg_sbr_r), FUNC(p8k_state::reg_sbr_w)).umask16(0x00ff);
	map(0x0ffd0, 0x0ffd1).rw(FUNC(p8k_state::reg_nbr_r), FUNC(p8k_state::reg_nbr_w)).umask16(0x00ff);
	map(0x0ffd8, 0x0ffd9).w(FUNC(p8k_state::run_led_on_w)).umask16(0x00ff);   // LEDEIN: RUN lamp on
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
	// Ignore the sent device state and forward the daisy chain state instead.
	// A request that arrives while an instruction runs is only seen by the CPU
	// once the next instruction has started: the single step logic relies on
	// that (CTC0 channel 3 expires during the IRET that enters the program, and
	// the program's first instruction must still be executed). A timer covers
	// a CPU that sits in HALT and fetches nothing.
	if (m_daisy->update_irq_state())
	{
		if (!m_vi_asserted && !m_vi_pending)
		{
			m_vi_pending = true;
			m_vi_timer->adjust(attotime::from_ticks(64, CLK_CPU));
		}
	}
	else
	{
		m_vi_pending = false;
		m_vi_asserted = false;
		m_vi_timer->reset();
		m_cpu16->set_input_line(z8001_device::VI_LINE, CLEAR_LINE);
	}
}

void p8k_state::vi_deliver()
{
	m_vi_pending = false;
	m_vi_asserted = true;
	m_vi_timer->reset();
	m_cpu16->set_input_line(z8001_device::VI_LINE, ASSERT_LINE);
}

TIMER_CALLBACK_MEMBER(p8k_state::vi_timer_cb)
{
	if (m_vi_pending)
		vi_deliver();
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

// the console runs in ADM-31 mode, as the terminal of the P8000 does after power on
void p8k_console_devices(device_slot_interface &device)
{
	default_rs232_devices(device);
	device.option_add("adm31", P8K_TERMINAL);
}

void p8k_state::p8k(machine_config &config)
{
	/* basic machine hardware */
	z80_device& maincpu(Z80(config, "maincpu", 16_MHz_XTAL / 4));
	maincpu.set_daisy_config(p8k_daisy_chain);
	maincpu.set_addrmap(AS_PROGRAM, &p8k_state::p8k_memmap);
	maincpu.set_addrmap(AS_IO, &p8k_state::p8k_iomap);

	/* peripheral hardware */
	ua858d_device &dma(P8K_DMA(config, m_dma, 16_MHz_XTAL / 4));
	// DMA BUSREQ drives the Z80 BUSREQ, Z80 BUSACK returns to the DMA's BAI
	dma.out_busreq_callback().set_inputline(m_maincpu, Z80_INPUT_LINE_BUSREQ);
	maincpu.busack_cb().set(m_dma, FUNC(z80dma_device::bai_w));
	dma.out_int_callback().set(FUNC(p8k_state::p8k_dma_irq_w));
	dma.in_mreq_callback().set(FUNC(p8k_state::memory_read_byte));
	dma.out_mreq_callback().set(FUNC(p8k_state::memory_write_byte));
	dma.in_iorq_callback().set(FUNC(p8k_state::io_read_byte));
	dma.out_iorq_callback().set(FUNC(p8k_state::io_write_byte));

	z80ctc_device &ctc0(Z80CTC(config, m_ctc8[0], 1229000));    /* 1.22MHz clock */
	// Baud rate generators: the baud rate oscillator is the trigger input of the CTC
	// channels, their zero count outputs clock the SIO channels (WEGA sio.h):
	//   CTC1 ch0 = SIO0 A (tty0), ch1 = SIO0 B (tty1, console), ch2 = SIO1 A (tty2)
	//   CTC0 ch0 = SIO1 B (tty3); CTC0 also has the FDC and the system channel
	ctc0.intr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	ctc0.set_reload_on_reset(true); // required by the monitor's hardware self-test
	ctc0.set_clk<0>(CLK_CTC);
	ctc0.zc_callback<0>().set([this] (int state) { baud_w(7, state); });

	z80ctc_device &ctc1(Z80CTC(config, m_ctc8[1], 1229000));    /* 1.22MHz clock */
	ctc1.intr_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	ctc1.set_reload_on_reset(true);
	ctc1.set_clk<0>(CLK_CTC);
	ctc1.set_clk<1>(CLK_CTC);
	ctc1.set_clk<2>(CLK_CTC);
	ctc1.zc_callback<0>().set([this] (int state) { baud_w(4, state); });
	ctc1.zc_callback<1>().set([this] (int state) { baud_w(5, state); });
	ctc1.zc_callback<2>().set([this] (int state) { baud_w(6, state); });

	rs232_port_device &tty0(RS232_PORT(config, "tty0", default_rs232_devices, nullptr));
	tty0.rxd_handler().set(m_sio8_0, FUNC(z80sio_device::rxa_w));
	tty0.cts_handler().set(m_sio8_0, FUNC(z80sio_device::ctsa_w));
	tty0.dcd_handler().set(m_sio8_0, FUNC(z80sio_device::dcda_w));

	// tty1 is the system console
	rs232_port_device &rs232(RS232_PORT(config, "rs232", p8k_console_devices, "adm31"));
	rs232.rxd_handler().set(m_sio8_0, FUNC(z80sio_device::rxb_w));
	rs232.cts_handler().set(m_sio8_0, FUNC(z80sio_device::ctsb_w));


	z80sio_device &sio(Z80SIO(config, m_sio8_0, 16_MHz_XTAL / 4));
	sio.out_txda_callback().set(tty0, FUNC(rs232_port_device::write_txd));
	sio.out_dtra_callback().set(tty0, FUNC(rs232_port_device::write_dtr));
	sio.out_rtsa_callback().set(tty0, FUNC(rs232_port_device::write_rts));
	sio.out_txdb_callback().set(rs232, FUNC(rs232_port_device::write_txd));
	sio.out_dtrb_callback().set(rs232, FUNC(rs232_port_device::write_dtr));
	sio.out_rtsb_callback().set(rs232, FUNC(rs232_port_device::write_rts));
	sio.out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	rs232_port_device &tty2(RS232_PORT(config, "tty2", default_rs232_devices, nullptr));
	tty2.rxd_handler().set(m_sio8_1, FUNC(z80sio_device::rxa_w));
	tty2.cts_handler().set(m_sio8_1, FUNC(z80sio_device::ctsa_w));
	tty2.dcd_handler().set(m_sio8_1, FUNC(z80sio_device::dcda_w));
	rs232_port_device &tty3(RS232_PORT(config, "tty3", default_rs232_devices, nullptr));
	tty3.rxd_handler().set(m_sio8_1, FUNC(z80sio_device::rxb_w));
	tty3.cts_handler().set(m_sio8_1, FUNC(z80sio_device::ctsb_w));
	tty3.dcd_handler().set(m_sio8_1, FUNC(z80sio_device::dcdb_w));


	z80sio_device &sio1(Z80SIO(config, m_sio8_1, 16_MHz_XTAL / 4));
	sio1.out_txda_callback().set(tty2, FUNC(rs232_port_device::write_txd));
	sio1.out_dtra_callback().set(tty2, FUNC(rs232_port_device::write_dtr));
	sio1.out_rtsa_callback().set(tty2, FUNC(rs232_port_device::write_rts));
	sio1.out_txdb_callback().set(tty3, FUNC(rs232_port_device::write_txd));
	sio1.out_dtrb_callback().set(tty3, FUNC(rs232_port_device::write_dtr));
	sio1.out_rtsb_callback().set(tty3, FUNC(rs232_port_device::write_rts));
	sio1.out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	Z80PIO(config, m_pio0, 1229000);
	m_pio0->out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	z80pio_device &pio1(Z80PIO(config, m_pio1, 1229000));
	pio1.out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);

	Z80PIO(config, m_pio2, 1229000);
	m_pio2->out_int_callback().set_inputline(m_maincpu, INPUT_LINE_IRQ0);
	m_pio2->in_pa_callback().set_ioport("DSW");
	m_pio2->out_pa_callback().set(FUNC(p8k_state::fdc_motor_w));
	m_pio2->out_pb_callback().set(FUNC(p8k_state::fdc_reset_w));

	I8272A(config, m_i8272, 16_MHz_XTAL / 2, true);
	m_i8272->drq_wr_callback().set(m_dma, FUNC(z80dma_device::rdy_w));
	m_i8272->intrq_wr_callback().set(FUNC(p8k_state::fdc_irq));
	FLOPPY_CONNECTOR(config, "i8272:0", p8k_floppies, "525qd", floppy_image_device::default_mfm_floppy_formats);
	FLOPPY_CONNECTOR(config, "i8272:1", p8k_floppies, "525qd", floppy_image_device::default_mfm_floppy_formats);

	SOFTWARE_LIST(config, "flop_list").set_original("p8000_flop");

	/* sound hardware */
	SPEAKER(config, "mono").front_center();

	BEEP(config, "beeper", 3250).add_route(ALL_OUTPUTS, "mono", 0.5);
}

void p8k_state::p8k_16(machine_config &config)
{
	// the 16 bit system always consists of an 8 bit board and a 16 bit board
	// that are coupled through two PIOs
	p8k(config);

	// main memory of the memory cards: 1 MiB by default, "-ram 2m|4m|8m" for more (see the memory size notes above)
	RAM(config, m_ramdev).set_default_size("1M").set_extra_options("2M,4M,8M");

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
	m_ctc[0]->set_clk<0>(CLK_CTC);
	m_ctc[0]->set_clk<1>(CLK_CTC);
	m_ctc[0]->set_clk<2>(CLK_CTC);
	m_ctc[0]->zc_callback<0>().set([this] (int state) { baud_w(0, state); });
	m_ctc[0]->zc_callback<1>().set([this] (int state) { baud_w(1, state); });
	m_ctc[0]->zc_callback<2>().set([this] (int state) { baud_w(2, state); });

	Z80CTC(config, m_ctc[1], CLK_CPU).intr_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));
	m_ctc[1]->set_clk<0>(CLK_CTC);
	m_ctc[1]->zc_callback<0>().set([this] (int state) { baud_w(3, state); });
	m_ctc[1]->zc_callback<2>().set(m_ctc[1], FUNC(z80ctc_device::trg3));

	// Real-time (line) clock: CTC 1 channels 2 & 3 are cascaded and run in
	// COUNTER mode, counting the oscillator fed to the CTC trigger input.
	// WEGA programs ch2 /256 and ch3 /80 (event.s) -> a 60 Hz jiffy interrupt.
	// Feed ch2 only through trg2: configuring both its internal clock and this
	// external trigger would count the oscillator twice.
	clock_device &rtc_clk(CLOCK(config, "rtc_clk", CLK_CTC));
	rtc_clk.signal_handler().set(m_ctc[1], FUNC(z80ctc_device::trg2));

	rs232_port_device &tty5(RS232_PORT(config, "u_rs232", default_rs232_devices, nullptr));
	tty5.rxd_handler().set(m_sio[0], FUNC(z80sio_device::rxb_w));
	tty5.cts_handler().set(m_sio[0], FUNC(z80sio_device::ctsb_w));

	rs232_port_device &tty4(RS232_PORT(config, "tty4", default_rs232_devices, nullptr));
	tty4.rxd_handler().set(m_sio[0], FUNC(z80sio_device::rxa_w));
	tty4.cts_handler().set(m_sio[0], FUNC(z80sio_device::ctsa_w));
	tty4.dcd_handler().set(m_sio[0], FUNC(z80sio_device::dcda_w));

	Z80SIO(config, m_sio[0], CLK_CPU);
	m_sio[0]->out_txda_callback().set(tty4, FUNC(rs232_port_device::write_txd));
	m_sio[0]->out_dtra_callback().set(tty4, FUNC(rs232_port_device::write_dtr));
	m_sio[0]->out_rtsa_callback().set(tty4, FUNC(rs232_port_device::write_rts));
	m_sio[0]->out_txdb_callback().set(tty5, FUNC(rs232_port_device::write_txd));
	m_sio[0]->out_dtrb_callback().set(tty5, FUNC(rs232_port_device::write_dtr));
	m_sio[0]->out_rtsb_callback().set(tty5, FUNC(rs232_port_device::write_rts));
	m_sio[0]->out_int_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

	rs232_port_device &tty6(RS232_PORT(config, "tty6", default_rs232_devices, nullptr));
	tty6.rxd_handler().set(m_sio[1], FUNC(z80sio_device::rxa_w));
	tty6.cts_handler().set(m_sio[1], FUNC(z80sio_device::ctsa_w));
	tty6.dcd_handler().set(m_sio[1], FUNC(z80sio_device::dcda_w));
	rs232_port_device &tty7(RS232_PORT(config, "tty7", default_rs232_devices, nullptr));
	tty7.rxd_handler().set(m_sio[1], FUNC(z80sio_device::rxb_w));
	tty7.cts_handler().set(m_sio[1], FUNC(z80sio_device::ctsb_w));
	tty7.dcd_handler().set(m_sio[1], FUNC(z80sio_device::dcdb_w));

	// tty5 (SIO0 B) is the terminal of the 16 bit monitor when the 8 bit part is inactive
	Z80SIO(config, m_sio[1], CLK_CPU);
	m_sio[1]->out_txda_callback().set(tty6, FUNC(rs232_port_device::write_txd));
	m_sio[1]->out_dtra_callback().set(tty6, FUNC(rs232_port_device::write_dtr));
	m_sio[1]->out_rtsa_callback().set(tty6, FUNC(rs232_port_device::write_rts));
	m_sio[1]->out_txdb_callback().set(tty7, FUNC(rs232_port_device::write_txd));
	m_sio[1]->out_dtrb_callback().set(tty7, FUNC(rs232_port_device::write_dtr));
	m_sio[1]->out_rtsb_callback().set(tty7, FUNC(rs232_port_device::write_rts));
	m_sio[1]->out_int_callback().set(FUNC(p8k_state::p8k_16_daisy_interrupt));

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
ROM_START( p8000 )
	// MON8 3.1 of an 8 bit only P8000 (it does not try to talk to a 16 bit board): dumped from the EPROMs of a real machine,
	// there is no release of these on the EAW floppies
	ROM_REGION( 0x2000, "maincpu", 0 )
	ROM_LOAD("mon8_1_3.1_nur8bit",  0x0000, 0x1000, CRC(a8e97c26) SHA1(ff98f97308e5d6bea002600a8c0703642abbf0d8))
	ROM_LOAD("mon8_2_3.1_nur8bit",  0x1000, 0x1000, CRC(060bb1b7) SHA1(123222076615cddad945afea37471af2806446b5))
ROM_END

ROM_START( p8000_16 )
	// MON8 3.1 for the 8 bit board coupled with a 16 bit board (starts it): file names as on the EAW UDOS and WEGA start floppies
	// (MON8_1_3.1, MON8_2_3.1), MON16 as MON16_*_3.1_UDOS on the WEGA start floppy
	ROM_REGION( 0x2000, "maincpu", 0 )
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
COMP( 1989, p8000,  0,      0,      p8k,     p8k,   p8k_state, empty_init,   "EAW electronic Treptow", "P8000 (8bit Board)", MACHINE_SUPPORTS_SAVE)
COMP( 1989, p8000_16, p8000, 0,     p8k_16,  p8k,   p8k_state, empty_init,   "EAW electronic Treptow", "P8000 (8bit and 16bit Board)", MACHINE_SUPPORTS_SAVE)
