# ddr/p8k.cpp: Robotron/EAW P8000, now with the 16 bit board, boots WEGA

This moves the P8000 driver from `skeleton/` to `ddr/` and makes it work. The
16 bit board (Z8001) is emulated and coupled to the 8 bit board (Z80), the
Winchester disk controller and the clock module are there, and WEGA 3.2 boots
from a hard disk image to a login and a shell.

The driver needed small changes in three shared Z80 devices (`z80pio`, `z80ctc`,
`z80dma`). Those are the part that can affect other drivers, so they come first.

## 1. Shared devices

### z80pio: two fixes

Both were needed for the interrupt driven coupling of the two boards and for
WEGA's disk I/O. `Z80PIO` is used by a lot of drivers, so please look at these
two with care.

First, `z80daisy_irq_ack()` now only acknowledges a port whose interrupt is
enabled:

```diff
-		if (port.m_ip)
+		if (port.m_ie && port.m_ip)
```

`m_ip` can be set while the enable flip-flop is cleared (mode 3 match, strobe).
The port does not drive INT then, but the acknowledge still picked it, and on the
P8000 the CPU got the vector of a port that never asked for an interrupt. The
Z80 PIO spec says a pending request is only enabled onto the interrupt line once
the enable flag is set (4.3) and that an interrupt is generated if interrupts
are enabled and the mask condition is met (5.3). It never says outright that the
enable flag gates the vector, I derived that.

Second, a mode 3 match is latched even while the port is being serviced:

```diff
-		if (!m_match && match && !m_ius)
+		if (!m_match && match)
```

The mode 3 condition is edge triggered (5.3, "false to true"), so if it is
dropped while `m_ius` is set it never comes back. The 16 bit board signals INT8
to the 8 bit board this way, and one lost edge hung the WEGA kernel in its
coupling loop. Whether INT is really driven is decided later in
`check_interrupts()`, which already looks at `m_ius` and the daisy chain. The
spec does not talk about a new condition on a port that is in service, so this
rests on the quotes above and on the failure I saw.

### z80ctc: opt-in `set_reload_on_reset()`

```cpp
// After a software reset (STOP = 1) the down counter is continuously
// reloaded from the time constant, so a read returns the time constant and
// not the frozen count.
void set_reload_on_reset(bool reload) { m_reload_on_reset = reload; }
```

Off by default, only `p8k.cpp` turns it on, so nothing changes for other
drivers. I think it is how the real Zilog CTC behaves too (see below), but I
could not test other drivers, so I left the default alone.

The P8000 monitor's self test (`U880EIG.S`) starts each channel, resets it with
control word 3 and reads it back, and it expects exactly the time constant. The
CTC data sheets don't say what a read returns after a reset: the Zilog spec only
says the channel "stops counting", the U857D data sheet says the registers are
not changed (p. 18). The circuit description in the Zilog CTC technical manual
does explain it: a reset sets flip-flop FF8, which generates STOP (p. 37), and
"the down counter is loaded when it reaches a zero count or when the channel
stops counting or timing. Transistors 86 through 90 generate the load DC signal
whenever STOP or TO1 = 1" (p. 44, Fig. 4-15). So while STOP is set the counter
holds the time constant.

### z80dma: a virtual hook, no change in behaviour

`z80dma_device::byte_counter()` is new and virtual, and `ua858d_device` got a
protected constructor that takes a device type. Nothing changes for existing
users (Spectrum DMA cards included).

The P8000's UA858D reads back one less than the bytes transferred. That lives in
`p8k_dma_device` in `p8k.cpp`, which overrides `byte_counter()`. The U858D data
sheet describes it: the byte counter is only initialised after the first byte
(section 4, p. 14), one byte more than the block length is always transferred,
and in every row of tables 1 and 2 (p. 15) the counter reads the number of
transferred bytes minus one. The Zilog Z80 DMA spec only says one more byte than
the block length is transferred and says nothing about the counter value, and
since I don't claim the two chips are identical I kept the quirk in the P8000
driver. The monitor's DMA self test needs it.

There are no unit tests for these devices in MAME, so I didn't add any. The
changes were checked with the firmware self tests and the WEGA boot below.

## 2. The P8000

The P8000 was built by VEB Elektro-Apparate-Werke Berlin-Treptow (EAW) in the
GDR. It has an 8 bit board (U880 at 4 MHz, floppy controller, DMA, 64 KiB RAM,
8 KiB monitor EPROM, serial ports, two 5.25" drives) and optionally a 16 bit
board (U8001 with three U8010 MMUs, 16 KiB EPROM, up to four memory cards). The
boards talk to each other through PIOs, and a separate Z80 based Winchester
controller hangs on a PIO of the 16 bit board. OS/M (CP/M like) and UDOS (a
Z80-RIO clone) run on the 8 bit board, WEGA (a Zilog ZEUS clone, System III
UNIX) on the 16 bit board.

There are two machines: `p8000_8` (8 bit board only, formerly `p8000`) and
`p8000_16` (both boards, clone of `p8000_8`). Besides `p8k.cpp` there are
`p8k_wdc.cpp/.h` (Winchester controller) and `p8k_u130.cpp/.h` (clock module).
`mame.lst` is updated and `MACHINE_NOT_WORKING` is gone.

Several revisions of the boards exist. I followed the hardware manual: 8 bit
board Index 3 (`V: x3xx`, chapter 4; the memory control is the same on Index 1)
and 16 bit board Index 4 (`V: 4xxx`, chapter 7), both with monitor 3.1. An 8 bit
board Index 3 or higher only works with a 16 bit board Index 4 or higher. This
is also noted at the top of `p8k.cpp`.

### 8 bit board

The existing driver, fixed so that the firmware's hardware test passes (`T` at
the monitor prompt runs all steps). Two changes are worth mentioning.

The memory control now works as the manual describes it (3.3/4.3). The address
space is 16 slices of 4K, and each slice selects nothing, the EPROM (8 KiB,
repeating), the 2 KiB static RAM or the 64 KiB DRAM through the write-only
address port at I/O 00H (the slice is the high address byte of `OUT (C),r`). A11
is not decoded, so the SRAM shows up twice in its 4K window, and it is the same
2 KiB in every slice. After a reset only the EPROM is visible until a dummy
access to I/O 04H releases the address port. Before, every slice had its own 4K
of "static RAM" and a control word of 0 was ignored.

`p8000_8` uses its own monitor EPROMs (`mon8_1_3.1_nur8bit`,
`mon8_2_3.1_nur8bit`), because the normal ones try to start a 16 bit board that
isn't there. `p8000_16` keeps `mon8_1_3.1` and `mon8_2_3.1`. A UDOS system floppy
boots on `p8000_8` to the `%` prompt, and `STATUS`, `DATE`, `ECHO` and `CAT P=&`
work. A plain `CAT` prints `FILE NOT FOUND` on that floppy because it only lists
non-secret files and every file on it has the S property.

### 16 bit board

Z8001 with three Z8010 MMUs and the board's special logic (system/normal mode,
segmented user space, break registers), the on-board EPROM and RAM, two SIOs,
two CTCs, three PIOs with the Z80 style daisy chain, and the main memory. With
the NMI key (F12) at the "Press NMI" prompt the 16 bit monitor runs its hardware
test and then boots.

The 16 bit board is, apart from a few changes, the Zilog System 8000 CPU-A board
that `src/devices/bus/zbi/s8k_cpu.cpp` already emulates: same Z8001 + 3x Z8010
arrangement, same selection logic, same registers (FFC1 system configuration,
FFC9 system break, FFD1 normal break, FFE1 RETI port, FFE9 soft reset, FFF1/FFF9
segment trap, SIO/CTC/PIO from FF81). The MON16 sources use these addresses. So
I wrote that part of `p8k.cpp` to look like `s8k_cpu.cpp` (same function and
member names, same `SCR_*` bits, same structure of the MMU selection and address
translation), so a diff of the two files is meaningful and fixes can move across.
The interrupt handling follows `zbi.cpp`: the VI line forwards the state of the
whole daisy chain and is re-evaluated after an acknowledge and after RETI. The
earlier per-device handling let the clock interrupt hide a pending disk
interrupt.

Differences from the s8k board: no ZBI bus (the memory is on the board, and
there are two SIOs, two CTCs, three PIOs); the operating system is always
segmented, so only that branch of the MMU selection exists; no parity/ECC NMI
(no such memory in the P8000), no DMA mode and no segment violation register
(FFD9 is the RUN lamp here); `soft_reset_w` only resets the 16 bit board;
`nmiack_r` returns the NMI key's identifier (0001); the on-board RAM is 2 KiB
(mirrored); the stack accesses pulse CTC0 CLK/TRG3 for single stepping.

The system break register (FFC9) is a special case. The manual lists it only for
the development board and the schematic marks it "GLE", but the 16 bit monitor's
hardware test writes it and reads it back (error 84 otherwise), and WEGA writes
it too. It stays a plain register that the MMU selection ignores.

On-board memory: 16 KiB EPROM at 0000-3FFF, 2 KiB static RAM at 4000-47FF that
shows up four times up to 5FFF, 6000-7FFF empty, main memory from 8000 while
the on-board memory is active. Writes to FFD9 and FFB9 switch the RUN lamp
output `run16`.

### Coupling the boards

The 16 bit board's PIO0 sends to the 8 bit board (PIO0 port A, bit 0 of port B is
"INT8"), and the 8 bit board sends through two output latches to the 16 bit
board's PIO1. PIO0 port B bit 7 of the 8 bit board holds the 16 bit board in
reset. That is how the 8 bit monitor starts the 16 bit CPU (`X`) and how the 16
bit board's console output reaches the 8 bit board's terminal.

### Winchester controller

The real controller is a separate Z80 board. `p8k_wdc_device` emulates it in
software on PIO2 of the 16 bit board, only the protocol on that interface:
commands, parameter table, bad track table and error codes. It was derived from
signal captures of a real WDC 4.2. The disk is a `harddisk_image_device`
(`-hard`), and the controller's /ST0 line raises the interrupt WEGA's disk driver
uses.

The bad track table sits in the parameter sector as a byte count, up to 40
entries (cylinder, head) in ascending order and the end mark `ff ff ff` right
behind the last entry, like the WDC firmware does it. A track found bad while
formatting is entered in sorted order, bad tracks are skipped by the block and
sector commands (every requested track moves up by one for each bad track before
it), and a track in the table is not formatted.

### Clock module

`p8k_u130_device` is the quartz clock module on the 16 bit board (I/O FEF1 to
FEFF): a six digit LCD and keys that the 16 bit side reads digit by digit (see
`timer.c` in the WEGA kernel and `sa.timer`). The emulation returns the host's
UTC time. `sa.timer`'s correction sequence is accepted but the new value is
dropped, so setting the clock does nothing. Without the module the kernel asks
for the date at boot. The hardware (watch IC plus 4000 series buffers and a
4035 for the keys) is described at the top of `p8k_u130.cpp`.

### Real-time clock, single step, serial ports

The real-time interrupt comes from CTC1 channels 2 and 3, cascaded on the
1.2288 MHz clock (WEGA programs /256 and /80, so 60 Hz), wired like on the System
8000 board.

CTC0 channel 3 ("Single Step Steuerung" in the manual) counts stack memory
requests, each one pulses its CLK/TRG3 input. WEGA arms it with a time constant
of 17 to get an interrupt after one instruction, and the monitor's NEXT command
counts stack operations the same way. As in `s8k_cpu.cpp` the pulse is made in
the address translation. I did not test single stepping itself, only that
everything around it works.

Both boards have two SIOs, so there are eight terminal channels (WEGA `sio.h`:
tty0-3 on the 8 bit board, tty4-7 on the 16 bit board). The console is tty1 (SIO0
B of the 8 bit board; the 16 bit monitor reaches it through the coupling PIOs).
Each channel has an RS232 port (slots `tty0`, `rs232` = tty1, `tty2`, `tty3`,
`tty4`, `u_rs232` = tty5, `tty6`, `tty7`), and only the console has a terminal
by default. The baud rates come from the CTCs as in the System 8000. The manual
shows the chain: a 9.8304 MHz crystal divided by 8 feeds the CTC trigger inputs,
and a divide-by-two flip-flop behind each zero count output clocks the SIO
channel. With the SIO in x32 mode a time constant of 2 gives 9600 baud. Which
channel is a terminal and which a modem line is a software setting
(`/etc/ttyconfig`, `/etc/inittab`).

## 3. Testing

Everything was run headless with the WEGA SD card image (`-hard sd.hd -rs232
null_modem`).

- `p8000_8`: the hardware test runs through without an error.
- `p8000_16`: `X` at the 8 bit monitor starts the U8001, NMI starts the 16 bit
  hardware test, and the monitor boots WEGA (`md(0,16000)wega`).
- WEGA 3.2 (stock kernel) boots to multi-user mode, the date comes from the
  clock module and the clock keeps running, and login as `wega` works with
  `ls`, `who`, `date`.
- `sa.timer` finds the clock module, shows the host date and time and accepts a
  new one.
- Login prompts appear on tty0, tty1, tty2, tty5, tty6 and tty7, a full login on
  tty1 and tty2, and output works on all eight ports. `ttyconfig -m 3` / `-t 3`
  switches tty3 between modem and terminal.
- The 16 bit console shows 7E1 parity at the first `login:` prompt. That is
  right: WEGA's getty sets up an ADM-31 (7 bit even parity) and the terminal
  doesn't strip bit 7.

Floppies: the drives are 5.25" double sided, 96 tpi (`FLOPPY_525_QD`). Three
connections were missing and made every disk access fail: the drive motors (PIO2
port A bits 0-3), the FDC interrupt (PIO2 B4) and the DMA access to the FDC data
register (`i8272a_device::dma_r/dma_w`). With that, Return at the U880 monitor
boots the WEGA start floppy (`-flop1 w31start.cqm`), and under WEGA `tar` works
in both directions on a TAR floppy, a WEGA file system floppy mounts, and
`format` of an empty image works. MAME can't save CopyQM images, so writes only
last until the image is unloaded; use e.g. `.mfi` to keep them. A non-UDOS disk
in drive 2 at boot stops UDOS with "DISK INITIALIZATION ERROR C4", insert such
disks after the boot.

Winchester formatting: with the start floppy in drive 0, `ud(0,0)sa.format` on
an empty 48 MB image reads the controller version, writes the parameter table
(NEC D5126: 615 cylinders, 4 heads, 18 sectors) and formats all 2460 tracks. Two
bad tracks entered by hand end up in the table in sorted order, `sa.verify`
lists them and reports two tracks less (44172 useful blocks), and a second
`sa.format` finds the parameters valid. A copy of the WEGA image with one track
of padding and a bad track in its table boots WEGA and shows the same file
systems, which checks the block mapping around a bad track.

## 4. Known limitations

- No modem is emulated, the modem lines were only switched with `ttyconfig`.
- The clock module is minimal (host time, setting the clock does nothing).
- Only the console has a terminal; the other ports were tested with login
  prompts and output. The real P8000 has no parallel printer port.

## 5. Sources

A lot of the work rests on material Oliver Lehmann collected and published or
drew himself, which is what made it possible to check the emulation against the
real thing:

- the schematic of the 16 bit board (drawn from the real board) and the
  schematic of the U130 clock module; both may contain mistakes, and the "GLE"
  parts of the 16 bit schematic are only on development boards;
- the P8000 hardware manual and the UDOS system manual;
- the firmware sources (MON8, MON16 and the WDC), <https://github.com/OlliL/P8000>;
- the WEGA kernel and driver sources and the standalone programs (`sa.format`,
  `sa.verify`, `sa.timer`, ...) in the same repository;
- the WDC hardware emulator with its firmware sources and the signal captures
  of a real WDC, <https://github.com/OlliL/P8000_WDC_Emulator>.

The Z80 CTC, PIO and DMA references are named where they are used in section 1.
