# ddr/p8k.cpp: EAW P8000, now with the 16 bit board, boots WEGA

This moves the P8000 driver from `skeleton/` to `ddr/` and makes it work. The
16 bit board (Z8001) is emulated and coupled to the 8 bit board (Z80), the
Winchester disk controller and the clock module are there, and WEGA 3.2 boots
from a hard disk image to a login and a shell.

The driver needed small changes in three shared Z80 devices (`z80pio`, `z80ctc`,
`z80dma`). Those are the part that can affect other drivers, so they come first.

## 1. Shared devices

### z8000: DAB fix

`DAB` computed the high-nibble correction from the byte after the +6 low-nibble
correction. For bytes 0xFA-0xFF that addition overflows and hides the need for
the 0x60 correction. The high digit is now judged from the original byte after
an addition. WEGA's C library converts longs to decimal with a BCD loop, so
before this fix numbers like 16000...16260 and 100000 were printed wrongly
(`expr 13000 + 3000` returned 0, which made `sysgen` write `md(0,0)wega` as
default boot path).

The DAB table of the U8000 CPU description (arithmetic instructions, DAB, p. 68;
Z8000 compatible) lists the correction (00, 06, 60, 66, FA, A0, 9A) and the
resulting carry for each combination of preceding instruction (ADDB/ADCB or
SUBB/SBCB), C, H and the two digits. The Z8000 books by Leventhal (Table 3-12,
p. 3-168) and Moore (Table 2-8, p. 133) and the Z8000 programming book with
"DAB R" (p. 132) give the same table. I checked all 764 byte/flag combinations
that the table covers against the code: no difference. Z and S are set from the
result, P/V is not touched, as listed ("CZS---"). Combinations outside the table
are undefined in all of these sources.

### z80pio: three fixes

The first two were needed for the interrupt driven coupling of the two boards and for
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

Third, the daisy chain priority inside the chip. A port under service blocked
the other port in both directions. The manual says "Within a PIO port A
interrupts have higher priority than those of Port B" and "Higher priority
devices may interrupt the servicing of lower priority devices" (Z80 PIO Technical
Manual, 2.0, p. 4). So B being in service must not hide a request of A; only A
in service blocks B. `z80daisy_irq_state()` and `check_interrupts()` now stop
at the first port under service and let ports in front of it through. The
acknowledge and RETI handling already walked A before B and needed no change.

### z8000: PC offset wraps at FFFF

The instruction fetch advanced the 32 bit PC with `m_pc += 2`, so an opcode or
operand word at offset FFFE carried into the segment number. The U8000 (Z8000
compatible) description says that instructions and multi-byte data elements
cannot cross segment boundaries (3.4.2) and that the offset of an indexed, base
or relative address wraps without touching the segment number (5.5.5, 5.5.7,
5.5.8), which the address arithmetic of the CPU core already did. Moore's
"Z8000 Handbook" (chapter 2, p. 19) states it for the PC itself: "Any time the PC
offset register is incremented past FFFF it will simply wrap around to 0000
without affecting the segment register. There is no carry from the offset
register to the segment register." The fetch now wraps the offset the same way.
This also keeps the segment that a Z8001 in non-segmented mode takes from the PC
intact. WEGA still boots.

### z8010: "set all inhibit" commands

The special I/O commands %15 (set all CPU-inhibit attribute flags) and %16 (set
all DMA-inhibit attribute flags) cleared the flag in all descriptors when the
data byte was zero. The ZiLOG System 8000 Hardware Reference Manual (Table 3-2)
and the Z8010 data sheet list these commands as "Set All CPU-Inhibit Attribute
Flags" / "Set All DMA-Inhibit Attribute Flags" and know no command that clears
them, and the data byte is not part of the command. The U8010 technical
description says the same (6.2.21/6.2.22: "Der MMU müssen dazu keine Daten
übergeben werden", the register used as source is chosen arbitrarily, its content
is ignored). They now always set the flag. WEGA's `_invsdrs` writes %15 with whatever is left in a register (I saw
00, 3E and D8), which with the old behaviour cleared or set the flags at random.

The U8010 technical description (6.2.13, 6.2.14, 6.2.16) says: "Dem Deskriptor 63 folgt dabei
der Deskriptor 0". The auto-incremented SAR therefore wraps from 63 to 0 instead
of staying at 63 (and instead of staying at the last field of descriptor 63), so
`INC_SAR()` and `INC_DSC_SAR()` now wrap. Moore's "Z8000 Handbook" (chapter 5, p. 355,
358) agrees: the SAR is a 6 bit register for 64 descriptors, and Table 5-3 lists
%15/%16 as "Set All CPU-Inhibit Flags" / "Set All DMA-Inhibit Flags". Nothing in
the P8000 software depends on it, since it always writes the SAR before an access.

The same description (6.2.17, Table 6.2) lists command %10, "reset of MR, VTR and DSCR":
all bits of these three registers become zero, the SAR is not mentioned. The device
did not know this command, so it is added.

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

There are two machines: `p8000` (8 bit board only) and
`p8000_16` (both boards, clone of `p8000`). Besides `p8k.cpp` there are
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

I checked the board against the schematics of the 8 bit board (see Sources): the
two 74138 decoders give the I/O map in the driver (00-1F and 20-3F), the memory
chips are what the memory model assumes (2 x 2732, 4 x 2114, 8 x 2164), the
baud rate CTC channels clock the SIO channels through divide by two flip-flops as
in `baud_w()`, the 9.8304 MHz baud quartz divided by eight gives the 1.2288 MHz
CTC trigger, and the interrupt priority chain is DMA, PIO2, CTC0, SIO0, SIO1,
PIO0, PIO1, CTC1. I also went through the floppy controller wiring. PIO2 port A
bit 4 (4 or 8 MHz clock, i.e. 250 or 500 kbit/s), bit 6 (TC, ORed with the DMA
interrupt) and port B bit 7 (FDC reset) are now connected; drives 0 and 1 are
modelled. Not modelled: head load (PA5, the drives have no head load magnet),
the write precompensation outputs (PB5/PB6), a separate drive select (the FDC
unit select lines decide), drives 2 and 3 and 8" drives. Input PA7 is the reset
state (high after power-on, low after operator reset); a DIP switch stands in
for it.

`p8000` uses its own monitor EPROMs (`mon8_1_3.1_nur8bit`,
`mon8_2_3.1_nur8bit`), because the normal ones try to start a 16 bit board that
isn't there. `p8000_16` keeps `mon8_1_3.1` and `mon8_2_3.1`. A UDOS system floppy
boots on `p8000` to the `%` prompt, and `STATUS`, `DATE`, `ECHO` and `CAT P=&`
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
UTC time at the start. `sa.timer`'s correction sequence works: the new time
or date only changes the module's own clock (an offset to the host time, the
host clock is never touched), and the offset is saved in the device's NVRAM file
like the battery of the real module keeps the time, so a reset or a restart of
the emulation doesn't lose it. Without the module the kernel asks for the date
at boot. The hardware (watch IC plus 4000 series buffers and a
4035 for the keys) is described at the top of `p8k_u130.cpp`.

### Real-time clock, single step, serial ports

The real-time interrupt comes from CTC1 channels 2 and 3, cascaded on the
1.2288 MHz clock (WEGA programs /256 and /80, so 60 Hz), wired like on the System
8000 board.

CTC0 channel 3 ("Single Step Steuerung" in the manual) counts stack memory
requests, each one pulses its CLK/TRG3 input. WEGA arms it with a time constant
of 17 to get an interrupt after one instruction, and the monitor's NEXT command
counts stack operations the same way. As in `s8k_cpu.cpp` the pulse is made in
the address translation. The CTC expires during the `IRET` that enters the
program, so the interrupt request must only reach the CPU once the next
instruction has started, otherwise the program's first instruction is never
executed. The 16 bit board delays a vectored interrupt request until the next
instruction fetch (with a timer as fallback for a CPU in HALT). With that the
monitor's `N` command steps one instruction at a time (`N 5` five, a plain
RETURN repeats it).

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

### Software list

`hash/p8000_flop.xml` lists the floppies I have dumped: the UDOS 3.1 system disk, the OS/M 3.0 system disk, the WEGA 3.1 start disk and the WEGA 3.1 installation disks (root 1-5, usr 1-9, doc 1-2). The multi-disk sets have one part per disk, e.g. `mame p8000_16 w31start -flop2 w31root:flop1`. The images are not part of the pull request.

## 3. Testing

Everything was run headless with the WEGA SD card image (`-hard sd.hd -rs232
null_modem`).

- `p8000`: the hardware test runs through without an error.
- `p8000_16`: `X` at the 8 bit monitor starts the U8001, NMI starts the 16 bit
  hardware test, and the monitor boots WEGA (`md(0,16000)wega`).
- WEGA 3.2 (stock kernel) boots to multi-user mode, the date comes from the
  clock module and the clock keeps running, and login as `wega` works with
  `ls`, `who`, `date`.
- `sa.timer` finds the clock module, shows the host date and time and accepts a
  new one. The setting is kept in the NVRAM file of the module, so the kernel
  (and the date prompt at boot) shows that time on the next start.
- A fresh install on an empty image works with the software list disks: `sa.format`,
  `sa.verify`, `sa.mkfs`, `sa.install` (root and /usr), `/etc/new.install`, `sysgen`
  (new kernel and boot program), reboot with plain NMI (the new boot program
  starts `md(0,16000)wega` by itself), `mkfs`/`labelit` of /z and `df -t`. All
  counts match a run on the real machine.
- The main memory is 1 MiB by default and can be set with MAME's `-ram` option
  (`2m`, `4m`, `8m`). With 4 MiB the monitor reports `MAXSEG=<3F>`, with 8 MiB
  `MAXSEG=<7E>`, and WEGA boots with 4022784 / 8217088 bytes of user memory.
  8 MiB is the maximum (128 segments of 64 KiB). Segment 127 is always empty,
  like on the real 16 MiB memory card, which leaves out 7FH ("8 MB minus 64 KB
  can be used"). Without that hole the firmware memory test (step 70) would
  never end.
- A full login (user, password, a command) works on every port with a getty:
  tty0, tty1, tty2, tty5, tty6 and tty7. Output works on all eight ports. `ttyconfig -m 3` / `-t 3`
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
last until the image is unloaded; use e.g. `.mfi` to keep them. UDOS stops
with "DISK INITIALIZATION ERROR C4" if drive 2 holds a non-UDOS disk at boot. The
real P8000 does the same, so insert such disks after the boot.

Winchester formatting: with the start floppy in drive 0, `ud(0,0)sa.format` on
an empty 48 MB image reads the controller version, writes the parameter table
(NEC D5126: 615 cylinders, 4 heads, 18 sectors) and formats all 2460 tracks. Two
bad tracks entered by hand end up in the table in sorted order, `sa.verify`
lists them and reports two tracks less (44172 useful blocks), and a second
`sa.format` finds the parameters valid. A copy of the WEGA image with one track
of padding and a bad track in its table boots WEGA and shows the same file
systems, which checks the block mapping around a bad track.

## 4. Known limitations

Points where I found no statement in the official documents, so the existing
behaviour is unchanged:

- Bit 15 of the saved PC segment word: the U8000 description (6.6.2, Fig. 6.1)
  says the 7-bit segment number and the 16-bit offset are saved, in the order
  identifier, FCW, PC segment, PC offset (rising addresses), and the program
  status area (Fig. 6.2) has a "SEG" bit in front of the segment number. Nothing
  says whether that bit is pushed or what IRET does with it, so the existing
  behaviour is unchanged.
- Z8010 write warnings: the U8010 description (5.2, p. 34) sets the secondary write
  warning only for a system stack access (status %9) and the fatal flag for any other
  write warning once a flag is set. The device checks system mode only, so a system
  mode write warning from a non-stack access sets SWW instead of FATL. This is a rare
  case and DMA is involved, so I left it unchanged.
- Z80 PIO: an interrupt that is pending when the enable flag is cleared stays
  pending (the manual is silent, MAME is consistent with 4.3).
- Z8010 behaviour when the base register of a segment equals the NBR: not
  specified, WEGA treats it as data.

- The clock module's stop watch key (ST) and the weekday symbols are not
  emulated, the software doesn't use them.

## 5. Sources

Official documents behind the corrections in section 1 and the clocks and memory:

- Zilog Z80 PIO Technical Manual (priority 2.0, interrupt enable 4.3, mode 3 5.3,
  IEO until RETI 6.0);
- ZiLOG System 8000 Hardware Reference Manual, Table 3-2 (MMU special I/O);
- U8000 CPU description, DAB (p. 68) and Leventhal, Z8000 Assembly Language Programming, 3-167/3-168 (DAB table);
- Leventhal, Z8000 Assembly Language Programming, chapter 12: p. 12-10 (stack frame
  identifier, FCW, PC segment, PC offset), p. 12-17 (interrupt priorities), p. 12-18
  (IRET pops and discards the identifier; reset loads FCW from 0002, PC segment from
  0004, PC offset from 0006 on the Z8001), p. 12-23 (Z80 PIO: port A before port B,
  lower priority devices wait for the acknowledge); no difference found, the book
  says nothing about bit 15 of the saved segment word or a pending PIO interrupt
  when the enable flag is cleared;
- Richard Mateosian, Programming the Z8000, chapter II p. 41-45 (stack after interrupt: reason,
  FCW, PC; for internal traps the reason is the first instruction word and the saved PC
  points at the second word; PSAP low byte zero) and appendix E p. 294 (program status
  area layout); no difference found, nothing about bit 15 of the saved segment word;
- Martin L. Moore, Z8000 Handbook, chapter 2, p. 19 (PC offset wraps without carry into the segment; additional confirmation);
- Zilog Z8010 MMU data sheet (special I/O commands, SAR autoincrement) and the
  U8010 (Z8010 compatible) technical description, 3.2.1, 4.1, 6.2.13-6.2.16,
  6.2.17, 6.2.21, 6.2.22 and E.3 (SAR wraps 63 to 0; %15/%16 transfer no data and the
  data byte is arbitrary; command code table);
- Leventhal, Z8000 Assembly Language Programming, chapter 13: p. 13-5/13-6 (64 descriptors,
  limit byte in 256 byte blocks, valid blocks 0..limit), p. 13-8/13-9 (mode register),
  p. 13-10/13-11 (Table 13-1: commands %08-%0F, %11, %13-%16, %20), p. 13-16/13-17
  (violation type register, write warnings, fatal); additional confirmation;
- U8010 description 5.4, p. 35 and 5.4 examples p. 37/38: the valid offsets of a normal
  segment are 0 to 256*N+255, of a stack segment (DIRW set) 256*N to 65535, with N the
  limit field and N = 256 - n for n blocks; the lowest block gives the write warning
  (matches `so > limit` and `so < limit` in the device; the figure in Leventhal p. 13-6
  reads differently, the U8010 description is followed);
- Martin L. Moore, Z8000 Handbook (Prentice-Hall), chapter 5, Z8010 MMU, p. 355
  (SAR is 6 bits, autoincrement) and p. 358 (Table 5-3, commands %15/%16); a book
  on the original chips, used as additional confirmation;
- P8000 hardware manual 5.2 and the parts list of the 16 bit board (baud rate
  quartz 9.832 MHz, divided by eight = 1.229 MHz; I had assumed 9.8304 MHz);
- U8000 CPU technical description, 3.4.2, 5.5.5-5.5.8, 6.6.2, Fig. 6.1 and 6.2
  (segment boundaries, offset arithmetic, saved program status, program status
  area);
- the description of the 16 MiB memory card (segment 7FH left out).

A lot of this rests on material I collected, published or drew myself, which is
what made this emulation possible:

- the schematics of the 16 bit board and of the U130 clock module, drawn from
  the real boards (<http://www.pofo.de/P8000/notes/plaene/eigene/>); they may
  contain mistakes, and the parts marked "GLE" in the 16 bit schematic are only
  on development boards;
- the schematics, parts list and layout of the 8 bit board
  (<http://www.pofo.de/P8000/notes/plaene/8-Bit-Rechner/>);
- the P8000 hardware manual and the UDOS system manual;
- the firmware sources (MON8, MON16, WDC), <https://github.com/OlliL/P8000>;
- the WEGA kernel and driver sources and the standalone programs (`sa.format`,
  `sa.verify`, `sa.timer`, ...) in the same repository;
- my WDC hardware emulator with its firmware sources and the signal captures of
  a real WDC, <https://github.com/OlliL/P8000_WDC_Emulator>.

Chip documents used for section 1:

- Z80 PIO Product Specification, <https://z80.mirrors.tinycomputers.io/zip/z80piomn.pdf>
- Z80 Family Interrupt Structure, <https://z80.mirrors.tinycomputers.io/1653.htm>
- Z80 CTC Product Specification, <https://bitsavers.mirrorservice.org/components/zilog/z80/Z80_CTC_Product_Specification_Feb80.pdf>
- Z80-CTC Technical Manual (circuit description, pp. 37, 38, 44), <https://www.ecworld.ru/media/museum/Z80_ctc.pdf>
- Z80 DMA Product Specification, <https://bitsavers.mirrorservice.org/components/zilog/z80/Z80_DMA_Product_Specification_Feb80.pdf>
- U857D (CTC) technical description, VEB Funkwerk Erfurt, <https://www.tiffe.de/robotron/Bausteinuebersicht/U857D-CTC.pdf>
- U858D (DMA) technical description, VEB Funkwerk Erfurt, <https://www.tiffe.de/robotron/Bausteinuebersicht/U858D-DMA.pdf>
