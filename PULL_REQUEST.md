# ddr/p8k.cpp: Robotron/EAW P8000 (8 bit board and 8+16 bit system), boots WEGA

This pull request moves the P8000 driver out of `skeleton/` and turns it into a
working system: the 16 bit board (Z8001) is emulated and coupled to the 8 bit
board (Z80), the Winchester disk controller and the clock module are emulated,
and the UNIX-like operating system WEGA 3.2 boots from a hard disk image up to
the login prompt and a shell.

It also touches three shared Z80 family devices (`z80pio`, `z80ctc`, `z80dma`).
Those changes are described first, together with the evidence I could find for
them, because they are the part that affects other drivers.

## 1. Changes to shared Z80 family devices

### 1.1 `z80pio`: two fixes (affects every driver that uses `Z80PIO`)

Both fixes were needed to get the interrupt-driven coupling between the 8 bit
and the 16 bit board (two PIOs on each side, mode 0/1/3) and the interrupt
driven disk I/O of WEGA working. They are small, but `Z80PIO` is used by about
100 drivers, so please review them with that in mind.

**(a) `z80daisy_irq_ack()` only acknowledges a port whose interrupt is enabled**

```diff
-		if (port.m_ip)
+		if (port.m_ie && port.m_ip)
```

*Problem:* `m_ip` (interrupt pending) can be set while the interrupt enable
flip-flop is cleared, for example by a mode 3 match or a strobe. The port does
not drive INT in that state, but the acknowledge logic still picked it. On the
P8000 the CPU received the vector of a port that did not request an interrupt
and jumped to a wrong address.

*Evidence:* Zilog Z80 PIO Product Specification
(<https://z80.mirrors.tinycomputers.io/zip/z80piomn.pdf>):

- Section 4.3: "If an interrupt is pending when the enable flag is set, it will
  then be enabled onto the CPU interrupt request line." A pending request with
  a cleared enable flag therefore does not reach the interrupt line.
- Section 5.3: "An interrupt will be generated if interrupts from the port are
  enabled and the data on the port data lines satisfies the logical equation
  defined by the 8-bit mask and 2-bit mask control registers."
- Section 6.0: the PIO "with IEI high and IEO Low during /INTA, will place the
  8-bit interrupt vector of the appropriate port on the data bus", i.e. the
  port that is requesting.

The data sheet never states in so many words that the enable flip-flop gates the
vector; that is derived from the quotes above.

**(b) a mode 3 match is latched even while the port is under service**

```diff
-		if (!m_match && match && !m_ius)
+		if (!m_match && match)
```

*Problem:* if the port-B bit control condition became true while the same port
was still being serviced (`m_ius` set), the interrupt request was dropped. The
condition is edge triggered ("false to true"), so it was never raised again. On
the P8000 the 16 bit board signals "INT8" to the 8 bit board this way; one lost
edge left the WEGA kernel hanging in its coupling loop. Whether
INT is actually driven is already decided later by `check_interrupts()`, which
takes `m_ius` and the daisy chain into account, so the request can safely be
latched.

*Evidence:* Same specification:

- Section 5.3: "A Mode 3 interrupt will be generated only if the result of a
  Mode 3 logical operation changes from false to true." The event is the edge;
  if it is discarded it does not come back.
- Section 3.0: IEO is "High only if IEI is High and the CPU is not servicing an
  interrupt from this PIO". The in-service state blocks *lower priority*
  devices through IEO; nothing says it discards a new condition of the port.
- Section 2.0: "lower priority devices may not interrupt higher priority
  devices that have not had their interrupt service routine completed."
- Zilog "Z80 Family Interrupt Structure"
  (<https://z80.mirrors.tinycomputers.io/1653.htm>): a device that is blocked by
  the daisy chain keeps its interrupt request until the CPU acknowledges it.

The data sheet does not address a new condition on the *same* port while it is
in service explicitly. The conclusion is based on the quotes above and on the
observed failure.

### 1.2 `z80ctc`: opt-in `set_reload_on_reset()`; default behaviour unchanged

```cpp
// After a software reset (STOP = 1) the down counter is continuously
// reloaded from the time constant, so a read returns the time constant and
// not the frozen count.
void set_reload_on_reset(bool reload) { m_reload_on_reset = reload; }
```

The flag is off by default and only `p8k.cpp` sets it, so no other driver sees a
different behaviour. The evidence below indicates that this is the behaviour of
the original Zilog Z80 CTC as well, not a quirk of the U857D clone; the default
was left unchanged only because other drivers could not be tested here.

*Additional evidence (Zilog Z80-CTC technical manual, the circuit description
chapter 4, printed pp. 37, 38, 44; Figs. 4-10B and 4-15):*

- p. 37, Fig. 4-10B: a control word with the reset bit (D1) or the RESET input
  sets flip-flop FF8, which generates STOP; writing the time constant clears it.
- p. 38: after a reset "counting is resumed when a time constant is loaded".
- p. 44, section 4.9: "In both the counter and the timer mode the down counter
  is loaded when it reaches a zero count or when the channel stops counting or
  timing. Transistors 86 through 90 generate the load DC signal whenever STOP or
  TO1 = 1." (Fig. 4-15: LOAD DC = NOT(NOR(STOP, TO1))).

While STOP is set, LOAD DC is therefore active and the down counter holds the
time constant. This matches the monitor self-test and tests by other P8000
hobbyists on U857D and Zilog parts.

*Evidence:* the German data sheet of the U857D ("Technische Beschreibung
Schaltkreis fuer Zaehler- und Zeitgeberfunktion CTC U 857 D", VEB Funkwerk
Erfurt, <https://www.tiffe.de/robotron/Bausteinuebersicht/U857D-CTC.pdf>) does
not state explicitly what a read returns after a software reset:

- p. 18, channel control word bit 1 (reset): "Kanal stoppt die laufende
  Operation. Die Register werden nicht verändert." The channel stops and its
  registers (time constant) are kept. This is consistent with the flag, but
  does not prove it.
- section 8.2, p. 32: a read returns "der aktuelle Zählerstand … die Anzahl der
  noch notwendigen Zählschritte des Rückwärtszählers bis zum Nulldurchgang";
  nothing is said about a stopped channel.

The Zilog Z80 CTC Product Specification
(<https://bitsavers.mirrorservice.org/components/zilog/z80/Z80_CTC_Product_Specification_Feb80.pdf>)
likewise says only that a channel that receives a software reset "stops
counting". The behaviour therefore comes from the P8000 monitor's hardware
self-test (`U880EIG.S`, routine `CZPR` and the test after it): each channel is
started, runs, is reset with control word 3 and then read back; the monitor
requires exactly the time constant. That is why the driver sets the flag.

The data sheet confirms two other things the driver relies on: ZC/TO is a high
pulse of 200 to 600 ns at every zero count (p. 13, 33), not a toggling level,
and channel 3 has no ZC/TO output (p. 13).

### 1.3 `z80dma`: virtual hook only, no change in behaviour

`z80dma_device::byte_counter()` is a new virtual function that returns
`m_byte_counter`, and `ua858d_device` got a protected constructor that takes a
device type. Nothing else changes for existing users, including the Spectrum
DMA cards (Datagear, MB02+).

The P8000's UA858D reads back one less than the number of bytes transferred.
That quirk lives in `p8k_dma_device` inside `p8k.cpp`, which overrides
`byte_counter()`.

*Evidence:* the German data sheet of the U858D (UA858D) DMA ("Technische
Beschreibung", VEB Funkwerk Erfurt,
<https://www.tiffe.de/robotron/Bausteinuebersicht/U858D-DMA.pdf>) describes
exactly this behaviour:

- section 4, p. 14: the byte counter is initialised (set to zero) only after
  the first byte has been transferred, so it holds "unbestimmt" (undefined)
  after the 1st byte, 0 after the 2nd and 1 after the 3rd. "Als
  zwangsläufige Konsequenz dieser Arbeitsweise ergibt sich, daß bei
  Übertragungen stets ein Byte mehr übertragen wird, als im
  Blocklängenregister vereinbart wurde."
- tables 1 and 2, p. 15: when a block ends with programmed length N, N+1 bytes
  have been transferred and the byte counter reads N; after a stop by byte
  match on byte M (transfer, or search in byte mode) M bytes have been
  transferred and the counter reads M-1; in search mode with burst or
  continuous operation M+1 bytes are transferred and the counter reads M. In
  every row the counter is the number of transferred bytes minus one, which is
  what `p8k_dma_device::byte_counter()` returns.
- section 10.2, p. 30: the byte counter is read through RR1/RR2 (read mask
  bits 1 and 2).

The Zilog Z80 DMA Product Specification
(<https://bitsavers.mirrorservice.org/components/zilog/z80/Z80_DMA_Product_Specification_Feb80.pdf>)
only says that "the actual number of bytes transferred is one more than
specified by the block length" and does not describe the value that the byte
counter returns when read. Because the U858D data sheet documents a specific
counter semantics and the Z80 DMA is not claimed to be identical, the quirk is
kept inside the P8000 driver and `z80dma` only gets the neutral virtual hook.
The P8000 monitor's hardware self-test (DMA steps) relies on this value.

### 1.4 Tests

MAME has no unit tests for these components (`tests/` only covers `attotime`,
`util` and `rgbutil` and does not link the device framework), so no tests were
added. The changes were checked with the P8000 firmware self-tests and WEGA boot
described below.

## 2. The P8000 system

### 2.1 Hardware

The P8000 was developed by VEB Elektro-Apparate-Werke Berlin-Treptow (EAW) in
the GDR (see <https://de.wikipedia.org/wiki/P8000>). A system consists of

- an **8 bit board**: U880 (Z80) at 4 MHz, U880 peripherals, U8272 floppy
  controller, U858 DMA, 64 KiB RAM, 8 KiB EPROM (monitor), 2 KiB static RAM,
  serial channels and two 5.25" floppy drives;
- a **16 bit board** (the 16 bit system always has both boards): U8001 (Z8001)
  at 4 MHz, three U8010 (Z8010) MMUs, 16 KiB EPROM, 2 KiB static RAM, serial
  channels and up to four memory cards;
- the two boards are coupled through parallel interfaces (PIOs);
- an external **Winchester disk controller** (a Z80 board with its own firmware)
  connected to a PIO of the 16 bit board;
- a terminal (ADM-31 / VT100 emulation); WEGA's console is an ADM-31.

The operating systems: OS/M (a CP/M clone) and UDOS (a Z80-RIO clone) are
available on the 8 bit board; WEGA (UNIX like) is available on the 16 bit board
only.

**Board revisions.** Several revisions of the boards were built. The emulation
follows the P8000 hardware manual ("Hardwarehandbuch") as follows:

| Board | Emulated revision | Manual chapter |
|---|---|---|
| 8 bit board | Index 3 (marking `V: x3xx`), monitor MON8 3.1 | 4 (the memory control, 3.3/4.3, is identical on Index 1, `V: x1xx`) |
| 16 bit board | Index 4 (marking `V: 4xxx`), monitor MON16 3.1 | 7 |

An 8 bit board Index >= 3 may only be coupled with a 16 bit board Index >= 4.
The development ("GLE") boards and Index 0 are not emulated. According to the
manual the system break register SBR (FFC9) exists only on the 16 bit board
Index 0; it is kept as a plain register because the MON16 hardware test writes
and reads it back (see 2.3). The same revisions are noted in the header comment
of `p8k.cpp`.

### 2.2 Machines and file layout

| Machine | Description |
|---|---|
| `p8000_8` | 8 bit board only (formerly `p8000`) |
| `p8000_16` | 8 bit board + 16 bit board (clone of `p8000_8`) |

The driver is in `src/mame/ddr/p8k.cpp` together with two P8000 specific
devices in their own files, `p8k_wdc.h/.cpp` (Winchester disk controller) and
`p8k_u130.h/.cpp` (clock module). It was in `skeleton/`; a manufacturer
directory for a single file did not seem right, and `ddr/` is the collection for
East German hardware. `mame.lst` is updated. Both machines are no longer flagged
`MACHINE_NOT_WORKING` (see the known limitations below).

### 2.3 What is emulated

**8 bit board** (existing driver, fixed up so that the firmware's hardware test
passes): Z80 with the bank switching, CTC/SIO/PIO daisy chain, UA858D DMA
(`p8k_dma_device`), U8272 floppy controller and the monitor ROMs. The 8 bit
hardware test (`T` at the monitor prompt) runs through all steps (10 to 33).

**Monitor EPROMs:** the 8 bit board alone (`p8000_8`) uses its own MON8 3.1
EPROM pair (`mon8_1_3.1_nur8bit`, `mon8_2_3.1_nur8bit`, "NUR8BIT"). Unlike the
EPROMs of the coupled system (`mon8_1_3.1`, `mon8_2_3.1`, used by `p8000_16`)
this monitor never tries to start or talk to a 16 bit board, which does not
exist in that configuration (the first EPROM differs in one byte, the second in
the code for the 16 bit coupling). A UDOS system floppy boots on `p8000_8`
to the `%` prompt and runs `STATUS`, `DATE`, `ECHO` and `CAT P=&`
(`CAT F=L P=& ST*` lists STATUS with type, length and dates). A plain `CAT` prints
`FILE NOT FOUND`: it lists only non-secret files and all files on that
floppy carry the S (secret) property, so there is nothing to show.

**8 bit memory control** (hardware manual 3.3/4.3): the 64 KiB address space
is divided into 16 slices of 4K. Each slice selects nothing, the EPROM (8 KiB,
repeating), the 2 KiB static RAM or the 64 KiB dynamic RAM through the
write-only address port ADP (I/O 00H, slice = high address byte of
`OUT (C),r`). The static RAM is not decoded on A11, so it appears twice in the
selected 4K window and is the same 2 KiB in every slice. After reset the reset
flip-flop is set: only the EPROM is visible and the ADP is blocked until a
dummy IN/OUT to RES_RFF (I/O 04H). Previously every slice had its own 4K of
"static RAM" and a control word of 0 was ignored.

**16 bit board** (`p8000_16`): Z8001 with three Z8010 MMUs and the board's
special logic (system/normal mode, segmented user space, break registers), the
on-board EPROM/RAM, two SIOs, two CTCs, three PIOs, the Z80 style daisy chain
(see 2.4) and the main memory. With the NMI key (F12) the 16 bit monitor runs
its hardware test and then boots.

**Coupling of the boards:** the 16 bit board's PIO0 sends to the 8 bit board
(PIO0 port A, bit 0 of port B is "INT8"), the 8 bit board sends through two
output latches to the 16 bit board's PIO1. The 8 bit board's PIO0 port B bit 7
holds the 16 bit board in reset. This is what lets the 8 bit monitor start the
16 bit CPU (`X`) and what carries the 16 bit board's console output to the 8 bit
board's serial terminal.

**Winchester disk controller:** emulated in software (`p8k_wdc_device` in `p8k_wdc.cpp`, PIO2 of
the 16 bit board). Only the protocol visible on the PIO interface is
implemented: commands, parameter table, bad track table and error codes follow
the behaviour of Oliver Lehmann's P8000 WDC emulator, which was derived from
signal captures of a real WDC 4.2. The disk is a `harddisk_image_device`, so it
is attached with `-hard`. The controller's /ST0 line raises the interrupt used by
the WEGA disk driver.

*Bad track table (BTT):* the table is kept in the parameter sector (sector 0) as
the byte count, up to 40 entries (cylinder high/low, head) in ascending order and
the end mark `ff ff ff` behind the last entry. It follows the WDC firmware
(`wdc.firm.s`: `btt_cl`, `t_btt`, `sc_def`, `f_def`), which the AVR emulator
ignores: the clear command (0x48) leaves count 0 and the end mark at the start of
the table (`sa.format` inserts new entries in front of the first greater entry,
so the end mark must be right behind the last one), a track found bad while
formatting is entered in sorted order (and only once), and PAR&BTT (0xc2) replaces
an invalid table with an empty one. The bad tracks are skipped by the block
commands (0x21/0x22) and the sector commands with the BTT bit (0x11/0x12): each
requested track moves up by one for every bad track at or before it, and the
blocks available are the total minus the bad tracks. A track in the BTT is not
formatted. An empty table is accepted whatever follows it, so images of the AVR
emulator keep working.

**U130 clock module** (`p8k_u130_device` in `p8k_u130.cpp`; 16 bit board, I/O ports FEF1 to FEFF): a quartz clock
with a six digit seven segment display and two keys that the 16 bit side reads
digit by digit (see `timer.c` of the WEGA kernel and the standalone program
`sa.timer`). The emulation returns the UTC time of the host; the correction
sequence of `sa.timer` is accepted and shows the counting fields, but the new
value is dropped (setting the clock is a no-op). Without a clock module the
kernel asks for the date at boot.

**System clock:** the real-time interrupt comes from CTC1: channels 2 and 3 are
cascaded and count the 1.2288 MHz oscillator (WEGA programs /256 and /80, i.e.
60 Hz). It is wired the same way as the System 8000 CPU board does it.

**Single stepping:** CTC0 channel 3 ("Single Step Steuerung" in the hardware
manual) counts the stack memory requests: each one pulses its CLK/TRG3 input. WEGA
(`event.s`) arms it with a time constant of 17 (14 words of `ldm` plus the three
of `iret`) to get an interrupt after one instruction, and the monitor's NEXT command
counts stack operations the same way. As in `s8k_cpu.cpp` the pulse is generated in
the address translation; the stack accesses to the on-board RAM are counted too.

**On-board memory:** 16 KiB EPROM at 0000-3FFF and 2 KiB static RAM (four 2114)
at 4000-47FF. The RAM select decodes 8 KiB, so the RAM shows up four times up to
5FFF, and 6000-7FFF is empty; with the on-board memory active the main memory
starts at 8000 (hardware manual 3.7.3.2, monitor RAM test: 0x400 words).

**RUN lamp:** writes to FFD9 (LEDEIN) and FFB9 (LEDAUS) switch the output `run16`.

**Serial ports and baud rates:** both boards have two SIOs, so with the 8 bit and
the 16 bit board there are eight terminal channels (WEGA `sio.h`: tty0-3 on the
8 bit board, tty4-7 on the 16 bit board; the system console is tty1, SIO0 B of
the 8 bit board - the 16 bit monitor reaches it through the coupling PIOs, its
own SIO0 B is only used when the 8 bit part is inactive). Every channel has an
RS232 port (slot names `tty0`, `rs232` = tty1, `tty2`, `tty3`, `tty4`,
`u_rs232` = tty5, `tty6`, `tty7`); only the console has a terminal attached by
default. As in the System 8000 the baud rates come from the CTCs. The hardware
manual (figure "Teilerkette zur Baud-Raten Generierung") describes the chain: a
9.8304 MHz crystal, a divide by 8 (1.2288 MHz) that feeds the CTC trigger inputs,
the CTC channel as counter and a divide by two flip-flop behind each zero count
output, which clocks the SIO channel (8 bit: CTC1 ch0/1/2 = SIO0 A/B, SIO1 A,
CTC0 ch0 = SIO1 B; 16 bit: CTC0 ch0/1/2 = SIO0 A/B, SIO1 A, CTC1 ch0 = SIO1 B).
With the SIO in x32 mode a time constant of 2 gives 9600 baud and the WEGA table
(`siospeed`) 19200/TC. The same 1.2288 MHz also feeds the real-time clock. Which channel is a terminal and which a modem line is not
hardware: `/etc/ttyconfig -t 0-7` (called from `rc_csh`) sets the modem bit of
each channel in the kernel (`sioset`, `TIOCMCS`), and `/etc/inittab` starts a
getty on tty0, console, tty2, tty5, tty6 and tty7 in multi-user mode (tty3 and
tty4 have none). Tested with WEGA: login prompts appear on tty0, tty1, tty2,
tty5, tty6 and tty7; a full login (input and output) on tty1 and tty2; output
to all eight ports (`echo > /dev/ttyN`); `ttyconfig -m 3` / `-t 3` switches tty3
between modem and terminal.

### 2.4 Why the 16 bit board follows the System 8000 CPU board

The P8000's 16 bit board is, apart from a few changes, a copy of the Zilog
System 8000 CPU-A board that `src/devices/bus/zbi/s8k_cpu.cpp` already emulates.
Both use the same Z8001 + 3 x Z8010 arrangement with the same "special logic" that selects the MMU, and the
same I/O registers: system configuration register (FFC1), system break register
(FFC9), normal break register (FFD1), the RETI port (FFE1), soft reset (FFE9),
the segment trap registers (FFF1, FFF9) and the SIO/CTC/PIO block at FF81 and
above. The P8000 firmware uses exactly these addresses (`SBREAK := %FFC9`,
`NBREAK := %FFD1`, `RETI_P := %FFE1` in the MON16 sources). The hardware manual
lists FFC9 as unused and the schematic of the 16 bit board marks the register
"GLE" (development version only), but the 16 bit monitor's hardware test writes
and reads it back (error 84), so it is kept as a plain register that the MMU
selection does not use (the operating system is always segmented). The hardware
manual's description of the other registers matches the emulation. The WEGA kernel
emulates RETI the same way as described in the comment of `zbi_bus_device::reti_w()`
(it writes ED, 4D to FFE1).

Therefore the 16 bit part of `p8k.cpp` was restructured to resemble
`s8k_cpu.cpp` as closely as possible, so that a diff of the two files is
meaningful and fixes can be carried from one to the other:

- the same function names and order: `segt_interrupt`, `observe_bus_cycle`,
  `segtack_r`, `mmu_instruction_end`, `viack_r`, `nviack_r`, `nmiack_r`,
  `soft_reset_w`, `select_code_mmu`, `select_data_mmu`, `ram_r`/`ram_w`,
  `install_memory`, `reg_scr_r/w`, `reg_sbr_r/w`, `reg_nbr_r/w`, `reg_trpl_r`,
  `reg_if1l_r`, `out_ns_cb`, `mmu_cmd_r/w`, `addrmap_sio`, `translate_addr`;
- the same member names (`m_reg_scr`, `m_reg_trpl`, `m_lad_low`, `m_is_seg_user`,
  `m_normal_mode`, ...), the same `SCR_*` bit definitions and the same
  commented structure of the MMU selection and address translation;
- the clock/daisy chain handling follows `zbi.cpp`: the VI line forwards the state
  of the whole daisy chain, and is re-evaluated after an interrupt
  acknowledge and after RETI. The old per-device handling let a clock interrupt
  hide a pending disk interrupt, which stopped the boot as soon as the
  system clock was running.

Deliberate differences:

- there is no ZBI bus; the memory sits on the board; the peripherals are two SIOs,
  two CTCs and three PIOs, so there is no peripheral array for the 4 SIOs/3 CTCs
  of the CPU-A board;
- the P8000's operating system is always segmented, so only the segmented-OS
  branch of the MMU selection exists (`m_is_seg_os` is constant);
- no parity/ECC NMI (the P8000 has no parity or ECC memory), no DMA mode, no segment violation register (FFD9 is the
  "RUN16" lamp output on the P8000);
- `soft_reset_w` resets only the 16 bit board, not the whole machine; the 8 bit
  board keeps running; `nmiack_r` returns the NMI identifier of the NMI key (0001);
- the on-board RAM is 2 KiB (mirrored), the stack accesses pulse CTC0 CLK/TRG3
  (single step), the baud rate chain has the divide by two flip-flops of the
  hardware manual;
- WDC and U130 are P8000 specific devices in their own files
  (`p8k_wdc`, `p8k_u130`); the PIO coupling sits in its own section of `p8k.cpp`.

The segment trap latches (`TRPL`, `IF1L`) were part of the driver already; the
system break register (FFC9) was added when the driver was aligned, because the
16 bit monitor's hardware test reads and writes it.

## 3. Testing

All of this was run headless with the WEGA SD card image
(`-hard sd.hd -rs232 null_modem`), no sound or video.

- `p8000_8`: the monitor's hardware test (`T`) runs all steps and returns to the
  monitor prompt without an error.
- `p8000_16`: `X` at the 8 bit monitor starts the U8001; NMI starts the 16 bit
  hardware test, which runs through its steps (40 to 73), and the monitor
  boots (`md(0,16000)wega`).
- WEGA 3.2 (stock kernel): boots to multi-user (`fsck` of all file systems), the
  date comes from the U130 module, the clock keeps running, login as `wega`
  works and `ls`, `who`, `date` run.
- `sa.timer` (from the 16 bit monitor with `O D`, then `md(0,16000)sa.timer`):
  finds the clock module, prints the host date and time, and accepts a new
  date and time.
- the 16 bit board's console shows 7E1 parity at the first `login:` prompt;
  WEGA's getty configures the console for an ADM-31 (7 bit even parity), and
  the terminal does not strip bit 7. That is correct, not an emulation error.

### Floppy drives

The two drives are 5.25" double sided, double density, 96 tpi (hardware manual,
section 3.1), so they are `FLOPPY_525_QD`. Three connections were missing and
made every disk access fail with "DISK ERROR": the drive motors (PIO2 port A
bits 0-3, /LCK0-3, low = on), the FDC interrupt (PIO2 B4) and the DMA access to
the FDC data register. The DMA reads and writes it with DACK, which is
`i8272a_device::dma_r/dma_w`; the CPU's FIFO access returns 0xFF in DMA mode.

- U880 monitor: Return boots the WEGA start floppy (`-flop1 w31start.cqm`, a
  UDOS disk with 16 x 256 byte MFM sectors). It starts the coupling software,
  the U8000 monitor and, after NMI, the WEGA boot from the SD image.
- Under WEGA a disk can be changed at run time (`/dev/rfd196ds9` is drive 2,
  720K): `tar tvf` and `tar xf` of a TAR floppy (`vsh.cqm`, 78 files) work, and a
  WEGA file system floppy (`wgaend22.cqm`) mounts with `/etc/mount` and lists.
  The change was done through the image device (`-flop2`/File Manager/Lua);
  WEGA needs the disk in the drive when the device is opened.
- Writing works too: `tar cvf /dev/rfd196ds9 ...` followed by `tar tvf`/`tar xf`
  and `cmp` gives identical files, and files copied to the mounted WEGA floppy
  are intact after `umount` and a new `mount` (free blocks 169 -> 167). MAME
  cannot save CopyQM images, so a write only lasts until the image is unloaded;
  use a format MAME can write (e.g. `.mfi`) to keep the result.
- Formatting works on a new, empty image (`format /dev/rfd196ds9` from the console,
  answer `y`): `verify` reports no error, `tar cvf`/`tar tvf` and `mkfs` + `mount`
  + `cp` + `cmp` on the new disk succeed. A new `.mfi` image (created through the
  image device) is saved by MAME, unlike CopyQM.
- A non-UDOS disk in drive 2 at boot makes UDOS stop with "DISK INITIALIZATION
  ERROR C4"; insert such disks after the boot.

### Winchester formatting

With the WEGA start floppy in drive 0: Return at the U880 monitor, NMI, `O U` at the
U8000 monitor, Return at the primary boot loader (it loads the secondary one), and
`ud(0,0)sa.format` at its `:` prompt, on an empty 48 MB image (`-hard`):

- `sa.format` reads the controller version and the drive state ("Cyl 0 Hd 0 Sc 1 of
  Drive 0 not readable"), offers the drive types, writes the parameter table
  (NEC D5126: 615 cylinders, 4 heads, 18 sectors) and formats cylinder 0 head 0.
- Two bad tracks entered by hand (cylinder 10 head 1, cylinder 20 head 3) are in the
  BTT in sorted order after formatting all 2460 tracks; PAR&BTT is written back
  (0xc2), and a second pass of `sa.format` finds the parameters valid.
- `sa.verify` then lists both entries, "Blocks/Drive: 44208" and "useful Blocks of
  Drive 0: 44172" (two tracks of 18 sectors less) and verifies the whole disk.
- A copy of the WEGA SD image with one track of padding after cylinder 0 and the
  entry cylinder 1 head 0 in its BTT boots WEGA and shows the same file systems
  (`df`), which proves the block mapping around a bad track; the stock image (empty
  BTT) boots as before.

## 4. Known limitations

- Modem lines (carrier detect handshake) were only switched with `ttyconfig`, no
  modem was emulated.
- The U130 clock module is a minimal emulation (host time, set is a no-op).
- Only the console has a terminal attached (null modem); the other serial ports were
  tested with login prompts and output (see 2.3). The real P8000 has no parallel
  printer port, printers are connected serially.

## 5. Commits

- `p8k: pass the 8-bit firmware hardware self-test`
- `p8k: add software emulation of the WDC hard disk controller, fix 16-bit PIO decode`
- `p8k: add memory banking, Z8010 MMUs and special registers to the 16-bit board`
- `p8k: couple the 8 bit and 16 bit boards (p8000_16 = Z80 + Z8001)`
- `p8k: add U130 clock module on the 16 bit board`
- `p8k: align the 16 bit board with the System 8000 CPU-A board (s8k_cpu.cpp)`
- `p8k: wire the real-time clock on the 16 bit board, forward the daisy chain state`
- `z80dma/p8k: keep the UA858D byte counter quirk P8000 specific`
- `p8k: move the WDC and the U130 clock module into p8k_wdc and p8k_u130 devices`
- `p8k: move driver from skeleton/ to ddr/`
