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
// Some CTC clones (e.g. the U857D in the Robotron P8000) report the time
// constant, not the frozen down counter, when read back after a software reset.
void set_reload_on_reset(bool reload) { m_reload_on_reset = reload; }
```

The flag is off by default and only `p8k.cpp` sets it, so no other driver sees a
different behaviour.

*Evidence:* none from a data sheet. The Zilog Z80 CTC Product Specification
(<https://bitsavers.mirrorservice.org/components/zilog/z80/Z80_CTC_Product_Specification_Feb80.pdf>)
says that a channel that receives a software reset "stops counting" and that
"a new time constant word must be written" afterwards, but it does not say what
a read returns in between. I did not find a data sheet for the U857D (the East
German CTC). The behaviour is taken from what the P8000 monitor's hardware
self-test requires; the driver sets the flag for exactly that reason.

### 1.3 `z80dma`: virtual hook only, no change in behaviour

`z80dma_device::byte_counter()` is a new virtual function that returns
`m_byte_counter`, and `ua858d_device` got a protected constructor that takes a
device type. Nothing else changes for existing users, including the Spectrum
DMA cards (Datagear, MB02+).

The P8000's UA858D reads back one less than the number of bytes transferred.
That quirk lives in `p8k_dma_device` inside `p8k.cpp`, which overrides
`byte_counter()`.

*Evidence:* the Zilog Z80 DMA Product Specification
(<https://bitsavers.mirrorservice.org/components/zilog/z80/Z80_DMA_Product_Specification_Feb80.pdf>)
only says that "the actual number of bytes transferred is one more than
specified by the block length" and that the programmed block length is one less
than the desired length. It does not describe the value that the byte counter
returns when read. As with the CTC, the behaviour is taken from what the P8000
self-test expects, which is why it is kept inside the P8000 driver.

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

The operating systems are UDOS and WEGA (UNIX like).

### 2.2 Machines and file layout

| Machine | Description |
|---|---|
| `p8000_8` | 8 bit board only (formerly `p8000`) |
| `p8000_16` | 8 bit board + 16 bit board (clone of `p8000_8`) |

The driver is in `src/mame/ddr/p8k.cpp` together with two P8000 specific
devices in their own files, `p8k_wdc.h/.cpp` (Winchester disk controller) and
`p8k_u130.h/.cpp` (clock module). It was in `skeleton/`; a manufacturer
directory for a single file did not seem right, and `ddr/` is the collection for
East German hardware. `mame.lst` is updated. Both machines are still flagged
`MACHINE_NOT_WORKING` (see the limitations below).

### 2.3 What is emulated

**8 bit board** (existing driver, fixed up so that the firmware's hardware test
passes): Z80 with the bank switching, CTC/SIO/PIO daisy chain, UA858D DMA
(`p8k_dma_device`), U8272 floppy controller and the monitor ROMs. The 8 bit
hardware test (`T` at the monitor prompt) runs through all steps (10 to 33).

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

**U130 clock module** (`p8k_u130_device` in `p8k_u130.cpp`; 16 bit board, I/O ports FEF1 to FEFF): a quartz clock
with a six digit seven segment display and two keys that the 16 bit side reads
digit by digit (see `timer.c` of the WEGA kernel and the standalone program
`sa.timer`). The emulation returns the UTC time of the host; the correction
sequence of `sa.timer` is accepted and shows the counting fields, but the new
value is dropped (setting the clock is a no-op). Without a clock module the
kernel asks for the date at boot.

**System clock:** the real-time interrupt comes from CTC1: channels 2 and 3 are
cascaded and count a 1.2288 MHz oscillator (WEGA programs /256 and /80, i.e.
60 Hz). It is wired the same way as the System 8000 CPU board does it.

**Serial ports and baud rates:** both boards have two SIOs, so with the 8 bit and
the 16 bit board there are eight terminal channels (WEGA `sio.h`: tty0-3 on the
8 bit board, tty4-7 on the 16 bit board; the system console is tty1, SIO0 B of
the 8 bit board - the 16 bit monitor reaches it through the coupling PIOs, its
own SIO0 B is only used when the 8 bit part is inactive). Every channel has an
RS232 port (slot names `tty0`, `rs232` = tty1, `tty2`, `tty3`, `tty4`,
`u_rs232` = tty5, `tty6`, `tty7`); only the console has a terminal attached by
default. As in the System 8000 the baud rates come from the CTCs: a baud rate
oscillator (614.4 kHz) feeds the CTC trigger inputs, and the zero count outputs
clock the SIO channels (8 bit: CTC1 ch0/1/2 = SIO0 A/B, SIO1 A, CTC0 ch0 =
SIO1 B; 16 bit: CTC0 ch0/1/2 = SIO0 A/B, SIO1 A, CTC1 ch0 = SIO1 B). With the
SIO in x32 mode a time constant of 2 gives 9600 baud and the WEGA table
(`siospeed`) 19200/TC. The 1.2288 MHz oscillator is only used for the real-time
clock. Tested with the WEGA login on tty0, tty1 and tty4.

### 2.4 Why the 16 bit board follows the System 8000 CPU board

The P8000's 16 bit board is, apart from a few changes, a copy of the Zilog
System 8000 CPU-A board that `src/devices/bus/zbi/s8k_cpu.cpp` already emulates.
Both use the same Z8001 + 3 x Z8010 arrangement with the same "special logic" that selects the MMU, and the
same I/O registers: system configuration register (FFC1), system break register
(FFC9), normal break register (FFD1), the RETI port (FFE1), soft reset (FFE9),
the segment trap registers (FFF1, FFF9) and the SIO/CTC/PIO block at FF81 and
above. The P8000 firmware uses exactly these addresses (`SBREAK := %FFC9`,
`NBREAK := %FFD1`, `RETI_P := %FFE1` in the MON16 sources), and the WEGA kernel
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
- no parity/ECC NMI, no DMA mode, no segment violation register (FFD9 is the
  "RUN16" lamp output on the P8000);
- `soft_reset_w` resets only the 16 bit board, not the whole machine; the 8 bit
  board keeps running; `nmiack_r` returns just 0xFFFF;
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

## 4. Known limitations

- Only the console (tty1) has been used interactively; tty0 and tty4 were checked
  for the login prompt and for output from the shell, the other ports are wired
  the same way but untested.
- Floppy access from the 16 bit side (the UDOS/WEGA start floppy path) has not
  been tested, and neither has the 8 bit board's terminal-side keyboard.
- The WDC formatting and bad-track-table commands have not been tested.
- The WDC firmware ROMs and the terminal character generator ROMs were removed from
  the ROM definitions: the controller is a software emulation that does not
  execute them, and the terminal is a serial console, so nothing uses them.
- Parity and ECC are not emulated.
- The U130 clock module is a minimal emulation (host time, set is a no-op).
- Both machines remain `MACHINE_NOT_WORKING` until the points above are
  settled.

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
