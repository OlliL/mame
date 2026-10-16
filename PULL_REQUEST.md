# ddr/p8k.cpp, machine/z80pio.cpp, machine/z8010.cpp, cpu/z8000: Emulated the P8000 16 bit board, WEGA boots

AI disclosure: this work was done with the help of Claude Sonnet 5.5 (Anthropic).
I ran and checked everything on the emulator myself, and the behaviour that is
not obvious from the documents was confirmed on a real P8000.

The P8000 driver moves from `skeleton/` to `ddr/` and now emulates the 16 bit
board (Z8001 with three Z8010 MMUs) next to the 8 bit board (Z80). The two
boards talk through their PIOs, as on the real machine. I added the Winchester
disk controller (a software model of the protocol on PIO2), the U130 clock
module and a software list with the WEGA and UDOS floppies. With these WEGA
installs onto an empty hard disk image and boots to a login. The 8 bit board
alone (`p8000`) runs OS/M and UDOS.

The driver needed fixes in shared devices. These are the ones that matter:

- `z8000`: the DAB instruction gave wrong results (WEGA printed `13000 + 3000`
  as 0), and the PC offset did not wrap at FFFF.
- `z80pio`: a port acknowledged an interrupt it had not asked for, the mode 3
  match was lost while the port was in service, and port A did not have
  priority over port B.
- `z8010`: registers %15 and %16 are always set, SAR wraps from 63 to 0, command
  %10 resets the mode, and the system stack write warning only applies to
  system stack accesses (U8010 description, p. 34).

Two more changes add an opt-in hook and do not change other systems:
`z80ctc` gets `set_reload_on_reset()` (the P8000 monitor self test needs the
channels to reload on reset), and `z80dma` gets a virtual `byte_counter()`
because the DDR UA858D reads back one count less than the Z80 DMA.

I checked each device change against the U880/U855/U8000/U8010 descriptions.
WEGA still boots to multi-user when the PIO priority, mode 3 and Z8000 PC wrap
changes are reverted, so those are corrections I came across and can go in a
separate PR if preferred.

Behaviour I could not settle from the documents I measured on a real machine,
for example that the saved PC segment word keeps bit 15 as the PC holds it.
The data/stack MMU selection for a non-segmented user follows the original
circuit diagram of the 16 bit board (two 74S85 comparators on the break
register, equality goes to the stack MMU).

Not emulated: the stop watch and weekday symbols of the clock module.

Tested: a full build, `mame -validate`, the monitor self tests of both boards,
a complete WEGA installation from the floppies and a boot to multi-user with
1 MiB of memory. Sources are the P8000 hardware manual, the U880/U855/U8000/U8010
descriptions, the original circuit diagrams of both boards and the firmware and
WEGA sources.
