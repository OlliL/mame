# ddr/p8k.cpp, machine/z80pio, z80ctc, z80dma, cpu/z8000: Emulated the P8000 16 bit board, WEGA boots

AI disclosure: this work was done with the help of Claude Sonnet 5.5 (Anthropic).
I, @OlliL, reviewed and ran everything on the emulator myself, and the behaviour that is
not obvious from the documents was confirmed on a real P8000.

The P8000 driver moves from `skeleton/` to `ddr/` and now emulates the 16 bit
board (Z8001 with three Z8010 MMUs) next to the 8 bit board (Z80). The two
boards talk through their PIOs, as on the real machine. I added the Winchester
disk controller (a software model of the protocol on PIO2), the U130 clock
module and a software list with the WEGA and UDOS floppies. With these WEGA
installs onto an empty hard disk image and boots to a login. The 8 bit board
alone (`p8000`) runs OS/M and UDOS.

The driver needs small changes in three shared devices, which are part of this PR:
`z8000` DAB gave wrong results (WEGA printed `13000 + 3000` as 0), `z80pio` answered
an interrupt acknowledge for a port that had not enabled its interrupt and lost a
mode 3 match while the port was in service (one lost edge hangs the WEGA kernel
in the coupling loop), `z80ctc` gets an opt-in `set_reload_on_reset()` (the
monitor self test needs it) and `z80dma` a virtual `byte_counter()` because the
UA858D reads back one count less than the Z80 DMA. The last two do not change
other systems.

Further corrections I found on the way (Z8000 PC wrap, PIO port priority and
mode 3 after RETI, several Z8010 commands) are not needed to boot WEGA, so they
are separate PRs.

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
