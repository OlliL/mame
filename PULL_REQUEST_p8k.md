# ddr/p8k.cpp, machine/z80pio, z80ctc, z80dma, cpu/z8000: Emulated the P8000 16 bit board, WEGA boots

AI disclosure: this work was done with the help of Claude Sonnet 5.5, Claude Haiku 5.5 and
Claude Opus 5.5 (Anthropic).
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
for example that the saved PC segment word keeps bit 15 as the PC holds it, and
that with MMU ON and no MMU driving the bus the board's A8-A23 keep the value of
the last memory cycle (a fetch at 9100 right after the code MMU was switched off
read 9000); the standalone diagnostics need this.
The data/stack MMU selection for a non-segmented user follows the original
circuit diagram of the 16 bit board (two 74S85 comparators on the break
register, equality goes to the stack MMU).

The ROMs of `p8000` (`mon8_*_3.1_nur8bit`) are dumps from the EPROMs of a real 8 bit
only P8000; there is no release of them on the EAW floppies. The ROMs of `p8000_16`
(MON8 for the 8 bit board and MON16 for the 16 bit board) are named as the files on the
EAW UDOS and WEGA start floppies.

The console is the terminal of the P8000. Instead of simulating escape sequences it runs
the terminal firmware on a U882 (Z8) with an i8275 CRTC, the refresh memory, the two
character generators (ZG0 ASCII, ZG1 German umlauts), an IBM XT keyboard and the bell; it
starts in ADM-31 mode. It is a private RS-232 port device (`p8k_term.*`) and the default of
the `rs232` slot of both machines (`-rs232 terminal` still selects the generic terminal).
Its ROMs are the device romset `p8k_terminal` (`p8t_1_5.0` firmware, `p8tezs` and `p8tdzs`
character generators), so the machines need that set for the default console. The terminal
takes every key while its keyboard is active (F3 is its VIDEO key, not the emulator reset),
so the RESET button of the computer is Ctrl+Pause and the NMI button moved from F12 to Pause.

Not emulated: the stop watch and weekday symbols of the clock module.

Tested: a full build, `mame -validate`, the terminal on both machines (banner of the
firmware, keys reach the monitor, save and load), the monitor self tests of both boards,
a complete WEGA installation from the floppies and a boot to multi-user with
1 MiB of memory, and the standalone diagnostics (`sa.diags`). Sources are the P8000
hardware manual, the U880/U855/U8000/U8010 descriptions, the original circuit diagrams of both boards and the firmware and
WEGA sources.

The reasons for each change in the shared devices, with the document and
section they are based on, are in the commit messages. Sources for the driver:
the P8000 hardware manual, the schematics of the 8 bit and 16 bit boards and of
the clock module, the firmware sources (MON8, MON16, WDC) and the WEGA kernel
and driver sources, <https://github.com/OlliL/P8000> and <http://www.pofo.de/P8000/>.
