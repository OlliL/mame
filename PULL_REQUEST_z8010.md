# machine/z8010: fixes for the "set all inhibit" commands, SAR wrap, command %10 and SWW

AI disclosure: this change was made with the help of Claude Sonnet 5.5 (Anthropic). I, @OlliL, reviewed it myself, checked it against the documents named below and ran it on the emulator.

Four corrections, each from the Z8010 documents (U8010 technical description,
ZiLOG System 8000 Hardware Reference Manual, Moore's "Z8000 Handbook").

- Commands %15 and %16 (set all CPU-inhibit / DMA-inhibit flags) cleared the
  flags in all descriptors when the data byte was zero. The data byte is not part
  of the command (U8010 6.2.21/6.2.22; System 8000 manual table 3-2; Moore ch. 5),
  so they now always set the flags.
- The auto-incremented SAR wraps from 63 to 0 ("Dem Deskriptor 63 folgt dabei der
  Deskriptor 0", U8010 6.2.13, 6.2.14, 6.2.16). `INC_SAR()` and `INC_DSC_SAR()`
  stayed at 63.
- Command %10 resets MR, VTR and DSCR (all bits zero, SAR untouched; U8010 6.2.17,
  table 6.2). The device did not know it.
- The secondary write warning (SWW) is only set for a write to the system stack
  (status %9, system mode) while a flag is already set; any other write warning in
  that state is FATL (U8010 5.2, p. 34). The device checked the mode only, so a
  system mode write from a data access set SWW.

I found these with the P8000 driver (separate PR). WEGA boots with and without
them; its `_invsdrs` writes %15 with whatever is left in a register, which the old
behaviour turned into random flag changes.
