# cpu/z8000: PC offset wraps at FFFF instead of carrying into the segment

AI disclosure: this change was made with the help of Claude Sonnet 5.5 (Anthropic). I, @OlliL, reviewed it myself, checked it against the documents named below and ran it on the emulator.

The instruction fetch advanced the 32 bit PC with `m_pc += 2`, so an opcode or
operand word at offset FFFE carried into the segment number. Instructions and
multi-byte data cannot cross segment boundaries (U8000 description, 3.4.2), and
the offset of indexed, base and relative addresses already wraps without
touching the segment number in the core (5.5.5, 5.5.7, 5.5.8). Moore's
"Z8000 Handbook" (chapter 2, p. 19) says the same for the PC: past FFFF it wraps
to 0000 with no carry into the segment register. I checked it on a real P8000
(Z8001): an instruction at the last word of a segment is followed by the word at
offset 0 of the same segment, not of the next one.

The fetch now wraps the offset the same way. This also keeps the segment intact
that a Z8001 in non-segmented mode takes from the PC.

I found this while bringing up the P8000 (separate PR), where it is not needed
to boot WEGA. I did not test other drivers that use the Z8000 core, so please
check whether any of them is affected.
