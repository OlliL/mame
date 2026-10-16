# machine/z80pio: port A has priority over port B, mode 3 request dropped after RETI

AI disclosure: this change was made with the help of Claude Sonnet 5.5 (Anthropic). I, @OlliL, reviewed it myself, checked it against the documents named below and ran it on the emulator.

Two changes in the interrupt logic, both from the Z80 PIO Technical Manual and
the U855 D technical description.

Priority inside the chip: a port under service blocked the other port in both
directions. The manual says "Within a PIO port A interrupts have higher priority
than those of Port B" and "Higher priority devices may interrupt the servicing
of lower priority devices" (2.0, p. 4). So port B in service must not hide a
request of port A; only port A in service blocks port B.
`z80daisy_irq_state()` and `check_interrupts()` now stop at the first port under
service and let the ports in front of it through.

Mode 3 after RETI: a request that arrives while the channel is in service is
stored, except in mode 3, where it is only passed on if the logic condition is
still met after RETI (U855 D description, 5.3, p. 16 and 7.1, p. 25).
`z80daisy_irq_reti()` now drops a pending request when the mode 3 condition is
no longer true.

`Z80PIO` is used by many drivers, so this needs a careful look. I only tested the
P8000 driver (separate PR), which boots WEGA with and without these two changes.
