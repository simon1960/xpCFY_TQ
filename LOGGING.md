# xpCFY_TQ Logging

The plugin creates a new `xpCFY_TQ.log` in the same directory as
`xpCFY_TQ.xpl` on every plugin start. Any log from the previous run is deleted
before the new session begins.

## Entry format

Each entry contains the following fields in order:

1. Local timestamp in `DD.MM.YYYY HH:MM:SS.mmm` format.
2. Source file name in a 30-character, right-aligned field.
3. Source line number in a 5-character, right-aligned field.
4. Log message.

For example:

```text
05.09.2026 09:15:32.047                         main.c   124 xpCFY_TQ starting
```

The leading spaces in the two fixed-width source fields align messages from
different files and line numbers for easier comparison. Only the base file
name is logged; compiler-provided directory paths are removed. A source file
name longer than 30 characters is truncated from the left so that its final
30 characters remain visible.

## Developer interface

Call `log_write()` as before:

```c
log_write("Connected to USB PoKeys serial %u", serial_number);
```

`log.h` defines `log_write(...)` as a call-site macro that supplies `__FILE__`
and `__LINE__` to `log_write_at()`. Existing calls therefore acquire source
location details without manually passing them, and new calls should continue
to use `log_write()`.

Log writes remain serialized by the logging lock so entries from X-Plane and
the PoKeys worker cannot overlap.
