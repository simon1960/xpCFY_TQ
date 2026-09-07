# xpCFY_TQ PoKeys Hardware Interface

This document is the hardware interface reference for the 64-bit `xpCFY_TQ`
X-Plane plugin. It describes every PoKeys resource currently configured or
driven by the plugin, the logical-to-electrical conversion, and the worker
thread functions that own the device.

## Hardware ownership

Only the PoKeys connection worker in `src/pokeys_thread.c` calls
`PoKeyslib.dll`. X-Plane callbacks and window callbacks only copy status or
queue commands through the public functions in `inc/pokeys_thread.h`. This
prevents simultaneous DLL access from the X-Plane and device threads.

The plugin enables all six PWM channels with a common period of `500000`, the
same 50 Hz configuration used by the original CFY application. A duty of zero
stops a PWM channel.

## Digital pins

Actuator outputs, the backlight, and the parking-brake lamp use the TQ's
active-low conversion through `set_logical_output()`.

| PoKeys API index | Function | Logical `true` | Logical `false` | Plugin use |
|---:|---|---|---|---|
| 0 | Parking-brake handle switch (inverted digital input; physical pin 1) | Parking brake set | Parking brake released | Direct simulator command and interlock supervision |
| 1 | Left/Captain TO/GA switch (inverted digital input; physical pin 2) | Button pressed | Button released | `state->lt_toga` and `left_toga_handler()` |
| 2 | Right/First Officer TO/GA switch (inverted digital input; physical pin 3) | Button pressed | Button released | `state->rt_toga` and `right_toga_handler()` |
| 3 | Left/Captain A/T disconnect switch (inverted digital input; physical pin 4) | Button pressed | Button released | `state->lt_at_disco` and `left_at_disco_handler()` |
| 4 | Right/First Officer A/T disconnect switch (inverted digital input; physical pin 5) | Button pressed | Button released | `state->rt_at_disco` and `right_at_disco_handler()` |
| 5 | Left fuel-cutoff switch (inverted digital input; physical pin 6) | CUTOFF | IDLE | `0.0f`/`1.0f` to `DREF_FUEL_CUTOFF_LT` |
| 6 | Right fuel-cutoff switch (inverted digital input; physical pin 7) | CUTOFF | IDLE | `0.0f`/`1.0f` to `DREF_FUEL_CUTOFF_RT` |
| 7 | Stabilizer-trim MAIN ELECT switch (plain digital input; physical pin 8) | NORMAL | CUTOUT | Synchronises the Zibo electric-trim cutout switch |
| 9 | Stabilizer-trim AUTOPILOT switch (plain digital input; physical pin 10) | NORMAL | CUTOUT | Synchronises the Zibo autopilot-trim cutout switch |
| 10 | Decals/backlight output | Illuminated (electrical low) | Off (electrical high) | Follows `acData.battery_on` |
| 11 | Parking-brake indicator lamp | Illuminated (electrical low) | Off (electrical high) | Active-low, battery-gated `acData.pb_indicator` annunciation |
| 22 | Left throttle motor direction | Drive toward fully open | Drive toward fully closed | Throttle test |
| 23 | Right throttle motor direction | Drive toward fully open | Drive toward fully closed | Throttle test |
| 24 | Left throttle motor enable | Motor bridge enabled/braking | Motor bridge disabled/coasting | Throttle test |
| 25 | Speedbrake motor direction | Push up/extend | Retract/pull down | Live controls and automatic pull-down |
| 26 | Speedbrake motor enable | Motor bridge enabled/braking | Motor bridge disabled/coasting | Speedbrake movement |
| 27 | Stabilizer-trim direction A | See variant table below | See variant table below | Trim-wheel motor direction/braking |
| 28 | Stabilizer-trim enable | Motor bridge enabled | Motor bridge disabled/coasting | V3 enable/brake; V4/Pro bridge enable |
| 29 | Right throttle motor enable | Motor bridge enabled/braking | Motor bridge disabled/coasting | Throttle test |
| 30 | Speedbrake flight-detent interlock | Retracted/released | Engaged | Ground operation, calibration, and speedbrake motor override |
| 31 | Stabilizer-trim direction B | See variant table below | See variant table below | V4/Pro second H-bridge input; held false on V3 |

API indices 0 through 6 are configured with
`PK_PinCap_digitalInput | PK_PinCap_invertPin`; indices 7 and 9 are plain
digital inputs, and the remaining pins in this table are configured with
`PK_PinCap_digitalOutput`. Motor enable outputs are placed in the
logical-false/coast state when a device connects and again when movement
finishes or the plugin stops.

## Backlight

PoKeys API pin 10 drives the CFY TQ decals/backlight. After each completed
dataref read, the X-Plane flight-loop callback publishes whether
`acData.battery_on` is true. The PoKeys worker applies a changed state using
the TQ's active-low output convention. Initial connection and plugin shutdown
explicitly command the backlight off; the latest battery state is reapplied
after a connection or reconnection.

## Analogue inputs

`PK_AnalogIOGetAsArray()` returns seven 12-bit samples. The plugin configures
PoKeys pins 40 through 46 with `PK_PinCap_analogInput`.

| PoKeys pin | Array index | Hardware function | Published value |
|---:|---:|---|---|
| 40 | 0 | Left throttle position | `4095 - raw[0]` |
| 41 | 1 | Right throttle position | `4095 - raw[1]` |
| 42 | 2 | Speedbrake position | `4095 - raw[2]` |
| 43 | 3 | Stabilizer trim position | `4095 - raw[3]` |
| 44 | 4 | Left reverser position | `raw[4]` |
| 45 | 5 | Right reverser position | `raw[5]` |
| 46 | 6 | Flap lever position | `raw[6]` |

Every value is clamped to the range 0 through 4095 before it is published to
the calibration/live-position window.

## Flap lever control

PoKeys analogue pin 46 (`raw[6]`) supplies the physical flap-lever position.
The saved `flaps_min_position` and `flaps_max_position` calibration endpoints
scale the input to the original application's corrected 0..4095 range.

The plugin retains the original 22-sample MinMax treatment for this lever. It
discards the lowest and highest samples and averages the remaining 20. The
filter also rejects isolated second-extreme differences greater than 400 ADC
counts and applies the original bounded handling for changes greater than 3000
counts. Its initial buffer is seeded from the first live sample so startup does
not temporarily command UP while the filter fills.

The corrected position selects one of nine discrete Zibo detents:

| Corrected ADC range | Flap selection | `laminar/B738/flt_ctrls/flap_lever` |
|---:|---:|---:|
| 0..200 | UP | 0.000 |
| 201..900 | 1 | 0.125 |
| 901..1300 | 2 | 0.250 |
| 1301..1950 | 5 | 0.375 |
| 1951..2400 | 10 | 0.500 |
| 2401..2900 | 15 | 0.625 |
| 2901..3300 | 25 | 0.750 |
| 3301..3900 | 30 | 0.875 |
| 3901..4095 | 40 | 1.000 |

During First Run, the first valid calibrated hardware sample is authoritative:
the plugin writes its resolved detent to `DREF_FLAPS_LEVER` when the simulator
does not already match it. This synchronizes the Zibo lever even when the
physical lever has not moved after plugin startup. Normal operation writes only
when the resolved hardware detent changes, preventing ADC noise inside a
detent from repeatedly writing to X-Plane. All writes execute on X-Plane's
flight-loop thread after `GetDataRefValues()`. The corresponding XPUIPC/FSUIPC
offset is `0x0BDC`.

## PWM channels

| Channel | Original/current hardware role | Current plugin behavior |
|---:|---|---|
| 0 | Parking-brake interlock servo | Duty 34000 releases; duty 43000 sets; zero after 500 ms |
| 1 | Stabilizer trim indicator servo | Tracks the filtered trim-pot position using the original 0-to-15-unit servo curve |
| 2 | Stabilizer trim wheel motor | Closed-loop PWM from calibrated minimum speed through 100%, with endpoint and feedback-loss protection |
| 3 | Speedbrake lever motor | 350000 retract/pull-down; 400000 push-up/extend; zero after 2000 ms |
| 4 | Right throttle lever motor | Normal A/T follow plus 175000 medium/250000 fast test operation |
| 5 | Left throttle lever motor | Normal A/T follow plus 175000 medium/250000 fast test operation |

PWM updates always send the complete six-channel duty array. This preserves an
active command on one channel when another actuator is updated.

## Stabilizer trim

The trim feedback is PoKeys analogue pin 43 (`4095 - raw[3]`). The hardware
worker applies the original twelve-sample MinMax treatment by dropping the
lowest and highest readings before using the remaining ten-sample mean for
the indicator and motor governor. The X-Plane control path also retains its
ten-sample moving average before it recognises manual wheel movement.

PWM channel 1 positions the trim indicator. It linearly interpolates the
original application's calibrated duties at trim units 0 through 15:
`47415, 44985, 43466, 41900, 40270, 38511, 36433, 35139, 33908, 32533,
30695, 29081, 27786, 26428, 25005, 23439`.

PWM channel 2 drives the trim wheel between the original protected feedback
limits 400 and 3695. On normal target arrival the governor progressively
reduces PWM by 5 percentage points every 50 ms, then applies the bridge brake.
It changes from conservative gain to full gain at 800 counts, adds the saved
calibration value `trim_min_speed`, and limits output to 100%. A direction
reversal retains its non-blocking 100 ms brake interval. Loss of an analogue
sample stops and coasts the motor immediately.

Trim MAIN ELEC pin 7 and AUTOPILOT pin 9 are configured as plain digital inputs,
matching original PoKeys mode `2`; they must not use the `0x80` inversion flag
used by the other switches. A hardware NORMAL state therefore remains NORMAL in
the aircraft data path. The MAIN ELEC switch inhibits the original manual trim
path; the AUTOPILOT switch inhibits the original A/P closed-loop path. During
First Run both cutout switches must be NORMAL because simulator position owns
the initial wheel synchronisation.

The `[connection]` value `trim_motor_variant` selects the physical bridge:

| Value | Hardware | Pin 27 | Pin 31 | Pin 28 |
|---:|---|---|---|---|
| 3 | CFY TQ V3 | Direction | Held false | Enable/brake |
| 4 | CFY TQ V4 | H-bridge input A | H-bridge input B | Bridge enable |
| 5 | CFY TQ Pro | H-bridge input A | H-bridge input B | Bridge enable |

The current development TQ serial 28630 is identified by the original
application as V4, so newly generated configuration files default to value 4.

The `[connection]` value `network_protocol` selects the PoKeys Ethernet
transport and is stored as `TCP` or `UDP`. Missing or invalid values default to
TCP. The legacy numeric `network_use_udp` value (`0` for TCP, `1` for UDP) is
still read for compatibility. On startup, an older configuration is rewritten
with both keys so its effective selection becomes explicit and persistent.
The calibration-window **PoKeys network: TCP/UDP** button writes the complete
configuration atomically and verifies the saved protocol before applying it.
When Ethernet is active, the worker first makes the motor outputs safe,
disconnects, and rediscovers the controller using the new transport. USB
remains connected because this setting applies only to PoKeys Ethernet sessions.

The writable `sim/flightmodel/controls/elv_trim` dataref provides the simulator
target and manual-wheel output. Its X-Plane range is mapped to the original
generic/XPUIPC travel (`-16383..12312`) and then to trim-pot travel
(`400..3695`). Manual trim is read directly from Zibo's
`laminar/B738/switch/capt_trim_pos` and
`laminar/B738/switch/fo_trim_pos` datarefs. Each is a three-position value:
`0.5` is the released centre detent and `0.0`/`1.0` are the two operated end
positions. The plugin uses a 0.25..0.75 neutral deadband, so a released switch
cannot be mistaken for a trim demand. The physical yoke is assigned to
`sim/flight_controls/pitch_trim_down_elec` and
`sim/flight_controls/pitch_trim_up_elec`; their Begin/Continue/End phases are
the authoritative manual-motor input and select direction immediately. While
either Zibo switch dataref is outside its neutral deadband, simulator trim
movement supplies a secondary direction fallback. Command observation never
consumes the command.

With the autopilot engaged, the A/P owns the original closed-loop path. The
physical wheel follows `elv_trim` through the 50-count governor, subject to the
AUTOPILOT cutout and pause state; manual trim commands continue to reach
X-Plane normally and any resulting simulator target is followed. With the
autopilot disengaged, an active trim command drives the wheel directly at 80%
for V3 hardware or 60% for V4/Pro hardware, subject to aircraft battery power,
the MAIN ELEC cutout, and protected endpoints. Releasing the command brakes the
motor after the same progressive PWM reduction; loss of battery power coasts
it immediately, matching the enabled option in the
original application configuration. Independent
manual wheel movement is written to X-Plane only while the A/P is disengaged
and the motor is not running. Every simulator write is change-driven and runs
on the flight-loop thread after `GetDataRefValues()`.

The A/P state is tested at its numeric ON detent (`>= 0.5`) rather than by an
exact nonzero comparison, so a small residual or animated value cannot falsely
deny manual trim. The electrical command callbacks publish motor direction
immediately and the 50 Hz FLCB maintains that direction after each dataref read
pass. A/T engagement is not part of trim ownership and never inhibits the
manual path.

The log records only trim follow/start/stop transitions. The temporary
parking-brake snapshot diagnostics used during fault investigation have been
removed from the flight loop.

## First Run synchronisation

`TqControlsReset()` starts a new First Run whenever the supported aircraft is
initialised. The flight-loop thread waits for a completed simulator dataref
read and valid coherent PoKeys snapshots before applying any synchronisation.

Before the aircraft data-gatherer flight-loop callback is registered, the
PoKeys worker must complete its 500 ms parking-brake RELEASE pulse. Registration
remains deferred while the device is disconnected or the pulse is incomplete.
Consequently First Run cannot begin until the physical interlock is confirmed
retracted; reconnect retries are idempotent and do not restart an active pulse.

The maintained hardware controls are authoritative during First Run:

- Parking-brake pin 0 sets or releases the simulator brake. A SET state first
  confirms the mechanical interlock, then calls `XPLMCommandBegin()` for
  `CMD_PB_SET` and holds the command across flight-loop updates. The command is
  ended with `XPLMCommandEnd()` only after `acData.pb_indicator` acknowledges
  the set brake (or a release/cancellation safety path occurs). A RELEASE state
  pulses `CMD_PB_SET` once after physical release and retains the interlock-
  release logic.
- Fuel-cutoff pins 5 and 6 write both Zibo mixture/cutoff datarefs from their
  first valid hardware sample.
- Trim-cutout pins 7 and 9 first open their simulator guards, then use the
  existing Zibo toggle commands until both simulator positions match the
  maintained hardware switches. Command retries are limited to one every
  500 ms while acknowledgement is outstanding.

Momentary TO/GA and A/T-disconnect buttons retain their normal press/release
handling; they are not treated as maintained simulator state.

The stabilizer trim wheel and indicator are the deliberate exception. During
First Run, `sim/flightmodel/controls/elv_trim` is authoritative: PWM channel 2
drives the physical wheel to that position while PWM channel 1 immediately
shows the simulator target. Physical trim feedback is not written to X-Plane
during synchronisation. Normal manual/autopilot ownership resumes when wheel
feedback is within the 50-count deadband. Battery power, pause, and trim-cutout
safety conditions continue to inhibit motor power, leaving First Run pending
until movement is safe.

## Speedbrake behavior

The flight-detent interlock is normally released while the aircraft is on the
ground and engaged while it is in flight. Calibration and speedbrake motor
commands temporarily force it to the released state.

The physical lever is resolved using the original application's corrected ADC
bands: 0..250 is DOWN, 251..1250 is ARM, 1251..3217 is the flight range, and
3218..4095 is FLT through UP. These bands are mapped to Zibo's lever positions
0.0 (DOWN), 0.0889 (ARM), 0.667 (FLT), and 1.0 (UP). ARM is therefore a fixed
detent rather than a positive continuous-axis value that would extend the
spoilers.

On an airborne-to-ground transition with the physical lever in ARM, the worker
releases the flight-detent lock and drives the lever to full extension. Five
seconds after touchdown, advancing either throttle with both reversers stowed
drives an extended lever back to DOWN. The moving physical ADC value remains
the source of the corresponding change-driven simulator writes.

The automatic pull-down threshold is the calibrated speedbrake minimum plus 75
counts. The trigger rearms only after the position rises a further 100 counts,
which prevents repeated motor pulses while the lever remains at the stop.

The live controls perform the following operations:

- **Speedbrake DOWN:** release the flight detent, select retract direction,
  apply channel 3 duty 350000, then stop and coast after 2000 ms.
- **Speedbrake UP:** release the flight detent, select extend direction, apply
  channel 3 duty 400000, then stop and coast after 2000 ms.

The live window derives the speedbrake state from analogue position feedback.
Within 75 counts of the calibrated retracted endpoint, **Speedbrake DOWN** is
green and **Speedbrake UP** is amber. Within 75 counts of the extended endpoint,
the colours are reversed. Both remain amber while the lever is between endpoints.

## Parking-brake behavior

The parking-brake implementation has three independent paths: the physical TQ
switch commands X-Plane, aircraft and pedal state supervise the mechanical
interlock, and the simulator annunciator drives the hardware lamp. It does not
route control state through a window event.

### Physical switch and ordered simulator command

The worker publishes a coherent, sequenced snapshot of API pin 0. When that
switch changes to set, the flight loop first requests the set interlock pulse
and waits for the worker to confirm that the PWM update started. Only then does
it begin and hold `CMD_PB_SET` until the simulator lamp confirms the brake. On a
release edge it ends any held command and calls
`XPLMCommandOnce(cmdTable[CMD_PB_SET].handle)` after physical pin 0 reports
released, but only if the freshly read simulator parking-brake ratio still
reports SET. This guard is required because toe braking can release the
simulator before the motorised handle finishes moving; sending the toggle in
that state would set the brake again. The plugin no longer writes the
parking-brake dataref. The interlock release pulse remains unchanged.

After the SET command begins, mismatch supervision is suspended until a later completed
dataref read confirms the requested simulator state. This prevents the previous
frame's value from reversing the new set command while the simulator catches
up. A failed interlock update is retried; unchanged successful commands are
suppressed.

### Interlock supervision

Connection and the first coherent switch sample always request the safe released
interlock state. The first sample is treated as synchronisation rather than a
user edge, so released pedals at startup cannot set the mechanism.

Setting remains ordered: the pilot presses both pedals and pulls the handle;
the plugin confirms the interlock SET pulse before holding the simulator command.
After the simulator confirms SET, both pedals must first fall below
`2000/16383`. A subsequent press above `4000/16383` arms an unambiguous toe-brake
release and releases the physical interlock. The release remains latched until
the motorised switch reports released. It then waits for one further completed
dataref read and pulses `CMD_PB_SET` only if X-Plane still reports the parking
brake set. The extra read removes the race between X-Plane's automatic toe
release and the slower motorised handle. The pedal dead band prevents threshold
chatter. An off handle owns the released interlock state.

The pedal inputs are `sim/cockpit2/controls/left_brake_ratio` and
`sim/cockpit2/controls/right_brake_ratio`. The previous
`sim/flightmodel/controls/l_brake_add` and `r_brake_add` datarefs represented
additional braking output rather than the pilot pedal deflections and caused
the release/set pulse oscillation seen in testing. PWM channel 0 uses the duties
and bounded 500 ms pulse documented above, then returns to zero.

The original application also contains `ThrottleQ_ReleaseParkingBrake(short)`,
a proportional helper mapping a 0-to-4096 input to approximately 22972-to-59333
PWM duty. No call site exists in the recovered normal parking-brake path, so the
native plugin does not use it.

### Indicator lamp

Pin 11 follows `acData.pb_indicator` through the TQ's active-low electrical
conversion. The lamp may therefore illuminate one or two flight-loop updates
after the parking-brake write, when the simulator reports that the brake was
successfully set. The original
`OPTION_ENERGIZE_STATE_OF_AIRCRAFT_INFLUENCES_PB_LIGHT` behavior is permanently
enabled: battery master off always forces the lamp off.

The native plugin contains no parking-brake blink path. The recovered .NET
source has a one-second blink timer, but normal initialization leaves it disabled
and no active parking-brake path enables it.

### Original .NET and FSUIPC reference

These offsets explain the source behavior from which the native logic was
ported. The native plugin consumes the equivalent X-Plane values through its
dataref table and aircraft data block.

| Offset | Size | Direction | Original use |
|---:|---:|---|---|
| `0x0BC8` | 2 bytes | Read/write | Parking-brake state; readings above 1000 mean set. Writes use 32767 for newer simulators, 16383 for FS2004 or earlier, and 0 for release. |
| `0x3102` | 1 byte | Read | Aircraft battery state used to gate the indicator. |
| `0x0366` | 2 bytes | Read | On-ground state used by interlock supervision. |
| `0x02B4` | 4 bytes | Read | Ground speed scaled by 65536; the divided result must be below 1 for pedal-based positioning. |
| `0x0BC4` | 2 bytes | Read | Direct left toe-brake value. |
| `0x0BC6` | 2 bytes | Read | Direct right toe-brake value. |
| `0x3416` | 2 bytes | Read | Calibrated left toe-brake value; the original uses the greater of this and `0x0BC4`. |
| `0x3418` | 2 bytes | Read | Calibrated right toe-brake value; the original uses the greater of this and `0x0BC6`. |

The original input handler accepts switch changes only while the TQ is
calibrated and the simulator is connected. Its window event updates a global
switch value used indirectly by interlock supervision; the native plugin avoids
that UI dependency. Alternative original transports send
`S_MIP_PARKING_BRAKE = On/Off` to ProSim or set `ParkingBrakeCommand` in the
AMST UDP message instead of writing the FSUIPC offset.

### Live-window indication

After a successful release pulse, **Park brake RELEASE** is green and **Park
brake SET** is amber. After a successful set pulse, the colours are reversed.
Until a command succeeds after connection, both remain amber because the
hardware provides no position-feedback input for the interlock.

All button fills use a forced opaque OpenGL path that disables blending,
enables all RGBA colour channels and writes alpha as 1.0. Both plugin windows
initially open centred on the X-Plane screen and can then be dragged. Copyright
notices are horizontally centred using the measured proportional-font width.

## A/T disconnect buttons

The worker reads physical pins 4 and 5 (PoKeys API indices 3 and 4) as inverted
digital inputs. It publishes one coherent snapshot and marks it unavailable
after three consecutive read failures. On X-Plane's flight-loop thread, the
snapshot is converted explicitly to `int`, stored in `state->lt_at_disco` and
`state->rt_at_disco`, then passed to `left_at_disco_handler()` and
`right_at_disco_handler()`. Those existing change-aware handlers begin the
corresponding Zibo command on press and end it on release. Disconnect or invalid
input forces a safe command release.

## Fuel-cutoff switches

The worker reads physical pins 6 and 7 (PoKeys API indices 5 and 6) as inverted
digital inputs and publishes a coherent left/right snapshot. Logical true is
CUTOFF and writes `0.0f`; logical false is IDLE and writes `1.0f`. Writes use
`DREF_FUEL_CUTOFF_LT` and `DREF_FUEL_CUTOFF_RT`, run on X-Plane's flight-loop
thread immediately after the dataref read pass, and occur only when the input
snapshot changes. Three consecutive PoKeys read failures mark both inputs
unavailable and suppress simulator writes until valid readings resume.

## Reverser levers

Left and right reverser positions use analogue array indices 4 and 5 and their
saved calibration endpoints. Each axis retains the original ten-sample moving
average. Reverse becomes active only while on the ground, when its corresponding
forward throttle is at idle, and the reverser passes 250 of 4095 calibrated
counts. It remains active until it falls below 150 counts, reproducing the
original hysteresis.

While reverse is active, the plugin scales the calibrated reverser position
from 0.0 (closed) to -2.0 (full reverse) and writes it to the same per-engine
`sim/cockpit2/engine/actuators/throttle_jet_rev_ratio` dataref used for forward
thrust. Forward throttle travel is independently calibrated from 0.0 (closed)
to 1.0 (open). Forward writes for that engine are suppressed until the reverser
is stowed. Forward and reverse writes are inhibited only while the simulator
owns the throttles through an active A/T thrust mode and are issued only when
the value changes. On the ground, merely arming A/T does not inhibit manual
lever writes; this remains true until TO/GA is selected. This analogue path is
common to V3, V4, and Pro hardware.

## PFD autothrottle speed modes

`laminar/B738/autopilot/pfd_spd_mode` is a float array. Index `0` is the
Captain-side PFD/FMC and index `1` is the First Officer-side PFD/FMC. Both
indices use the same integer-valued mode codes:

| Value | FMA annunciation | TQ throttle behaviour |
|---:|---|---|
| `0.0` | Blank/off | Manual control; no A/T mode is annunciated |
| `1.0` | ARM | Manual control; motors coast and hardware positions are written to X-Plane |
| `2.0` | N1 | A/T motor follow after TO/GA on the ground, or whenever airborne |
| `3.0` | MCP SPD | A/T motor follow after TO/GA on the ground, or whenever airborne |
| `4.0` | FMC SPD | A/T motor follow after TO/GA on the ground, or whenever airborne |
| `5.0` | GA | A/T motor follow; this is the approach go-around mode |
| `6.0` | THR HLD | Manual control; motors coast and hardware positions are written to X-Plane |
| `7.0` | RETARD | A/T motor follow after TO/GA on the ground, or whenever airborne |

A/T ARM must also be on before any motor-follow mode can own the physical
levers. If the two PFD indices momentarily disagree, either index reporting a
motor-follow mode is sufficient to retain simulator ownership. ARM and THR HLD
do not trigger the manual-intervention A/T-disconnect detector.

## Normal autothrottle motor follow

A physical left or right TO/GA press sends its corresponding Zibo command. The
simulator owns the physical handles only when A/T ARM is on and either PFD
speed-mode index 0 (Captain FMC) or index 1 (First Officer FMC) reports a mode
that commands throttle travel: N1 (`2`), MCP SPD (`3`), FMC SPD (`4`), GA (`5`)
or RETARD (`7`). ARM (`1`) and THR HLD (`6`) explicitly coast the motors and
retain manual hardware-to-simulator throttle control. Recognising GA as mode
`5` allows approach TO/GA to acquire the motors after the command takes effect.
When A/T ownership ends, each hardware value is compared with the freshly read
simulator throttle as well as the previous hardware write. Consequently, an
unchanged physical lever immediately replaces the last A/T-commanded simulator
position instead of being suppressed by a stale change-detection cache.

While simulator ownership is active, the per-engine
`throttle_jet_rev_ratio` values are clamped to the forward range 0.0..1.0 and
mapped to the shared 0..4095 corrected-position scale. After the original
twelve-sample MinMax filtering, each physical ADC reading is independently
mapped from its saved MIN/MAX endpoints to that same 0..4095 scale. This
reproduces the original application's `CorrectedPos()` processing, so equal
simulator thrust commands represent equal angular lever positions even when
the two potentiometers have different raw spans. PWM channels 5 and 4 then
drive the left and right physical handles to those corrected targets. The
complete paired command (both targets, both calibrated minimum speeds and the
enable state) is published with a sequence guard. If the 100 Hz publisher is
pre-empted part-way through an update, the PoKeys worker retains the preceding
coherent command for one pass rather than momentarily coasting the motors. It
applies both PWM duties in one `PK_PWMUpdateDirectly()` call, preventing either
lever from receiving a new command one worker cycle before the other.
A/T ARM off, loss of all
qualifying thrust modes, an A/T-disconnect command, or simulator pause releases
both motors to coast. Before TO/GA and while on the ground, manual throttle
movement continues to write thrust even when A/T is armed.

Once TO/GA has been selected during the current A/T ARM session, or whenever
the aircraft is airborne, pilot manipulation during a motor-driven mode issues
the corresponding Zibo A/T-disconnect command and immediately coasts both
motors. Manual movement in ARM or THR HLD remains permitted and does not issue
an A/T-disconnect command.
The worker follows the original manual-input safeguards: a 65-count error must
persist for approximately 280 ms (seven samples at the 40 ms worker interval)
after a 1.5-second powered-motor grace period, or a settled/coasting lever must
move more than 65 counts after a one-second grace period. These tests prevent
commanded motor travel, drivetrain overrun, and ADC noise from being mistaken
for pilot input. When no thrust mode owns the motors, the flight-loop path uses
the same 65-count movement threshold before writing the new physical position
to X-Plane. A/T ARM off resets the TO/GA session and permits a later re-arm.

The X-Plane flight-loop callback runs at 100 Hz and publishes each coherent
left/right target pair to the PoKeys worker. The worker services feedback and
motor control every 40 ms, matching the original application's PoKeys polling
interval and avoiding command saturation on a network controller. It applies
the original twelve-sample MinMax filter
independently to each throttle feedback potentiometer, discarding the lowest
and highest sample before calculating the ten-sample mean. The filtered value
is then converted to corrected 0..4095 travel before entering the governor.
The original governor bands therefore operate in their intended domain:
proportional gain 0.045 below 1000 corrected counts, 0.06 from 1000 through
1599 counts, and 0.5 at 1600 counts or more. Each calibrated minimum motor speed is
added and output is capped at the original 50%.

Normal following uses an 8-count stop band and a 24-count restart band. A
moving lever therefore continues through the former 50-count stop/start zone,
while a stopped lever cannot reverse repeatedly because of ADC noise or
drivetrain overrun. When both normalised simulator targets are within 2.5%, a
bounded correction of up to sixteen PWM percentage points slows the leading
lever and accelerates the lagging lever. The comparison measures each lever's
distance from its own target in corrected angular travel, so a small intentional
left/right target difference is preserved. Both corrected PWM values are
sent in the same `PK_PWMUpdateDirectly` transaction. A direction reversal first
coasts the bridge for one worker pass. The ten-leg ground-test sequencer has
priority over normal A/T follow, and analogue feedback loss immediately coasts
both throttle motors.

When X-Plane reports that the user's aircraft has been unloaded, the main
thread publishes an explicit inactive state to the PoKeys worker before it
clears dataref handles. The worker stops simulator-owned motors once, marks its
input snapshots unavailable and suspends high-rate analogue/digital polling.
The connection and one-second health check remain active, as does any requested
parking-brake interlock release pulse. Polling resumes when a supported aircraft
becomes active or while the calibration window owns the hardware. A failed
analogue read also stops a motor only if it was powered; later failures cannot
flood the device with repeated coast/PWM transactions.

## Throttle test

The **Test Throttles** live-window button queues this ten-leg sequence:

1. Left throttle at medium speed to fully open.
2. Left throttle at medium speed back to fully closed.
3. Left throttle at fast speed to fully open.
4. Left throttle at fast speed back to fully closed.
5. Right throttle at medium speed to fully open.
6. Right throttle at medium speed back to fully closed.
7. Right throttle at fast speed to fully open.
8. Right throttle at fast speed back to fully closed.
9. Both throttles at fast speed to fully open.
10. Both throttles at fast speed back to fully closed.

The medium setting is 35% PWM (`175000`) and the fast setting is 50% PWM
(`250000`). Both-throttle movement remains at fast speed. These settings lie
inside the original controller's 8% minimum and 50% maximum operating range.

Each leg uses the saved calibration endpoints and completes within 50 raw
counts of its target. A leg is limited to 15000 ms. Loss of analogue feedback,
a movement timeout, a PWM failure, or device disconnection stops and coasts
both throttle motors. The live window displays the current stage or failure.

## PoKeyslib.dll functions

The worker dynamically resolves these 64-bit exports:

| Export | Purpose |
|---|---|
| `PK_EnumerateUSBDevices` | Enumerate USB devices; the PoKeys API exposes no per-call timeout |
| `PK_EnumerateNetworkDevices` | Bounded network discovery |
| `PK_ConnectToDevice` | Connect to a USB candidate |
| `PK_ConnectToNetworkDevice` | Connect to a network candidate |
| `PK_DisconnectDevice` | Close a device connection |
| `PK_DeviceDataGet` | Read identity/firmware and perform health checks |
| `PK_PinConfigurationGet` | Read the current pin configuration |
| `PK_PinConfigurationSet` | Apply analogue and digital pin functions |
| `PK_AnalogIOGetAsArray` | Read all lever feedback inputs |
| `PK_DigitalIOSetSingle` | Set active-low actuator, backlight, and parking-brake lamp outputs |
| `PK_DigitalIOGetSingle` | Read inverted parking-brake, TO/GA, A/T-disconnect, and fuel-cutoff inputs on API indices 0-6 |
| `PK_PWMConfigurationSetDirectly` | Enable six PWM channels at period 500000 |
| `PK_PWMUpdateDirectly` | Atomically update all six PWM duty values |

If any required export is unavailable, the worker leaves the device
disconnected, publishes the DLL error to the status window, and issues no
hardware commands.

## Plugin worker API

| Function | Purpose |
|---|---|
| `pokeys_thread_start()` | Load the DLL and start discovery/communication; network discovery is bounded, while the USB enumeration API has no timeout parameter |
| `pokeys_thread_stop()` | Signal the worker, cancel delayed synchronous I/O if necessary, and wait for safe actuator/device cleanup |
| `pokeys_get_status()` | Copy connection identity and protocol status |
| `pokeys_set_network_protocol()` | Select TCP/UDP and safely refresh an active Ethernet connection |
| `pokeys_get_lever_positions()` | Copy a coherent seven-axis snapshot |
| `pokeys_get_parking_brake_input()` | Copy the coherent pin-0 switch snapshot |
| `pokeys_get_toga_inputs()` | Copy the coherent left/right TO/GA snapshot from pins 1 and 2 |
| `pokeys_get_at_disconnect_inputs()` | Copy the coherent left/right A/T-disconnect snapshot from physical pins 4 and 5 |
| `pokeys_get_fuel_cutoff_inputs()` | Copy the coherent left/right fuel-cutoff snapshot from physical pins 6 and 7 |
| `pokeys_get_trim_cutout_inputs()` | Copy the coherent MAIN ELECT/AUTOPILOT trim-cutout snapshot from physical pins 8 and 10 |
| `pokeys_set_parking_brake_indicator()` | Queue the battery-gated pin-11 lamp state |
| `pokeys_set_backlight()` | Queue the battery-master-controlled pin-10 decals/backlight state |
| `pokeys_set_simulator_aircraft_active()` | Put hardware polling and simulator-owned motors into standby across aircraft unload/load transitions |
| `pokeys_set_aircraft_in_flight()` | Select normal flight-detent policy |
| `pokeys_set_calibration_active()` | Force detent release during calibration |
| `pokeys_set_trim_target()` | Publish a bounded target and enable/disable the trim-wheel governor |
| `pokeys_set_trim_manual_command()` | Publish the observed three-state yoke trim direction for immediate manual motor drive |
| `pokeys_set_trim_indicator_target()` | Select simulator-owned indicator positioning during First Run |
| `pokeys_set_trim_min_speed()` | Publish the saved trim-motor minimum PWM percentage |
| `pokeys_trim_motor_is_running()` | Report motor state so physical feedback is not echoed to X-Plane |
| `pokeys_set_throttle_follow_targets()` | Publish calibrated left/right simulator targets, minimum speeds, and A/T ownership |
| `pokeys_set_speedbrake_closed_position()` | Set the automatic pull-down reference |
| `pokeys_speedbrake_retract_and_pull_down()` | Queue speedbrake DOWN |
| `pokeys_speedbrake_push_up_and_extend()` | Queue speedbrake UP |
| `pokeys_parking_brake_interlock_release()` | Queue the parking-brake release pulse |
| `pokeys_parking_brake_interlock_set()` | Queue the parking-brake set pulse |
| `pokeys_ensure_parking_brake_interlock_retracted()` | Idempotently queue startup/shutdown release |
| `pokeys_parking_brake_interlock_is_retracted()` | Confirm the RELEASE pulse completed |
| `pokeys_set_throttle_test_limits()` | Provide the four calibrated throttle endpoints |
| `pokeys_start_throttle_test()` | Queue the complete ten-leg throttle test |
| `pokeys_is_throttle_test_running()` | Report queued or active test state |
| `pokeys_get_throttle_test_status()` | Copy the current stage/result text |

## Safety and shutdown

- Device calls are serialized on one worker thread.
- Network enumeration uses the configured discovery timeout. The PoKeys USB
  enumeration and connect functions expose no timeout parameter; shutdown
  requests cancellation and then waits for cooperative cleanup rather than
  terminating the worker while hardware outputs may still be active.
- Every open-loop actuator command has a bounded duration.
- Throttle movement additionally requires valid calibrated endpoint feedback.
- Trim movement requires live analogue feedback and is bounded by the original
  400..3695 feedback endpoints.
- Three consecutive analogue read failures abort an active throttle test;
  three consecutive TO/GA, A/T-disconnect, or fuel-cutoff read failures publish
  an unavailable/safe input state.
- A connection health failure stops motors before disconnect is attempted.
- Plugin shutdown first unregisters the aircraft data gatherer, then requests
  and waits up to 1.5 seconds for a completed parking-brake RELEASE pulse before
  stopping the worker. The worker performs a final bounded release fallback,
  then zeros PWM, coasts all motor bridges, switches off the backlight and
  parking-brake lamp, releases the speedbrake detent, disconnects the PoKeys
  device, and unloads the DLL.
- Hardware testing must be performed with the quadrant clear of hands and
  obstructions and with an operator ready to remove motor power.
