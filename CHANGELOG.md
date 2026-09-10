# Changelog

All notable changes to `xpCFY_TQ` are recorded here.

## 1.0.5 - 2026-09-10

- Changed the missing `enhanced_logging` configuration fallback to disabled for
  both new and existing configuration files and persisted the migrated setting.
- Added a V3 First Run trim-wheel safety gate using the central 1059..3036 ADC
  range, with directional repositioning guidance and valid-position confirmation.
  Trim targets are constrained to 400..3695 and endpoint shutdown now overrides
  an active progressive-braking ramp.
- Removed binding-table detail from enhanced trim tracing to keep diagnostic logs
  focused on meaningful trim state and motion changes.

## 1.0.4 - 2026-09-07

- Reduced the PoKeys motor-control schedule to 10 ms and replaced nine serial
  digital-input reads with one bulk transaction per pass.
- Applied both throttle H-bridge enable/direction states as a paired transaction
  so Ethernet latency cannot start one lever several commands before the other.
- Preserved fractional PWM governor output and added wider crossing/reversal
  hysteresis to remove coarse low-speed steps and target hunting.
- Added coupled throttle start timing and calibrated target-relative correction
  to improve simultaneous lever motion without erasing commanded engine asymmetry.
- Added a progressive trim-motor start ramp and retained controlled braking near
  the commanded trim position.
- Added a General Configuration window for selecting V3, V4, or Pro hardware and
  TCP or UDP communication, with verified persistence in `xpCFY_TQ.cfg`.
- Moved the ground-only speedbrake, parking-brake, and throttle tests into the
  General Configuration window.
- Corrected custom button rendering so state and action buttons use fully opaque
  background colours under X-Plane's managed graphics state.

## 1.0.3 - 2026-09-07

- Restored the original application's 40 ms PoKeys control cadence and
  1000/1600-count throttle governor ranges to reduce network command pressure
  and remove premature motor-speed transitions.
- Retained the last coherent paired throttle command during a transient
  publisher/worker sequence collision instead of pulsing both motors to coast.
- Increased bounded left/right correction and based harmonisation on each
  lever's target-relative calibrated position, preserving intentional engine
  asymmetry while improving simultaneous movement.
- Added an explicit aircraft-active state to the PoKeys worker. Aircraft unload
  now stops simulator-owned motors once and suspends high-rate hardware polling
  while keeping connection health and parking-brake release servicing active.
- Prevented repeated feedback-failure shutdown commands from flooding an
  unresponsive PoKeys controller after the motors are already safe.
- Persisted the Ethernet selection as `network_protocol=TCP|UDP`, retained the
  legacy numeric setting, added automatic migration, and verified each save.


## 1.0.2 - 2026-09-06

- Hardened plugin enable, disable, aircraft-change, command-handler and PoKeys
  worker lifecycle handling.
- Added source-file and line-number information to diagnostic log entries.

## 1.0.1 - 2026-09-06

- Normalised each throttle feedback potentiometer through its own calibrated
  endpoints before motor-governor and left/right synchronisation calculations.
- Added coherent paired throttle target publication and simultaneous PWM updates.
- Added the calibration-window TCP/UDP selector and saved configuration field.
- Increased the X-Plane flight-loop update rate to 100 Hz.

## 1.0.0 - 2026-09-05

- Initial 64-bit X-Plane 12 plugin release for CFY TQ V3, V4 and Pro hardware.
- Implemented PoKeys discovery/connection, calibration, live/status windows,
  throttles, reversers, TO/GA, A/T disconnect, fuel cutoff, speedbrake, parking
  brake, stabiliser trim, flap control, backlighting and first-run synchronisation.
