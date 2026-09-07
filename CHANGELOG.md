# Changelog

All notable changes to `xpCFY_TQ` are recorded here.

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
- Updated all source release banners and runtime copyright strings to 1.0.3.

## 1.0.2 - 2026-09-06

- Hardened plugin enable, disable, aircraft-change, command-handler and PoKeys
  worker lifecycle handling.
- Restored the X-Plane character-data type and dataref emission metadata.
- Added source-file and line-number information to diagnostic log entries.
- Published updated hardware/control documentation and installable binaries.

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
  brake, stabilizer trim, flap control, backlighting and first-run synchronisation.
- Added the packaged `xpCFY_TQ.xpl` and `PoKeyslib.dll` installation files.
