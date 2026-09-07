# xpCFY_TQ

`xpCFY_TQ` version 1.0.3 is a 64-bit Windows X-Plane plugin for CFY Boeing 737 throttle
quadrants. It connects X-Plane 12 and the Zibo 737 to CFY TQ V3, V4 and Pro
hardware through a PoKeys controller.

The plugin synchronises the physical throttle quadrant with the aircraft and
provides:

- Left and right forward-throttle and reverser control.
- Autothrottle motor following and manual A/T disconnect handling.
- TO/GA and A/T-disconnect pushbuttons.
- Left and right fuel-cutoff switches.
- Speedbrake lever, flight-detent interlock and automatic landing deployment.
- Parking-brake switch, mechanical interlock and indicator lamp.
- Stabilizer-trim wheel motor, position indicator and cutout switches.
- Nine-position flap-lever control.
- Battery-controlled TQ backlighting.
- X-Plane status, hardware calibration and live-position windows.

Dedicated rejected-takeoff detection is intentionally not implemented while
the Boeing RTO operating sequence is being investigated. Normal Zibo autobrake
operation is not overridden by the plugin.

## Requirements

- Microsoft Windows 64-bit.
- X-Plane 12.
- Zibo 737-800X.
- A supported CFY TQ V3, V4 or Pro connected through its PoKeys interface.

## Installation

1. Close X-Plane.
2. Download the contents of the repository's `plugins` directory.
3. In the X-Plane installation, create this directory if it does not exist:

   ```text
   X-Plane 12\Resources\plugins\xpCFY_TQ\win_x64
   ```

4. Copy both files into `win_x64`:

   ```text
   xpCFY_TQ.xpl
   PoKeyslib.dll
   ```

5. Start the PoKeys-equipped throttle quadrant and ensure it is reachable by
   the computer.
6. Start X-Plane and load the Zibo 737-800X.

The resulting layout must be:

```text
X-Plane 12
└── Resources
    └── plugins
        └── xpCFY_TQ
            └── win_x64
                ├── xpCFY_TQ.xpl
                └── PoKeyslib.dll
```

The plugin loads `PoKeyslib.dll` from the same directory as `xpCFY_TQ.xpl`.

## First start and calibration

On first start, the plugin searches for the PoKeys controller and opens the TQ
calibration window if no valid calibration file exists. Follow the prompts and
save the calibration before normal operation.

Calibration can be repeated later from the **xpCFY_TQ > TQ Calibration** item
in X-Plane's Plugins menu. The status and live-position windows are available
from the same menu.

The calibration window also contains a **PoKeys network: TCP/UDP** toggle.
TCP is the default. Selecting the button saves the new transport in
`xpCFY_TQ.cfg`; an active Ethernet connection is safely closed and rediscovered
with the selected protocol. An active USB connection is left unchanged, but
the saved choice is used for the next Ethernet connection.

The saved `[connection]` section contains a readable
`network_protocol=TCP` or `network_protocol=UDP` entry. The plugin also writes
the legacy `network_use_udp=0|1` entry for compatibility. Existing files that
lack `network_protocol` are migrated automatically at startup, and every save
is read back to verify that the selected protocol reached disk.

The plugin creates its runtime files beside `xpCFY_TQ.xpl`:

- `xpCFY_TQ.calibration.cfg` — saved hardware calibration.
- `xpCFY_TQ.cfg` — plugin configuration.
- `xpCFY_TQ_Acf.conf` — aircraft configuration, created if required.
- `xpCFY_TQ.log` — current-session diagnostic log.

The log is recreated whenever the plugin starts.

During an aircraft change, the PoKeys connection remains open but simulator
motor control and high-rate input polling enter standby until the next supported
aircraft is ready. This prevents stale throttle targets and repeated hardware
commands from crossing the X-Plane aircraft-unload boundary.

## Troubleshooting

- If the status window reports that the PoKeys DLL is not loaded, confirm that
  `PoKeyslib.dll` is in the same `win_x64` directory as `xpCFY_TQ.xpl`.
- If no TQ is found, confirm the controller has power, its USB/network
  connection is available, and firewall rules permit PoKeys discovery.
- If lever positions are incorrect, run TQ Calibration again.
- Consult `POKEYS_HARDWARE.md` for pin mappings, active-low behaviour, PWM
  assignments and detailed control logic.

## Repository contents

- `src/*.c` — plugin implementation.
- `inc/*.h` and `pokeys/*.h` — plugin and PoKeys API declarations.
- `POKEYS_HARDWARE.md` — hardware and control-logic reference.
- `CHANGELOG.md` — versioned release history.
- `plugins/` — ready-to-install 64-bit plugin binaries.
- `xpCFY_TQ.slnx`, `xpCFY_TQ.vcxproj` and `xpCFY_TQ.vcxproj.filters` — Visual
  Studio solution and project metadata.
- `xpCFY_TQ_Acf.conf` — default supported-aircraft configuration copied into
  the build output when it is not already present.

Generated output, user-specific files and the third-party X-Plane SDK remain
excluded from the repository.

## Building from source

1. Install 64-bit Microsoft Visual Studio with the Desktop development with C++
   workload.
2. Download the X-Plane SDK and place its `CHeaders` and `Libraries` directories
   beneath an `XP_SDK` directory in the repository root.
3. Open `xpCFY_TQ.slnx`, select **Release** and **x64**, and build the project.

The project produces
`Release/plugins/xpCFY_TQ/win_x64/xpCFY_TQ.xpl` and copies `PoKeyslib.dll`
beside it. The `XP_SDK` directory is intentionally ignored so SDK updates do
not become repository changes.
