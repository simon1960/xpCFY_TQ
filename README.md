# xpCFY_TQ

`xpCFY_TQ` version 1.0.6 is a 64-bit Windows and Linux X-Plane plugin for CFY Boeing 737 throttle
quadrants. It connects X-Plane 12 and the Zibo 737/LevelUp V2 to CFY TQ V3, V4 and V4 Pro
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

- Microsoft Windows 64-bit or Linux x86-64.
- X-Plane 12.
- Zibo 737-800X, LevelUp V2.S1 .
- A supported CFY TQ V3, V4 or V4 Pro connected through its PoKeys interface.

## Latest Stable Download
The official plugin download site is XPlane.org and the latest stable version can be found at
https://forums.x-plane.org/files/file/101415-xpcfy_tq-plugin-for-cockpitforyou-motorised-tqs/

The GitHub build archive for version 1.0.6 is available from the
[xpCFY_TQ v1.0.6 release](https://github.com/simon1960/xpCFY_TQ/releases/tag/v1.0.6).

## Manual Installation

1. Close X-Plane.
2. Download the contents of the repository's `plugins` directory.
3. In the X-Plane installation, create this directory if it does not exist:

   ```text
   X-Plane 12\Resources\plugins\xpCFY_TQ\win_x64
   ```

4. Copy the files for the required operating system into its platform directory:

   ```text
   Windows win_x64:
   xpCFY_TQ.xpl
   PoKeyslib.dll

   Linux lin_x64:
   xpCFY_TQ.xpl
   libPoKeys.so
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
            ├── win_x64
            │   ├── xpCFY_TQ.xpl
            │   └── PoKeyslib.dll
            └── lin_x64
                ├── xpCFY_TQ.xpl
                └── libPoKeys.so
```

The Windows plugin loads `PoKeyslib.dll` from the same directory as `xpCFY_TQ.xpl`.
The Linux release includes `libPoKeys.so` in `lin_x64`; the plugin also supports the
system installation at `/usr/lib/libPoKeys.so`.

## First start and calibration

On first start, the plugin searches for the PoKeys controller and opens the TQ
calibration window if no valid calibration file exists. Follow the prompts and
save the calibration before normal operation.

Calibration can be repeated later from the **xpCFY_TQ > TQ Calibration** item
in X-Plane's Plugins menu. The status and live-position windows are available
from the same menu.

The **xpCFY_TQ > General Configuration** window selects the CFY TQ V3, V4 or
Pro hardware topology and contains the **PoKeys network: TCP/UDP** toggle. Select
**Save** to write both settings to `xpCFY_TQ.cfg`. When the hardware variant
changes, the worker makes the motor outputs safe and reconnects before applying
the new bridge topology. TCP is the default network transport. An active
Ethernet connection is safely closed and rediscovered when its protocol changes.
An active USB connection retains its transport, but the saved choice is used
for the next Ethernet connection.

The speedbrake, parking-brake and throttle test buttons are also located in the
General Configuration window. They are enabled only while the aircraft battery
is off and the aircraft is on the ground.

The saved `[connection]` section contains a readable
`network_protocol=TCP` or `network_protocol=UDP` entry. The plugin also writes
the legacy `network_use_udp=0|1` entry for compatibility. Existing files that
lack `network_protocol` are migrated automatically at startup, and every save
is read back to verify that the selected protocol reached disk.

Enhanced diagnostic logging is controlled by `enhanced_logging=0|1`. It defaults
to off for new configuration files and is also set to off when an existing file
lacks the entry. The migrated value is written back to the configuration file.

Due to a design flaw in the Version 3 throttle quadrants, it is possible to move
the trim wheel beyond its NOSE UP or NOSE DOWN limits which causes the trim wheel
motor to lock up. I have implemented a software gate that prevents the trim wheel
being moved electrically outside of defined limits.
At the start of the First Run synchronisation, the Ver 3 trim wheel position is read
and if the ADC position is between 400 and 3695, this is accepted immediately.
If the initial position is outside of these limits, the plugin inhibits the trim motor
and displays a warning directing the user to switch off the MAIN ELEC cutout and
manually rotate the wheel NOSE UP or NOSE DOWN as appropriate. After this warning, the
wheel must be moved into the central recovery range of 1059 through 3036 before
the position is accepted. A confirmation is then displayed and normal
simulator-owned trim synchronisation continues. The check is latched complete
and cannot interrupt subsequent motor-driven synchronisation.

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
- `linux/src/*.c` and `linux/inc/*.h` — Linux source and platform compatibility layer.
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

The Linux source is stored in `linux/src` and `linux/inc`. Build it as an x86-64
shared library with the Linux X-Plane SDK definitions and link against OpenGL,
pthread, `dl`, `m` and PoKeys. Install the resulting `xpCFY_TQ.xpl` and
`libPoKeys.so` beneath `Resources/plugins/xpCFY_TQ/lin_x64`.
