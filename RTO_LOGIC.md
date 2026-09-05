# Rejected Takeoff (RTO) Logic Review

**Review date:** 5 September 2026  
**Scope:** Original CFY TQ .NET application, current `xpCFY_TQ` X-Plane plugin, PoKeys hardware, and equivalent FSUIPC/XPUIPC offsets.

**Implementation status:** The experimental plugin RTO detector and all of its dedicated actions were removed on 5 September 2026 pending further investigation of the Boeing RTO process. Sections 5 through 8 preserve the removed implementation as a review record only; they do not describe active plugin behaviour.

## 1. Executive finding

The original .NET application does **not** contain a discrete rejected-takeoff detector. It does not read the autobrake selector, does not test for an `RTO` selector position, does not combine throttle-idle and brake-pressure inputs into an RTO event, and does not command maximum wheel braking.

The original application contains three related but separate behaviours:

1. Physical throttle idle detection, with a 500-count set threshold and 510-count release threshold.
2. Automatic physical speedbrake movement after touchdown or reverse-thrust selection.
3. Toe-brake thresholds of 4000 and 2000 for **parking-brake interlock supervision only**.

The now-removed plugin `ProcessTqRejectedTakeoff()` function was therefore new, requirement-driven logic rather than a direct port of an original RTO branch. It used a brake-ratio threshold of `0.20`, derived from the test log. No dedicated RTO detector or action remains in the plugin.

## 2. Signal-domain warning

The plugin communicates directly with X-Plane through datarefs and commands. It does not read or write FSUIPC/XPUIPC offsets. The offsets in this document are equivalent legacy interfaces for comparison with the original application.

Do not assume that numerically similar values have the same meaning:

- FSUIPC brake application uses `0..16383`.
- X-Plane brake-ratio datarefs use `0.0f..1.0f`.
- The CFY throttle potentiometers use corrected `0..4095` ADC counts.
- X-Plane forward throttle uses `0.0f..1.0f`; reverse uses `0.0f..-2.0f` in this plugin.
- FSUIPC engine throttle offsets use signed values from `-4096..+16384`.

## 3. Definitive FSUIPC offset map

These definitions were checked against `D:\FSUIPC6\SDK\FSUIPC for Programmers.pdf`.

| Offset | Size | Direction in original app | SDK meaning | Use in RTO-related flow |
|---:|---:|---|---|---|
| `0x02B4` | 4 bytes | Read | Groundspeed, `65536 * metres/second` | Original speedbrake/reverser automation tests for greater than 28 m/s; parking-brake supervision tests for less than 1 m/s |
| `0x0366` | 2 bytes | Read | Aircraft on-ground flag: 0 airborne, 1 on ground | Original and plugin ground gating |
| `0x0810` | 4 bytes | Read/write capable | Autothrottle Arm | Used elsewhere for A/T state; **not** a current plugin RTO trigger |
| `0x088C` | 2 bytes, signed | Read/write | Engine 1 throttle lever, `-4096..+16384` | Legacy equivalent of the plugin's left throttle-ratio dataref |
| `0x0924` | 2 bytes, signed | Read/write | Engine 2 throttle lever, `-4096..+16384` | Legacy equivalent of the plugin's right throttle-ratio dataref |
| `0x0BC4` | 2 bytes | Read; writable by SDK | Left brake application read-out, 0 off and 16383 full | Current plugin left-brake trigger input; one of two left-brake sources compared by the original app |
| `0x0BC6` | 2 bytes | Read; writable by SDK | Right brake application read-out, 0 off and 16383 full | Current plugin right-brake trigger input; one of two right-brake sources compared by the original app |
| `0x0BC8` | 2 bytes | Read/write | Parking brake, 0 off and 32767 on | Original parking-brake logic only; not an RTO output |
| `0x0BCC` | 4 bytes | Read/write | Spoilers armed, 0 off and 1 armed | Original touchdown extension input; related to landing, not a rejected-takeoff trigger |
| `0x0BD0` | 4 bytes | Write from lever feedback | Spoilers control: 0 off, 4800 armed, 5620 minimum extension, 16383 fully deployed | Legacy equivalent of the plugin's speedbrake-lever dataref |
| `0x3416` | 2 bytes | Read | Left brake axis input after calibration and before simulator application | Original app compares this with `0x0BC4` and retains the greater value for parking-brake supervision |
| `0x3418` | 2 bytes | Read | Right brake axis input after calibration and before simulator application | Original app compares this with `0x0BC6` and retains the greater value for parking-brake supervision |

The original PMDG SDK structure declares `MAIN_AutobrakeSelector`, but application code never reads that member. No fixed FSUIPC offset for an autobrake selector participates in the original flow.

The current `acf_dref.c` comment against `DREF_SPD_BRAKE_LEVER` spells the offset as `0x0BDO` with a final letter **O**. The SDK-defined offset is `0x0BD0` with a final zero. This is a documentation typo in the source comment, not a different offset used at runtime.

## 4. Original .NET application flow

### 4.1 What is not present

A complete source search finds no operational reference to `RTO`, rejected takeoff, `MAIN_AutobrakeSelector`, or `EVT_MPM_AUTOBRAKE_SELECTOR`. The latter two symbols exist only as generated PMDG SDK declarations.

Consequently, the original application has no branch equivalent to:

```text
autobrake selector = RTO
AND throttles = idle
AND brake force >= threshold
THEN disconnect A/T and extend speedbrake
```

It also does not write `0x0BC4` or `0x0BC6` to force maximum braking.

### 4.2 Original physical throttle idle state

Each corrected forward-throttle ADC value maintains a hysteretic idle flag:

```text
corrected throttle < 500  -> throttleLeverIdle = true
corrected throttle > 510  -> throttleLeverIdle = false
500..510                  -> retain previous state
```

On the plugin's normalized `0.0f..1.0f` scale:

- Idle set threshold: `500 / 4095 = 0.12210`.
- Idle release threshold: `510 / 4095 = 0.12454`.

This idle state is used by the original reverser and post-landing speedbrake logic. It is not combined with brake force to declare an RTO.

### 4.3 Original landing/reverse speedbrake automation

```text
Every internal-supervision update
    |
    +-- If state was FLYING and on-ground becomes true
    |      |
    |      +-- If physical speedbrake is ARMED
    |             -> drive physical speedbrake to 4095 (full extension)
    |      |
    |      +-- enter ON_GROUND state and start time-after-landing timer
    |
    +-- While ON_GROUND
           |
           +-- If either reverser is active
           |   AND groundspeed > 28 m/s
           |   AND physical speedbrake is DOWN or STOWED
           |      -> drive physical speedbrake to 4095
           |
           +-- If more than 5 seconds after landing
               AND either forward throttle is not idle
               AND (left reverser inactive OR right reverser inactive)
               AND physical speedbrake is UP
                  -> retract the physical speedbrake
```

The reverser expression above is transcribed literally from the original code. It does not require both reversers to be inactive; retraction remains eligible when either one is inactive.

The physical speedbrake's ADC feedback then passes through the normal simulator-write path. For generic FSUIPC operation that path writes offset `0x0BD0`. The original code does not use brake force or the autobrake selector in this sequence.

### 4.4 Original parking-brake pedal thresholds

The original application reads both the calibrated brake-axis inputs and the applied-brake read-outs:

```text
leftToeBrake  = max(FSUIPC 0x3416, FSUIPC 0x0BC4)
rightToeBrake = max(FSUIPC 0x3418, FSUIPC 0x0BC6)
```

When on the ground, almost stationary, and both simulator and physical parking-brake states are released:

```text
both toe brakes > 4000 -> release/retract the physical parking-brake interlock
both toe brakes < 2000 -> apply/set the physical parking-brake interlock
```

Normalized values are:

- `4000 / 16383 = 0.24416`.
- `2000 / 16383 = 0.12208`.

These are **parking-brake interlock thresholds**, not RTO brake thresholds. No original source path interprets 4000 as "RTO braking" or "maximum braking."

## 5. Removed plugin RTO inputs and triggers

Before removal, `ProcessTqRejectedTakeoff()` was called on every X-Plane flight-loop callback after `GetDataRefValues()` and `ProcessTqLeverWrites()`, and before normal A/T throttle-motor follow.

### 5.1 Input map

| Input | C storage/type | X-Plane source | PoKeys source | FSUIPC equivalent | Required state |
|---|---|---|---|---|---|
| Aircraft on ground | `acData.on_ground`, `int` | `sim/flightmodel2/gear/on_ground[2]` | None | `0x0366`, 2 bytes | Non-zero |
| Captain PFD speed mode | `acData.pfd_speed_mode_ca`, `float` | `laminar/B738/autopilot/pfd_spd_mode[0]` | None | No defined equivalent used here | `6.0f` (`THR HLD`) on either PFD |
| F/O PFD speed mode | `acData.pfd_speed_mode_fo`, `float` | `laminar/B738/autopilot/pfd_spd_mode[1]` | None | No defined equivalent used here | `6.0f` (`THR HLD`) on either PFD |
| Physical left throttle | retained filtered `float` | None | Analogue pin 40, array index 0, inverted and calibrated | `0x088C` only as simulator-axis equivalent | `<= 500/4095` |
| Physical right throttle | retained filtered `float` | None | Analogue pin 41, array index 1, inverted and calibrated | `0x0924` only as simulator-axis equivalent | `<= 500/4095` |
| Left brake ratio | `acData.left_brake`, `float` | `sim/cockpit2/controls/left_brake_ratio` | None | `0x0BC4`, 2 bytes, 0..16383 | `>= 0.20f` |
| Right brake ratio | `acData.right_brake`, `float` | `sim/cockpit2/controls/right_brake_ratio` | None | `0x0BC6`, 2 bytes, 0..16383 | `>= 0.20f` |
| Physical-position validity | internal `int` flag | None | Set after valid filtered PoKeys throttle samples | None | True |
| One-shot state | `g_rto_action_issued`, `int` | None | None | None | False |

`acData.at_arm` was not tested by the removed RTO function. The autobrake selector was also not present in the aircraft data block and was not tested.

### 5.2 Complete trigger expression

The removed implementation declared an RTO when all of the following were true in one flight-loop pass:

```text
valid retained physical throttle positions
AND aircraft on ground
AND (Captain PFD mode = THR HLD OR F/O PFD mode = THR HLD)
AND physical left throttle <= 0.12210
AND physical right throttle <= 0.12210
AND left simulator brake ratio >= 0.20
AND right simulator brake ratio >= 0.20
AND RTO one-shot latch is clear
```

The source names the brake booleans `left_brake_max` and `right_brake_max`, but `0.20` is a detection threshold, not the full-scale value `1.0`.

### 5.3 Latch and re-arm

The RTO action is latched before external actions are issued. Brake pressure or PFD-mode changes cannot repeat it during the same reject.

The latch is cleared only when:

- the aircraft leaves the ground; or
- either physical forward throttle moves above the idle threshold.

Dropping below the brake threshold or leaving `THR HLD` does not by itself re-arm the action.

## 6. Removed plugin RTO outputs

| Output/action | Interface and destination | PoKeys pins/channels | FSUIPC equivalent | Notes |
|---|---|---|---|---|
| Relinquish throttle motors | `pokeys_set_throttle_follow_targets(..., enabled = 0)` | Left direction 22, enable 24, PWM 5; right direction 23, enable 29, PWM 4 | None | Both throttle bridges are placed in coast state immediately |
| Inhibit further A/T ownership | Internal flags `g_manual_throttle_disconnect_pending = 1` and `g_unowned_throttle_tracking = 0` | None | None | Prevents a still-annunciated thrust mode from driving levers forward again |
| Write physical left idle position | `DREF_THROTTLE_RATIO_LT`, `sim/cockpit2/engine/actuators/throttle_jet_rev_ratio[0]` | Source feedback pin 40 | Legacy equivalent `0x088C` | Change-driven direct X-Plane write, normally near 0.0 |
| Write physical right idle position | `DREF_THROTTLE_RATIO_RT`, `sim/cockpit2/engine/actuators/throttle_jet_rev_ratio[1]` | Source feedback pin 41 | Legacy equivalent `0x0924` | Change-driven direct X-Plane write, normally near 0.0 |
| Disconnect autothrottle | One `XPLMCommandOnce()` on `laminar/B738/autopilot/left_at_dis_press` | None | No FSUIPC offset is used | Captain-side command only, to avoid a second command acknowledging/clearing the warning |
| Release speedbrake flight detent | PoKeys logical output | Pin 30, active-low | None | Forced retracted before motor movement |
| Drive speedbrake to full extension | Queued `pokeys_speedbrake_push_up_and_extend()` | Direction 25, enable 26, PWM channel 3 at duty 400000; bounded 2000 ms | Physical feedback ultimately corresponds to `0x0BD0` | The request is asynchronous; normal ADC feedback writes the moving lever position to X-Plane |
| Update simulator speedbrake lever | `DREF_SPD_BRAKE_LEVER`, `laminar/B738/flt_ctrls/speedbrake_lever` | Analogue pin 42, array index 2 | `0x0BD0`, 4 bytes | Occurs through the normal change-driven lever path, not as an immediate forced `1.0f` write in the RTO function |
| Diagnostic record | `xpCFY_TQ.log` | None | None | Records gate changes and the one-shot action result |

There was no RTO output to either wheel-brake dataref. The removed implementation observed the brake ratios; it did not force them to `1.0f`.

## 7. Removed execution flow

```text
X-Plane flight-loop callback
    |
    +-- GetDataRefValues()
    |      Reads on-ground, PFD modes, brake ratios and simulator levers
    |
    +-- ProcessTqLeverWrites()
    |      Filters/retains physical throttle positions
    |      Performs normal eligible lever writes
    |
    +-- ProcessTqRejectedTakeoff()
    |      |
    |      +-- Invalid physical throttle feedback? -> no action
    |      +-- Calculate six gates and diagnostic mask
    |      +-- Airborne or either throttle not idle? -> clear latch and exit
    |      +-- Not THR HLD, either brake below threshold, or already latched?
    |      |      -> exit
    |      +-- Latch event
    |      +-- Coast throttle motors and inhibit A/T ownership
    |      +-- Write retained physical idle throttle values to X-Plane
    |      +-- Issue Captain A/T-disconnect command once
    |      +-- Queue physical speedbrake full-extension command
    |
    +-- ProcessTqThrottleMotorFollow()
           Pending disconnect prevents motor reacquisition

PoKeys worker thread, subsequent cycle
    |
    +-- Consume speedbrake extension request
    +-- Retract flight-detent lock
    +-- Enable and drive speedbrake motor toward full extension
    +-- Stop after bounded movement interval
    +-- Publish ADC feedback

Later flight-loop callbacks
    |
    +-- Map speedbrake ADC feedback to 0.0..1.0
    +-- Write changed speedbrake lever value to Zibo dataref
```

## 8. Test-log evidence

The relocated test log captured this rejected-takeoff sequence with the previously deployed `4000/16383` (`0.24416`) threshold:

```text
16:53:57.281  ground=1 THR_HLD=1 throttle L 0.898/R 0.895
               idle L 0/R 0 brake L 0.000/R 0.000 max L 0/R 0

16:54:00.402  ground=1 THR_HLD=1 throttle L 0.094/R 0.096
               idle L 1/R 1 brake L 0.201/R 0.201 max L 0/R 0

16:54:21.936  ground=1 THR_HLD=0 throttle L 0.000/R 0.000
               idle L 1/R 1 brake L 0.254/R 0.254 max L 1/R 1
```

At the critical moment, the aircraft was on the ground, both PFDs reported `THR HLD`, and both physical throttles were idle. Zibo exposed `0.201` on both brake-ratio datarefs, so the then-deployed `0.24416` test failed. By the time both brake ratios exceeded `0.24416`, `THR HLD` had cleared.

The removed implementation's final local threshold was `0.20f`. On the FSUIPC 0..16383 scale, this is approximately 3277 counts. The observed `0.201` is approximately 3293 counts. That experimental implementation has now been deleted rather than deployed.

## 9. Comparison summary

| Behaviour | Original .NET application | Plugin after RTO-code removal |
|---|---|---|
| Explicit RTO detector | No | No |
| Autobrake selector required | No; selector is not consumed | Not applicable |
| PFD `THR HLD` required | No equivalent branch | Not applicable to RTO |
| Both physical throttles idle required | No RTO branch; idle flags used elsewhere | Not applicable to RTO |
| Both brake values required | Only for parking-brake interlock supervision | No RTO brake test |
| Forces maximum wheel braking | No | No |
| Disconnects A/T as an RTO action | No discrete RTO action | No |
| Immediately stops throttle motors as an RTO action | No discrete RTO action | No |
| Extends speedbrake following reverse selection | Yes, on ground above 28 m/s | Separate automatic path retained |
| Extends speedbrake from RTO signature | No | No |
| Speedbrake simulator update source | Physical ADC after motor movement | Physical ADC after motor movement |

## 10. Items requiring design agreement

The following choices cannot be resolved by copying the original application because it has no discrete RTO implementation:

1. **Event authority:** Should RTO be inferred from `THR HLD + throttles idle + brake ratio`, or should the Zibo autobrake selector explicitly have to be in RTO? The latter requires an additional dataref and aircraft-data-block member.
2. **Brake threshold semantics:** The test shows `0.201`, but it must be agreed whether this is a stable Zibo RTO demand, actual delivered brake force, or only the current scenario's brake-ratio value.
3. **PFD agreement:** The plugin currently accepts `THR HLD` on either PFD. Requiring both would reduce false positives but could miss a transient/asymmetric update.
4. **Speedbrake command authority:** The plugin currently moves the physical lever first and lets ADC feedback command Zibo. An alternative is to command the simulator lever immediately and make the hardware follow it.
5. **RTO versus manual braking:** Without an autobrake-selector input, sufficiently strong manual braking in `THR HLD` with both throttles idle is indistinguishable from an autobrake RTO event.
6. **Completion feedback:** The speedbrake motor command is time-bounded. The RTO path does not currently verify that the physical lever reaches its calibrated full-extension endpoint before declaring the request complete.

No RTO code should be reintroduced until these points have been reviewed and the intended authority chain has been selected.

## 11. Source locations reviewed

- Original control flow: `C:\CFY_TQ_SAFE\CFY_TQ_XPlane\CFY_TQ_Console\CFY_TQ_ReadValues.cs`, especially `InternalSupervision()`, throttle idle detection, spoiler ADC processing, and `ThrottleQ_DeploySpoilers()`.
- Original FSUIPC registration and data acquisition: `C:\CFY_TQ_SAFE\CFY_TQ_XPlane\CFY_TQ_Console\FsInterface.cs`.
- Original generated PMDG declarations: `C:\CFY_TQ_SAFE\CFY_TQ_XPlane\CFY_TQ_Console\PMDG_SDK.cs`.
- Removed RTO implementation and surviving dataref logic: `D:\B738_Sim_Development\xpCFY_TQ\src\acf_dref.c`.
- Current PoKeys actuator flow: `D:\B738_Sim_Development\xpCFY_TQ\src\pokeys_thread.c`.
- Current aircraft data block and table enumerations: `D:\B738_Sim_Development\xpCFY_TQ\inc\datastructures.h`.
- Definitive offset specification: `D:\FSUIPC6\SDK\FSUIPC for Programmers.pdf`.
- Test evidence: `\\192.168.1.253\X-Plane 12 Test\Resources\plugins (disabled)\xpCFY_TQ\win_x64\xpCFY_TQ.log`.
