/**********************************************************************************/
/* FILE NAME: ppokeys_thread.h                                                    */
/*   VERSION: 1.0                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQ plugin for X-Plane 12.                 */
/**********************************************************************************/

#ifndef _XPCFY_TQ_POKEYS_THREAD_H_
#define _XPCFY_TQ_POKEYS_THREAD_H_

/* standard include files */
#include <stdint.h>
#include "plugin_config.h"

typedef struct PokeysStatus
{
	int			connected;
	uint32_t	serial_number;
	char		ip_address[16];
	char		protocol[8];
	char		firmware_version[32];
	char		detail[128];
} PokeysStatus;

enum PokeysLeverIndex
{
	POKEYS_LEVER_THROTTLE_1 = 0,
	POKEYS_LEVER_THROTTLE_2,
	POKEYS_LEVER_SPEED_BRAKE,
	POKEYS_LEVER_TRIM,
	POKEYS_LEVER_REVERSER_1,
	POKEYS_LEVER_REVERSER_2,
	POKEYS_LEVER_FLAPS,
	POKEYS_LEVER_COUNT
};

typedef struct PokeysLeverPositions
{
	int			connected;
	int			valid;
	uint32_t	value[POKEYS_LEVER_COUNT];
	uint64_t	sequence;
} PokeysLeverPositions;

/* coherent snapshot of the inverted parking-brake switch on PoKeys pin 0. */
typedef struct PokeysParkingBrakeInput
{
	int			connected;
	int			valid;
	int			engaged;
	uint64_t	sequence;
} PokeysParkingBrakeInput;

/* coherent snapshot of the inverted TO/GA buttons on PoKeys pins 1 and 2. */
typedef struct PokeysTogaInputs
{
	int			connected;
	int			valid;
	int			left_pressed;
	int			right_pressed;
	uint64_t	sequence;
} PokeysTogaInputs;

/* coherent snapshot of the inverted A/T disconnect buttons on API pins 3/4. */
typedef struct PokeysAtDisconnectInputs
{
	int			connected;
	int			valid;
	int			left_pressed;
	int			right_pressed;
	uint64_t	sequence;
} PokeysAtDisconnectInputs;

/* coherent snapshot of the inverted fuel-cutoff switches on API pins 5/6. */
typedef struct PokeysFuelCutoffInputs
{
	int			connected;
	int			valid;
	int			left_cutoff;
	int			right_cutoff;
	uint64_t	sequence;
} PokeysFuelCutoffInputs;

/* persistent stabilizer-trim cutout switches on API pins 7 and 9. */
typedef struct PokeysTrimCutoutInputs
{
	int			connected;
	int			valid;
	int			electric_normal;
	int			autopilot_normal;
	uint64_t	sequence;
} PokeysTrimCutoutInputs;

enum PokeysParkingBrakeState
{
	POKEYS_PARKING_BRAKE_UNKNOWN = 0,
	POKEYS_PARKING_BRAKE_RELEASED,
	POKEYS_PARKING_BRAKE_SET
};

/* forward declaration of functions */
int pokeys_thread_start(const PluginConfig* config);
int pokeys_thread_stop(void);
int pokeys_is_connected(void);
void pokeys_get_status(PokeysStatus* status);
void pokeys_get_lever_positions(PokeysLeverPositions* positions);
void pokeys_get_parking_brake_input(PokeysParkingBrakeInput* input);
void pokeys_get_toga_inputs(PokeysTogaInputs* inputs);
void pokeys_get_at_disconnect_inputs(PokeysAtDisconnectInputs* inputs);
void pokeys_get_fuel_cutoff_inputs(PokeysFuelCutoffInputs* inputs);
void pokeys_get_trim_cutout_inputs(PokeysTrimCutoutInputs* inputs);
void pokeys_set_parking_brake_indicator(int illuminated);
void pokeys_set_backlight(int illuminated);
void pokeys_set_aircraft_in_flight(int in_flight);
void pokeys_set_calibration_active(int active);
int pokeys_is_flight_detent_retracted(void);
void pokeys_set_speedbrake_closed_position(uint32_t position);
int pokeys_speedbrake_retract_and_pull_down(void);
int pokeys_speedbrake_push_up_and_extend(void);
int pokeys_parking_brake_interlock_release(void);
int pokeys_parking_brake_interlock_set(void);
int pokeys_get_parking_brake_state(void);
/*
 * Queue a release only when one is not already queued/in progress, and report
 * whether a completed release pulse has confirmed the interlock is retracted.
 * These calls let X-Plane startup wait without repeatedly restarting the pulse.
 */
int pokeys_ensure_parking_brake_interlock_retracted(void);
int pokeys_parking_brake_interlock_is_retracted(void);
/*
 * Publish the simulator-derived trim target to the PoKeys worker. Position is
 * in the trim feedback potentiometer's 0..4095 domain. The worker owns all
 * motor direction, braking and PWM calls.
 */
void pokeys_set_trim_target(uint32_t position, int enabled);
/* direction: -1=nose down/decreasing feedback, 0=stop, +1=nose up/increasing */
void pokeys_set_trim_manual_command(int direction, int enabled);
void pokeys_set_trim_indicator_target(uint32_t position, int simulator_owned);
void pokeys_set_trim_min_speed(uint32_t percent);
int pokeys_trim_motor_is_running(void);
/*
 * Publish calibrated raw-ADC throttle targets for normal A/T operation.
 * The worker owns all PoKeys PWM/direction I/O; the X-Plane thread only
 * supplies targets and whether simulator-follow currently owns the levers.
 */
void pokeys_set_throttle_follow_targets(uint32_t left_position,
	uint32_t right_position, uint32_t left_min_speed,
	uint32_t right_min_speed, int enabled);
/* Return and atomically clear bit 0=left / bit 1=right pilot intervention. */
int pokeys_take_throttle_manual_override(void);
void pokeys_set_throttle_test_limits(uint32_t left_min, uint32_t left_max,
	uint32_t right_min, uint32_t right_max);
int pokeys_start_throttle_test(void);
int pokeys_is_throttle_test_running(void);
void pokeys_get_throttle_test_status(char* status, uint32_t status_size);

#endif // !_XPCFY_TQ_POKEYS_THREAD_H_
