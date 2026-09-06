/**********************************************************************************/
/* FILE NAME: acf_dref.c                                                          */
/*   VERSION: 1.0                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQplugin for X-Plane 12.                  */
/**********************************************************************************/

/* standard include files */
#include <stdbool.h>
#include <string.h>

/* project include files*/
#include "datastructures.h"
#include "acf_dref.h"
#include "log.h"
#include "pokeys_thread.h"
#include "XPLMProcessing.h"

/* global variables */
TQAircraftData	acData;																				// aircraft data block

/*
 * Lever input processing is owned by X-Plane's flight-loop thread. The
 * PoKeys worker only publishes coherent ADC snapshots and never calls XPLM.
 */

#define TQ_MOVING_AVERAGE_SAMPLES 10U
#define TQ_FLAPS_FILTER_SAMPLES 22U

typedef struct TqMovingAverage
{
	float				samples[TQ_MOVING_AVERAGE_SAMPLES];
	float				total;
	uint32_t			next;
	int					initialised;
} TqMovingAverage;

/*
 * The original .NET application gives the flap lever a 22-sample MinMax
 * filter. The lowest and highest samples are discarded, leaving the mean of
 * 20 samples; the second extremes are rejected when separated from their
 * neighbours by more than 400 ADC counts.
 */
typedef struct TqFlapsFilter
{
	float				samples[TQ_FLAPS_FILTER_SAMPLES];
	float				average;
	uint32_t			next;
	uint32_t			sample_count;
	uint32_t			large_change_count;
	int					initialised;
} TqFlapsFilter;

static TqCalibration	g_control_calibration;
static int				g_control_calibration_valid;
static uint64_t			g_last_lever_sequence;
static TqMovingAverage	g_throttle_left_filter;
static TqMovingAverage	g_throttle_right_filter;
static TqMovingAverage	g_speedbrake_filter;
static TqMovingAverage	g_trim_filter;
static TqMovingAverage	g_reverser_left_filter;
static TqMovingAverage	g_reverser_right_filter;
static TqFlapsFilter		g_flaps_filter;
static float			g_last_throttle_left_written;
static float			g_last_throttle_right_written;
static float			g_last_speedbrake_written;
static int				g_throttle_left_written;
static int				g_throttle_right_written;
static int				g_speedbrake_written;
static float			g_physical_throttle_left;
static float			g_physical_throttle_right;
static int				g_physical_throttle_positions_valid;
static int				g_reverser_left_active;
static int				g_reverser_right_active;
static int				g_last_flaps_detent;
static int				g_flaps_written;
static uint64_t			g_last_trim_sequence;
static float			g_last_trim_position;
static float			g_last_simulator_trim;
static int				g_trim_input_initialised;
static uint64_t			g_last_parking_brake_switch_sequence;
static int				g_last_parking_brake_interlock_request = POKEYS_PARKING_BRAKE_UNKNOWN;
static int				g_parking_brake_input_initialised;
static int				g_parking_brake_toe_release_armed;
static uint64_t			g_last_fuel_cutoff_sequence;
static uint64_t			g_last_trim_cutout_sequence;
static int				g_trim_cutout_electric_commanded;
static int				g_trim_cutout_autopilot_commanded;
static float			g_trim_cutout_electric_command_time;
static float			g_trim_cutout_autopilot_command_time;
static int				g_trim_cutout_waiting_for_guards;
static int				g_trim_first_run_active;
static int				g_manual_trim_command_direction;
static int				g_last_trim_dataref_direction;
static int				g_trim_dataref_input_initialised;
static int				g_toga_selected_since_at_arm;
static int				g_manual_throttle_disconnect_pending;
static int				g_unowned_throttle_tracking;
static float			g_unowned_throttle_left_reference;
static float			g_unowned_throttle_right_reference;
static int				g_last_speedbrake_state;
static int				g_speedbrake_ground_state_initialised;
static int				g_speedbrake_was_on_ground;
static int				g_speedbrake_auto_extend_issued;
static int				g_speedbrake_auto_retract_issued;
static float			g_speedbrake_touchdown_time;
static uint32_t			g_first_run_pending;

/*
 * set after CMD_PB_SET begins and retained until a following read pass reports
 * the simulator parking-brake lamp. While set, normal mismatch supervision
 * must not undo the interlock command using the previous frame's simulator
 * value.
 */

static int				g_parking_brake_set_awaiting_simulator;
static float			g_parking_brake_set_started_at;
static int				g_parking_brake_set_timeout_logged;
static int				g_parking_brake_set_command_active;
static int				g_parking_brake_release_state;

/* Original FSUIPC pedal thresholds converted from 0..16383 to X-Plane 0..1. */
#define TQ_PARKING_BRAKE_PEDALS_HIGH				(4000.0f / 16383.0f)
#define TQ_PARKING_BRAKE_PEDALS_LOW					(2000.0f / 16383.0f)
#define TQ_PARKING_BRAKE_MAX_GROUND_SPEED_MPS		1.0f
#define TQ_PARKING_BRAKE_CONFIRM_TIMEOUT_SECONDS	2.0f
/* Original .NET idle latch: set below 500 counts, clear above 510 counts. */
#define TQ_THROTTLE_IDLE_THRESHOLD					(500.0f / 4095.0f)
#define TQ_REVERSER_ACTIVATE_THRESHOLD				(250.0f / 4095.0f)
#define TQ_REVERSER_RELEASE_THRESHOLD				(150.0f / 4095.0f)
#define TQ_REVERSER_FULL_SCALE						-2.0f
#define TQ_THROTTLE_MANUAL_MOVEMENT_THRESHOLD		(65.0f / 4095.0f)
#define TQ_THROTTLE_DATAREF_EPSILON					(0.5f / 4095.0f)

#define TQ_SPEEDBRAKE_DOWN_MAX_COUNTS				250U
#define TQ_SPEEDBRAKE_ARM_MAX_COUNTS				1250U
#define TQ_SPEEDBRAKE_UP_MIN_COUNTS					3218U
#define TQ_SPEEDBRAKE_ARM_VALUE						0.0889f
#define TQ_SPEEDBRAKE_FLIGHT_VALUE					0.667f
#define TQ_SPEEDBRAKE_AUTO_RETRACT_DELAY_SECONDS	5.0f
#define TQ_TRIM_POSITION_MIN						400.0f
#define TQ_TRIM_POSITION_MAX						3695.0f
#define TQ_TRIM_PHYSICAL_CHANGE_COUNTS				2.0f
#define TQ_TRIM_SYNC_TOLERANCE_COUNTS				50.0f
#define TQ_TRIM_SIM_CHANGE_EPSILON					0.000001f
#define TQ_ZIBO_TRIM_SWITCH_LOW_MAX					0.25f
#define TQ_ZIBO_TRIM_SWITCH_HIGH_MIN				0.75f

/* original generic/XPUIPC mapping: -16383..12312 -> 0..4095 counts. */
#define TQ_TRIM_SIM_MIN								-1.0f
#define TQ_TRIM_SIM_MAX								(12312.0f / 16383.0f)
#define TQ_SWITCH_COMMAND_RETRY_SECONDS				0.5f
#define TQ_FLAPS_DATAREF_EPSILON					0.0001f

/* various states for speedbrake and parking brake */
enum TqSpeedbrakeState
{
	TQ_SPEEDBRAKE_DOWN = 0,
	TQ_SPEEDBRAKE_ARMED,
	TQ_SPEEDBRAKE_FLIGHT,
	TQ_SPEEDBRAKE_UP
};

enum TqParkingBrakeReleaseState
{
	TQ_PB_RELEASE_IDLE = 0,
	TQ_PB_RELEASE_WAITING_FOR_SWITCH,
	TQ_PB_RELEASE_VERIFYING_SIMULATOR
};

enum TqFirstRunComponent
{
	TQ_FIRST_RUN_PARKING_BRAKE = 1U << 0,
	TQ_FIRST_RUN_FUEL_CUTOFFS = 1U << 1,
	TQ_FIRST_RUN_TRIM_CUTOUTS = 1U << 2,
	TQ_FIRST_RUN_TRIM_POSITION = 1U << 3,
	TQ_FIRST_RUN_FLAPS = 1U << 4
};

/*
 * the original application receives a three-state yoke trim input (0=idle,
 * 1=nose down, 2=nose up). X-Plane supplies the same information as command
 * phases. The electrical commands are the physical yoke assignments on the
 * test installation; the remaining bindings retain aircraft compatibility.
 * Observing them does not consume or replace X-Plane/Zibo's own handlers.
 */
typedef struct TqTrimCommandBinding
{
	const char*		name;
	XPLMCommandRef	handle;
	int				direction;
	int				active;
} TqTrimCommandBinding;

static TqTrimCommandBinding g_trim_command_bindings[] =
{
	/* Physical yoke assignments reported from the test installation. */
	{ "sim/flight_controls/pitch_trim_down_elec", NULL, -1, 0 },
	{ "sim/flight_controls/pitch_trim_up_elec", NULL, 1, 0 },
	/* Zibo Captain and First Officer wrappers used by its flight-control code. */
	{ "laminar/B738/flight_controls/pitch_trim_down", NULL, -1, 0 },
	{ "laminar/B738/flight_controls/pitch_trim_up", NULL, 1, 0 },
	{ "laminar/B738/flight_controls/fo_pitch_trim_down", NULL, -1, 0 },
	{ "laminar/B738/flight_controls/fo_pitch_trim_up", NULL, 1, 0 },
	/* Native X-Plane commands retained for non-wrapped and direct assignments. */
	{ "sim/flight_controls/pitch_trim_down", NULL, -1, 0 },
	{ "sim/flight_controls/pitch_trim_up", NULL, 1, 0 },
	{ "sim/flight_controls/pitch_trimA_down", NULL, -1, 0 },
	{ "sim/flight_controls/pitch_trimA_up", NULL, 1, 0 },
	{ "sim/flight_controls/pitch_trimB_down", NULL, -1, 0 },
	{ "sim/flight_controls/pitch_trimB_up", NULL, 1, 0 }
};
static int g_trim_command_handlers_registered;

#define TQ_FIRST_RUN_ALL (TQ_FIRST_RUN_PARKING_BRAKE | TQ_FIRST_RUN_FUEL_CUTOFFS | TQ_FIRST_RUN_TRIM_CUTOUTS | TQ_FIRST_RUN_TRIM_POSITION | TQ_FIRST_RUN_FLAPS)

static void first_run_component_complete(uint32_t component, const char* description)
{
	if ((g_first_run_pending & component) == 0U)
		return;
	g_first_run_pending &= ~component;
	log_write("First Run synchronised: %s", description);
	if (g_first_run_pending == 0U)
		log_write("First Run complete: simulator switches and flap lever match the hardware TQ; trim wheel and indicator match the simulator");
}

static float clamp_unit(float value)
{
	if (value < 0.0f) return (0.0f);
	if (value > 1.0f) return (1.0f);
	return (value);
}

static float map_calibrated_axis(float value, uint32_t minimum, uint32_t maximum)
{
	if (maximum <= minimum)
		return (0.0f);

	return (clamp_unit((value - (float)minimum) / (float)(maximum - minimum)));
}

/* seed all slots from the first sample so connection cannot cause a zero ramp. */
static float moving_average_add(TqMovingAverage* filter, float sample)
{
	uint32_t index;
	if (!filter->initialised)
	{
		for (index = 0; index < TQ_MOVING_AVERAGE_SAMPLES; ++index)
			filter->samples[index] = sample;
		filter->total = sample * (float)TQ_MOVING_AVERAGE_SAMPLES;
		filter->next = 0;
		filter->initialised = 1;
		return(sample);
	}

	filter->total -= filter->samples[filter->next];
	filter->samples[filter->next] = sample;
	filter->total += sample;
	filter->next = (filter->next + 1U) % TQ_MOVING_AVERAGE_SAMPLES;
	return(filter->total / (float)TQ_MOVING_AVERAGE_SAMPLES);
}

static float absolute_difference(float left, float right)
{
	float difference = left - right;
	return (difference < 0.0f ? -difference : difference);
}

/* Port of the original flap lever's 22-sample MinMax input treatment. */
static float flaps_filter_add(TqFlapsFilter* filter, float sample)
{
	float sorted[TQ_FLAPS_FILTER_SAMPLES];
	float total = 0.0f;
	uint32_t index;

	/* Seed from live hardware so First Run never ramps up from zero. */
	if (!filter->initialised)
	{
		for (index = 0U; index < TQ_FLAPS_FILTER_SAMPLES; ++index)
			filter->samples[index] = sample;
		filter->average = sample;
		filter->next = 0U;
		filter->sample_count = 1U;
		filter->large_change_count = 0U;
		filter->initialised = 1;
		return (sample);
	}

	/*
	 * After the filter has settled, reject a greater-than-3000-count jump for
	 * ten samples. An intentional full-travel movement is accepted on the
	 * eleventh sample, matching the original application's timeout.
	 */
	if (filter->sample_count > 200U && absolute_difference(sample, filter->average) > 3000.0f)
	{
		if (filter->large_change_count < 10U)
		{
			sample = filter->average;
			++filter->large_change_count;
		}
		else
		{
			for (index = 0U; index < TQ_FLAPS_FILTER_SAMPLES; ++index)
				filter->samples[index] = sample;
			filter->average = sample;
			filter->next = 0U;
			filter->large_change_count = 0U;
			++filter->sample_count;
			return (sample);
		}
	}
	else
	{
		filter->large_change_count = 0U;
	}

	filter->samples[filter->next] = sample;
	filter->next = (filter->next + 1U) % TQ_FLAPS_FILTER_SAMPLES;
	memcpy(sorted, filter->samples, sizeof(sorted));
	for (index = 1U; index < TQ_FLAPS_FILTER_SAMPLES; ++index)
	{
		float value = sorted[index];
		uint32_t position = index;
		while (position > 0U && sorted[position - 1U] > value)
		{
			sorted[position] = sorted[position - 1U];
			--position;
		}
		sorted[position] = value;
	}

	if (absolute_difference(sorted[1], sorted[2]) > 400.0f)
		sorted[1] = filter->average;
	if (absolute_difference(sorted[TQ_FLAPS_FILTER_SAMPLES - 3U], sorted[TQ_FLAPS_FILTER_SAMPLES - 2U]) > 400.0f)
		sorted[TQ_FLAPS_FILTER_SAMPLES - 2U] = filter->average;
	for (index = 1U; index < TQ_FLAPS_FILTER_SAMPLES - 1U; ++index)
		total += sorted[index];
	filter->average = total / (float)(TQ_FLAPS_FILTER_SAMPLES - 2U);
	++filter->sample_count;
	return (filter->average);
}

/* Reproduce DC_motorController.CorrectedPos() as an integer ADC value. */
static uint32_t flaps_corrected_position(float raw_position)
{
	float corrected;
	uint32_t minimum = g_control_calibration.flaps_min_position;
	uint32_t maximum = g_control_calibration.flaps_max_position;

	if (maximum <= minimum || raw_position <= (float)minimum) return (0U);
	if (raw_position >= (float)maximum) return (4095U);
	corrected = (raw_position - (float)minimum) * 4095.0f / (float)(maximum - minimum);
	return ((uint32_t)corrected);
}

/* Original corrected-position bands for UP, 1, 2, 5, 10, 15, 25, 30, 40. */
static int flaps_position_to_detent(uint32_t position)
{
	if (position <= 200U) return (0);
	if (position <= 900U) return (1);
	if (position <= 1300U) return (2);
	if (position <= 1950U) return (3);
	if (position <= 2400U) return (4);
	if (position <= 2900U) return (5);
	if (position <= 3300U) return (6);
	if (position <= 3900U) return (7);
	return (8);
}

/*
 * Hold Zibo/X-Plane's parking-brake SET command across flight-loop updates.
 * A one-shot command was observed to set and then immediately release the
 * simulator lever. The simulator parking-brake lamp is the acknowledgement
 * that permits CommandEnd; release/cancellation paths also end a held command
 * so X-Plane is never left with a permanently active command phase.
 */
static int begin_parking_brake_set_command(void)
{
	if (g_parking_brake_set_command_active) return(1);
	if (cmdTable[CMD_PB_SET].handle == NULL)
	{
		log_write("Parking-brake SET command unavailable: CMD_PB_SET has no handle");
		return(0);
	}
	XPLMCommandBegin(cmdTable[CMD_PB_SET].handle);
	g_parking_brake_set_command_active = 1;
	log_write("Parking-brake SET command begun; holding until simulator lamp acknowledgement");
	return(1);
}

static void end_parking_brake_set_command(const char* reason)
{
	if (!g_parking_brake_set_command_active) return;
	if (cmdTable[CMD_PB_SET].handle != NULL)
		XPLMCommandEnd(cmdTable[CMD_PB_SET].handle);
	g_parking_brake_set_command_active = 0;
	log_write("Parking-brake SET command ended: %s", reason);
}

/*
 * Zibo's parking-brake toggle command also clears its internal held-brake
 * state. Release paths call it only after pin 0 confirms that the physical
 * motorised switch is released; no parking-brake dataref write is required.
 */
static int ensure_parking_brake_released(void)
{
	/*
	 * CMD_PB_SET is a toggle. Toe braking can clear X-Plane's parking brake
	 * before the motorised TQ handle reaches RELEASE, so never pulse a
	 * simulator state that the current completed read already reports clear.
	 */
	if (acData.parking_brake < 0.5f)
	{
		log_write("Parking-brake simulator state already released; CMD_PB_SET pulse suppressed");
		return(1);
	}
	if (cmdTable[CMD_PB_SET].handle == NULL)
	{
		log_write("Parking-brake release command unavailable: CMD_PB_SET has no handle");
		return(0);
	}
	XPLMCommandOnce(cmdTable[CMD_PB_SET].handle);
	log_write("Parking-brake physical switch released: CMD_PB_SET pulsed once to clear X-Plane internal brake state");
	return(1);
}

static void update_manual_trim_command_direction(void)
{
	size_t index;
	int up = 0;
	int down = 0;
	for (index = 0; index < sizeof(g_trim_command_bindings) / sizeof(g_trim_command_bindings[0]); ++index)
	{
		if (!g_trim_command_bindings[index].active) 
			continue;
		if (g_trim_command_bindings[index].direction > 0)
			up = 1;
		if (g_trim_command_bindings[index].direction < 0) 
			down = 1;
	}
	/* Opposing commands cancel, matching a neutral three-state switch input. */
	g_manual_trim_command_direction = up == down ? 0 : (up ? 1 : -1);
}

static int tq_trim_command_handler(XPLMCommandRef command, XPLMCommandPhase phase, void* refcon)
{
	TqTrimCommandBinding* binding = (TqTrimCommandBinding*)refcon;
	int previous_direction = g_manual_trim_command_direction;
	int motor_allowed;

	(void)command;

	if (binding == NULL) 
		return(1);
	binding->active = phase != xplm_CommandEnd;
	update_manual_trim_command_direction();
	/*
	 * Publish manual trim immediately from the command callback so a short
	 * electrical yoke command cannot be missed between FLCBs. The following
	 * read pass maintains the same direction while the command remains active.
	 * A/T state is deliberately absent: only an engaged A/P transfers ownership.
	 */
	if (!g_trim_first_run_active && acData.ap_engaged < 0.5)
	{
		motor_allowed = acData.paused == 0 && acData.battery_on != 0.0 && acData.el_trim_pos < 0.5f;
		pokeys_set_trim_target(0U, 0);
		pokeys_set_trim_manual_command(g_manual_trim_command_direction,	motor_allowed);
	}
	else if (acData.ap_engaged >= 0.5)
	{
		pokeys_set_trim_manual_command(0, 0);
	}
	if (previous_direction != g_manual_trim_command_direction)
		log_write("Manual trim command changed: %s via %s (A/P %s)", g_manual_trim_command_direction < 0 ? "nose down" : (g_manual_trim_command_direction > 0 ? "nose up" : "released"), binding->name,	acData.ap_engaged >= 0.5 ? "engaged" : "disengaged");
	
	/* Observe only: X-Plane and Zibo must still receive and act on the command. */
	return(1);
}

void UnregisterTqTrimCommandHandlers(void)
{
	size_t index;
	if (!g_trim_command_handlers_registered) return;
	for (index = 0; index < sizeof(g_trim_command_bindings) / sizeof(g_trim_command_bindings[0]); ++index)
	{
		TqTrimCommandBinding* binding = &g_trim_command_bindings[index];
		if (binding->handle != NULL)
			XPLMUnregisterCommandHandler(binding->handle, tq_trim_command_handler, 1, binding);
		binding->handle = NULL;
		binding->active = 0;
	}
	g_trim_command_handlers_registered = 0;
	g_manual_trim_command_direction = 0;
	pokeys_set_trim_manual_command(0, 0);
}

static void RegisterTqTrimCommandHandlers(void)
{
	size_t index;
	UnregisterTqTrimCommandHandlers();
	for (index = 0; index < sizeof(g_trim_command_bindings) / sizeof(g_trim_command_bindings[0]); ++index)
	{
		TqTrimCommandBinding* binding = &g_trim_command_bindings[index];
		binding->handle = XPLMFindCommand(binding->name);
		binding->active = 0;
		if (binding->handle == NULL)
		{
			log_write("Unable to find manual trim command %s", binding->name);
			continue;
		}
		XPLMRegisterCommandHandler(binding->handle,	tq_trim_command_handler, 1, binding);
	}
	g_trim_command_handlers_registered = 1;
	update_manual_trim_command_direction();
	log_write("Manual trim command monitoring registered, including X-Plane electrical yoke trim commands");
}

void TqControlsReset(void)
{
	end_parking_brake_set_command("TQ controls reset");
	pokeys_set_trim_target(0U, 0);
	pokeys_set_trim_manual_command(0, 0);
	pokeys_set_trim_indicator_target(0U, 0);
	pokeys_set_throttle_follow_targets(0U, 0U, 0U, 0U, 0);
	(void)pokeys_take_throttle_manual_override();
	memset(&g_throttle_left_filter, 0, sizeof(g_throttle_left_filter));
	memset(&g_throttle_right_filter, 0, sizeof(g_throttle_right_filter));
	memset(&g_speedbrake_filter, 0, sizeof(g_speedbrake_filter));
	memset(&g_trim_filter, 0, sizeof(g_trim_filter));
	memset(&g_reverser_left_filter, 0, sizeof(g_reverser_left_filter));
	memset(&g_reverser_right_filter, 0, sizeof(g_reverser_right_filter));
	memset(&g_flaps_filter, 0, sizeof(g_flaps_filter));
	g_last_lever_sequence = 0;
	g_throttle_left_written = 0;
	g_throttle_right_written = 0;
	g_speedbrake_written = 0;
	g_physical_throttle_left = 0.0f;
	g_physical_throttle_right = 0.0f;
	g_physical_throttle_positions_valid = 0;
	g_reverser_left_active = 0;
	g_reverser_right_active = 0;
	g_last_flaps_detent = -1;
	g_flaps_written = 0;
	g_last_trim_sequence = 0;
	g_last_trim_position = 0.0f;
	g_last_simulator_trim = 0.0f;
	g_trim_input_initialised = 0;
	g_last_parking_brake_switch_sequence = 0;
	g_last_parking_brake_interlock_request = POKEYS_PARKING_BRAKE_UNKNOWN;
	g_parking_brake_input_initialised = 0;
	g_parking_brake_toe_release_armed = 0;
	g_parking_brake_set_awaiting_simulator = 0;
	g_parking_brake_set_started_at = 0.0f;
	g_parking_brake_set_timeout_logged = 0;
	g_parking_brake_set_command_active = 0;
	g_parking_brake_release_state = TQ_PB_RELEASE_IDLE;
	g_last_fuel_cutoff_sequence = 0;
	g_last_trim_cutout_sequence = 0;
	g_trim_cutout_electric_commanded = 0;
	g_trim_cutout_autopilot_commanded = 0;
	g_trim_cutout_electric_command_time = 0.0f;
	g_trim_cutout_autopilot_command_time = 0.0f;
	g_trim_cutout_waiting_for_guards = 0;
	g_trim_first_run_active = 1;
	g_manual_trim_command_direction = 0;
	g_last_trim_dataref_direction = 0;
	g_trim_dataref_input_initialised = 0;
	g_toga_selected_since_at_arm = 0;
	g_manual_throttle_disconnect_pending = 0;
	g_unowned_throttle_tracking = 0;
	g_unowned_throttle_left_reference = 0.0f;
	g_unowned_throttle_right_reference = 0.0f;
	g_last_speedbrake_state = -1;
	g_speedbrake_ground_state_initialised = 0;
	g_speedbrake_was_on_ground = 0;
	g_speedbrake_auto_extend_issued = 0;
	g_speedbrake_auto_retract_issued = 0;
	g_speedbrake_touchdown_time = 0.0f;
	g_first_run_pending = TQ_FIRST_RUN_ALL;
	log_write("First Run started: waiting for simulator data and valid TQ inputs");
}

void TqControlsSetCalibration(const TqCalibration* calibration, int valid)
{
	if (calibration != NULL) g_control_calibration = *calibration;
	g_control_calibration_valid = valid != 0;
	pokeys_set_trim_min_speed(valid && calibration != NULL ? calibration->trim_min_speed : 0U);
	TqControlsReset();
}

/* 
 * check the aircraft is on the ground and the battery master is off
 * before we allow any motorised test runs (e.g. speedbrake, throttles)
 */
int TqGroundTestControlsAllowed(void)
{
	return (acData.battery_on == 0.0f && acData.on_ground != 0);
}

static void set_float_dataref(dataRefLine line, float value)
{
	drefTable_p entry = &drefTable[line];

	if (entry->handle == NULL || !entry->isWriteable) 
		return;
	
	if (entry->isArray)
		XPLMSetDatavf(entry->handle, &value, entry->arrayOffset, 1);
	else
		XPLMSetDataf(entry->handle, value);
	entry->value.fltData = value;

	if (entry->ptrVal != NULL) 
		*(float*)entry->ptrVal = value;
}

/*
 * queue an interlock pulse until the PoKeys worker confirms that the PWM pulse
 * actually started.  The public PoKeys call only queues work, so accepting the
 * call is not itself proof that the mechanical-state command was applied.
 */
static void request_parking_brake_interlock(int requested_state)
{
	int accepted;
	if (requested_state == g_last_parking_brake_interlock_request && pokeys_get_parking_brake_state() == requested_state) 
		return;
	
	accepted = requested_state == POKEYS_PARKING_BRAKE_SET ? pokeys_parking_brake_interlock_set() :	pokeys_parking_brake_interlock_release();
	
	if (accepted) 
		g_last_parking_brake_interlock_request = requested_state;
}

/*
 * port of the original parking-brake switch, lamp and interlock supervision.
 * pin I/O remains on the PoKeys worker; only the change-driven simulator write
 * is performed here on X-Plane's flight-loop thread.
 */
static void ProcessTqParkingBrake(void)
{
	PokeysParkingBrakeInput input;
	int simulator_indicator_set;
	int both_pedals_high;
	int both_pedals_low;
	float now;

	/* The PoKeys worker applies the lamp's active-low electrical polarity. */
	pokeys_set_parking_brake_indicator(acData.battery_on > 0.0f && acData.pb_indicator != 0);

	pokeys_get_parking_brake_input(&input);
	if (!input.connected || !input.valid)
	{
		end_parking_brake_set_command("parking-brake hardware input unavailable");
		g_last_parking_brake_interlock_request = POKEYS_PARKING_BRAKE_UNKNOWN;
		g_parking_brake_input_initialised = 0;
		g_parking_brake_toe_release_armed = 0;
		g_parking_brake_set_awaiting_simulator = 0;
		g_parking_brake_set_started_at = 0.0f;
		g_parking_brake_set_timeout_logged = 0;
		g_parking_brake_release_state = TQ_PB_RELEASE_IDLE;
		return;
	}

	simulator_indicator_set = acData.pb_indicator != 0;
	both_pedals_high = acData.left_brake > TQ_PARKING_BRAKE_PEDALS_HIGH && acData.right_brake > TQ_PARKING_BRAKE_PEDALS_HIGH;
	both_pedals_low = acData.left_brake < TQ_PARKING_BRAKE_PEDALS_LOW && acData.right_brake < TQ_PARKING_BRAKE_PEDALS_LOW;
	now = XPLMGetElapsedTime();

	/*
	 * First Run makes the maintained hardware switch authoritative. A SET
	 * switch still preserves the required ordering: confirm the mechanical
	 * interlock first, then hold CMD_PB_SET until the simulator lamp responds. A
	 * RELEASE switch clears both
	 * sides immediately. This branch remains active until the worker has
	 * accepted the required interlock state.
	 */
	if (!g_parking_brake_input_initialised)
	{
		if (input.engaged)
		{
			request_parking_brake_interlock(POKEYS_PARKING_BRAKE_SET);
			if (pokeys_get_parking_brake_state() != POKEYS_PARKING_BRAKE_SET)
				return;
			if (!simulator_indicator_set)
			{
				if (!begin_parking_brake_set_command()) 
					return;
				g_parking_brake_set_awaiting_simulator = 1;
				g_parking_brake_set_started_at = now;
				g_parking_brake_set_timeout_logged = 0;
			}
		}
		else
		{
			request_parking_brake_interlock(POKEYS_PARKING_BRAKE_RELEASED);
			if (pokeys_get_parking_brake_state() !=	POKEYS_PARKING_BRAKE_RELEASED)
				return;
			/* Do not toggle an already-released simulator on First Run. */
			if (acData.parking_brake >= 0.5f &&	!ensure_parking_brake_released())
				return;
		}
		g_parking_brake_input_initialised = 1;
		g_last_parking_brake_switch_sequence = input.sequence;
		g_parking_brake_release_state = TQ_PB_RELEASE_IDLE;
		g_parking_brake_toe_release_armed = 0;
		if (!input.engaged || simulator_indicator_set)
			first_run_component_complete(TQ_FIRST_RUN_PARKING_BRAKE, "parking-brake switch and interlock");
		
		log_write("Parking-brake First Run synchronised from hardware: switch %s", input.engaged ? "set" : "released");
		return;
	}

	/*
	 * After a toe-brake release command, do not re-issue SET merely because the
	 * motorised switch has not yet completed its movement.  The PoKeys worker's
	 * release pulse takes longer than one flight-loop pass.
	 */
	if (g_parking_brake_release_state != TQ_PB_RELEASE_IDLE)
	{
		if (g_parking_brake_release_state ==
			TQ_PB_RELEASE_WAITING_FOR_SWITCH)
		{
			if (input.engaged)
			{
				request_parking_brake_interlock(POKEYS_PARKING_BRAKE_RELEASED);
				return;
			}

			/*
			 * Wait for the next completed dataref read before deciding whether a
			 * toggle is needed. This lets X-Plane's toe-release response settle.
			 */
			g_parking_brake_release_state =
				TQ_PB_RELEASE_VERIFYING_SIMULATOR;
			g_last_parking_brake_switch_sequence = input.sequence;
			log_write("Parking-brake physical switch released; verifying simulator state on next flight-loop update");
			return;
		}

		if (!ensure_parking_brake_released()) return;
		g_parking_brake_release_state = TQ_PB_RELEASE_IDLE;
		log_write("Parking-brake toe release completed after physical switch release");
		return;
	}

	/*
	 * On a SET edge, wait for the mechanical interlock before beginning and
	 * holding X-Plane's CMD_PB_SET command until lamp acknowledgement.
	 * On a RELEASE edge, pulse the simulator command and release the interlock.
	 */
	if (input.sequence != g_last_parking_brake_switch_sequence) 
	{
		if (input.engaged) 
		{
			g_parking_brake_release_state = TQ_PB_RELEASE_IDLE;
			request_parking_brake_interlock(POKEYS_PARKING_BRAKE_SET);
			if (pokeys_get_parking_brake_state() != POKEYS_PARKING_BRAKE_SET)
				return;
			if (!simulator_indicator_set)
			{
				if (!begin_parking_brake_set_command()) return;
				g_parking_brake_set_awaiting_simulator = 1;
				g_parking_brake_set_started_at = now;
				g_parking_brake_set_timeout_logged = 0;
			}
			g_parking_brake_toe_release_armed = 0;
			log_write("Parking-brake switch set: interlock confirmed; SET command held pending simulator lamp acknowledgement");
		} 
		else 
		{
			end_parking_brake_set_command("hardware parking-brake switch released");
			g_parking_brake_release_state = TQ_PB_RELEASE_IDLE;
			g_parking_brake_set_awaiting_simulator = 0;
			g_parking_brake_set_started_at = 0.0f;
			g_parking_brake_set_timeout_logged = 0;
			g_parking_brake_toe_release_armed = 0;
			if (!ensure_parking_brake_released()) return;
			request_parking_brake_interlock(POKEYS_PARKING_BRAKE_RELEASED);
			log_write("Parking-brake switch released: simulator release verified and interlock release requested");
		}
		g_last_parking_brake_switch_sequence = input.sequence;
		return;
	}

	/*
	 * The pedals held down while setting must be released before a later pedal
	 * press may release the brake.  Update this latch even while simulator
	 * acknowledgement is pending so a failed acknowledgement cannot disable the
	 * physical release path.
	 */
	if (both_pedals_low)
		g_parking_brake_toe_release_armed = 1;

	if (g_parking_brake_toe_release_armed && both_pedals_high &&
		acData.on_ground != 0 &&
		acData.groundspeed_mps < TQ_PARKING_BRAKE_MAX_GROUND_SPEED_MPS)
	{
		end_parking_brake_set_command("parking brake released by toe pedals");
		g_parking_brake_toe_release_armed = 0;
		g_parking_brake_set_awaiting_simulator = 0;
		g_parking_brake_set_started_at = 0.0f;
		g_parking_brake_set_timeout_logged = 0;
		g_parking_brake_release_state = TQ_PB_RELEASE_WAITING_FOR_SWITCH;
		request_parking_brake_interlock(POKEYS_PARKING_BRAKE_RELEASED);
		log_write("Parking brake toe release requested; awaiting physical switch release (left %.3f, right %.3f)",
			acData.left_brake, acData.right_brake);
		return;
	}

	if (g_parking_brake_set_awaiting_simulator) 
	{
		if (!input.engaged)
		{
			end_parking_brake_set_command("hardware parking-brake switch no longer set");
			g_parking_brake_set_awaiting_simulator = 0;
			g_parking_brake_set_started_at = 0.0f;
			g_parking_brake_set_timeout_logged = 0;
		}
		else if (simulator_indicator_set)
		{
			end_parking_brake_set_command("simulator parking-brake lamp acknowledged SET");
			g_parking_brake_set_awaiting_simulator = 0;
			g_parking_brake_set_started_at = 0.0f;
			g_parking_brake_set_timeout_logged = 0;
			first_run_component_complete(TQ_FIRST_RUN_PARKING_BRAKE,
				"parking-brake switch, interlock and simulator indication");
			log_write("X-Plane confirmed parking brake set via simulator indicator");
		}
		else if (!g_parking_brake_set_timeout_logged &&
			(now - g_parking_brake_set_started_at) >=
			TQ_PARKING_BRAKE_CONFIRM_TIMEOUT_SECONDS)
		{
			/*
			 * Do not cancel SET on a timeout: the command must remain held until
			 * the simulator lamp acknowledges it. Log once for diagnostics.
			 */
			g_parking_brake_set_timeout_logged = 1;
			log_write("Parking-brake SET still awaiting simulator lamp after %.1f seconds; command remains held",
				TQ_PARKING_BRAKE_CONFIRM_TIMEOUT_SECONDS);
			return;
		}
		else
		{
			/* Await a later completed dataref read without blocking toe release. */
			return;
		}
	}

	/* A released handle always owns a released interlock. */
	if (!input.engaged)
	{
		g_parking_brake_toe_release_armed = 0;
		request_parking_brake_interlock(POKEYS_PARKING_BRAKE_RELEASED);
		return;
	}

	/* The simulator lamp is the authoritative indication of a successful SET. */
	if (!simulator_indicator_set)
	{
		g_parking_brake_toe_release_armed = 0;
		request_parking_brake_interlock(POKEYS_PARKING_BRAKE_RELEASED);
		return;
	}

	request_parking_brake_interlock(POKEYS_PARKING_BRAKE_SET);
}

/*
 * Physical pins 6/7 are API indices 5/6. PoKeys inversion makes logical true
 * mean that the lever is in CUTOFF; Zibo expects 0.0 for CUTOFF and 1.0 for IDLE.
 */
static void ProcessTqFuelCutoffSwitches(void)
{
	PokeysFuelCutoffInputs inputs;

	pokeys_get_fuel_cutoff_inputs(&inputs);
	if (!inputs.connected || !inputs.valid ||
		inputs.sequence == g_last_fuel_cutoff_sequence)
		return;

	set_float_dataref(DREF_FUEL_CUTOFF_LT,
		inputs.left_cutoff ? 0.0f : 1.0f);
	set_float_dataref(DREF_FUEL_CUTOFF_RT,
		inputs.right_cutoff ? 0.0f : 1.0f);
	g_last_fuel_cutoff_sequence = inputs.sequence;
	first_run_component_complete(TQ_FIRST_RUN_FUEL_CUTOFFS,
		"left and right fuel-cutoff switches");
	log_write("Fuel-cutoff switches applied: left=%s right=%s",
		inputs.left_cutoff ? "CUTOFF" : "IDLE",
		inputs.right_cutoff ? "CUTOFF" : "IDLE");
}

/*
 * Synchronise the two maintained trim cutout switches. The original TQ maps
 * electric trim NORMAL to PoKeys API pin 7 and autopilot trim NORMAL to pin 9.
 * Zibo exposes toggle commands, so each command is issued only while the
 * simulator position differs from the hardware position and is retried at a
 * bounded interval if the simulator has not acknowledged it.
 */
static void ProcessTqTrimCutoutSwitches(void)
{
	PokeysTrimCutoutInputs inputs;
	float electric_target;
	float autopilot_target;
	float now;
	int electric_matches;
	int autopilot_matches;

	pokeys_get_trim_cutout_inputs(&inputs);
	if (!inputs.connected || !inputs.valid)
	{
		g_last_trim_cutout_sequence = 0;
		g_trim_cutout_electric_commanded = 0;
		g_trim_cutout_autopilot_commanded = 0;
		return;
	}

	if (inputs.sequence != g_last_trim_cutout_sequence)
	{
		g_last_trim_cutout_sequence = inputs.sequence;
		g_trim_cutout_electric_commanded = 0;
		g_trim_cutout_autopilot_commanded = 0;
		log_write("Trim cutout hardware changed: electric=%s autopilot=%s",
			inputs.electric_normal ? "NORMAL" : "CUTOUT",
			inputs.autopilot_normal ? "NORMAL" : "CUTOUT");
	}
	now = XPLMGetElapsedTime();

	/* Both guards must be open before their maintained switches can move. */
	if (acData.el_trimlock_pos < 0.5f || acData.ap_trimlock_pos < 0.5f)
	{
		if (acData.el_trimlock_pos < 0.5f &&
			cmdTable[CMD_EL_TRIMLOCK].handle != NULL &&
			(!g_trim_cutout_electric_commanded ||
			 now - g_trim_cutout_electric_command_time >=
				TQ_SWITCH_COMMAND_RETRY_SECONDS))
		{
			XPLMCommandOnce(cmdTable[CMD_EL_TRIMLOCK].handle);
			g_trim_cutout_electric_commanded = 1;
			g_trim_cutout_electric_command_time = now;
		}
		if (acData.ap_trimlock_pos < 0.5f &&
			cmdTable[CMD_AP_TRIMLOCK].handle != NULL &&
			(!g_trim_cutout_autopilot_commanded ||
			 now - g_trim_cutout_autopilot_command_time >=
				TQ_SWITCH_COMMAND_RETRY_SECONDS))
		{
			XPLMCommandOnce(cmdTable[CMD_AP_TRIMLOCK].handle);
			g_trim_cutout_autopilot_commanded = 1;
			g_trim_cutout_autopilot_command_time = now;
		}
		g_trim_cutout_waiting_for_guards = 1;
		return;
	}
	if (g_trim_cutout_waiting_for_guards)
	{
		g_trim_cutout_waiting_for_guards = 0;
		g_trim_cutout_electric_commanded = 0;
		g_trim_cutout_autopilot_commanded = 0;
	}

	electric_target = inputs.electric_normal ? 0.0f : 1.0f;
	autopilot_target = inputs.autopilot_normal ? 0.0f : 1.0f;
	electric_matches = (acData.el_trim_pos >= 0.5f) ==
		(electric_target >= 0.5f);
	autopilot_matches = (acData.ap_trim_pos >= 0.5f) ==
		(autopilot_target >= 0.5f);
	if (!electric_matches && cmdTable[CMD_EL_TRIM].handle != NULL &&
		(!g_trim_cutout_electric_commanded ||
		 now - g_trim_cutout_electric_command_time >=
			TQ_SWITCH_COMMAND_RETRY_SECONDS))
	{
		XPLMCommandOnce(cmdTable[CMD_EL_TRIM].handle);
		g_trim_cutout_electric_commanded = 1;
		g_trim_cutout_electric_command_time = now;
	}
	else if (electric_matches)
	{
		g_trim_cutout_electric_commanded = 0;
	}

	if (!autopilot_matches && cmdTable[CMD_AP_TRIM].handle != NULL &&
		(!g_trim_cutout_autopilot_commanded ||
		 now - g_trim_cutout_autopilot_command_time >=
			TQ_SWITCH_COMMAND_RETRY_SECONDS))
	{
		XPLMCommandOnce(cmdTable[CMD_AP_TRIM].handle);
		g_trim_cutout_autopilot_commanded = 1;
		g_trim_cutout_autopilot_command_time = now;
	}
	else if (autopilot_matches)
	{
		g_trim_cutout_autopilot_commanded = 0;
	}

	if (electric_matches && autopilot_matches)
		first_run_component_complete(TQ_FIRST_RUN_TRIM_CUTOUTS,
			"electric and autopilot trim cutout switches");
}

/* Convert the fixed 400..3695 trim potentiometer travel to X-Plane trim. */
static float trim_position_to_simulator(float position)
{
	float normalised;
	if (position < TQ_TRIM_POSITION_MIN) position = TQ_TRIM_POSITION_MIN;
	if (position > TQ_TRIM_POSITION_MAX) position = TQ_TRIM_POSITION_MAX;
	normalised = (position - TQ_TRIM_POSITION_MIN) /
		(TQ_TRIM_POSITION_MAX - TQ_TRIM_POSITION_MIN);
	return TQ_TRIM_SIM_MIN + normalised *
		(TQ_TRIM_SIM_MAX - TQ_TRIM_SIM_MIN);
}

/* Inverse of the original XPUIPC -16383..12312 to 0..4095 mapping. */
static uint32_t simulator_trim_to_position(float trim)
{
	float normalised;
	if (trim < TQ_TRIM_SIM_MIN) trim = TQ_TRIM_SIM_MIN;
	if (trim > TQ_TRIM_SIM_MAX) trim = TQ_TRIM_SIM_MAX;
	normalised = (trim - TQ_TRIM_SIM_MIN) /
		(TQ_TRIM_SIM_MAX - TQ_TRIM_SIM_MIN);
	return (uint32_t)(TQ_TRIM_POSITION_MIN + normalised *
		(TQ_TRIM_POSITION_MAX - TQ_TRIM_POSITION_MIN) + 0.5f);
}

/*
 * Zibo's Captain and First Officer datarefs are three-position controls: 0.5
 * is released and the two end positions represent the two trim directions.
 * The end-position sign is not duplicated here; while either switch is away
 * from its centre detent, derive direction from the simulator trim change and
 * retain it until release. A command phase, when available, supplies direction
 * without waiting for that first change. Values in the 0.25..0.75 centre
 * deadband unconditionally cancel the manual motor command, preventing normal
 * simulator trim drift from being mistaken for a held yoke switch.
 */
static int manual_trim_direction_from_datarefs(float simulator_delta,
	int* datarefs_available)
{
	int captain_available = drefTable[DREF_TRIM_POS_CA].handle != NULL;
	int first_officer_available = drefTable[DREF_TRIM_POS_FO].handle != NULL;
	int captain_active;
	int first_officer_active;
	int switch_active;

	if (datarefs_available != NULL)
		*datarefs_available = captain_available || first_officer_available;
	captain_active = captain_available &&
		(acData.trim_pos_ca < TQ_ZIBO_TRIM_SWITCH_LOW_MAX ||
			acData.trim_pos_ca > TQ_ZIBO_TRIM_SWITCH_HIGH_MIN);
	first_officer_active = first_officer_available &&
		(acData.trim_pos_fo < TQ_ZIBO_TRIM_SWITCH_LOW_MAX ||
			acData.trim_pos_fo > TQ_ZIBO_TRIM_SWITCH_HIGH_MIN);
	switch_active = captain_active || first_officer_active;
	if (!switch_active) return(0);
	if (g_manual_trim_command_direction != 0)
		return g_manual_trim_command_direction;
	if (simulator_delta > TQ_TRIM_SIM_CHANGE_EPSILON) return(1);
	if (simulator_delta < -TQ_TRIM_SIM_CHANGE_EPSILON) return -1;
	return g_last_trim_dataref_direction;
}

/*
 * Port of the original application's two distinct trim modes.
 *
 * With A/P engaged the simulator owns position: its trim value is mapped to a
 * motor target and the original 50-count closed-loop governor follows it. With
 * A/P disengaged, the original does not chase small simulator-value changes;
 * the active yoke trim command drives the motor directly and physical wheel
 * movement is written back to X-Plane. This separation is essential because
 * treating normal simulator jitter as a motor target causes continuous high-
 * power reversals. First Run remains the deliberate simulator-owned exception.
 */
static void ProcessTqTrim(void)
{
	PokeysLeverPositions positions;
	float physical_position;
	float simulator_before;
	float physical_trim;
	float physical_delta;
	int autopilot_engaged;
	int motor_allowed;
	int motor_running;
	int manual_direction;
	int trim_datarefs_available;

	if (!g_control_calibration_valid)
	{
		pokeys_set_trim_target(0U, 0);
		pokeys_set_trim_manual_command(0, 0);
		pokeys_set_trim_indicator_target(0U, 0);
		g_trim_input_initialised = 0;
		g_trim_first_run_active = 1;
		return;
	}

	pokeys_get_lever_positions(&positions);
	if (!positions.connected || !positions.valid)
	{
		pokeys_set_trim_target(0U, 0);
		pokeys_set_trim_manual_command(0, 0);
		pokeys_set_trim_indicator_target(0U, 0);
		g_trim_input_initialised = 0;
		g_trim_first_run_active = 1;
		return;
	}

	physical_position = positions.sequence != g_last_trim_sequence ?
		moving_average_add(&g_trim_filter,
			(float)positions.value[POKEYS_LEVER_TRIM]) :
		g_last_trim_position;
	simulator_before = acData.elevator_trim;
	/* Treat the animated/numeric Zibo value as engaged only at its ON detent. */
	autopilot_engaged = acData.ap_engaged >= 0.5;
	motor_running = pokeys_trim_motor_is_running();
	manual_direction = g_manual_trim_command_direction;
	if (manual_direction == 0)
		manual_direction = manual_trim_direction_from_datarefs(
			simulator_before - g_last_simulator_trim,
			&trim_datarefs_available);
	else
		trim_datarefs_available =
			drefTable[DREF_TRIM_POS_CA].handle != NULL ||
			drefTable[DREF_TRIM_POS_FO].handle != NULL;
	if (!g_trim_dataref_input_initialised ||
		manual_direction != g_last_trim_dataref_direction)
	{
		log_write("Manual trim input: Captain=%.1f First Officer=%.1f, request=%s, source=%s",
			acData.trim_pos_ca, acData.trim_pos_fo,
			manual_direction < 0 ? "nose down" :
			(manual_direction > 0 ? "nose up" : "released"),
			g_manual_trim_command_direction != 0 ? "XPLM electrical/trim command" :
			(trim_datarefs_available ? "Zibo datarefs" : "no active input"));
		g_last_trim_dataref_direction = manual_direction;
		g_trim_dataref_input_initialised = 1;
	}
	physical_trim = trim_position_to_simulator(physical_position);

	if (!g_trim_input_initialised)
	{
		g_trim_input_initialised = 1;
		g_last_trim_sequence = positions.sequence;
		g_last_trim_position = physical_position;
		g_last_simulator_trim = simulator_before;
		g_trim_first_run_active = 1;
	}

	/*
	 * Trim is the deliberate First Run exception: the simulator is
	 * authoritative. Drive the wheel to the simulator target and command the
	 * indicator directly from that target while the mechanical wheel catches
	 * up. Never write physical trim back to X-Plane during this phase.
	 */
	if (g_trim_first_run_active)
	{
		uint32_t target_position = simulator_trim_to_position(simulator_before);
		float distance = physical_position - (float)target_position;
		if (distance < 0.0f) distance = -distance;
		motor_allowed = acData.paused == 0 && acData.battery_on != 0.0 &&
			acData.ap_trim_pos < 0.5f && acData.el_trim_pos < 0.5f;
		pokeys_set_trim_manual_command(0, 0);
		pokeys_set_trim_indicator_target(target_position, 1);
		pokeys_set_trim_target(target_position, motor_allowed);
		g_last_trim_sequence = positions.sequence;
		g_last_trim_position = physical_position;
		g_last_simulator_trim = simulator_before;
		if (distance <= TQ_TRIM_SYNC_TOLERANCE_COUNTS)
		{
			g_trim_first_run_active = 0;
			pokeys_set_trim_indicator_target(target_position, 0);
			first_run_component_complete(TQ_FIRST_RUN_TRIM_POSITION,
				"trim wheel and trim indicator from simulator position");
		}
		return;
	}

	physical_delta = physical_position - g_last_trim_position;
	if (physical_delta < 0.0f) physical_delta = -physical_delta;

	/*
	 * The original gives the A/P closed-loop path ownership whenever the
	 * autopilot is engaged. A manual trim command is still delivered to
	 * X-Plane by its normal command handler; any resulting trim target change
	 * is therefore followed here without fighting the simulator or A/P.
	 */
	if (autopilot_engaged)
	{
		uint32_t target_position = simulator_trim_to_position(simulator_before);
		/* A/P trim cutout alone inhibits the original automatic trim path. */
		motor_allowed = acData.paused == 0 && acData.ap_trim_pos < 0.5f;
		pokeys_set_trim_manual_command(0, 0);
		pokeys_set_trim_target(target_position, motor_allowed);
		g_last_trim_sequence = positions.sequence;
		g_last_trim_position = physical_position;
		g_last_simulator_trim = simulator_before;
		return;
	}

	/*
	 * With A/P disengaged, the original 0/1/2 yoke input directly drives the
	 * trim wheel. Zibo's Captain/First Officer switch-position datarefs provide
	 * the equivalent -1/0/+1 state; command phases are a legacy fallback.
	 */
	if (manual_direction != 0)
	{
		motor_allowed = acData.paused == 0 && acData.battery_on != 0.0 && acData.el_trim_pos < 0.5f;
		pokeys_set_trim_target(0U, 0);
		pokeys_set_trim_manual_command(manual_direction, motor_allowed);
		g_last_trim_sequence = positions.sequence;
		g_last_trim_position = physical_position;
		g_last_simulator_trim = simulator_before;
		return;
	}

	/* Idle manual mode brakes the motor, matching the original switch branch. */
	motor_allowed = acData.paused == 0 && acData.battery_on != 0.0 && acData.el_trim_pos < 0.5f;
	pokeys_set_trim_target(0U, 0);
	pokeys_set_trim_manual_command(0, motor_allowed);

	/* Manual wheel movement remains authoritative while A/P is disengaged. */
	if (!motor_running && physical_delta >= TQ_TRIM_PHYSICAL_CHANGE_COUNTS)
	{
		if (physical_trim != simulator_before)
			set_float_dataref(DREF_ELEVATOR_TRIM, physical_trim);
		simulator_before = physical_trim;
	}

	g_last_trim_sequence = positions.sequence;
	g_last_trim_position = physical_position;
	g_last_simulator_trim = simulator_before;
}

/* Compare a float-array FMA value with one of Zibo's integer mode codes. */
static int TqPfdSpeedModeEquals(float value, int mode)
{
	return value > (float)mode - 0.5f && value < (float)mode + 0.5f;
}

/* Return non-zero when either FMC announces Zibo mode 5: approach GA. */
static int TqGaModeActive(void)
{
	return TqPfdSpeedModeEquals(acData.pfd_speed_mode_ca, 5) ||	TqPfdSpeedModeEquals(acData.pfd_speed_mode_fo, 5);
}

/*
 * Modes that physically command throttle travel. ARM (1) and THR HLD (6)
 * deliberately return false so the pilot retains direct lever authority.
 */
static int TqPfdSpeedModeDrivesThrottle(float mode)
{
	return TqPfdSpeedModeEquals(mode, 2) ||  /* N1 */
		TqPfdSpeedModeEquals(mode, 3) ||      /* MCP SPD */
		TqPfdSpeedModeEquals(mode, 4) ||      /* FMC SPD */
		TqPfdSpeedModeEquals(mode, 5) ||      /* GA */
		TqPfdSpeedModeEquals(mode, 7);        /* RETARD */
}

static int TqAutothrottleMotorModeActive(void)
{
	return TqPfdSpeedModeDrivesThrottle(acData.pfd_speed_mode_ca) || TqPfdSpeedModeDrivesThrottle(acData.pfd_speed_mode_fo);
}

/*
 * Maintain the TO/GA session independently of the momentary pushbuttons.
 * Simulator mode feedback also establishes the session after a plugin reload.
 * A/T ARM off ends the session and acknowledges a completed manual disconnect.
 */
static void UpdateTqAutothrottleSession(void)
{
	if (acData.at_arm <= 0.0f)
	{
		g_toga_selected_since_at_arm = 0;
		g_manual_throttle_disconnect_pending = 0;
		g_unowned_throttle_tracking = 0;
	}
	else if (TqGaModeActive())
	{
		g_toga_selected_since_at_arm = 1;
	}
}

/*
 * A/T ARM alone permits manual throttle positioning while on the ground.
 * After TO/GA, or whenever airborne, physical movement instead requests an
 * A/T disconnect.  A pending disconnect suppresses further motor commands.
 */
static int TqManualThrottleDisconnectRequired(void)
{
	return acData.at_arm > 0.0f && TqAutothrottleMotorModeActive() &&
		(g_toga_selected_since_at_arm || acData.on_ground == 0) &&
		!g_manual_throttle_disconnect_pending;
}

/*
 * Return non-zero while the simulator owns the physical throttle levers.
 * The original application required A/T ARM and an active thrust mode. Ground
 * operation remains manual until TO/GA has been selected. The PFD speed-mode
 * values are authoritative so ARM and THR HLD cannot accidentally energise the
 * motors merely because a separate A/T status dataref remains asserted.
 */
static int TqAutothrottleOwnsLevers(void)
{
	if (g_manual_throttle_disconnect_pending || acData.at_arm <= 0.0f)
		return(0);
	if (acData.on_ground != 0 && !g_toga_selected_since_at_arm)
		return(0);
	return TqAutothrottleMotorModeActive();
}

/*
 * Emulate one press of the appropriate physical A/T-disconnect button. If both
 * levers caused the request, issue only the Captain command so a second command
 * cannot acknowledge/clear the first disconnect annunciation.
 */
static void RequestTqManualThrottleDisconnect(int lever_mask)
{
	int command_index;

	if (lever_mask == 0 || !TqManualThrottleDisconnectRequired()) return;
	command_index = (lever_mask & 1) ? CMD_LT_AT_DISCO : CMD_RT_AT_DISCO;
	if (cmdTable[command_index].handle == NULL)
	{
		log_write("Unable to disconnect A/T after manual throttle movement: command handle is unavailable");
		return;
	}

	g_manual_throttle_disconnect_pending = 1;
	g_unowned_throttle_tracking = 0;
	pokeys_set_throttle_follow_targets(0U, 0U, 0U, 0U, 0);
	XPLMCommandOnce(cmdTable[command_index].handle);
	log_write("Manual %s throttle movement detected after %s; A/T disconnect command issued",
		lever_mask == 3 ? "left/right" : (lever_mask & 1 ? "left" : "right"),
		acData.on_ground == 0 ? "lift-off" : "TO/GA selection");
}

/*
 * A/T ownership inhibits throttle writes but not filtering. Speedbrake changes are
 * always written, including motor-originated motion. Its DOWN and ARM detents
 * are resolved before the continuous flight range so ARM cannot be mistaken
 * for partial extension. The flap lever is resolved to one of the original
 * application's nine discrete detents.
 */
static void ProcessTqLeverWrites(void)
{
	static const float flaps_value[9] = {
		0.000f, 0.125f, 0.250f, 0.375f, 0.500f,
		0.625f, 0.750f, 0.875f, 1.000f
	};
	static const char* flaps_name[9] = {
		"UP", "1", "2", "5", "10", "15", "25", "30", "40"
	};
	PokeysLeverPositions positions;
	float throttle_left;
	float throttle_right;
	float speedbrake;
	float speedbrake_output;
	float reverser_left;
	float reverser_right;
	float flaps_position;
	float throttle_left_output;
	float throttle_right_output;
	uint32_t flaps_corrected;
	uint32_t speedbrake_corrected;
	int flaps_detent;
	int speedbrake_state;
	int manual_override_mask;

	if (!g_control_calibration_valid) return;

	pokeys_get_lever_positions(&positions);
	
	if (!positions.connected || !positions.valid ||	positions.sequence == g_last_lever_sequence)
		return;
	g_last_lever_sequence = positions.sequence;

	throttle_left = map_calibrated_axis(moving_average_add(&g_throttle_left_filter, (float)positions.value[POKEYS_LEVER_THROTTLE_1]), g_control_calibration.lever1_min_position, g_control_calibration.lever1_max_position);

	throttle_right = map_calibrated_axis(moving_average_add(&g_throttle_right_filter, (float)positions.value[POKEYS_LEVER_THROTTLE_2]),	g_control_calibration.lever2_min_position, g_control_calibration.lever2_max_position);
	g_physical_throttle_left = throttle_left;
	g_physical_throttle_right = throttle_right;
	g_physical_throttle_positions_valid = 1;

	/*
	 * The worker detects a pilot opposing an energised throttle motor. When no
	 * A/T thrust mode currently owns the motors, retain a 65-count reference and
	 * detect physical movement here before that movement is written to X-Plane.
	 * The threshold is the original application's throttle user-input threshold.
	 */
	manual_override_mask = pokeys_take_throttle_manual_override();
	RequestTqManualThrottleDisconnect(manual_override_mask);
	if (TqManualThrottleDisconnectRequired() && !TqAutothrottleOwnsLevers())
	{
		if (!g_unowned_throttle_tracking)
		{
			g_unowned_throttle_left_reference = throttle_left;
			g_unowned_throttle_right_reference = throttle_right;
			g_unowned_throttle_tracking = 1;
		}
		else
		{
			manual_override_mask = 0;
			if (absolute_difference(throttle_left,
				g_unowned_throttle_left_reference) >=
				TQ_THROTTLE_MANUAL_MOVEMENT_THRESHOLD)
				manual_override_mask |= 1;
			if (absolute_difference(throttle_right,
				g_unowned_throttle_right_reference) >=
				TQ_THROTTLE_MANUAL_MOVEMENT_THRESHOLD)
				manual_override_mask |= 2;
			RequestTqManualThrottleDisconnect(manual_override_mask);
		}
	}
	else
	{
		g_unowned_throttle_tracking = 0;
	}
	
	speedbrake = map_calibrated_axis(moving_average_add(&g_speedbrake_filter, (float)positions.value[POKEYS_LEVER_SPEED_BRAKE]), g_control_calibration.spoiler_min_position, g_control_calibration.spoiler_max_position);
	speedbrake_corrected = (uint32_t)(speedbrake * 4095.0f + 0.5f);
	if (speedbrake_corrected <= TQ_SPEEDBRAKE_DOWN_MAX_COUNTS)
	{
		speedbrake_state = TQ_SPEEDBRAKE_DOWN;
		speedbrake_output = 0.0f;
	}
	else if (speedbrake_corrected <= TQ_SPEEDBRAKE_ARM_MAX_COUNTS)
	{
		speedbrake_state = TQ_SPEEDBRAKE_ARMED;
		speedbrake_output = TQ_SPEEDBRAKE_ARM_VALUE;
	}
	else if (speedbrake_corrected < TQ_SPEEDBRAKE_UP_MIN_COUNTS)
	{
		float travel = (float)(speedbrake_corrected -
			TQ_SPEEDBRAKE_ARM_MAX_COUNTS) /
			(float)(TQ_SPEEDBRAKE_UP_MIN_COUNTS -
				TQ_SPEEDBRAKE_ARM_MAX_COUNTS);
		speedbrake_state = TQ_SPEEDBRAKE_FLIGHT;
		speedbrake_output = TQ_SPEEDBRAKE_ARM_VALUE + travel *
			(TQ_SPEEDBRAKE_FLIGHT_VALUE - TQ_SPEEDBRAKE_ARM_VALUE);
	}
	else
	{
		float travel = (float)(speedbrake_corrected -
			TQ_SPEEDBRAKE_UP_MIN_COUNTS) /
			(float)(4095U - TQ_SPEEDBRAKE_UP_MIN_COUNTS);
		speedbrake_state = TQ_SPEEDBRAKE_UP;
		speedbrake_output = TQ_SPEEDBRAKE_FLIGHT_VALUE + travel *
			(1.0f - TQ_SPEEDBRAKE_FLIGHT_VALUE);
	}
	reverser_left = map_calibrated_axis(moving_average_add(&g_reverser_left_filter,
		(float)positions.value[POKEYS_LEVER_REVERSER_1]),
		g_control_calibration.reverser1_min_position,
		g_control_calibration.reverser1_max_position);
	reverser_right = map_calibrated_axis(moving_average_add(&g_reverser_right_filter,
		(float)positions.value[POKEYS_LEVER_REVERSER_2]),
		g_control_calibration.reverser2_min_position,
		g_control_calibration.reverser2_max_position);
	flaps_position = flaps_filter_add(&g_flaps_filter,
		(float)positions.value[POKEYS_LEVER_FLAPS]);
	flaps_corrected = flaps_corrected_position(flaps_position);
	flaps_detent = flaps_position_to_detent(flaps_corrected);

	/*
	 * The original application arms reverse only on the ground with the related
	 * forward lever at idle. A 250/150-count hysteresis prevents chatter around
	 * the stowed position. The combined per-engine throttle/reverser datarefs
	 * use 0..1 for forward thrust and 0..-2 for reverse thrust.
	 */
	if (!g_reverser_left_active && acData.on_ground != 0 &&
		throttle_left <= TQ_THROTTLE_IDLE_THRESHOLD &&
		reverser_left > TQ_REVERSER_ACTIVATE_THRESHOLD)
		g_reverser_left_active = 1;
	else if (g_reverser_left_active && reverser_left < TQ_REVERSER_RELEASE_THRESHOLD)
		g_reverser_left_active = 0;

	if (!g_reverser_right_active && acData.on_ground != 0 &&
		throttle_right <= TQ_THROTTLE_IDLE_THRESHOLD &&
		reverser_right > TQ_REVERSER_ACTIVATE_THRESHOLD)
		g_reverser_right_active = 1;
	else if (g_reverser_right_active && reverser_right < TQ_REVERSER_RELEASE_THRESHOLD)
		g_reverser_right_active = 0;

	throttle_left_output = g_reverser_left_active ?
		TQ_REVERSER_FULL_SCALE * reverser_left : throttle_left;
	throttle_right_output = g_reverser_right_active ?
		TQ_REVERSER_FULL_SCALE * reverser_right : throttle_right;

	/*
	 * A/T ARM by itself must not suppress manual thrust. Simulator ownership
	 * begins only in N1, MCP SPD, FMC SPD, GA or RETARD. ARM and THR HLD retain
	 * manual authority. Compare against both the last hardware write and the
	 * freshly read simulator value: A/T may have moved the simulator throttle
	 * while the unchanged physical lever still equals its stale write cache.
	 */
	if (!TqAutothrottleOwnsLevers())
	{
		if (!g_throttle_left_written ||
			throttle_left_output != g_last_throttle_left_written ||
			absolute_difference(throttle_left_output,
				acData.throttle_ratio_left) > TQ_THROTTLE_DATAREF_EPSILON)
		{
			set_float_dataref(DREF_THROTTLE_RATIO_LT, throttle_left_output);
			g_last_throttle_left_written = throttle_left_output;
			g_throttle_left_written = 1;
		}
		if (!g_throttle_right_written ||
			throttle_right_output != g_last_throttle_right_written ||
			absolute_difference(throttle_right_output,
				acData.throttle_ratio_right) > TQ_THROTTLE_DATAREF_EPSILON)
		{
			set_float_dataref(DREF_THROTTLE_RATIO_RT, throttle_right_output);
			g_last_throttle_right_written = throttle_right_output;
			g_throttle_right_written = 1;
		}
	}

	if (!g_speedbrake_written || speedbrake_output != g_last_speedbrake_written)
	{
		set_float_dataref(DREF_SPD_BRAKE_LEVER, speedbrake_output);
		g_last_speedbrake_written = speedbrake_output;
		g_speedbrake_written = 1;
	}
	if (speedbrake_state != g_last_speedbrake_state)
	{
		static const char* state_name[] = { "DOWN", "ARMED", "FLIGHT", "UP" };
		log_write("Physical speedbrake selected: %s (ADC %u, simulator %.3f)",
			state_name[speedbrake_state], speedbrake_corrected,
			speedbrake_output);
		g_last_speedbrake_state = speedbrake_state;
	}

	/*
	 * Port the original landing automation. An armed lever is driven to UP on
	 * the airborne-to-ground transition. Once five seconds have elapsed after
	 * touchdown, advancing either forward throttle with both reversers stowed
	 * drives the lever back DOWN. Motor movement is fed back through the ADC and
	 * therefore updates X-Plane through the normal change-driven write above.
	 */
	if (!g_speedbrake_ground_state_initialised)
	{
		g_speedbrake_was_on_ground = acData.on_ground != 0;
		g_speedbrake_ground_state_initialised = 1;
	}
	else if (!g_speedbrake_was_on_ground && acData.on_ground != 0)
	{
		g_speedbrake_touchdown_time = XPLMGetElapsedTime();
		g_speedbrake_auto_extend_issued = 0;
		g_speedbrake_auto_retract_issued = 0;
		if (speedbrake_state == TQ_SPEEDBRAKE_ARMED &&
			pokeys_speedbrake_push_up_and_extend())
		{
			g_speedbrake_auto_extend_issued = 1;
			log_write("Touchdown with speedbrake ARMED: automatic full extension requested");
		}
	}
	else if (g_speedbrake_was_on_ground && acData.on_ground == 0)
	{
		g_speedbrake_auto_extend_issued = 0;
		g_speedbrake_auto_retract_issued = 0;
		g_speedbrake_touchdown_time = 0.0f;
	}
	g_speedbrake_was_on_ground = acData.on_ground != 0;

	if (acData.on_ground != 0 && !g_speedbrake_auto_retract_issued &&
		g_speedbrake_touchdown_time > 0.0f &&
		XPLMGetElapsedTime() - g_speedbrake_touchdown_time >=
			TQ_SPEEDBRAKE_AUTO_RETRACT_DELAY_SECONDS &&
		speedbrake_state == TQ_SPEEDBRAKE_UP &&
		(throttle_left > TQ_THROTTLE_IDLE_THRESHOLD ||
			throttle_right > TQ_THROTTLE_IDLE_THRESHOLD) &&
		!g_reverser_left_active && !g_reverser_right_active &&
		pokeys_speedbrake_retract_and_pull_down())
	{
		g_speedbrake_auto_retract_issued = 1;
		log_write("Forward throttle advanced after landing: automatic speedbrake retraction requested");
	}

	/*
	 * Establish the hardware flap detent as the First Run authority, even when
	 * the lever has not moved since connection. Afterwards, only a resolved
	 * detent change is considered; ADC noise within a detent causes no writes.
	 */
	if ((!g_flaps_written || flaps_detent != g_last_flaps_detent) &&
		drefTable[DREF_FLAPS_LEVER].handle != NULL &&
		drefTable[DREF_FLAPS_LEVER].isWriteable)
	{
		int first_sync = !g_flaps_written;
		float desired = flaps_value[flaps_detent];
		if (absolute_difference(acData.flaps_lever, desired) >
			TQ_FLAPS_DATAREF_EPSILON)
			set_float_dataref(DREF_FLAPS_LEVER, desired);
		g_last_flaps_detent = flaps_detent;
		g_flaps_written = 1;
		if (first_sync)
		{
			first_run_component_complete(TQ_FIRST_RUN_FLAPS,
				"flap lever from hardware position");
			log_write("Flap lever First Run synchronised from hardware: %s (ADC %u)",
				flaps_name[flaps_detent], flaps_corrected);
		}
		else
		{
			log_write("Flap lever selected: %s (ADC %u)",
				flaps_name[flaps_detent], flaps_corrected);
		}
	}
}

/*
 * Convert the simulator's current 0..1 forward-thrust ratios to the common
 * 0..4095 corrected-position domain used by the original .NET governor. The
 * PoKeys worker converts each filtered ADC reading from its individual
 * calibration span into the same domain before calculating error. This runs
 * every FLCB pass (not only on an ADC change), so a TO/GA/FMC target reaches
 * both throttle governors as one coherent pair immediately.
 */
static void ProcessTqThrottleMotorFollow(void)
{
	float left;
	float right;
	uint32_t left_target;
	uint32_t right_target;
	int enabled;

	if (!g_control_calibration_valid)
	{
		pokeys_set_throttle_follow_targets(0U, 0U, 0U, 0U, 0);
		return;
	}
	/*
	 * Simulator mode feedback, rather than a local button latch, owns the
	 * motors. This allows approach TO/GA to take control when either FMC reports
	 * GA (mode 5), and releases control in ARM (1) or THR HLD (6). Pausing
	 * X-Plane always coasts the motors.
	 */
	enabled = TqAutothrottleOwnsLevers() && acData.paused == 0;
	left = clamp_unit(acData.throttle_ratio_left);
	right = clamp_unit(acData.throttle_ratio_right);
	left_target = (uint32_t)(left * 4095.0f + 0.5f);
	right_target = (uint32_t)(right * 4095.0f + 0.5f);
	pokeys_set_throttle_follow_targets(left_target, right_target,
		g_control_calibration.lever1_min_speed,
		g_control_calibration.lever2_min_speed, enabled);
}

/**********************************************************************************/
/* PUSHBUTTON SWITCH HANDLERS                                                     */
/**********************************************************************************/

/*
 * Transfer the PoKeys worker's coherent button snapshot into the state table on
 * X-Plane's flight-loop thread.  The handlers issue XPLM commands and therefore
 * must never be called directly by the PoKeys connection thread.
 */
static void ProcessTqTogaButtons(state_table_p state)
{
	PokeysTogaInputs inputs;

	if (state == NULL) return;
	pokeys_get_toga_inputs(&inputs);
	if (inputs.connected && inputs.valid && acData.at_arm > 0.0f &&
		((inputs.left_pressed && state->lt_toga_prev != 1) ||
			(inputs.right_pressed && state->rt_toga_prev != 1)))
	{
		g_toga_selected_since_at_arm = 1;
		g_manual_throttle_disconnect_pending = 0;
		g_unowned_throttle_tracking = 0;
		log_write("TO/GA selected with A/T armed: manual throttle movement will disconnect A/T");
	}

	/* Invalid/disconnected input releases any command that was active. */
	state->lt_toga = (int)((inputs.connected && inputs.valid) ?
		inputs.left_pressed : 0);
	left_toga_handler((void*)state);

	state->rt_toga = (int)((inputs.connected && inputs.valid) ?
		inputs.right_pressed : 0);
	right_toga_handler((void*)state);
}

/* Publish the physical A/T disconnect buttons to the existing command handlers. */
static void ProcessTqAtDisconnectButtons(state_table_p state)
{
	PokeysAtDisconnectInputs inputs;

	if (state == NULL) return;
	pokeys_get_at_disconnect_inputs(&inputs);

	/* Invalid/disconnected input releases any command that was active. */
	state->lt_at_disco = (int)((inputs.connected && inputs.valid) ?
		inputs.left_pressed : 0);
	left_at_disco_handler((void*)state);

	state->rt_at_disco = (int)((inputs.connected && inputs.valid) ?
		inputs.right_pressed : 0);
	right_at_disco_handler((void*)state);
}

/* process left (captain) TO/GA switch */
void left_toga_handler(void* param)
{
	/* cast the opaque pointer to the state table */
	state_table_p ptr = (state_table_p)param;

	/* check for change */
	if (ptr->lt_toga_prev != ptr->lt_toga)
	{
		if (ptr->lt_toga == 1 && !ptr->ltTogaIsActive)
		{
			ptr->lt_toga_prev = ptr->lt_toga;
			ptr->ltTogaIsActive = true;
			XPLMCommandBegin(cmdTable[CMD_LT_TOGA].handle);
		}
		else if (ptr->lt_toga == 0 && ptr->ltTogaIsActive)
		{
			ptr->lt_toga_prev = ptr->lt_toga;
			ptr->ltTogaIsActive = false;
			XPLMCommandEnd(cmdTable[CMD_LT_TOGA].handle);
		}
	}
}

/* process left (f/o) TO/GA switch */
void right_toga_handler(void* param)
{
	/* cast the opaque pointer to the state table */
	state_table_p ptr = (state_table_p)param;

	/* check for change */
	if (ptr->rt_toga_prev != ptr->rt_toga)
	{
		if (ptr->rt_toga == 1 && !ptr->rtTogaIsActive)
		{
			ptr->rt_toga_prev = ptr->rt_toga;
			ptr->rtTogaIsActive = true;
			XPLMCommandBegin(cmdTable[CMD_RT_TOGA].handle);
		}
		else if (ptr->rt_toga == 0 && ptr->rtTogaIsActive)
		{
			ptr->rt_toga_prev = ptr->rt_toga;
			ptr->rtTogaIsActive = false;
			XPLMCommandEnd(cmdTable[CMD_RT_TOGA].handle);
		}
	}
}

/* process left (captain) A/T switch */
void left_at_disco_handler(void* param)
{
	/* cast the opaque pointer to the state table */
	state_table_p ptr = (state_table_p)param;

	/* check for change */
	if (ptr->lt_at_disco_prev != ptr->lt_at_disco)
	{
		if (ptr->lt_at_disco == 1 && !ptr->ltAtDiscoIsActive)
		{
			ptr->lt_at_disco_prev = ptr->lt_at_disco;
			ptr->ltAtDiscoIsActive = true;
			XPLMCommandBegin(cmdTable[CMD_LT_AT_DISCO].handle);
		}
		else if (ptr->lt_at_disco == 0 && ptr->ltAtDiscoIsActive)
		{
			ptr->lt_at_disco_prev = ptr->lt_at_disco;
			ptr->ltAtDiscoIsActive = false;
			XPLMCommandEnd(cmdTable[CMD_LT_AT_DISCO].handle);
		}
	}
}

/* process right (f/o) A/T switch */
void right_at_disco_handler(void* param)
{
	/* cast the opaque pointer to the state table */
	state_table_p ptr = (state_table_p)param;

	/* check for change */
	if (ptr->rt_at_disco_prev != ptr->rt_at_disco)
	{
		if (ptr->rt_at_disco == 1 && !ptr->rtAtDiscoIsActive)
		{
			ptr->rt_at_disco_prev = ptr->rt_at_disco;
			ptr->rtAtDiscoIsActive = true;
			XPLMCommandBegin(cmdTable[CMD_RT_AT_DISCO].handle);
		}
		else if (ptr->rt_at_disco == 0 && ptr->rtAtDiscoIsActive)
		{
			ptr->rt_at_disco_prev = ptr->rt_at_disco;
			ptr->rtAtDiscoIsActive = false;
			XPLMCommandEnd(cmdTable[CMD_RT_AT_DISCO].handle);
		}
	}
}

/**********************************************************************************/
/* DATAREF HANDLERS, COMMAND HANDLERS & FLIGHTLOOP CALLBACK FROM HERE ON          */
/**********************************************************************************/
/* load the datref table */
struct DREF_TABLE drefTable[DREF_END] = 
{
	/* CFY TQ specific datarefs */
	{.datarefName = "laminar/B738/electric/battery_pos", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = true, .isEmittable = true, .ptrVal = (void*)&acData.battery_on},										// 0	DREF_BATTERY_ON                  FSUIPC offset 0x3102
	{.datarefName = "sim/time/paused", .handle = NULL, .dataType = XP_INT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.paused},															// 1	DREF_SIM_PAUSED                  FSUIPC offset 0x0264
	{.datarefName = "sim/flightmodel2/gear/on_ground", .handle = NULL, .dataType = XP_INT, .isArray = true, .arrayOffset = 2, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.on_ground},											// 2	DREF_ON_GROUND                   FSUIPC offset 0x0366
	{.datarefName = "sim/flightmodel2/position/groundspeed", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.groundspeed_mps},								// 3	DREF_GND_SPEED                   FSUIPC offset 0x02B4
	{.datarefName = "sim/flightmodel2/position/y_agl", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.radio_altitude_m},									// 4	DREF_RADIO_ALT                   FSUIPC offset 0x31E4
	{.datarefName = "laminar/B738/parking_brake_pos", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.parking_brake},										// 5	DREF_PARKING_BRAKE               FSUIPC offset 0x0BC8 (read only; release uses CMD_PB_SET)
	{.datarefName = "sim/cockpit2/controls/left_brake_ratio", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.left_brake},									// 6	DREF_LEFT_BRAKE                  FSUIPC offset 0x0BC4
	{.datarefName = "sim/cockpit2/controls/right_brake_ratio", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.right_brake},								// 7	DREF_RIGHT_BRAKE                 FSUIPC offset 0x0BC6
	{.datarefName = "sim/cockpit2/engine/actuators/throttle_jet_rev_ratio", .handle = NULL, .dataType = XP_FLT, .isArray = true, .arrayOffset = 0, .arrayCount = 1, .isWriteable = true, .isEmittable = true, .ptrVal = (void*)&acData.throttle_ratio_left},			// 8	DREF_THROTTLE_RATIO_LT           FSUIPC offset 0x088C
	{.datarefName = "sim/cockpit2/engine/actuators/throttle_jet_rev_ratio", .handle = NULL, .dataType = XP_FLT, .isArray = true, .arrayOffset = 1, .arrayCount = 1, .isWriteable = true, .isEmittable = true, .ptrVal = (void*)&acData.throttle_ratio_right},			// 9	DREF_THROTTLE_RATIO_RT           FSUIPC offset 0x0924
	{.datarefName = "sim/flightmodel/controls/elv_trim", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = true, .isEmittable = true, .ptrVal = (void*)&acData.elevator_trim},									// 10	DREF_ELEVATOR_TRIM               FSUIPC offset 0x0BC2
	{.datarefName = "laminar/B738/flt_ctrls/speedbrake_arm", .handle = NULL, .dataType = XP_DBL, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = true, .isEmittable = true, .ptrVal = (void*)&acData.speedbrake_armed},								// 11	DREF_SPD_BRAKE_ARM               FSUIPC offset 0x0BCC
	{.datarefName = "laminar/B738/flt_ctrls/speedbrake_lever", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = true, .isEmittable = true, .ptrVal = (void*)&acData.speedbrake_lever},							// 12	DREF_SPD_BRAKE_LEVER             FSUIPC offset 0x0BDO
	{.datarefName = "laminar/B738/autopilot/autothrottle_arm_pos", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.at_arm},								// 13	DREF_AUTO_THROTTLE_ARM           FSUIPC offset 0x0810
	{.datarefName = "laminar/B738/autopilot/autothrottle_status1", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.at_active},								// 14	DREF_AUTO_THROTTLE_ACT           /* no known FSUIPC offset */
	{.datarefName = "laminar/autopilot/ap_on", .handle = NULL, .dataType = XP_DBL, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.ap_engaged},												// 15	DREF_AP_ENGAGED                  FSUIPC offset 0x07BC
	{.datarefName = "laminar/B738/annunciator/parking_brake", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.pb_ind_raw},									// 16	DREF_PB_IND_RAW                  FSUIPC offset 0x07BC
	{.datarefName = "laminar/B738/toggle_switch/ap_trim_lock_pos", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.ap_trimlock_pos},						// 17	DREF_AP_TRIMLOCK_POS             /* no known FSUIPC offset */ 0=closed, 1=open 
	{.datarefName = "laminar/B738/toggle_switch/ap_trim_pos", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.ap_trim_pos},								// 18	DREF_AP_TRIM_POS                 /* no known FSUIPC offset */ 0=normal, 1=cut out
	{.datarefName = "laminar/B738/toggle_switch/el_trim_lock_pos", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.el_trimlock_pos},						// 19	DREF_EL_TRIMLOCK_POS             /* no known FSUIPC offset */ 0=closed, 1=open 
	{.datarefName = "laminar/B738/toggle_switch/el_trim_pos", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.el_trim_pos},								// 20	DREF_EL_TRIM_POS                 /* no known FSUIPC offset */ 0=normal, 1=cut out
	{.datarefName = "laminar/B738/engine/mixture_ratio1", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = true, .isEmittable = true, .ptrVal = (void*)&acData.fuel_cutoff_lt},									// 21	DREF_FUEL_CUTOFF_LT              /* no known FSUIPC offset */ 0=cutoff, 1=idle
	{.datarefName = "laminar/B738/engine/mixture_ratio2", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = true, .isEmittable = true, .ptrVal = (void*)&acData.fuel_cutoff_rt},									// 22	DREF_FUEL_CUTOFF_RT              /* no known FSUIPC offset */ 0=cutoff, 1=idle
	{.datarefName = "laminar/B738/flt_ctrls/flap_lever", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = true, .isEmittable = true, .ptrVal = (void*)&acData.flaps_lever},										// 23	DREF_FLAPS_LEVER                 FSUIPC offset 0x0BDC
	{.datarefName = "laminar/B738/switch/capt_trim_pos", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.trim_pos_ca},										// 24	DREF_TRIM_POS_CA                 Captain yoke trim switch: 0/1=direction, 0.5=released
	{.datarefName = "laminar/B738/switch/fo_trim_pos", .handle = NULL, .dataType = XP_FLT, .isArray = false, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.trim_pos_fo},										// 25	DREF_TRIM_POS_FO                 First Officer yoke trim switch: 0/1=direction, 0.5=released
	{.datarefName = "laminar/B738/autopilot/pfd_spd_mode", .handle = NULL, .dataType = XP_FLT, .isArray = true, .arrayOffset = 0, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.pfd_speed_mode_ca},								// 26	DREF_PFD_SPD_MODE_CA
	{.datarefName = "laminar/B738/autopilot/pfd_spd_mode", .handle = NULL, .dataType = XP_FLT, .isArray = true, .arrayOffset = 1, .arrayCount = 1, .isWriteable = false, .isEmittable = true, .ptrVal = (void*)&acData.pfd_speed_mode_fo},								// 27	DREF_PFD_SPD_MODE_FO
};

/* load the command table */
struct CMD_TABLE cmdTable[CMD_END] =
{
	{.commandName = "laminar/B738/autopilot/left_at_dis_press", .handle = NULL},																																														// 0	CMD_LT_AT_DISCO                  Left A/T disconnect button press
	{.commandName = "laminar/B738/autopilot/right_at_dis_press", .handle = NULL},																																														// 1	CMD_RT_AT_DISCO                  Right A/T disconnect button press
	{.commandName = "laminar/B738/autopilot/left_toga_press", .handle = NULL},																																															// 2	CMD_LT_TOGA                      Left TO/GA button press
	{.commandName = "laminar/B738/autopilot/right_toga_press", .handle = NULL},																																															// 3	CMD_RT_TOGA                      Right TO/GA button press
	{.commandName = "laminar/B738/autopilot/right_toga_press", .handle = NULL},																																															// 4	CMD_GEAR_HORN                    Gear horn warning cutout pushbutton
	{.commandName = "laminar/B738/alert/gear_horn_cutout", .handle = NULL},																																																// 5	CMD_PB_BRAKE_MAX                 Use as an alternative to directly setting the P/B position
	{.commandName = "laminar/B738/toggle_switch/el_trim", .handle = NULL},																																																// 6	CMD_EL_TRIM                      Electric trim cutout switch
	{.commandName = "laminar/B738/toggle_switch/el_trim_lock", .handle = NULL},																																															// 7	CMD_EL_TRIMLOCK                  Electric trim switch lock
	{.commandName = "laminar/B738/toggle_switch/ap_trim", .handle = NULL},																																																// 8	CMD_AP_TRIM                      A/P trim cutout switch
	{.commandName = "laminar/B738/toggle_switch/ap_trim_lock", .handle = NULL},																																															// 9	CMD_AP_TRIMLOCK                  A/P trim switch lock
	{.commandName = "sim/flight_controls/brakes_toggle_max", .handle = NULL},																																															// 10	CMD_PB_SET                       Set parking brake (toggle)
};

/* find the datarefs and load the drefTable with the opaque handles */
void GetDataRefHandles(void)
{
	uint16_t	ctr;

	for (ctr = 0; ctr < DREF_END; ctr++)
	{
		drefTable[ctr].handle = XPLMFindDataRef(drefTable[ctr].datarefName);
		if (drefTable[ctr].handle == NULL)
		{
			log_write("Index[%04d]. Unable to find dataref \"%s\".", ctr, drefTable[ctr].datarefName);
		}
	}
}

/* find the commands and load the cmdTable with the opaque handles */
void GetCommandHandles(void)
{
	uint16_t	ctr;

	for (ctr = 0; ctr < CMD_END; ctr++)
	{
		cmdTable[ctr].handle = XPLMFindCommand(cmdTable[ctr].commandName);
		if (cmdTable[ctr].handle == NULL)
		{
			log_write("Index[%04d]. Unable to find command \"%s\".", ctr, cmdTable[ctr].commandName);
		}
	}
	RegisterTqTrimCommandHandlers();
}

/* get the dataref values from the simulator and into the drefTable */
void GetDataRefValues(void* arg)
{
	uint16_t	ctr;
	drefTable_p ptrTable = &drefTable[0];

	state_table_p state = (state_table_p)arg;

	for (ctr = 0; ctr < DREF_END; ctr++)
	{
		if (ptrTable->handle != NULL)															// only get the dataref value if we have a valid handle
		{
			/*!
			 * get acDataBlock values
			 */
			if (ptrTable->dataType == XP_INT)
			{
				int* val = (int*)ptrTable->ptrVal;

				if (ptrTable->isArray)
				{
					int data;
					XPLMGetDatavi(ptrTable->handle, &data, ptrTable->arrayOffset, ptrTable->arrayCount);
					ptrTable->value.intData = data;
				}
				else
				{
					ptrTable->value.intData = XPLMGetDatai(ptrTable->handle);
				}

				if (ptrTable->ptrVal != NULL)
				{
					*val = ptrTable->value.intData;
				}
			}
			if (ptrTable->dataType == XP_FLT)
			{
				float* val = (float*)ptrTable->ptrVal;

				if (ptrTable->isArray)
				{
					float data;
					XPLMGetDatavf(ptrTable->handle, &data, ptrTable->arrayOffset, ptrTable->arrayCount);
					ptrTable->value.fltData = data;
				}
				else
				{
					ptrTable->value.fltData = XPLMGetDataf(ptrTable->handle);
				}
				if (ptrTable->ptrVal != NULL)
				{
					*val = ptrTable->value.fltData;
				}
			}
			if (ptrTable->dataType == XP_DBL)
			{
				double* val = (double*)ptrTable->ptrVal;

				ptrTable->value.dblData = XPLMGetDatad(ptrTable->handle);
				if (ptrTable->ptrVal != NULL)
				{
					*val = ptrTable->value.dblData;
				}
			}
		}
		ptrTable++;
	}
}

/**********************************************************************************/
/* THIS IS THE FLIGHT LOOP CALLBACK                                               */
/**********************************************************************************/
float GetAircraftDataFLCB(float elapsedMe, float elapsedSim, int counter, void* inRefcon)
{
	state_table_p state = (state_table_p)inRefcon;

	/* get the update rate ready to send to X-Plane */
	float flcbReturn = state->flcbUpdateRate;

	/* get the dataref values and populate the acData structure */
	GetDataRefValues((void*)state);

	/* adjust data block variables as required */
	if (acData.pb_ind_raw > 0.0f) acData.pb_indicator = 1; else acData.pb_indicator = 0;			// set the parking brake indicator

	/* A/T control */
	UpdateTqAutothrottleSession();

	/* The battery master directly controls the TQ decals/backlight. */
	pokeys_set_backlight(acData.battery_on > 0.0f);

	/* Apply TO/GA and A/T-disconnect inputs through change-aware handlers. */
	ProcessTqTogaButtons(state);
	ProcessTqAtDisconnectButtons(state);
	ProcessTqFuelCutoffSwitches();
	ProcessTqTrimCutoutSwitches();
	
	/* All X-Plane writes follow the completed read pass and are change-driven. */
	ProcessTqParkingBrake();
	ProcessTqTrim();
	ProcessTqLeverWrites();
	ProcessTqThrottleMotorFollow();

	return(flcbReturn);
}
