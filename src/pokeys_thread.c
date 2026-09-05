/**********************************************************************************/
/* FILE NAME: pokeys_thread.c                                                     */
/*   VERSION: 1.0                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQplugin for X-Plane 12.                  */
/**********************************************************************************/

/* standard include files*/
#include <string.h>
#include <stdio.h>
#include <Windows.h>

/* PoKeys SDK include file */
#include "PoKeysLib.h"

/* project include files */
#include "pokeys_thread.h"
#include "log.h"
#include "plugin_paths.h"


/* local variables */
typedef int32_t								(*EnumerateUsbFn)(void);
typedef int32_t								(*EnumerateNetworkFn)(sPoKeysNetworkDeviceSummary*, uint32_t);
typedef sPoKeysDevice*						(*ConnectIndexFn)(uint32_t);
typedef sPoKeysDevice*						(*ConnectNetworkFn)(sPoKeysNetworkDeviceSummary*);
typedef void								(*DisconnectFn)(sPoKeysDevice*);
typedef int32_t								(*DeviceDataGetFn)(sPoKeysDevice*);
typedef int32_t								(*PinConfigurationFn)(sPoKeysDevice*);
typedef int32_t								(*AnalogGetArrayFn)(sPoKeysDevice*, uint32_t*);
typedef int32_t								(*DigitalIOSetSingleFn)(sPoKeysDevice*, uint8_t, uint8_t);
typedef int32_t								(*DigitalIOGetSingleFn)(sPoKeysDevice*, uint8_t, uint8_t*);
typedef int32_t								(*PWMConfigurationSetDirectlyFn)(sPoKeysDevice*, uint32_t, uint8_t*);
typedef int32_t								(*PWMUpdateDirectlyFn)(sPoKeysDevice*, uint32_t*);

#define SPEEDBRAKE_FLIGHT_DETENT_PIN		30U
#define SPEEDBRAKE_DIRECTION_PIN			25U
#define SPEEDBRAKE_ENABLE_PIN				26U
#define POKEYS_PWM_CHANNELS					6U
#define POKEYS_PWM_PERIOD					500000U
#define SPEEDBRAKE_PWM_CHANNEL				3U
#define SPEEDBRAKE_RETRACT_DUTY				350000U
#define SPEEDBRAKE_EXTEND_DUTY				400000U
#define SPEEDBRAKE_RUN_TIME_MS				2000U
#define PARKING_BRAKE_PWM_CHANNEL			0U
#define PARKING_BRAKE_RELEASE_DUTY			34000U
#define PARKING_BRAKE_SET_DUTY				43000U
#define PARKING_BRAKE_PULSE_MS				500U
#define PARKING_BRAKE_SWITCH_PIN			0U
#define LEFT_TOGA_SWITCH_PIN				1U
#define RIGHT_TOGA_SWITCH_PIN				2U
#define LEFT_AT_DISCONNECT_SWITCH_PIN		3U
#define RIGHT_AT_DISCONNECT_SWITCH_PIN		4U
#define LEFT_FUEL_CUTOFF_SWITCH_PIN			5U
#define RIGHT_FUEL_CUTOFF_SWITCH_PIN		6U
#define ELECTRIC_TRIM_NORMAL_SWITCH_PIN		7U
#define AUTOPILOT_TRIM_NORMAL_SWITCH_PIN	9U
#define BACKLIGHT_PIN						10U
#define PARKING_BRAKE_INDICATOR_PIN			11U
#define TRIM_DIRECTION_A_PIN				27U
#define TRIM_ENABLE_PIN						28U
#define TRIM_DIRECTION_B_PIN				31U
#define TRIM_INDICATOR_PWM_CHANNEL			1U
#define TRIM_MOTOR_PWM_CHANNEL				2U
#define TRIM_POSITION_MIN					400U
#define TRIM_POSITION_MAX					3695U
#define TRIM_POSITION_DEADBAND				50L
#define TRIM_CONSERVATIVE_RANGE				800L
#define TRIM_DIRECTION_BRAKE_MS				100U
#define TRIM_BRAKE_RAMP_STEP_MS				50U
#define TRIM_BRAKE_RAMP_STEP_DUTY			25000U
#define TRIM_FEEDBACK_SAMPLES				12U
#define SPEEDBRAKE_CLOSED_TOLERANCE			75U
#define THROTTLE_LEFT_DIRECTION_PIN			22U
#define THROTTLE_RIGHT_DIRECTION_PIN		23U
#define THROTTLE_LEFT_ENABLE_PIN			24U
#define THROTTLE_RIGHT_ENABLE_PIN			29U
#define THROTTLE_LEFT_PWM_CHANNEL			5U
#define THROTTLE_RIGHT_PWM_CHANNEL			4U
#define THROTTLE_MEDIUM_DUTY				175000U
#define THROTTLE_FAST_DUTY					250000U
#define THROTTLE_ENDPOINT_TOLERANCE			50U
#define THROTTLE_LEG_TIMEOUT_MS				15000U
#define THROTTLE_FOLLOW_DEADBAND			50L
#define THROTTLE_CONSERVATIVE_RANGE			800L
#define THROTTLE_MEDIUM_RANGE				1200L
#define THROTTLE_MAX_SPEED_PERCENT			50L
#define THROTTLE_MANUAL_ERROR_COUNTS		65L
#define THROTTLE_MANUAL_ERROR_SAMPLES		6
#define THROTTLE_MANUAL_DRIVE_GRACE_MS		1500U
#define THROTTLE_MANUAL_COAST_GRACE_MS		1000U
#define THROTTLE_MANUAL_MIN_DUTY			37500U

enum SpeedbrakeCommand
{
	SPEEDBRAKE_COMMAND_NONE = 0,
	SPEEDBRAKE_COMMAND_RETRACT = 1,
	SPEEDBRAKE_COMMAND_EXTEND = 2
};

enum ParkingBrakeCommand
{
	PARKING_BRAKE_COMMAND_NONE = 0,
	PARKING_BRAKE_COMMAND_RELEASE = 1,
	PARKING_BRAKE_COMMAND_SET = 2
};

enum ThrottleTestStage
{
	THROTTLE_TEST_IDLE = 0,
	THROTTLE_TEST_LEFT_MEDIUM_OPEN,
	THROTTLE_TEST_LEFT_MEDIUM_CLOSE,
	THROTTLE_TEST_LEFT_FAST_OPEN,
	THROTTLE_TEST_LEFT_FAST_CLOSE,
	THROTTLE_TEST_RIGHT_MEDIUM_OPEN,
	THROTTLE_TEST_RIGHT_MEDIUM_CLOSE,
	THROTTLE_TEST_RIGHT_FAST_OPEN,
	THROTTLE_TEST_RIGHT_FAST_CLOSE,
	THROTTLE_TEST_BOTH_FAST_OPEN,
	THROTTLE_TEST_BOTH_FAST_CLOSE
};

static HANDLE g_stop_event;
static HANDLE g_thread;
static PluginConfig g_config;
static volatile LONG g_connected;
static SRWLOCK g_status_lock = SRWLOCK_INIT;
static PokeysStatus g_status;
static SRWLOCK g_lever_lock = SRWLOCK_INIT;
static PokeysLeverPositions g_levers;
static SRWLOCK g_parking_brake_input_lock = SRWLOCK_INIT;
static PokeysParkingBrakeInput g_parking_brake_input;
static SRWLOCK g_toga_input_lock = SRWLOCK_INIT;
static PokeysTogaInputs g_toga_inputs;
static SRWLOCK g_at_disconnect_input_lock = SRWLOCK_INIT;
static PokeysAtDisconnectInputs g_at_disconnect_inputs;
static SRWLOCK g_fuel_cutoff_input_lock = SRWLOCK_INIT;
static PokeysFuelCutoffInputs g_fuel_cutoff_inputs;
static SRWLOCK g_trim_cutout_input_lock = SRWLOCK_INIT;
static PokeysTrimCutoutInputs g_trim_cutout_inputs;
static volatile LONG g_aircraft_in_flight;
static volatile LONG g_calibration_active;
static volatile LONG g_detent_update_requested = 1;
static volatile LONG g_detent_retracted;
static volatile LONG g_speedbrake_detent_override;
static volatile LONG g_speedbrake_command;
static volatile LONG g_parking_brake_command;
static volatile LONG g_parking_brake_state;
static volatile LONG g_parking_brake_active_command;
static volatile LONG g_parking_brake_release_complete;
static volatile LONG g_parking_brake_indicator;
static volatile LONG g_parking_brake_indicator_update_requested;
static volatile LONG g_backlight;
static volatile LONG g_backlight_update_requested;
static volatile LONG g_trim_target_position;
static volatile LONG g_trim_motor_enabled;
static volatile LONG g_trim_manual_enabled;
static volatile LONG g_trim_manual_direction;
static volatile LONG g_trim_min_speed;
static volatile LONG g_trim_motor_running;
static volatile LONG g_trim_indicator_target_position;
static volatile LONG g_trim_indicator_simulator_owned;
static volatile LONG g_speedbrake_closed_position;
static volatile LONG g_throttle_test_requested;
static volatile LONG g_throttle_test_running;
static volatile LONG g_throttle_limits_valid;
static volatile LONG g_throttle_left_min;
static volatile LONG g_throttle_left_max;
static volatile LONG g_throttle_right_min;
static volatile LONG g_throttle_right_max;
static volatile LONG g_throttle_follow_enabled;
static volatile LONG g_throttle_left_target;
static volatile LONG g_throttle_right_target;
static volatile LONG g_throttle_left_min_speed;
static volatile LONG g_throttle_right_min_speed;
static volatile LONG g_throttle_manual_override_mask;
static SRWLOCK g_throttle_test_status_lock = SRWLOCK_INIT;
static char g_throttle_test_status[128] = "Throttle test ready";

/* pokeys data structure */
typedef struct PokeysApi
{
	HMODULE							module;
	EnumerateUsbFn					enumerate_usb;
	EnumerateNetworkFn				enumerate_network;
	ConnectIndexFn					connect_index;
	ConnectNetworkFn				connect_network;
	DisconnectFn					disconnect;
	DeviceDataGetFn					device_data_get;
	PinConfigurationFn				pin_configuration_get;
	PinConfigurationFn				pin_configuration_set;
	AnalogGetArrayFn				analog_get_array;
	DigitalIOSetSingleFn			digital_io_set_single;
	DigitalIOGetSingleFn			digital_io_get_single;
	PWMConfigurationSetDirectlyFn	pwm_configuration_set_directly;
	PWMUpdateDirectlyFn				pwm_update_directly;
} PokeysApi;

/*
 * The original application smooths each motor feedback potentiometer with
 * twelve samples and rejects the lowest and highest readings. Keep that
 * filtering local to the PoKeys worker so motor control never depends on
 * unsynchronised X-Plane-thread state.
 */
typedef struct MinMaxFeedbackFilter
{
	uint32_t samples[TRIM_FEEDBACK_SAMPLES];
	uint32_t next;
	int initialised;
} MinMaxFeedbackFilter;

/*
 * Worker-owned state used to distinguish pilot intervention from normal motor
 * travel. It mirrors the original .NET controller's 65-count error threshold,
 * six consecutive error samples, 1.5-second powered grace period and
 * one-second coast grace period.
 */
typedef struct ThrottleManualMonitor
{
	LONG previous_position;
	LONG previous_target;
	LONG coast_reference;
	ULONGLONG drive_started_at;
	ULONGLONG coast_started_at;
	int previous_direction;
	int motor_error_count;
	int initialised;
} ThrottleManualMonitor;

/* Worker-owned, non-blocking equivalent of the .NET RampDownMotor method. */
typedef struct TrimBrakeRamp
{
	ULONGLONG next_step_at;
	LONG target_at_start;
	int manual_mode;
	int active;
} TrimBrakeRamp;

static ThrottleManualMonitor g_throttle_manual_monitor[2];

static void minmax_feedback_filter_reset(MinMaxFeedbackFilter* filter)
{
	memset(filter, 0, sizeof(*filter));
}

static uint32_t minmax_feedback_filter_add(MinMaxFeedbackFilter* filter, uint32_t sample)
{
	uint32_t sorted[TRIM_FEEDBACK_SAMPLES];
	uint64_t total = 0U;
	uint32_t i;

	if (!filter->initialised) 
	{
		for (i = 0U; i < TRIM_FEEDBACK_SAMPLES; ++i)
			filter->samples[i] = sample;
	
		filter->initialised = 1;
	}
	else
	{
		filter->samples[filter->next] = sample;
		filter->next = (filter->next + 1U) % TRIM_FEEDBACK_SAMPLES;
	}

	memcpy(sorted, filter->samples, sizeof(sorted));
	for (i = 1U; i < TRIM_FEEDBACK_SAMPLES; ++i) 
	{
		uint32_t value = sorted[i];
		uint32_t j = i;

		while (j > 0U && sorted[j - 1U] > value) 
		{
			sorted[j] = sorted[j - 1U];
			--j;
		}
		sorted[j] = value;
	}
	for (i = 1U; i < TRIM_FEEDBACK_SAMPLES - 1U; ++i)
		total += sorted[i];

	return((uint32_t)(total / (TRIM_FEEDBACK_SAMPLES - 2U)));
}

static int flight_detent_should_be_retracted(void)
{
	return(InterlockedCompareExchange(&g_calibration_active, 0, 0) != 0 || InterlockedCompareExchange(&g_speedbrake_detent_override, 0, 0) != 0 || InterlockedCompareExchange(&g_aircraft_in_flight, 0, 0) == 0);
}

static int set_logical_output(PokeysApi* api, sPoKeysDevice* device, uint8_t pin, int state)
{
	/* PoKeysDevice.SetOutput uses active-low values on this TQ. */
	return(api->digital_io_set_single(device, pin, (uint8_t)(state ? 0U : 1U)) == PK_OK);
}

/*
 * Parking-brake lamp pin 11 is electrically active-low on the CFY TQ. Hardware
 * testing confirmed that logical ON must drive zero and logical OFF must drive
 * one, matching the other logical outputs handled by set_logical_output().
 */
static int set_parking_brake_indicator_output(PokeysApi* api, sPoKeysDevice* device, int illuminated)
{
	return(set_logical_output(api, device, PARKING_BRAKE_INDICATOR_PIN, illuminated));
}

/*
 * PoKeys API pin 10 drives the CFY TQ decals/backlight. The original .NET
 * application uses SetOutput(10, state), so it has the same active-low
 * electrical convention as the other logical TQ outputs.
 */
static int set_backlight_output(PokeysApi* api, sPoKeysDevice* device, int illuminated)
{
	return (set_logical_output(api, device, BACKLIGHT_PIN, illuminated));
}

static void throttle_test_status_set(const char* status)
{
	AcquireSRWLockExclusive(&g_throttle_test_status_lock);
	strncpy_s(g_throttle_test_status, sizeof(g_throttle_test_status), status, _TRUNCATE);
	ReleaseSRWLockExclusive(&g_throttle_test_status_lock);
}

static int apply_flight_detent_state(PokeysApi* api, sPoKeysDevice* device, int force)
{
	static int applied_state = -1;
	static int failure_logged;
	int retract = flight_detent_should_be_retracted();
	int requested = InterlockedExchange(&g_detent_update_requested, 0) != 0;
	int32_t result;

	if (!force && !requested && applied_state == retract) return (1);
	/*
	 * The CFY wiring and the original PoKeys .NET SetOutput implementation
	 * use active-low logic: logical TRUE (release/retract) is transmitted as
	 * zero, while logical FALSE (engage) is transmitted as one.
	 */
	result = api->digital_io_set_single(device, SPEEDBRAKE_FLIGHT_DETENT_PIN, (uint8_t)(retract ? 0U : 1U));
	if (result != PK_OK) 
	{
		InterlockedExchange(&g_detent_retracted, 0);
		InterlockedExchange(&g_detent_update_requested, 1);
		if (!failure_logged)
		{
			log_write("Unable to %s speedbrake flight-detent lock on pin %u (result %ld)", retract ? "retract" : "engage", SPEEDBRAKE_FLIGHT_DETENT_PIN, (long)result);
			failure_logged = 1;
		}
		return(0);
	}

	failure_logged = 0;
	if (applied_state != retract)
		log_write("Speedbrake flight-detent lock %s", retract ? "retracted" : "engaged for flight");
	applied_state = retract;
	InterlockedExchange(&g_detent_retracted, retract ? 1 : 0);
	return(1);
}

static void levers_set_disconnected(void)
{
	AcquireSRWLockExclusive(&g_lever_lock);
	g_levers.connected = 0;
	g_levers.valid = 0;
	++g_levers.sequence;
	ReleaseSRWLockExclusive(&g_lever_lock);
}

static void levers_set_unavailable(void)
{
	AcquireSRWLockExclusive(&g_lever_lock);
	g_levers.connected = 1;
	g_levers.valid = 0;
	++g_levers.sequence;
	ReleaseSRWLockExclusive(&g_lever_lock);
}

static void levers_update(const uint32_t* raw)
{
	AcquireSRWLockExclusive(&g_lever_lock);
	g_levers.connected = 1;
	g_levers.valid = 1;
	g_levers.value[POKEYS_LEVER_THROTTLE_1] = 4095U - (raw[0] > 4095U ? 4095U : raw[0]);
	g_levers.value[POKEYS_LEVER_THROTTLE_2] = 4095U - (raw[1] > 4095U ? 4095U : raw[1]);
	g_levers.value[POKEYS_LEVER_SPEED_BRAKE] = 4095U - (raw[2] > 4095U ? 4095U : raw[2]);
	g_levers.value[POKEYS_LEVER_TRIM] = 4095U - (raw[3] > 4095U ? 4095U : raw[3]);
	g_levers.value[POKEYS_LEVER_REVERSER_1] = raw[4] > 4095U ? 4095U : raw[4];
	g_levers.value[POKEYS_LEVER_REVERSER_2] = raw[5] > 4095U ? 4095U : raw[5];
	g_levers.value[POKEYS_LEVER_FLAPS] = raw[6] > 4095U ? 4095U : raw[6];
	++g_levers.sequence;
	ReleaseSRWLockExclusive(&g_lever_lock);
}

static void parking_brake_input_set_disconnected(void)
{
	AcquireSRWLockExclusive(&g_parking_brake_input_lock);
	g_parking_brake_input.connected = 0;
	g_parking_brake_input.valid = 0;
	++g_parking_brake_input.sequence;
	ReleaseSRWLockExclusive(&g_parking_brake_input_lock);
}

static void parking_brake_input_set_unavailable(void)
{
	AcquireSRWLockExclusive(&g_parking_brake_input_lock);
	g_parking_brake_input.connected = 1;
	g_parking_brake_input.valid = 0;
	++g_parking_brake_input.sequence;
	ReleaseSRWLockExclusive(&g_parking_brake_input_lock);
}

static void parking_brake_input_update(int engaged)
{
	AcquireSRWLockExclusive(&g_parking_brake_input_lock);
	g_parking_brake_input.connected = 1;
	if (!g_parking_brake_input.valid ||
		g_parking_brake_input.engaged != (engaged != 0)) {
		g_parking_brake_input.engaged = engaged != 0;
		++g_parking_brake_input.sequence;
	}
	g_parking_brake_input.valid = 1;
	ReleaseSRWLockExclusive(&g_parking_brake_input_lock);
}

/* Publish a safe released state when the device connection is unavailable. */
static void toga_inputs_set_disconnected(void)
{
	AcquireSRWLockExclusive(&g_toga_input_lock);
	g_toga_inputs.connected = 0;
	g_toga_inputs.valid = 0;
	g_toga_inputs.left_pressed = 0;
	g_toga_inputs.right_pressed = 0;
	++g_toga_inputs.sequence;
	ReleaseSRWLockExclusive(&g_toga_input_lock);
}

/* Invalidate both buttons after repeated read failures to prevent stuck commands. */
static void toga_inputs_set_unavailable(void)
{
	AcquireSRWLockExclusive(&g_toga_input_lock);
	g_toga_inputs.connected = 1;
	g_toga_inputs.valid = 0;
	g_toga_inputs.left_pressed = 0;
	g_toga_inputs.right_pressed = 0;
	++g_toga_inputs.sequence;
	ReleaseSRWLockExclusive(&g_toga_input_lock);
}

/* Publish both sides under one lock so the flight loop sees a coherent sample. */
static void toga_inputs_update(int left_pressed, int right_pressed)
{
	left_pressed = left_pressed != 0;
	right_pressed = right_pressed != 0;
	AcquireSRWLockExclusive(&g_toga_input_lock);
	g_toga_inputs.connected = 1;
	if (!g_toga_inputs.valid || g_toga_inputs.left_pressed != left_pressed || g_toga_inputs.right_pressed != right_pressed)
	{
		g_toga_inputs.left_pressed = left_pressed;
		g_toga_inputs.right_pressed = right_pressed;
		++g_toga_inputs.sequence;
	}
	g_toga_inputs.valid = 1;
	ReleaseSRWLockExclusive(&g_toga_input_lock);
}

static void at_disconnect_inputs_set_disconnected(void)
{
	AcquireSRWLockExclusive(&g_at_disconnect_input_lock);
	g_at_disconnect_inputs.connected = 0;
	g_at_disconnect_inputs.valid = 0;
	g_at_disconnect_inputs.left_pressed = 0;
	g_at_disconnect_inputs.right_pressed = 0;
	++g_at_disconnect_inputs.sequence;
	ReleaseSRWLockExclusive(&g_at_disconnect_input_lock);
}

static void at_disconnect_inputs_set_unavailable(void)
{
	AcquireSRWLockExclusive(&g_at_disconnect_input_lock);
	g_at_disconnect_inputs.connected = 1;
	g_at_disconnect_inputs.valid = 0;
	g_at_disconnect_inputs.left_pressed = 0;
	g_at_disconnect_inputs.right_pressed = 0;
	++g_at_disconnect_inputs.sequence;
	ReleaseSRWLockExclusive(&g_at_disconnect_input_lock);
}

static void at_disconnect_inputs_update(int left_pressed, int right_pressed)
{
	left_pressed = left_pressed != 0;
	right_pressed = right_pressed != 0;
	AcquireSRWLockExclusive(&g_at_disconnect_input_lock);
	g_at_disconnect_inputs.connected = 1;
	if (!g_at_disconnect_inputs.valid ||
		g_at_disconnect_inputs.left_pressed != left_pressed ||
		g_at_disconnect_inputs.right_pressed != right_pressed) {
		g_at_disconnect_inputs.left_pressed = left_pressed;
		g_at_disconnect_inputs.right_pressed = right_pressed;
		++g_at_disconnect_inputs.sequence;
	}
	g_at_disconnect_inputs.valid = 1;
	ReleaseSRWLockExclusive(&g_at_disconnect_input_lock);
}

static void fuel_cutoff_inputs_set_disconnected(void)
{
	AcquireSRWLockExclusive(&g_fuel_cutoff_input_lock);
	g_fuel_cutoff_inputs.connected = 0;
	g_fuel_cutoff_inputs.valid = 0;
	g_fuel_cutoff_inputs.left_cutoff = 0;
	g_fuel_cutoff_inputs.right_cutoff = 0;
	++g_fuel_cutoff_inputs.sequence;
	ReleaseSRWLockExclusive(&g_fuel_cutoff_input_lock);
}

static void fuel_cutoff_inputs_set_unavailable(void)
{
	AcquireSRWLockExclusive(&g_fuel_cutoff_input_lock);
	g_fuel_cutoff_inputs.connected = 1;
	g_fuel_cutoff_inputs.valid = 0;
	++g_fuel_cutoff_inputs.sequence;
	ReleaseSRWLockExclusive(&g_fuel_cutoff_input_lock);
}

static void fuel_cutoff_inputs_update(int left_cutoff, int right_cutoff)
{
	left_cutoff = left_cutoff != 0;
	right_cutoff = right_cutoff != 0;
	AcquireSRWLockExclusive(&g_fuel_cutoff_input_lock);
	g_fuel_cutoff_inputs.connected = 1;
	if (!g_fuel_cutoff_inputs.valid ||
		g_fuel_cutoff_inputs.left_cutoff != left_cutoff ||
		g_fuel_cutoff_inputs.right_cutoff != right_cutoff) {
		g_fuel_cutoff_inputs.left_cutoff = left_cutoff;
		g_fuel_cutoff_inputs.right_cutoff = right_cutoff;
		++g_fuel_cutoff_inputs.sequence;
	}
	g_fuel_cutoff_inputs.valid = 1;
	ReleaseSRWLockExclusive(&g_fuel_cutoff_input_lock);
}

static void trim_cutout_inputs_set_disconnected(void)
{
	AcquireSRWLockExclusive(&g_trim_cutout_input_lock);
	g_trim_cutout_inputs.connected = 0;
	g_trim_cutout_inputs.valid = 0;
	++g_trim_cutout_inputs.sequence;
	ReleaseSRWLockExclusive(&g_trim_cutout_input_lock);
}

static void trim_cutout_inputs_set_unavailable(void)
{
	AcquireSRWLockExclusive(&g_trim_cutout_input_lock);
	g_trim_cutout_inputs.connected = 1;
	g_trim_cutout_inputs.valid = 0;
	++g_trim_cutout_inputs.sequence;
	ReleaseSRWLockExclusive(&g_trim_cutout_input_lock);
}

/* Publish both maintained trim switches as one coherent first-run snapshot. */
static void trim_cutout_inputs_update(int electric_normal,
	int autopilot_normal)
{
	electric_normal = electric_normal != 0;
	autopilot_normal = autopilot_normal != 0;
	AcquireSRWLockExclusive(&g_trim_cutout_input_lock);
	g_trim_cutout_inputs.connected = 1;
	if (!g_trim_cutout_inputs.valid || g_trim_cutout_inputs.electric_normal != electric_normal || g_trim_cutout_inputs.autopilot_normal != autopilot_normal) 
	{
		g_trim_cutout_inputs.electric_normal = electric_normal;
		g_trim_cutout_inputs.autopilot_normal = autopilot_normal;
		++g_trim_cutout_inputs.sequence;
	}
	g_trim_cutout_inputs.valid = 1;
	ReleaseSRWLockExclusive(&g_trim_cutout_input_lock);
}

static void status_set_disconnected(const char* detail)
{
	AcquireSRWLockExclusive(&g_status_lock);
	memset(&g_status, 0, sizeof(g_status));
	strcpy_s(g_status.ip_address, sizeof(g_status.ip_address), "Not available");
	strcpy_s(g_status.protocol, sizeof(g_status.protocol), "None");
	strcpy_s(g_status.firmware_version, sizeof(g_status.firmware_version), "Not available");
	strcpy_s(g_status.detail, sizeof(g_status.detail), detail ? detail : "Searching for TQ");
	ReleaseSRWLockExclusive(&g_status_lock);
	InterlockedExchange(&g_connected, 0);
}

static void status_set_connected(const sPoKeysDevice* device, const char* ip_address, const char* protocol)
{
	AcquireSRWLockExclusive(&g_status_lock);
	memset(&g_status, 0, sizeof(g_status));
	g_status.connected = 1;
	g_status.serial_number = device->DeviceData.SerialNumber;
	strcpy_s(g_status.ip_address, sizeof(g_status.ip_address), ip_address);
	strcpy_s(g_status.protocol, sizeof(g_status.protocol), protocol);
	snprintf(g_status.firmware_version, sizeof(g_status.firmware_version), "%u.%u.%u", 1u + (device->DeviceData.FirmwareVersionMajor >> 4), device->DeviceData.FirmwareVersionMajor & 0x0fu, device->DeviceData.FirmwareVersionMinor);
	strcpy_s(g_status.detail, sizeof(g_status.detail), "PoKeys communication active");
	ReleaseSRWLockExclusive(&g_status_lock);
	InterlockedExchange(&g_connected, 1);
}

/**********************************************************************************/
/* check for reqquired PoKeys API functions                                       */
/**********************************************************************************/
static FARPROC api_proc(HMODULE module, const char* name)
{
	FARPROC result = GetProcAddress(module, name);
	if (!result) log_write("PoKeyslib.dll does not export %s (error %lu)", name, GetLastError());
	return result;
}

/**********************************************************************************/
/* Load the PoKeys API and initialise the API data structure                      */
/**********************************************************************************/
static int api_load(PokeysApi* api, char* error_detail, size_t error_detail_size)
{
	char path[MAX_PATH];
	memset(api, 0, sizeof(*api));

	if (!plugin_file_path(path, sizeof(path), "PoKeyslib.dll"))
	{
		strcpy_s(error_detail, error_detail_size, "PoKeys DLL path is too long");
		return (0);
	}

	api->module = LoadLibraryA(path);

	if (!api->module) 
	{
		DWORD error = GetLastError();
		log_write("Unable to load %s (error %lu)", path, error);
		snprintf(error_detail, error_detail_size,
			"PoKeyslib.dll load failed (Windows error %lu)", error);
		return(0);
	}
	
	api->enumerate_usb = (EnumerateUsbFn)api_proc(api->module, "PK_EnumerateUSBDevices");
	api->enumerate_network = (EnumerateNetworkFn)api_proc(api->module, "PK_EnumerateNetworkDevices");
	api->connect_index = (ConnectIndexFn)api_proc(api->module, "PK_ConnectToDevice");
	api->connect_network = (ConnectNetworkFn)api_proc(api->module, "PK_ConnectToNetworkDevice");
	api->disconnect = (DisconnectFn)api_proc(api->module, "PK_DisconnectDevice");
	api->device_data_get = (DeviceDataGetFn)api_proc(api->module, "PK_DeviceDataGet");
	api->pin_configuration_get = (PinConfigurationFn)api_proc(api->module, "PK_PinConfigurationGet");
	api->pin_configuration_set = (PinConfigurationFn)api_proc(api->module, "PK_PinConfigurationSet");
	api->analog_get_array = (AnalogGetArrayFn)api_proc(api->module, "PK_AnalogIOGetAsArray");
	api->digital_io_set_single = (DigitalIOSetSingleFn)api_proc(api->module, "PK_DigitalIOSetSingle");
	api->digital_io_get_single = (DigitalIOGetSingleFn)api_proc(api->module, "PK_DigitalIOGetSingle");
	api->pwm_configuration_set_directly = (PWMConfigurationSetDirectlyFn)api_proc(api->module, "PK_PWMConfigurationSetDirectly");
	api->pwm_update_directly = (PWMUpdateDirectlyFn)api_proc(api->module, "PK_PWMUpdateDirectly");
	if (!api->enumerate_usb || !api->enumerate_network || !api->connect_index || !api->connect_network || !api->disconnect || !api->device_data_get || !api->pin_configuration_get || !api->pin_configuration_set || !api->analog_get_array || !api->digital_io_set_single || !api->digital_io_get_single || !api->pwm_configuration_set_directly || !api->pwm_update_directly) 
	{
		FreeLibrary(api->module);
		memset(api, 0, sizeof(*api));
		strcpy_s(error_detail, error_detail_size, "PoKeyslib.dll has missing API exports");
		return(0);
	}
	return (1);
}

static int configure_actuator_outputs(PokeysApi* api, sPoKeysDevice* device)
{
	uint8_t enabled_channels[POKEYS_PWM_CHANNELS] = { 1U, 1U, 1U, 1U, 1U, 1U };
	uint32_t duty_cycles[POKEYS_PWM_CHANNELS] = { 0U };

	if (!device->Pins || device->info.iPinCount <= TRIM_DIRECTION_B_PIN) 
	{
		log_write("PoKeys serial %u does not expose required actuator pin %u", device->DeviceData.SerialNumber, TRIM_DIRECTION_B_PIN);
		return(0);
	}
	if (api->pin_configuration_get(device) != PK_OK) 
	{
		log_write("Unable to read PoKeys pin configuration for TQ actuators");
		return(0);
	}
	device->Pins[SPEEDBRAKE_DIRECTION_PIN].PinFunction = PK_PinCap_digitalOutput;
	device->Pins[SPEEDBRAKE_ENABLE_PIN].PinFunction = PK_PinCap_digitalOutput;
	device->Pins[SPEEDBRAKE_FLIGHT_DETENT_PIN].PinFunction = PK_PinCap_digitalOutput;
	device->Pins[THROTTLE_LEFT_DIRECTION_PIN].PinFunction = PK_PinCap_digitalOutput;
	device->Pins[THROTTLE_RIGHT_DIRECTION_PIN].PinFunction = PK_PinCap_digitalOutput;
	device->Pins[THROTTLE_LEFT_ENABLE_PIN].PinFunction = PK_PinCap_digitalOutput;
	device->Pins[THROTTLE_RIGHT_ENABLE_PIN].PinFunction = PK_PinCap_digitalOutput;
	device->Pins[TRIM_DIRECTION_A_PIN].PinFunction = PK_PinCap_digitalOutput;
	device->Pins[TRIM_ENABLE_PIN].PinFunction = PK_PinCap_digitalOutput;
	device->Pins[TRIM_DIRECTION_B_PIN].PinFunction = PK_PinCap_digitalOutput;
	/* Most maintained and momentary TQ controls use active-low inputs. */
	device->Pins[PARKING_BRAKE_SWITCH_PIN].PinFunction = PK_PinCap_digitalInput | PK_PinCap_invertPin;
	device->Pins[LEFT_TOGA_SWITCH_PIN].PinFunction = PK_PinCap_digitalInput | PK_PinCap_invertPin;
	device->Pins[RIGHT_TOGA_SWITCH_PIN].PinFunction = PK_PinCap_digitalInput | PK_PinCap_invertPin;
	device->Pins[LEFT_AT_DISCONNECT_SWITCH_PIN].PinFunction = PK_PinCap_digitalInput | PK_PinCap_invertPin;
	device->Pins[RIGHT_AT_DISCONNECT_SWITCH_PIN].PinFunction = PK_PinCap_digitalInput | PK_PinCap_invertPin;
	device->Pins[LEFT_FUEL_CUTOFF_SWITCH_PIN].PinFunction = PK_PinCap_digitalInput | PK_PinCap_invertPin;
	device->Pins[RIGHT_FUEL_CUTOFF_SWITCH_PIN].PinFunction = PK_PinCap_digitalInput | PK_PinCap_invertPin;
	/*
	 * Trim MAIN ELEC and AUTOPILOT are the exceptions: the original application
	 * configures API pins 7 and 9 with mode 2 (plain digital input), not mode
	 * 130 (digital input plus inversion). Hardware NORMAL therefore maps
	 * directly to a non-zero PoKeys value.
	 */
	device->Pins[ELECTRIC_TRIM_NORMAL_SWITCH_PIN].PinFunction =	PK_PinCap_digitalInput;
	device->Pins[AUTOPILOT_TRIM_NORMAL_SWITCH_PIN].PinFunction = PK_PinCap_digitalInput;
	device->Pins[BACKLIGHT_PIN].PinFunction = PK_PinCap_digitalOutput;
	device->Pins[PARKING_BRAKE_INDICATOR_PIN].PinFunction = PK_PinCap_digitalOutput;
	/* Active-low OFF is one; seed it before applying output mode. */
	device->Pins[BACKLIGHT_PIN].DigitalValueSet = 1U;
	device->Pins[PARKING_BRAKE_INDICATOR_PIN].DigitalValueSet = 1U;
	if (api->pin_configuration_set(device) != PK_OK) 
	{
		log_write("Unable to configure speedbrake actuator output pins");
		return(0);
	}
	if (!set_logical_output(api, device, SPEEDBRAKE_ENABLE_PIN, 0)) 
	{
		log_write("Unable to place speedbrake motor in safe coast state");
		return(0);
	}
	if (!set_logical_output(api, device, THROTTLE_LEFT_ENABLE_PIN, 0) || !set_logical_output(api, device, THROTTLE_RIGHT_ENABLE_PIN, 0)) 
	{
		log_write("Unable to place throttle motors in safe coast state");
		return(0);
	}
	if (!set_logical_output(api, device, TRIM_ENABLE_PIN, 0) || !set_logical_output(api, device, TRIM_DIRECTION_A_PIN, 0) || !set_logical_output(api, device, TRIM_DIRECTION_B_PIN, 0))
	{
		log_write("Unable to place stabiliser-trim motor in safe coast state");
		return(0);
	}
	if (!set_parking_brake_indicator_output(api, device, 0)) 
	{
		log_write("Unable to switch off parking-brake indicator during initialisation");
		return(0);
	}
	if (!set_backlight_output(api, device, 0)) 
	{
		log_write("Unable to switch off TQ backlight during initialisation");
		return(0);
	}
	if (api->pwm_configuration_set_directly(device, POKEYS_PWM_PERIOD, enabled_channels) != PK_OK || api->pwm_update_directly(device, duty_cycles) != PK_OK) 
	{
		log_write("Unable to initialise TQ PWM outputs");
		return(0);
	}
	log_write("TQ actuator PWM configured at period %u", POKEYS_PWM_PERIOD);
	log_write("Stabiliser-trim hardware configured for CFY TQ %s topology",	g_config.trim_motor_variant == 3U ? "V3" : (g_config.trim_motor_variant == 5U ? "Pro" : "V4"));	InterlockedExchange(&g_detent_update_requested, 1);
	InterlockedExchange(&g_parking_brake_indicator_update_requested, 1);
	InterlockedExchange(&g_backlight_update_requested, 1);
	/* Always drive the interlock to its safe released state after connection. */
	InterlockedExchange(&g_parking_brake_state, POKEYS_PARKING_BRAKE_UNKNOWN);
	InterlockedExchange(&g_parking_brake_active_command, PARKING_BRAKE_COMMAND_NONE);
	InterlockedExchange(&g_parking_brake_release_complete, 0);
	InterlockedExchange(&g_parking_brake_command, PARKING_BRAKE_COMMAND_RELEASE);
	return(apply_flight_detent_state(api, device, 1));
}

static int configure_analog_inputs(PokeysApi* api, sPoKeysDevice* device)
{
	uint32_t index;
	if (!device->Pins || device->info.iPinCount < 47U)
	{
		log_write("PoKeys serial %u does not expose the required analogue pins",
			device->DeviceData.SerialNumber);
		return(0);
	}
	if (api->pin_configuration_get(device) != PK_OK) 
	{
		log_write("Unable to read PoKeys pin configuration");
		return(0);
	}
	for (index = 40U; index <= 46U; ++index)
		device->Pins[index].PinFunction = PK_PinCap_analogInput;
	if (api->pin_configuration_set(device) != PK_OK) 
	{
		log_write("Unable to configure TQ analogue input pins 40-46");
		return(0);
	}
	log_write("TQ analogue input pins 40-46 configured");
	return(1);
}

/**********************************************************************************/
/* check for PoKeys device match                                                  */
/**********************************************************************************/
static int device_matches(const sPoKeysDevice* device)
{
	if (g_config.preferred_serial && device->DeviceData.SerialNumber != g_config.preferred_serial) 
		return(0);
	return(!g_config.require_cfy_user_id || device->DeviceData.UserID == 12);
}

/**********************************************************************************/
/* connect a PoKeys USB device                                                    */
/**********************************************************************************/
static sPoKeysDevice* connect_usb(PokeysApi* api)
{
	int32_t count = api->enumerate_usb(), index;
	if (count < 0) 
	{
		log_write("USB PoKeys enumeration failed with result %ld", (long)count);
		return NULL;
	}
	if (count > 0) log_write("USB discovery found %ld device(s)", (long)count);
	for (index = 0; index < count && WaitForSingleObject(g_stop_event, 0) != WAIT_OBJECT_0; ++index)
	{
		sPoKeysDevice* device = api->connect_index((uint32_t)index);
		int32_t data_result;
		if (!device) 
		{
			log_write("Unable to connect to USB PoKeys candidate %ld", (long)index);
			continue;
		}
		data_result = api->device_data_get(device);
		if (data_result != PK_OK) 
		{
			log_write("Unable to read USB PoKeys candidate %ld device data (result %ld)",
				(long)index, (long)data_result);
			api->disconnect(device);
			continue;
		}
		log_write("USB candidate %ld reports serial %u, user ID %u", (long)index, device->DeviceData.SerialNumber, device->DeviceData.UserID);
		if (device_matches(device))
		{
			if (!configure_analog_inputs(api, device))
				log_write("USB connection retained, but lever readings may be unavailable");
			if (!configure_actuator_outputs(api, device)) 
			{
				log_write("USB connection rejected because the TQ actuators could not be made safe");
				api->disconnect(device);
				continue;
			}
			log_write("Connected to USB Pokeys serial %u", device->DeviceData.SerialNumber);
			status_set_connected(device, "Not applicable", "USB");
			return (device);
		}
		log_write("USB PoKeys serial %u rejected by configured serial/user-ID filter", device->DeviceData.SerialNumber);
		api->disconnect(device);
	}
	return (NULL);
}

/**********************************************************************************/
/* connect a PoKeys Ethernet device                                               */
/**********************************************************************************/
static sPoKeysDevice* connect_network(PokeysApi* api)
{
	sPoKeysNetworkDeviceSummary devices[64];
	int32_t count, index;
	uint32_t timeout = g_config.network_timeout_ms;
	if (timeout > g_config.discovery_timeout_ms) timeout = g_config.discovery_timeout_ms;
	memset(devices, 0, sizeof(devices));
	count = api->enumerate_network(devices, timeout);
	if (count < 0) {
		log_write("Network PoKeys enumeration failed with result %ld", (long)count);
		return NULL;
	}
	if (count > 64) count = 64;
	if (count > 0) log_write("Network discovery found %ld device(s)", (long)count);
	for (index = 0; index < count && WaitForSingleObject(g_stop_event, 0) != WAIT_OBJECT_0; ++index) 
	{
		sPoKeysDevice* device;
		int32_t data_result;
		log_write("Network candidate %ld: serial %u, summary user ID %u, IP %u.%u.%u.%u, %s",
			(long)index, devices[index].SerialNumber, devices[index].UserID,
			devices[index].IPaddress[0], devices[index].IPaddress[1],
			devices[index].IPaddress[2], devices[index].IPaddress[3],
			devices[index].useUDP ? "UDP" : "TCP");

		/* The discovery summary can report UserID 0 even when DeviceDataGet reports
		   the configured value. Only serial number is trustworthy before connect. */
		if (g_config.preferred_serial && devices[index].SerialNumber != g_config.preferred_serial) 
		{
			log_write("Network PoKeys serial %u rejected by preferred-serial filter", devices[index].SerialNumber);
			continue;
		}
	
		device = api->connect_network(&devices[index]);
		if (!device) 
		{
			log_write("Unable to connect to network PoKeys serial %u", devices[index].SerialNumber);
			continue;
		}
		data_result = api->device_data_get(device);
		if (data_result != PK_OK) 
		{
			log_write("Unable to read network PoKeys serial %u device data (result %ld)", devices[index].SerialNumber, (long)data_result);
			api->disconnect(device);
			continue;
		}
		log_write("Connected candidate serial %u reports user ID %u", device->DeviceData.SerialNumber, device->DeviceData.UserID);
		if (device_matches(device)) 
		{
			char ip_address[16];
			const char* protocol = devices[index].useUDP ? "UDP" : "TCP";
			snprintf(ip_address, sizeof(ip_address), "%u.%u.%u.%u",	devices[index].IPaddress[0], devices[index].IPaddress[1], devices[index].IPaddress[2], devices[index].IPaddress[3]);
			if (!configure_analog_inputs(api, device))
				log_write("Network connection retained, but lever readings may be unavailable");
			if (!configure_actuator_outputs(api, device)) 
			{
				log_write("Network connection rejected because the TQ actuators could not be made safe");
				api->disconnect(device);
				continue;
			}
			log_write("Connected to network Pokeys %u at %u.%u.%u.%u", device->DeviceData.SerialNumber,	devices[index].IPaddress[0], devices[index].IPaddress[1], devices[index].IPaddress[2], devices[index].IPaddress[3]);
			status_set_connected(device, ip_address, protocol);
			return (device);
		}
		log_write("Network PoKeys serial %u rejected by configured user-ID filter",	device->DeviceData.SerialNumber);
		api->disconnect(device);
	}
	return (NULL);
}

static int pwm_update(PokeysApi* api, sPoKeysDevice* device, uint32_t duty_cycles[POKEYS_PWM_CHANNELS], const char* operation)
{
	int32_t result = api->pwm_update_directly(device, duty_cycles);
	if (result == PK_OK) 
		return(1);
	log_write("PoKeys PWM update failed while %s (result %ld)", operation,
		(long)result);
	return(0);
}

/*
 * Apply the battery-gated simulator annunciator state on the device thread.
 * The public setter only updates atomics so X-Plane callbacks never enter the
 * PoKeys DLL. Lamp pin 11 uses active-low electrical polarity.
 */
static void process_parking_brake_indicator(PokeysApi* api,
	sPoKeysDevice* device, int* applied_state)
{
	int requested = InterlockedExchange(&g_parking_brake_indicator_update_requested, 0) != 0;
	int desired = InterlockedCompareExchange(&g_parking_brake_indicator, 0, 0) != 0;

	if (!requested && *applied_state == desired) 
		return;

	if (set_parking_brake_indicator_output(api, device, desired)) 
	{
		*applied_state = desired;
		log_write("Parking-brake indicator %s", desired ? "on" : "off");
	}
	else
	{
		InterlockedExchange(&g_parking_brake_indicator_update_requested, 1);
		log_write("Unable to update parking-brake indicator on pin %u",	PARKING_BRAKE_INDICATOR_PIN);
	}
}

/*
 * Apply the battery-master backlight state on the PoKeys worker. X-Plane's
 * flight-loop callback only publishes atomics and therefore never calls the
 * PoKeys DLL directly.
 */
static void process_backlight(PokeysApi* api, sPoKeysDevice* device, int* applied_state)
{
	int requested = InterlockedExchange(&g_backlight_update_requested, 0) != 0;
	int desired = InterlockedCompareExchange(&g_backlight, 0, 0) != 0;

	if (!requested && *applied_state == desired) 
		return;
	if (set_backlight_output(api, device, desired)) 
	{
		*applied_state = desired;
		log_write("TQ backlight %s", desired ? "on" : "off");
	}
	else
	{
		InterlockedExchange(&g_backlight_update_requested, 1);
		log_write("Unable to update TQ backlight on pin %u", BACKLIGHT_PIN);
	}
}

static void stop_speedbrake_motor(PokeysApi* api, sPoKeysDevice* device, uint32_t duty_cycles[POKEYS_PWM_CHANNELS])
{
	duty_cycles[SPEEDBRAKE_PWM_CHANNEL] = 0U;
	pwm_update(api, device, duty_cycles, "stopping the speedbrake motor");
	if (!set_logical_output(api, device, SPEEDBRAKE_ENABLE_PIN, 0))
		log_write("Unable to place speedbrake motor in coast state");
}

static int start_speedbrake_motor(PokeysApi* api, sPoKeysDevice* device, uint32_t duty_cycles[POKEYS_PWM_CHANNELS], int extend)
{
	duty_cycles[SPEEDBRAKE_PWM_CHANNEL] = 0U;
	if (!pwm_update(api, device, duty_cycles, "preparing the speedbrake motor") || !set_logical_output(api, device, SPEEDBRAKE_ENABLE_PIN, 1) || !set_logical_output(api, device, SPEEDBRAKE_DIRECTION_PIN, extend ? 1 : 0)) 
	{
		log_write("Unable to prepare speedbrake motor for %s", extend ? "extension" : "retraction/pull-down");
		set_logical_output(api, device, SPEEDBRAKE_ENABLE_PIN, 0);
		return(0);
	}
	duty_cycles[SPEEDBRAKE_PWM_CHANNEL] = extend ? SPEEDBRAKE_EXTEND_DUTY : SPEEDBRAKE_RETRACT_DUTY;
	if (!pwm_update(api, device, duty_cycles, extend ? "extending the speedbrake" : "retracting the speedbrake")) 
	{
		stop_speedbrake_motor(api, device, duty_cycles);
		return(0);
	}
	log_write("Speedbrake motor started: %s", extend ? "push up and full extension" : "retract and pull down");
	return(1);
}

/*
 * Reproduce the original 0..15-unit trim-indicator calibration curve. The
 * resulting values are direct 50 Hz PWM duties for channel 1.
 */
static uint32_t trim_indicator_duty(uint32_t position)
{
	static const uint32_t duty_at_unit[16] = 
	{
		47415U, 44985U, 43466U, 41900U, 40270U, 38511U, 36433U, 35139U,
		33908U, 32533U, 30695U, 29081U, 27786U, 26428U, 25005U, 23439U
	};
	
	float units;
	float fraction;
	int index;

	/* clamp checks */
	if (position < TRIM_POSITION_MIN) 
		position = TRIM_POSITION_MIN;
	if (position > TRIM_POSITION_MAX) 
		position = TRIM_POSITION_MAX;
	
	units = ((float)(position - TRIM_POSITION_MIN) / (float)(TRIM_POSITION_MAX - TRIM_POSITION_MIN)) * 15.0f;
	
	index = (int)units;
	
	if (index >= 15) 
		return(duty_at_unit[15]);

	fraction = units - (float)index;
	return((uint32_t)((float)duty_at_unit[index] + fraction * ((float)duty_at_unit[index + 1] - (float)duty_at_unit[index]) + 0.5f));
}

/* Apply coast, brake or direction using the topology selected in the config. */
static int apply_trim_bridge(PokeysApi* api, sPoKeysDevice* device, int state)
{
	int wiper_motor = g_config.trim_motor_variant != 3U;
	int ok = 1;

	if (wiper_motor) 
	{
		/* V4/Pro: pin 27/31 are the two H-bridge inputs. */
		ok &= set_logical_output(api, device, TRIM_DIRECTION_A_PIN,	state == -1 || state == 0);
		ok &= set_logical_output(api, device, TRIM_DIRECTION_B_PIN,	state == 1 || state == 0);
		ok &= set_logical_output(api, device, TRIM_ENABLE_PIN, state != 2);
	} 
	else
	{
		/* V3: pin 27 selects direction and pin 28 enables/brakes the bridge. */
		ok &= set_logical_output(api, device, TRIM_DIRECTION_A_PIN, state == 1);
		ok &= set_logical_output(api, device, TRIM_DIRECTION_B_PIN, 0);
		ok &= set_logical_output(api, device, TRIM_ENABLE_PIN, state != 2);
	}
	return(ok);
}

static void stop_trim_motor(PokeysApi* api, sPoKeysDevice* device, uint32_t duty_cycles[POKEYS_PWM_CHANNELS], int brake, int* applied_direction)
{
	int bridge_state = brake ? 0 : 2;
	if (duty_cycles[TRIM_MOTOR_PWM_CHANNEL] != 0U) 
	{
		duty_cycles[TRIM_MOTOR_PWM_CHANNEL] = 0U;
		pwm_update(api, device, duty_cycles, brake ? "braking the stabiliser-trim motor" : "coasting the stabiliser-trim motor");
	}
	if (*applied_direction != bridge_state) 
	{
		if (!apply_trim_bridge(api, device, bridge_state)) 
		{
			log_write("Unable to place stabiliser-trim motor in %s state", brake ? "brake" : "coast");
		} 
		else
		{
			*applied_direction = bridge_state;
		}
	}
	if (InterlockedExchange(&g_trim_motor_running, 0) != 0)
		log_write("Stabiliser-trim motor stopped (%s)",	brake ? "brake" : "coast");
}

static void cancel_trim_brake_ramp(TrimBrakeRamp* ramp)
{
	ramp->active = 0;
	ramp->next_step_at = 0;
}

/*
 * Progressively remove motor torque before applying the bridge brake. This
 * ports DC_motorController.RampDownMotor(): subtract five percent PWM every
 * 50 ms and brake when zero is reached. The worker remains responsive during
 * the ramp; a new command cancels it immediately.
 */
static void progressively_brake_trim_motor(PokeysApi* api, sPoKeysDevice* device, uint32_t duty_cycles[POKEYS_PWM_CHANNELS], int* applied_direction, TrimBrakeRamp* ramp, LONG target, int manual_mode)
{
	ULONGLONG now = GetTickCount64();
	uint32_t current_duty = duty_cycles[TRIM_MOTOR_PWM_CHANNEL];
	uint32_t next_duty;

	if (current_duty == 0U)
	{
		cancel_trim_brake_ramp(ramp);
		stop_trim_motor(api, device, duty_cycles, 1, applied_direction);
		return;
	}
	if (!ramp->active)
	{
		ramp->active = 1;
		ramp->next_step_at = now;
		ramp->target_at_start = target;
		ramp->manual_mode = manual_mode;
		log_write("Stabiliser-trim progressive brake started at duty %u", current_duty);
	}
	if (now < ramp->next_step_at) return;

	next_duty = current_duty > TRIM_BRAKE_RAMP_STEP_DUTY ? current_duty - TRIM_BRAKE_RAMP_STEP_DUTY : 0U;
	duty_cycles[TRIM_MOTOR_PWM_CHANNEL] = next_duty;
	if (!pwm_update(api, device, duty_cycles, "ramping down the stabiliser-trim motor"))
	{
		cancel_trim_brake_ramp(ramp);
		stop_trim_motor(api, device, duty_cycles, 0, applied_direction);
		return;
	}
	if (next_duty == 0U)
	{
		cancel_trim_brake_ramp(ramp);
		stop_trim_motor(api, device, duty_cycles, 1, applied_direction);
		return;
	}
	ramp->next_step_at = now + TRIM_BRAKE_RAMP_STEP_MS;
}

/*
 * Closed-loop trim-wheel control. This is the original 50-count dead band,
 * 800-count conservative/full transition and calibrated minimum-speed scheme.
 * Direction changes include the original 100 ms brake interval without ever
 * blocking the connection thread.
 */
static void process_trim_outputs(PokeysApi* api, sPoKeysDevice* device, uint32_t duty_cycles[POKEYS_PWM_CHANNELS], uint32_t current_position, uint32_t* indicator_applied, int* applied_direction, int* pending_direction, ULONGLONG* direction_deadline, TrimBrakeRamp* brake_ramp)
{
	ULONGLONG now = GetTickCount64();
	LONG target = InterlockedCompareExchange(&g_trim_target_position, 0, 0);
	LONG enabled = InterlockedCompareExchange(&g_trim_motor_enabled, 0, 0);
	LONG manual_enabled = InterlockedCompareExchange(&g_trim_manual_enabled, 0, 0);
	LONG manual_direction = InterlockedCompareExchange(&g_trim_manual_direction, 0, 0);
	LONG minimum_speed = InterlockedCompareExchange(&g_trim_min_speed, 0, 0);
	LONG indicator_target = InterlockedCompareExchange(&g_trim_indicator_target_position, 0, 0);
	LONG indicator_simulator_owned = InterlockedCompareExchange(&g_trim_indicator_simulator_owned, 0, 0);
	uint32_t indicator = trim_indicator_duty(indicator_simulator_owned ? (uint32_t)indicator_target : current_position);
	LONG error = 0;
	LONG distance;
	LONG speed_percent;
	int desired_direction;

	if (*indicator_applied != indicator) 
	{
		duty_cycles[TRIM_INDICATOR_PWM_CHANNEL] = indicator;
		if (pwm_update(api, device, duty_cycles, "positioning the stabiliser-trim indicator"))
			*indicator_applied = indicator;
	}

	if (!manual_enabled && !enabled) 
	{
		*pending_direction = 0;
		*direction_deadline = 0;
		cancel_trim_brake_ramp(brake_ramp);
		stop_trim_motor(api, device, duty_cycles, 0, applied_direction);
		return;
	}

	/*
	 * Complete an established ramp even if wheel inertia carries feedback across
	 * the target. A genuinely new manual direction, ownership mode, or A/P target
	 * cancels the ramp so trim response is never delayed.
	 */
	if (brake_ramp->active)
	{
		int manual_mode = manual_enabled != 0;
		int new_command = brake_ramp->manual_mode != manual_mode || (manual_mode ? manual_direction != 0 : target != brake_ramp->target_at_start);
		if (!new_command)
		{
			progressively_brake_trim_motor(api, device, duty_cycles, applied_direction, brake_ramp, target, manual_mode);
			return;
		}
		cancel_trim_brake_ramp(brake_ramp);
	}

	if (manual_enabled)
	{
		/*
		 * Original manual mode: the three-state yoke switch directly selects
		 * motor direction. V3 uses 80%; the V4/Pro wiper bridge uses 60%.
		 */
		desired_direction = manual_direction < 0 ? -1 : (manual_direction > 0 ? 1 : 0);
		distance = desired_direction == 0 ? 0L : 4095L;
		speed_percent = g_config.trim_motor_variant == 3U ? 80L : 60L;
	}
	else
	{
		error = target - (LONG)current_position;
		distance = error < 0 ? -error : error;
		desired_direction = error < 0 ? -1 : 1;
		speed_percent = 0L;
	}
	if ((desired_direction < 0 && current_position <= TRIM_POSITION_MIN) ||	(desired_direction > 0 && current_position >= TRIM_POSITION_MAX)) 
	{
		*pending_direction = 0;
		*direction_deadline = 0;
		cancel_trim_brake_ramp(brake_ramp);
		stop_trim_motor(api, device, duty_cycles, 1, applied_direction);
		return;
	}
	if ((!manual_enabled && distance <= TRIM_POSITION_DEADBAND) || desired_direction == 0) 
	{
		*pending_direction = 0;
		*direction_deadline = 0;
		progressively_brake_trim_motor(api, device, duty_cycles, applied_direction, brake_ramp, target, manual_enabled != 0);
		return;
	}
	cancel_trim_brake_ramp(brake_ramp);

	if (*pending_direction != 0) 
	{
		if (*pending_direction != desired_direction) 
		{
			*pending_direction = desired_direction;
			*direction_deadline = now + TRIM_DIRECTION_BRAKE_MS;
		}
		if (now < *direction_deadline) 
			return;
		if (!apply_trim_bridge(api, device, desired_direction)) 
		{
			log_write("Unable to select stabiliser-trim motor direction");
			*pending_direction = 0;
			cancel_trim_brake_ramp(brake_ramp);
			stop_trim_motor(api, device, duty_cycles, 0, applied_direction);
			return;
		}
		*applied_direction = desired_direction;
		*pending_direction = 0;
		*direction_deadline = 0;
	} 
	else if (*applied_direction != desired_direction) 
	{
		cancel_trim_brake_ramp(brake_ramp);
		stop_trim_motor(api, device, duty_cycles, 1, applied_direction);
		*pending_direction = desired_direction;
		*direction_deadline = now + TRIM_DIRECTION_BRAKE_MS;
		return;
	}

	if (!manual_enabled)
	{
		if (minimum_speed == 0)
			minimum_speed = g_config.trim_motor_variant == 3U ? 50L : 40L;
		if (minimum_speed > 100L) minimum_speed = 100L;
		speed_percent = minimum_speed +	(LONG)((distance < TRIM_CONSERVATIVE_RANGE ? 0.7f : 2.0f) *	(float)distance);
		if (speed_percent > 100L) 
			speed_percent = 100L;
		if (speed_percent < minimum_speed) 
			speed_percent = minimum_speed;
	}

	if (duty_cycles[TRIM_MOTOR_PWM_CHANNEL] != (uint32_t)speed_percent * 5000U) 
	{
		duty_cycles[TRIM_MOTOR_PWM_CHANNEL] = (uint32_t)speed_percent * 5000U;
		if (!pwm_update(api, device, duty_cycles, "driving the stabiliser-trim wheel")) 
		{
			cancel_trim_brake_ramp(brake_ramp);
			stop_trim_motor(api, device, duty_cycles, 0, applied_direction);
			return;
		}
	}
	if (InterlockedExchange(&g_trim_motor_running, 1) == 0)
		log_write("Stabiliser-trim motor started (%s): target %ld current %u direction %s duty %u",	manual_enabled ? "manual command" : "A/P follow", manual_enabled ? -1L : target, current_position, desired_direction < 0 ? "decrease" : "increase",	duty_cycles[TRIM_MOTOR_PWM_CHANNEL]);
}

static void process_actuator_commands(PokeysApi* api, sPoKeysDevice* device, uint32_t duty_cycles[POKEYS_PWM_CHANNELS], ULONGLONG* speedbrake_deadline,	ULONGLONG* parking_brake_deadline)
{
	ULONGLONG now = GetTickCount64();
	LONG speedbrake_command;
	LONG parking_command;

	if (*speedbrake_deadline && now >= *speedbrake_deadline) 
	{
		stop_speedbrake_motor(api, device, duty_cycles);
		*speedbrake_deadline = 0;
		InterlockedExchange(&g_speedbrake_detent_override, 0);
		InterlockedExchange(&g_detent_update_requested, 1);
		log_write("Speedbrake motor stopped after bounded %u ms run", SPEEDBRAKE_RUN_TIME_MS);
	}
	if (*parking_brake_deadline && now >= *parking_brake_deadline) 
	{
		LONG completed_command = InterlockedCompareExchange( &g_parking_brake_active_command, 0, 0);
		duty_cycles[PARKING_BRAKE_PWM_CHANNEL] = 0U;
		if (pwm_update(api, device, duty_cycles, "ending the parking-brake pulse") && completed_command == PARKING_BRAKE_COMMAND_RELEASE) 
		{
			/* A release is confirmed only after its complete bounded pulse. */
			InterlockedExchange(&g_parking_brake_release_complete, 1);
			log_write("Parking-brake interlock release confirmed; interlock retracted");
		}
		InterlockedExchange(&g_parking_brake_active_command, PARKING_BRAKE_COMMAND_NONE);
		*parking_brake_deadline = 0;
		log_write("Parking-brake interlock pulse completed");
	}

	speedbrake_command = InterlockedExchange(&g_speedbrake_command,	SPEEDBRAKE_COMMAND_NONE);
	if (speedbrake_command != SPEEDBRAKE_COMMAND_NONE) 
	{
		int extend = speedbrake_command == SPEEDBRAKE_COMMAND_EXTEND;
		if (*speedbrake_deadline)
			stop_speedbrake_motor(api, device, duty_cycles);
		InterlockedExchange(&g_speedbrake_detent_override, 1);
		InterlockedExchange(&g_detent_update_requested, 1);
		if (apply_flight_detent_state(api, device, 1) && start_speedbrake_motor(api, device, duty_cycles, extend)) 
		{
			*speedbrake_deadline = now + SPEEDBRAKE_RUN_TIME_MS;
		}
		else
		{
			*speedbrake_deadline = 0;
			InterlockedExchange(&g_speedbrake_detent_override, 0);
			InterlockedExchange(&g_detent_update_requested, 1);
		}
	}

	parking_command = InterlockedExchange(&g_parking_brake_command,	PARKING_BRAKE_COMMAND_NONE);
	if (parking_command != PARKING_BRAKE_COMMAND_NONE) 
	{
		uint32_t duty = parking_command == PARKING_BRAKE_COMMAND_SET ? PARKING_BRAKE_SET_DUTY : PARKING_BRAKE_RELEASE_DUTY;
		duty_cycles[PARKING_BRAKE_PWM_CHANNEL] = duty;
		if (pwm_update(api, device, duty_cycles, parking_command == PARKING_BRAKE_COMMAND_SET ?	"setting the parking-brake interlock" :	"releasing the parking-brake interlock")) 
		{
			InterlockedExchange(&g_parking_brake_release_complete, 0);
			InterlockedExchange(&g_parking_brake_active_command, parking_command);
			InterlockedExchange(&g_parking_brake_state,	parking_command == PARKING_BRAKE_COMMAND_SET ? POKEYS_PARKING_BRAKE_SET : POKEYS_PARKING_BRAKE_RELEASED);
			*parking_brake_deadline = now + PARKING_BRAKE_PULSE_MS;
			log_write("Parking-brake interlock %s pulse started (duty %u)", parking_command == PARKING_BRAKE_COMMAND_SET ? "set" : "release", duty);
		}
	}
}

static const char* throttle_test_stage_name(int stage)
{
	switch (stage) 
	{
	case THROTTLE_TEST_LEFT_MEDIUM_OPEN: return "Left throttle: medium to fully open";
	case THROTTLE_TEST_LEFT_MEDIUM_CLOSE: return "Left throttle: medium return to closed";
	case THROTTLE_TEST_LEFT_FAST_OPEN: return "Left throttle: fast to fully open";
	case THROTTLE_TEST_LEFT_FAST_CLOSE: return "Left throttle: fast return to closed";
	case THROTTLE_TEST_RIGHT_MEDIUM_OPEN: return "Right throttle: medium to fully open";
	case THROTTLE_TEST_RIGHT_MEDIUM_CLOSE: return "Right throttle: medium return to closed";
	case THROTTLE_TEST_RIGHT_FAST_OPEN: return "Right throttle: fast to fully open";
	case THROTTLE_TEST_RIGHT_FAST_CLOSE: return "Right throttle: fast return to closed";
	case THROTTLE_TEST_BOTH_FAST_OPEN: return "Both throttles: fast to fully open";
	case THROTTLE_TEST_BOTH_FAST_CLOSE: return "Both throttles: fast return to closed";
	default: return "Throttle test ready";
	}
}

static int throttle_stage_uses_left(int stage)
{
	return(stage <= THROTTLE_TEST_LEFT_FAST_CLOSE || stage >= THROTTLE_TEST_BOTH_FAST_OPEN);
}

static int throttle_stage_uses_right(int stage)
{
	return((stage >= THROTTLE_TEST_RIGHT_MEDIUM_OPEN &&	stage <= THROTTLE_TEST_RIGHT_FAST_CLOSE) ||stage >= THROTTLE_TEST_BOTH_FAST_OPEN);
}

static int throttle_stage_is_opening(int stage)
{
	return((stage & 1) != 0);
}

static void stop_throttle_motors(PokeysApi* api, sPoKeysDevice* device,	uint32_t duty_cycles[POKEYS_PWM_CHANNELS])
{
	duty_cycles[THROTTLE_LEFT_PWM_CHANNEL] = 0U;
	duty_cycles[THROTTLE_RIGHT_PWM_CHANNEL] = 0U;
	pwm_update(api, device, duty_cycles, "stopping the throttle motors");
	if (!set_logical_output(api, device, THROTTLE_LEFT_ENABLE_PIN, 0) || !set_logical_output(api, device, THROTTLE_RIGHT_ENABLE_PIN, 0))
		log_write("Unable to place both throttle motors in coast state");
}

/* Reset one worker-side manual-input detector at a known lever position. */
static void reset_throttle_manual_monitor(ThrottleManualMonitor* monitor, LONG target, LONG position, ULONGLONG now)
{
	memset(monitor, 0, sizeof(*monitor));
	monitor->previous_position = position;
	monitor->previous_target = target;
	monitor->coast_reference = position;
	monitor->coast_started_at = now;
	monitor->previous_direction = 2;
	monitor->initialised = 1;
}

/*
 * Detect pilot intervention without treating commanded motor travel as input.
 * A movement from a settled/coasting position is uncommanded. While powered,
 * intervention is indicated when the lever remains more than 65 counts from
 * target but repeatedly fails to travel in the commanded direction.
 */
static int detect_throttle_manual_override(int index, LONG target, LONG position, int desired_direction, uint32_t requested_duty, ULONGLONG now)
{
	ThrottleManualMonitor* monitor = &g_throttle_manual_monitor[index];
	LONG distance = target - position;
	LONG delta;
	int target_changed;
	int result = 0;

	if (distance < 0) distance = -distance;
	if (!monitor->initialised)
	{
		reset_throttle_manual_monitor(monitor, target, position, now);
		return(0);
	}

	target_changed = target != monitor->previous_target;
	delta = position - monitor->previous_position;
	if (target_changed && monitor->previous_direction == 2)
	{
		/* Simulator-requested motion is never classified as pilot input. */
		monitor->coast_reference = position;
		monitor->coast_started_at = now;
		monitor->motor_error_count = 0;
	}
	else if (monitor->previous_direction == 2 && now - monitor->coast_started_at >= THROTTLE_MANUAL_COAST_GRACE_MS)
	{
		LONG uncommanded = position - monitor->coast_reference;
		if (uncommanded < 0) 
			uncommanded = -uncommanded;
		if (uncommanded > THROTTLE_MANUAL_ERROR_COUNTS)
			result = 1;
	}

	if (desired_direction != 2)
	{
		if (monitor->previous_direction != desired_direction)
		{
			monitor->drive_started_at = now;
			monitor->motor_error_count = 0;
		}
		else if (now - monitor->drive_started_at >= THROTTLE_MANUAL_DRIVE_GRACE_MS && requested_duty > THROTTLE_MANUAL_MIN_DUTY && distance > THROTTLE_MANUAL_ERROR_COUNTS)
		{
			int travelled_as_commanded = desired_direction == 1 ? delta > 0 : delta < 0;
			if (!travelled_as_commanded)
				++monitor->motor_error_count;
			else if (monitor->motor_error_count > 0)
				--monitor->motor_error_count;
			if (monitor->motor_error_count > THROTTLE_MANUAL_ERROR_SAMPLES)
				result = 1;
		}
	}
	else
	{
		if (monitor->previous_direction != 2)
		{
			monitor->coast_reference = position;
			monitor->coast_started_at = now;
		}
		monitor->drive_started_at = 0;
		monitor->motor_error_count = 0;
	}

	monitor->previous_position = position;
	monitor->previous_target = target;
	monitor->previous_direction = desired_direction;
	if (result)
	{
		InterlockedOr(&g_throttle_manual_override_mask, 1L << index);
		InterlockedExchange(&g_throttle_follow_enabled, 0);
		reset_throttle_manual_monitor(monitor, target, position, now);
	}
	return(result);
}

/*
 * Normal A/T lever following, ported from the .NET throttle governor. The
 * lever positions supplied here use the original twelve-sample MinMax filter.
 * A 50-count final deadband prevents bridge reversals caused by ADC noise and
 * the physical drivetrain's overrun at the target.
 *
 * The
 * original selects proportional gains of 0.045, 0.06 and 0.5 for errors below
 * 800, below 1200 and at/above 1200 corrected ADC counts.  Calibrated minimum
 * drive is added and output is capped at the original 50 percent maximum.
 *
 * A direction change first coasts the affected bridge for one 50 ms worker
 * pass, exceeding the original controller's 5 ms throttle reversal delay.
 */
static void process_throttle_follow(PokeysApi* api, sPoKeysDevice* device, uint32_t duty_cycles[POKEYS_PWM_CHANNELS], uint32_t left_position, uint32_t right_position, int* left_direction, int* right_direction)
{
	LONG enabled = InterlockedCompareExchange(&g_throttle_follow_enabled, 0, 0);
	LONG target[2];
	LONG current[2];
	LONG minimum[2];
	LONG* applied[2];
	uint8_t direction_pin[2] = { THROTTLE_LEFT_DIRECTION_PIN, THROTTLE_RIGHT_DIRECTION_PIN };
	uint8_t enable_pin[2] = { THROTTLE_LEFT_ENABLE_PIN, THROTTLE_RIGHT_ENABLE_PIN };
	uint8_t pwm_channel[2] = { THROTTLE_LEFT_PWM_CHANNEL, THROTTLE_RIGHT_PWM_CHANNEL };
	uint32_t requested_duty[2] = { 0U, 0U };
	int desired[2] = { 2, 2 };
	int changed = 0;
	int i;
	ULONGLONG now = GetTickCount64();

	target[0] = InterlockedCompareExchange(&g_throttle_left_target, 0, 0);
	target[1] = InterlockedCompareExchange(&g_throttle_right_target, 0, 0);
	current[0] = (LONG)left_position;
	current[1] = (LONG)right_position;
	minimum[0] = InterlockedCompareExchange(&g_throttle_left_min_speed, 0, 0);
	minimum[1] = InterlockedCompareExchange(&g_throttle_right_min_speed, 0, 0);
	applied[0] = left_direction;
	applied[1] = right_direction;

	if (!enabled || InterlockedCompareExchange(&g_throttle_limits_valid, 0, 0) == 0)
	{
		memset(g_throttle_manual_monitor, 0, sizeof(g_throttle_manual_monitor));
		if (*left_direction != 2 || *right_direction != 2 || duty_cycles[THROTTLE_LEFT_PWM_CHANNEL] != 0U || duty_cycles[THROTTLE_RIGHT_PWM_CHANNEL] != 0U)
		{
			stop_throttle_motors(api, device, duty_cycles);
			*left_direction = 2;
			*right_direction = 2;
			log_write("A/T throttle motor follow stopped; motors coasting");
		}
		return;
	}

	for (i = 0; i < 2; ++i)
	{
		LONG error = target[i] - current[i];
		LONG distance = error < 0 ? -error : error;
		LONG speed_percent;
		float gain;

		if (distance <= THROTTLE_FOLLOW_DEADBAND)
			continue;
		desired[i] = error > 0 ? 1 : 0;
		gain = distance < THROTTLE_CONSERVATIVE_RANGE ? 0.045f : (distance < THROTTLE_MEDIUM_RANGE ? 0.06f : 0.5f);
		speed_percent = minimum[i] + (LONG)(gain * (float)distance);
		if (speed_percent > THROTTLE_MAX_SPEED_PERCENT)
			speed_percent = THROTTLE_MAX_SPEED_PERCENT;
		if (speed_percent <= minimum[i]) 
			speed_percent = minimum[i] + 1L;
		requested_duty[i] = (uint32_t)speed_percent * 5000U;
	}

	/* Detection runs after the governor has selected direction and duty. */
	if (detect_throttle_manual_override(0, target[0], current[0], desired[0], requested_duty[0], now) | detect_throttle_manual_override(1, target[1], current[1], desired[1], requested_duty[1], now))
	{
		stop_throttle_motors(api, device, duty_cycles);
		*left_direction = 2;
		*right_direction = 2;
		log_write("Pilot throttle intervention detected; A/T motors coasting pending simulator disconnect");
		return;
	}

	for (i = 0; i < 2; ++i)
	{
		if (desired[i] != 2 && *applied[i] != 2 &&
			*applied[i] != desired[i])
		{
			/* Coast before changing polarity; resume on the next worker pass. */
			duty_cycles[pwm_channel[i]] = 0U;
			set_logical_output(api, device, enable_pin[i], 0);
			*applied[i] = 2;
			requested_duty[i] = 0U;
			changed = 1;
			continue;
		}
		if (desired[i] == 2)
		{
			if (*applied[i] != 2)
				set_logical_output(api, device, enable_pin[i], 0);
			*applied[i] = 2;
		}
		else if (*applied[i] != desired[i])
		{
			if (!set_logical_output(api, device, direction_pin[i], desired[i]) || !set_logical_output(api, device, enable_pin[i], 1))
			{
				log_write("Unable to prepare %s A/T throttle motor", i == 0 ? "left" : "right");
				requested_duty[i] = 0U;
				desired[i] = 2;
			}
			*applied[i] = desired[i];
		}
		if (duty_cycles[pwm_channel[i]] != requested_duty[i])
		{
			duty_cycles[pwm_channel[i]] = requested_duty[i];
			changed = 1;
		}
	}
	if (changed && !pwm_update(api, device, duty_cycles, "following simulator A/T throttle targets"))
	{
		stop_throttle_motors(api, device, duty_cycles);
		*left_direction = 2;
		*right_direction = 2;
	}
}

static int start_throttle_test_stage(PokeysApi* api, sPoKeysDevice* device, uint32_t duty_cycles[POKEYS_PWM_CHANNELS], int stage)
{
	int use_left = throttle_stage_uses_left(stage);
	int use_right = throttle_stage_uses_right(stage);
	int opening = throttle_stage_is_opening(stage);
	uint32_t duty = ((stage >= THROTTLE_TEST_LEFT_FAST_OPEN && stage <= THROTTLE_TEST_LEFT_FAST_CLOSE) || stage >= THROTTLE_TEST_RIGHT_FAST_OPEN) ? THROTTLE_FAST_DUTY : THROTTLE_MEDIUM_DUTY;

	stop_throttle_motors(api, device, duty_cycles);
	if ((use_left && (!set_logical_output(api, device, THROTTLE_LEFT_DIRECTION_PIN,	opening) || !set_logical_output(api, device, THROTTLE_LEFT_ENABLE_PIN, 1))) || (use_right && (!set_logical_output(api, device, THROTTLE_RIGHT_DIRECTION_PIN, opening) || !set_logical_output(api, device, THROTTLE_RIGHT_ENABLE_PIN, 1)))) 
	{
		log_write("Unable to prepare throttle motors for test stage %d", stage);
		stop_throttle_motors(api, device, duty_cycles);
		return(0);
	}
	if (use_left) duty_cycles[THROTTLE_LEFT_PWM_CHANNEL] = duty;
	if (use_right) duty_cycles[THROTTLE_RIGHT_PWM_CHANNEL] = duty;
	if (!pwm_update(api, device, duty_cycles, throttle_test_stage_name(stage))) 
	{
		stop_throttle_motors(api, device, duty_cycles);
		return(0);
	}
	throttle_test_status_set(throttle_test_stage_name(stage));
	log_write("Throttle test stage %d/10 started: %s (duty %u)", stage,	throttle_test_stage_name(stage), duty);
	return(1);
}

static int throttle_test_endpoint_reached(int stage, uint32_t left_position,
	uint32_t right_position)
{
	uint32_t left_min = (uint32_t)InterlockedCompareExchange(&g_throttle_left_min, 0, 0);
	uint32_t left_max = (uint32_t)InterlockedCompareExchange(&g_throttle_left_max, 0, 0);
	uint32_t right_min = (uint32_t)InterlockedCompareExchange(&g_throttle_right_min, 0, 0);
	uint32_t right_max = (uint32_t)InterlockedCompareExchange(&g_throttle_right_max, 0, 0);
	int opening = throttle_stage_is_opening(stage);
	int left_reached = !throttle_stage_uses_left(stage) || (opening ? left_position + THROTTLE_ENDPOINT_TOLERANCE >= left_max :	left_position <= left_min + THROTTLE_ENDPOINT_TOLERANCE);
	int right_reached = !throttle_stage_uses_right(stage) || (opening ?	right_position + THROTTLE_ENDPOINT_TOLERANCE >= right_max :	right_position <= right_min + THROTTLE_ENDPOINT_TOLERANCE);
	return left_reached && right_reached;
}

static void abort_throttle_test(PokeysApi* api, sPoKeysDevice* device, uint32_t duty_cycles[POKEYS_PWM_CHANNELS], int* stage, ULONGLONG* deadline, const char* reason)
{
	stop_throttle_motors(api, device, duty_cycles);
	*stage = THROTTLE_TEST_IDLE;
	*deadline = 0;
	InterlockedExchange(&g_throttle_test_running, 0);
	throttle_test_status_set(reason);
	log_write("Throttle test aborted: %s", reason);
}

static void process_throttle_test(PokeysApi* api, sPoKeysDevice* device, uint32_t duty_cycles[POKEYS_PWM_CHANNELS], uint32_t left_position,	uint32_t right_position, int* stage, ULONGLONG* deadline)
{
	ULONGLONG now = GetTickCount64();

	if (InterlockedExchange(&g_throttle_test_requested, 0) != 0) 
	{
		if (*stage != THROTTLE_TEST_IDLE) return;
		if (InterlockedCompareExchange(&g_throttle_limits_valid, 0, 0) == 0) 
		{
			throttle_test_status_set("Throttle test unavailable: calibration is invalid");
			return;
		}
		*stage = THROTTLE_TEST_LEFT_MEDIUM_OPEN;
		*deadline = 0;
		InterlockedExchange(&g_throttle_test_running, 1);
		log_write("Full throttle motor test requested");
	}

	if (*stage == THROTTLE_TEST_IDLE) return;
	if (*deadline == 0) 
	{
		if (throttle_test_endpoint_reached(*stage, left_position, right_position)) 
		{
			++*stage;
			if (*stage > THROTTLE_TEST_BOTH_FAST_CLOSE) 
			{
				stop_throttle_motors(api, device, duty_cycles);
				*stage = THROTTLE_TEST_IDLE;
				InterlockedExchange(&g_throttle_test_running, 0);
				throttle_test_status_set("Throttle test completed successfully");
				log_write("Full throttle motor test completed successfully");
				return;
			}
		}
		if (!start_throttle_test_stage(api, device, duty_cycles, *stage)) 
		{
			abort_throttle_test(api, device, duty_cycles, stage, deadline, "Throttle test aborted: unable to start motor stage");
			return;
		}
		*deadline = now + THROTTLE_LEG_TIMEOUT_MS;
		return;
	}

	if (throttle_test_endpoint_reached(*stage, left_position, right_position))
	{
		log_write("Throttle test stage %d/10 reached its calibrated endpoint", *stage);
		stop_throttle_motors(api, device, duty_cycles);
		++*stage;
		*deadline = 0;
		if (*stage > THROTTLE_TEST_BOTH_FAST_CLOSE) 
		{
			*stage = THROTTLE_TEST_IDLE;
			InterlockedExchange(&g_throttle_test_running, 0);
			throttle_test_status_set("Throttle test completed successfully");
			log_write("Full throttle motor test completed successfully");
		}
	} 
	else if (now >= *deadline)
	{
		abort_throttle_test(api, device, duty_cycles, stage, deadline, "Throttle test stopped: calibrated endpoint timeout");
	}
}

static void request_automatic_speedbrake_pull_down(uint32_t position, int motor_running, int* armed)
{
	uint32_t closed = (uint32_t)InterlockedCompareExchange(&g_speedbrake_closed_position, 0, 0);
	uint32_t threshold = closed + SPEEDBRAKE_CLOSED_TOLERANCE;
	uint32_t rearm_threshold;
	if (threshold > 4095U) threshold = 4095U;
	rearm_threshold = threshold < 3995U ? threshold + 100U : 4095U;

	if (position > rearm_threshold) 
	{
		*armed = 1;
	} 
	else if (position <= threshold) 
	{
		if (*armed && !motor_running) 
		{
			InterlockedCompareExchange(&g_speedbrake_command, SPEEDBRAKE_COMMAND_RETRACT, SPEEDBRAKE_COMMAND_NONE);
			log_write("Speedbrake entered closed/stop range; pull-down requested");
		}
		*armed = 0;
	}
}

/**********************************************************************************/
/* PoKeys device connection poller thread                                         */
/**********************************************************************************/
static DWORD WINAPI connection_thread(LPVOID parameter)
{
	PokeysApi api;
	sPoKeysDevice* device = NULL;
	char api_error[128] = "PoKeyslib.dll could not be loaded";
	unsigned int read_failures = 0;
	unsigned int health_counter = 0;
	unsigned int digital_read_failures = 0;
	unsigned int toga_read_failures = 0;
	unsigned int at_disconnect_read_failures = 0;
	unsigned int fuel_cutoff_read_failures = 0;
	unsigned int trim_cutout_read_failures = 0;
	uint32_t duty_cycles[POKEYS_PWM_CHANNELS] = { 0U };
	ULONGLONG speedbrake_deadline = 0;
	ULONGLONG parking_brake_deadline = 0;
	int parking_brake_indicator_applied = -1;
	int backlight_applied = -1;
	int automatic_pull_down_armed = 1;
	int throttle_test_stage = THROTTLE_TEST_IDLE;
	ULONGLONG throttle_test_deadline = 0;
	int throttle_left_direction = 2;
	int throttle_right_direction = 2;
	uint32_t trim_indicator_applied = UINT32_MAX;
	int trim_applied_direction = 2;
	int trim_pending_direction = 0;
	ULONGLONG trim_direction_deadline = 0;
	TrimBrakeRamp trim_brake_ramp;
	MinMaxFeedbackFilter trim_feedback_filter;
	MinMaxFeedbackFilter throttle_left_feedback_filter;
	MinMaxFeedbackFilter throttle_right_feedback_filter;
	(void)parameter;
	minmax_feedback_filter_reset(&trim_feedback_filter);
	minmax_feedback_filter_reset(&throttle_left_feedback_filter);
	minmax_feedback_filter_reset(&throttle_right_feedback_filter);
	memset(&trim_brake_ramp, 0, sizeof(trim_brake_ramp));
	log_write("Pokeys connection thread started");
	status_set_disconnected("Starting PoKeys discovery");
	if (!api_load(&api, api_error, sizeof(api_error)))
	{
		log_write("Pokeys connection thread stopped: DLL load failed");
		status_set_disconnected(api_error);
		return (1);
	}
	while (WaitForSingleObject(g_stop_event, 0) != WAIT_OBJECT_0) 
	{
		if (!device && g_config.search_usb) device = connect_usb(&api);
		if (!device && g_config.search_network) device = connect_network(&api);
		if (device) 
		{
			uint32_t raw[POKEYS_LEVER_COUNT] = { 0 };
			uint32_t throttle_left_position;
			uint32_t throttle_right_position;
			if (WaitForSingleObject(g_stop_event, 50) == WAIT_OBJECT_0) break;
			process_actuator_commands(&api, device, duty_cycles, &speedbrake_deadline, &parking_brake_deadline);
			process_parking_brake_indicator(&api, device, &parking_brake_indicator_applied);
			process_backlight(&api, device, &backlight_applied);
			apply_flight_detent_state(&api, device, 0);
			if (api.analog_get_array(device, raw) == PK_OK)
			{
				uint32_t trim_position = minmax_feedback_filter_add(&trim_feedback_filter, 4095U - (raw[3] > 4095U ? 4095U : raw[3]));
				throttle_left_position = minmax_feedback_filter_add(&throttle_left_feedback_filter,	4095U - (raw[0] > 4095U ? 4095U : raw[0]));
				throttle_right_position = minmax_feedback_filter_add(&throttle_right_feedback_filter, 4095U - (raw[1] > 4095U ? 4095U : raw[1]));
				levers_update(raw);
				process_trim_outputs(&api, device, duty_cycles, trim_position, &trim_indicator_applied, &trim_applied_direction, &trim_pending_direction, &trim_direction_deadline, &trim_brake_ramp);
				process_throttle_test(&api, device, duty_cycles, throttle_left_position, throttle_right_position, &throttle_test_stage, &throttle_test_deadline);
				if (throttle_test_stage == THROTTLE_TEST_IDLE)
					process_throttle_follow(&api, device, duty_cycles, throttle_left_position, throttle_right_position, &throttle_left_direction, &throttle_right_direction);
				else
				{
					/* Test stages own the same bridges and always have priority. */
					throttle_left_direction = 3;
					throttle_right_direction = 3;
				}
				request_automatic_speedbrake_pull_down(4095U - (raw[2] > 4095U ? 4095U : raw[2]), speedbrake_deadline != 0, &automatic_pull_down_armed);
				read_failures = 0;
			}
			else 
			{
				/* Never leave any closed-loop motor powered without feedback. */
				trim_pending_direction = 0;
				trim_direction_deadline = 0;
				cancel_trim_brake_ramp(&trim_brake_ramp);
				throttle_left_direction = 2;
				throttle_right_direction = 2;
				minmax_feedback_filter_reset(&trim_feedback_filter);
				minmax_feedback_filter_reset(&throttle_left_feedback_filter);
				minmax_feedback_filter_reset(&throttle_right_feedback_filter);
				stop_trim_motor(&api, device, duty_cycles, 0, &trim_applied_direction);
				stop_throttle_motors(&api, device, duty_cycles);
				if (++read_failures == 3U) 
				{
					log_write("Three consecutive PoKeys analogue reads failed; lever display is unavailable");
					levers_set_unavailable();
					if (throttle_test_stage != THROTTLE_TEST_IDLE)
						abort_throttle_test(&api, device, duty_cycles, &throttle_test_stage, &throttle_test_deadline, "Throttle test stopped: analogue feedback unavailable");
				}
			}

			{
				uint8_t parking_brake_switch = 0U;
				if (api.digital_io_get_single(device, PARKING_BRAKE_SWITCH_PIN,	&parking_brake_switch) == PK_OK) 
				{
					parking_brake_input_update(parking_brake_switch != 0U);
					digital_read_failures = 0;
				}
				else if (++digital_read_failures == 3U) 
				{
					parking_brake_input_set_unavailable();
					log_write("Three consecutive reads of parking-brake switch pin %u failed", PARKING_BRAKE_SWITCH_PIN);
				}
			}

			{
				uint8_t left_toga = 0U;
				uint8_t right_toga = 0U;
				int32_t left_result = api.digital_io_get_single(device, LEFT_TOGA_SWITCH_PIN, &left_toga);
				int32_t right_result = api.digital_io_get_single(device, RIGHT_TOGA_SWITCH_PIN, &right_toga);

				if (left_result == PK_OK && right_result == PK_OK) 
				{
					toga_inputs_update(left_toga != 0U, right_toga != 0U);
					toga_read_failures = 0;
				}
				else if (++toga_read_failures == 3U) 
				{
					toga_inputs_set_unavailable();
					log_write("Three consecutive reads of TO/GA switch pins %u and %u failed", LEFT_TOGA_SWITCH_PIN, RIGHT_TOGA_SWITCH_PIN);
				}
			}

			{
				uint8_t left_at_disconnect = 0U;
				uint8_t right_at_disconnect = 0U;
				int32_t left_result = api.digital_io_get_single(device, LEFT_AT_DISCONNECT_SWITCH_PIN, &left_at_disconnect);
				int32_t right_result = api.digital_io_get_single(device, RIGHT_AT_DISCONNECT_SWITCH_PIN, &right_at_disconnect);

				if (left_result == PK_OK && right_result == PK_OK) 
				{
					at_disconnect_inputs_update(left_at_disconnect != 0U, right_at_disconnect != 0U);
					at_disconnect_read_failures = 0;
				}
				else if (++at_disconnect_read_failures == 3U) 
				{
					at_disconnect_inputs_set_unavailable();
					log_write("Three consecutive reads of A/T disconnect switch pins %u and %u failed", LEFT_AT_DISCONNECT_SWITCH_PIN, RIGHT_AT_DISCONNECT_SWITCH_PIN);
				}
			}

			{
				uint8_t left_fuel_cutoff = 0U;
				uint8_t right_fuel_cutoff = 0U;
				int32_t left_result = api.digital_io_get_single(device, LEFT_FUEL_CUTOFF_SWITCH_PIN, &left_fuel_cutoff);
				int32_t right_result = api.digital_io_get_single(device, RIGHT_FUEL_CUTOFF_SWITCH_PIN, &right_fuel_cutoff);

				if (left_result == PK_OK && right_result == PK_OK) 
				{
					fuel_cutoff_inputs_update(left_fuel_cutoff != 0U, right_fuel_cutoff != 0U);
					fuel_cutoff_read_failures = 0;
				}
				else if (++fuel_cutoff_read_failures == 3U) 
				{
					fuel_cutoff_inputs_set_unavailable();
					log_write("Three consecutive reads of fuel-cutoff switch pins %u and %u failed", LEFT_FUEL_CUTOFF_SWITCH_PIN, RIGHT_FUEL_CUTOFF_SWITCH_PIN);
				}
			}

			{
				uint8_t electric_normal = 0U;
				uint8_t autopilot_normal = 0U;
				int32_t electric_result = api.digital_io_get_single(device, ELECTRIC_TRIM_NORMAL_SWITCH_PIN, &electric_normal);
				int32_t autopilot_result = api.digital_io_get_single(device, AUTOPILOT_TRIM_NORMAL_SWITCH_PIN, &autopilot_normal);

				if (electric_result == PK_OK && autopilot_result == PK_OK) 
				{
					trim_cutout_inputs_update(electric_normal != 0U, autopilot_normal != 0U);
					trim_cutout_read_failures = 0;
				}
				else if (++trim_cutout_read_failures == 3U) 
				{
					trim_cutout_inputs_set_unavailable();
					log_write("Three consecutive reads of trim cutout switch pins %u and %u failed", ELECTRIC_TRIM_NORMAL_SWITCH_PIN, AUTOPILOT_TRIM_NORMAL_SWITCH_PIN);
				}
			}
			if (++health_counter >= 20U) 
			{
				health_counter = 0;
				if (api.device_data_get(device) == PK_OK) continue;
				log_write("PoKeys connection health check failed; discovery will resume");
				stop_throttle_motors(&api, device, duty_cycles);
				stop_trim_motor(&api, device, duty_cycles, 0, &trim_applied_direction);
				throttle_test_stage = THROTTLE_TEST_IDLE;
				throttle_test_deadline = 0;
				InterlockedExchange(&g_throttle_test_running, 0);
				throttle_test_status_set("Throttle test unavailable: TQ disconnected");
				api.disconnect(device);
				device = NULL;
				minmax_feedback_filter_reset(&trim_feedback_filter);
				minmax_feedback_filter_reset(&throttle_left_feedback_filter);
				minmax_feedback_filter_reset(&throttle_right_feedback_filter);
				read_failures = 0;
				digital_read_failures = 0;
				toga_read_failures = 0;
				at_disconnect_read_failures = 0;
				fuel_cutoff_read_failures = 0;
				trim_cutout_read_failures = 0;
				memset(duty_cycles, 0, sizeof(duty_cycles));
				speedbrake_deadline = 0;
				parking_brake_deadline = 0;
				parking_brake_indicator_applied = -1;
				backlight_applied = -1;
				automatic_pull_down_armed = 1;
				trim_indicator_applied = UINT32_MAX;
				trim_applied_direction = 2;
				trim_pending_direction = 0;
				trim_direction_deadline = 0;
				cancel_trim_brake_ramp(&trim_brake_ramp);
				InterlockedExchange(&g_speedbrake_command, SPEEDBRAKE_COMMAND_NONE);
				InterlockedExchange(&g_parking_brake_command, PARKING_BRAKE_COMMAND_NONE);
				InterlockedExchange(&g_parking_brake_state, POKEYS_PARKING_BRAKE_UNKNOWN);
				InterlockedExchange(&g_parking_brake_active_command, PARKING_BRAKE_COMMAND_NONE);
				InterlockedExchange(&g_parking_brake_release_complete, 0);
				InterlockedExchange(&g_parking_brake_indicator_update_requested, 1);
				InterlockedExchange(&g_backlight_update_requested, 1);
				InterlockedExchange(&g_throttle_test_requested, 0);
				InterlockedExchange(&g_speedbrake_detent_override, 0);
				InterlockedExchange(&g_detent_retracted, 0);
				InterlockedExchange(&g_detent_update_requested, 1);
				levers_set_disconnected();
				parking_brake_input_set_disconnected();
				toga_inputs_set_disconnected();
				at_disconnect_inputs_set_disconnected();
				fuel_cutoff_inputs_set_disconnected();
				trim_cutout_inputs_set_disconnected();
				status_set_disconnected("Connection lost; discovery will retry");
			}
		} 
		else 
		{
			char detail[128];
			snprintf(detail, sizeof(detail), "No matching TQ found; retrying in %u ms",	g_config.retry_delay_ms);
			status_set_disconnected(detail);
			log_write("No matching CFY Pokeys device found; retrying in %u ms", g_config.retry_delay_ms);
			if (WaitForSingleObject(g_stop_event, g_config.retry_delay_ms) == WAIT_OBJECT_0) break;
		}
	}
	if (device) 
	{
		/*
		 * XPluginStop normally waits for a completed release before signalling
		 * this thread. Keep a direct, bounded fallback here so every worker exit
		 * also leaves the physical parking-brake interlock retracted.
		 */
		if (!InterlockedCompareExchange(&g_parking_brake_release_complete, 0, 0)) 
		{
			duty_cycles[PARKING_BRAKE_PWM_CHANNEL] = PARKING_BRAKE_RELEASE_DUTY;
			if (pwm_update(&api, device, duty_cycles, "releasing the parking-brake interlock at shutdown")) 
			{
				Sleep(PARKING_BRAKE_PULSE_MS);
				duty_cycles[PARKING_BRAKE_PWM_CHANNEL] = 0U;
				if (pwm_update(&api, device, duty_cycles,"ending the shutdown parking-brake release pulse")) 
				{
					InterlockedExchange(&g_parking_brake_release_complete, 1);
					InterlockedExchange(&g_parking_brake_state,	POKEYS_PARKING_BRAKE_RELEASED);
					log_write("Parking-brake interlock retracted by shutdown fallback");
				}
			}
		}
		stop_throttle_motors(&api, device, duty_cycles);
		stop_speedbrake_motor(&api, device, duty_cycles);
		stop_trim_motor(&api, device, duty_cycles, 0, &trim_applied_direction);
		duty_cycles[TRIM_INDICATOR_PWM_CHANNEL] = 0U;
		duty_cycles[PARKING_BRAKE_PWM_CHANNEL] = 0U;
		pwm_update(&api, device, duty_cycles, "making actuator outputs safe at shutdown");
		if (!set_parking_brake_indicator_output(&api, device, 0))
			log_write("Unable to switch off parking-brake indicator at shutdown");
		if (!set_backlight_output(&api, device, 0))
			log_write("Unable to switch off TQ backlight at shutdown");
		InterlockedExchange(&g_speedbrake_detent_override, 0);
		InterlockedExchange(&g_aircraft_in_flight, 0);
		InterlockedExchange(&g_calibration_active, 0);
		InterlockedExchange(&g_detent_update_requested, 1);
		apply_flight_detent_state(&api, device, 1);
		api.disconnect(device);
	}
	InterlockedExchange(&g_detent_retracted, 0);
	levers_set_disconnected();
	parking_brake_input_set_disconnected();
	toga_inputs_set_disconnected();
	at_disconnect_inputs_set_disconnected();
	fuel_cutoff_inputs_set_disconnected();
	trim_cutout_inputs_set_disconnected();
	status_set_disconnected("PoKeys connection thread stopped");
	FreeLibrary(api.module);
	log_write("Pokeys connection thread stopped");
	return (0);
}

/**********************************************************************************/
/* start the PoKeys device connection poller thread                               */
/**********************************************************************************/
int pokeys_thread_start(const PluginConfig* config)
{
	if (g_thread) return (1);
	
	g_config = *config;
	
	status_set_disconnected("Waiting for PoKeys connection thread");
	memset(&g_levers, 0, sizeof(g_levers));
	memset(&g_parking_brake_input, 0, sizeof(g_parking_brake_input));
	memset(&g_toga_inputs, 0, sizeof(g_toga_inputs));
	memset(&g_at_disconnect_inputs, 0, sizeof(g_at_disconnect_inputs));
	memset(&g_fuel_cutoff_inputs, 0, sizeof(g_fuel_cutoff_inputs));
	memset(&g_trim_cutout_inputs, 0, sizeof(g_trim_cutout_inputs));
	InterlockedExchange(&g_aircraft_in_flight, 0);
	InterlockedExchange(&g_calibration_active, 0);
	InterlockedExchange(&g_detent_update_requested, 1);
	InterlockedExchange(&g_detent_retracted, 0);
	InterlockedExchange(&g_speedbrake_detent_override, 0);
	InterlockedExchange(&g_speedbrake_command, SPEEDBRAKE_COMMAND_NONE);
	InterlockedExchange(&g_parking_brake_command, PARKING_BRAKE_COMMAND_NONE);
	InterlockedExchange(&g_parking_brake_state, POKEYS_PARKING_BRAKE_UNKNOWN);
	InterlockedExchange(&g_parking_brake_active_command, PARKING_BRAKE_COMMAND_NONE);
	InterlockedExchange(&g_parking_brake_release_complete, 0);
	InterlockedExchange(&g_parking_brake_indicator, 0);
	InterlockedExchange(&g_parking_brake_indicator_update_requested, 1);
	InterlockedExchange(&g_backlight, 0);
	InterlockedExchange(&g_backlight_update_requested, 1);
	InterlockedExchange(&g_trim_target_position, 0);
	InterlockedExchange(&g_trim_motor_enabled, 0);
	InterlockedExchange(&g_trim_manual_enabled, 0);
	InterlockedExchange(&g_trim_manual_direction, 0);
	InterlockedExchange(&g_trim_min_speed, 0);
	InterlockedExchange(&g_trim_motor_running, 0);
	InterlockedExchange(&g_trim_indicator_target_position, 0);
	InterlockedExchange(&g_trim_indicator_simulator_owned, 0);
	InterlockedExchange(&g_throttle_test_requested, 0);
	InterlockedExchange(&g_throttle_test_running, 0);
	InterlockedExchange(&g_throttle_follow_enabled, 0);
	InterlockedExchange(&g_throttle_left_target, 0);
	InterlockedExchange(&g_throttle_right_target, 0);
	InterlockedExchange(&g_throttle_left_min_speed, 0);
	InterlockedExchange(&g_throttle_right_min_speed, 0);
	InterlockedExchange(&g_throttle_manual_override_mask, 0);
	memset(g_throttle_manual_monitor, 0, sizeof(g_throttle_manual_monitor));
	throttle_test_status_set("Throttle test ready");
	g_stop_event = CreateEventA(NULL, TRUE, FALSE, NULL);
	
	if (!g_stop_event) 
		return (0);
	
	g_thread = CreateThread(NULL, 0, connection_thread, NULL, 0, NULL);
	if (!g_thread) 
	{
		CloseHandle(g_stop_event);
		g_stop_event = NULL;
		return (0);
	}
	return (1);
}

/**********************************************************************************/
/* stop the PoKeys device connection poller thread                                */
/**********************************************************************************/
int pokeys_thread_stop(void)
{
	DWORD wait_result;
	int stopped_cleanly = 1;

	InterlockedExchange(&g_trim_motor_enabled, 0);
	InterlockedExchange(&g_trim_manual_enabled, 0);
	InterlockedExchange(&g_trim_manual_direction, 0);
	InterlockedExchange(&g_throttle_follow_enabled, 0);
	if (!g_thread) 
		return(1);
	
	SetEvent(g_stop_event);
	
	wait_result = WaitForSingleObject(g_thread, g_config.discovery_timeout_ms);
	if (wait_result == WAIT_TIMEOUT) 
	{
		log_write("Pokeys discovery did not stop within %u ms; cancelling synchronous I/O",	g_config.discovery_timeout_ms);
		CancelSynchronousIo(g_thread);
		wait_result = WaitForSingleObject(g_thread, 2000);
	}
	if (wait_result != WAIT_OBJECT_0) 
	{
		log_write("Pokeys discovery thread required forced termination (wait result %lu)", wait_result);
		TerminateThread(g_thread, ERROR_TIMEOUT);
		WaitForSingleObject(g_thread, 1000);
		stopped_cleanly = 0;
	}
	CloseHandle(g_thread);
	CloseHandle(g_stop_event);
	g_thread = NULL;
	g_stop_event = NULL;
	return stopped_cleanly;
}

/**********************************************************************************/
/* return PoKeys device connected                                                 */
/**********************************************************************************/
int pokeys_is_connected(void)
{
	return InterlockedCompareExchange(&g_connected, 0, 0) != 0;
}

/**********************************************************************************/
/* copy a coherent device status snapshot for the X-Plane main/UI thread          */
/**********************************************************************************/
void pokeys_get_status(PokeysStatus* status)
{
	if (!status) 
		return;
	AcquireSRWLockShared(&g_status_lock);
	*status = g_status;
	ReleaseSRWLockShared(&g_status_lock);
}

void pokeys_get_lever_positions(PokeysLeverPositions* positions)
{
	if (!positions) 
		return;
	AcquireSRWLockShared(&g_lever_lock);
	*positions = g_levers;
	ReleaseSRWLockShared(&g_lever_lock);
}

void pokeys_get_parking_brake_input(PokeysParkingBrakeInput* input)
{
	if (!input) 
		return;
	AcquireSRWLockShared(&g_parking_brake_input_lock);
	*input = g_parking_brake_input;
	ReleaseSRWLockShared(&g_parking_brake_input_lock);
}

void pokeys_get_toga_inputs(PokeysTogaInputs* inputs)
{
	if (!inputs) 
		return;
	AcquireSRWLockShared(&g_toga_input_lock);
	*inputs = g_toga_inputs;
	ReleaseSRWLockShared(&g_toga_input_lock);
}

void pokeys_get_at_disconnect_inputs(PokeysAtDisconnectInputs* inputs)
{
	if (!inputs) 
		return;
	AcquireSRWLockShared(&g_at_disconnect_input_lock);
	*inputs = g_at_disconnect_inputs;
	ReleaseSRWLockShared(&g_at_disconnect_input_lock);
}

void pokeys_get_fuel_cutoff_inputs(PokeysFuelCutoffInputs* inputs)
{
	if (!inputs) 
		return;
	AcquireSRWLockShared(&g_fuel_cutoff_input_lock);
	*inputs = g_fuel_cutoff_inputs;
	ReleaseSRWLockShared(&g_fuel_cutoff_input_lock);
}

void pokeys_get_trim_cutout_inputs(PokeysTrimCutoutInputs* inputs)
{
	if (!inputs) 
		return;
	AcquireSRWLockShared(&g_trim_cutout_input_lock);
	*inputs = g_trim_cutout_inputs;
	ReleaseSRWLockShared(&g_trim_cutout_input_lock);
}

void pokeys_set_parking_brake_indicator(int illuminated)
{
	LONG value = illuminated ? 1 : 0;
	if (InterlockedExchange(&g_parking_brake_indicator, value) != value)
		InterlockedExchange(&g_parking_brake_indicator_update_requested, 1);
}

void pokeys_set_backlight(int illuminated)
{
	LONG value = illuminated ? 1 : 0;
	if (InterlockedExchange(&g_backlight, value) != value)
		InterlockedExchange(&g_backlight_update_requested, 1);
}

void pokeys_set_aircraft_in_flight(int in_flight)
{
	LONG value = in_flight ? 1 : 0;
	if (InterlockedExchange(&g_aircraft_in_flight, value) != value)
		InterlockedExchange(&g_detent_update_requested, 1);
}

void pokeys_set_calibration_active(int active)
{
	LONG value = active ? 1 : 0;
	if (InterlockedExchange(&g_calibration_active, value) != value)
		InterlockedExchange(&g_detent_update_requested, 1);
}

int pokeys_is_flight_detent_retracted(void)
{
	return InterlockedCompareExchange(&g_detent_retracted, 0, 0) != 0;
}

void pokeys_set_speedbrake_closed_position(uint32_t position)
{
	if (position > 4095U) position = 4095U;
	InterlockedExchange(&g_speedbrake_closed_position, (LONG)position);
}

int pokeys_speedbrake_retract_and_pull_down(void)
{
	if (!pokeys_is_connected()) 
		return(0);
	InterlockedExchange(&g_speedbrake_command, SPEEDBRAKE_COMMAND_RETRACT);
	return(1);
}

int pokeys_speedbrake_push_up_and_extend(void)
{
	if (!pokeys_is_connected()) 
		return(0);
	InterlockedExchange(&g_speedbrake_command, SPEEDBRAKE_COMMAND_EXTEND);
	return(1);
}

int pokeys_parking_brake_interlock_release(void)
{
	if (!pokeys_is_connected()) 
		return(0);
	InterlockedExchange(&g_parking_brake_release_complete, 0);
	InterlockedExchange(&g_parking_brake_command, PARKING_BRAKE_COMMAND_RELEASE);
	return(1);
}

int pokeys_parking_brake_interlock_set(void)
{
	if (!pokeys_is_connected()) 
		return(0);
	InterlockedExchange(&g_parking_brake_release_complete, 0);
	InterlockedExchange(&g_parking_brake_command, PARKING_BRAKE_COMMAND_SET);
	return(1);
}

int pokeys_get_parking_brake_state(void)
{
	if (!pokeys_is_connected()) 
		return(POKEYS_PARKING_BRAKE_UNKNOWN);

	return((int)InterlockedCompareExchange(&g_parking_brake_state, 0, 0));
}

int pokeys_ensure_parking_brake_interlock_retracted(void)
{
	LONG queued_command;
	LONG active_command;

	if (!pokeys_is_connected()) 
		return(0);
	if (InterlockedCompareExchange(&g_parking_brake_release_complete, 0, 0))
		return(1);

	queued_command = InterlockedCompareExchange(&g_parking_brake_command, 0, 0);
	active_command = InterlockedCompareExchange(&g_parking_brake_active_command, 0, 0);

	if (queued_command != PARKING_BRAKE_COMMAND_RELEASE && active_command != PARKING_BRAKE_COMMAND_RELEASE) 
	{
		InterlockedExchange(&g_parking_brake_command, PARKING_BRAKE_COMMAND_RELEASE);
	}
	return(1);
}

int pokeys_parking_brake_interlock_is_retracted(void)
{
	return(pokeys_is_connected() && InterlockedCompareExchange(&g_parking_brake_release_complete, 0, 0) != 0);
}

void pokeys_set_trim_target(uint32_t position, int enabled)
{
	if (position > 4095U) position = 4095U;
	InterlockedExchange(&g_trim_target_position, (LONG)position);
	InterlockedExchange(&g_trim_motor_enabled, enabled ? 1 : 0);
}

void pokeys_set_trim_manual_command(int direction, int enabled)
{
	if (direction < 0) direction = -1;
	else if (direction > 0) direction = 1;
	InterlockedExchange(&g_trim_manual_direction, (LONG)direction);
	InterlockedExchange(&g_trim_manual_enabled, enabled ? 1 : 0);
}

void pokeys_set_trim_indicator_target(uint32_t position, int simulator_owned)
{
	if (position > 4095U) position = 4095U;
	InterlockedExchange(&g_trim_indicator_target_position, (LONG)position);
	InterlockedExchange(&g_trim_indicator_simulator_owned, simulator_owned ? 1 : 0);
}

void pokeys_set_trim_min_speed(uint32_t percent)
{
	if (percent > 100U) percent = 100U;
	InterlockedExchange(&g_trim_min_speed, (LONG)percent);
}

int pokeys_trim_motor_is_running(void)
{
	return((int)InterlockedCompareExchange(&g_trim_motor_running, 0, 0));
}

int pokeys_take_throttle_manual_override(void)
{
	return (int)InterlockedExchange(&g_throttle_manual_override_mask, 0);
}

void pokeys_set_throttle_follow_targets(uint32_t left_position,
	uint32_t right_position, uint32_t left_min_speed,
	uint32_t right_min_speed, int enabled)
{
	if (left_position > 4095U) left_position = 4095U;
	if (right_position > 4095U) right_position = 4095U;
	if (left_min_speed > 49U) left_min_speed = 49U;
	if (right_min_speed > 49U) right_min_speed = 49U;
	InterlockedExchange(&g_throttle_left_target, (LONG)left_position);
	InterlockedExchange(&g_throttle_right_target, (LONG)right_position);
	InterlockedExchange(&g_throttle_left_min_speed, (LONG)left_min_speed);
	InterlockedExchange(&g_throttle_right_min_speed, (LONG)right_min_speed);
	InterlockedExchange(&g_throttle_follow_enabled, enabled ? 1 : 0);
}

void pokeys_set_throttle_test_limits(uint32_t left_min, uint32_t left_max, uint32_t right_min, uint32_t right_max)
{
	int valid = left_max <= 4095U && right_max <= 4095U &&
		left_max > left_min && left_max - left_min >= 100U &&
		right_max > right_min && right_max - right_min >= 100U;
	
	InterlockedExchange(&g_throttle_left_min, (LONG)left_min);
	InterlockedExchange(&g_throttle_left_max, (LONG)left_max);
	InterlockedExchange(&g_throttle_right_min, (LONG)right_min);
	InterlockedExchange(&g_throttle_right_max, (LONG)right_max);
	InterlockedExchange(&g_throttle_limits_valid, valid ? 1 : 0);

	if (!valid)
		throttle_test_status_set("Throttle test unavailable: calibration is invalid");
	else if (!pokeys_is_throttle_test_running())
		throttle_test_status_set("Throttle test ready");
}

int pokeys_start_throttle_test(void)
{
	if (!pokeys_is_connected() || InterlockedCompareExchange(&g_throttle_limits_valid, 0, 0) == 0 || InterlockedCompareExchange(&g_throttle_test_running, 0, 0) != 0)
		return(0);
	if (InterlockedCompareExchange(&g_throttle_test_requested, 1, 0) != 0)
		return(0);
	throttle_test_status_set("Throttle test queued");
	return(1);
}

int pokeys_is_throttle_test_running(void)
{
	return(InterlockedCompareExchange(&g_throttle_test_requested, 0, 0) != 0 || InterlockedCompareExchange(&g_throttle_test_running, 0, 0) != 0);
}

void pokeys_get_throttle_test_status(char* status, uint32_t status_size)
{
	if (!status || status_size == 0U) return;
	AcquireSRWLockShared(&g_throttle_test_status_lock);
	strncpy_s(status, status_size, g_throttle_test_status, _TRUNCATE);
	ReleaseSRWLockShared(&g_throttle_test_status_lock);
}
