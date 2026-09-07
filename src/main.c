/**********************************************************************************/
/* FILE NAME: main.c                                                              */
/*   VERSION: 1.0.2                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQplugin for X-Plane 12.                  */
/**********************************************************************************/


/* standard include files */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <Windows.h>

/* required X-Plane SDK include files */
#include "XPLMDefs.h"
#include "XPLMPlugin.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include "XPLMMenus.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"

/* project include files */
#include "datastructures.h"
#include "acf_dref.h"
#include "aircraft_config.h"
#include "calibration.h"
#include "calibration_window.h"
#include "log.h"
#include "plugin_config.h"
#include "plugin_paths.h"
#include "pokeys_thread.h"
#include "status_window.h"

#ifndef XPLM430
#error This plugin must be compiled against the XPLM 4.3 SDK.
#endif

/* X-Plane version used during aircraft validation */
static int				xPlaneVersion;

/* local variables */
static TqCalibration	g_calibration;
static PluginConfig		g_config;
static int				g_started;
static int				g_enabled;
static int				g_deferred_callback_registered;
static int				g_detent_callback_registered;
static int				g_calibration_required;

enum DeferredInitStage
{
	DEFER_INIT_VALIDATE_AIRCRAFT = 0,
	DEFER_INIT_WAIT_FOR_PARKING_BRAKE_RELEASE
};
static int				g_deferred_init_stage = DEFER_INIT_VALIDATE_AIRCRAFT;

/* state table pointer */
state_table_p			state;

/* system datarefs */
const char				*szAcfTailNumDataRef = "sim/aircraft/view/acf_tailnum";
const char				*szAcfDescriptionDataRef = "sim/aircraft/view/acf_descrip";
const char				*szXplaneVersionDataRef = "sim/version/xplane_internal_version";

XPLMDataRef				drAcfTailNum;																// aircraft tail number
XPLMDataRef				drAcfDescription;															// aircraft description
XPLMDataRef				drXplaneVersion;															// x-plane version number
XPLMDataRef				drOnGround;

/* forward declaration of functions */
bool CheckValidAcf(char* acf_loaded, char* acf_compare);
float DeferredAircraftInitialisation(float elapsedMe, float elapsedSim, int counter, void* refcon);
float UpdateFlightDetentState(float elapsedMe, float elapsedSim, int counter, void* refcon);
static int WaitForParkingBrakeInterlockRelease(DWORD timeout_ms);
static void StopOperationalRuntime(void);

#if IBM
BOOL APIENTRY DllMain(HANDLE module, DWORD reason, LPVOID reserved)
{
	(void)module; (void)reason; (void)reserved;
	return TRUE;
}
#endif

/**********************************************************************************/
/* PLUGIN ENTRY POINT                                                             */
/**********************************************************************************/
PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc)
{
	strcpy(outName, "xpCFY_TQ");
	strcpy(outSig, "simonflightsimulation.com.xpCFY_TQ");
	strcpy(outDesc, "CockpitForYou Motorised Throttle Quadrant Controller");

	/* force use of X-Plane native paths */
	XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

	/* get the plugin paht */
	if (!plugin_paths_initialise()) 
	{
		XPLMDebugString("xpCFY_TQ: unable to determine the plugin directory\n");
		return (0);
	}

	/* get the system datarefs */
	drAcfTailNum = XPLMFindDataRef(szAcfTailNumDataRef);											// get an opaque handle to the aircraft tail number
	drAcfDescription = XPLMFindDataRef(szAcfDescriptionDataRef);									// get an opaque handle to the aircraft description
	drXplaneVersion = XPLMFindDataRef(szXplaneVersionDataRef);										// get a handle to the internal x-plane version number
	drOnGround = XPLMFindDataRef("sim/flightmodel/failures/onground_any");							// get a handle to the on ground state

	/* initialise logging */
	if (!log_initialise())
	{
		XPLMDebugString("xpCFY_TQ: unable to initialise plugin log\n");
		return(0);
	}
	log_write("xpCFY_TQ starting");
	log_write("Plugin directory resolved to %s", plugin_directory());

	/* get the aircraft config - valid tail number etc */
	if (!aircraft_config_initialise())
	{
		log_write("Aircraft configuration file could not be created or opened");
		log_shutdown();
		return(0);
	}

	/* set the config defaults and then read the configuration file */
	plugin_config_defaults(&g_config);
	if (!plugin_config_read(&g_config)) 
	{
		log_write("Configuration missing; writing defaults");
		if (!plugin_config_write(&g_config))
		{
			log_write("Unable to persist default plugin configuration; startup aborted");
			log_shutdown();
			return(0);
		}
	}

	/* create and set the state table */
	state = malloc(sizeof(state_table_t));
	if (state == NULL)
	{
		log_write("Unable to allocate memory for state table");
		log_shutdown();
		return(0);
	}
	
	/* set the initial flags */
	state->blGblSimulatorRunning = false;
	state->blFlightModelActive = false;
	state->blFlcbIsActive = false;
	state->blInitRunning = false;

	/* set the update rate for the data gatherer */
	state->flcbUpdateRate = 1.0f / (float)UPDATE_RATE;

	/* seed the pushbutton state flags */
	state->lt_toga = 0;
	state->lt_toga_prev = 0;
	state->ltTogaIsActive = false;
	state->rt_toga = 0;
	state->rt_toga_prev = 0;
	state->rtTogaIsActive = false;
	state->lt_at_disco = 0;
	state->lt_at_disco_prev = 0;
	state->ltAtDiscoIsActive = false;
	state->rt_at_disco = 0;
	state->rt_at_disco_prev = 0;
	state->rtAtDiscoIsActive = false;

	/* check for TQ calibration */
	tq_calibration_defaults(&g_calibration);
	g_calibration_required = !tq_calibration_read(&g_calibration);
	if (g_calibration_required)
	{
		log_write("Calibration missing or invalid; user calibration is required.");
		tq_calibration_defaults(&g_calibration);
	}
	TqControlsSetCalibration(&g_calibration, !g_calibration_required);

	/* make sure the speedbrake is retracted and pulled down */
	pokeys_set_speedbrake_closed_position(g_calibration.spoiler_min_position);
	
	if (g_calibration_required)
		pokeys_set_throttle_test_limits(0, 0, 0, 0);
	else
		pokeys_set_throttle_test_limits(g_calibration.lever1_min_position, g_calibration.lever1_max_position, g_calibration.lever2_min_position, g_calibration.lever2_max_position);

	/* and start the PoKeys worker thread */
	if (!pokeys_thread_start(&g_config)) 
	{
		log_write("Pokeys connection thread could not be started.");
		free(state);
		state = NULL;
		log_shutdown();
		return(0);
	}

	XPLMRegisterFlightLoopCallback(UpdateFlightDetentState, 0.10f, NULL);
	g_detent_callback_registered = 1;

	if (!status_window_initialise())
	{
		log_write("Status menu/window could not be created");
		XPLMUnregisterFlightLoopCallback(UpdateFlightDetentState, NULL);
		g_detent_callback_registered = 0;
		pokeys_thread_stop();
		free(state);
		state = NULL;
		log_shutdown();
		return(0);
	}
	if (!calibration_window_initialise(&g_calibration, &g_config))
	{
		log_write("Lever positions/calibration window could not be created");
		status_window_shutdown();
		XPLMUnregisterFlightLoopCallback(UpdateFlightDetentState, NULL);
		g_detent_callback_registered = 0;
		pokeys_thread_stop();
		free(state);
		state = NULL;
		log_shutdown();
		return(0);
	}
	if (g_calibration_required) calibration_window_begin(1);

	state->blGblSimulatorRunning = true;
	g_started = 1;
	g_enabled = 1;

	return(1);
}

PLUGIN_API void XPluginStop(void)
{
	if (!g_started) return;

	log_write("xpCFY_TQ plugin stopping.");
	StopOperationalRuntime();
	free(state);
	state = NULL;
	log_write("xpCFY_TQ stopped");
	log_shutdown();
	g_started = 0;
}

PLUGIN_API void XPluginDisable(void) 
{
	if (!g_started || !g_enabled) return;
	log_write("xpCFY_TQ plugin disabled; stopping operational services");
	StopOperationalRuntime();
}

PLUGIN_API int XPluginEnable(void)
{
	if (!g_started) return(0);
	if (g_enabled) return(1);

	if (!pokeys_thread_start(&g_config))
	{
		log_write("Unable to restart PoKeys worker while enabling plugin");
		return(0);
	}
	XPLMRegisterFlightLoopCallback(UpdateFlightDetentState, 0.10f, NULL);
	g_detent_callback_registered = 1;
	if (!status_window_initialise())
	{
		XPLMUnregisterFlightLoopCallback(UpdateFlightDetentState, NULL);
		g_detent_callback_registered = 0;
		pokeys_thread_stop();
		return(0);
	}
	if (!calibration_window_initialise(&g_calibration, &g_config))
	{
		status_window_shutdown();
		XPLMUnregisterFlightLoopCallback(UpdateFlightDetentState, NULL);
		g_detent_callback_registered = 0;
		pokeys_thread_stop();
		return(0);
	}
	if (g_calibration_required) calibration_window_begin(1);
	state->blGblSimulatorRunning = true;
	state->blFlightModelActive = false;
	state->blFlcbIsActive = false;
	state->blInitRunning = true;
	g_deferred_init_stage = DEFER_INIT_VALIDATE_AIRCRAFT;
	XPLMRegisterFlightLoopCallback(DeferredAircraftInitialisation, 0.10f, (void*)state);
	g_deferred_callback_registered = 1;
	g_enabled = 1;
	log_write("xpCFY_TQ plugin enabled; operational services restarted");
	return(1);
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int inMsg, void* inRefcon)
{
	(void)from;
	if (!g_started || !g_enabled || state == NULL) return;

	/* Plane index zero is the user's aircraft; ignore AI aircraft messages. */
	if ((inMsg == XPLM_MSG_PLANE_LOADED ||
		inMsg == XPLM_MSG_PLANE_UNLOADED) && (intptr_t)inRefcon != 0)
		return;

	/*   if a new aircraft is loaded, we need to re-initialise everything   */
	if (inMsg == XPLM_MSG_PLANE_LOADED)
	{
		if (!state->blInitRunning)																	// only run if the initialisation routine is not doing anything
		{
			/*
			 * stop using the preceding aircraft's dataref and command handles immediately.
			 * X-Plane changes the user aircraft before the deferred validation callback,
			 * so leaving its gatherer active here could write through stale refs.
			 */
			ReleaseTqPushbuttonCommands((void*)state);
			UnregisterTqTrimCommandHandlers();
			if (state->blFlcbIsActive)
			{
				XPLMUnregisterFlightLoopCallback(GetAircraftDataFLCB, (void*)state);
				state->blFlcbIsActive = false;
			}

			TqControlsDeactivate();
			state->blFlightModelActive = false;
			g_deferred_init_stage = DEFER_INIT_VALIDATE_AIRCRAFT;
			state->blInitRunning = true;															// set the flag and ...
			if (!g_deferred_callback_registered)
			{
				XPLMRegisterFlightLoopCallback(DeferredAircraftInitialisation, (float)0.10, (void*)state);
				g_deferred_callback_registered = 1;
			}
			else
			{
				XPLMSetFlightLoopCallbackInterval(DeferredAircraftInitialisation, (float)0.10, 1, (void*)state);
			}
		}
	}

	if (inMsg == XPLM_MSG_PLANE_UNLOADED)
	{
		ReleaseTqPushbuttonCommands((void*)state);
		UnregisterTqTrimCommandHandlers();
		pokeys_set_aircraft_in_flight(0);
		if (state->blFlcbIsActive)
		{
			XPLMUnregisterFlightLoopCallback(GetAircraftDataFLCB, (void*)state);
			state->blFlcbIsActive = false;
		}
		TqControlsDeactivate();
		pokeys_ensure_parking_brake_interlock_retracted();
		g_deferred_init_stage = DEFER_INIT_VALIDATE_AIRCRAFT;
		if (g_deferred_callback_registered)
		{
			XPLMUnregisterFlightLoopCallback(DeferredAircraftInitialisation, NULL);
			g_deferred_callback_registered = 0;
		}
		state->blFlightModelActive = false;
		state->blInitRunning = false;
		log_write("Aircraft unloaded; TQ connection remains active.");
	}
}

/*!
 * stop all simulator callbacks and hardware activity when
 * the plugin is stop or disabled by X-Plane.
 */
static void StopOperationalRuntime(void)
{
	if (!g_enabled) return;

	ReleaseTqPushbuttonCommands((void*)state);
	UnregisterTqTrimCommandHandlers();
	if (state && state->blFlcbIsActive)
	{
		XPLMUnregisterFlightLoopCallback(GetAircraftDataFLCB, (void*)state);
		state->blFlcbIsActive = false;
	}
	if (g_deferred_callback_registered)
	{
		XPLMUnregisterFlightLoopCallback(DeferredAircraftInitialisation, NULL);
		g_deferred_callback_registered = 0;
	}
	if (g_detent_callback_registered)
	{
		XPLMUnregisterFlightLoopCallback(UpdateFlightDetentState, NULL);
		g_detent_callback_registered = 0;
	}
	if (state)
	{
		state->blGblSimulatorRunning = false;
		state->blFlightModelActive = false;
		state->blInitRunning = false;
	}
	TqControlsDeactivate();
	status_window_shutdown();
	calibration_window_shutdown();
	if (!WaitForParkingBrakeInterlockRelease(1500U))
		log_write("Parking-brake interlock release was not confirmed before worker shutdown");
	if (!pokeys_thread_stop())
		log_write("Pokeys worker exceeded its normal shutdown interval but completed safe cleanup");
	g_enabled = 0;
}

/*!
 * speedbrake stuff
 */
float UpdateFlightDetentState(float elapsedMe, float elapsedSim, int counter, void* refcon)
{
	int in_flight = 0;
	(void)elapsedMe;
	(void)elapsedSim;
	(void)counter;
	(void)refcon;

	/* keep the lock released while no supported aircraft is active. */
	if (state && state->blFlightModelActive && drOnGround)
		in_flight = XPLMGetDatai(drOnGround) == 0;
	pokeys_set_aircraft_in_flight(in_flight);
	return(0.10f);
}

/*!
 * an aircraft has been loaded, so we can go ahead and start
 * the initialisation properly.
 */
float DeferredAircraftInitialisation(float elapsedMe, float elapsedSim, int counter, void* refcon)
{
	char acfDesc[50];																				// aircraft descriptiom
	char* acfName = "Boeing 737-800X";																// default Zibo filename. anything else is probably Threshold LU

	state_table_p state = (state_table_p)refcon;

	if (g_deferred_init_stage == DEFER_INIT_WAIT_FOR_PARKING_BRAKE_RELEASE)
	{
		/* idempotently queue RELEASE after a late connection/reconnection. */
		pokeys_ensure_parking_brake_interlock_retracted();

		if (!pokeys_parking_brake_interlock_is_retracted())
			return(0.05f);

		/* release is complete; First Run synchronization may now execute. */
		state->blFlcbIsActive = true;
		XPLMRegisterFlightLoopCallback(GetAircraftDataFLCB,	state->flcbUpdateRate, (void*)state);
		state->blInitRunning = false;
		g_deferred_init_stage = DEFER_INIT_VALIDATE_AIRCRAFT;
		log_write("Parking-brake interlock retracted; aircraft data gatherer registered");
		return(0);
	}

	memset(acfDesc, 0, sizeof(acfDesc));
	XPLMGetDatab(drAcfDescription, acfDesc, 0, (int)sizeof(acfDesc) - 1);							// preserve a trailing null
	log_write("Loaded ACF '%s'", acfDesc);

	xPlaneVersion = XPLMGetDatai(drXplaneVersion) / 10000;											// extract the major version number
	if (xPlaneVersion == 12)
	{
		state->blFlightModelActive = CheckValidAcf(acfDesc, acfName);								// and check we've got a valid aircraft
		if (state->blFlightModelActive)																// if the aircraft is valid then...
		{
			if (state->blFlcbIsActive)
			{
				XPLMUnregisterFlightLoopCallback(GetAircraftDataFLCB, (void*)state);
				state->blFlcbIsActive = false;
			}

			/* reset all T/Q controls*/
			TqControlsReset();

			/* get dataref and command handles */
			GetDataRefHandles();																	// load the dataref handles
			GetCommandHandles();																	// load the command handles

			TqControlsSetAircraftActive(1);

			/*
			 * the data gatherer contains First Run synchronization and must not be
			 * registered until a complete physical RELEASE pulse has retracted the
			 * parking-brake interlock.
			 */
			pokeys_ensure_parking_brake_interlock_retracted();
			g_deferred_init_stage = DEFER_INIT_WAIT_FOR_PARKING_BRAKE_RELEASE;
			log_write("Aircraft initialisation waiting for parking-brake interlock release confirmation");
			return(0.05f);
		}
		else
		{
			state->blFlcbIsActive = false;
			TqControlsDeactivate();
		}

		state->blInitRunning = false;																// flag we've finished with the initialisation routines
	}
	state->blInitRunning = false;
	return(0);																						// and put the function to sleep (ie. don't call again until/unless we wake it)
}

static int WaitForParkingBrakeInterlockRelease(DWORD timeout_ms)
{
	ULONGLONG deadline;

	if (!pokeys_ensure_parking_brake_interlock_retracted())
		return(0);
	deadline = GetTickCount64() + timeout_ms;
	do
	{
		if (pokeys_parking_brake_interlock_is_retracted()) 
			return(1);
		Sleep(10U);
	} while (GetTickCount64() < deadline);
	return (pokeys_parking_brake_interlock_is_retracted());
}

bool CheckValidAcf(char* acf_loaded, char* acf_compare)
{
	char tail_number[41] = { 0 };
	int returned_size;

	returned_size = XPLMGetDatab(drAcfTailNum, tail_number, 0, sizeof(tail_number) - 1);

	if (returned_size <= 0 || !aircraft_config_contains_tail_number(tail_number))
	{
		log_write("Unsupported aircraft loaded - Tailnumber is \"%s\". Data services now in standby.", tail_number);
		return(false);
	}

	log_write("Valid aircraft loaded - Tailnumber is \"%s\". Aircraft description \"%s\"/\"%s\"", tail_number, acf_compare, acf_loaded);
	return(true);
}
