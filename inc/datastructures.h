/**********************************************************************************/
/* FILE NAME: datastructures.h                                                    */
/*   VERSION: 1.0.5                                                               */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQplugin for X-Plane 12.                  */
/**********************************************************************************/

#ifndef _DATASTRUCTURES_H_
#define _DATASTRUCTURES_H_

/* standard include files */
#include <stdint.h>
#include <stdbool.h>

/* X-Plane SDK include files */
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

/*
 * Plugin release version. Keep the three numeric components as the single
 * source of truth; XPCFY_TQ_VERSION_STRING is assembled at preprocessing time
 * for UI text and any future diagnostic output.
 */
#define XPCFY_TQ_VERSION_MAJOR 1
#define XPCFY_TQ_VERSION_MINOR 0
#define XPCFY_TQ_VERSION_MICRO 5

#define XPCFY_TQ_STRINGIFY_INNER(value) #value
#define XPCFY_TQ_STRINGIFY(value) XPCFY_TQ_STRINGIFY_INNER(value)

#define XPCFY_TQ_VERSION_STRING XPCFY_TQ_STRINGIFY(XPCFY_TQ_VERSION_MAJOR) "." XPCFY_TQ_STRINGIFY(XPCFY_TQ_VERSION_MINOR) "." XPCFY_TQ_STRINGIFY(XPCFY_TQ_VERSION_MICRO)
#define XPCFY_TQ_COPYRIGHT_STRING "xpCFY_TQ Version " XPCFY_TQ_VERSION_STRING " - Copyright (c) 2026 S.W. Grainger."

#define UPDATE_RATE 100 // FLCB updates per second

/* TQ calibration fields */
typedef struct CalibrationField
{
	const char* name;
	uint32_t* value;
} CalibrationField;

/* TQ calibration data table */
typedef struct TqCalibration
{
	uint32_t lever1_min_speed;
	uint32_t lever2_min_speed;
	uint32_t trim_min_speed;
	uint32_t lever1_min_position;
	uint32_t lever1_max_position;
	uint32_t lever2_min_position;
	uint32_t lever2_max_position;
	uint32_t spoiler_min_position;
	uint32_t spoiler_max_position;
	uint32_t reverser1_min_position;
	uint32_t reverser1_max_position;
	uint32_t reverser2_min_position;
	uint32_t reverser2_max_position;
	uint32_t flaps_min_position;
	uint32_t flaps_max_position;
	char calibration_id[64];
} TqCalibration;

/* plugin configuration data structure */
typedef struct PluginConfig
{
	uint32_t preferred_serial;
	uint32_t network_timeout_ms;
	uint32_t retry_delay_ms;
	uint32_t discovery_timeout_ms;
	uint32_t trim_motor_variant; // 3=V3, 4=V4, 5=Pro
	int search_usb;
	int search_network;
	int network_use_udp; // 0=TCP (default), 1=UDP
	int require_cfy_user_id;
	int enhanced_logging; // 0=normal logging, 1=enhanced diagnostic logging
} PluginConfig;

/* aircraft data */
typedef struct TQAircraftData
{
	float battery_on;
	int paused;
	int on_ground;
	float groundspeed_mps;
	float radio_altitude_m;
	float parking_brake;
	float left_brake;
	float right_brake;
	float throttle_ratio_left;
	float throttle_ratio_right;
	float elevator_trim;
	double speedbrake_armed;
	float speedbrake_lever;
	float at_arm;
	float at_active;
	double ap_engaged;
	float pb_ind_raw;		 // raw indicator lamp value from X-Plane
	int pb_indicator;		 // indicator lamp. 0=off, 1=on
	float ap_trimlock_pos;	 // 0=closed, 1=open
	float ap_trim_pos;		 // 0=normal, 1=cut out
	float el_trimlock_pos;	 // 0=closed, 1=open
	float el_trim_pos;		 // 0=normal, 1=cut out
	float fuel_cutoff_lt;	 // 0=cutoff, 1=idle
	float fuel_cutoff_rt;	 // 0=cutoff, 1=idle
	float flaps_lever;		 // UP=0.000, FL1=0.125, FL2=0.250, FL5=0.375, FL10=0.500, FL15=0.625, FL25=0.750, FL30=0.875, FL40=1.000
	float trim_pos_ca;		 // Captain yoke trim: 0/1=direction, 0.5=released
	float trim_pos_fo;		 // First Officer yoke trim: 0/1=direction, 0.5=released
	float pfd_speed_mode_ca; // PFD speed mode - left flight management computer (captain)
	float pfd_speed_mode_fo; // PFD speed mode - right flight management computer (f/o)
} TQAircraftData;

/*
 * data types used by X-Plane.
 */
#define XP_CHR 1 // char/byte data type
#define XP_INT 2 // integer data type
#define XP_FLT 3 // float data type
#define XP_DBL 4 // double precision data type

/*
 * data union
 */
union XP_DTYPE
{
	unsigned char chrData; // byte data - use XPLMGetDatab and XPLMSetDatab - requires array start pos and len - see SDK
	int intData;		   // integer data - can be single int or part of int array - see SDK
	float fltData;		   // float data - as above
	double dblData;		   // double precision float - not used in internal arrays in X-Plane
};

/*!
 * data table enumeration list
 */
typedef enum
{
	DREF_BATTERY_ON = 0,
	DREF_SIM_PAUSED,
	DREF_ON_GROUND,
	DREF_GND_SPEED,
	DREF_RADIO_ALT,
	DREF_PARKING_BRAKE,
	DREF_LEFT_BRAKE,
	DREF_RIGHT_BRAKE,
	DREF_THROTTLE_RATIO_LT,
	DREF_THROTTLE_RATIO_RT,
	DREF_ELEVATOR_TRIM,
	DREF_SPD_BRAKE_ARM,
	DREF_SPD_BRAKE_LEVER,
	DREF_AUTO_THROTTLE_ARM,
	DREF_AUTO_THROTTLE_ACT,
	DREF_AP_ENGAGED,
	DREF_PB_IND_RAW,
	DREF_AP_TRIMLOCK_POS,
	DREF_AP_TRIM_POS,
	DREF_EL_TRIMLOCK_POS,
	DREF_EL_TRIM_POS,
	DREF_FUEL_CUTOFF_LT,
	DREF_FUEL_CUTOFF_RT,
	DREF_FLAPS_LEVER,
	DREF_TRIM_POS_CA,
	DREF_TRIM_POS_FO,
	DREF_SPD_MODE_CA,
	DREF_SPD_MODE_FO,
	/* end of list */
	DREF_END
} dataRefLine;

/*!
 * dataref table for all datarefs used in Zibo 737.
 * indices are based on enumerates contained in
 * 'datastructures.h'
 *
 * Enumerate order must be the same in all client apps.
 *
 */
struct DREF_TABLE
{
	char* datarefName;	  // linux format dataref string
	XPLMDataRef handle;	  // dataref handle
	uint8_t dataType;	  // X-Plane data type - valid values are XP_CHR, XP_INT, XP_FLT and XP_DBL
	bool isArray;		  // set true if data is held in an array
	int arrayOffset;	  // starting offset into array
	int arrayCount;		  // array counter
	bool isWriteable;	  // set true if dataref is writeable
	bool isEmittable;	  // set true if dataref is transmitted to client side app
	union XP_DTYPE value; // received data value from client side app
	void* ptrVal;		  // pointer to member into aircraft data structure
};
typedef struct DREF_TABLE drefTable_t;
typedef drefTable_t* drefTable_p;

/*!
 * command table enumeration list
 */
typedef enum
{
	CMD_LT_AT_DISCO = 0,
	CMD_RT_AT_DISCO,
	CMD_LT_TOGA,
	CMD_RT_TOGA,
	CMD_EL_TRIM,
	CMD_EL_TRIMLOCK,
	CMD_AP_TRIM,
	CMD_AP_TRIMLOCK,
	CMD_PB_SET,
	/* end of list */
	CMD_END,
} cmdRefLine;

/*!
 * commandref table for all commands used in Zibo 737.
 * indices are based on enumerates contained in
 * 'datastructures.h'
 *
 * Enumerate order must be the same in all client apps.
 */
struct CMD_TABLE
{
	char* commandName;
	XPLMCommandRef handle;
};

/*!
 * the state table contains all the operation states
 * for the simulator
 */
struct STATE_TABLE
{
	/* operational states */
	volatile bool blGblSimulatorRunning; // simulator is running flag!
	volatile bool blFlightModelActive;	 // active flight model loaded!
	volatile bool blFlcbIsActive;		 // flight loop callback active flag
	volatile bool blInitRunning;		 // initialisation of system underway flag!

	/* update rate */
	float flcbUpdateRate; // update rate in milliseconds - derived from 1/UPDATE_RATE

	/* pushbutton state flags */
	bool ltTogaIsActive; // pushbutton is active flag
	int lt_toga;		 // left TO/GA state
	int lt_toga_prev;	 // left TO/GA previous state

	bool rtTogaIsActive; // pushbutton is active flag
	int rt_toga;		 // right TO/GA state
	int rt_toga_prev;	 // right TO/GA previous state

	bool ltAtDiscoIsActive; // pushbutton is active flag
	int lt_at_disco;		// left A/T disconnect state
	int lt_at_disco_prev;	// left A/T disconnect previous state

	bool rtAtDiscoIsActive; // pushbutton is active flag
	int rt_at_disco;		// right A/T disconnect state
	int rt_at_disco_prev;	// right A/T disconnect previous state
};
typedef struct STATE_TABLE state_table_t;
typedef state_table_t* state_table_p;

#endif // !_DATASTRUCTURES_H_
