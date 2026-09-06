/**********************************************************************************/
/* FILE NAME: calibration_window.c                                                */
/*   VERSION: 1.0.2                                                               */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQplugin for X-Plane 12.                  */
/**********************************************************************************/
/* xpCFY_TQ live lever-position and manual calibration window                     */
/**********************************************************************************/

/* standard include files */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <Windows.h>
#include <GL/gl.h>

/* X-Plane SDK include files */
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"

/* project include files */
#include "datastructures.h"
#include "calibration.h"
#include "calibration_window.h"
#include "acf_dref.h"
#include "log.h"
#include "plugin_config.h"
#include "pokeys_thread.h"

#define CAL_WINDOW_WIDTH  850
#define CAL_WINDOW_HEIGHT 540
#define DISPLAYED_LEVERS  6

typedef struct LeverDefinition
{
	const char* name;
	int source_index;
} LeverDefinition;

static const LeverDefinition g_definition[DISPLAYED_LEVERS] = 
{
	{ "Throttle 1", POKEYS_LEVER_THROTTLE_1 },
	{ "Throttle 2", POKEYS_LEVER_THROTTLE_2 },
	{ "Speed brake", POKEYS_LEVER_SPEED_BRAKE },
	{ "Left reverser", POKEYS_LEVER_REVERSER_1 },
	{ "Right reverser", POKEYS_LEVER_REVERSER_2 },
	{ "Flaps", POKEYS_LEVER_FLAPS }
};

static XPLMWindowID g_window;
static TqCalibration* g_calibration;
static PluginConfig* g_config;
static TqCalibration g_working;
static unsigned char g_min_captured[DISPLAYED_LEVERS];
static unsigned char g_max_captured[DISPLAYED_LEVERS];
static int g_calibrating;
static int g_calibration_saved;
static char g_message[160] = "Live raw PoKeys lever positions";

static void draw_text(const char* text, int x, int y, float* colour)
{
	XPLMDrawString(colour, x, y, text, NULL, xplmFont_Proportional);
}

static void draw_centred_text(const char* text, int left, int right, int y, float* colour)
{
	int width = (int)(XPLMMeasureString(xplmFont_Proportional, text, (int)strlen(text)) + 0.5f);
	draw_text(text, left + ((right - left) - width) / 2, y, colour);
}

static void set_open_gl_ui_state(void)
{
	XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

static void draw_filled_rectangle(int left, int bottom, int right, int top,	float red, float green, float blue, float alpha)
{
	set_open_gl_ui_state();
	glColor4f(red, green, blue, alpha);

	/* X-Plane's core OpenGL profile does not guarantee GL_QUADS support. */
	glBegin(GL_TRIANGLES);
	glVertex2i(left, bottom);
	glVertex2i(right, bottom);
	glVertex2i(right, top);
	glVertex2i(left, bottom);
	glVertex2i(right, top);
	glVertex2i(left, top);
	glEnd();
}

/*
 * State buttons must hide the floating window beneath them completely.
 * Supplying alpha 1.0 while GL blending is enabled is not sufficient on every
 * X-Plane rendering backend because the floating-window framebuffer itself is
 * later composited. Disable blending for the fill so both its colour and alpha
 * replace the destination pixels; later drawing helpers restore normal UI
 * blending for outlines and text.
 */
static void draw_opaque_filled_rectangle(int left, int bottom, int right,
	int top, float red, float green, float blue)
{
	XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);
	glDisable(GL_BLEND);
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	glBlendFunc(GL_ONE, GL_ZERO);
	glColor4f(red, green, blue, 1.00f);
	glBegin(GL_TRIANGLES);
	glVertex2i(left, bottom);
	glVertex2i(right, bottom);
	glVertex2i(right, top);
	glVertex2i(left, bottom);
	glVertex2i(right, top);
	glVertex2i(left, top);
	glEnd();
}

static void draw_filled_triangle(int x1, int y1, int x2, int y2, int x3, int y3, float red, float green, float blue, float alpha)
{
	set_open_gl_ui_state();
	glColor4f(red, green, blue, alpha);
	glBegin(GL_TRIANGLES);
	glVertex2i(x1, y1);
	glVertex2i(x2, y2);
	glVertex2i(x3, y3);
	glEnd();
}

static void draw_rectangle_outline(int left, int bottom, int right, int top, float red, float green, float blue, float alpha)
{
	set_open_gl_ui_state();
	glColor4f(red, green, blue, alpha);
	glBegin(GL_LINE_LOOP);
	glVertex2i(left, bottom);
	glVertex2i(right, bottom);
	glVertex2i(right, top);
	glVertex2i(left, top);
	glEnd();
}

static void draw_position_meter(int column_left, int top, uint32_t value, int valid)
{
	const int meter_left = column_left + 33;
	const int meter_right = meter_left + 52;
	const int meter_bottom = top - 263;
	const int meter_top = top - 105;
	const int meter_height = meter_top - meter_bottom;
	const int inner_bottom = meter_bottom + 3;
	const int inner_top = meter_top - 3;
	const int inner_height = inner_top - inner_bottom;
	int fill_top = inner_bottom;
	int tick;

	if (value > 4095U) value = 4095U;
	if (valid)
		fill_top += (int)((uint64_t)value * (uint64_t)inner_height / 4095U);

	draw_filled_rectangle(meter_left, meter_bottom, meter_right, meter_top,	0.055f, 0.075f, 0.095f, 1.00f);
	if (valid && value > 0U)
	{
		if (fill_top <= inner_bottom) fill_top = inner_bottom + 1;
		draw_filled_rectangle(meter_left + 3, inner_bottom, meter_right - 3, fill_top, 0.12f, 0.78f, 0.28f, 1.00f);
	}

	set_open_gl_ui_state();
	glColor4f(0.46f, 0.58f, 0.68f, 0.90f);
	glBegin(GL_LINES);
	for (tick = 0; tick <= 10; ++tick) 
	{
		int y = meter_bottom + tick * meter_height / 10;
		int tick_length = (tick % 5 == 0) ? 10 : 6;
		glVertex2i(meter_left - tick_length, y);
		glVertex2i(meter_left, y);
		glVertex2i(meter_right, y);
		glVertex2i(meter_right + tick_length, y);
	}
	glEnd();
	
	if (valid) 
	{
		draw_filled_triangle(meter_left - 12, fill_top,	meter_left - 2, fill_top + 6, meter_left - 2, fill_top - 6,	1.00f, 0.82f, 0.18f, 1.00f);
		draw_filled_triangle(meter_right + 12, fill_top, meter_right + 2, fill_top + 6, meter_right + 2, fill_top - 6, 1.00f, 0.82f, 0.18f, 1.00f);
	}
	draw_rectangle_outline(meter_left, meter_bottom, meter_right, meter_top, valid ? 0.45f : 0.32f, valid ? 0.92f : 0.36f, valid ? 0.58f : 0.40f, 1.00f);
}

static uint32_t* minimum_value(TqCalibration* value, int lever)
{
	switch (lever)
	{
	case 0: return &value->lever1_min_position;
	case 1: return &value->lever2_min_position;
	case 2: return &value->spoiler_min_position;
	case 3: return &value->reverser1_min_position;
	case 4: return &value->reverser2_min_position;
	default: return &value->flaps_min_position;
	}
}

static uint32_t* maximum_value(TqCalibration* value, int lever)
{
	switch (lever) 
	{
	case 0: return &value->lever1_max_position;
	case 1: return &value->lever2_max_position;
	case 2: return &value->spoiler_max_position;
	case 3: return &value->reverser1_max_position;
	case 4: return &value->reverser2_max_position;
	default: return &value->flaps_max_position;
	}
}

static int all_captured(void)
{
	int lever;
	for (lever = 0; lever < DISPLAYED_LEVERS; ++lever)
		if (!g_min_captured[lever] || !g_max_captured[lever]) 
			return (0);
	return (1);
}

static int calibration_ranges_valid(void)
{
	int lever;
	for (lever = 0; lever < DISPLAYED_LEVERS; ++lever) 
	{
		uint32_t minimum = *minimum_value(&g_working, lever);
		uint32_t maximum = *maximum_value(&g_working, lever);
		if (maximum <= minimum || maximum - minimum < 100U)
			return (0);
	}
	return (1);
}

static void draw_button(const char* label, int left, int bottom, int right, int top, int enabled)
{
	static float enabled_colour[] = { 1.0f, 1.0f, 1.0f };
	static float disabled_colour[] = { 0.45f, 0.48f, 0.52f };
	draw_opaque_filled_rectangle(left, bottom, right, top,
		enabled ? 0.12f : 0.08f, enabled ? 0.25f : 0.10f,
		enabled ? 0.34f : 0.12f);
	draw_rectangle_outline(left, bottom, right, top, enabled ? 0.30f : 0.22f, enabled ? 0.78f : 0.25f, enabled ? 0.96f : 0.28f, 1.00f);
	draw_text(label, left + 14, bottom + 8, enabled ? enabled_colour : disabled_colour);
}

static void draw_state_button(const char* label, int left, int bottom, int right, int top, int enabled, int selected)
{
	static float text_colour[] = { 0.02f, 0.03f, 0.03f };
	static float disabled_text_colour[] = { 0.45f, 0.48f, 0.52f };
	if (!enabled) 
	{
		draw_opaque_filled_rectangle(left, bottom, right, top,
			0.08f, 0.10f, 0.12f);
		draw_rectangle_outline(left, bottom, right, top,
			0.22f, 0.25f, 0.28f, 1.00f);
		draw_text(label, left + 14, bottom + 8, disabled_text_colour);
		return;
	}
	if (selected)
	{
		draw_opaque_filled_rectangle(left, bottom, right, top,
			0.00f, 0.78f, 0.16f);
		draw_rectangle_outline(left, bottom, right, top, 0.45f, 1.00f, 0.55f, 1.00f);
	} 
	else 
	{
		draw_opaque_filled_rectangle(left, bottom, right, top,
			1.00f, 0.58f, 0.00f);
		draw_rectangle_outline(left, bottom, right, top, 1.00f, 0.82f, 0.30f, 1.00f);
	}
	draw_text(label, left + 14, bottom + 8, text_colour);
}

static void draw_calibration_window(XPLMWindowID window, void* refcon)
{
	static float heading_colour[] = { 0.30f, 0.85f, 1.00f };
	static float normal_colour[] = { 1.00f, 1.00f, 1.00f };
	static float active_colour[] = { 0.25f, 1.00f, 0.35f };
	static float warning_colour[] = { 1.00f, 0.65f, 0.20f };
	static float copyright_colour[] = { 0.80f, 0.84f, 0.90f };
	
	PokeysLeverPositions positions;

	char throttle_test_status[128];
	
	int left, top, right, bottom, lever;
	int detent_ready;
	int speedbrake_retracted = 0;
	int speedbrake_extended = 0;
	int parking_brake_state;
	int ground_controls_allowed;
	const char* message;
	(void)refcon;

	pokeys_get_lever_positions(&positions);
	pokeys_get_throttle_test_status(throttle_test_status, (uint32_t)sizeof(throttle_test_status));
	
	detent_ready = !g_calibrating || pokeys_is_flight_detent_retracted();
	message = g_calibrating && !detent_ready ? "Waiting for speedbrake flight-detent lock to retract..." : g_message;
	
	XPLMGetWindowGeometry(window, &left, &top, &right, &bottom);
	
	parking_brake_state = pokeys_get_parking_brake_state();
	ground_controls_allowed = TqGroundTestControlsAllowed();
	
	if (positions.valid && g_calibration) 
	{
		uint32_t speedbrake = positions.value[POKEYS_LEVER_SPEED_BRAKE];
		uint32_t minimum = g_calibration->spoiler_min_position;
		uint32_t maximum = g_calibration->spoiler_max_position;
		speedbrake_retracted = speedbrake <= minimum + 75U;
		speedbrake_extended = speedbrake >= maximum - (maximum >= 75U ? 75U : maximum);
	}
	draw_text(g_calibrating ? "TQ hardware calibration" : "TQ lever positions",	left + 20, top - 35, heading_colour);
	draw_text(message, left + 20, top - 60,	positions.valid && detent_ready ? normal_colour : warning_colour);

	for (lever = 0; lever < DISPLAYED_LEVERS; ++lever) 
	{
		int column_left = left + 18 + lever * 138;
		uint32_t value = positions.value[g_definition[lever].source_index];
		char text[48];
		draw_text(g_definition[lever].name, column_left, top - 88, normal_colour);
		draw_position_meter(column_left, top, value, positions.valid);
		snprintf(text, sizeof(text), "Raw: %4u", positions.valid ? value : 0U);
		draw_text(text, column_left + 12, top - 278, normal_colour);

		if (g_calibrating) 
		{
			draw_button("Set MIN", column_left, top - 329, column_left + 118, top - 300, positions.valid && detent_ready && !g_min_captured[lever]);
			draw_button("Set MAX", column_left, top - 367, column_left + 118, top - 338, positions.valid && detent_ready && !g_max_captured[lever]);
			snprintf(text, sizeof(text), "Min: %4u %s", *minimum_value(&g_working, lever), g_min_captured[lever] ? "OK" : "--");
			draw_text(text, column_left + 4, top - 392, g_min_captured[lever] ? active_colour : normal_colour);
			snprintf(text, sizeof(text), "Max: %4u %s", *maximum_value(&g_working, lever), g_max_captured[lever] ? "OK" : "--");
			draw_text(text, column_left + 4, top - 411, g_max_captured[lever] ? active_colour : normal_colour);
		}
	}

	if (g_calibrating) 
	{
		char protocol_label[48];
		snprintf(protocol_label, sizeof(protocol_label), "PoKeys network: %s",
			g_config && g_config->network_use_udp ? "UDP" : "TCP");
		draw_button(protocol_label, left + 18, top - 460, left + 250,
			top - 426, g_config != NULL);
		draw_button("Save calibration", left + 270, top - 460, left + 420, top - 426, detent_ready && all_captured());
		draw_button("Cancel", left + 438, top - 460, left + 548, top - 426, 1);
	} 
	else if (ground_controls_allowed) 
	{
		draw_state_button("Speedbrake DOWN", left + 18, top - 330, left + 208, top - 296, positions.connected, speedbrake_retracted);
		draw_state_button("Speedbrake UP", left + 220, top - 330, left + 410, top - 296, positions.connected, speedbrake_extended);
		draw_state_button("Park brake RELEASE", left + 422, top - 330, left + 620, top - 296, positions.connected, parking_brake_state == POKEYS_PARKING_BRAKE_RELEASED);
		draw_state_button("Park brake SET", left + 632, top - 330, left + 830, top - 296, positions.connected, parking_brake_state == POKEYS_PARKING_BRAKE_SET);
		draw_button("Test Throttles", left + 300, top - 378, left + 550, top - 344,	positions.connected && !pokeys_is_throttle_test_running());
		draw_text(throttle_test_status, left + 20, top - 402, pokeys_is_throttle_test_running() ? warning_colour : normal_colour);
	} 
	else 
	{
		draw_text("Hardware controls require battery OFF and aircraft on the ground.", left + 20, top - 330, warning_colour);
	}
	if (!g_calibrating)
		draw_button("Close", left + 365, top - 452, left + 485, top - 418, !pokeys_is_throttle_test_running());

	draw_centred_text(XPCFY_TQ_COPYRIGHT_STRING, left, right, top - 505, copyright_colour);
}

static int inside(int x, int y, int left, int bottom, int right, int top)
{
	return x >= left && x <= right && y >= bottom && y <= top;
}

static int handle_mouse(XPLMWindowID window, int x, int y, XPLMMouseStatus mouse, void* refcon)
{
	PokeysLeverPositions positions;
	int left, top, right, bottom, lever;
	(void)refcon;
	if (mouse != xplm_MouseDown) return 1;
	XPLMGetWindowGeometry(window, &left, &top, &right, &bottom);

	if (!g_calibrating) 
	{
		if (TqGroundTestControlsAllowed() && inside(x, y, left + 18, top - 330, left + 208, top - 296)) 
		{
			strcpy_s(g_message, sizeof(g_message), pokeys_speedbrake_retract_and_pull_down() ? "Speedbrake retract/pull-down requested; flight detent released first" :	"Speedbrake command unavailable: TQ is not connected");
		} 
		else if (TqGroundTestControlsAllowed() && inside(x, y, left + 220, top - 330, left + 410, top - 296)) 
		{
			strcpy_s(g_message, sizeof(g_message), pokeys_speedbrake_push_up_and_extend() ?	"Speedbrake push-up/full-extension requested; flight detent released first" : "Speedbrake command unavailable: TQ is not connected");
		} 
		else if (TqGroundTestControlsAllowed() && inside(x, y, left + 422, top - 330, left + 620, top - 296)) 
		{
			strcpy_s(g_message, sizeof(g_message), pokeys_parking_brake_interlock_release() ? "Parking-brake interlock release requested" :	"Parking-brake command unavailable: TQ is not connected");
		} 
		else if (TqGroundTestControlsAllowed() && inside(x, y, left + 632, top - 330, left + 830, top - 296)) 
		{
			strcpy_s(g_message, sizeof(g_message), pokeys_parking_brake_interlock_set() ? "Parking-brake interlock set requested" :	"Parking-brake command unavailable: TQ is not connected");
		} 
		else if (TqGroundTestControlsAllowed() && inside(x, y, left + 300, top - 378, left + 550, top - 344)) 
		{
			strcpy_s(g_message, sizeof(g_message), pokeys_start_throttle_test() ? "Throttle test started; keep the quadrant clear" : "Throttle test unavailable: check connection, calibration, or running test");
		} 
		else if (!pokeys_is_throttle_test_running() && inside(x, y, left + 365, top - 452, left + 485, top - 418)) 
		{
			XPLMSetWindowIsVisible(window, 0);
			g_calibration_saved = 0;
		}
		return (1);
	}

	/*
	 * The network protocol is independent of lever capture and can therefore
	 * be changed while the flight-detent release is still pending. Persist the
	 * complete configuration before asking the worker to reconnect so a write
	 * failure never leaves the running and saved selections inconsistent.
	 */
	if (g_config && inside(x, y, left + 18, top - 460, left + 250,
		top - 426))
	{
		int previous = g_config->network_use_udp;
		g_config->network_use_udp = previous ? 0 : 1;
		if (!plugin_config_write(g_config))
		{
			g_config->network_use_udp = previous;
			strcpy_s(g_message, sizeof(g_message),
				"Unable to save PoKeys protocol; selection was not changed");
			log_write("PoKeys network protocol change was not applied because configuration persistence failed");
		}
		else
		{
			pokeys_set_network_protocol(g_config->network_use_udp);
			snprintf(g_message, sizeof(g_message),
				"PoKeys network protocol saved as %s; network connection will refresh",
				g_config->network_use_udp ? "UDP" : "TCP");
			log_write("PoKeys network protocol changed to %s from calibration window",
				g_config->network_use_udp ? "UDP" : "TCP");
		}
		return (1);
	}

	if (inside(x, y, left + 438, top - 460, left + 548, top - 426)) 
	{
		g_calibrating = 0;
		g_calibration_saved = 0;
		pokeys_set_calibration_active(0);
		strcpy_s(g_message, sizeof(g_message), "Calibration cancelled; values were not changed");
		return (1);
	}

	if (!pokeys_is_flight_detent_retracted()) 
	{
		strcpy_s(g_message, sizeof(g_message), "Calibration blocked until the speedbrake flight-detent lock is retracted");
		return (1);
	}
	
	pokeys_get_lever_positions(&positions);

	for (lever = 0; lever < DISPLAYED_LEVERS; ++lever) 
	{
		int column_left = left + 18 + lever * 138;
		uint32_t value = positions.value[g_definition[lever].source_index];
	
		if (positions.valid && !g_min_captured[lever] && inside(x, y, column_left, top - 329, column_left + 118, top - 300)) 
		{
			*minimum_value(&g_working, lever) = value;
			g_min_captured[lever] = 1;
			snprintf(g_message, sizeof(g_message), "%s minimum captured at %u",	g_definition[lever].name, value);
			return (1);
		}

		if (positions.valid && !g_max_captured[lever] && inside(x, y, column_left, top - 367, column_left + 118, top - 338))
		{
			*maximum_value(&g_working, lever) = value;
			g_max_captured[lever] = 1;
			snprintf(g_message, sizeof(g_message), "%s maximum captured at %u", g_definition[lever].name, value);
			return (1);
		}
	}

	if (inside(x, y, left + 270, top - 460, left + 420, top - 426)) 
	{
		if (!all_captured()) 
		{
			strcpy_s(g_message, sizeof(g_message), "Capture MIN and MAX for every lever before saving");
		}
		else if (!calibration_ranges_valid()) 
		{
			strcpy_s(g_message, sizeof(g_message), "Invalid range: every MAX must exceed MIN by at least 100 counts");
		}
		else 
		{
			strcpy_s(g_working.calibration_id, sizeof(g_working.calibration_id), TQ_CALIBRATION_ID);
			if (tq_calibration_write(&g_working)) 
			{
				*g_calibration = g_working;
				TqControlsSetCalibration(&g_working, 1);
				pokeys_set_speedbrake_closed_position(g_working.spoiler_min_position);
				pokeys_set_throttle_test_limits(g_working.lever1_min_position, g_working.lever1_max_position, g_working.lever2_min_position, g_working.lever2_max_position);
				g_calibrating = 0;
				g_calibration_saved = 1;
				pokeys_set_calibration_active(0);
				strcpy_s(g_message, sizeof(g_message), "Calibration saved successfully. Select Close when finished.");
				log_write("Manual TQ lever calibration completed");
			}
			else 
			{
				strcpy_s(g_message, sizeof(g_message), "Unable to save calibration file; see log");
			}
		}
		return (1);
	}
	return (1);
}

static void handle_key(XPLMWindowID window, char key, XPLMKeyFlags flags, char virtual_key, void* refcon, int losing_focus)
{
	(void)window; (void)key; (void)flags; (void)virtual_key; (void)refcon; (void)losing_focus;
}

static XPLMCursorStatus handle_cursor(XPLMWindowID window, int x, int y, void* refcon)
{
	(void)window; (void)x; (void)y; (void)refcon;
	return (xplm_CursorArrow);
}

static int handle_wheel(XPLMWindowID window, int x, int y, int wheel, int clicks, void* refcon)
{
	(void)window; (void)x; (void)y; (void)wheel; (void)clicks; (void)refcon;
	return (1);
}

int calibration_window_initialise(TqCalibration* calibration, PluginConfig* config)
{
	XPLMCreateWindow_t parameters;
	int screen_left, screen_top, screen_right, screen_bottom;
	g_calibration = calibration;
	g_config = config;
	memset(&parameters, 0, sizeof(parameters));
	XPLMGetScreenBoundsGlobal(&screen_left, &screen_top, &screen_right, &screen_bottom);
	parameters.structSize = sizeof(parameters);
	parameters.left = screen_left + ((screen_right - screen_left) - CAL_WINDOW_WIDTH) / 2;
	parameters.top = screen_top - ((screen_top - screen_bottom) - CAL_WINDOW_HEIGHT) / 2;
	parameters.right = parameters.left + CAL_WINDOW_WIDTH;
	parameters.bottom = parameters.top - CAL_WINDOW_HEIGHT;
	parameters.visible = 0;
	parameters.drawWindowFunc = draw_calibration_window;
	parameters.handleMouseClickFunc = handle_mouse;
	parameters.handleKeyFunc = handle_key;
	parameters.handleCursorFunc = handle_cursor;
	parameters.handleMouseWheelFunc = handle_wheel;
	parameters.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
	parameters.layer = xplm_WindowLayerFloatingWindows;
	parameters.handleRightClickFunc = handle_mouse;
	g_window = XPLMCreateWindowEx(&parameters);

	if (!g_window) 
		return (0);
	
	XPLMSetWindowTitle(g_window, "xpCFY_TQ Lever Positions and Calibration");
	XPLMSetWindowResizingLimits(g_window, CAL_WINDOW_WIDTH, CAL_WINDOW_HEIGHT, CAL_WINDOW_WIDTH, CAL_WINDOW_HEIGHT);
	XPLMSetWindowPositioningMode(g_window, xplm_WindowPositionFree, -1);
	return (1);
}

void calibration_window_shutdown(void)
{
	pokeys_set_calibration_active(0);
	if (g_window) XPLMDestroyWindow(g_window);
	g_window = NULL;
	g_calibration = NULL;
	g_config = NULL;
}

void calibration_window_show_positions(void)
{
	if (!g_window) 
		return;
	pokeys_set_calibration_active(0);
	g_calibrating = 0;
	g_calibration_saved = 0;
	strcpy_s(g_message, sizeof(g_message), "Live raw PoKeys lever positions");
	XPLMSetWindowIsVisible(g_window, 1);
	XPLMBringWindowToFront(g_window);
}

void calibration_window_begin(int automatic_request)
{
	if (!g_window || !g_calibration) return;
	/* The worker confirms the active-low pin 30 release before capture is enabled. */
	pokeys_set_calibration_active(1);
	g_working = *g_calibration;
	memset(g_min_captured, 0, sizeof(g_min_captured));
	memset(g_max_captured, 0, sizeof(g_max_captured));
	g_calibrating = 1;
	g_calibration_saved = 0;
	strcpy_s(g_message, sizeof(g_message), automatic_request ? "Calibration is required. Move each lever to MIN and MAX and capture both." : "Move each lever to MIN and MAX and capture both endpoints.");
	XPLMSetWindowIsVisible(g_window, 1);
	XPLMBringWindowToFront(g_window);
	log_write("Manual TQ calibration requested%s", automatic_request ? " because no valid calibration file was found" : " from the X-Plane menu");
}
