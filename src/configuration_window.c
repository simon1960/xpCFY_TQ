/**********************************************************************************/
/* FILE NAME: configuration_window.c                                              */
/*   VERSION: 1.0.5                                                               */
/*      DATE: 07 SEP 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: xpCFY_TQ general hardware configuration and test window.          */
/**********************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <Windows.h>
#include <GL/gl.h>

#include "XPLMDisplay.h"
#include "XPLMGraphics.h"

#include "acf_dref.h"
#include "configuration_window.h"
#include "datastructures.h"
#include "log.h"
#include "plugin_config.h"
#include "pokeys_thread.h"

#define CONFIG_WINDOW_WIDTH 850
#define CONFIG_WINDOW_HEIGHT 560

static XPLMWindowID g_window;
static PluginConfig* g_config;
static TqCalibration* g_calibration;
static uint32_t g_working_variant;
static int g_working_use_udp;
static int g_working_enhanced_logging;
static int g_message_warning;
static char g_message[180] = "Select the TQ hardware and PoKeys network settings.";

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

/*
 * Fill button backgrounds with horizontal scanlines. On some X-Plane/OpenGL
 * combinations filled immediate-mode polygons are rendered in line mode even
 * though their separately drawn border is correct. GL_LINES is already used
 * reliably by the plugin, so scanlines provide a deterministic solid fill
 * without depending on the host's polygon rasterisation state.
 */
static void draw_opaque_filled_rectangle(int left, int bottom, int right, int top, float red, float green, float blue)
{
	int y;

	set_open_gl_ui_state();
	glColor4f(red, green, blue, 1.0f);
	glLineWidth(1.0f);
	glBegin(GL_LINES);
	for (y = bottom; y <= top; ++y)
	{
		glVertex2i(left, y);
		glVertex2i(right, y);
	}
	glEnd();
}

static void draw_rectangle_outline(int left, int bottom, int right, int top, float red, float green, float blue)
{
	set_open_gl_ui_state();
	glColor4f(red, green, blue, 1.0f);
	glBegin(GL_LINE_LOOP);
	glVertex2i(left, bottom);
	glVertex2i(right, bottom);
	glVertex2i(right, top);
	glVertex2i(left, top);
	glEnd();
}

static void draw_button(const char* label, int left, int bottom, int right, int top, int enabled)
{
	static float text_colour[] = {1.0f, 1.0f, 1.0f};
	draw_opaque_filled_rectangle(left, bottom, right, top, enabled ? 0.12f : 0.08f, enabled ? 0.25f : 0.10f, enabled ? 0.34f : 0.12f);
	draw_rectangle_outline(left, bottom, right, top, enabled ? 0.30f : 0.22f, enabled ? 0.78f : 0.25f, enabled ? 0.96f : 0.28f);
	draw_centred_text(label, left, right, bottom + 8, text_colour);
}

static void draw_state_button(const char* label, int left, int bottom, int right, int top, int enabled, int selected)
{
	static float text_colour[] = {1.0f, 1.0f, 1.0f};
	if (!enabled)
	{
		draw_opaque_filled_rectangle(left, bottom, right, top, 0.08f, 0.10f, 0.12f);
		draw_rectangle_outline(left, bottom, right, top, 0.22f, 0.25f, 0.28f);
		draw_centred_text(label, left, right, bottom + 8, text_colour);
		return;
	}
	if (selected)
	{
		draw_opaque_filled_rectangle(left, bottom, right, top, 0.00f, 0.78f, 0.16f);
		draw_rectangle_outline(left, bottom, right, top, 0.45f, 1.00f, 0.55f);
	}
	else
	{
		draw_opaque_filled_rectangle(left, bottom, right, top, 1.00f, 0.58f, 0.00f);
		draw_rectangle_outline(left, bottom, right, top, 1.00f, 0.82f, 0.30f);
	}
	draw_centred_text(label, left, right, bottom + 8, text_colour);
}

static void draw_tick_box(const char* label, int left, int bottom, int selected)
{
	static float text_colour[] = {1.00f, 1.00f, 1.00f};
	draw_opaque_filled_rectangle(left, bottom, left + 22, bottom + 22, 0.08f, 0.10f, 0.12f);
	draw_rectangle_outline(left, bottom, left + 22, bottom + 22, selected ? 0.30f : 0.45f, selected ? 0.95f : 0.50f, selected ? 0.40f : 0.55f);
	if (selected)
	{
		set_open_gl_ui_state();
		glColor4f(0.30f, 1.00f, 0.40f, 1.00f);
		glLineWidth(3.0f);
		glBegin(GL_LINE_STRIP);
		glVertex2i(left + 4, bottom + 11);
		glVertex2i(left + 9, bottom + 5);
		glVertex2i(left + 19, bottom + 18);
		glEnd();
		glLineWidth(1.0f);
	}
	draw_text(label, left + 32, bottom + 6, text_colour);
}

static void draw_configuration_window(XPLMWindowID window, void* refcon)
{
	static float heading_colour[] = {0.30f, 0.85f, 1.00f};
	static float normal_colour[] = {1.00f, 1.00f, 1.00f};
	static float warning_colour[] = {1.00f, 0.65f, 0.20f};
	static float label_colour[] = {0.70f, 0.75f, 0.82f};
	static float copyright_colour[] = {0.80f, 0.84f, 0.90f};
	PokeysLeverPositions positions;
	char protocol_label[48];
	char throttle_status[128];
	int left, top, right, bottom;
	int tests_allowed;
	int speedbrake_retracted = 0;
	int speedbrake_extended = 0;
	int parking_brake_state;
	(void)refcon;

	pokeys_get_lever_positions(&positions);
	pokeys_get_throttle_test_status(throttle_status, (uint32_t)sizeof(throttle_status));
	tests_allowed = TqGroundTestControlsAllowed();
	parking_brake_state = pokeys_get_parking_brake_state();
	if (positions.valid && g_calibration && g_calibration->spoiler_max_position > g_calibration->spoiler_min_position)
	{
		uint32_t value = positions.value[POKEYS_LEVER_SPEED_BRAKE];
		uint32_t minimum = g_calibration->spoiler_min_position;
		uint32_t maximum = g_calibration->spoiler_max_position;
		/* State colouring uses the same bounded raw-position test as before. */
		speedbrake_retracted = value <= minimum + 75U;
		speedbrake_extended = value >= maximum - (maximum >= 75U ? 75U : maximum);
	}

	XPLMGetWindowGeometry(window, &left, &top, &right, &bottom);
	draw_text("TQ general configuration", left + 20, top - 35, heading_colour);
	draw_text(g_message, left + 20, top - 60, g_message_warning ? warning_colour : normal_colour);

	draw_text("Throttle quadrant variant", left + 20, top - 100, label_colour);
	draw_state_button("CFY TQ Ver 3", left + 20, top - 145, left + 210, top - 111, 1, g_working_variant == 3U);
	draw_state_button("CFY TQ Ver 4", left + 230, top - 145, left + 420, top - 111, 1, g_working_variant == 4U);
	draw_state_button("CFY TQ Ver 4 Pro", left + 440, top - 145, left + 630, top - 111, 1, g_working_variant == 5U);

	draw_text("PoKeys network protocol", left + 20, top - 185, label_colour);
	snprintf(protocol_label, sizeof(protocol_label), "PoKeys network: %s", g_working_use_udp ? "UDP" : "TCP");
	draw_button(protocol_label, left + 20, top - 230, left + 250, top - 196, g_config != NULL);
	draw_tick_box("Enhanced Logging", left + 290, top - 224, g_working_enhanced_logging);

	draw_text("Ground hardware tests", left + 20, top - 270, label_colour);
	if (!tests_allowed)
		draw_text("Controls require battery OFF and aircraft on the ground.", left + 220, top - 270, warning_colour);
	draw_state_button("Speedbrake DOWN", left + 20, top - 315, left + 220, top - 281, tests_allowed && positions.connected, speedbrake_retracted);
	draw_state_button("Speedbrake UP", left + 240, top - 315, left + 440, top - 281, tests_allowed && positions.connected, speedbrake_extended);
	draw_state_button("Park brake RELEASE", left + 20, top - 360, left + 220, top - 326, tests_allowed && positions.connected, parking_brake_state == POKEYS_PARKING_BRAKE_RELEASED);
	draw_state_button("Park brake SET", left + 240, top - 360, left + 440, top - 326, tests_allowed && positions.connected, parking_brake_state == POKEYS_PARKING_BRAKE_SET);
	draw_button("Test Throttles", left + 20, top - 405, left + 270, top - 371, tests_allowed && positions.connected && !pokeys_is_throttle_test_running());
	draw_text(throttle_status, left + 20, top - 430, pokeys_is_throttle_test_running() ? warning_colour : normal_colour);

	draw_button("Save", left + 275, top - 480, left + 395, top - 446, g_config != NULL);
	draw_button("Close", left + 415, top - 480, left + 535, top - 446, !pokeys_is_throttle_test_running());
	draw_centred_text(XPCFY_TQ_COPYRIGHT_STRING, left, right, top - 525, copyright_colour);
}

static int inside(int x, int y, int left, int bottom, int right, int top)
{
	return x >= left && x <= right && y >= bottom && y <= top;
}

static void save_configuration(void)
{
	uint32_t previous_variant;
	int previous_protocol;
	int previous_trim_logging;
	int variant_changed;
	int protocol_changed;
	int trim_logging_changed;

	if (!g_config)
		return;
	previous_variant = g_config->trim_motor_variant;
	previous_protocol = g_config->network_use_udp;
	previous_trim_logging = g_config->enhanced_logging;
	variant_changed = previous_variant != g_working_variant;
	protocol_changed = previous_protocol != g_working_use_udp;
	trim_logging_changed = previous_trim_logging != g_working_enhanced_logging;
	g_config->trim_motor_variant = g_working_variant;
	g_config->network_use_udp = g_working_use_udp;
	g_config->enhanced_logging = g_working_enhanced_logging;
	if (!plugin_config_write(g_config))
	{
		g_config->trim_motor_variant = previous_variant;
		g_config->network_use_udp = previous_protocol;
		g_config->enhanced_logging = previous_trim_logging;
		g_message_warning = 1;
		strcpy_s(g_message, sizeof(g_message), "Unable to save configuration; selections were not applied.");
		log_write("General TQ configuration persistence failed");
		return;
	}

	/* Only publish runtime changes after the complete file is durable. */
	if (protocol_changed)
		pokeys_set_network_protocol(g_working_use_udp);
	if (variant_changed)
	{
		TqControlsSetTrimMotorVariant(g_working_variant);
		pokeys_set_trim_motor_variant(g_working_variant);
	}
	if (trim_logging_changed)
	{
		TqControlsSetEnhancedTrimLogging(g_working_enhanced_logging);
		pokeys_set_enhanced_logging(g_working_enhanced_logging);
	}
	g_message_warning = 0;
	if (variant_changed || protocol_changed || trim_logging_changed)
		strcpy_s(g_message, sizeof(g_message), "Configuration saved; changed hardware settings are being applied.");
	else
		strcpy_s(g_message, sizeof(g_message), "Configuration saved.");
	log_write("General TQ configuration saved: variant V%u, protocol %s, enhanced logging %s", g_working_variant, g_working_use_udp ? "UDP" : "TCP", g_working_enhanced_logging ? "enabled" : "disabled");
}

static int handle_mouse(XPLMWindowID window, int x, int y, XPLMMouseStatus mouse, void* refcon)
{
	int left, top, right, bottom;
	int tests_allowed;
	(void)refcon;
	if (mouse != xplm_MouseDown)
		return 1;
	XPLMGetWindowGeometry(window, &left, &top, &right, &bottom);
	tests_allowed = TqGroundTestControlsAllowed();

	if (inside(x, y, left + 20, top - 145, left + 210, top - 111))
	{
		g_working_variant = 3U;
		g_message_warning = 0;
		strcpy_s(g_message, sizeof(g_message), "CFY TQ V3 selected; select Save to apply.");
	}
	else if (inside(x, y, left + 230, top - 145, left + 420, top - 111))
	{
		g_working_variant = 4U;
		g_message_warning = 0;
		strcpy_s(g_message, sizeof(g_message), "CFY TQ V4 selected; select Save to apply.");
	}
	else if (inside(x, y, left + 440, top - 145, left + 630, top - 111))
	{
		g_working_variant = 5U;
		g_message_warning = 0;
		strcpy_s(g_message, sizeof(g_message), "CFY TQ Pro selected; select Save to apply.");
	}
	else if (g_config && inside(x, y, left + 20, top - 230, left + 250, top - 196))
	{
		g_working_use_udp = !g_working_use_udp;
		g_message_warning = 0;
		snprintf(g_message, sizeof(g_message), "PoKeys %s selected; select Save to apply.", g_working_use_udp ? "UDP" : "TCP");
	}
	else if (g_config && inside(x, y, left + 290, top - 228, left + 520, top - 192))
	{
		g_working_enhanced_logging = !g_working_enhanced_logging;
		g_message_warning = 0;
		snprintf(g_message, sizeof(g_message), "Enhanced logging %s; select Save to apply.", g_working_enhanced_logging ? "enabled" : "disabled");
	}
	else if (tests_allowed && inside(x, y, left + 20, top - 315, left + 220, top - 281))
	{
		strcpy_s(g_message, sizeof(g_message), pokeys_speedbrake_retract_and_pull_down() ? "Speedbrake retract/pull-down requested; flight detent released first." : "Speedbrake command unavailable: TQ is not connected.");
	}
	else if (tests_allowed && inside(x, y, left + 240, top - 315, left + 440, top - 281))
	{
		strcpy_s(g_message, sizeof(g_message), pokeys_speedbrake_push_up_and_extend() ? "Speedbrake push-up/full-extension requested; flight detent released first." : "Speedbrake command unavailable: TQ is not connected.");
	}
	else if (tests_allowed && inside(x, y, left + 20, top - 360, left + 220, top - 326))
	{
		strcpy_s(g_message, sizeof(g_message), pokeys_parking_brake_interlock_release() ? "Parking-brake interlock release requested." : "Parking-brake command unavailable: TQ is not connected.");
	}
	else if (tests_allowed && inside(x, y, left + 240, top - 360, left + 440, top - 326))
	{
		strcpy_s(g_message, sizeof(g_message), pokeys_parking_brake_interlock_set() ? "Parking-brake interlock set requested." : "Parking-brake command unavailable: TQ is not connected.");
	}
	else if (tests_allowed && inside(x, y, left + 20, top - 405, left + 270, top - 371))
	{
		strcpy_s(g_message, sizeof(g_message), pokeys_start_throttle_test() ? "Throttle test started; keep the quadrant clear." : "Throttle test unavailable: check connection, calibration, or running test.");
	}
	else if (inside(x, y, left + 275, top - 480, left + 395, top - 446))
	{
		save_configuration();
	}
	else if (!pokeys_is_throttle_test_running() && inside(x, y, left + 415, top - 480, left + 535, top - 446))
	{
		XPLMSetWindowIsVisible(window, 0);
	}
	return (1);
}

static void handle_key(XPLMWindowID window, char key, XPLMKeyFlags flags, char virtual_key, void* refcon, int losing_focus)
{
	(void)window;
	(void)key;
	(void)flags;
	(void)virtual_key;
	(void)refcon;
	(void)losing_focus;
}

static XPLMCursorStatus handle_cursor(XPLMWindowID window, int x, int y, void* refcon)
{
	(void)window;
	(void)x;
	(void)y;
	(void)refcon;
	return xplm_CursorArrow;
}

static int handle_wheel(XPLMWindowID window, int x, int y, int wheel, int clicks, void* refcon)
{
	(void)window;
	(void)x;
	(void)y;
	(void)wheel;
	(void)clicks;
	(void)refcon;
	return 1;
}

int configuration_window_initialise(PluginConfig* config, TqCalibration* calibration)
{
	XPLMCreateWindow_t parameters;
	int screen_left, screen_top, screen_right, screen_bottom;
	g_config = config;
	g_calibration = calibration;
	if (!g_config || !g_calibration)
		return 0;
	g_working_variant = g_config->trim_motor_variant;
	g_working_use_udp = g_config->network_use_udp;
	g_working_enhanced_logging = g_config->enhanced_logging;

	memset(&parameters, 0, sizeof(parameters));
	XPLMGetScreenBoundsGlobal(&screen_left, &screen_top, &screen_right, &screen_bottom);
	parameters.structSize = sizeof(parameters);
	parameters.left = screen_left + ((screen_right - screen_left) - CONFIG_WINDOW_WIDTH) / 2;
	parameters.top = screen_top - ((screen_top - screen_bottom) - CONFIG_WINDOW_HEIGHT) / 2;
	parameters.right = parameters.left + CONFIG_WINDOW_WIDTH;
	parameters.bottom = parameters.top - CONFIG_WINDOW_HEIGHT;
	parameters.visible = 0;
	parameters.drawWindowFunc = draw_configuration_window;
	parameters.handleMouseClickFunc = handle_mouse;
	parameters.handleKeyFunc = handle_key;
	parameters.handleCursorFunc = handle_cursor;
	parameters.handleMouseWheelFunc = handle_wheel;
	parameters.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
	parameters.layer = xplm_WindowLayerFloatingWindows;
	parameters.handleRightClickFunc = handle_mouse;
	g_window = XPLMCreateWindowEx(&parameters);
	if (!g_window)
		return 0;

	XPLMSetWindowTitle(g_window, "xpCFY_TQ General Configuration");
	XPLMSetWindowResizingLimits(g_window, CONFIG_WINDOW_WIDTH, CONFIG_WINDOW_HEIGHT, CONFIG_WINDOW_WIDTH, CONFIG_WINDOW_HEIGHT);
	XPLMSetWindowPositioningMode(g_window, xplm_WindowPositionFree, -1);
	return 1;
}

void configuration_window_shutdown(void)
{
	if (g_window)
		XPLMDestroyWindow(g_window);
	g_window = NULL;
	g_config = NULL;
	g_calibration = NULL;
}

void configuration_window_show(void)
{
	if (!g_window || !g_config)
		return;
	g_working_variant = g_config->trim_motor_variant;
	g_working_use_udp = g_config->network_use_udp;
	g_working_enhanced_logging = g_config->enhanced_logging;
	g_message_warning = 0;
	strcpy_s(g_message, sizeof(g_message), "Select the TQ hardware and PoKeys network settings.");
	XPLMSetWindowIsVisible(g_window, 1);
	XPLMBringWindowToFront(g_window);
}
