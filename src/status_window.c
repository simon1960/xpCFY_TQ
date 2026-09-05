/**********************************************************************************/
/* FILE NAME: status_window.c                                                     */
/*   VERSION: 1.0                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQplugin for X-Plane 12.                  */
/**********************************************************************************/
/* xpCFY_TQ menu and connection-status window                                     */
/**********************************************************************************/

/* standard include files */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* required X-Plane SDK include files */
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"

/* project include files */
#include "pokeys_thread.h"
#include "calibration_window.h"
#include "status_window.h"

/* local variables */
#define STATUS_WINDOW_WIDTH  440
#define STATUS_WINDOW_HEIGHT 310

static XPLMWindowID     g_status_window;
static XPLMMenuID       g_plugins_menu;
static XPLMMenuID       g_plugin_menu;
static int              g_plugin_menu_item = -1;


static void draw_text(const char* text, int x, int y, float* colour)
{
	XPLMDrawString(colour, x, y, text, NULL, xplmFont_Proportional);
}

static void draw_centred_text(const char* text, int left, int right, int y, float* colour)
{
	int width = (int)(XPLMMeasureString(xplmFont_Proportional, text, (int)strlen(text)) + 0.5f);
	draw_text(text, left + ((right - left) - width) / 2, y, colour);
}

static void draw_field(const char* label, const char* value, int left, int y)
{
	static float label_colour[] = { 0.70f, 0.75f, 0.82f };
	static float value_colour[] = { 1.00f, 1.00f, 1.00f };
	draw_text(label, left, y, label_colour);
	draw_text(value, left + 145, y, value_colour);
}

static void draw_status_window(XPLMWindowID window, void* refcon)
{
	static float connected_colour[] = { 0.25f, 1.00f, 0.35f };
	static float disconnected_colour[] = { 1.00f, 0.35f, 0.25f };
	static float copyright_colour[] = { 0.80f, 0.84f, 0.90f };
	PokeysStatus status;
	char serial_number[32];
	int left, top, right, bottom;
	(void)refcon;

	pokeys_get_status(&status);
	XPLMGetWindowGeometry(window, &left, &top, &right, &bottom);

	draw_text("Connection state:", left + 24, top - 48,
		status.connected ? connected_colour : disconnected_colour);
	draw_text(status.connected ? "CONNECTED" : "DISCONNECTED", left + 169, top - 48,
		status.connected ? connected_colour : disconnected_colour);

	if (status.connected) 
	{
		snprintf(serial_number, sizeof(serial_number), "%u", status.serial_number);
		draw_field("Serial number:", serial_number, left + 24, top - 82);
		draw_field("IP address:", status.ip_address, left + 24, top - 112);
		draw_field("Protocol:", status.protocol, left + 24, top - 142);
		draw_field("Firmware version:", status.firmware_version, left + 24, top - 172);
	}
	else
	{
		draw_field("Serial number:", "Not available", left + 24, top - 82);
		draw_field("IP address:", "Not available", left + 24, top - 112);
		draw_field("Protocol:", "None", left + 24, top - 142);
		draw_field("Firmware version:", "Not available", left + 24, top - 172);
	}

	draw_field("Status detail:", status.detail, left + 24, top - 202);

	draw_centred_text("xpCFY_TQ Version 1.0 - Copyright (c) 2026 S.W. Grainger.", left, right, top - 270, copyright_colour);
}

/**********************************************************************************/
/* DUMMY HANDLER FUNCTIONS                                                        */
/**********************************************************************************/
static int handle_mouse(XPLMWindowID window, int x, int y, XPLMMouseStatus mouse, void* refcon)
{
	(void)window; (void)x; (void)y; (void)mouse; (void)refcon;
	return(1);
}

static void handle_key(XPLMWindowID window, char key, XPLMKeyFlags flags, char virtual_key, void* refcon, int losing_focus)
{
	(void)window; (void)key; (void)flags; (void)virtual_key; (void)refcon; (void)losing_focus;
}

static XPLMCursorStatus handle_cursor(XPLMWindowID window, int x, int y, void* refcon)
{
	(void)window; (void)x; (void)y; (void)refcon;
	return xplm_CursorArrow;
}

static int handle_mouse_wheel(XPLMWindowID window, int x, int y, int wheel,
	int clicks, void* refcon)
{
	(void)window; (void)x; (void)y; (void)wheel; (void)clicks; (void)refcon;
	return(1);
}

/**********************************************************************************/
/* MENU HANDLER, WINDOW INIT & SHUTDOWN FUNCTIONS                                 */
/**********************************************************************************/
static void menu_handler(void* menu_ref, void* item_ref)
{
	uintptr_t item = (uintptr_t)item_ref;
	(void)menu_ref;
	if (item == 2U) 
	{
		calibration_window_show_positions();
	}
	else if (item == 3U) 
	{
		calibration_window_begin(0);
	}
	else if (g_status_window) 
	{
		XPLMSetWindowIsVisible(g_status_window, 1);
		XPLMBringWindowToFront(g_status_window);
	}
}

int status_window_initialise(void)
{
	XPLMCreateWindow_t parameters;
	int screen_left, screen_top, screen_right, screen_bottom;

	memset(&parameters, 0, sizeof(parameters));
	XPLMGetScreenBoundsGlobal(&screen_left, &screen_top, &screen_right, &screen_bottom);
	parameters.structSize = sizeof(parameters);
	parameters.left = screen_left +	((screen_right - screen_left) - STATUS_WINDOW_WIDTH) / 2;
	parameters.top = screen_top - ((screen_top - screen_bottom) - STATUS_WINDOW_HEIGHT) / 2;
	parameters.right = parameters.left + STATUS_WINDOW_WIDTH;
	parameters.bottom = parameters.top - STATUS_WINDOW_HEIGHT;
	parameters.visible = 0;
	parameters.drawWindowFunc = draw_status_window;
	parameters.handleMouseClickFunc = handle_mouse;
	parameters.handleKeyFunc = handle_key;
	parameters.handleCursorFunc = handle_cursor;
	parameters.handleMouseWheelFunc = handle_mouse_wheel;
	parameters.refcon = NULL;
	parameters.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
	parameters.layer = xplm_WindowLayerFloatingWindows;
	parameters.handleRightClickFunc = handle_mouse;

	g_status_window = XPLMCreateWindowEx(&parameters);
	if (!g_status_window) return 0;
	XPLMSetWindowTitle(g_status_window, "xpCFY_TQ Status");
	XPLMSetWindowResizingLimits(g_status_window, STATUS_WINDOW_WIDTH, STATUS_WINDOW_HEIGHT, STATUS_WINDOW_WIDTH, STATUS_WINDOW_HEIGHT);
	XPLMSetWindowPositioningMode(g_status_window, xplm_WindowPositionFree, -1);

	g_plugins_menu = XPLMFindPluginsMenu();
	if (!g_plugins_menu) 
	{
		status_window_shutdown();
		return(0);
	}
	g_plugin_menu_item = XPLMAppendMenuItem(g_plugins_menu, "xpCFY_TQ", NULL, 0);
	if (g_plugin_menu_item < 0) 
	{
		status_window_shutdown();
		return(0);
	}
	g_plugin_menu = XPLMCreateMenu("xpCFY_TQ", g_plugins_menu, g_plugin_menu_item, menu_handler, NULL);
	if (!g_plugin_menu) 
	{
		status_window_shutdown();
		return(0);
	}
	XPLMAppendMenuItem(g_plugin_menu, "TQ Connection Status...", (void*)(uintptr_t)1, 0);
	XPLMAppendMenuItem(g_plugin_menu, "Lever Positions...", (void*)(uintptr_t)2, 0);
	XPLMAppendMenuSeparator(g_plugin_menu);
	XPLMAppendMenuItem(g_plugin_menu, "TQ Calibration...", (void*)(uintptr_t)3, 0);
	return(1);
}

void status_window_shutdown(void)
{
	if (g_plugin_menu) {
		XPLMDestroyMenu(g_plugin_menu);
		g_plugin_menu = NULL;
	}
	if (g_plugins_menu && g_plugin_menu_item >= 0) {
		XPLMRemoveMenuItem(g_plugins_menu, g_plugin_menu_item);
	}
	g_plugins_menu = NULL;
	g_plugin_menu_item = -1;
	if (g_status_window) {
		XPLMDestroyWindow(g_status_window);
		g_status_window = NULL;
	}
}
