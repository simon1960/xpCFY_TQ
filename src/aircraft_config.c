/**********************************************************************************/
/* FILE NAME: aircraft_config.c                                                   */
/*   VERSION: 1.0.4                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQ plugin for X-Plane 12.                 */
/**********************************************************************************/

/* standard include files */
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <Windows.h>

/* project include files */
#include "log.h"
#include "aircraft_config.h"
#include "plugin_paths.h"

#define AIRCRAFT_CONFIG_FILENAME "xpCFY_TQ_Acf.conf"

static char g_aircraft_config_path[MAX_PATH];

static char* trim(char* text)
{
	char* end;
	while (*text && isspace((unsigned char)*text)) ++text;
	end = text + strlen(text);
	while (end > text && isspace((unsigned char)end[-1])) *--end = '\0';
	return text;
}

static int write_default_config(void)
{
	char temporary[MAX_PATH];
	FILE* stream;
	int count, write_ok, close_ok;

	count = snprintf(temporary, sizeof(temporary), "%s.tmp", g_aircraft_config_path);
	if (count <= 0 || (size_t)count >= sizeof(temporary) ||
		fopen_s(&stream, temporary, "w") != 0) {
		log_write("Unable to create temporary aircraft configuration file");
		return(0);
	}

	if (fprintf(stream,
		"# Simon's Flight Simulation Avionics\n"
		"# Valid aircraft tail numbers for xpCFY_TQ; one per line\n"
		"ZB738\n"
		"B736\n"
		"B737\n"
		"B738\n"
		"B739\n"
		) < 0)
	{
		fclose(stream);
		DeleteFileA(temporary);
		log_write("Unable to write aircraft configuration defaults");
		return(0);
	}

	write_ok = fflush(stream) == 0 && !ferror(stream);
	close_ok = fclose(stream) == 0;
	if (!write_ok || !close_ok || !MoveFileExA(temporary, g_aircraft_config_path, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) 
	{
		DeleteFileA(temporary);
		log_write("Unable to persist aircraft configuration file %s (error %lu)", g_aircraft_config_path, GetLastError());
		return(0);
	}
	log_write("Aircraft configuration created at %s", g_aircraft_config_path);
	return(1);
}

int aircraft_config_initialise(void)
{
	DWORD attributes;
	FILE* stream;
	if (!plugin_file_path(g_aircraft_config_path, sizeof(g_aircraft_config_path),
		AIRCRAFT_CONFIG_FILENAME)) {
		return(0);
	}
	attributes = GetFileAttributesA(g_aircraft_config_path);
	if (attributes == INVALID_FILE_ATTRIBUTES) return write_default_config();
	if (attributes & FILE_ATTRIBUTE_DIRECTORY) 
	{
		log_write("Aircraft configuration path refers to a directory: %s", g_aircraft_config_path);
		return(0);
	}
	if (fopen_s(&stream, g_aircraft_config_path, "r") != 0) 
	{
		log_write("Aircraft configuration is not readable: %s", g_aircraft_config_path);
		return(0);
	}
	fclose(stream);
	log_write("Aircraft configuration loaded from %s", g_aircraft_config_path);
	return(1);
}

bool aircraft_config_contains_tail_number(const char* tail_number)
{
	char buffer[128];
	FILE* stream;

	if (!tail_number || !*tail_number || fopen_s(&stream, g_aircraft_config_path, "r") != 0) 
	{
		log_write("Unable to read aircraft configuration file %s", g_aircraft_config_path);
		return(false);
	}

	while (fgets(buffer, sizeof(buffer), stream)) 
	{
		char* entry = trim(buffer);
		size_t length;
		if (!*entry || *entry == '#' || *entry == ';') continue;
		length = strlen(entry);
		if (length >= 2 && entry[0] == ':' && entry[length - 1] == ':') 
		{
			entry[length - 1] = '\0';
			++entry;
		}
		if (strcmp(entry, tail_number) == 0) {
			fclose(stream);
			return(true);
		}
	}
	fclose(stream);
	return (false);
}
