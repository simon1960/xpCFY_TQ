/**********************************************************************************/
/* FILE NAME: plugin_paths.c                                                      */
/*   VERSION: 1.0.6                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQplugin for X-Plane 12.                  */
/**********************************************************************************/

/* standard include files */
#include <stdio.h>
#include <string.h>
#include "platform.h"
#include <dlfcn.h>
#include <stdlib.h>

/* project include files */
#include "plugin_paths.h"

static char g_plugin_directory[MAX_PATH];

int plugin_paths_initialise(void)
{
	Dl_info module;
	char path[MAX_PATH];
	char* separator;

	/* Resolve the loaded ELF module so paths never depend on X-Plane's working directory. */
	if (!dladdr((void*)&plugin_paths_initialise, &module) || !module.dli_fname || !realpath(module.dli_fname, path))
		return (0);
	separator = strrchr(path, '/');
	if (!separator)
		return (0);
	if (separator == path)
		separator[1] = '\0';
	else
		*separator = '\0';
	return (strcpy_s(g_plugin_directory, sizeof(g_plugin_directory), path) == 0);
}

const char* plugin_directory(void)
{
	return (g_plugin_directory);
}

int plugin_file_path(char* destination, size_t destination_size, const char* filename)
{
	int count = snprintf(destination, destination_size, "%s/%s", g_plugin_directory, filename);
	return (count > 0 && (size_t)count < destination_size);
}
