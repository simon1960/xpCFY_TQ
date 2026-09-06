/**********************************************************************************/
/* FILE NAME: plugin_paths.c                                                      */
/*   VERSION: 1.0.2                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQplugin for X-Plane 12.                  */
/**********************************************************************************/

/* standard include files */
#include <stdio.h>
#include <string.h>
#include <Windows.h>

/* project include files */
#include "plugin_paths.h"

static char g_plugin_directory[MAX_PATH];

int plugin_paths_initialise(void)
{
    HMODULE module;
    DWORD length;
    char path[MAX_PATH];
    char* separator;

    /* XPLMGetPluginInfo may return a path relative to the X-Plane directory.
       Resolve the loaded plugin module through Windows instead so every file
       lookup is independent of the process working directory. */

    if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)&plugin_paths_initialise, &module)) 
        return(0);
    
    length = GetModuleFileNameA(module, path, (DWORD)sizeof(path));
    if (length == 0 || length >= (DWORD)sizeof(path)) 
        return (0);

    separator = strrchr(path, '\\');
    if (!separator) separator = strrchr(path, '/');
    if (!separator) 
        return (0);
    *separator = '\0';
    return (strcpy_s(g_plugin_directory, sizeof(g_plugin_directory), path) == 0);
}

const char* plugin_directory(void) { return(g_plugin_directory); }

int plugin_file_path(char* destination, size_t destination_size, const char* filename)
{
    int count = snprintf(destination, destination_size, "%s\\%s", g_plugin_directory, filename);
    return(count > 0 && (size_t)count < destination_size);
}
