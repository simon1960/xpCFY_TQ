/**********************************************************************************/
/* FILE NAME: log.c                                                               */
/*   VERSION: 1.0                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQ plugin for X-Plane 12.                 */
/**********************************************************************************/

/* standard include files */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <Windows.h>

/* X-Plane SDK include files */
#include "XPLMUtilities.h"

/* project include files */
#include "plugin_paths.h"
#include "log.h"

/*local variables */
static SRWLOCK g_log_lock = SRWLOCK_INIT;
static char g_log_path[MAX_PATH];
static int g_log_ready;


int log_initialise(void)
{
    FILE* stream;
    char message[MAX_PATH + 96];

    g_log_ready = 0;
    
    if (!plugin_file_path(g_log_path, sizeof(g_log_path), "xpCFY_TQ.log")) 
        return (0);
    
    /* Each plugin load owns a fresh session log; remove any prior run first. */
    if (DeleteFileA(g_log_path) == 0 && GetLastError() != ERROR_FILE_NOT_FOUND)
    {
        snprintf(message, sizeof(message), "xpCFY_TQ: unable to delete old log file %s (Windows error %lu)\n", g_log_path, GetLastError());
        XPLMDebugString(message);
        return (0);
    }

    if (fopen_s(&stream, g_log_path, "w") != 0) 
    {
        snprintf(message, sizeof(message), "xpCFY_TQ: unable to open log file %s (Windows error %lu)\n", g_log_path, GetLastError());
        XPLMDebugString(message);
        return (0);
    }
    if (fclose(stream) != 0) 
    {
        snprintf(message, sizeof(message), "xpCFY_TQ: unable to close log file %s after validation\n", g_log_path);
        XPLMDebugString(message);
        return (0);
    }
    g_log_ready = 1;
    return (1);
}

/* Return only the file-name component supplied by the compiler. */
static const char* log_source_name(const char* source_file)
{
    const char* slash;
    const char* backslash;
    const char* name;
    size_t length;

    if (!source_file || source_file[0] == '\0') return ("unknown");
    slash = strrchr(source_file, '/');
    backslash = strrchr(source_file, '\\');
    name = source_file;
    if (slash && slash + 1 > name) name = slash + 1;
    if (backslash && backslash + 1 > name) name = backslash + 1;

    /* Keep the field fixed at 30 characters if a future name is longer. */
    length = strlen(name);
    if (length > 30U) name += length - 30U;
    return (name);
}

void log_write_at(const char* source_file, unsigned int source_line,
    const char* format, ...)
{
    FILE* stream;
    SYSTEMTIME now;
    va_list arguments;
    if (!g_log_ready) return;
    AcquireSRWLockExclusive(&g_log_lock);
    if (fopen_s(&stream, g_log_path, "a") == 0)
    {
        GetLocalTime(&now);
        fprintf(stream,
            "%02u.%02u.%04u %02u:%02u:%02u.%03u %30.30s %5u ",
            now.wDay, now.wMonth, now.wYear, now.wHour, now.wMinute,
            now.wSecond, now.wMilliseconds, log_source_name(source_file),
            source_line);
        va_start(arguments, format);
        vfprintf(stream, format, arguments);
        va_end(arguments);
        fputc('\n', stream);
        fclose(stream);
    }
    else 
    {
        char message[MAX_PATH + 96];
        snprintf(message, sizeof(message), "xpCFY_TQ: unable to append to log file %s\n", g_log_path);
        OutputDebugStringA(message);
    }
    ReleaseSRWLockExclusive(&g_log_lock);
}

void log_shutdown(void) { g_log_ready = 0; }
