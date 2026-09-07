/**********************************************************************************/
/* FILE NAME: plugin_config.c                                                     */
/*   VERSION: 1.0.4                                                                 */
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
#include "log.h"
#include "plugin_paths.h"
#include "plugin_config.h"

/**********************************************************************************/
/* set the plugin configuration defaults                                          */
/**********************************************************************************/
void plugin_config_defaults(PluginConfig* config)
{
    config->preferred_serial = 0;
    config->network_timeout_ms = 1000;
    config->retry_delay_ms = 5000;
    config->discovery_timeout_ms = 10000;
    /* The development TQ (serial 28630) is a V4. V3 and Pro select 3 and 5. */
    config->trim_motor_variant = 4;
    config->search_usb = 1;
    config->search_network = 1;
    config->network_use_udp = 0;
    config->require_cfy_user_id = 1;
}

static uint32_t ini_uint(const char* path, const char* name, uint32_t fallback, uint32_t minimum, uint32_t maximum)
{
    UINT value = GetPrivateProfileIntA("connection", name, fallback, path);
    return(value >= minimum && value <= maximum ? value : fallback);
}

/*
 * Read the human-readable protocol key first, then accept the numeric key
 * written by the first implementation. needs_rewrite tells the caller to
 * migrate an old or incomplete file so the selected protocol is explicit.
 */
static int ini_network_protocol(const char* path, int fallback,
    int* needs_rewrite)
{
    char value[16];
    DWORD length;

    *needs_rewrite = 0;
    length = GetPrivateProfileStringA("connection", "network_protocol", "",
        value, (DWORD)sizeof(value), path);
    if (length != 0)
    {
        if (_stricmp(value, "UDP") == 0) return(1);
        if (_stricmp(value, "TCP") == 0) return(0);
        log_write("Invalid network_protocol value '%s'; using %s", value,
            fallback ? "UDP" : "TCP");
        *needs_rewrite = 1;
        return(fallback);
    }

    length = GetPrivateProfileStringA("connection", "network_use_udp", "",
        value, (DWORD)sizeof(value), path);
    *needs_rewrite = 1;
    if (length != 0)
    {
        if (strcmp(value, "1") == 0) return(1);
        if (strcmp(value, "0") == 0) return(0);
        log_write("Invalid legacy network_use_udp value '%s'; using %s",
            value, fallback ? "UDP" : "TCP");
    }
    return(fallback);
}

/**********************************************************************************/
/* read the plugin configuration                                                  */
/**********************************************************************************/
int plugin_config_read(PluginConfig* config)
{
    char path[MAX_PATH];
    DWORD attributes;
    int needs_protocol_rewrite;
    if (!plugin_file_path(path, sizeof(path), "xpCFY_TQ.cfg")) 
        return(0);
    attributes = GetFileAttributesA(path);

    if (attributes == INVALID_FILE_ATTRIBUTES || (attributes & FILE_ATTRIBUTE_DIRECTORY))
        return(0);

    config->preferred_serial = ini_uint(path, "preferred_serial", config->preferred_serial, 0, UINT32_MAX);
    config->network_timeout_ms = ini_uint(path, "network_timeout_ms", config->network_timeout_ms, 100, 10000);
    config->retry_delay_ms = ini_uint(path, "retry_delay_ms", config->retry_delay_ms, 250, 60000);
    config->discovery_timeout_ms = ini_uint(path, "discovery_timeout_ms", config->discovery_timeout_ms, 1000, 30000);
    config->trim_motor_variant = ini_uint(path, "trim_motor_variant", config->trim_motor_variant, 3, 5);
    config->search_usb = ini_uint(path, "search_usb", config->search_usb, 0, 1) != 0;
    config->search_network = ini_uint(path, "search_network", config->search_network, 0, 1) != 0;
    config->network_use_udp = ini_network_protocol(path,
        config->network_use_udp, &needs_protocol_rewrite);
    config->require_cfy_user_id = ini_uint(path, "require_cfy_user_id", config->require_cfy_user_id, 0, 1) != 0;
    log_write("Configuration loaded from %s (TQ variant %s, PoKeys network protocol %s)",
        path, config->trim_motor_variant == 3U ? "V3" :
        (config->trim_motor_variant == 5U ? "Pro" : "V4"),
        config->network_use_udp ? "UDP" : "TCP");
    if (needs_protocol_rewrite)
    {
        log_write("Migrating plugin configuration to persist the selected PoKeys protocol");
        if (!plugin_config_write(config)) return(0);
    }
    return(1);
}

/**********************************************************************************/
/* write the plugin configuration                                                 */
/**********************************************************************************/
int plugin_config_write(const PluginConfig* config)
{
    char path[MAX_PATH], temporary[MAX_PATH];
    FILE* stream;
    int count, write_ok, close_ok, ok;
    char persisted_protocol[16];
    UINT persisted_variant;

    if (!plugin_file_path(path, sizeof(path), "xpCFY_TQ.cfg"))
        return(0);
    count = snprintf(temporary, sizeof(temporary), "%s.tmp", path);
    if (count <= 0 || (size_t)count >= sizeof(temporary) || fopen_s(&stream, temporary, "w") != 0)
    {
        log_write("Unable to open temporary plugin configuration file");
        return(0);
    }
    
    fprintf(stream, "# xpCFY_TQ hardware connection settings\n[connection]\n");
    fprintf(stream, "preferred_serial=%u\n", config->preferred_serial);
    fprintf(stream, "search_usb=%d\nsearch_network=%d\n", config->search_usb, config->search_network);
    fprintf(stream, "network_protocol=%s\nnetwork_use_udp=%d\n",
        config->network_use_udp ? "UDP" : "TCP", config->network_use_udp);
    fprintf(stream, "require_cfy_user_id=%d\n", config->require_cfy_user_id);
    fprintf(stream, "network_timeout_ms=%u\nretry_delay_ms=%u\ndiscovery_timeout_ms=%u\n", config->network_timeout_ms, config->retry_delay_ms, config->discovery_timeout_ms);
    fprintf(stream, "trim_motor_variant=%u\n", config->trim_motor_variant);
    write_ok = fflush(stream) == 0 && !ferror(stream);
    close_ok = fclose(stream) == 0;
    ok = write_ok && close_ok && MoveFileExA(temporary, path, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH);
    if (ok)
    {
        persisted_protocol[0] = '\0';
        GetPrivateProfileStringA("connection", "network_protocol", "",
            persisted_protocol, (DWORD)sizeof(persisted_protocol), path);
        persisted_variant = GetPrivateProfileIntA("connection",
            "trim_motor_variant", 0, path);
        ok = _stricmp(persisted_protocol,
            config->network_use_udp ? "UDP" : "TCP") == 0 &&
            persisted_variant == config->trim_motor_variant;
        if (!ok)
            log_write("Plugin configuration verification failed for %s", path);
    }
    if (!ok) 
    {
        log_write("Unable to persist plugin configuration %s (error %lu)", path, GetLastError());
        DeleteFileA(temporary);
    }
    return(ok);
}
