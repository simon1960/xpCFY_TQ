/**********************************************************************************/
/* FILE NAME: plugin_config.h                                                     */
/*   VERSION: 1.0.4                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQ plugin for X-Plane 12.                 */
/**********************************************************************************/

#ifndef _XPCFY_TQ_PLUGIN_CONFIG_H_
#define _XPCFY_TQ_PLUGIN_CONFIG_H_

/* standard include files */
#include <stdint.h>

/* project include files */
#include "datastructures.h"

/* forward declatation of functions */
void plugin_config_defaults(PluginConfig* config);
int plugin_config_read(PluginConfig* config);
int plugin_config_write(const PluginConfig* config);

#endif // !_XPCFY_TQ_PLUGIN_CONFIG_H_
