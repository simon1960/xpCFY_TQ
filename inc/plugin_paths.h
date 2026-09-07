/**********************************************************************************/
/* FILE NAME: plugin_paths.h                                                      */
/*   VERSION: 1.0.3                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQ plugin for X-Plane 12.                 */
/**********************************************************************************/

#ifndef _XPCFY_TQ_PLUGIN_PATHS_H_
#define _XPCFY_TQ_PLUGIN_PATHS_H_

/* standard include files */
#include <stddef.h>

/* forward declaration of functions */
int plugin_paths_initialise(void);
const char* plugin_directory(void);
int plugin_file_path(char* destination, size_t destination_size, const char* filename);

#endif // !_XPCFY_TQ_PLUGIN_PATHS_H_
