/**********************************************************************************/
/* FILE NAME: aircraft_config.h                                                   */
/*   VERSION: 1.0.5                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQ plugin for X-Plane 12.                 */
/**********************************************************************************/
#ifndef _XPCFY_TQ_AIRCRAFT_CONFIG_H_
#define _XPCFY_TQ_AIRCRAFT_CONFIG_H_

/* standard include files */
#include <stdbool.h>

/* forward declaration of functions */
int aircraft_config_initialise(void);
bool aircraft_config_contains_tail_number(const char* tail_number);

#endif
