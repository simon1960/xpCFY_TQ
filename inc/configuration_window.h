/**********************************************************************************/
/* FILE NAME: configuration_window.h                                              */
/*   VERSION: 1.0.5                                                               */
/*      DATE: 07 SEP 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: xpCFY_TQ general hardware configuration and test window.          */
/**********************************************************************************/

#ifndef _XPCFY_TQ_CONFIGURATION_WINDOW_H_
#define _XPCFY_TQ_CONFIGURATION_WINDOW_H_

#include "datastructures.h"

int configuration_window_initialise(PluginConfig* config, TqCalibration* calibration);
void configuration_window_shutdown(void);
void configuration_window_show(void);

#endif
