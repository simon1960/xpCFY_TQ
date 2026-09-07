/**********************************************************************************/
/* FILE NAME: calibration_window.h                                                */
/*   VERSION: 1.0.4                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQplugin for X-Plane 12.                  */
/**********************************************************************************/
/* xpCFY_TQ live lever-position and manual calibration window                     */
/**********************************************************************************/

#ifndef _XPCFY_TQ_CALIBRATION_WINDOW_H_
#define _XPCFY_TQ_CALIBRATION_WINDOW_H_

/* project include files */
#include "datastructures.h"

/* forward declaration of functions */
int calibration_window_initialise(TqCalibration* calibration);
void calibration_window_shutdown(void);
void calibration_window_show_positions(void);
void calibration_window_begin(int automatic_request);

#endif
