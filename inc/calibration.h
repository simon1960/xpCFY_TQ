/**********************************************************************************/
/* FILE NAME: calibration.h                                                       */
/*   VERSION: 1.0.5                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQ plugin for X-Plane 12.                 */
/**********************************************************************************/

#ifndef _XPCFY_TQ_CALIBRATION_H_
#define _XPCFY_TQ_CALIBRATION_H_

/* standard include files */
#include <stdint.h>

/* project include files */
#include "datastructures.h"

#define TQ_CALIBRATION_ID "xpCFY_TQ-calibration-v1"

/* forward declaration of functions */
void tq_calibration_defaults(TqCalibration* calibration);
int tq_calibration_read(TqCalibration* calibration);
int tq_calibration_write(const TqCalibration* calibration);

#endif // !_XPCFY_TQ_CALIBRATION_H_
