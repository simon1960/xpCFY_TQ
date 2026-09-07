/**********************************************************************************/
/* FILE NAME: acf_dref.h                                                          */
/*   VERSION: 1.0.3                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQplugin for X-Plane 12.                  */
/**********************************************************************************/
#ifndef _ACF_DREF_H_
#define _ACF_DREF_H_

/* project include files */
#include "datastructures.h"

extern struct DREF_TABLE drefTable[DREF_END];
extern struct CMD_TABLE cmdTable[CMD_END];

/* switch handlers */
extern void left_toga_handler(void* param);
extern void right_toga_handler(void* param);
extern void left_at_disco_handler(void* param);
extern void right_at_disco_handler(void* param);

/* dataref and command handlers */
extern void GetDataRefHandles(void);
extern void GetCommandHandles(void);
extern void UnregisterTqTrimCommandHandlers(void);
extern void ReleaseTqPushbuttonCommands(void* param);
extern float GetAircraftDataFLCB(float elapsedMe, float elapsedSim, int counter, void* inRefcon);		// data gatherer function

/* TQ calibration and test */
extern void TqControlsSetCalibration(const TqCalibration* calibration, int valid);
extern void TqControlsReset(void);
extern void TqControlsDeactivate(void);
extern void TqControlsSetAircraftActive(int active);
extern int TqGroundTestControlsAllowed(void);

#endif // !_ACF_DREF_H_
