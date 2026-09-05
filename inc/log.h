/**********************************************************************************/
/* FILE NAME: log.h                                                               */
/*   VERSION: 1.0                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQ plugin for X-Plane 12.                 */
/**********************************************************************************/

#ifndef _XPCFY_TQ_LOG_H_
#define _XPCFY_TQ_LOG_H_

/* forward declaration of functions */
int log_initialise(void);
void log_write_at(const char* source_file, unsigned int source_line, const char* format, ...);
void log_shutdown(void);

/*
 * Preserve the existing log_write(...) interface while automatically adding
 * the source location of every call. Keeping the capture in this header means
 * __FILE__ and __LINE__ identify the caller rather than the logging module.
 */
#define log_write(...) log_write_at(__FILE__, (unsigned int)__LINE__, __VA_ARGS__)

#endif // !_XPCFY_TQ_LOG_H_
