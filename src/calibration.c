/**********************************************************************************/
/* FILE NAME: calibration.c                                                       */
/*   VERSION: 1.0.5                                                                 */
/*      DATE: 27 AUG 2026                                                         */
/*    AUTHOR: Simon Grainger                                                      */
/*            Copyright © 2026 - S.W.Grainger                                     */
/*                                                                                */
/* DESCRIPTION: CockpitForYou Motorised TQ plugin for X-Plane 12.                 */
/**********************************************************************************/

/* standard include files*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Windows.h>

/* project include files */
#include "datastructures.h"
#include "log.h"
#include "plugin_paths.h"
#include "calibration.h"

/**********************************************************************************/
/* trim whitespace utility                                                        */
/**********************************************************************************/
static void trim_line(char* text)
{
	char* end;
	while (*text == ' ' || *text == '\t')
		memmove(text, text + 1, strlen(text));
	end = text + strlen(text);
	while (end > text && (end[-1] == '\r' || end[-1] == '\n' || end[-1] == ' ' || end[-1] == '\t'))
		*--end = '\0';
}

/**********************************************************************************/
/* set the TQ calibration defaults                                                */
/**********************************************************************************/
void tq_calibration_defaults(TqCalibration* value)
{
	memset(value, 0, sizeof(*value));
	value->lever1_min_speed = 8;
	value->lever2_min_speed = 8;
	value->trim_min_speed = 50;
	value->lever1_max_position = value->lever2_max_position = 4095;
	value->spoiler_max_position = value->reverser1_max_position = 4095;
	value->reverser2_max_position = value->flaps_max_position = 4095;
	strcpy_s(value->calibration_id, sizeof(value->calibration_id), "uninitialised");
}

/**********************************************************************************/
/* read the TQ calibration file                                                   */
/**********************************************************************************/
int tq_calibration_read(TqCalibration* value)
{
	char path[MAX_PATH], line[256];
	FILE* stream;
	uint32_t seen = 0;
	CalibrationField fields[] = {{"lever1_min_speed", &value->lever1_min_speed}, {"lever2_min_speed", &value->lever2_min_speed}, {"trim_min_speed", &value->trim_min_speed}, {"lever1_min_position", &value->lever1_min_position}, {"lever1_max_position", &value->lever1_max_position}, {"lever2_min_position", &value->lever2_min_position}, {"lever2_max_position", &value->lever2_max_position}, {"spoiler_min_position", &value->spoiler_min_position}, {"spoiler_max_position", &value->spoiler_max_position}, {"reverser1_min_position", &value->reverser1_min_position}, {"reverser1_max_position", &value->reverser1_max_position}, {"reverser2_min_position", &value->reverser2_min_position}, {"reverser2_max_position", &value->reverser2_max_position}, {"flaps_min_position", &value->flaps_min_position}, {"flaps_max_position", &value->flaps_max_position}};
	if (!plugin_file_path(path, sizeof(path), "xpCFY_TQ.calibration.cfg") || fopen_s(&stream, path, "r") != 0)
		return (0);
	while (fgets(line, sizeof(line), stream))
	{
		char* equals;
		size_t index;
		trim_line(line);

		if (!line[0] || line[0] == '#' || line[0] == ';' || !(equals = strchr(line, '=')))
			continue;

		*equals++ = '\0';

		trim_line(line);
		trim_line(equals);

		if (strcmp(line, "calibration_id") == 0)
		{
			strncpy_s(value->calibration_id, sizeof(value->calibration_id), equals, _TRUNCATE);
			seen |= UINT32_C(1) << 15;
			continue;
		}
		for (index = 0; index < sizeof(fields) / sizeof(fields[0]); ++index)
		{
			if (strcmp(line, fields[index].name) == 0)
			{
				char* end;
				unsigned long number = strtoul(equals, &end, 10);
				if (*equals && !*end && number <= UINT32_MAX)
				{
					*fields[index].value = (uint32_t)number;
					seen |= UINT32_C(1) << index;
				}
				break;
			}
		}
	}
	fclose(stream);

	if (seen != UINT32_C(0xffff) || strcmp(value->calibration_id, TQ_CALIBRATION_ID) != 0 || value->lever1_min_speed > 255 || value->lever2_min_speed > 255 || value->trim_min_speed > 255 || value->lever1_max_position > 4095 || value->lever2_max_position > 4095 || value->spoiler_max_position > 4095 || value->reverser1_max_position > 4095 || value->reverser2_max_position > 4095 || value->flaps_max_position > 4095 || (value->lever1_min_position > value->lever1_max_position || value->lever1_max_position - value->lever1_min_position < 100U) || (value->lever2_min_position > value->lever2_max_position || value->lever2_max_position - value->lever2_min_position < 100U) || (value->spoiler_min_position > value->spoiler_max_position || value->spoiler_max_position - value->spoiler_min_position < 100U) || (value->reverser1_min_position > value->reverser1_max_position || value->reverser1_max_position - value->reverser1_min_position < 100U) || (value->reverser2_min_position > value->reverser2_max_position || value->reverser2_max_position - value->reverser2_min_position < 100U) || (value->flaps_min_position > value->flaps_max_position || value->flaps_max_position - value->flaps_min_position < 100U))
	{
		log_write("Calibration is incomplete or contains an invalid value");
		return (0);
	}

	log_write("Calibration loaded from %s", path);
	return (1);
}

/**********************************************************************************/
/* write the TQ calibration file                                                  */
/**********************************************************************************/
int tq_calibration_write(const TqCalibration* value)
{
	char path[MAX_PATH], temporary[MAX_PATH];
	FILE* stream;
	int count, write_ok, close_ok, ok;
	if (!plugin_file_path(path, sizeof(path), "xpCFY_TQ.calibration.cfg"))
		return 0;
	count = snprintf(temporary, sizeof(temporary), "%s.tmp", path);
	if (count <= 0 || (size_t)count >= sizeof(temporary) || fopen_s(&stream, temporary, "w") != 0)
	{
		log_write("Unable to open temporary calibration file");
		return 0;
	}
	fprintf(stream, "# xpCFY_TQ calibration - raw 12-bit Pokeys readings\ncalibration_id=%s\n", value->calibration_id);
	fprintf(stream, "lever1_min_speed=%u\nlever2_min_speed=%u\ntrim_min_speed=%u\n", value->lever1_min_speed, value->lever2_min_speed, value->trim_min_speed);
	fprintf(stream, "lever1_min_position=%u\nlever1_max_position=%u\n", value->lever1_min_position, value->lever1_max_position);
	fprintf(stream, "lever2_min_position=%u\nlever2_max_position=%u\n", value->lever2_min_position, value->lever2_max_position);
	fprintf(stream, "spoiler_min_position=%u\nspoiler_max_position=%u\n", value->spoiler_min_position, value->spoiler_max_position);
	fprintf(stream, "reverser1_min_position=%u\nreverser1_max_position=%u\n", value->reverser1_min_position, value->reverser1_max_position);
	fprintf(stream, "reverser2_min_position=%u\nreverser2_max_position=%u\n", value->reverser2_min_position, value->reverser2_max_position);
	fprintf(stream, "flaps_min_position=%u\nflaps_max_position=%u\n", value->flaps_min_position, value->flaps_max_position);
	write_ok = fflush(stream) == 0 && !ferror(stream);
	close_ok = fclose(stream) == 0;
	ok = write_ok && close_ok && MoveFileExA(temporary, path, MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH);
	if (ok)
	{
		log_write("Calibration saved to %s", path);
	}
	else
	{
		log_write("Unable to persist calibration %s (error %lu)", path, GetLastError());
		DeleteFileA(temporary);
	}
	return ok;
}
