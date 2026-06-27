/*
	OpenBikeComputer core application
    Copyright (C) 2023  LAMBS Pierre-Antoine

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "esp_timer.h"

#include "log.h"

#define MAX_LOG_LENGTH 256

void _write_log(const char* log_level_txt, const char * filename, const char * function, int line, const char * format, ...){
	va_list args;
	char time_string[32];
	char log_va_list_buffer[MAX_LOG_LENGTH];

	/*
	 * Get va_args into log_va_list_buffer
	 */
	va_start(args,format);
	vsnprintf(log_va_list_buffer, sizeof(log_va_list_buffer), format, args);
	va_end(args);

	/*
	 * make string from time since boot
	 */
	uint64_t us  = (uint64_t)esp_timer_get_time();
	uint64_t sec = us / 1000000ULL;
	uint64_t ms  = (us % 1000000ULL) / 1000ULL;
	snprintf(time_string, sizeof(time_string), "%llu:%03llu", (unsigned long long)sec, (unsigned long long)ms);

	/*
	 * Extract filename basename from full path
	 */
	const char *short_filename = strrchr(filename, '/');
	short_filename = short_filename ? short_filename + 1 : filename;

	/*
	 * Print log on console
	 */
	printf( "[%s][%s:%d:%s]%s %s", time_string, short_filename, line, function, log_level_txt, log_va_list_buffer);

	return;
}
