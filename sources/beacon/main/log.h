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

#ifndef _LOG_HEADER_
#define _LOG_HEADER_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define LOG_LEVEL_ERROR 3
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_INFO 1
#define LOG_LEVEL_DEBUG 0

/* Define default log level */
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

// --- LOG ---
//#define log(...)
//#define log_error(...)
//#define log_warn(...)
//#define log_info(...)
//#define log_debug(...)

// --- Common die ---
//#define fail(return_value, ...)
//#define fail_if_negative_or_zero(value_to_check, return_value, ...)
//#define fail_if_negative(value_to_check, return_value, ...)
//#define fail_if_zero(value_to_check, return_value, ...)
//#define fail_if_not_zero(value_to_check, return_value, ...)
//#define fail_if_positive(value_to_check, return_value, ...)
//#define fail_if_positive_or_zero(value_to_check, return_value, ...)
//#define fail_if_null(value_to_check, return_value, ...)
//#define fail_if_not_null(value_to_check, return_value, ...)
//#define fail_if_true(value_to_check, return_value, ...)
//#define fail_if_false(value_to_check, return_value, ...)
//#define fail_if_equal(value_to_check_1, value_to_check_2, return_value, ...)
//#define fail_if_not_equal(value_to_check_1, value_to_check_2, return_value, ...)
//#define fail_if_superior(value_to_check_1, value_to_check_2, return_value, ...)
//#define fail_if_inferior(value_to_check_1, value_to_check_2, return_value, ...)
//#define fail_if_superior_or_equal(value_to_check_1, value_to_check_2, return_value, ...)
//#define fail_if_inferior_or_equal(value_to_check_1, value_to_check_2, return_value, ...)

//_write_log() function don't use it directy, use log() macro instead
void _write_log(const char* log_level_txt, const char * filename, const char * function, int line, const char * format, ...);

//#########################################
//#                                       #
//#               Log section             #
//#                                       #
//#########################################

#define log_with_level(level, level_txt, ...) do { if(level >= LOG_LEVEL) { _write_log(level_txt,__FILE__,__func__,__LINE__,__VA_ARGS__); } } while(0)
#define log_without_level(level_txt, ...) do { _write_log(level_txt,__FILE__,__func__,__LINE__,__VA_ARGS__); } while(0)

/* Write simple log */
#define log_error(...) log_with_level(LOG_LEVEL_ERROR,"[ERROR]",__VA_ARGS__)
#define log_warn(...)  log_with_level(LOG_LEVEL_WARN,"[WARN]",__VA_ARGS__)
#define log_info(...)  log_with_level(LOG_LEVEL_INFO,"[INFO]",__VA_ARGS__)
#define log_debug(...) log_with_level(LOG_LEVEL_DEBUG,"[DEBUG]",__VA_ARGS__)
#define log(...)       log_without_level("[LOG]",__VA_ARGS__)

//#########################################
//#                                       #
//#           Common die section          #
//#                                       #
//#########################################

/*
 * LOG IF THE VALUE IS EXPECTED (does not return, only logs)
 */

//Log if value_to_check is stricly 'Negative'
#define log_if_negative(value_to_check, ...)                               \
	if(value_to_check < 0){                                                   \
		log_error(__VA_ARGS__);                                               \
	}

/*
 * DIE IF THE VALUE IS EXPECTED
 */

//Just DIE...
#define fail(return_value, ...){                                        \
	log_error(__VA_ARGS__);                                                   \
	return return_value;                                                      \
}

//Die if value_to_check is 'Negative' OR 'zero'
#define fail_if_negative_or_zero(value_to_check, return_value, ...)        \
	if(value_to_check <= 0){                                                  \
		log_error(__VA_ARGS__);                                               \
		return return_value;                                                  \
	}

//Die if value_to_check is stricly 'Negative'
#define fail_if_negative(value_to_check, return_value, ...)                \
	if(value_to_check < 0){                                                   \
		log_error(__VA_ARGS__);                                               \
		return return_value;                                                  \
	}

//Die if value_to_check is stricly 'Zero'
#define fail_if_zero(value_to_check, return_value, ...)                    \
	if(value_to_check == 0){                                                  \
		log_error(__VA_ARGS__);                                               \
		return return_value;                                                  \
	}

//Die if value_to_check is stricly different from 'Zero'
#define fail_if_not_zero(value_to_check, return_value, ...)                \
	if(value_to_check != 0){                                                  \
		log_error(__VA_ARGS__);                                               \
		return return_value;                                                  \
	}

//Die if value_to_check is stricly 'Positive'
#define fail_if_positive(value_to_check, return_value, ...)                \
	if(value_to_check > 0){                                                   \
		log_error(__VA_ARGS__);                                               \
		return return_value;                                                  \
	}

//Die if value_to_check is 'Positive' OR 'Zero'
#define fail_if_positive_or_zero(value_to_check, return_value, ...)        \
	if(value_to_check >= 0){                                                  \
		log_error(__VA_ARGS__);                                               \
		return return_value;                                                  \
	}

//Die if value_to_check is 'NULL'
#define fail_if_null(value_to_check, return_value, ...)                    \
	if(value_to_check == NULL){                                               \
		log_error(__VA_ARGS__);                                               \
		return return_value;                                                  \
	}

//Die if value_to_check is 'NOT NULL'
#define fail_if_not_null(value_to_check, return_value, ...)                \
	if(value_to_check != NULL){                                               \
		log_error(__VA_ARGS__);                                               \
		return return_value;                                                  \
	}

//Die if value_to_check is 'TRUE'
#define fail_if_true(value_to_check, return_value, ...)                    \
	if(value_to_check == true){                                               \
		log_error(__VA_ARGS__);                                               \
		return return_value;                                                  \
	}

//Die if value_to_check is 'FALSE'
#define fail_if_false(value_to_check, return_value, ...)                   \
	if(value_to_check == false){                                              \
		log_error(__VA_ARGS__);                                               \
		return return_value;                                                  \
	}

//Die if value_to_check_1 and value_to_check_2 are 'EQUAL'
#define fail_if_equal(value_to_check_1, value_to_check_2, return_value, ...)    \
	if(value_to_check_1 == value_to_check_2){                                      \
		log_error(__VA_ARGS__);                                                    \
		return return_value;                                                       \
	}

//Die if value_to_check_1 and value_to_check_2 are 'NOT EQUAL'
#define fail_if_not_equal(value_to_check_1, value_to_check_2, return_value, ...)\
	if(value_to_check_1 != value_to_check_2){                                      \
		log_error(__VA_ARGS__);                                                    \
		return return_value;                                                       \
	}

//Die if value_to_check_1 is 'SUPERIOR TO' value_to_check_2
#define fail_if_superior(value_to_check_1, value_to_check_2, return_value, ...) \
	if(value_to_check_1 > value_to_check_2){                                       \
		log_error(__VA_ARGS__);                                                    \
		return return_value;                                                       \
	}

//Die if value_to_check_1 is 'INFERIOR TO' value_to_check_2
#define fail_if_inferior(value_to_check_1, value_to_check_2, return_value, ...) \
	if(value_to_check_1 < value_to_check_2){                                       \
		log_error(__VA_ARGS__);                                                    \
		return return_value;                                                       \
	}

//Die if value_to_check_1 is 'SUPERIOR OR EQUAL TO' value_to_check_2
#define fail_if_superior_or_equal(value_to_check_1, value_to_check_2, return_value, ...)\
	if(value_to_check_1 >= value_to_check_2){                                              \
		log_error(__VA_ARGS__);                                                            \
		return return_value;                                                               \
	}

//Die if value_to_check_1 is 'INFERIOR OR EQUAL TO' value_to_check_2
#define fail_if_inferior_or_equal(value_to_check_1, value_to_check_2, return_value, ...)\
	if(value_to_check_1 <= value_to_check_2){                                              \
		log_error(__VA_ARGS__);                                                            \
		return return_value;                                                               \
	}

#endif //_LOG_HEADER_
