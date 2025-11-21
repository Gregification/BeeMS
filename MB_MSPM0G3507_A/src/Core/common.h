/*
 * common.h
 *
 *  Created on: Nov 8, 2025
 *      Author: turtl
 */

#ifndef SRC_CORE_COMMON_H_
#define SRC_CORE_COMMON_H_

#include <stdint.h>
#include <stdbool.h>


//--- unit conversions ---------------------------------------

#define KELVIN_TO_CELSIUS(C) (C - 273.15);
#define K2C(C) KELVIN_TO_CELSIUS(C)

//--- program macros ------------------------------------------

#define ARRANDN(ARR)                (ARR),sizeof(ARR)
#define PTRANDN(ARR)                (&(ARR)),sizeof(ARR)  // for pointer to a object
#define ARRLEN(ARR)                 (sizeof((ARR))/sizeof((ARR)[0]))

//--- program constants ---------------------------------------

#define NEWLINE                     "\n\r"
#define CLIERROR                    "\033[38;2;255;0;0m"
#define CLIHIGHLIGHT                "\033[38;2;255;255;0m"
#define CLIBAD                      CLIERROR
#define CLIGOOD                     "\033[38;2;0;255;0m"
#define CLIYES                      "\033[38;2;0;255;255m"
#define CLINO                       "\033[38;2;255;0;255m"
#define CLIWARN                     "\033[38;2;255;100;0m"
#define CLIRESET                    "\033[0m"
#define CLICLEAR                    "\033[2J\033[H\033[0m"

#define MAX_STR_LEN_COMMON          125   // assumed max length of a string if not specified. to minimize the damage of overruns.
#define MAX_STR_ERROR_LEN           (MAX_STR_LEN_COMMON * 2)

//--- program types -------------------------------------------

typedef uint16_t buffersize_t;

//--- functions -----------------------------------------------

bool arrCmp(void const * a, void const * b, buffersize_t len);

#endif /* SRC_CORE_COMMON_H_ */
