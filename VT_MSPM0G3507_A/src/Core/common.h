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

/* for the uart nputs(char*,num len) command */
#define ARRANDN(ARR)                (ARR),sizeof(ARR)

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

typedef uint16_t buffersize_t;


#endif /* SRC_CORE_COMMON_H_ */
