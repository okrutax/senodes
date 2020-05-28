/**************************************************************************
 * @file       debug_hal.h                                                *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      It's a HAL of print and println functions for Arduino.     *
 *************************************************************************/

#ifndef _DEBUG_H_ // Debug
#define _DEBUG_H_ 

#ifdef _cplusplus_
extern "C"
{
#endif

#define DEBUG_DEC   (10)
#define DEBUG_HEX   (16)
#define DEBUG_OCT   (8)
#define DEBUG_BIN   (2)

/**************************************************************************
 * Global function declarations.                                          *
 **************************************************************************
 */
void DEBUG_PrintString(const char * message);
void DEBUG_PrintStringln(const char * message);
void DEBUG_PrintNumber(const int n, const int base);

#ifdef _cplusplus
}
#endif

#endif // _DEBUG_H_