/**************************************************************************
 * @file       debug_hal.cpp                                              *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      It's a HAL of print and println functions for Arduino.     *
 *************************************************************************/

#include <Arduino.h>
#include "debug_hal.h"
#include <stdio.h>

void DEBUG_PrintString(const char * message)
{
  Serial.print(message);
}

/**************************************************************************/

void DEBUG_PrintStringln(const char * message)
{
  Serial.println(message);
}

/**************************************************************************/

void DEBUG_PrintNumber(const int n, const int base)
{
   Serial.print(n, base);
}