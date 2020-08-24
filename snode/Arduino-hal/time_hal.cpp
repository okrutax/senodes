/**************************************************************************
 * @file       time_hal.cpp                                               *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      It's the HAL of get milliseconds function for Arduino.     *
 *************************************************************************/

#include <Arduino.h>
#include "time_hal.h"

/**************************************************************************
 * @brief This function returns the number of milliseconds passed since   *
 *        the Arduino board began running                                 *
 *                                                                        *
 * @param none                                                            *
 * @return Milliseconds.                                                  *
 **************************************************************************
 */
uint32_t TIME_GetMs(void)
{
 return millis();
}

/**************************************************************************/

void TIME_DelayMs(unsigned long ms)
{
  delay(ms);
}

/**************************************************************************/

void TIME_DelayUs(unsigned long us)
{
  delayMicroseconds(us);
}