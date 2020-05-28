/**************************************************************************
 * @file       time_hal.h                                                 *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      It's the HAL of get milliseconds function for Arduino.     *
 *************************************************************************/

#ifndef _TIME_H_ // Time
#define _TIME_H_ 

#ifdef _cplusplus_
extern "C"
{
#endif

#include <stdint.h>

/**************************************************************************
 * Global function declarations.                                          *
 **************************************************************************
 */
uint32_t TIME_GetMs(void);
void     TIME_DelayMs(unsigned long ms);
void     TIME_DelayUs(unsigned long us);

#ifdef _cplusplus
}
#endif

#endif // _TIME_H_