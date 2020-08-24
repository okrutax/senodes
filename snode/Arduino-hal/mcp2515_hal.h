/**************************************************************************
 * @file       mcp2515_hal.h                                              *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      The hal of the MCP2515 driver that implements              *
 *             Controller Area Network(CAN) for Arduino.                  *
 *************************************************************************/

#ifndef _MCP2515_HAL_H_ // MCP2515 Hal
#define _MCP2515_HAL_H_ 

#ifdef _cplusplus_
extern "C"
{
#endif

#include "mcp2515.h"

#if defined (snodeesp32)
#define MCP2515_HAL_CE_PIN         (15)
#else
#define MCP2515_HAL_CE_PIN         (PIN_SPI_SS)
#endif
#define MCP2515_HAL_SPI_CLOCK      (10000000) //! 10MHz.

extern MCP2515_HAL_REG  Mcp2515HalReg;

#ifdef _cplusplus
}
#endif

#endif // _MCP2515_HAL_H_