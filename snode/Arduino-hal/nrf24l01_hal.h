/**************************************************************************
 * @file       nrf24l01_hal.h                                             *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      The hal of the NRF24L01P+ driver for Arduino.              *
 *************************************************************************/

#ifndef _NRF24L01P_HAL_H_
#define _NRF24L01P_HAL_H_ 

#ifdef _cplusplus_
extern "C"
{
#endif

#include "nrf24l01.h"

#define  NRF24L01_HAL_CSN_PIN         (10)      //! SPI Chip select.
#define  NRF24L01_HAL_CE_PIN          (9)       //! "Chip Enable" pin, activates the RX or TX role.
#define  NRF24L01_HAL_SPI_CLOCK       (4000000) //! 4MHz.Speed SPI communication.

extern NRF24L01_HAL_REG Nrf24l01HalReg;

#ifdef _cplusplus
}
#endif

#endif // _NRF24L01P_HAL_H_