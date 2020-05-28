/**************************************************************************
 * @file       nrf24l01_hal.cpp                                           *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      The hal of the NRF24L01P+ driver for Arduino.              *
 *************************************************************************/

#include <SPI.h>
#include "nrf24l01_hal.h"

/**************************************************************************
 * Static function declarations.                                          *
 **************************************************************************
 */
static void NRF24L01_HalDigitalWriteCePin(NRF24L01_DIGITAL_PIN value);
static NRF24L01_ERROR NRF24L01_HalSpiInit(void);
static NRF24L01_ERROR NRF24L01_HalSpiStart(void);
static NRF24L01_ERROR NRF24L01_HalSpiEnd(void);
static uint8_t NRF24L01_HalSpiTransfer(const uint8_t data);

/**************************************************************************
 * Extern variables.                                                      *
 **************************************************************************
 */
NRF24L01_HAL_REG Nrf24l01HalReg = 
{
  .digitalWriteCePin = NRF24L01_HalDigitalWriteCePin,
  .spiInit           = NRF24L01_HalSpiInit,
  .spiTransfer       = NRF24L01_HalSpiTransfer,
  .spiBegin          = NRF24L01_HalSpiStart,
  .spiEnd            = NRF24L01_HalSpiEnd
};

/**************************************************************************/

static void NRF24L01_HalDigitalWriteCePin(NRF24L01_DIGITAL_PIN value)
{
  if ( NRF24L01_PIN_HIGH == value )
  {
    digitalWrite(NRF24L01_HAL_CE_PIN, HIGH);
  }
  else
  {
    digitalWrite(NRF24L01_HAL_CE_PIN, LOW);
  }
}

/**************************************************************************/

static NRF24L01_ERROR NRF24L01_HalSpiInit(void)
{
  SPI.begin();
  pinMode(NRF24L01_HAL_CSN_PIN, OUTPUT);
  pinMode(NRF24L01_HAL_CE_PIN, OUTPUT);
  digitalWrite(NRF24L01_HAL_CSN_PIN, HIGH);
  SPI.endTransaction();
  return NRF24L01_SUCCESS;
}

/**************************************************************************/

static NRF24L01_ERROR NRF24L01_HalSpiStart(void)
{
  SPI.beginTransaction(SPISettings(NRF24L01_HAL_SPI_CLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(NRF24L01_HAL_CSN_PIN, LOW);
  return NRF24L01_SUCCESS;
}

/**************************************************************************/

static NRF24L01_ERROR NRF24L01_HalSpiEnd(void)
{
  digitalWrite(NRF24L01_HAL_CSN_PIN, HIGH);
  SPI.endTransaction();
  return NRF24L01_SUCCESS;
}

/**************************************************************************/

static uint8_t NRF24L01_HalSpiTransfer(const uint8_t data)
{
  return SPI.transfer(data);
}
