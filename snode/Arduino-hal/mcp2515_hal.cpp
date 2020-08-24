/**************************************************************************
 * @file       mcp2515_hal.cpp                                            *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      The hal of the MCP2515 driver that implements              *
 *             Controller Area Network(CAN) for Arduino.                  *
 *************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "mcp2515_hal.h"

/**************************************************************************/

#if defined (snodeesp32)
  SPIClass HardwareSPI(HSPI);
#endif

/**************************************************************************
 * Static function declarations.                                          *
 **************************************************************************
 */
static MCP2515_ERROR MCP2515_HalSpiInit(void);
static MCP2515_ERROR MCP2515_HalSpiStart(void);
static MCP2515_ERROR MCP2515_HalSpiEnd(void);
static uint8_t MCP2515_HalSpiTransfer(const uint8_t data);

/**************************************************************************
 * Extern variables.                                                      *
 **************************************************************************
 */
MCP2515_HAL_REG  Mcp2515HalReg = 
{
  .spiInit      = MCP2515_HalSpiInit,
  .spiTransfer  = MCP2515_HalSpiTransfer,
  .spiBegin     = MCP2515_HalSpiStart,
  .spiEnd       = MCP2515_HalSpiEnd
};

/**************************************************************************/

static MCP2515_ERROR MCP2515_HalSpiInit(void)
{
#if defined (snodeesp32)
  SPI = HardwareSPI;
#endif
  SPI.begin();
  pinMode(MCP2515_HAL_CE_PIN, OUTPUT);
  digitalWrite(MCP2515_HAL_CE_PIN, HIGH);
  SPI.endTransaction();
  return MCP2515_SUCCESS;
}

/**************************************************************************/

static MCP2515_ERROR MCP2515_HalSpiStart(void)
{
  SPI.beginTransaction(SPISettings(MCP2515_HAL_SPI_CLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(MCP2515_HAL_CE_PIN, LOW);
  return MCP2515_SUCCESS;
}

/**************************************************************************/

static MCP2515_ERROR MCP2515_HalSpiEnd(void)
{
  digitalWrite(MCP2515_HAL_CE_PIN, HIGH);
  SPI.endTransaction();
  return MCP2515_SUCCESS;
}

/**************************************************************************/

static uint8_t MCP2515_HalSpiTransfer(const uint8_t data)
{
  return SPI.transfer(data);
}
