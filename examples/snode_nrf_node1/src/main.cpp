#include <Arduino.h>
#include "nrf24l01.h"
#include "mcp2515.h"
#include "Arduino-hal/mcp2515_hal.h"
#include "Arduino-hal/nrf24l01_hal.h"

/**************************************************************************/

void setup()
{
  Serial.begin(115200);
  MCP2515_Init((PMCP2515_HAL_REG)&Mcp2515HalReg);
  NRF24L01_Init((PNRF24L01_HAL_REG)&Nrf24l01HalReg, 12, 90);
}

/**************************************************************************/

 void loop()
{
  CAN_FRAME canReceive;
  CAN_FRAME canSend;

  memset(&canReceive, 0x00, sizeof(canReceive));

  if ( MCP2515_SUCCESS == MCP2515_ReadMessage(&canReceive) )
  {
    NRF24L01_SendData(13, (uint8_t *)&canReceive, sizeof(canReceive));
    Serial.println("Data is sent");
  }

  if ( NRF24L01_HasData() )
  {
    memset(&canSend, 0x00, sizeof(canSend));
    NRF24L01_ReadData((uint8_t *)&canSend);
    Serial.println("Data is received");
    MCP2515_SendMessage(&canSend);
  }
//delay(1000);
}