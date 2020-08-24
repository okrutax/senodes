#include <Arduino.h>
#include "snode.h"

/**
 * Pin to pin connection.
 * MCP2515 - ESP32
 * SS      - 15
 * MISO    - 12
 * MOSI    - 13
 * SCK     - 14
 **/

/**************************************************************************/

#define CAN_FRAME_SOURCE_ID           (90)
#define REMOTE_CAN_FRAME_SOURCE_ID50  (50)

#define SNODE_LED                     (2)

/**************************************************************************/

SNODE_ERROR SNODE_SetLedData(void *data);

/**************************************************************************/

SNODE_SENSOR_REG SNODE_LedSensor =
{
  .addr        = 0x00,
  .type        = SNODE_LED_SENSOR_TYPE,
  .mode        = SNODE_OUTPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_BUTTONS_SENSOR_TYPE, // Output interface should be subscribed to the other input or control interfaces.
  .callback = SNODE_SetLedData,
};

/**************************************************************************/

SNODE_ERROR SNODE_SetLedData(void * data)
{
  PSNODE_SENSOR_CALLBACK_DATA  pSensorCallbackData = (PSNODE_SENSOR_CALLBACK_DATA) data;

  if ( pSensorCallbackData->type & SNODE_BUTTONS_SENSOR_TYPE )
  {
    if ( REMOTE_CAN_FRAME_SOURCE_ID50 == pSensorCallbackData->id )
    {
      if ( 0x01 == pSensorCallbackData->addr )
      {
        if ( pSensorCallbackData->data )
        {
          digitalWrite(SNODE_LED, HIGH);
        }
        else
        {
          digitalWrite(SNODE_LED, LOW);
        }
      }
    }
  }

  return SNODE_SUCCESS;
}
/**************************************************************************/

void setup()
{
  Serial.begin(115200);
  Serial.println("Testing");

  pinMode(SNODE_LED, OUTPUT);
  digitalWrite(SNODE_LED, LOW);

  SNODE_Init((uint16_t)CAN_FRAME_SOURCE_ID);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_LedSensor); //! The Led Address is 0x01.
}

 void loop()
{
  SNODE_ProcessPacket();
}