#include <Arduino.h>
#include "snode.h"

/**************************************************************************/

#define CAN_FRAME_SOURCE_ID           (70)
#define REMOTE_CAN_FRAME_SOURCE_ID60  (60)
#define REMOTE_CAN_FRAME_SOURCE_ID50  (50)

#define SNODE_LED_R         (PD2)
#define SNODE_LED_G         (PD3)
#define SNODE_LED_B         (PD4)

/**************************************************************************/

SNODE_ERROR SNODE_SetLedData(void *data);

/**************************************************************************/

SNODE_SENSOR_REG SNODE_LedSensor =
{
  .addr        = 0x00,
  .type        = SNODE_RGB_LED_SENSOR_TYPE,
  .mode        = SNODE_OUTPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_BUTTONS_SENSOR_TYPE,  // Output interface should be subscribed to the other input or control interfaces.
  .callback = SNODE_SetLedData,
};

/**************************************************************************/

SNODE_ERROR SNODE_SetLedData(void * data)
{
  PSNODE_SENSOR_CALLBACK_DATA  pSensorCallbackData = (PSNODE_SENSOR_CALLBACK_DATA) data;

  if ( pSensorCallbackData->type & SNODE_BUTTONS_SENSOR_TYPE )
  {
    if ( REMOTE_CAN_FRAME_SOURCE_ID60 == pSensorCallbackData->id )
    {
      if ( pSensorCallbackData->data )
      {
        digitalWrite(SNODE_LED_R, HIGH);
      }
      else
      {
        digitalWrite(SNODE_LED_R, LOW);
      }
    }

    if ( REMOTE_CAN_FRAME_SOURCE_ID50 == pSensorCallbackData->id )
    {
      if ( 0x02 == pSensorCallbackData->addr )
      {
        if ( pSensorCallbackData->data )
        {
          digitalWrite(SNODE_LED_G, HIGH);
        }
        else
        {
          digitalWrite(SNODE_LED_G, LOW);
        }
      }

      if ( 0x03 == pSensorCallbackData->addr )
      {
        if ( pSensorCallbackData->data )
        {
          digitalWrite(SNODE_LED_B, HIGH);
        }
        else
        {
          digitalWrite(SNODE_LED_B, LOW);
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

  pinMode(SNODE_LED_R, OUTPUT);
  pinMode(SNODE_LED_G, OUTPUT);
  pinMode(SNODE_LED_B, OUTPUT);

  SNODE_Init((uint16_t)CAN_FRAME_SOURCE_ID);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_LedSensor);
}

/**************************************************************************/

 void loop()
{
  SNODE_ProcessPacket();
}