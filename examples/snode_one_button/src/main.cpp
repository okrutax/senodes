#include <Arduino.h>
#include "snode.h"

/**************************************************************************/

#define CAN_FRAME_SOURCE_ID           (60)
#define REMOTE_CAN_FRAME_SOURCE_ID50  (50)

#define SNODE_BUTTON         (PD2)
#define SNODE_LED            (PD3)

/**************************************************************************/

SNODE_ERROR SNODE_GetButtonData(void * data);
SNODE_ERROR SNODE_SetLedData(void *data);

/**************************************************************************/

SNODE_SENSOR_REG SNODE_Button =
{
  .addr        = 0x00,
  .type        = SNODE_BUTTONS_SENSOR_TYPE,
  .mode        = SNODE_INPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_NONE_SENSOR_TYPE,
  .callback = SNODE_GetButtonData,
};

SNODE_SENSOR_REG SNODE_LedSensor =
{
  .addr        = 0x00,
  .type        = SNODE_LED_SENSOR_TYPE,
  .mode        = SNODE_OUTPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_BUTTONS_SENSOR_TYPE, // Output interface should be subscribed to the other input or control interfaces.
  .callback = SNODE_SetLedData,
};

/**************************************************************************/

SNODE_ERROR SNODE_GetButtonData(void * data)
{
  uint32_t *pButton = (uint32_t *) data;

  *pButton = !digitalRead(SNODE_BUTTON);

  return SNODE_SUCCESS;
}

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

  pinMode(SNODE_BUTTON, INPUT_PULLUP);
  pinMode(SNODE_LED, OUTPUT);

  SNODE_Init((uint16_t)CAN_FRAME_SOURCE_ID);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_Button); //! The Button Address is 0x01.
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_LedSensor); //! The Led Address is 0x02.
}

 void loop()
{
  SNODE_ProcessPacket();
}