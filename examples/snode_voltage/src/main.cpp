#include <Arduino.h>
#include "snode.h"

/**************************************************************************/

#define CAN_FRAME_SOURCE_ID  (40)

/**************************************************************************/

SNODE_ERROR SNODE_GetButtonData(void * data);
SNODE_ERROR SNODE_GetAdcBatteryData(void * data);

/**************************************************************************/

SNODE_SENSOR_REG SNODE_Buttons =
{
  .addr        = 0x00,
  .type        = SNODE_BUTTONS_SENSOR_TYPE,
  .mode        = SNODE_INPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_NONE_SENSOR_TYPE,
  .callback = SNODE_GetButtonData,
};

SNODE_SENSOR_REG SNODE_Battery =
{
  .addr        = 0x00,
  .type        = SNODE_INTERNAL_VOLTAGE_SENSOR_TYPE,
  .mode        = SNODE_INPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_NONE_SENSOR_TYPE,
  .callback = SNODE_GetAdcBatteryData,
};

/**************************************************************************/

SNODE_ERROR SNODE_GetAdcBatteryData(void * data)
{
  uint32_t *adc = (uint32_t *) data;

  *adc = analogRead(A3);

  return SNODE_SUCCESS;
}

/**************************************************************************/

SNODE_ERROR SNODE_GetButtonData(void * data)
{
  uint32_t *pButtons = (uint32_t *) data;

  *pButtons = !digitalRead(PD7);

  return SNODE_SUCCESS;
}

/**************************************************************************/

void setup()
{
  Serial.begin(115200);

  pinMode(A3, INPUT);         // ADC Battery
  pinMode(PD7, INPUT_PULLUP); // Button

  SNODE_Init((uint16_t)CAN_FRAME_SOURCE_ID);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_Battery);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_Buttons);
}

 void loop()
{
  SNODE_ProcessPacket();
}