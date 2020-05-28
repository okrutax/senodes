#include <Arduino.h>
#include "snode.h"

/**************************************************************************/

#define CAN_FRAME_SOURCE_ID  (10)

/**************************************************************************/

SNODE_ERROR SNODE_GetButtonData(void * data);
SNODE_ERROR SNODE_GetJoystickData(void * data);

/**************************************************************************/

SNODE_SENSOR_REG SNODE_Buttons =
{
  .addr        = 0x00,
  .type        = SNODE_BUTTONS_SENSOR_TYPE,
  .mode        = SNODE_INPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_NONE_SENSOR_TYPE,
  .callback = SNODE_GetButtonData,
};

SNODE_SENSOR_REG SNODE_Joystick =
{
  .addr        = 0x00,
  .type        = SNODE_JOYSTICK_SENSOR_TYPE,
  .mode        = SNODE_INPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_NONE_SENSOR_TYPE,
  .callback = SNODE_GetJoystickData,
};

/**************************************************************************/

SNODE_ERROR SNODE_GetJoystickData(void * data)
{
  PSNODE_JOYXY joyXY = (PSNODE_JOYXY) data;

  joyXY->x = analogRead(A4);
  joyXY->y = analogRead(A5);

  return SNODE_SUCCESS;
}

/**************************************************************************/

SNODE_ERROR SNODE_GetButtonData(void * data)
{
  uint32_t *pButtonss = (uint32_t *) data;

  *pButtonss = !digitalRead(PD7);

  return SNODE_SUCCESS;
}

/**************************************************************************/

void setup()
{
  Serial.begin(115200);

  pinMode(A4, INPUT); // X
  pinMode(A5, INPUT); // Y
  pinMode(PD7, INPUT_PULLUP); // Button

  SNODE_Init((uint16_t)CAN_FRAME_SOURCE_ID);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_Joystick);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_Buttons);
}

 void loop()
{
  SNODE_ProcessPacket();
}