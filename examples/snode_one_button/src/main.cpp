#include <Arduino.h>
#include "snode.h"

/**************************************************************************/

#define CAN_FRAME_SOURCE_ID  (60)

#define SNODE_BUTTON         (PD2)

/**************************************************************************/

SNODE_ERROR SNODE_GetButtonData(void * data);

/**************************************************************************/

SNODE_SENSOR_REG SNODE_Button =
{
  .addr        = 0x00,
  .type        = SNODE_BUTTONS_SENSOR_TYPE,
  .mode        = SNODE_INPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_NONE_SENSOR_TYPE,
  .callback = SNODE_GetButtonData,
};

/**************************************************************************/

SNODE_ERROR SNODE_GetButtonData(void * data)
{
  uint32_t *pButton = (uint32_t *) data;

  *pButton = digitalRead(SNODE_BUTTON);

  return SNODE_SUCCESS;
}

/**************************************************************************/

void setup()
{
  Serial.begin(115200);

  pinMode(SNODE_BUTTON, INPUT_PULLUP);

  SNODE_Init((uint16_t)CAN_FRAME_SOURCE_ID);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_Button);
}

 void loop()
{
  SNODE_ProcessPacket();
}