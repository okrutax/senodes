#include <Arduino.h>
#include "snode.h"

/**************************************************************************/

#define CAN_FRAME_SOURCE_ID  (50)

#define SNODE_BUTTON_SB2      (PD7)
#define SNODE_BUTTON_S2       (6)
#define SNODE_BUTTON_S4       (12)

/**************************************************************************/

SNODE_ERROR SNODE_GetButtonSB2Data(void * data);
SNODE_ERROR SNODE_GetButtonS2Data(void * data);
SNODE_ERROR SNODE_GetButtonS4Data(void * data);

/**************************************************************************/

SNODE_SENSOR_REG SNODE_SB2Button =
{
  .addr        = 0x00,
  .type        = SNODE_BUTTONS_SENSOR_TYPE,
  .mode        = SNODE_INPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_NONE_SENSOR_TYPE,
  .callback = SNODE_GetButtonSB2Data,
};

SNODE_SENSOR_REG SNODE_S2Button =
{
  .addr        = 0x00,
  .type        = SNODE_BUTTONS_SENSOR_TYPE,
  .mode        = SNODE_INPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_NONE_SENSOR_TYPE,
  .callback = SNODE_GetButtonS2Data,
};

SNODE_SENSOR_REG SNODE_S4Button =
{
  .addr        = 0x00,
  .type        = SNODE_BUTTONS_SENSOR_TYPE,
  .mode        = SNODE_INPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_NONE_SENSOR_TYPE,
  .callback = SNODE_GetButtonS4Data,
};

/**************************************************************************/

SNODE_ERROR SNODE_GetButtonSB2Data(void * data)
{
  uint32_t *pButtons = (uint32_t *) data;

  *pButtons = !digitalRead(SNODE_BUTTON_SB2);

  return SNODE_SUCCESS;
}

SNODE_ERROR SNODE_GetButtonS2Data(void * data)
{
  uint32_t *pButtons = (uint32_t *) data;

  *pButtons = !digitalRead(SNODE_BUTTON_S2);

  return SNODE_SUCCESS;
}

SNODE_ERROR SNODE_GetButtonS4Data(void * data)
{
  uint32_t *pButtons = (uint32_t *) data;

  *pButtons = !digitalRead(SNODE_BUTTON_S4);

  return SNODE_SUCCESS;
}

/**************************************************************************/

void setup()
{
  Serial.begin(115200);

  pinMode(SNODE_BUTTON_SB2, INPUT_PULLUP);
  pinMode(SNODE_BUTTON_S2, INPUT_PULLUP);
  pinMode(SNODE_BUTTON_S4, INPUT_PULLUP);

  SNODE_Init((uint16_t)CAN_FRAME_SOURCE_ID);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_SB2Button);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_S2Button);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_S4Button);
}

 void loop()
{
  SNODE_ProcessPacket();
}