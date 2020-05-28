#include <Arduino.h>
#include "snode.h"

#include <SSD_13XX.h>

/**************************************************************************/

SSD_13XX tft = SSD_13XX(11, 9, 8);

/**************************************************************************/

#define CAN_FRAME_SOURCE_ID  (20)

/**************************************************************************/

SNODE_ERROR SNODE_SetDisplayData(void *data);

/**************************************************************************/

SNODE_SENSOR_REG SNODE_DisplaySensor =
{
  .addr        = 0x00,
  .type        = SNODE_DISPLAY_SENSOR_TYPE,
  .mode        = SNODE_OUTPUT_SENSOR_MODE_TYPE,
  .subscriber  = SNODE_BUTTONS_SENSOR_TYPE | SNODE_TEMPERATURE_SENSOR_TYPE | SNODE_JOYSTICK_SENSOR_TYPE | SNODE_INTERNAL_VOLTAGE_SENSOR_TYPE, // Output interface should be subscribed to the other input or control interfaces.
  .callback = SNODE_SetDisplayData,
};

/**************************************************************************/

SNODE_ERROR SNODE_SetDisplayData(void * data)
{
  PSNODE_SENSOR_CALLBACK_DATA  pSensorCallbackData = (PSNODE_SENSOR_CALLBACK_DATA) data;

  if ( pSensorCallbackData->type & SNODE_JOYSTICK_SENSOR_TYPE )
  {
    PSNODE_JOYXY joyXY = (PSNODE_JOYXY) &pSensorCallbackData->data;

    //! Connected.
    if ( pSensorCallbackData->status )
    {
      tft.ringMeter(joyXY->x , 0, 1024, 0, 0, 10, 3, WHITE, 170, 20);
      tft.setCursor(15, 13);
      tft.setTextColor(WHITE);
      tft.setTextScale(2);
      tft.print("X");

      tft.ringMeter(joyXY->y, 0, 1024, 50, 0, 10, 3, WHITE, 170, 20);
      tft.setCursor(65, 13);
      tft.setTextColor(WHITE);
      tft.setTextScale(2);
      tft.print("Y");
    }
    else //! Disconnected.
    {
      tft.clearScreen();
    }
  }

  if ( pSensorCallbackData->type & SNODE_BUTTONS_SENSOR_TYPE )
  {
    if ( pSensorCallbackData->status )
    {
      if ( pSensorCallbackData->data )
      {
        Serial.print("The button is pressed, Node: id = ");
        Serial.print(pSensorCallbackData->id);
        Serial.print(", sensor addr = ");
        Serial.print(pSensorCallbackData->addr);
        Serial.println("");
      }
      else
      {
      }
    }
    else
    {
    }
  }

  if ( pSensorCallbackData->type & SNODE_TEMPERATURE_SENSOR_TYPE )
  {
    //! Connected.
    if ( pSensorCallbackData->status )
    {
      tft.setTextScale(0);
      tft.setCursor(0, 45);
      tft.setTextColor(BLUE, BLACK);
      tft.print("Temperature = ");
      tft.print(int32_t(0.0078125 * pSensorCallbackData->data));
      tft.println("  ");
    }
    else
    {
      tft.clearScreen();
    }
  }

  if ( pSensorCallbackData->type & SNODE_INTERNAL_VOLTAGE_SENSOR_TYPE )
  {
    //! Connected.
    if ( pSensorCallbackData->status )
    {
      tft.setTextScale(0);
      tft.setCursor(0, 55);
      tft.setTextColor(BLUE, BLACK);
      tft.print("Voltage = ");
      tft.print(pSensorCallbackData->data);
      tft.println("  ");
    }
    else
    {
      tft.clearScreen();
    }
  }

  return SNODE_SUCCESS;
}

/**************************************************************************/

void setup()
{
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(0);

  SNODE_Init((uint16_t)CAN_FRAME_SOURCE_ID);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_DisplaySensor);
}

/**************************************************************************/

 void loop()
{
  SNODE_ProcessPacket();
}