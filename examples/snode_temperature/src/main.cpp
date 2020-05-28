#include "snode.h"

#include <OneWire.h>
#include <DallasTemperature.h>

/**************************************************************************/

//! Data wire is conntec to the Arduino digital pin 4.
#define ONE_WIRE_BUS 4

//! Setup a oneWire instance to communicate with any OneWire devices.
OneWire oneWire(ONE_WIRE_BUS);

//! Pass our oneWire reference to Dallas Temperature sensor.
DallasTemperature sensors(&oneWire);

/**************************************************************************/

#define CAN_FRAME_SOURCE_ID  (30)

/**************************************************************************/

SNODE_ERROR SNODE_GetTemperatureData(void * data);

/**************************************************************************/

SNODE_SENSOR_REG SNODE_TemperatureSensor =
{
  .addr       = 0x00,
  .type       = SNODE_TEMPERATURE_SENSOR_TYPE,
  .mode       = SNODE_INPUT_SENSOR_MODE_TYPE,
  .subscriber = SNODE_NONE_SENSOR_TYPE,
  .callback   = SNODE_GetTemperatureData,
};

/**************************************************************************/

SNODE_ERROR SNODE_GetTemperatureData(void * data)
{
  DeviceAddress deviceAddress;
  uint32_t      *pTemp = (uint32_t *) data;

  sensors.getAddress(deviceAddress, 0);
  //! 0.0078125 * sensors.getTemp((uint8_t*) deviceAddress) to convert from raw to Celsius.
  *pTemp = sensors.getTemp((uint8_t*) deviceAddress);

  return SNODE_SUCCESS;
}

/**************************************************************************/

void setup()
{
  Serial.begin(115200);

  sensors.begin();
  sensors.setWaitForConversion(false);

  SNODE_Init((uint16_t)CAN_FRAME_SOURCE_ID);
  SNODE_Registration((PSNODE_SENSOR_REG)&SNODE_TemperatureSensor);
}

 void loop()
{
  //! Calling sensors.requestTemperatures() to issue a global temperature.
  sensors.requestTemperatures();

  SNODE_ProcessPacket();
}
