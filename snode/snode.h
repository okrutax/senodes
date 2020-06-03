/**************************************************************************
 * @file       snode.h                                                    *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      The snode project.                                         *
 *************************************************************************/

#ifndef _SNODE_H_ // Sensor Node
#define _SNODE_H_ 

#ifdef _cplusplus_
extern "C"
{
#endif

#include <stdint.h>
#include <string.h>
#include "snode_cnfg.h"

/**************************************************************************
 * Configuration of SNODE (Sensor Node).                                  *
 **************************************************************************
 */
#define _SNODE_DEBUG_  //! Uncommet it to show debug information.

#define SNODE_MAX_NUM_OF_SENSORS             (0x04) //! The max number of sensors for each node (max value is 0x0F).
#define SNODE_CACHE_ENTRY_MAX_NUM_OF_NODES   (0x08) //! The max number of nodes in one network.

#define SNODE_CACHE_ENTRY_POLL_TIMEOUT       (1600)  //! 1600 ms. Max timeout of the entry polling.
#define SNODE_CACHE_ENTRY_POLL_SEND_TIME     (110)  //! 110 ms. Sending cache every 150 ms.

#define SNODE_CACHE_ENTRY_POLL_CHECK_TIME    (2)   //! Checking received cache nodes every 2 ms.

#define SNODE_TRANSPORT_TYPE_POLL_TIME       (200)
#define SNODE_TRANSPORT_SUBSCRIBER_POLL_TIME (150)

#define SNODE_RESERVED_ADDR                  (0x01) //! The address of sensors for each node is started from 0x01. And generated automatically when a new record is added by the SNODE_Registration function.

/**************************************************************************/

typedef enum _SNODE_ERROR
{
  SNODE_SUCCESS  = 0x00,
  SNODE_FAIL     = 0x01
}SNODE_ERROR, *PSNODE_ERROR;

/**************************************************************************
 * SNODE Sensor type.                                                     *
 **************************************************************************
 */
typedef enum _SNODE_SENSOR_MODE_TYPE
{
  SNODE_INPUT_SENSOR_MODE_TYPE   = 0x00,
  SNODE_OUTPUT_SENSOR_MODE_TYPE  = 0x01
//SNODE_CONTROL_SENSOR_MODE_TYPE = 0x02
}SNODE_SENSOR_MODE_TYPE, *PSNODE_SENSOR_MODE_TYPE;

/**************************************************************************/

typedef enum _SNODE_SENSOR_TYPE
{
  SNODE_NONE_SENSOR_TYPE              = 0x00,

//! Output interfaces - SNODE_OUTPUT_SENSOR_MODE_TYPE.
  SNODE_DC_MOTOR_SENSOR_TYPE          = 0x01,  //! DC motor.
  SNODE_STEPPER_MOTOR_SENSOR_TYPE     = 0x02,  //! Stepper motor.
  SNODE_SERVO_SENSOR_TYPE             = 0x04,  //! Servo motor.
  SNODE_LED_SENSOR_TYPE               = 0x08,  //! Led.
  SNODE_RGB_LED_SENSOR_TYPE           = 0x10,  //! RGB led.
  SNODE_MATRIX_LED_SENSOR_TYPE        = 0x20,  //! Matrix led.
  SNODE_SPEAKER_SENSOR_TYPE           = 0x40,  //! Speaker sensor.
  SNODE_BUZZER_SENSOR_TYPE            = 0x80,  //! Buzzer.
  SNODE_DISPLAY_SENSOR_TYPE           = 0x100, //! Display.

//! Input interfaces - SNODE_INPUT_SENSOR_MODE_TYPE.
  SNODE_COLOR_SENSOR_TYPE             = 0x01,   //! Color.
  SNODE_ULTRASONIC_SENSOR_TYPE        = 0x02,   //! Ultrasonic.
  SNODE_GYROSCOPE_SENSOR_TYPE         = 0x04,   //! Gyroscope.
  SNODE_LIGHT_INTENSITY_SENSOR_TYPE   = 0x08,   //! Light intensity.
  SNODE_SOUND_INTENSITY_SENSOR_TYPE   = 0x10,   //! Sound intensity.
  SNODE_TEMPERATURE_SENSOR_TYPE       = 0x20,   //! Temperature.
  SNODE_HUMIDITY_SENSOR_TYPE          = 0x40,   //! Humidity.
  SNODE_SOIL_SENSOR_TYPE              = 0x80,   //! Soil.
  SNODE_PIR_SENSOR_TYPE               = 0x100,  //! Pir.
  SNODE_DSP_SENSOR_TYPE               = 0x200,  //! Dsp.
  SNODE_RPM_SENSOR_TYPE               = 0x400,  //! Revolutions per minute.
  SNODE_INTERNAL_VOLTAGE_SENSOR_TYPE  = 0x800,  //! Internal voltage.
  SNODE_EXTERNAL_VOLTAGE_SENSOR_TYPE  = 0x1000, //! External voltage.
//! Control interfaces.
  SNODE_BUTTONS_SENSOR_TYPE           = 0x2000, //! Buttons.
  SNODE_KNOB_SENSOR_TYPE              = 0x1000, //! Knob.
  SNODE_JOYSTICK_SENSOR_TYPE          = 0x4000, //! Joystick.
  SNODE_ENCODER_SENSOR_TYPE           = 0x8000, //! Encoder.
}SNODE_SENSOR_TYPE, *PSNODE_SENSOR_TYPE;

/**************************************************************************
 * SNODE Registration.                                                    *
 **************************************************************************
 */
typedef SNODE_ERROR (* SNODE_SENSOR_CALLBACK)(void *);

/**************************************************************************/

typedef struct _SNODE_SENSOR_REG //! Registration.
{
  uint8_t                     addr; //! Generated automatically when a new record is added by the SNODE_Registration function.
  const uint16_t              type; //! SNODE_SENSOR_TYPE.
  const uint8_t               mode; //! SNODE_SENSOR_MODE_TYPE.
  const uint16_t              subscriber;
  const SNODE_SENSOR_CALLBACK callback;
}SNODE_SENSOR_REG, *PSNODE_SENSOR_REG;

/**************************************************************************
 * SNODE Commands.                                                        *
 **************************************************************************
 */
typedef enum _SNODE_CMD
{
  SNODE_DISCOVER_CMD     = 0x08,
//SNODE_MEASUREMENT_CMD  = 0x0A,
  SNODE_TRANSPORT_CMD    = 0x0B
}SNODE_CMD, *PSNODE_CMD;

/**************************************************************************/

typedef void (* SNODE_CMD_HANDLER)(void * request , uint8_t * requestLength);

/**************************************************************************/

typedef struct _SNODE_GET_CMD_HANDLER
{
  SNODE_CMD          cmd;
  SNODE_CMD_HANDLER  handler;
}SNODE_GET_CMD_HANDLER, *PSNODE_GET_CMD_HANDLER;

/**************************************************************************
 * SNODE Frame head.                                                      *
 **************************************************************************
 */
typedef struct _SNODE_FRAME_HEAD_CMD
{
  uint8_t cmd;    //! SNODE_CMD (5 bits).
  uint8_t data[];
}SNODE_FRAME_HEAD_CMD, *PSNODE_FRAME_HEAD_CMD;

/**************************************************************************
 * SNODE Discover command.                                                *
 **************************************************************************
 */
typedef struct _SNODE_BROADCAST_TYPE
{
  uint16_t sourceId        :11; //! SID[10:0]: Standard Identifier bits.
  uint16_t numberOfSensors :4 ; //! Sensors which are used for one node.
  uint16_t reserved        :1 ;
}SNODE_BROADCAST_TYPE, *PSNODE_BROADCAST_TYPE;

/**************************************************************************/

typedef struct _SNODE_DISCOVER_CMD_REQ //! Don't need a response.
{
  //! TODO: Should be added physical layer types (MCP or NRF).
  SNODE_BROADCAST_TYPE node;
  uint8_t              checksum;
}SNODE_DISCOVER_CMD_REQ, *PSNODE_DISCOVER_CMD_REQ;

/**************************************************************************
 * SNODE Transport command.                                               *
 **************************************************************************
 */
typedef enum _SNODE_TRANSPORT_PACKET_CMD
{
  SNODE_TRANSPORT_TYPE_CMD       = 0x01,
  SNODE_TRANSPORT_SUBSCRIBER_CMD = 0x02
}SNODE_TRANSPORT_PACKET_CMD, *PSNODE_TRANSPORT_PACKET_CMD;

/**************************************************************************/

typedef enum _SNODE_TRANSPORT_CMD_FLAGS
{
  SNODE_TRANSPORT_CMD_REQ_FLAG = 0x00,
  SNODE_TRANSPORT_CMD_RSP_FLAG = 0x01
}SNODE_TRANSPORT_CMD_FLAGS, *PSNODE_TRANSPORT_CMD_FLAGS;

/**************************************************************************
 * SNODE Transport frame head.                                            *
 **************************************************************************
 */
typedef struct _SNODE_TRANSPORT_FRAME_HEAD
{
  uint16_t sourceId :11;
  uint16_t cmd      :3 ; //! SNODE_TRANSPORT_PACKET_CMD.
  uint16_t flag     :1 ; //! Request or response. SNODE_TRANSPORT_CMD_FLAGS.
  uint16_t reserved :1 ;
  uint8_t  data[];
}SNODE_TRANSPORT_FRAME_HEAD, *PSNODE_TRANSPORT_FRAME_HEAD;

/**************************************************************************
 * SNODE Transport type command.                                          *
 **************************************************************************
 */
typedef struct _SNODE_NODE_DATA_TYPE
{
  uint32_t addrOfSensor     : 4;
  uint32_t type             :16; //! SNODE_SENSOR_TYPE.
  uint32_t mode             : 2; //! SNODE_SENSOR_MODE_TYPE.
  uint32_t reserved         :10;
  uint16_t subscriber;
}SNODE_NODE_DATA_TYPE, *PSNODE_NODE_DATA_TYPE;

/**************************************************************************
 * SNODE Transport subscriber command.                                    *
 **************************************************************************
 */
typedef struct _SNODE_SUBSCRIBER_FRAME_HEAD
{
  uint8_t numOfSubscriberData;
  uint8_t data[];             //! Pointer to SNODE_SUBSCRIBER_DATA_TYPE.
//uint8_t checksum;           //! Added to the end of subscriber of the SNODE_SUBSCRIBER_FRAME_HEAD structure.
}SNODE_SUBSCRIBER_FRAME_HEAD, *PSNODE_SUBSCRIBER_FRAME_HEAD;

/**************************************************************************/

typedef struct _SNODE_SUBSCRIBER_DATA_TYPE
{
  uint16_t destAddrOfSensor       :4;
  uint16_t sourceAddrOfSensor     :4;
  uint16_t sizeOfSourceDataSensor :6; //! TODO:
  uint16_t reserved               :2;
  uint32_t sensorData;
}SNODE_SUBSCRIBER_DATA_TYPE, *PSNODE_SUBSCRIBER_DATA_TYPE;

/**************************************************************************
 * Configuration of SNODE ISO TP (transport protocol).                    *
 **************************************************************************
 */
#define SNODE_TP_BLOCK_SIZE               (2)   //! Max number of messages.
#define SNODE_TP_DATA_SIZE                (3)   //! ( min = 3, max = 4 )

#define SNODE_TP_ST_MIN                   (0)   //! The STmin parameter value specifies the minimum time gap allowed between
                                                //! the transmission of consecutive frame network protocol data units.
                                                //! In this version of implementation the STmin parameter should be always set to zero.

#define SNODE_TP_SN_MAX                   (0xFF)

#define SNODE_TP_N_BS_TIMEOUT             (200) //! In ms.
#define SNODE_TP_N_CR_TIMEOUT             (200) //! In ms.

#define SNODE_TP_MAX_BUFFER_MESSAGE_SIZE  (32)

/**************************************************************************/

typedef enum _SNODE_TP_ERROR
{
  SNODE_TP_SUCCESS     = 0x00,
  SNODE_TP_FAIL        = 0x01
}SNODE_TP_ERROR, *PSNODE_TP_ERROR;

/**************************************************************************
 * SNODE TP Sender status.                                                *
 **************************************************************************
 */
typedef enum _SNODE_TP_SEND_STATUS_TYPE
{
  SNODE_TP_SEND_STATUS_IDLE       = 0x00,
  SNODE_TP_SEND_STATUS_INPROGRESS = 0x01,
  SNODE_TP_SEND_STATUS_ERROR      = 0x02
}SNODE_TP_SEND_STATUS_TYPE, *PSNODE_TP_SEND_STATUS_TYPE;

/**************************************************************************
 * SNODE TP Receiver status.                                              *
 **************************************************************************
 */
typedef enum _SNODE_TP_RECEIVE_STATUS_TYPE
{
  SNODE_TP_RECEIVE_STATUS_IDLE       = 0x00,
  SNODE_TP_RECEIVE_STATUS_INPROGRESS = 0x01,
  SNODE_TP_RECEIVE_STATUS_FULL       = 0x02
}SNODE_TP_RECEIVE_STATUS_TYPE, *PSNODE_TP_RECEIVE_STATUS_TYPE;

/**************************************************************************
 * SNODE TP Frame defination.                                             *
 **************************************************************************
 */
typedef struct _SNODE_TP_PCI_TYPE
{
  uint8_t reserved :4;
  uint8_t type     :4;
  uint8_t data[];
}SNODE_TP_PCI_TYPE, *PSNODE_TP_PCI_TYPE;

/**************************************************************************/

typedef struct _SNODE_TP_SINGLE_FRAME
{
  uint8_t dl   :4; //! Data length.
  uint8_t type :4;
  uint8_t data[];
}SNODE_TP_SINGLE_FRAME, *PSNODE_TP_SINGLE_FRAME;

/**************************************************************************/

typedef struct _SNODE_TP_FIRST_FRAME
{
  uint8_t dlHigh :4;
  uint8_t type   :4;
  uint8_t dlLow;
  uint8_t data[];
}SNODE_TP_FIRST_FRAME, *PSNODE_TP_FIRST_FRAME;

/**************************************************************************/

typedef struct _SNODE_TP_CONSECUTIVE_FRAME
{
  uint8_t sn   :4; //! Sequence number.
  uint8_t type :4;
  uint8_t data[];
}SNODE_TP_CONSECUTIVE_FRAME, *PSNODE_TP_CONSECUTIVE_FRAME;

/**************************************************************************/

typedef struct _SNODE_TP_FLOW_CONTROL
{
  uint8_t fs   :4; //! Flow status.
  uint8_t type :4;
  uint8_t bs;      //! Block size.
  uint8_t stMin;
}SNODE_TP_FLOW_CONTROL, *PSNODE_TP_FLOW_CONTROL;

/**************************************************************************
 * Protocol Control Information (PCI) types, for identifying each frame   *
 * of an SNODE-ISO-TP message.                                            *
 **************************************************************************
 */
typedef enum _SNODE_TP_PCI_FRAME_TYPE
{
  SNODE_TP_PCI_TYPE_SINGLE_FRAME       = 0x00,
  SNODE_TP_PCI_TYPE_FIRST_FRAME        = 0x01,
  SNODE_TP_PCI_TYPE_CONSECUTIVE_FRAME  = 0x02,
  SNODE_TP_PCI_TYPE_FLOW_CONTROL_FRAME = 0x03
}SNODE_TP_PCI_FRAME_TYPE, *PSNODE_TP_PCI_FRAME_TYPE;

/**************************************************************************
 * Protocol Control Information (PCI) flow control identifiers.           *
 **************************************************************************
 */
typedef enum _SNODE_TP_PCI_FLOW_STATUS_TYPE
{
  SNODE_TP_PCI_FLOW_STATUS_CONTINUE = 0x00,
  SNODE_TP_PCI_FLOW_STATUS_WAIT     = 0x01,
  SNODE_TP_PCI_FLOW_STATUS_OVERFLOW = 0x02
}SNODE_TP_PCI_FLOW_STATUS_TYPE, *PSNODE_TP_PCI_FLOW_STATUS_TYPE;

/**************************************************************************
 * SNODE TP Link ctx.                                                     *
 **************************************************************************
 */
typedef struct _SNODE_TP_LINK_FRAME_CTX
{
  //! Message buffer.
  uint8_t buffer[SNODE_TP_MAX_BUFFER_MESSAGE_SIZE];
  uint8_t size;
  uint8_t offset;

  //! Multi-frame flags.
  uint8_t  sn;
  uint16_t bs;//! Block size.
  uint8_t  status;
}SNODE_TP_LINK_FRAME_CTX, *PSNODE_TP_LINK_FRAME_CTX;

/**************************************************************************/

typedef struct _SNODE_TP_LINK_TIMER_CTX
{
  uint32_t nBs; //! Timeout on the Sender side.
  uint32_t nCr; //! Timeout on the Receiver side.
}SNODE_TP_LINK_TIMER_CTX, *PSNODE_TP_LINK_TIMER_CTX;

/**************************************************************************/

typedef struct _SNODE_TP_LINK_CTX
{
  uint16_t                id; //! Used to reply the consecutive frames.
  SNODE_TP_LINK_FRAME_CTX send;
  SNODE_TP_LINK_FRAME_CTX receive;
  SNODE_TP_LINK_TIMER_CTX timer;
}SNODE_TP_LINK_CTX, *PSNODE_TP_LINK_CTX;

/**************************************************************************/

typedef SNODE_TP_ERROR (* SNODE_TP_CMD_HANDLER)(PSNODE_TP_LINK_CTX pLink, void * pMessage, uint8_t length);

/**************************************************************************/

typedef struct _SNODE_TP_PCI_CMD_HANDLER
{
  SNODE_TP_PCI_FRAME_TYPE type;
  SNODE_TP_CMD_HANDLER    handler;
}SNODE_TP_PCI_CMD_HANDLER, *PSNODE_TP_PCI_CMD_HANDLER;

/**************************************************************************
 * SNODE Cache entry and sensor ctxs.                                     *
 **************************************************************************
 */
typedef struct _SNODE_CACHE_ENTRY
{
  SNODE_BROADCAST_TYPE node;
  bool                 dataNodeIsRead;
  SNODE_NODE_DATA_TYPE dataNode[SNODE_MAX_NUM_OF_SENSORS];
  SNODE_TP_LINK_CTX    linkCtx;    //! Transport protocol ctx for each node.
  uint32_t             timestamp;  //! Timestamp.
}SNODE_CACHE_ENTRY, *PSNODE_CACHE_ENTRY;

/**************************************************************************/

typedef struct _SNODE_SENSOR_CTX
{
  uint8_t                      currentIndex; //! It uses for pSensor[SNODE_MAX_NUM_OF_SENSORS].
  PSNODE_SENSOR_REG            pSensor[SNODE_MAX_NUM_OF_SENSORS];
  PSNODE_CACHE_ENTRY           pCacheNode;
  uint8_t                      cacheNodeLength;
  PSNODE_GET_CMD_HANDLER       pGetCmdHandler;
  uint8_t                      getCmdHandlerLength;
  uint16_t                     canFrameSourceId;
}SNODE_SENSOR_CTX, *PSNODE_SENSOR_CTX;

/**************************************************************************
 * SNODE Sensor callback data.                                            *
 **************************************************************************
 */
typedef struct _SNODE_SENSOR_CALLBACK_DATA
{
  uint16_t id;
  uint8_t  addr;
  uint16_t type;
  bool     status; //! Connected or disconnected.
  uint32_t data;
}SNODE_SENSOR_CALLBACK_DATA, *PSNODE_SENSOR_CALLBACK_DATA;

/**************************************************************************
 * Global function declarations.                                          *
 **************************************************************************
 */
SNODE_ERROR SNODE_Init(uint16_t canFrameSourceId);
SNODE_ERROR SNODE_Registration(const PSNODE_SENSOR_REG pSensor);
SNODE_ERROR SNODE_ProcessPacket(void);

#ifdef _cplusplus
}
#endif

#endif // _SNODE_H_
