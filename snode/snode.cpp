/**************************************************************************
 * @file       snode.cpp                                                  *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      The snode project.                                         *
 *************************************************************************/

#include <stdlib.h>
#include "snode.h"
#include "mcp2515.h"
#include "Arduino-hal/mcp2515_hal.h"
#include "timer.h"
#include "Arduino-hal/time_hal.h"

#if defined(_SNODE_DEBUG_)
#include "Arduino-hal/debug_hal.h"
#endif

/**************************************************************************
 * SNODE Static function declarations.                                    *
 **************************************************************************
 */
static void SNODE_DiscoverCmd(void * request , uint8_t * requestLength);
static void SNODE_TransportCmd(void * request , uint8_t * requestLength);

/**************************************************************************/

static PSNODE_SENSOR_REG   SNODE_GetSensorCtx(uint8_t addr);
static PSNODE_CACHE_ENTRY  SNODE_SearchCacheNode(const uint16_t id);
static SNODE_ERROR         SNODE_AddCacheNode(const PSNODE_BROADCAST_TYPE pNode);

/**************************************************************************/

static bool SNODE_CheckPollCacheNode(void *);
static bool SNODE_SendPollCacheNode(void *);

/**************************************************************************/

static bool SNODE_SendTransportTypeReqFrame(void *);                   //! Request.
static void SNODE_SendTransportTypeRspFrame(PSNODE_TP_LINK_CTX pLink); //! Response.
static bool SNODE_TransportSubscriberPoll(void *);

/**************************************************************************/

static uint8_t SNODE_Checksum(uint8_t * data, uint8_t length);

/**************************************************************************
 * SNODE Static variables.                                                *
 **************************************************************************
 */
const static SNODE_GET_CMD_HANDLER  snode_get_cmd[] =
{
  {SNODE_DISCOVER_CMD,          SNODE_DiscoverCmd },
  {SNODE_TRANSPORT_CMD,         SNODE_TransportCmd}
};

static SNODE_SENSOR_CTX  snode_ctx;
static SNODE_CACHE_ENTRY snode_cache_entry[SNODE_CACHE_ENTRY_MAX_NUM_OF_NODES];

/**************************************************************************
 * SNODE TP Static function declarations.                                 *
 **************************************************************************
 */
static SNODE_TP_ERROR SNODE_TP_SendSingleFrame(PSNODE_TP_LINK_CTX pLink);
static SNODE_TP_ERROR SNODE_TP_SendFirstFrame(PSNODE_TP_LINK_CTX pLink);
static SNODE_TP_ERROR SNODE_TP_SendConsecutiveFrame(PSNODE_TP_LINK_CTX pLink);
static SNODE_TP_ERROR SNODE_TP_SendFlowControl(PSNODE_TP_LINK_CTX pLink, uint8_t flowStatus, uint8_t blockSize);

/**************************************************************************/

static SNODE_TP_ERROR SNODE_TP_ReceiveSingleFrame(PSNODE_TP_LINK_CTX pLink, void * pMessage, uint8_t length);
static SNODE_TP_ERROR SNODE_TP_ReceiveFirstFrame(PSNODE_TP_LINK_CTX pLink, void * pMessage, uint8_t length);
static SNODE_TP_ERROR SNODE_TP_ReceiveConsecutiveFrame(PSNODE_TP_LINK_CTX pLink, void * pMessage, uint8_t length);
static SNODE_TP_ERROR SNODE_TP_ReceiveFlowControlFrame(PSNODE_TP_LINK_CTX pLink, void * pMessage, uint8_t length);

/**************************************************************************/

static SNODE_TP_ERROR  SNODE_TP_SendCanMessage(uint16_t id, const uint8_t * pData, const uint8_t size);
static void            SNODE_TP_SnWrap(uint8_t *pSn);

/**************************************************************************/

static void           SNODE_TP_InitLink(PSNODE_TP_LINK_CTX pLink, uint16_t id);
static void           SNODE_TP_Poll(PSNODE_TP_LINK_CTX pLink);                                           //! Call this function periodically to handle timeouts, send consecutive frames.
static void           SNODE_TP_HandleCanMessage(PSNODE_TP_LINK_CTX pLink, void * pData, uint8_t length); //! Handles incoming CAN messages.
static SNODE_TP_ERROR SNODE_TP_Send(PSNODE_TP_LINK_CTX pLink, const uint8_t * pData, uint8_t size);
static SNODE_TP_ERROR SNODE_TP_Receive(PSNODE_TP_LINK_CTX pLink, uint8_t * pData, const uint8_t dataSize, uint8_t * pOutSize);

/**************************************************************************
 * SNODE TP Static variables.                                             *
 **************************************************************************
 */
const static SNODE_TP_PCI_CMD_HANDLER  snode_tp_cmd_handler[] =
{
  {SNODE_TP_PCI_TYPE_SINGLE_FRAME,       SNODE_TP_ReceiveSingleFrame      },
  {SNODE_TP_PCI_TYPE_FIRST_FRAME,        SNODE_TP_ReceiveFirstFrame       },
  {SNODE_TP_PCI_TYPE_CONSECUTIVE_FRAME,  SNODE_TP_ReceiveConsecutiveFrame },
  {SNODE_TP_PCI_TYPE_FLOW_CONTROL_FRAME, SNODE_TP_ReceiveFlowControlFrame }
};

/**************************************************************************/

SNODE_ERROR SNODE_Init(uint16_t canFrameSourceId)
{
  PSNODE_SENSOR_CTX  pCtx  = (PSNODE_SENSOR_CTX) &snode_ctx;

  MCP2515_Init((PMCP2515_HAL_REG)&Mcp2515HalReg);

  //! Set the filter configurations.
  MCP2515_SetMask(MCP2515_RXM0REGISTERS, 0x7FF);
  MCP2515_SetMask(MCP2515_RXM1REGISTERS, 0x7FF);

  MCP2515_SetFilter(MCP2515_RXF0REGISTERS, CAN_FRAME_BROADCAST_ID);
  MCP2515_SetFilter(MCP2515_RXF1REGISTERS, canFrameSourceId);

  memset(pCtx, 0x00, sizeof(*pCtx));
  memset(snode_cache_entry, 0x00, sizeof(snode_cache_entry));

  pCtx->pCacheNode            = (PSNODE_CACHE_ENTRY) &snode_cache_entry[0];
  pCtx->cacheNodeLength       = SNODE_CACHE_ENTRY_MAX_NUM_OF_NODES;

  pCtx->pGetCmdHandler        = (PSNODE_GET_CMD_HANDLER) &snode_get_cmd[0];
  pCtx->getCmdHandlerLength   = sizeof(snode_get_cmd) / sizeof(snode_get_cmd[0]);

  pCtx->canFrameSourceId      = canFrameSourceId;

  //! pTxQueue = FIFO_Create(); //! :TODO Check retval value. Should no be equeld to NULL.

  TIMER_Init();

  TIMER_AddTask(SNODE_CACHE_ENTRY_POLL_SEND_TIME, //! Sending broadcast packet every 150 ms.
                SNODE_SendPollCacheNode,
                NULL); 

  TIMER_AddTask(SNODE_CACHE_ENTRY_POLL_CHECK_TIME, //! Checking received cache nodes every 15 ms.
                SNODE_CheckPollCacheNode,
                NULL);

  TIMER_AddTask(SNODE_TRANSPORT_TYPE_POLL_TIME,
                SNODE_SendTransportTypeReqFrame, 
                NULL);

  TIMER_AddTask(SNODE_TRANSPORT_SUBSCRIBER_POLL_TIME,
                SNODE_TransportSubscriberPoll,
                NULL);

  return SNODE_SUCCESS;
}

/**************************************************************************/

SNODE_ERROR SNODE_Registration(const PSNODE_SENSOR_REG pSensor)
{
  PSNODE_SENSOR_CTX  pCtx  = (PSNODE_SENSOR_CTX) &snode_ctx;

  if ( NULL != pSensor )
  {
    if ( pCtx->currentIndex < SNODE_MAX_NUM_OF_SENSORS )
    {
      pCtx->pSensor[pCtx->currentIndex]        = pSensor;
      pCtx->pSensor[pCtx->currentIndex]->addr  = pCtx->currentIndex + SNODE_RESERVED_ADDR; //! TODO: Add an ability to set manualy address.

      ++pCtx->currentIndex;

      return SNODE_SUCCESS;
    }
  }

  return SNODE_FAIL;
}

/**************************************************************************/

static PSNODE_SENSOR_REG SNODE_GetSensorCtx(uint8_t addr)
{
  PSNODE_SENSOR_CTX  pCtx  = (PSNODE_SENSOR_CTX) &snode_ctx;
  uint8_t            i     = 0;

  while ( i < pCtx->currentIndex )
  {
    if ( pCtx->pSensor[i]->addr == addr )
    {
      return pCtx->pSensor[i];
    }
    ++i;
  }

  return NULL;
}

/**************************************************************************/

static PSNODE_CACHE_ENTRY SNODE_SearchCacheNode(const uint16_t id)
{
  PSNODE_SENSOR_CTX   pCtx       = (PSNODE_SENSOR_CTX) &snode_ctx;
  PSNODE_CACHE_ENTRY  pCacheNode = pCtx->pCacheNode;
  uint8_t             i          = 0;

  while ( i < pCtx->cacheNodeLength )
  {
    if ( pCacheNode->node.sourceId == id )
    {
      return pCacheNode;
    }
    ++pCacheNode;
    ++i;
  }

  return NULL;
}

/**************************************************************************/

static SNODE_ERROR SNODE_AddCacheNode(const PSNODE_BROADCAST_TYPE pNode)
{
  PSNODE_SENSOR_CTX   pCtx       = (PSNODE_SENSOR_CTX) &snode_ctx;
  PSNODE_CACHE_ENTRY  pCacheNode = pCtx->pCacheNode;
  SNODE_ERROR         retval     = SNODE_FAIL;
  uint8_t             i          = 0;

  while ( i < pCtx->cacheNodeLength )
  {
    if ( pCacheNode->node.sourceId == 0x00 ) //! Look for a free place.
    {
      pCacheNode->node.sourceId        = pNode->sourceId;
      pCacheNode->node.numberOfSensors = pNode->numberOfSensors;

      pCacheNode->timestamp = TIME_GetMs();

#if defined(_SNODE_DEBUG_)
      DEBUG_PrintString("Id = ");
      DEBUG_PrintNumber(pCacheNode->node.sourceId, DEBUG_DEC);
      DEBUG_PrintString(" ( numberOfSensors = ");
      DEBUG_PrintNumber(pCacheNode->node.numberOfSensors, DEBUG_DEC);
      DEBUG_PrintStringln(" ) is added to the cache node.");
#endif

      SNODE_TP_InitLink(&pCacheNode->linkCtx, pCacheNode->node.sourceId);
      retval = SNODE_SUCCESS;
      break;
    }
    ++pCacheNode;
    ++i;
  }

  return retval;
}

/**************************************************************************/

static bool SNODE_CheckPollCacheNode(void *)
{
  PSNODE_SENSOR_CTX  pCtx       = (PSNODE_SENSOR_CTX) &snode_ctx;
  PSNODE_CACHE_ENTRY pCacheNode = pCtx->pCacheNode;
  uint32_t           time       = 0;
  uint8_t            i          = 0;

  while ( i < pCtx->cacheNodeLength )
  {
    if ( pCacheNode->node.sourceId != 0x00 )
    {
      time = TIME_GetMs();
      if (( time - pCacheNode->timestamp) >= SNODE_CACHE_ENTRY_POLL_TIMEOUT)
      {

#if defined(_SNODE_DEBUG_)
        DEBUG_PrintString("Id = ");
        DEBUG_PrintNumber(pCacheNode->node.sourceId, DEBUG_DEC);
        DEBUG_PrintStringln(" is deleted from the cache node.");
#endif
        //! Send the clear flag to callbacks.
        for ( uint8_t j = SNODE_RESERVED_ADDR; j <= pCacheNode->node.numberOfSensors; j++ )
        {
          PSNODE_NODE_DATA_TYPE pDataNode = &pCacheNode->dataNode[ j - SNODE_RESERVED_ADDR ];
          uint8_t               k         = 0;

          if ( SNODE_INPUT_SENSOR_MODE_TYPE == pDataNode->mode )
          {
            while ( k < pCtx->currentIndex )
            {
              if( (0x00 != pCtx->pSensor[k]->addr) && ( SNODE_OUTPUT_SENSOR_MODE_TYPE == pCtx->pSensor[k]->mode) )
              {
                if ( 0x00 != (pDataNode->type & pCtx->pSensor[k]->subscriber) )
                {
                  SNODE_SENSOR_CALLBACK_DATA sensorCallbackData;

                  sensorCallbackData.id     = pCacheNode->node.sourceId;
                  sensorCallbackData.addr   = pDataNode->addrOfSensor;
                  sensorCallbackData.type   = pDataNode->type;
                  sensorCallbackData.status = false;
                  sensorCallbackData.data   = 0x00;
                  pCtx->pSensor[k]->callback(&sensorCallbackData);
                }
              }
              ++k;
            }
          }
        }
        memset(pCacheNode, 0x00, sizeof(*pCacheNode)); //! Clearing a cach of the node because timeout is over.
      }
      else //! Polling link ctx of the node to handle multiple frame transmitions, timeouts.
      {
        SNODE_TP_Poll(&pCacheNode->linkCtx);
      }
    }
    ++pCacheNode;
    ++i;
  }

  return true;
}

/**************************************************************************/

static bool SNODE_SendTransportTypeReqFrame(void *)
{
  PSNODE_SENSOR_CTX   pCtx       = (PSNODE_SENSOR_CTX) &snode_ctx;
  PSNODE_CACHE_ENTRY  pCacheNode = pCtx->pCacheNode;
  uint8_t             i          = 0;

  while ( i < pCtx->cacheNodeLength )
  {
    if ( (0x00 != pCacheNode->node.sourceId) && (false == pCacheNode->dataNodeIsRead) && (0x00 != pCtx->currentIndex) ) //! pCacheNode->dataNodeIsRead sets to true if response is received.
    {
      uint8_t                     *pBuffer         = (uint8_t *) malloc( (pCtx->currentIndex * sizeof(SNODE_NODE_DATA_TYPE)) + sizeof(SNODE_TRANSPORT_FRAME_HEAD)+ sizeof(uint8_t) ); //! sizeof(uint8_t) is needed for checksum.
      PSNODE_TRANSPORT_FRAME_HEAD  pTransportFrame = (PSNODE_TRANSPORT_FRAME_HEAD) pBuffer;
      PSNODE_NODE_DATA_TYPE        nodeData        = (PSNODE_NODE_DATA_TYPE) pTransportFrame->data;
      uint8_t                      *pChecksum;
      uint8_t                      size;
      PSNODE_SENSOR_REG            pSensor;
      SNODE_TP_ERROR               retval;

      //! Prepare header.
      pTransportFrame->cmd  = SNODE_TRANSPORT_TYPE_CMD;     //! Type cmd.
      pTransportFrame->flag = SNODE_TRANSPORT_CMD_REQ_FLAG; //! Request.

      size = sizeof(SNODE_TRANSPORT_FRAME_HEAD);

      for ( uint8_t i = SNODE_RESERVED_ADDR; i <= pCtx->currentIndex; i++ )
      {
        pSensor = SNODE_GetSensorCtx(i);

        if ( NULL != pSensor )
        {
          nodeData->addrOfSensor     = i; //! Address of the sensor.
          nodeData->mode             = pSensor->mode;
          nodeData->type             = pSensor->type;
          nodeData->subscriber       = pSensor->subscriber;

          ++nodeData;
          size += sizeof(SNODE_NODE_DATA_TYPE);
        }
      }

      //! Adding checksum.
      pChecksum  = (uint8_t *) nodeData;
      *pChecksum = SNODE_Checksum((uint8_t *) pTransportFrame->data, size - sizeof(SNODE_TRANSPORT_FRAME_HEAD) - sizeof(uint8_t));
      size += sizeof(uint8_t); //! Checksum.

      retval = SNODE_TP_Send(&pCacheNode->linkCtx, (uint8_t *)pBuffer, size);
      if (SNODE_TP_SUCCESS == retval)
      {
      }
      else
      {
      }

      free(pBuffer);
    }
    ++pCacheNode;
    ++i;
  }

  return true;
}

/**************************************************************************/

static void SNODE_SendTransportTypeRspFrame(PSNODE_TP_LINK_CTX pLink)
{
  uint8_t                      *pRspBuffer     = (uint8_t *) malloc(sizeof(SNODE_TRANSPORT_FRAME_HEAD));
  PSNODE_TRANSPORT_FRAME_HEAD  pTransportFrame = (PSNODE_TRANSPORT_FRAME_HEAD) pRspBuffer;

  //! Preparing header.
  pTransportFrame->cmd  = SNODE_TRANSPORT_TYPE_CMD;     //! Transport type cmd.
  pTransportFrame->flag = SNODE_TRANSPORT_CMD_RSP_FLAG; //! Response.

  SNODE_TP_Send(pLink, (uint8_t *)pRspBuffer, sizeof(SNODE_TRANSPORT_FRAME_HEAD)); //! TODO: Check retval values.
  free(pRspBuffer);
}

/**************************************************************************
 * @brief Sends broadcast packets to all nodes. It sends packet every     *
 *        SNODE_CACHE_ENTRY_POLL_SEND_TIME (150 ms).                      *
 *                                                                        *
 * @param  Pointer to void.                                               *
 * @return True value. It uses for a periodic timer function.             *
 **************************************************************************
 */
static bool SNODE_SendPollCacheNode(void *)
{
  CAN_FRAME                canSend;
  PSNODE_FRAME_HEAD_CMD    pHead        = (PSNODE_FRAME_HEAD_CMD) canSend.data;
  PSNODE_DISCOVER_CMD_REQ  pDiscoverReq = (PSNODE_DISCOVER_CMD_REQ) pHead->data;
  PSNODE_SENSOR_CTX        pCtx         = (PSNODE_SENSOR_CTX) &snode_ctx;
  MCP2515_ERROR            retval;

  memset(&canSend, 0x00, sizeof(canSend));

  canSend.id  = CAN_FRAME_BROADCAST_ID;
  pHead->cmd  = SNODE_DISCOVER_CMD;

  pDiscoverReq->node.sourceId        = pCtx->canFrameSourceId;
  pDiscoverReq->node.numberOfSensors = pCtx->currentIndex;

  pDiscoverReq->checksum = SNODE_Checksum((uint8_t*) &pDiscoverReq->node, sizeof(pDiscoverReq->node)); //! TODO: Workaround. There is a problem of MCP driver.
  canSend.length         = sizeof(*pHead) + sizeof(*pDiscoverReq);

  retval = MCP2515_SendMessage(&canSend);
  if (retval == MCP2515_SUCCESS)
  {
  }

  return true; //! Repeat.
}

/**************************************************************************/

static bool SNODE_TransportSubscriberPoll(void *)
{
  PSNODE_SENSOR_CTX   pCtx       = (PSNODE_SENSOR_CTX) &snode_ctx;
  PSNODE_CACHE_ENTRY  pCacheNode = pCtx->pCacheNode;
  uint8_t             i          = 0;
 
  while ( i < pCtx->cacheNodeLength )
  {
    if ( (0x00 != pCacheNode->node.sourceId) && (true == pCacheNode->dataNodeIsRead) )
    {
      uint8_t                      *pBuffer;
      PSNODE_TRANSPORT_FRAME_HEAD  pTransportFrame;
      PSNODE_SUBSCRIBER_FRAME_HEAD pSubscriberHead;
      PSNODE_SUBSCRIBER_DATA_TYPE  pSubscriberData;
      uint8_t                      sizeOfBuffer, sizeOfBufferConst, sendDataSize;
      uint8_t                      *pChecksum;
      SNODE_TP_ERROR               retval;

      //! sizeof(uint8_t) is needed for checksum.
      sizeOfBufferConst = sizeof(uint8_t) + \
                          sizeof(SNODE_SUBSCRIBER_FRAME_HEAD) + \
                          sizeof(SNODE_TRANSPORT_FRAME_HEAD);

      //! Allocate the max size of a needed buffer.
      sizeOfBuffer = sizeOfBufferConst + (pCacheNode->node.numberOfSensors * pCtx->currentIndex * sizeof(SNODE_SUBSCRIBER_DATA_TYPE));

      pBuffer = (uint8_t *) malloc( sizeOfBuffer );
      memset(pBuffer, 0x00, sizeOfBuffer);

      pTransportFrame = (PSNODE_TRANSPORT_FRAME_HEAD) pBuffer;
      pSubscriberHead = (PSNODE_SUBSCRIBER_FRAME_HEAD) pTransportFrame->data;
      pSubscriberData = (PSNODE_SUBSCRIBER_DATA_TYPE) pSubscriberHead->data;

      //! Preparing header.
      pTransportFrame->cmd  = SNODE_TRANSPORT_SUBSCRIBER_CMD; //! Subscriber cmd.
      pTransportFrame->flag = SNODE_TRANSPORT_CMD_REQ_FLAG;   //! Request.

      for ( uint8_t j = SNODE_RESERVED_ADDR; j <= pCacheNode->node.numberOfSensors; j++ )
      {
        PSNODE_NODE_DATA_TYPE pDataNode = &pCacheNode->dataNode[ j - SNODE_RESERVED_ADDR ];
        uint8_t               k         = 0;

        if ( SNODE_OUTPUT_SENSOR_MODE_TYPE == pDataNode->mode )
        {
          while ( k < pCtx->currentIndex )
          {
            if( (0x00 != pCtx->pSensor[k]->addr) && ( SNODE_INPUT_SENSOR_MODE_TYPE == pCtx->pSensor[k]->mode) )
            {
              if ( 0x00 != (pDataNode->subscriber & pCtx->pSensor[k]->type) )
              {
                SNODE_ERROR retval;

                pSubscriberData->destAddrOfSensor   = pDataNode->addrOfSensor;
                pSubscriberData->sourceAddrOfSensor = pCtx->pSensor[k]->addr;
                //pSubscriberData->sizeOfSourceDataSensor

                retval = pCtx->pSensor[k]->callback(&pSubscriberData->sensorData);
                if ( SNODE_SUCCESS == retval )
                {
                  ++pSubscriberHead->numOfSubscriberData;
                  ++pSubscriberData;
                }
              }
            }
            ++k;
          }
        }
      }

      if ( 0x00 != pSubscriberHead->numOfSubscriberData ) //! If there is any data to be sent.
      {
        //! Adding checksum.
        sendDataSize = sizeOfBufferConst + (pSubscriberHead->numOfSubscriberData * sizeof(SNODE_SUBSCRIBER_DATA_TYPE));
        pChecksum    = (uint8_t *) pSubscriberData;
        *pChecksum   = SNODE_Checksum((uint8_t*) pSubscriberHead, sendDataSize - sizeof(uint8_t)); //! size = sendDataSize - sizeof(checksum).

        retval = SNODE_TP_Send(&pCacheNode->linkCtx, (uint8_t *)pBuffer, sendDataSize);
        if (SNODE_TP_SUCCESS == retval)
        {
        }
        else
        {
        }
      }
      free(pBuffer);
    }
    ++pCacheNode;
    ++i;
  }

  return true; //! Repeat.
}

/**************************************************************************
 * @brief Receives broadcast packets from all nodes. If a node is found   *
 *        in the cache then update a timestamp fot it.                    *
 *                                                                        *
 * @param  request*                                                       *
 * @param  requestLength*                                                 *
 * @return Void.                                                          *
 **************************************************************************
 */
static void SNODE_DiscoverCmd(void * request , uint8_t * requestLength)
{
  PSNODE_DISCOVER_CMD_REQ     pDiscoverReq = (PSNODE_DISCOVER_CMD_REQ) request;
  PSNODE_CACHE_ENTRY          pCacheNode;
  PSNODE_SENSOR_CTX           pCtx         = (PSNODE_SENSOR_CTX) &snode_ctx;

  if ( pCtx->canFrameSourceId != pDiscoverReq->node.sourceId ) //! Double-check id. Destination and source.
  {
    uint8_t checksum = SNODE_Checksum((uint8_t*) &pDiscoverReq->node, sizeof(pDiscoverReq->node));
    if ( (checksum == pDiscoverReq->checksum) && (pDiscoverReq->node.numberOfSensors <= SNODE_MAX_NUM_OF_SENSORS) ) //! Checking checksum and number of sensors per node.
    {
      pCacheNode = SNODE_SearchCacheNode(pDiscoverReq->node.sourceId);
      if ( NULL == pCacheNode ) //! The node is not found.
      {
        SNODE_AddCacheNode(&pDiscoverReq->node);
      }
      else
      {
        pCacheNode->timestamp = TIME_GetMs(); //! Update a timestamp value for a node in a cache array.
      }
    }
  }
}

/**************************************************************************/

static uint8_t SNODE_Checksum(uint8_t * data, uint8_t length)
{
  uint8_t sum = 0;
  uint8_t i   = 0;

  if ( NULL != data )
  {
    while ( i < length )
    {
      sum += data[i++];
    }

    return 0xFF - sum;
  }

  return 0;
}

/**************************************************************************/

static void SNODE_TransportCmd(void * request , uint8_t * requestLength)
{
  PSNODE_TRANSPORT_FRAME_HEAD pTransport = (PSNODE_TRANSPORT_FRAME_HEAD) request;
  PSNODE_CACHE_ENTRY          pCacheNode;

  pCacheNode = SNODE_SearchCacheNode(pTransport->sourceId);
  if ( NULL != pCacheNode ) //! Check if this source id was identify. Discover cmd was finished.
  {
    uint8_t *pBuffer;
    uint8_t size, outSize;

    //! If received any interested transport message.
    SNODE_TP_HandleCanMessage(&pCacheNode->linkCtx, pTransport->data, *requestLength - sizeof(SNODE_FRAME_HEAD_CMD) - sizeof(SNODE_TRANSPORT_FRAME_HEAD));

    if ( pCacheNode->node.numberOfSensors > SNODE_MAX_NUM_OF_SENSORS )
    {
      return; //! TODO: Rewrite this check.
    }

    size    = SNODE_TP_MAX_BUFFER_MESSAGE_SIZE; //! pCacheNode->node.numberOfSensors * sizeof(SNODE_NODE_DATA_TYPE) + sizeof(SNODE_TRANSPORT_FRAME_HEAD);
    pBuffer = (uint8_t *) malloc( size ); //! TODO: Size of allocated buffer should be changed for another types of transport protocol.
    memset(pBuffer, 0x00, size);

    if ( SNODE_TP_SUCCESS == SNODE_TP_Receive(&pCacheNode->linkCtx, (uint8_t *)pBuffer, size, &outSize) )
    {
      //! Handle transport data.
      PSNODE_TRANSPORT_FRAME_HEAD  pTransportFrame = (PSNODE_TRANSPORT_FRAME_HEAD) pBuffer;
      switch ( pTransportFrame->cmd )
      {
        case SNODE_TRANSPORT_TYPE_CMD:
        {
          if ( SNODE_TRANSPORT_CMD_REQ_FLAG == pTransportFrame->flag ) //! Request.
          {
            PSNODE_NODE_DATA_TYPE nodeData = (PSNODE_NODE_DATA_TYPE) pTransportFrame->data;

            uint8_t *pTypeChecksum = (uint8_t *) pTransportFrame->data + (pCacheNode->node.numberOfSensors * sizeof(SNODE_NODE_DATA_TYPE)); //! Pointer to checksum value.
            uint8_t checksum       = SNODE_Checksum((uint8_t *) pTransportFrame->data, (pCacheNode->node.numberOfSensors * sizeof(SNODE_NODE_DATA_TYPE)) - sizeof(uint8_t)); //! Calculate checksum.

            if ( *pTypeChecksum == checksum )
            {
              for ( uint8_t i = SNODE_RESERVED_ADDR; i <= pCacheNode->node.numberOfSensors; i++, nodeData++)
              {
                memcpy(&pCacheNode->dataNode[i-SNODE_RESERVED_ADDR], nodeData, sizeof(*nodeData));

#if defined(_SNODE_DEBUG_)
                DEBUG_PrintString("nodeData->addrOfSensor = ");
                DEBUG_PrintNumber(nodeData->addrOfSensor, DEBUG_DEC);
                DEBUG_PrintString(", nodeData->mode = 0x");
                DEBUG_PrintNumber(nodeData->mode, DEBUG_HEX);
                DEBUG_PrintString(", nodeData->type = 0x");
                DEBUG_PrintNumber(nodeData->type, DEBUG_HEX);
                DEBUG_PrintString(", nodeData->subscriber = 0x");
                DEBUG_PrintNumber(nodeData->subscriber, DEBUG_HEX);
                DEBUG_PrintStringln("");
#endif
              }

              SNODE_SendTransportTypeRspFrame(&pCacheNode->linkCtx); //! Send response.
            }
          }
          else if ( SNODE_TRANSPORT_CMD_RSP_FLAG == pTransportFrame->flag ) //! Response.
          {
            if ( outSize == sizeof(PSNODE_TRANSPORT_FRAME_HEAD) ) //! TODO: Received a response. An additional ckecks should be added.
            {
              pCacheNode->dataNodeIsRead = true; //! Data was saved on the other side of the node.
            }
          }
          break;
        }

        case SNODE_TRANSPORT_SUBSCRIBER_CMD:
        {
          if ( SNODE_TRANSPORT_CMD_REQ_FLAG == pTransportFrame->flag ) //! Request.
          {
            PSNODE_SUBSCRIBER_FRAME_HEAD pSubscriberHead = (PSNODE_SUBSCRIBER_FRAME_HEAD) pTransportFrame->data;
            PSNODE_SUBSCRIBER_DATA_TYPE  pSubscriberData = (PSNODE_SUBSCRIBER_DATA_TYPE) pSubscriberHead->data;
            uint8_t subscriberPacketLength;

            subscriberPacketLength = sizeof(*pSubscriberHead) + ( pSubscriberHead->numOfSubscriberData * sizeof(*pSubscriberData));

            //! Check packet length.
            if ( outSize >= subscriberPacketLength )
            {
              uint8_t *pSubscriberChecksum = (uint8_t *) pSubscriberHead + subscriberPacketLength; //! Pointer to checksum value.
              uint8_t checksum             = SNODE_Checksum((uint8_t *) pSubscriberHead, subscriberPacketLength); //! Calculate checksum of the subscriber packet.

              //! Checking checksum.
              if ( *pSubscriberChecksum == checksum )
              {
                for ( uint8_t i = 0; i < pSubscriberHead->numOfSubscriberData; i++, pSubscriberData++ )
                {
                  PSNODE_SENSOR_REG pSensor;

                  pSensor = SNODE_GetSensorCtx(pSubscriberData->destAddrOfSensor);
                  //! TODO: Add an additinal check: mode, sensorAddr, type.
                  if ( (NULL != pSensor) && (pSubscriberData->sourceAddrOfSensor <= SNODE_MAX_NUM_OF_SENSORS) )
                  {
                    SNODE_SENSOR_CALLBACK_DATA  sensorCallbackData;

                    sensorCallbackData.id     = pCacheNode->node.sourceId;
                    sensorCallbackData.addr   = pSubscriberData->sourceAddrOfSensor;
                    sensorCallbackData.type   = pCacheNode->dataNode[pSubscriberData->sourceAddrOfSensor - SNODE_RESERVED_ADDR].type;
                    sensorCallbackData.status = true;
                    sensorCallbackData.data   = pSubscriberData->sensorData;
                    pSensor->callback(&sensorCallbackData);
                  }
                }
              }
            }
          }
          break;
        }

        default:
          break;
      }
    }
    free(pBuffer); //! Free allocated memory.
  }
}

/**************************************************************************/

SNODE_ERROR SNODE_ProcessPacket(void)
{
  CAN_FRAME canReceive;

  memset(&canReceive, 0x00, sizeof(canReceive));

  if ( MCP2515_SUCCESS == MCP2515_ReadMessage(&canReceive) )
  {
    PSNODE_SENSOR_CTX  pCtx  = (PSNODE_SENSOR_CTX) &snode_ctx;

    if (( canReceive.id == CAN_FRAME_BROADCAST_ID ) || ( canReceive.id == pCtx->canFrameSourceId )) //! Double check. Should be set by mcp filters.
    {
      const PSNODE_FRAME_HEAD_CMD pHead = (PSNODE_FRAME_HEAD_CMD) canReceive.data;
      uint8_t                     i     = 0;

      while ( i < pCtx->getCmdHandlerLength )
      {
        if ( pHead->cmd == pCtx->pGetCmdHandler[i].cmd )
        {
          pCtx->pGetCmdHandler[i].handler(pHead->data, &canReceive.length);
          break;
        }
        ++i;
      }
    }
  }
  TIMER_Tick();

  return SNODE_SUCCESS;
}

/**************************************************************************/
/*                         Start SNODE ISO TP                             */
/**************************************************************************/

static void SNODE_TP_InitLink(PSNODE_TP_LINK_CTX pLink, uint16_t id) 
{
  PSNODE_TP_LINK_FRAME_CTX  pLinkSend     = (PSNODE_TP_LINK_FRAME_CTX) &pLink->send;
  PSNODE_TP_LINK_FRAME_CTX  pLinkReceive  = (PSNODE_TP_LINK_FRAME_CTX) &pLink->receive;

  memset(pLink, 0, sizeof(*pLink));

  pLink->id                = id;
  pLinkSend->status        = SNODE_TP_SEND_STATUS_IDLE;
  pLinkReceive->status     = SNODE_TP_RECEIVE_STATUS_IDLE;
}

/**************************************************************************/

static SNODE_TP_ERROR SNODE_TP_Send(PSNODE_TP_LINK_CTX pLink, const uint8_t * pData, uint8_t size)
{
  PSNODE_TP_LINK_FRAME_CTX pLinkSend  = (PSNODE_TP_LINK_FRAME_CTX) &pLink->send;
  SNODE_TP_ERROR           retval     = SNODE_TP_FAIL;

  //! Check the message. Increase SNODE_TP_MAX_BUFFER_MESSAGE_SIZE to set a larger buffer.
  if ( size <= sizeof(pLinkSend->buffer) )
  {
    //! Abort the message if the transmission is in a progress.
    if ( SNODE_TP_SEND_STATUS_INPROGRESS != pLinkSend->status )
    {
      //! Copy data into a local buffer.
      pLinkSend->size = size;
      memcpy(pLinkSend->buffer, pData, size);

      if ( pLinkSend->size < SNODE_TP_DATA_SIZE )
      {
        //! Send a single frame.
        retval = SNODE_TP_SendSingleFrame(pLink);
      }
      else
      {
        //! Send a multi-frame.
        retval = SNODE_TP_SendFirstFrame(pLink);
      }
    }
  }

  return retval;
}

/**************************************************************************/

static SNODE_TP_ERROR SNODE_TP_Receive(PSNODE_TP_LINK_CTX pLink, uint8_t * pData, const uint8_t dataSize, uint8_t * pOutSize)
{
  PSNODE_TP_LINK_FRAME_CTX  pLinkReceive  = (PSNODE_TP_LINK_FRAME_CTX) &pLink->receive;
  SNODE_TP_ERROR            retval        = SNODE_TP_FAIL;

  //! If the transport packet is received.
  if ( SNODE_TP_RECEIVE_STATUS_FULL == pLinkReceive->status )
  {
    if ( pLinkReceive->size <= dataSize )
    {
      memcpy(pData, pLinkReceive->buffer, pLinkReceive->size);
      *pOutSize            = pLinkReceive->size;
      pLinkReceive->status = SNODE_TP_RECEIVE_STATUS_IDLE;

      retval = SNODE_TP_SUCCESS;
    }
  }

  return retval;
}

/**************************************************************************/

static SNODE_TP_ERROR  SNODE_TP_SendCanMessage(uint16_t id, const uint8_t * pData, const uint8_t size)
{
  CAN_FRAME                   frame;
  PSNODE_FRAME_HEAD_CMD       pHead      = (PSNODE_FRAME_HEAD_CMD) frame.data;
  PSNODE_TRANSPORT_FRAME_HEAD pTransport = (PSNODE_TRANSPORT_FRAME_HEAD) pHead->data;
  PSNODE_SENSOR_CTX           pCtx       = (PSNODE_SENSOR_CTX) &snode_ctx;
  MCP2515_ERROR               retval;

  memset(&frame, 0x00, sizeof(frame));

  frame.id             = id;
  pHead->cmd           = SNODE_TRANSPORT_CMD;
  pTransport->sourceId = pCtx->canFrameSourceId;

  memcpy(pTransport->data, pData, size);
  frame.length = size + sizeof(*pHead) + sizeof(*pTransport);

  retval = MCP2515_SendMessage(&frame);
  if ( (MCP2515_TX_NO_FREE_BUFFER == retval) || (MCP2515_FAIL == retval) )
  {
    return SNODE_TP_FAIL;
  }

  return SNODE_TP_SUCCESS;
}

/**************************************************************************/

static SNODE_TP_ERROR SNODE_TP_SendSingleFrame(PSNODE_TP_LINK_CTX pLink)
{
  PSNODE_TP_LINK_FRAME_CTX pLinkSend  = (PSNODE_TP_LINK_FRAME_CTX) &pLink->send;
  uint8_t                  *pBuffer;
  PSNODE_TP_SINGLE_FRAME   pSingleFrame;
  SNODE_TP_ERROR           retval;

  //! Allocate the buffer for the single frame message.
  pBuffer       = (uint8_t *) malloc( SNODE_TP_DATA_SIZE );
  pSingleFrame = (PSNODE_TP_SINGLE_FRAME) pBuffer;

  //! Preparing message.
  pSingleFrame->type = SNODE_TP_PCI_TYPE_SINGLE_FRAME;
  pSingleFrame->dl   = (uint8_t) pLinkSend->size;

  memcpy(pSingleFrame->data, pLinkSend->buffer, pLinkSend->size);

  retval = SNODE_TP_SendCanMessage(pLink->id, (uint8_t *) pSingleFrame, pLinkSend->size + sizeof(*pSingleFrame));
  free(pBuffer);

  return retval;
}

/**************************************************************************/

static SNODE_TP_ERROR SNODE_TP_ReceiveSingleFrame(PSNODE_TP_LINK_CTX pLink, void * pMessage, uint8_t length)
{
  PSNODE_TP_LINK_FRAME_CTX  pLinkReceive = (PSNODE_TP_LINK_FRAME_CTX) &pLink->receive;
  PSNODE_TP_SINGLE_FRAME    pSingleFrame = (PSNODE_TP_SINGLE_FRAME) pMessage;
  SNODE_TP_ERROR            retval       = SNODE_TP_FAIL;

  //! Checking data length.
  if ( (0 != pSingleFrame->dl) || (pSingleFrame->dl <= length) )
  {
    memcpy(pLinkReceive->buffer, pSingleFrame->data, pSingleFrame->dl);

    pLinkReceive->size   = pSingleFrame->dl;
    pLinkReceive->status = SNODE_TP_RECEIVE_STATUS_FULL;

    retval = SNODE_TP_SUCCESS;
  }

  return retval;
}

/**************************************************************************/

static SNODE_TP_ERROR SNODE_TP_SendFirstFrame(PSNODE_TP_LINK_CTX pLink) 
{
  PSNODE_TP_LINK_FRAME_CTX pLinkSend   = (PSNODE_TP_LINK_FRAME_CTX) &pLink->send;
  uint8_t                  *pBuffer;
  PSNODE_TP_FIRST_FRAME    pFirstFrame;
  SNODE_TP_ERROR           retval;

  //! Allocate the buffer for the first frame message.
  pBuffer     = (uint8_t *) malloc( SNODE_TP_DATA_SIZE );
  pFirstFrame = (PSNODE_TP_FIRST_FRAME) pBuffer;

  //! Preparing message.
  pFirstFrame->type   = SNODE_TP_PCI_TYPE_FIRST_FRAME;
  pFirstFrame->dlLow  = (uint8_t) pLinkSend->size;
  pFirstFrame->dlHigh = (uint8_t) (0x0F & (pLinkSend->size >> 8));

  memcpy(pFirstFrame->data, pLinkSend->buffer, SNODE_TP_DATA_SIZE - sizeof(*pFirstFrame));

  retval = SNODE_TP_SendCanMessage(pLink->id, (uint8_t *) pFirstFrame, SNODE_TP_DATA_SIZE);
  if ( SNODE_TP_SUCCESS == retval )
  {
    pLinkSend->offset   = SNODE_TP_DATA_SIZE - sizeof(*pFirstFrame);

    //! Init multi-frame flags.
    pLinkSend->sn       = 0x00;
    pLinkSend->bs       = 0x00;
    pLinkSend->status   = SNODE_TP_SEND_STATUS_INPROGRESS;

    pLink->timer.nBs    = TIME_GetMs();
  }
  free(pBuffer);

  return retval;
}

/**************************************************************************/

static SNODE_TP_ERROR SNODE_TP_ReceiveFirstFrame(PSNODE_TP_LINK_CTX pLink, void * pMessage, uint8_t length)
{
  PSNODE_TP_LINK_FRAME_CTX  pLinkReceive  = (PSNODE_TP_LINK_FRAME_CTX) &pLink->receive;
  PSNODE_TP_FIRST_FRAME     pFirstFrame   = (PSNODE_TP_FIRST_FRAME) pMessage;
  uint16_t                  payloadLength; //! TODO: Change size of data payload. 
  uint8_t                   flowStatus;
  SNODE_TP_ERROR            retval;

  //! Checking data length.
  payloadLength = pFirstFrame->dlHigh;
  payloadLength = (payloadLength << 8) + pFirstFrame->dlLow;

  if ( payloadLength > sizeof(pLinkReceive->buffer) )
  {
    //! Multi frame is too large for receiving. Increase SNODE_TP_MAX_BUFFER_MESSAGE_SIZE to set a larger buffer.
    pLinkReceive->status = SNODE_TP_RECEIVE_STATUS_IDLE;
    flowStatus           = SNODE_TP_PCI_FLOW_STATUS_OVERFLOW;
    pLinkReceive->bs     = 0x00;
    retval               = SNODE_TP_FAIL;
  }
  else
  {
    memcpy(pLinkReceive->buffer, pFirstFrame->data, length - sizeof(*pFirstFrame));

    pLinkReceive->size    = payloadLength;
    pLinkReceive->offset  = length - sizeof(*pFirstFrame);

    //! Multi-frame flags.
    pLinkReceive->sn      = 0x00;
    pLinkReceive->status  = SNODE_TP_RECEIVE_STATUS_INPROGRESS;
    pLinkReceive->bs      = SNODE_TP_BLOCK_SIZE;

    pLink->timer.nCr      = TIME_GetMs();

    flowStatus = SNODE_TP_PCI_FLOW_STATUS_CONTINUE;
    retval     = SNODE_TP_SUCCESS;
  }

  SNODE_TP_SendFlowControl(pLink, flowStatus, pLinkReceive->bs);

  return retval;
}

/**************************************************************************/

static SNODE_TP_ERROR SNODE_TP_SendFlowControl(PSNODE_TP_LINK_CTX pLink, uint8_t flowStatus, uint8_t blockSize) 
{
  SNODE_TP_FLOW_CONTROL flowControl;

  //! Preparing message.
  flowControl.type  = SNODE_TP_PCI_TYPE_FLOW_CONTROL_FRAME;
  flowControl.fs    = flowStatus;
  flowControl.bs    = blockSize;
  flowControl.stMin = SNODE_TP_ST_MIN;

  return SNODE_TP_SendCanMessage(pLink->id, (uint8_t *) &flowControl, sizeof(flowControl));
}

/**************************************************************************/

static SNODE_TP_ERROR SNODE_TP_ReceiveFlowControlFrame(PSNODE_TP_LINK_CTX pLink, void * pMessage, uint8_t length)
{
  PSNODE_TP_LINK_FRAME_CTX pLinkSend     = (PSNODE_TP_LINK_FRAME_CTX) &pLink->send;
  PSNODE_TP_FLOW_CONTROL   pFlowControl  = (PSNODE_TP_FLOW_CONTROL) pMessage;
  SNODE_TP_ERROR           retval        = SNODE_TP_FAIL;

  //! Check if the sending is in progress.
  if ( SNODE_TP_SEND_STATUS_INPROGRESS == pLinkSend->status )
  {
    //! Check the message length of the flow control frame.
    if ( length == sizeof(*pFlowControl) )
    {
      switch ( pFlowControl->fs )
      {
        case SNODE_TP_PCI_FLOW_STATUS_OVERFLOW: //! If buffer is overflow.
              pLinkSend->status = SNODE_TP_SEND_STATUS_ERROR;
              retval            = SNODE_TP_FAIL;
              break;

        case SNODE_TP_PCI_FLOW_STATUS_CONTINUE: //! Permit the sending.
              pLink->timer.nBs = TIME_GetMs();
              pLinkSend->bs    = pFlowControl->bs;
              retval           = SNODE_TP_SUCCESS;  
              break;

        default:
              break;
      }
    }
  }

  return retval;
}

/**************************************************************************/

static SNODE_TP_ERROR SNODE_TP_SendConsecutiveFrame(PSNODE_TP_LINK_CTX pLink)
{
  PSNODE_TP_LINK_FRAME_CTX    pLinkSend = (PSNODE_TP_LINK_FRAME_CTX) &pLink->send;
  uint8_t                     *pBuffer;
  PSNODE_TP_CONSECUTIVE_FRAME pConsecutiveFrame;
  SNODE_TP_ERROR              retval;
  uint8_t                     sendDataSize;

  //! Allocate the buffer for the consecutive frame message.
  pBuffer           = (uint8_t *) malloc( SNODE_TP_DATA_SIZE );
  pConsecutiveFrame = (PSNODE_TP_CONSECUTIVE_FRAME) pBuffer;

  //! Preparing message.
  pConsecutiveFrame->type = SNODE_TP_PCI_TYPE_CONSECUTIVE_FRAME;
  pConsecutiveFrame->sn   = pLinkSend->sn;

  sendDataSize = SNODE_TP_DATA_SIZE - sizeof(*pConsecutiveFrame);

  if ( pLinkSend->size <= (pLinkSend->offset + sendDataSize) )
  {
    //! Send the last frame of the TP packet and set the status to IDLE.
    sendDataSize      = pLinkSend->size - pLinkSend->offset;
    pLinkSend->status = SNODE_TP_SEND_STATUS_IDLE;
  }

  memcpy(pConsecutiveFrame->data, pLinkSend->buffer + pLinkSend->offset, sendDataSize);

  retval = SNODE_TP_SendCanMessage(pLink->id, (uint8_t*) pConsecutiveFrame, sendDataSize + sizeof(*pConsecutiveFrame));
  if ( SNODE_TP_SUCCESS == retval )
  {
    pLinkSend->offset += sendDataSize;
    pLinkSend->sn++;
    pLinkSend->bs--;
    pLink->timer.nBs = TIME_GetMs();

    SNODE_TP_SnWrap(&pLinkSend->sn);
  }
  free(pBuffer);

  return retval;
}

/**************************************************************************/

static SNODE_TP_ERROR SNODE_TP_ReceiveConsecutiveFrame(PSNODE_TP_LINK_CTX pLink, void * pMessage, uint8_t length)
{
  PSNODE_TP_LINK_FRAME_CTX    pLinkReceive      = (PSNODE_TP_LINK_FRAME_CTX) &pLink->receive;
  PSNODE_TP_CONSECUTIVE_FRAME pConsecutiveFrame = (PSNODE_TP_CONSECUTIVE_FRAME) pMessage;
  SNODE_TP_ERROR              retval            = SNODE_TP_FAIL;
  uint8_t                     receiveDataSize;

  if ( SNODE_TP_RECEIVE_STATUS_INPROGRESS == pLinkReceive->status )
  {
    //! Check the sequence number.
    if ( pLinkReceive->sn == pConsecutiveFrame->sn )
    {
      receiveDataSize = SNODE_TP_DATA_SIZE - sizeof(*pConsecutiveFrame);

      if ( pLinkReceive->size <= (pLinkReceive->offset + receiveDataSize) )
      {
        //! Receive the last frame of the TP packet and set the status to FULL.
        receiveDataSize      = pLinkReceive->size - pLinkReceive->offset;
        pLinkReceive->status = SNODE_TP_RECEIVE_STATUS_FULL;
      }

      memcpy(pLinkReceive->buffer + pLinkReceive->offset, pConsecutiveFrame->data, receiveDataSize);

      pLinkReceive->offset += receiveDataSize;
      pLinkReceive->sn++;
      pLinkReceive->bs--;

      pLink->timer.nCr = TIME_GetMs();

      SNODE_TP_SnWrap(&pLinkReceive->sn);

      if ( 0x00 == pLinkReceive->bs )
      {
        pLinkReceive->bs = SNODE_TP_BLOCK_SIZE;
        SNODE_TP_SendFlowControl(pLink, SNODE_TP_PCI_FLOW_STATUS_CONTINUE, pLinkReceive->bs);
      }

      retval = SNODE_TP_SUCCESS;
    }
  }

  return retval;
}

/**************************************************************************/

static void SNODE_TP_HandleCanMessage(PSNODE_TP_LINK_CTX pLink, void * pData, uint8_t length)
{
  PSNODE_TP_PCI_TYPE        pTpType            = (PSNODE_TP_PCI_TYPE) pData;
  PSNODE_TP_PCI_CMD_HANDLER pTpCmdHandler      = (PSNODE_TP_PCI_CMD_HANDLER) &snode_tp_cmd_handler[0];
  uint8_t                   tpCmdHandlerLength = sizeof(snode_tp_cmd_handler) / sizeof(snode_tp_cmd_handler[0]);
  uint8_t                   i                  = 0;

  if ( SNODE_TP_DATA_SIZE >= length )
  {
    while ( i < tpCmdHandlerLength )
    {
      if ( pTpType->type == pTpCmdHandler[i].type )
      {
        pTpCmdHandler[i].handler(pLink, pData, length);
        break;
      }
      ++i;
    }
  }
}

/**************************************************************************/

static void SNODE_TP_SnWrap(uint8_t *pSn)
{
  //! When the SN reaches the value SNODE_TP_SN_MAX it will wraparound
  //! and set to 0x00 for the next consecutive frame.
  if ( *pSn > SNODE_TP_SN_MAX )
  {
    *pSn = 0x00;
  }
}

/**************************************************************************/

static void SNODE_TP_Poll(PSNODE_TP_LINK_CTX pLink)
{
  PSNODE_TP_LINK_FRAME_CTX  pLinkSend     = (PSNODE_TP_LINK_FRAME_CTX) &pLink->send;
  PSNODE_TP_LINK_FRAME_CTX  pLinkReceive  = (PSNODE_TP_LINK_FRAME_CTX) &pLink->receive;

  if ( SNODE_TP_SEND_STATUS_INPROGRESS == pLinkSend->status )
  {
    //! Continue sending data.
    if ( 0x00 != pLinkSend->bs )
    {
      SNODE_TP_SendConsecutiveFrame(pLink);
    }
    else
    {
      //! Check N_Bs timeout.
      if ( (TIME_GetMs() - pLink->timer.nBs) >= SNODE_TP_N_BS_TIMEOUT )
      {
#if defined(_SNODE_DEBUG_)
        DEBUG_PrintStringln("N_Bs timeout.");
#endif
        pLinkSend->status = SNODE_TP_SEND_STATUS_ERROR;
      }
    }
  }

  if ( SNODE_TP_RECEIVE_STATUS_INPROGRESS == pLinkReceive->status )
  {
    if ( (TIME_GetMs() - pLink->timer.nCr) >= SNODE_TP_N_CR_TIMEOUT )
    {
#if defined(_SNODE_DEBUG_)
      DEBUG_PrintStringln("N_Cr timeout.");
#endif
      pLinkReceive->status = SNODE_TP_RECEIVE_STATUS_IDLE;
    }
  }
}
