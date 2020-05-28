/**************************************************************************
 * @file       nrf24l01.cpp                                               *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      The NRF24L01P+ driver.                                     *
 *************************************************************************/

#include "nrf24l01.h"
#include <stdint.h>
#include <string.h>
#include "Arduino-hal/time_hal.h"

/**************************************************************************
 * Static function declarations.                                          *
 **************************************************************************
 */
static uint8_t        NRF24L01_ReadRegister(const NRF24L01_REGISTER_MAP_TABLE reg);
static void           NRF24L01_ReadSetOfRegisters(const NRF24L01_REGISTER_MAP_TABLE reg, uint8_t values[], const uint8_t size);
static NRF24L01_ERROR NRF24L01_SetRegister (const NRF24L01_REGISTER_MAP_TABLE reg, const uint8_t value);
static NRF24L01_ERROR NRF24L01_SetSetOfRegisters(const NRF24L01_REGISTER_MAP_TABLE reg, const uint8_t values[], const uint8_t n);

/**************************************************************************/

static NRF24L01_ERROR NRF24L01_FlushTx(void);
static NRF24L01_ERROR NRF24L01_FlushRx(void);

/**************************************************************************/

static NRF24L01_ERROR NRF24L01_StartRx(void);
static NRF24L01_ERROR NRF24L01_WaitForTxToComplete(void);
static void           NRF24L01_PrepareForTx(uint8_t toRadioId);

/**************************************************************************
 * SNODE Static variables.                                                *
 **************************************************************************
 */
static NRF24L01_CTX nrf24l01_ctx;

/**************************************************************************/

static uint8_t NRF24L01_ReadRegister(const NRF24L01_REGISTER_MAP_TABLE reg)
{
  PNRF24L01_CTX     pCtx = (PNRF24L01_CTX) &nrf24l01_ctx;
  PNRF24L01_HAL_REG pHal = (PNRF24L01_HAL_REG) pCtx->pHal;
  uint8_t           retval;

  pHal->spiBegin(); //! Csn is low.

  pHal->spiTransfer(NRF24L01_INSTR_SET_R_REGISTER | ( NRF24L01_INSTR_SET_REGISTER_MASK & reg ) );
  retval = pHal->spiTransfer(NRF24L01_INSTR_SET_NOP);

  pHal->spiEnd(); //! Csn is high

  return retval;
}

/**************************************************************************/

static void NRF24L01_ReadSetOfRegisters(const NRF24L01_REGISTER_MAP_TABLE reg, uint8_t values[], const uint8_t size)
{
  PNRF24L01_CTX     pCtx = (PNRF24L01_CTX) &nrf24l01_ctx;
  PNRF24L01_HAL_REG pHal = (PNRF24L01_HAL_REG) pCtx->pHal;

  pHal->spiBegin(); //! Csn is low.

  pHal->spiTransfer(NRF24L01_INSTR_SET_R_REGISTER | ( NRF24L01_INSTR_SET_REGISTER_MASK & reg ) );
  for (uint8_t i = 0; i < size; i++)
  {
    values[i] = pHal->spiTransfer(NRF24L01_INSTR_SET_NOP);
  }

  pHal->spiEnd(); //! Csn is high.
}

/**************************************************************************/

static NRF24L01_ERROR NRF24L01_SetRegister (const NRF24L01_REGISTER_MAP_TABLE reg, const uint8_t value)
{
  PNRF24L01_CTX     pCtx = (PNRF24L01_CTX) &nrf24l01_ctx;
  PNRF24L01_HAL_REG pHal = (PNRF24L01_HAL_REG) pCtx->pHal;

  pHal->spiBegin(); //! Csn is low.

  pHal->spiTransfer(NRF24L01_INSTR_SET_W_REGISTER | ( NRF24L01_INSTR_SET_REGISTER_MASK & reg ) );
  pHal->spiTransfer(value);

  pHal->spiEnd(); //! Csn is high.

  return NRF24L01_SUCCESS;
}

/**************************************************************************/

static NRF24L01_ERROR NRF24L01_SetSetOfRegisters(const NRF24L01_REGISTER_MAP_TABLE reg, const uint8_t values[], const uint8_t n)
{
  PNRF24L01_CTX     pCtx = (PNRF24L01_CTX) &nrf24l01_ctx;
  PNRF24L01_HAL_REG pHal = (PNRF24L01_HAL_REG) pCtx->pHal;

  pHal->spiBegin(); //! Csn is low.

  pHal->spiTransfer(NRF24L01_INSTR_SET_W_REGISTER | ( NRF24L01_INSTR_SET_REGISTER_MASK & reg ) );
  for (uint8_t i = 0; i < n; i++)
  {
    pHal->spiTransfer(values[i]);
  }

  pHal->spiEnd(); //! Csn is high.

  return NRF24L01_SUCCESS;
}

/**************************************************************************
 * Flush TX FIFO, used in TX mode.                                        *
 **************************************************************************
 */
static NRF24L01_ERROR NRF24L01_FlushTx(void)
{
  PNRF24L01_CTX     pCtx = (PNRF24L01_CTX) &nrf24l01_ctx;
  PNRF24L01_HAL_REG pHal = (PNRF24L01_HAL_REG) pCtx->pHal;

  pHal->spiBegin(); //! Csn is low.

  pHal->spiTransfer(NRF24L01_INSTR_SET_FLUSH_TX);
  pHal->spiTransfer(0x00);

  pHal->spiEnd(); //! Csn is high.

  return NRF24L01_SUCCESS;
}

/**************************************************************************
 * Flush RX FIFO, used in RX mode. Should not be executed during          *
 * transmission of acknowledge, that is, acknowledge package will not be  *
 * completed.                                                             *
 **************************************************************************
 */
static NRF24L01_ERROR NRF24L01_FlushRx(void)
{
  PNRF24L01_CTX     pCtx = (PNRF24L01_CTX) &nrf24l01_ctx;
  PNRF24L01_HAL_REG pHal = (PNRF24L01_HAL_REG) pCtx->pHal;

  pHal->spiBegin(); //! Csn is low.

  pHal->spiTransfer(NRF24L01_INSTR_SET_FLUSH_RX);
  pHal->spiTransfer(0x00);

  pHal->spiEnd(); //! Csn is high

  return NRF24L01_SUCCESS;
}

/**************************************************************************/

NRF24L01_ERROR NRF24L01_Init(PNRF24L01_HAL_REG pInit, uint8_t radioId, uint8_t channel)
{
  PNRF24L01_CTX                pCtx   = (PNRF24L01_CTX) &nrf24l01_ctx;
  NRF24L01_REG_RF_CH_TYPE      rfCh;
  NRF24L01_REG_RF_SETUP_TYPE   rfSetup;
  NRF24L01_REG_SETUP_RETR_TYPE setupPetr;
  NRF24L01_REG_DYNPD_TYPE      dynpd;
  NRF24L01_REG_FEATURE_TYPE    feature;
  NRF24L01_REG_STATUS_TYPE     status;

  memset(pCtx, 0x00, sizeof(*pCtx));

  pCtx->pHal = pInit;
  pCtx->pHal->spiInit(); //! Csn pin is high.

  TIME_DelayMs(NRF24L01_OFF_TO_POWERDOWN_MILLIS);

  memset(&rfCh, 0x00, sizeof(rfCh));
  rfCh.cnvrt.rfChannel = channel;
  NRF24L01_SetRegister(NRF24L01_REG_RF_CH, rfCh.reg); //! Sets the frequency channel. The range is 2400 - 2525 MHz, in 1 MHz increments.

  memset(&rfSetup, 0x00, sizeof(rfSetup));
  rfSetup.cnvrt.rfDrHigh = RF24L01_RF_SETUP_RF_DR_2MBPS;  //! Data rate is 2 Mbps.
  rfSetup.cnvrt.rfPwr    = NRF24L01_RF_SETUP_RF_PWR_0DBM; //! Output power is 0 dBm.
  NRF24L01_SetRegister(NRF24L01_REG_RF_SETUP, rfSetup.reg);

  memset(&setupPetr, 0x00, sizeof(setupPetr));
  setupPetr.cnvrt.arc = 0x0F; //! Auto Retransmit Count. Up to 15 Re-Transmit on fail of AA.
  setupPetr.cnvrt.ard = 0x01; //! Auto Re-transmit Delay. Wait 500+86uS.
  NRF24L01_SetRegister(NRF24L01_REG_SETUP_RETR, setupPetr.reg);

  pCtx->addrP1[0] = radioId; //! Assign the radioId to RX pipe 1.
  //! addrP1 = {0x00, 0x00, 0x00, 0x00, radioId}. This addr uses when another radio sends data to this radio.
  //! RX pipe 1 is used to store our address since the address in RX pipe 0 is used for auto-acknowledgment packets.
  NRF24L01_SetSetOfRegisters(NRF24L01_REG_RX_ADDR_P1, pCtx->addrP1, sizeof(pCtx->addrP1));

  //! Enable dynamically sized packets on the 2 RX pipes, P0 and P1.
  memset(&dynpd, 0x00, sizeof(dynpd));
  dynpd.cnvrt.dplP0 = NRF24L01_DYNPD_DPL_P0_BIT; //! RX P0 is used to for auto-acknowledgment packets from radios we transmit to.
  dynpd.cnvrt.dplP1 = NRF24L01_DYNPD_DPL_P1_BIT; //! RX P1 is used to for normal packets from radios that send us data.
  NRF24L01_SetRegister(NRF24L01_REG_DYNPD, dynpd.reg);

  memset(&feature, 0x00, sizeof(feature));
  feature.cnvrt.enDynAck = NRF24L01_FEATURE_EN_DYN_ACK_BIT; //! Enables the W_TX_PAYLOAD_NOACK command.
  feature.cnvrt.enAckPay = NRF24L01_FEATURE_EN_ACK_PAY_BIT; //! Enables Payload with ACK.
  feature.cnvrt.enDpl    = NRF24L01_FEATURE_EN_DPL_BIT;     //! Enables Dynamic Payload Length.
  NRF24L01_SetRegister(NRF24L01_REG_FEATURE, feature.reg);

  //! NRF24L01_REG_EN_AA - Enable or disable auto-acknowlede packets.
  //! This is enabled by default, so it's only needed if you want to turn it off for some reason (to be compatible with nRF2401).

  //! Ensure RX and TX buffers are empty. Each buffer can hold 3 packets.
  NRF24L01_FlushRx();
  NRF24L01_FlushTx();

  memset(&status, 0x00, sizeof(status));
  status.cnvrt.maxRt = NRF24L01_STATUS_MAX_RT_BIT;
  status.cnvrt.txDs  = NRF24L01_STATUS_TX_DS_BIT;
  status.cnvrt.rxDr  = NRF24L01_STATUS_RX_DR_BIT;
  NRF24L01_SetRegister(NRF24L01_REG_STATUS, status.reg); //! Clear any interrupts.

  return NRF24L01_StartRx();
}

/**************************************************************************/

static NRF24L01_ERROR NRF24L01_WaitForTxToComplete(void)
{
  PNRF24L01_CTX                 pCtx   = (PNRF24L01_CTX) &nrf24l01_ctx;
  NRF24L01_REG_FIFO_STATUS_TYPE fifoStatus;
  NRF24L01_REG_STATUS_TYPE      status;
  uint8_t                       txAttemptCount = 0x00;
  NRF24L01_ERROR                retval         = NRF24L01_FAIL;

  while ( txAttemptCount < NRF24L01_MAX_TX_ATTEMPT_COUNT )
  {
    memset(&fifoStatus, 0x00, sizeof(fifoStatus));
    fifoStatus.reg = NRF24L01_ReadRegister(NRF24L01_REG_FIFO_STATUS);

    if ( fifoStatus.cnvrt.txEmpty )
    {
      retval = NRF24L01_SUCCESS; //! Indicate success.
      break;
    }

    //! CE will be LOW so we must toggle it to send a packet.
    pCtx->pHal->digitalWriteCePin(NRF24L01_PIN_HIGH);
    TIME_DelayUs(NRF24L01_CE_TRANSMISSION_MICROS);
    pCtx->pHal->digitalWriteCePin(NRF24L01_PIN_LOW);

    TIME_DelayUs(NRF24L01_TRANSMISSION_RETRY_WAIT_MICROS);

    memset(&status, 0x00, sizeof(status));
    status.reg = NRF24L01_ReadRegister(NRF24L01_REG_STATUS);

    if ( status.cnvrt.txDs ) //! If the packet was sent.
    {
      status.cnvrt.txDs = NRF24L01_STATUS_TX_DS_BIT;
      NRF24L01_SetRegister(NRF24L01_REG_STATUS, status.reg); //! Clear TX success flag.
    }
    else if ( status.cnvrt.maxRt ) //! The packet could not be sent.
    {
      NRF24L01_FlushTx(); //! Clear TX buffer.
      status.cnvrt.maxRt = NRF24L01_STATUS_MAX_RT_BIT;
      NRF24L01_SetRegister(NRF24L01_REG_STATUS, status.reg); //! Clear max retry flag.
      break;
    }

    ++txAttemptCount;
  }

  return retval;
}

/**************************************************************************/

static NRF24L01_ERROR NRF24L01_StartRx(void)
{
  PNRF24L01_CTX            pCtx   = (PNRF24L01_CTX) &nrf24l01_ctx;
  NRF24L01_ERROR           retval = NRF24L01_FAIL;
  NRF24L01_REG_CONFIG_TYPE config;

  NRF24L01_WaitForTxToComplete();

  //! Put radio into Standby-I mode in order to transition into RX mode.
  pCtx->pHal->digitalWriteCePin(NRF24L01_PIN_LOW);

  //! Configure the radio for receiving.
  memset(&config, 0x00, sizeof(config));
  config.cnvrt.primRx = NRF24L01_CONFIG_PRIM_RX_BIT;
  config.cnvrt.pwrUP  = NRF24L01_CONFIG_PWR_UP_BIT;
  config.cnvrt.enCrc  = NRF24L01_CONFIG_EN_CRC_BIT;
  NRF24L01_SetRegister(NRF24L01_REG_CONFIG, config.reg);

  //! Put radio into RX mode.
  pCtx->pHal->digitalWriteCePin(NRF24L01_PIN_HIGH);

  //! Wait for the transition into RX mode.
  TIME_DelayMs(NRF24L01_POWERDOWN_TO_RXTX_MODE_MILLIS);

  memset(&config, 0x00, sizeof(config));
  config.reg = NRF24L01_ReadRegister(NRF24L01_REG_CONFIG);

  if ((NRF24L01_CONFIG_PRIM_RX_BIT == config.cnvrt.primRx) &&
      (NRF24L01_CONFIG_PWR_UP_BIT == config.cnvrt.pwrUP)   &&
      (NRF24L01_CONFIG_EN_CRC_BIT == config.cnvrt.enCrc)     )
  {
    retval = NRF24L01_SUCCESS;
  }

  return retval;
}

/**************************************************************************/

static void NRF24L01_PrepareForTx(uint8_t toRadioId)
{
  PNRF24L01_CTX                 pCtx   = (PNRF24L01_CTX) &nrf24l01_ctx;
  NRF24L01_REG_CONFIG_TYPE      config;
  NRF24L01_REG_FIFO_STATUS_TYPE fifoStatus;
  //! TX pipe address sets the destination radio for the data.
  //! RX pipe 0 is special and needs the same address in order to receive ACK packets from the destination radio.
  pCtx->addrP1[0] = toRadioId;
  //! addrP1 = {0x00, 0x00, 0x00, 0x00, radioId}.
  NRF24L01_SetSetOfRegisters(NRF24L01_REG_TX_ADDR, pCtx->addrP1, sizeof(pCtx->addrP1));
  NRF24L01_SetSetOfRegisters(NRF24L01_REG_RX_ADDR_P0, pCtx->addrP1, sizeof(pCtx->addrP1));

  //! Ensure radio is ready for TX operation.
  memset(&config, 0x00, sizeof(config));
  config.reg = NRF24L01_ReadRegister(NRF24L01_REG_CONFIG);
  if (config.cnvrt.primRx)
  {
    //! Put radio into Standby-I mode in order to transition into TX mode.
    pCtx->pHal->digitalWriteCePin(NRF24L01_PIN_LOW);
    config.cnvrt.primRx = !NRF24L01_CONFIG_PRIM_RX_BIT; //! Set to 0b.
    NRF24L01_SetRegister(NRF24L01_REG_CONFIG, config.reg);
    TIME_DelayMs(NRF24L01_POWERDOWN_TO_RXTX_MODE_MILLIS);
  }

  memset(&fifoStatus, 0x00, sizeof(fifoStatus));
  fifoStatus.reg = NRF24L01_ReadRegister(NRF24L01_REG_FIFO_STATUS);

  //! If RX buffer is full and we require an ACK, clear it so we can receive the ACK response.
  if ( fifoStatus.cnvrt.rxFull )
  {
    NRF24L01_FlushRx();
  }

  //! If TX buffer is full, wait for all queued packets to be sent.
  if ( fifoStatus.cnvrt.txFull )
  {
    NRF24L01_WaitForTxToComplete();
  }
}

/**************************************************************************/

NRF24L01_ERROR NRF24L01_SendData(uint8_t toRadioId, uint8_t *data, uint8_t length)
{
  PNRF24L01_CTX            pCtx = (PNRF24L01_CTX) &nrf24l01_ctx;
  PNRF24L01_HAL_REG        pHal = (PNRF24L01_HAL_REG) pCtx->pHal;
  NRF24L01_REG_STATUS_TYPE status;

  //! TODO: Add the check of NRF24L01_MAX_PAYLOAD_DATA_LEN.

  NRF24L01_PrepareForTx(toRadioId);

  //! Clear any previously asserted TX success or max retries flags.
  memset(&status, 0x00, sizeof(status));
  status.cnvrt.txDs  = NRF24L01_STATUS_TX_DS_BIT;
  status.cnvrt.maxRt = NRF24L01_STATUS_MAX_RT_BIT;
  NRF24L01_SetRegister(NRF24L01_REG_STATUS, status.reg);

  //! Add data to the TX buffer, with an ACK request.
  pHal->spiBegin(); //! Csn is low.
  pHal->spiTransfer(NRF24L01_INSTR_SET_W_TX_PAYLOAD);
  for (uint8_t i = 0; i < length; i++)
  {
    pHal->spiTransfer(data[i]);
  }
  pHal->spiEnd(); //! Csn is high.

  return NRF24L01_WaitForTxToComplete();
}

/**************************************************************************
 * Returns the length of the data packet in the RX buffer.                *
 **************************************************************************
 */
uint8_t NRF24L01_HasData(void)
{
  PNRF24L01_CTX            pCtx       = (PNRF24L01_CTX) &nrf24l01_ctx;
  PNRF24L01_HAL_REG        pHal       = (PNRF24L01_HAL_REG) pCtx->pHal;
  uint8_t                  dataLength = 0x00;
  NRF24L01_REG_CONFIG_TYPE config;
  NRF24L01_REG_STATUS_TYPE status;

  memset(&config, 0x00, sizeof(config));
  config.reg = NRF24L01_ReadRegister(NRF24L01_REG_CONFIG);

  //! Check if NRF24L01 is in RX mode.
  if ( NRF24L01_CONFIG_PRIM_RX_BIT != config.cnvrt.primRx )
  {
    NRF24L01_StartRx();
  }

  //! If we have a pipe 1 packet sitting at the top of the RX buffer, we have data.
  memset(&status, 0x00, sizeof(status));
  status.reg = NRF24L01_ReadRegister(NRF24L01_REG_STATUS);
  if ( status.cnvrt.rxPNo == 0x01 )
  {
    //! Get RX packet length.
    pHal->spiBegin(); //! Csn is low.
    pHal->spiTransfer(NRF24L01_INSTR_SET_R_RX_PL_WID); //! Read the length of the first data packet sitting in the RX buffer.
    dataLength = pHal->spiTransfer(NRF24L01_INSTR_SET_NOP);
    pHal->spiEnd(); //! Csn is high

    //! Verify the data length is valid (0 - 32 bytes).
    if (dataLength > NRF24L01_MAX_PAYLOAD_DATA_LEN)
    {
      NRF24L01_FlushRx(); //! Clear invalid data in the RX buffer.

      memset(&status, 0x00, sizeof(status));
      status.reg = NRF24L01_ReadRegister(NRF24L01_REG_STATUS);

      status.cnvrt.maxRt = NRF24L01_STATUS_MAX_RT_BIT;
      status.cnvrt.txDs  = NRF24L01_STATUS_TX_DS_BIT;
      status.cnvrt.rxDr  = NRF24L01_STATUS_RX_DR_BIT;
      NRF24L01_SetRegister(NRF24L01_REG_STATUS, status.reg);

      dataLength = 0x00;
    }
  }

  return dataLength; //! Return the length of the data packet in the RX buffer.
}

/**************************************************************************/

void NRF24L01_ReadData(uint8_t *data)
{
  PNRF24L01_CTX            pCtx       = (PNRF24L01_CTX) &nrf24l01_ctx;
  PNRF24L01_HAL_REG        pHal       = (PNRF24L01_HAL_REG) pCtx->pHal;
  uint8_t                  dataLength = 0x00;
  NRF24L01_REG_STATUS_TYPE status;

  //! Determine length of data in the RX buffer and read it.
  pHal->spiBegin(); //! Csn is low.
  pHal->spiTransfer(NRF24L01_INSTR_SET_R_RX_PL_WID);
  dataLength = pHal->spiTransfer(NRF24L01_INSTR_SET_NOP);
  pHal->spiEnd(); //! Csn is high.
//! TODO: Rewrite this part of code.
  pHal->spiBegin(); //! Csn is low.
  pHal->spiTransfer(NRF24L01_INSTR_SET_R_RX_PAYLOAD);
  for (uint8_t i = 0; i < dataLength; i++)
  {
    data[i] = pHal->spiTransfer(NRF24L01_INSTR_SET_NOP);
  }
  pHal->spiEnd(); //! Csn is high.

  //! Clear data received flag.
  memset(&status, 0x00, sizeof(status));
  status.reg = NRF24L01_ReadRegister(NRF24L01_REG_STATUS);
  if ( status.cnvrt.rxDr )
  {
    status.cnvrt.rxDr  = NRF24L01_STATUS_RX_DR_BIT;
    NRF24L01_SetRegister(NRF24L01_REG_STATUS, status.reg);
  }
}
