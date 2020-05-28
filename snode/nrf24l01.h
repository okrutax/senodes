/**************************************************************************
 * @file       nrf24l01.h                                                 *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      The NRF24L01P+ driver.                                     *
 *************************************************************************/

#ifndef _NRF24L01_H_
#define _NRF24L01_H_ 

#ifdef _cplusplus_
extern "C"
{
#endif

#include <stdint.h>

/**************************************************************************
 * Configuration of NRF24L01.                                             *
 **************************************************************************
 */
//! TX buffer can store 3 packets, sends retry up to 15 times, and the retry wait time is about half
//! the time necessary to send a 32 byte packet and receive a 32 byte ACK response. 3 x 15 x 2 = 90.
#define NRF24L01_MAX_TX_ATTEMPT_COUNT                 (90)

#define NRF24L01_TRANSMISSION_RETRY_WAIT_MICROS       (600) //! Micro seconds. 100 more than the retry delay.
#define NRF24L01_CE_TRANSMISSION_MICROS               (10)  //! Time to initiate data transmission.
#define NRF24L01_POWERDOWN_TO_RXTX_MODE_MILLIS        (5)   //! 4500uS to Standby + 130uS to RX or TX mode, so 5ms is enough.
#define NRF24L01_OFF_TO_POWERDOWN_MILLIS              (5)

#define NRF24L01_MAX_PAYLOAD_DATA_LEN                 (32)

/**************************************************************************
 * Register map table.                                                    *
 **************************************************************************
 */
typedef enum _NRF24L01_REGISTER_MAP_TABLE
{
  NRF24L01_REG_CONFIG      = 0x00, //! Configuration Register.
  NRF24L01_REG_EN_AA       = 0x01, //! Enable ‘Auto Acknowledgment’.
  NRF24L01_REG_EN_RXADDR   = 0x02, //! Enabled RX Addresses.
  NRF24L01_REG_SETUP_AW    = 0x03, //! Setup of Address Widths(common for all data pipes).
  NRF24L01_REG_SETUP_RETR  = 0x04, //! Setup of Automatic Retransmission.
  NRF24L01_REG_RF_CH       = 0x05, //! RF Channel.
  NRF24L01_REG_RF_SETUP    = 0x06, //! RF Setup Register.
  NRF24L01_REG_STATUS      = 0x07, //! Status Register (In parallel to the SPI command word applied on the MOSI pin, the STATUS register is shifted serially out on the MISO pin).
  NRF24L01_REG_OBSERVE_TX  = 0x08, //! Transmit observe register.
  NRF24L01_REG_CD          = 0x09, //! Carrier Detect.
  NRF24L01_REG_RX_ADDR_P0  = 0x0A, //! Receive address data pipe 0.
  NRF24L01_REG_RX_ADDR_P1  = 0x0B, //! Receive address data pipe 1.
  NRF24L01_REG_RX_ADDR_P2  = 0x0C, //! Receive address data pipe 2.
  NRF24L01_REG_RX_ADDR_P3  = 0x0D, //! Receive address data pipe 3.
  NRF24L01_REG_RX_ADDR_P4  = 0x0E, //! Receive address data pipe 4.
  NRF24L01_REG_RX_ADDR_P5  = 0x0F, //! Receive address data pipe 5.
  NRF24L01_REG_TX_ADDR     = 0x10, //! Transmit address. Used for a PTX device only.
  NRF24L01_REG_RX_PW_P0    = 0x11, //! Number of bytes in RX payload in data pipe 0 (1 to 32 bytes).
  NRF24L01_REG_RX_PW_P1    = 0x12, //! Number of bytes in RX payload in data pipe 1 (1 to 32 bytes).
  NRF24L01_REG_RX_PW_P2    = 0x13, //! Number of bytes in RX payload in data pipe 2 (1 to 32 bytes).
  NRF24L01_REG_RX_PW_P3    = 0x14, //! Number of bytes in RX payload in data pipe 3 (1 to 32 bytes).
  NRF24L01_REG_RX_PW_P4    = 0x15, //! Number of bytes in RX payload in data pipe 4 (1 to 32 bytes).
  NRF24L01_REG_RX_PW_P5    = 0x16, //! Number of bytes in RX payload in data pipe 5 (1 to 32 bytes).
  NRF24L01_REG_FIFO_STATUS = 0x17, //! FIFO Status Register.
  NRF24L01_REG_DYNPD       = 0x1C, //! Enable dynamic payload length.
  NRF24L01_REG_FEATURE     = 0x1D  //! Feature Register.
}NRF24L01_REGISTER_MAP_TABLE, *PNRF24L01_REGISTER_MAP_TABLE;

/**************************************************************************
 * Spi instruction set.                                                   *
 **************************************************************************
 */
typedef enum _NRF24L01_NSTRUCTION_SET
{
  NRF24L01_INSTR_SET_R_REGISTER          = 0x00,
  NRF24L01_INSTR_SET_W_REGISTER          = 0x20,
#define NRF24L01_INSTR_SET_REGISTER_MASK           (0x1F)
  NRF24L01_INSTR_SET_R_RX_PAYLOAD        = 0x61,
  NRF24L01_INSTR_SET_W_TX_PAYLOAD        = 0xA0,
  NRF24L01_INSTR_SET_FLUSH_TX            = 0xE1,
  NRF24L01_INSTR_SET_FLUSH_RX            = 0xE2,
  NRF24L01_INSTR_SET_REUSE_TX_PL         = 0xE3,
//NRF24L01_INSTR_SET_ACTIVATE            = 0x50,
  NRF24L01_INSTR_SET_R_RX_PL_WID         = 0x60,
  NRF24L01_INSTR_SET_W_ACK_PAYLOAD       = 0xA8,
  NRF24L01_INSTR_SET_W_TX_PAYLOAD_NO_ACK = 0xB0,
  NRF24L01_INSTR_SET_NOP                 = 0xFF
}NRF24L01_NSTRUCTION_SET, *PNRF24L01_NSTRUCTION_SET;

/**************************************************************************
 *  NRF24L01_REG_CONFIG Register. Configuration Register. (ADDRESS: 00h)  *
 **************************************************************************
 */
typedef enum _NRF24L01_CONFIG_TYPE
{
  NRF24L01_CONFIG_PRIM_RX_BIT     = 0x01,
  NRF24L01_CONFIG_PWR_UP_BIT      = 0x01,
  NRF24L01_CONFIG_CRCO_BIT        = 0x01,
  NRF24L01_CONFIG_EN_CRC_BIT      = 0x01,
  NRF24L01_CONFIG_MASK_MAX_RT_BIT = 0x01,
  NRF24L01_CONFIG_MASK_TX_DS_BIT  = 0x01,
  NRF24L01_CONFIG_MASK_RX_DR_BIT  = 0x01
}NRF24L01_CONFIG_TYPE, *PNRF24L01_CONFIG_TYPE;

/**************************************************************************/

typedef union _NRF24L01_REG_CONFIG_TYPE
{
  struct _convert
  {
    uint8_t primRx    :1; //! R/W RX/TX control. 1: PRX, 0: PTX.
    uint8_t pwrUP     :1; //! R/W 1: POWER UP, 0:POWER DOWN.
    uint8_t crco      :1; //! R/W CRC encoding scheme '0' - 1 byte '1' – 2 bytes.
    uint8_t enCrc     :1; //! R/W Enable CRC. Forced high if one of the bits in the EN_AA is high.
    uint8_t maskMaxRt :1; //! R/W Mask interrupt caused by MAX_RT 1: Interrupt not reflected on the IRQ pin 0: Reflect MAX_RT as active low interrupt on the IRQ pin.
    uint8_t maskTxDs  :1; //! R/W Mask interrupt caused by TX_DS 1: Interrupt not reflected on the IRQ pin 0: Reflect TX_DS as active low interrupt on the IRQ pin.
    uint8_t maskRXDr  :1; //! R/W Mask interrupt caused by RX_DR 1: Interrupt not reflected on the IRQ pin 0: Reflect RX_DR as active low interrupt on the IRQ pin.
    uint8_t reserved  :1; //! R/W Only '0' allowed.
  } cnvrt;                //! Convert.
  uint8_t reg;
}NRF24L01_REG_CONFIG_TYPE, *PNRF24L01_REG_CONFIG_TYPE;

/**************************************************************************
 *         NRF24L01_REG_RF_CH Register. RF Channel. (ADDRESS: 05h)        *
 **************************************************************************
 */
typedef union _NRF24L01_REG_RF_CH_TYPE
{
  struct _convert
  {
    uint8_t rfChannel :7; //! Sets the frequency channel nRF24L01 operates on.
    uint8_t reserved  :1;
  } cnvrt;                //! Convert.
  uint8_t reg;
}NRF24L01_REG_RF_CH_TYPE, *PNRF24L01_REG_RF_CH_TYPE;

/**************************************************************************
 *    NRF24L01_REG_RF_SETUP Register. RF Setup Register. (ADDRESS: 06h)   *
 **************************************************************************
 */
typedef enum _NRF24L01_RF_SETUP_RF_PWR_TYPE //! Output power.
{
  NRF24L01_RF_SETUP_RF_PWR_M_18DBM = 0x00, //! -18 dBm.
  NRF24L01_RF_SETUP_RF_PWR_M_12DBM = 0x01, //! -12 dBm.
  NRF24L01_RF_SETUP_RF_PWR_M_6DBM  = 0x02, //!  -6 dBm.
  NRF24L01_RF_SETUP_RF_PWR_0DBM    = 0x03  //!   0 dBm.
}NRF24L01_RF_SETUP_RF_PWR_TYPE, *PNRF24L01_RF_SETUP_RF_PWR_TYPE;

/**************************************************************************/

typedef enum _NRF24L01_RF_SETUP_RF_DR_TYPE //! Data rate.
{
  RF24L01_RF_SETUP_RF_DR_1MBPS = 0x00,
  RF24L01_RF_SETUP_RF_DR_2MBPS = 0x01
}NRF24L01_RF_SETUP_RF_DR_TYPE, *PNRF24L01_RF_SETUP_RF_DR_TYPE;

/**************************************************************************/

typedef union _NRF24L01_REG_RF_SETUP_TYPE
{
  struct _convert
  {
    uint8_t obsolete :1; //! Don't care.
    uint8_t rfPwr    :2; //! R/W Set RF output power in TX mode.
    uint8_t rfDrHigh :1; //! R/W Air Data Rate.
    uint8_t pllLock  :1; //! R/W Force PLL lock signal. Only used in test.
    uint8_t rfDrLow  :1; //! R/W Set RF Data Rate to 250kBps. See rfDrHigh.
    uint8_t reserved :1; //! R/W Only '0' allowed.
    uint8_t contWave :1; //! R/W Enables continuous carrier transmit when high.
  } cnvrt;               //! Convert.
  uint8_t reg;
}NRF24L01_REG_RF_SETUP_TYPE, *PNRF24L01_REG_RF_SETUP_TYPE;

/**************************************************************************
 *NRF24L01_REG_SETUP_RETR Register. RF Setup Petr Register. (ADDRESS: 04h)*
 **************************************************************************
 */
typedef union _NRF24L01_REG_SETUP_RETR_TYPE
{
  struct _convert
  {
    uint8_t arc :4; //! R/W Auto Retransmit Count.
    uint8_t ard :4; //! R/W Auto Retransmit Delay.
  } cnvrt;          //! Convert.
  uint8_t reg;
}NRF24L01_REG_SETUP_RETR_TYPE, *PNRF24L01_REG_SETUP_RETR_TYPE;

/**************************************************************************
 *   NRF24L01_REG_DYNPD Register. Dynamic payload length.(ADDRESS: 1Сh)   *
 **************************************************************************
 */
typedef enum _NRF24L01_DYNPD_TYPE
{
  NRF24L01_DYNPD_DPL_P0_BIT = 0x01,
  NRF24L01_DYNPD_DPL_P1_BIT = 0x01,
  NRF24L01_DYNPD_DPL_P2_BIT = 0x01,
  NRF24L01_DYNPD_DPL_P3_BIT = 0x01,
  NRF24L01_DYNPD_DPL_P4_BIT = 0x01,
  NRF24L01_DYNPD_DPL_P5_BIT = 0x01
}NRF24L01_DYNPD_TYPE, *PNRF24L01_DYNPD_TYPE;

/**************************************************************************/

typedef union _NRF24L01_REG_DYNPD_TYPE
{
  struct _convert
  {
    uint8_t dplP0 :1; //! R/W Enable dynamic payload length data pipe 0.(Requires EN_DPL and ENAA_P0).
    uint8_t dplP1 :1; //! R/W Enable dynamic payload length data pipe 1.(Requires EN_DPL and ENAA_P1).
    uint8_t dplP2 :1; //! R/W Enable dynamic payload length data pipe 2.(Requires EN_DPL and ENAA_P2).
    uint8_t dplP3 :1; //! R/W Enable dynamic payload length data pipe 3.(Requires EN_DPL and ENAA_P3).
    uint8_t dplP4 :1; //! R/W Enable dynamic payload length data pipe 4.(Requires EN_DPL and ENAA_P4).
    uint8_t dplP5 :1; //! R/W Enable dynamic payload length data pipe 5.(Requires EN_DPL and ENAA_P5).
    uint8_t reserved :2; //! R/W Only ‘00’ allowed.
  } cnvrt;            //! Convert.
  uint8_t reg;
}NRF24L01_REG_DYNPD_TYPE, *PNRF24L01_REG_DYNPD_TYPE;

/**************************************************************************
 *     NRF24L01_REG_FEATURE Register. Feature Register.(ADDRESS: 1Dh)     *
 **************************************************************************
 */
typedef enum _NRF24L01_FEATURE_TYPE
{
  NRF24L01_FEATURE_EN_DYN_ACK_BIT = 0x01,
  NRF24L01_FEATURE_EN_ACK_PAY_BIT = 0x01,
  NRF24L01_FEATURE_EN_DPL_BIT     = 0x01
}NRF24L01_FEATURE_TYPE, *PNRF24L01_FEATURE_TYPE;

/**************************************************************************/

typedef union _NRF24L01_REG_FEATURE_TYPE
{
  struct _convert
  {
    uint8_t enDynAck :1; //! R/W Enables the W_TX_PAYLOAD_NOACK command.
    uint8_t enAckPay :1; //! R/W Enables Payload with ACK.
    uint8_t enDpl    :1; //! R/W Enables Dynamic Payload Length.
    uint8_t reserved :5; //! R/W Only ‘00000’ allowed.
  } cnvrt;               //! Convert.
  uint8_t reg;
}NRF24L01_REG_FEATURE_TYPE, *PNRF24L01_REG_FEATURE_TYPE;

/**************************************************************************
 *NRF24L01_REG_EN_AA Register. Enable ‘Auto Acknowledgment’.(ADDRESS: 01h)*
 **************************************************************************
 */
typedef union _NRF24L01_REG_EN_AA_TYPE
{
  struct _convert
  {
    uint8_t enaaP0 :1; //! R/W Enable auto acknowledgement data pipe 0.
    uint8_t enaaP1 :1; //! R/W Enable auto acknowledgement data pipe 1.
    uint8_t enaaP2 :1; //! R/W Enable auto acknowledgement data pipe 2.
    uint8_t enaaP3 :1; //! R/W Enable auto acknowledgement data pipe 3.
    uint8_t enaaP4 :1; //! R/W Enable auto acknowledgement data pipe 4.
    uint8_t enaaP5 :1; //! R/W Enable auto acknowledgement data pipe 5.
    uint8_t reserved :2; //! R/W Only '00' allowed.
  } cnvrt;             //! Convert.
  uint8_t reg;
}NRF24L01_REG_EN_AA_TYPE, *PNRF24L01_REG_EN_AA_TYPE;

/**************************************************************************
 *     NRF24L01_REG_STATUS Register. Status Register.(ADDRESS: 07h)       *
 **************************************************************************
 */
typedef enum _NRF24L01_STATUS_TYPE
{
  NRF24L01_STATUS_MAX_RT_BIT = 0x01,
  NRF24L01_STATUS_TX_DS_BIT  = 0x01,
  NRF24L01_STATUS_RX_DR_BIT  = 0x01
}NRF24L01_STATUS_TYPE, *PNRF24L01_STATUS_TYPE;

/**************************************************************************/

typedef union _NRF24L01_REG_STATUS_TYPE
{
  struct _convert
  {
    uint8_t txFull :1; //! R TX FIFO full flag. 1: TX FIFO full. 0: Available locations in TX FIFO.
    uint8_t rxPNo  :3; //! R Data pipe number for the payload available for reading from RX_FIFO. 000-101: Data Pipe Number. 110: Not Used. 111: RX FIFO Empty.
    uint8_t maxRt  :1; //! R/W Maximum number of TX retransmits interrupt. Write 1 to clear bit. If MAX_RT is asserted it must be cleared to enable further communication.
    uint8_t txDs   :1; //! R/W Data Sent TX FIFO interrupt. Asserted when packet transmitted on TX. If AUTO_ACK is activated, this bit is set high only when ACK is received. Write 1 to clear bit.
    uint8_t rxDr   :1; //! R/W Data Ready RX FIFO interrupt. Asserted when new data arrives RX FIFO. Write 1 to clear bit.
    uint8_t reserved :1; //! R/W Only '0' allowed.
  } cnvrt;             //! Convert.
  uint8_t reg;
}NRF24L01_REG_STATUS_TYPE, *PNRF24L01_REG_STATUS_TYPE;

/**************************************************************************
 * NRF24L01_REG_FIFO_STATUS Register. FIFO Status Register.(ADDRESS: 17h) *
 **************************************************************************
 */
typedef union _NRF24L01_REG_FIFO_STATUS_TYPE
{
  struct _convert
  {
    uint8_t rxEmpty   :1; //! R RX FIFO empty flag. 1: RX FIFO empty. 0: Data in RX FIFO.
    uint8_t rxFull    :1; //! R RX FIFO full flag. 1: RX FIFO full. 0: Available locations in RX FIFO.
    uint8_t reserved1 :2; //! R/W Only '00' allowed.
    uint8_t txEmpty   :1; //! R TX FIFO empty flag. 1: TX FIFO empty. 0: Data in TX FIFO.
    uint8_t txFull    :1; //! R TX FIFO full flag. 1: TX FIFO full. 0: Available locations in TX FIFO.
    uint8_t txReuse   :1; //! R Used for a PTX device.
    uint8_t reserved2 :1; //! R/W Only '0' allowed.
  } cnvrt;                //! Convert.
  uint8_t reg;
}NRF24L01_REG_FIFO_STATUS_TYPE, *PNRF24L01_REG_FIFO_STATUS_TYPE;

/**************************************************************************/

typedef enum _NRF24L01_ERROR
{
  NRF24L01_SUCCESS = 0x00, // ERROR_OK
  NRF24L01_FAIL    = 0x01
}NRF24L01_ERROR, *PNRF24L01_ERROR;

/**************************************************************************/

typedef enum _NRF24L01_DIGITAL_PIN
{
  NRF24L01_PIN_LOW  = 0x00,
  NRF24L01_PIN_HIGH = 0x01
}NRF24L01_DIGITAL_PIN, *PNRF24L01_DIGITAL_PIN;

/**************************************************************************/

typedef NRF24L01_ERROR (* NRF24L01_HAL_SPI_INIT)(void);
typedef uint8_t        (* NRF24L01_HAL_SPI_DATA_TRANSFER)(const uint8_t data);
typedef NRF24L01_ERROR (* NRF24L01_HAL_SPI_BEGIN_END_TRANS)(void); //! Callback of begin and end transactions.
typedef void           (* NRF24L01_DIGITAL_WRITE)(NRF24L01_DIGITAL_PIN value);

/**************************************************************************
 * The HAL of NRF registration.                                           *
 **************************************************************************
 */
typedef struct _NRF24L01_HAL_REG
{
//const uint8_t                          cePin;    //! "Chip Enable" pin, activates the RX or TX role.
//const uint8_t                          csnPin;   //! SPI Chip select.
  const NRF24L01_DIGITAL_WRITE           digitalWriteCePin;
  const NRF24L01_HAL_SPI_INIT            spiInit;
  const NRF24L01_HAL_SPI_DATA_TRANSFER   spiTransfer;
  const NRF24L01_HAL_SPI_BEGIN_END_TRANS spiBegin; //! Transactions.
  const NRF24L01_HAL_SPI_BEGIN_END_TRANS spiEnd;   //! Transactions.
}NRF24L01_HAL_REG, *PNRF24L01_HAL_REG;

/**************************************************************************/

typedef struct _NRF24L01_CTX
{
  PNRF24L01_HAL_REG pHal;
  uint8_t           addrP1[5];
}NRF24L01_CTX, *PNRF24L01_CTX;

/**************************************************************************
 * Global function declarations.                                          *
 **************************************************************************
 */
NRF24L01_ERROR NRF24L01_Init(PNRF24L01_HAL_REG pInit, uint8_t radioId, uint8_t channel);
NRF24L01_ERROR NRF24L01_SendData(uint8_t toRadioId, uint8_t *data, uint8_t length);
uint8_t        NRF24L01_HasData(void);
void           NRF24L01_ReadData(uint8_t *data);

#ifdef _cplusplus
}
#endif

#endif // _NRF24L01_H_