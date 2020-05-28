/**************************************************************************
 * @file       mcp2515.h                                                  *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      This MCP2515 driver implements Controller Area Network(CAN)*
 *             controller.                                                *
 *************************************************************************/

#ifndef _MCP2515_H_ // MCP2515
#define _MCP2515_H_ 

#ifdef _cplusplus_
extern "C"
{
#endif

#include <stdint.h>
#include <string.h>

/**************************************************************************
 *                         MCP2515 Configuration                          *
 **************************************************************************
 */
#define  MCP2515_TXB_NUM_OF_BUFFERS  (0x03)
#define  MCP2515_RXB_NUM_OF_BUFFERS  (0x02)
#define  MCP2515_MAX_TIMEOUT_MS      (0x03) //! 3 ms.

/**************************************************************************/

typedef enum _MCP2515_ERROR
{
  MCP2515_SUCCESS                 = 0x00, //! ERROR_OK
  MCP2515_FAIL                    = 0x01,
//MCP2515_FAIL_INIT               = 0x03,
//MCP2515_FAIL_TX                 = 0x04,
  MCP2515_RX_NO_MSG_DATA          = 0x05,
  MCP2515_TX_NO_FREE_BUFFER       = 0x06,
  MCP2515_TX_DATA_STILL_IN_BUFFER = 0x07,
  MCP2515_TX_FAIL_MAX_LEN         = 0x08
}MCP2515_ERROR, *PMCP2515_ERROR;

/**************************************************************************/

typedef MCP2515_ERROR  (* MCP2515_HAL_SPI_INIT)(void);
typedef uint8_t        (* MCP2515_HAL_SPI_DATA_TRANSFER)(const uint8_t data);
typedef MCP2515_ERROR  (* MCP2515_HAL_SPI_BEGIN_END_TRANS)(void); //! Callback of begin and end transactions.

/**************************************************************************
 *   The structure of a HAL registaration of the mcp2515 init function    *
 **************************************************************************
 */
typedef struct _MCP2515_HAL_REG
{
  const MCP2515_HAL_SPI_INIT            spiInit;
  const MCP2515_HAL_SPI_DATA_TRANSFER   spiTransfer;
  const MCP2515_HAL_SPI_BEGIN_END_TRANS spiBegin; //! Transactions.
  const MCP2515_HAL_SPI_BEGIN_END_TRANS spiEnd;   //! Transactions.
}MCP2515_HAL_REG, *PMCP2515_HAL_REG;

/**************************************************************************
 *                          MCP2515 Registers                             *
 **************************************************************************
 */
typedef enum _MCP2515_NSTRUCTION_SET //! SPI instruction set.
{
  MCP2515_INSTR_SET_WRITE       = 0x02, //! Writes data to the register beginning at the selected address.
  MCP2515_INSTR_SET_READ        = 0x03, //! Reads data from the register beginning at selected address.
  MCP2515_INSTR_SET_BIT_MODIFY  = 0x05, //! Allows the user to set or clear individual bits in a particular register.
  MCP2515_INSTR_SET_LOAD_TX0    = 0x40, //! When loading a transmit buffer, reduces the overhead of a normal WRITE command.
  MCP2515_INSTR_SET_LOAD_TX1    = 0x42,
  MCP2515_INSTR_SET_LOAD_TX2    = 0x44,
  MCP2515_INSTR_SET_RTS_TX0     = 0x81, //! Instructs controller to begin message transmission sequence for any of the transmit buffers. 
  MCP2515_INSTR_SET_RTS_TX1     = 0x82, //! Request-to-Send for TXB1.
  MCP2515_INSTR_SET_RTS_TX2     = 0x84, //! Request-to-Send for TXB2.
  MCP2515_INSTR_SET_RTS_ALL     = 0x87,
  MCP2515_INSTR_SET_READ_RX0    = 0x90, //! When reading a receive buffer, reduces the overhead of a normal READ command.
  MCP2515_INSTR_SET_READ_RX1    = 0x94,
  MCP2515_INSTR_SET_READ_STATUS = 0xA0, //! Quick polling command that reads several status bits for transmit and receive functions.
  MCP2515_INSTR_SET_RX_STATUS   = 0xB0, //! Quick polling command that indicates filter match and message type (standard, extended and/or remote) of received message.
  MCP2515_INSTR_SET_RESET       = 0xC0  //! Resets internal registers to the default state, sets Configuration mode.
}MCP2515_NSTRUCTION_SET, *PMCP2515_NSTRUCTION_SET;

/**************************************************************************/

typedef enum _MCP2515_CONTROL_REGISTERS
{
  MCP2515_CTRL_REG_RXF0SIDH = 0x00,
  MCP2515_CTRL_REG_RXF0SIDL = 0x01,
  MCP2515_CTRL_REG_RXF0EID8 = 0x02,
  MCP2515_CTRL_REG_RXF0EID0 = 0x03,
  MCP2515_CTRL_REG_RXF1SIDH = 0x04,
  MCP2515_CTRL_REG_RXF1SIDL = 0x05,
  MCP2515_CTRL_REG_RXF1EID8 = 0x06,
  MCP2515_CTRL_REG_RXF1EID0 = 0x07,
  MCP2515_CTRL_REG_RXF2SIDH = 0x08,
  MCP2515_CTRL_REG_RXF2SIDL = 0x09,
  MCP2515_CTRL_REG_RXF2EID8 = 0x0A,
  MCP2515_CTRL_REG_RXF2EID0 = 0x0B,
  MCP2515_CTRL_REG_CANSTAT  = 0x0E, //! Can status register.
  MCP2515_CTRL_REG_CANCTRL  = 0x0F, //! Can control register.
  MCP2515_CTRL_REG_RXF3SIDH = 0x10,
  MCP2515_CTRL_REG_RXF3SIDL = 0x11,
  MCP2515_CTRL_REG_RXF3EID8 = 0x12,
  MCP2515_CTRL_REG_RXF3EID0 = 0x13,
  MCP2515_CTRL_REG_RXF4SIDH = 0x14,
  MCP2515_CTRL_REG_RXF4SIDL = 0x15,
  MCP2515_CTRL_REG_RXF4EID8 = 0x16,
  MCP2515_CTRL_REG_RXF4EID0 = 0x17,
  MCP2515_CTRL_REG_RXF5SIDH = 0x18,
  MCP2515_CTRL_REG_RXF5SIDL = 0x19,
  MCP2515_CTRL_REG_RXF5EID8 = 0x1A,
  MCP2515_CTRL_REG_RXF5EID0 = 0x1B,
  MCP2515_CTRL_REG_TEC      = 0x1C,
  MCP2515_CTRL_REG_REC      = 0x1D,
  MCP2515_CTRL_REG_RXM0SIDH = 0x20,
  MCP2515_CTRL_REG_RXM0SIDL = 0x21,
  MCP2515_CTRL_REG_RXM0EID8 = 0x22,
  MCP2515_CTRL_REG_RXM0EID0 = 0x23,
  MCP2515_CTRL_REG_RXM1SIDH = 0x24,
  MCP2515_CTRL_REG_RXM1SIDL = 0x25,
  MCP2515_CTRL_REG_RXM1EID8 = 0x26,
  MCP2515_CTRL_REG_RXM1EID0 = 0x27,
  MCP2515_CTRL_REG_CNF3     = 0x28,
  MCP2515_CTRL_REG_CNF2     = 0x29,
  MCP2515_CTRL_REG_CNF1     = 0x2A,
  MCP2515_CTRL_REG_CANINTE  = 0x2B,
  MCP2515_CTRL_REG_CANINTF  = 0x2C, //! Can interrupt flag register. 
  MCP2515_CTRL_REG_EFLG     = 0x2D,
  MCP2515_CTRL_REG_TXB0CTRL = 0x30,
  MCP2515_CTRL_REG_TXB0SIDH = 0x31,
  MCP2515_CTRL_REG_TXB0SIDL = 0x32,
  MCP2515_CTRL_REG_TXB0EID8 = 0x33,
  MCP2515_CTRL_REG_TXB0EID0 = 0x34,
  MCP2515_CTRL_REG_TXB0DLC  = 0x35,
  MCP2515_CTRL_REG_TXB0DATA = 0x36,
  MCP2515_CTRL_REG_TXB1CTRL = 0x40,
  MCP2515_CTRL_REG_TXB1SIDH = 0x41,
  MCP2515_CTRL_REG_TXB1SIDL = 0x42,
  MCP2515_CTRL_REG_TXB1EID8 = 0x43,
  MCP2515_CTRL_REG_TXB1EID0 = 0x44,
  MCP2515_CTRL_REG_TXB1DLC  = 0x45,
  MCP2515_CTRL_REG_TXB1DATA = 0x46,
  MCP2515_CTRL_REG_TXB2CTRL = 0x50,
  MCP2515_CTRL_REG_TXB2SIDH = 0x51,
  MCP2515_CTRL_REG_TXB2SIDL = 0x52,
  MCP2515_CTRL_REG_TXB2EID8 = 0x53,
  MCP2515_CTRL_REG_TXB2EID0 = 0x54,
  MCP2515_CTRL_REG_TXB2DLC  = 0x55,
  MCP2515_CTRL_REG_TXB2DATA = 0x56,
  MCP2515_CTRL_REG_RXB0CTRL = 0x60,
  MCP2515_CTRL_REG_RXB0SIDH = 0x61,
  MCP2515_CTRL_REG_RXB0SIDL = 0x62,
  MCP2515_CTRL_REG_RXB0EID8 = 0x63,
  MCP2515_CTRL_REG_RXB0EID0 = 0x64,
  MCP2515_CTRL_REG_RXB0DLC  = 0x65,
  MCP2515_CTRL_REG_RXB0DATA = 0x66,
  MCP2515_CTRL_REG_RXB1CTRL = 0x70,
  MCP2515_CTRL_REG_RXB1SIDH = 0x71,
  MCP2515_CTRL_REG_RXB1SIDL = 0x72,
  MCP2515_CTRL_REG_RXB1EID8 = 0x73,
  MCP2515_CTRL_REG_RXB1EID0 = 0x74,
  MCP2515_CTRL_REG_RXB1DLC  = 0x75,
  MCP2515_CTRL_REG_RXB1DATA = 0x76
}MCP2515_CONTROL_REGISTERS, *PMCP2515_CONTROL_REGISTERS;

/**************************************************************************
 *        CANINTE: CAN INTERRUPT ENABLE REGISTER (ADDRESS: 2Bh)           *
 **************************************************************************
 */
typedef enum _MCP2515_CANINTE_TYPE
{
  MCP2515_CANINTE_RX0IF_BIT = 0x01,
  MCP2515_CANINTE_RX1IF_BIT = 0x01,
  MCP2515_CANINTE_TX0IF_BIT = 0x01,
  MCP2515_CANINTE_TX1IF_BIT = 0x01,
  MCP2515_CANINTE_TX2IF_BIT = 0x01,
  MCP2515_CANINTE_ERRIF_BIT = 0x01,
  MCP2515_CANINTE_WAKIF_BIT = 0x01,
  MCP2515_CANINTE_MERRF_BIT = 0x01
}MCP2515_CANINTE_TYPE, *PMCP2515_CANINTE_TYPE;

/**************************************************************************/

#define MCP2515_REG_CANINTE_RX0IF_MASK  (0x01)
#define MCP2515_REG_CANINTE_RX1IF_MASK  (0x02)
#define MCP2515_REG_CANINTE_TX0IF_MASK  (0x04)
#define MCP2515_REG_CANINTE_TX1IF_MASK  (0x08)
#define MCP2515_REG_CANINTE_TX2IF_MASK  (0x10)
#define MCP2515_REG_CANINTE_ERRIF_MASK  (0x20)
#define MCP2515_REG_CANINTE_WAKIF_MASK  (0x40)
#define MCP2515_REG_CANINTE_MERRF_MASK  (0x80)

/**************************************************************************/

typedef union _MCP2515_CTRL_REG_CANINTE_TYPE
{
  struct _convert
  {
    uint8_t rx0if :1; //! Receive Buffer 0 Full Interrupt Flag bit.
    uint8_t rx1if :1; //! Receive Buffer 1 Full Interrupt Flag bit.
    uint8_t tx0if :1; //! Transmit Buffer 0 Empty Interrupt Flag bit.
    uint8_t tx1if :1; //! Transmit Buffer 1 Empty Interrupt Flag bit.
    uint8_t tx2if :1; //! Transmit Buffer 2 Empty Interrupt Flag bit.
    uint8_t errif :1; //! Error Interrupt Flag bit (multiple sources in EFLG register).
    uint8_t wakif :1; //! Wake-up Interrupt Flag bit.
    uint8_t merrf :1; //! Message Error Interrupt Flag bit.
  } cnvrt;            //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_CANINTE_TYPE, *PMCP2515_CTRL_REG_CANINTE_TYPE;

/**************************************************************************
 *          CANINTF: CAN INTERRUPT FLAG REGISTER (ADDRESS: 2Ch)           *
 **************************************************************************
 */
typedef enum _MCP2515_CANINTF_TYPE
{
  MCP2515_CANINTF_RX0IF_BIT = 0x01,
  MCP2515_CANINTF_RX1IF_BIT = 0x01,
  MCP2515_CANINTF_TX0IF_BIT = 0x01,
  MCP2515_CANINTF_TX1IF_BIT = 0x01,
  MCP2515_CANINTF_TX2IF_BIT = 0x01,
  MCP2515_CANINTF_ERRIF_BIT = 0x01,
  MCP2515_CANINTF_WAKIF_BIT = 0x01,
  MCP2515_CANINTF_MERRF_BIT = 0x01
}MCP2515_CANINTF_TYPE, *PMCP2515_CANINTF_TYPE;

/**************************************************************************/

#define MCP2515_REG_CANINTF_RX0IF_MASK  (0x01)
#define MCP2515_REG_CANINTF_RX1IF_MASK  (0x02)
#define MCP2515_REG_CANINTF_TX0IF_MASK  (0x04)
#define MCP2515_REG_CANINTF_TX1IF_MASK  (0x08)
#define MCP2515_REG_CANINTF_TX2IF_MASK  (0x10)
#define MCP2515_REG_CANINTF_ERRIF_MASK  (0x20)
#define MCP2515_REG_CANINTF_WAKIF_MASK  (0x40)
#define MCP2515_REG_CANINTF_MERRF_MASK  (0x80)

/**************************************************************************/

typedef union _MCP2515_CTRL_REG_CANINTF_TYPE
{
  struct _convert
  {
    uint8_t rx0if :1; //! Receive Buffer 0 Full Interrupt Flag bit.
    uint8_t rx1if :1; //! Receive Buffer 1 Full Interrupt Flag bit.
    uint8_t tx0if :1; //! Transmit Buffer 0 Empty Interrupt Flag bit.
    uint8_t tx1if :1; //! Transmit Buffer 1 Empty Interrupt Flag bit.
    uint8_t tx2if :1; //! Transmit Buffer 2 Empty Interrupt Flag bit.
    uint8_t errif :1; //! Error Interrupt Flag bit (multiple sources in EFLG register).
    uint8_t wakif :1; //! Wake-up Interrupt Flag bit.
    uint8_t merrf :1; //! Message Error Interrupt Flag bit.
  } cnvrt;            //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_CANINTF_TYPE, *PMCP2515_CTRL_REG_CANINTF_TYPE;

/**************************************************************************
 *        RXB0CTRL: RECEIVE BUFFER 0 CONTROL REGISTER (ADDRESS: 60h)      *
 **************************************************************************
 */
typedef enum _MCP2515_RXB0CTRL_TYPE
{
  MCP2515_RXB0CTRL_BUKT_BIT       = 0x01,
  MCP2515_RXB0CTRL_RXM_STDEXT_BIT = 0x00,
  MCP2515_RXB0CTRL_RXM_ANY_BIT    = 0x03  //! Receive any messages.
}MCP2515_RXB0CTRL_TYPE, *PMCP2515_RXB0CTRL_TYPE;

/**************************************************************************/

typedef union _MCP2515_CTRL_REG_RXB0CTRL_TYPE
{
  struct _convert
  {
    uint8_t filhit0   :1; //! Filter Hit bit (indicates which acceptance filter enabled reception of message).
    uint8_t bukt1     :1; //! Read-Only Copy of BUKT bit (used internally by the MCP2515).
#define MCP2515_CTRL_REG_RXB0CTRL_BUKT_MASK  (0x04)
    uint8_t bukt      :1; //! BUKT: Rollover Enable bit.
    uint8_t rxrtr     :1; //! Received Remote Transfer Request bit.
    uint8_t reserved1 :1; //! Unimplemented: Read as ‘0’.
#define MCP2515_CTRL_REG_RXB0CTRL_RXM_MASK   (0x60)
    uint8_t rxm       :2; //! Receive Buffer Operating mode bits.
    uint8_t reserved2 :1; //! Unimplemented: Read as ‘0’.
  } cnvrt;                //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_RXB0CTRL_TYPE, *PMCP2515_CTRL_REG_RXB0CTRL_TYPE;

/**************************************************************************
 *        RXB1CTRL: RECEIVE BUFFER 1 CONTROL REGISTER (ADDRESS: 60h)      *
 **************************************************************************
 */
typedef enum _MCP2515_RXB1CTRL_TYPE
{
  MCP2515_RXB1CTRL_RXM_STDEXT_BIT = 0x00, //! Turns mask/filters off; receives any message.
  MCP2515_RXB1CTRL_RXM_ANY_BIT    = 0x03  //! Receive any messages.
}MCP2515_RXB1CTRL_TYPE, *PMCP2515_RXB1CTRL_TYPE;

/**************************************************************************/

typedef union _MCP2515_CTRL_REG_RXB1CTRL_TYPE
{
  struct _convert
  {
    uint8_t filhit    :3; //! Filter Hit bits (indicates which acceptance filter enabled reception of message).
    uint8_t rxrtr     :1; //! Received Remote Transfer Request bit.
    uint8_t reserved1 :1; //! Unimplemented: Read as ‘0’.
#define MCP2515_CTRL_REG_RXB1CTRL_RXM_MASK   (0x60)
    uint8_t rxm       :2; //! Receive Buffer Operating mode bits.
    uint8_t reserved2 :1; //! Unimplemented: Read as ‘0’.
  } cnvrt;                //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_RXB1CTRL_TYPE, *PMCP2515_CTRL_REG_RXB1CTRL_TYPE;

/**************************************************************************
 * RXMnSIDH: MASK n STANDARD IDENTIFIER REGISTER HIGH (ADDRESS: 20h, 24h) *
 **************************************************************************
 */
typedef enum _MCP2515_RXMnREGISTERS
{
  MCP2515_RXM0REGISTERS = 0x00,
  MCP2515_RXM1REGISTERS = 0x01
}MCP2515_RXMnREGISTERS, *PMCP2515_RXMnREGISTERS;

/**************************************************************************/

typedef union _MCP2515_CTRL_REG_RXMnSIDH_TYPE
{
  struct _convert
  {
    uint8_t sid; //! SID[10:3]: Standard Identifier bits.
  } cnvrt;       //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_RXMnSIDH_TYPE, *PMCP2515_CTRL_REG_RXMnSIDH_TYPE;

/**************************************************************************
 * RXMnSIDL: MASK n STANDARD IDENTIFIER REGISTER LOW (ADDRESS: 21h, 25h)  *
 **************************************************************************
 */
typedef union _MCP2515_CTRL_REG_RXMnSIDL_TYPE
{
  struct _convert
  {
    uint8_t eid       :2; //! EID[17:16]: Extended Identifier bits.
    uint8_t reserved  :3; //! Unimplemented: Read as ‘0’.
    uint8_t sid       :3; //! SID[2:0]: Standard Identifier bits.
  } cnvrt;       //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_RXMnSIDL_TYPE, *PMCP2515_CTRL_REG_RXMnSIDL_TYPE;

/**************************************************************************
 *          RXFnSIDH: FILTER n STANDARD IDENTIFIER REGISTER HIGH          *
 *          (ADDRESS: 00h, 04h, 08h, 10h, 14h, 18h)                       *
 **************************************************************************
 */
typedef enum _MCP2515_RXFnREGISTERS
{
  MCP2515_RXF0REGISTERS = 0x00,
  MCP2515_RXF1REGISTERS = 0x01,
  MCP2515_RXF2REGISTERS = 0x02,
  MCP2515_RXF3REGISTERS = 0x03,
  MCP2515_RXF4REGISTERS = 0x04,
  MCP2515_RXF5REGISTERS = 0x05
}MCP2515_RXFnREGISTERS, *PMCP2515_RXFnREGISTERS;

/**************************************************************************/

typedef union _MCP2515_CTRL_REG_RXFnSIDH_TYPE
{
  struct _convert
  {
    uint8_t sid; //! SID[10:3]: Standard Identifier bits.
  } cnvrt;       //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_RXFnSIDH_TYPE, *PMCP2515_CTRL_REG_RXFnSIDH_TYPE;

/**************************************************************************
 *          RXFnSIDL: FILTER n STANDARD IDENTIFIER REGISTER LOW           *
 *          (ADDRESS: 01h, 05h, 09h, 11h, 15h, 19h)                       *
 **************************************************************************
 */
typedef union _MCP2515_CTRL_REG_RXFnSIDL_TYPE
{
  struct _convert
  {
    uint8_t eid       :2; //! EID[17:16]: Extended Identifier bits.
    uint8_t reserved2 :1; //! Unimplemented: Read as ‘0’.
    uint8_t exide     :1; //! Extended Identifier Enable bit.
    uint8_t reserved1 :1; //! Unimplemented: Read as ‘0’.
    uint8_t sid       :3; //! SID[2:0]: Standard Identifier bits.
  } cnvrt;       //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_RXFnSIDL_TYPE, *PMCP2515_CTRL_REG_RXFnSIDL_TYPE;

/**************************************************************************
 *                         CAN CONTROL REGISTER                           *
 **************************************************************************
 */
typedef enum _MCP2515_CANCTRL_REQOP_TYPE  //! REQOP - Request Operation Mode bits (modes of operation).
{
  MCP2515_CANCTRL_REQOP_NORMAL_MODE      = 0x00,
  MCP2515_CANCTRL_REQOP_SLEEP_MODE       = 0x01,
  MCP2515_CANCTRL_REQOP_LOOPBACK_MODE    = 0x02,
  MCP2515_CANCTRL_REQOP_LEISTENONLY_MODE = 0x03,
  MCP2515_CANCTRL_REQOP_CONFIG_MODE      = 0x04
}MCP2515_CANCTRL_REQOP_TYPE, *PMCP2515_CANCTRL_REQOP_TYPE;

/**************************************************************************/

typedef union _MCP2515_CTRL_REG_CANCTRL_TYPE
{
  struct _convert
  {
    uint8_t clkpre :2; //! CLKOUT Pin Prescaler bits.
    uint8_t clken  :1; //! CLKOUT Pin Enable bit.
    uint8_t osm    :1; //! One-Shot Mode bit.
    uint8_t abat   :1; //! Abort All Pending Transmissions bit.
#define MCP2515_REG_CANCTRL_REQOP_MASK  (0xE0) // Mask in register.
    uint8_t reqop  :3; //! Request Operation Mode bits.
  } cnvrt;             //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_CANCTRL_TYPE, *PMCP2515_CTRL_REG_CANCTRL_TYPE;

/**************************************************************************
 *                          CAN STATUS REGISTER                           *
 **************************************************************************
 */
typedef union _MCP2515_CTRL_REG_CANSTAT_TYPE
{
  struct _convert
  {
    uint8_t reserved1 :1; //! Unimplemented: Read as ‘0’.
#define MCP2515_REG_CANSTAT_ICOD_MASK   (0x0E) //! Mask in register.
    uint8_t icod      :3; //! Interrupt Flag Code bits.
    uint8_t reserved2 :1; //! Unimplemented: Read as ‘0’.
#define MCP2515_REG_CANSTAT_OPMOD_MASK  (0xE0) //! Mask in register.
    uint8_t opmod     :3; //! Operation Mode bits.
  } cnvrt;                //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_CANSTAT_TYPE, *PMCP2515_CTRL_REG_CANSTAT_TYPE;

/**************************************************************************
 *                        TXBn REGISTER ENUMs                             *
 **************************************************************************
 */
typedef enum _MCP2515_TXBnREGISTERS
{
  MCP2515_TXBnREG_CTRL = 0x00,
  MCP2515_TXBnREG_SIDH = 0x01,
  MCP2515_TXBnREG_SIDL = 0x02,
  MCP2515_TXBnREG_EID8 = 0x03,
  MCP2515_TXBnREG_EID0 = 0x04,
  MCP2515_TXBnREG_DLC  = 0x05,
  MCP2515_TXBnREG_DATA = 0x06,
  MCP2515_TXBnREG_END  = 0x07
}MCP2515_TXBnREGISTERS, *PMCP2515_TXBnREGISTERS;

/**************************************************************************
 *                        RXBn REGISTER ENUMs                             *
 **************************************************************************
 */
typedef enum _MCP2515_RXBnREGISTERS
{
  MCP2515_RXBnREG_CTRL = 0x00,
  MCP2515_RXBnREG_SIDH = 0x01,
  MCP2515_RXBnREG_SIDL = 0x02,
  MCP2515_RXBnREG_EID8 = 0x03,
  MCP2515_RXBnREG_EID0 = 0x04,
  MCP2515_RXBnREG_DLC  = 0x05,
  MCP2515_RXBnREG_DATA = 0x06,
  MCP2515_RXBnREG_END  = 0x07
}MCP2515_RXBnREGISTERS, *PMCP2515_RXBnREGISTERS;

/**************************************************************************
 *                  TRANSMIT BUFFER n CONTROL REGISTER                    *
 **************************************************************************
 */
typedef enum _MCP2515_TXBnCNRL_TXREG_TYPE
{
  MCP2515_TXBnCNRL_TXREG_BIT = 0x01
}MCP2515_TXBnCNRL_TXREG_TYPE, *P_MCP2515_TXBnCNRL_TXREG_TYPE;

/**************************************************************************/

typedef union _MCP2515_CTRL_REG_TXBnCNRL_TYPE
{
  struct _convert
  {
    uint8_t txp       :2; //! Transmit Buffer Priority bits.
    uint8_t reserved1 :1; //! Unimplemented: Read as ‘0’.
#define MMCP2515_REG_TXBnCNRL_TXREQ_MASK  (0x08) //! Mask in register.
    uint8_t txreg     :1; //! Message Transmit Request bit.
    uint8_t txerr     :1; //! Transmission Error Detected bit.
    uint8_t mloa      :1; //! Message Lost Arbitration bit.
    uint8_t abtf      :1; //! Message Aborted Flag bit.
    uint8_t reserved2 :1; //! Unimplemented: Read as ‘0’.
  } cnvrt;                //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_TXBnCNRL_TYPE, *PMCP2515_CTRL_REG_TXBnCNRL_TYPE;

/**************************************************************************
 *     TXBnSIDH: TRANSMIT BUFFER n STANDARD IDENTIFIER REGISTER HIGH      *
 **************************************************************************
 */
typedef union _MCP2515_CTRL_REG_TXBnSIDH_TYPE
{
  struct _convert
  {
    uint8_t sid; //! SID[10:3]: Standard Identifier bits.
  } cnvrt;       //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_TXBnSIDH_TYPE, *PMCP2515_CTRL_REG_TXBnSIDHL_TYPE;

/**************************************************************************
 *      TXBnSIDL: TRANSMIT BUFFER n STANDARD IDENTIFIER REGISTER LOW      *
 **************************************************************************
 */
typedef union _MCP2515_CTRL_REG_TXBnSIDL_TYPE
{
  struct _convert
  {
    uint8_t eid       :2; //! EID[17:16]: Extended Identifier bits.
    uint8_t reserved1 :1; //! Unimplemented: Read as ‘0’.
    uint8_t exide     :1; //! Extended Identifier Enable bit.
    uint8_t reserved2 :1; //! Unimplemented: Read as ‘0’.
    uint8_t sid       :3; //! SID[2:0]: Standard Identifier bits.
  } cnvrt;                //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_TXBnSIDL_TYPE, *PMCP2515_CTRL_REG_TXBnSIDL_TYPE;

/**************************************************************************
 *           TXBnDLC: TRANSMIT BUFFER n DATA LENGTH CODE REGISTER         *
 **************************************************************************
 */
typedef union _MCP2515_CTRL_REG_TXBnDLC_TYPE
{
  struct _convert
  {
    uint8_t dlc       :4; //! Data Length Code bits.
    uint8_t reserved1 :2; //! Unimplemented: Read as ‘0’.
    uint8_t rtr       :1; //! Remote Transmission Request bit.
    uint8_t reserved2 :1; //! Unimplemented: Read as ‘0’.
  } cnvrt;                //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_TXBnDLC_TYPE, *PMCP2515_CTRL_REG_TXBnDLC_TYPE;

/**************************************************************************
 *     RXBnSIDH: RECEIVE BUFFER n STANDARD IDENTIFIER REGISTER HIGH       *
 **************************************************************************
 */
typedef union _MCP2515_CTRL_REG_RXBnSIDH_TYPE
{
  struct _convert
  {
    uint8_t sid; //! SID[10:3]: Standard Identifier bits.
  } cnvrt;       //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_RXBnSIDH_TYPE, *PMCP2515_CTRL_REG_RXBnSIDHL_TYPE;

/**************************************************************************
 *     RXBnSIDL: RECEIVE BUFFER n STANDARD IDENTIFIER REGISTER LOW        *
 **************************************************************************
 */
typedef union _MCP2515_CTRL_REG_RXBnSIDL_TYPE
{
  struct _convert
  {
    uint8_t eid       :2; //! EID[17:16]: Extended Identifier bits.
    uint8_t reserved1 :1; //! Unimplemented: Read as ‘0’.
    uint8_t ide       :1; //! Extended Identifier Enable bit.
    uint8_t srr       :1; //! Standard Frame Remote Transmit Request bit (valid only if IDE bit = 0).
    uint8_t sid       :3; //! SID[2:0]: Standard Identifier bits.
  } cnvrt;                //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_RXBnSIDL_TYPE, *PMCP2515_CTRL_REG_RXBnSIDL_TYPE;

/**************************************************************************
 *          RXBnDLC: RECEIVE BUFFER n DATA LENGTH CODE REGISTER           *
 **************************************************************************
 */
typedef union _MCP2515_CTRL_REG_RXBnDLC_TYPE
{
  struct _convert
  {
    uint8_t dlc       :4; //! Data Length Code bits.
    uint8_t reserved1 :2; //! Unimplemented: Read as ‘0’.
    uint8_t rtr       :1; //! Extended Frame Remote Transmission Request bit (valid only when IDE (RXBnSIDL[3]) = 1).
    uint8_t reserved2 :1; //! Unimplemented: Read as ‘0’.
  } cnvrt;                //! Convert.
  uint8_t reg;
}MCP2515_CTRL_REG_RXBnDLC_TYPE, *PMCP2515_CTRL_REG_RXBnDLC_TYPE;

/**************************************************************************
 *                             Clock 8M                                   *
 **************************************************************************
 */
#define MCP2515_8MHz_1000kBPS_CFG1 (0x00)
#define MCP2515_8MHz_1000kBPS_CFG2 (0x80)
#define MCP2515_8MHz_1000kBPS_CFG3 (0x80)

#define MCP2515_8MHz_500kBPS_CFG1 (0x00)
#define MCP2515_8MHz_500kBPS_CFG2 (0x90)
#define MCP2515_8MHz_500kBPS_CFG3 (0x82)

#define MCP2515_8MHz_250kBPS_CFG1 (0x00)
#define MCP2515_8MHz_250kBPS_CFG2 (0xB1)
#define MCP2515_8MHz_250kBPS_CFG3 (0x85)

#define MCP2515_8MHz_200kBPS_CFG1 (0x00)
#define MCP2515_8MHz_200kBPS_CFG2 (0xB4)
#define MCP2515_8MHz_200kBPS_CFG3 (0x86)

#define MCP2515_8MHz_125kBPS_CFG1 (0x01)
#define MCP2515_8MHz_125kBPS_CFG2 (0xB1)
#define MCP2515_8MHz_125kBPS_CFG3 (0x85)

#define MCP2515_8MHz_100kBPS_CFG1 (0x01)
#define MCP2515_8MHz_100kBPS_CFG2 (0xB4)
#define MCP2515_8MHz_100kBPS_CFG3 (0x86)

#define MCP2515_8MHz_80kBPS_CFG1 (0x01)
#define MCP2515_8MHz_80kBPS_CFG2 (0xBF)
#define MCP2515_8MHz_80kBPS_CFG3 (0x87)

#define MCP2515_8MHz_50kBPS_CFG1 (0x03)
#define MCP2515_8MHz_50kBPS_CFG2 (0xB4)
#define MCP2515_8MHz_50kBPS_CFG3 (0x86)

#define MCP2515_8MHz_40kBPS_CFG1 (0x03)
#define MCP2515_8MHz_40kBPS_CFG2 (0xBF)
#define MCP2515_8MHz_40kBPS_CFG3 (0x87)

#define MCP2515_8MHz_33k3BPS_CFG1 (0x47)
#define MCP2515_8MHz_33k3BPS_CFG2 (0xE2)
#define MCP2515_8MHz_33k3BPS_CFG3 (0x85)

#define MCP2515_8MHz_31k25BPS_CFG1 (0x07)
#define MCP2515_8MHz_31k25BPS_CFG2 (0xA4)
#define MCP2515_8MHz_31k25BPS_CFG3 (0x84)

#define MCP2515_8MHz_20kBPS_CFG1 (0x07)
#define MCP2515_8MHz_20kBPS_CFG2 (0xBF)
#define MCP2515_8MHz_20kBPS_CFG3 (0x87)

#define MCP2515_8MHz_10kBPS_CFG1 (0x0F)
#define MCP2515_8MHz_10kBPS_CFG2 (0xBF)
#define MCP2515_8MHz_10kBPS_CFG3 (0x87)

#define MCP2515_8MHz_5kBPS_CFG1 (0x1F)
#define MCP2515_8MHz_5kBPS_CFG2 (0xBF)
#define MCP2515_8MHz_5kBPS_CFG3 (0x87)

/**************************************************************************
 *                            Clock 16M                                   *
 **************************************************************************
 */
#define MCP2515_16MHz_1000kBPS_CFG1 (0x00)
#define MCP2515_16MHz_1000kBPS_CFG2 (0xD0)
#define MCP2515_16MHz_1000kBPS_CFG3 (0x82)

#define MCP2515_16MHz_500kBPS_CFG1 (0x00)
#define MCP2515_16MHz_500kBPS_CFG2 (0xF0)
#define MCP2515_16MHz_500kBPS_CFG3 (0x86)

#define MCP2515_16MHz_250kBPS_CFG1 (0x41)
#define MCP2515_16MHz_250kBPS_CFG2 (0xF1)
#define MCP2515_16MHz_250kBPS_CFG3 (0x85)

#define MCP2515_16MHz_200kBPS_CFG1 (0x01)
#define MCP2515_16MHz_200kBPS_CFG2 (0xFA)
#define MCP2515_16MHz_200kBPS_CFG3 (0x87)

#define MCP2515_16MHz_125kBPS_CFG1 (0x03)
#define MCP2515_16MHz_125kBPS_CFG2 (0xF0)
#define MCP2515_16MHz_125kBPS_CFG3 (0x86)

#define MCP2515_16MHz_100kBPS_CFG1 (0x03)
#define MCP2515_16MHz_100kBPS_CFG2 (0xFA)
#define MCP2515_16MHz_100kBPS_CFG3 (0x87)

#define MCP2515_16MHz_80kBPS_CFG1 (0x03)
#define MCP2515_16MHz_80kBPS_CFG2 (0xFF)
#define MCP2515_16MHz_80kBPS_CFG3 (0x87)

#define MCP2515_16MHz_83k3BPS_CFG1 (0x03)
#define MCP2515_16MHz_83k3BPS_CFG2 (0xBE)
#define MCP2515_16MHz_83k3BPS_CFG3 (0x07)

#define MCP2515_16MHz_50kBPS_CFG1 (0x07)
#define MCP2515_16MHz_50kBPS_CFG2 (0xFA)
#define MCP2515_16MHz_50kBPS_CFG3 (0x87)

#define MCP2515_16MHz_40kBPS_CFG1 (0x07)
#define MCP2515_16MHz_40kBPS_CFG2 (0xFF)
#define MCP2515_16MHz_40kBPS_CFG3 (0x87)

#define MCP2515_16MHz_33k3BPS_CFG1 (0x4E)
#define MCP2515_16MHz_33k3BPS_CFG2 (0xF1)
#define MCP2515_16MHz_33k3BPS_CFG3 (0x85)

#define MCP2515_16MHz_20kBPS_CFG1 (0x0F)
#define MCP2515_16MHz_20kBPS_CFG2 (0xFF)
#define MCP2515_16MHz_20kBPS_CFG3 (0x87)

#define MCP2515_16MHz_10kBPS_CFG1 (0x1F)
#define MCP2515_16MHz_10kBPS_CFG2 (0xFF)
#define MCP2515_16MHz_10kBPS_CFG3 (0x87)

#define MCP2515_16MHz_5kBPS_CFG1 (0x3F)
#define MCP2515_16MHz_5kBPS_CFG2 (0xFF)
#define MCP2515_16MHz_5kBPS_CFG3 (0x87)

/**************************************************************************
 *                            Clock 20M                                   *
 **************************************************************************
 */
#define MCP2515_20MHz_1000kBPS_CFG1 (0x00)
#define MCP2515_20MHz_1000kBPS_CFG2 (0xD9)
#define MCP2515_20MHz_1000kBPS_CFG3 (0x82)

#define MCP2515_20MHz_500kBPS_CFG1 (0x00)
#define MCP2515_20MHz_500kBPS_CFG2 (0xFA)
#define MCP2515_20MHz_500kBPS_CFG3 (0x87)

#define MCP2515_20MHz_250kBPS_CFG1 (0x41)
#define MCP2515_20MHz_250kBPS_CFG2 (0xFB)
#define MCP2515_20MHz_250kBPS_CFG3 (0x86)

#define MCP2515_20MHz_200kBPS_CFG1 (0x01)
#define MCP2515_20MHz_200kBPS_CFG2 (0xFF)
#define MCP2515_20MHz_200kBPS_CFG3 (0x87)

#define MCP2515_20MHz_125kBPS_CFG1 (0x03)
#define MCP2515_20MHz_125kBPS_CFG2 (0xFA)
#define MCP2515_20MHz_125kBPS_CFG3 (0x87)

#define MCP2515_20MHz_100kBPS_CFG1 (0x04)
#define MCP2515_20MHz_100kBPS_CFG2 (0xFA)
#define MCP2515_20MHz_100kBPS_CFG3 (0x87)

#define MCP2515_20MHz_80kBPS_CFG1 (0x04)
#define MCP2515_20MHz_80kBPS_CFG2 (0xFF)
#define MCP2515_20MHz_80kBPS_CFG3 (0x87)

#define MCP2515_20MHz_50kBPS_CFG1 (0x09)
#define MCP2515_20MHz_50kBPS_CFG2 (0xFA)
#define MCP2515_20MHz_50kBPS_CFG3 (0x87)

#define MCP2515_20MHz_40kBPS_CFG1 (0x09)
#define MCP2515_20MHz_40kBPS_CFG2 (0xFF)
#define MCP2515_20MHz_40kBPS_CFG3 (0x87)

/**************************************************************************/

typedef enum _MCP2515_CLOCK
{
  MCP2515_CLOCK_20MHZ = 0x00,
  MCP2515_CLOCK_16MHZ = 0x01,
  MCP2515_CLOCK_8MHZ  = 0x02
}MCP2515_CLOCK, *PMCP2515_CLOCK;

/**************************************************************************
 *                      The mcp2515 ctx structure                         *
 **************************************************************************
 */
typedef struct _MCP2515_CTX
{
  PMCP2515_HAL_REG          pHal;
  MCP2515_CONTROL_REGISTERS (*pTXBnArray)[MCP2515_TXBnREG_END];
  MCP2515_CONTROL_REGISTERS (*pRXBnArray)[MCP2515_RXBnREG_END];
}MCP2515_CTX, *PMCP2515_CTX;

/**************************************************************************
 *                         The can bus definitions                        *
 **************************************************************************
 */
typedef enum _CAN_BUS_SPEED
{
  CAN_SPEED_5KBPS    = 0x00,
  CAN_SPEED_10KBPS   = 0x01,
  CAN_SPEED_20KBPS   = 0x02,
  CAN_SPEED_31K25BPS = 0x03,
  CAN_SPEED_33KBPS   = 0x04,
  CAN_SPEED_40KBPS   = 0x05,
  CAN_SPEED_50KBPS   = 0x06,
  CAN_SPEED_80KBPS   = 0x07,
  CAN_SPEED_83K3BPS  = 0x08,
  CAN_SPEED_95KBPS   = 0x09,
  CAN_SPEED_100KBPS  = 0x0A,
  CAN_SPEED_125KBPS  = 0x0B,
  CAN_SPEED_200KBPS  = 0x0C,
  CAN_SPEED_250KBPS  = 0x0D,
  CAN_SPEED_500KBPS  = 0x0E,
  CAN_SPEED_1000KBPS = 0x0F
}CAN_BUS_SPEED, *PCAN_BUS_SPEED;

/**************************************************************************/
 
#define CAN_FRAME_MAX_LEN       (8)     //! CAN payload length.
#define CAN_FRAME_BROADCAST_ID  (0x7FF) //! Broadcast id of sending frame. Should be allowed by MCP2515 filters.
//#define CAN_FRAME_SOURCE_ID           //! TODO can frame source id should be changed to auto addresing procedure.

/**************************************************************************/

typedef struct _CAN_FRAME
{
  uint16_t id;      //! Destination id.
  uint8_t  length;  //! Frame payload length in byte (0 .. CAN_FRAME_MAX_LEN).
  uint8_t  data[CAN_FRAME_MAX_LEN];
}CAN_FRAME, *PCAN_FRAME;

/**************************************************************************
 * Global function declarations.                                          *
 **************************************************************************
 */
MCP2515_ERROR MCP2515_Init(PMCP2515_HAL_REG pInit);
MCP2515_ERROR MCP2515_SendMessage(const PCAN_FRAME pFrame);
MCP2515_ERROR MCP2515_ReadMessage(PCAN_FRAME pFrame);

void MCP2515_SetMask(MCP2515_RXMnREGISTERS rxm, uint16_t mask);
void MCP2515_SetFilter(MCP2515_RXFnREGISTERS rxf, uint16_t filter);

#ifdef _cplusplus
}
#endif

#endif // _MCP2515_H_