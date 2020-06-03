/**************************************************************************
 * @file       mcp2515.cpp                                                *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      This MCP2515 driver implements Controller Area Network(CAN)*
 *             controller.                                                *
 *************************************************************************/

#include "mcp2515.h"
#include "timer.h"
#include "Arduino-hal/time_hal.h"

/**************************************************************************
 * Static function declarations.                                          *
 **************************************************************************
 */
static uint8_t       MCP2515_ReadRegister(const MCP2515_CONTROL_REGISTERS reg);
static void          MCP2515_ReadSetOfRegisters(const MCP2515_CONTROL_REGISTERS reg, uint8_t values[], const uint8_t size);
static MCP2515_ERROR MCP2515_SetRegister (const MCP2515_CONTROL_REGISTERS reg, const uint8_t value);
static MCP2515_ERROR MCP2515_SetSetOfRegisters(const MCP2515_CONTROL_REGISTERS reg, const uint8_t values[], const uint8_t n);

static MCP2515_ERROR MCP2515_GetFreeTxBuffer(uint8_t *pIndex);
static MCP2515_ERROR MCP2515_CheckRxBuffer(uint8_t *pIndex);

static MCP2515_ERROR MCP2515_Reset(void);
static MCP2515_ERROR MCP2515_SetMode(const MCP2515_CANCTRL_REQOP_TYPE mode);
static MCP2515_ERROR MCP2515_SetBitrate(const CAN_BUS_SPEED speed, const MCP2515_CLOCK clock);

/**************************************************************************
 * Static variables.                                                      *
 **************************************************************************
 */
static MCP2515_CTX mcp2515_ctx;

const static MCP2515_CONTROL_REGISTERS txbn_ctrl_regs[][MCP2515_TXBnREG_END] = 
{
  {MCP2515_CTRL_REG_TXB0CTRL, MCP2515_CTRL_REG_TXB0SIDH, MCP2515_CTRL_REG_TXB0SIDL, MCP2515_CTRL_REG_TXB0EID8, MCP2515_CTRL_REG_TXB0EID0, MCP2515_CTRL_REG_TXB0DLC, MCP2515_CTRL_REG_TXB0DATA}, 
  {MCP2515_CTRL_REG_TXB1CTRL, MCP2515_CTRL_REG_TXB1SIDH, MCP2515_CTRL_REG_TXB1SIDL, MCP2515_CTRL_REG_TXB1EID8, MCP2515_CTRL_REG_TXB1EID0, MCP2515_CTRL_REG_TXB1DLC, MCP2515_CTRL_REG_TXB1DATA}, 
  {MCP2515_CTRL_REG_TXB2CTRL, MCP2515_CTRL_REG_TXB2SIDH, MCP2515_CTRL_REG_TXB2SIDL, MCP2515_CTRL_REG_TXB2EID8, MCP2515_CTRL_REG_TXB2EID0, MCP2515_CTRL_REG_TXB2DLC, MCP2515_CTRL_REG_TXB2DATA}, 
};

const static MCP2515_CONTROL_REGISTERS rxbn_ctrl_regs[][MCP2515_RXBnREG_END] =
{
  {MCP2515_CTRL_REG_RXB0CTRL, MCP2515_CTRL_REG_RXB0SIDH, MCP2515_CTRL_REG_RXB0SIDL, MCP2515_CTRL_REG_RXB0EID8, MCP2515_CTRL_REG_RXB0EID0, MCP2515_CTRL_REG_RXB0DLC, MCP2515_CTRL_REG_RXB0DATA},
  {MCP2515_CTRL_REG_RXB1CTRL, MCP2515_CTRL_REG_RXB1SIDH, MCP2515_CTRL_REG_RXB1SIDL, MCP2515_CTRL_REG_RXB1EID8, MCP2515_CTRL_REG_RXB1EID0, MCP2515_CTRL_REG_RXB1DLC, MCP2515_CTRL_REG_RXB1DATA},
};

/**************************************************************************/

MCP2515_ERROR MCP2515_Init(PMCP2515_HAL_REG pInit)
{
  PMCP2515_CTX    pCtx   = (PMCP2515_CTX) &mcp2515_ctx;
  MCP2515_ERROR   retval;

  memset(pCtx, 0x00, sizeof(*pCtx));

  pCtx->pHal = pInit;
  pCtx->pHal->spiInit();

  pCtx->pTXBnArray = (MCP2515_CONTROL_REGISTERS (*)[MCP2515_TXBnREG_END])txbn_ctrl_regs;
  pCtx->pRXBnArray = (MCP2515_CONTROL_REGISTERS (*)[MCP2515_RXBnREG_END])rxbn_ctrl_regs;

  MCP2515_Reset();
  MCP2515_SetBitrate(CAN_SPEED_500KBPS, MCP2515_CLOCK_8MHZ);
  retval = MCP2515_SetMode(MCP2515_CANCTRL_REQOP_NORMAL_MODE);

 return retval;
}

/**************************************************************************/

static uint8_t MCP2515_ReadRegister(const MCP2515_CONTROL_REGISTERS reg)
{
  PMCP2515_CTX pCtx      = (PMCP2515_CTX) &mcp2515_ctx;
  PMCP2515_HAL_REG pHal  = (PMCP2515_HAL_REG) pCtx->pHal;
  uint8_t retval;

  pHal->spiBegin();  //! TODO check retval values.

  pHal->spiTransfer(MCP2515_INSTR_SET_READ);
  pHal->spiTransfer(reg);
  retval = pHal->spiTransfer(0x00);

  pHal->spiEnd();

  return retval;
}

/**************************************************************************/

static void MCP2515_ReadSetOfRegisters(const MCP2515_CONTROL_REGISTERS reg, uint8_t values[], const uint8_t size)
{
  PMCP2515_CTX pCtx      = (PMCP2515_CTX) &mcp2515_ctx;
  PMCP2515_HAL_REG pHal  = (PMCP2515_HAL_REG) pCtx->pHal;

  pHal->spiBegin();  //! TODO check retval values.

  pHal->spiTransfer(MCP2515_INSTR_SET_READ);
  pHal->spiTransfer(reg);

  for (uint8_t i = 0; i < size; i++)
  {
    values[i] = pHal->spiTransfer(0x00);
  }

  pHal->spiEnd();
}

/**************************************************************************/

static MCP2515_ERROR MCP2515_SetRegister(const MCP2515_CONTROL_REGISTERS reg, const uint8_t value)
{
  PMCP2515_CTX pCtx      = (PMCP2515_CTX) &mcp2515_ctx;
  PMCP2515_HAL_REG pHal  = (PMCP2515_HAL_REG) pCtx->pHal;
  MCP2515_ERROR retval   = MCP2515_SUCCESS;

  pHal->spiBegin();  //! TODO check retval values.
  pHal->spiTransfer(MCP2515_INSTR_SET_WRITE);
  pHal->spiTransfer(reg);
  pHal->spiTransfer(value);
  pHal->spiEnd();

  return retval;
}

/**************************************************************************/

static MCP2515_ERROR MCP2515_SetSetOfRegisters(const MCP2515_CONTROL_REGISTERS reg, const uint8_t values[], const uint8_t n)
{
  PMCP2515_CTX pCtx      = (PMCP2515_CTX) &mcp2515_ctx;
  PMCP2515_HAL_REG pHal  = (PMCP2515_HAL_REG) pCtx->pHal;
  MCP2515_ERROR retval   = MCP2515_SUCCESS;

  pHal->spiBegin();  //! TODO check retval values.
  pHal->spiTransfer(MCP2515_INSTR_SET_WRITE);
  pHal->spiTransfer(reg);

  for (uint8_t i = 0; i < n; i++)
  {
    pHal->spiTransfer(values[i]);
  }

  pHal->spiEnd();

  return retval;
}

/**************************************************************************/

static MCP2515_ERROR MCP2515_ModifyRegister(const MCP2515_CONTROL_REGISTERS reg,const uint8_t mask, const uint8_t data)
{
  PMCP2515_CTX pCtx      = (PMCP2515_CTX) &mcp2515_ctx;
  PMCP2515_HAL_REG pHal  = (PMCP2515_HAL_REG) pCtx->pHal;
  MCP2515_ERROR retval   = MCP2515_SUCCESS;

  pHal->spiBegin();  //! TODO check retval values.
  pHal->spiTransfer(MCP2515_INSTR_SET_BIT_MODIFY);

  pHal->spiTransfer(reg);
  pHal->spiTransfer(mask);
  pHal->spiTransfer(data);
  pHal->spiEnd();

  return retval;
}

/**************************************************************************/

static MCP2515_ERROR MCP2515_Reset(void)
{
  PMCP2515_CTX                   pCtx    = (PMCP2515_CTX) &mcp2515_ctx;
  PMCP2515_HAL_REG               pHal    = (PMCP2515_HAL_REG) pCtx->pHal;
  MCP2515_CTRL_REG_CANINTE_TYPE  cainte;
  MCP2515_CTRL_REG_RXB0CTRL_TYPE rxb0ctrl;
  MCP2515_CTRL_REG_RXB1CTRL_TYPE rxb1ctrl;
  MCP2515_ERROR                  retval  = MCP2515_SUCCESS;
  uint8_t                        data[14];

  pHal->spiBegin(); //! TODO check retval values.
  pHal->spiTransfer(MCP2515_INSTR_SET_RESET);
  pHal->spiEnd();

  memset(data, 0, sizeof(data));

  MCP2515_SetSetOfRegisters(MCP2515_CTRL_REG_TXB0CTRL, data, sizeof(data));
  MCP2515_SetSetOfRegisters(MCP2515_CTRL_REG_TXB1CTRL, data, sizeof(data));
  MCP2515_SetSetOfRegisters(MCP2515_CTRL_REG_TXB2CTRL, data, sizeof(data));

  MCP2515_SetRegister(MCP2515_CTRL_REG_RXB0CTRL, 0x00);
  MCP2515_SetRegister(MCP2515_CTRL_REG_RXB1CTRL, 0x00);

  memset(&cainte, 0, sizeof(cainte));
  cainte.cnvrt.rx0if = MCP2515_CANINTE_RX0IF_BIT;
  cainte.cnvrt.rx1if = MCP2515_CANINTE_RX1IF_BIT;
  cainte.cnvrt.errif = MCP2515_CANINTE_ERRIF_BIT;
  cainte.cnvrt.merrf = MCP2515_CANINTE_MERRF_BIT;
  MCP2515_SetRegister(MCP2515_CTRL_REG_CANINTE, cainte.reg);

  memset(&rxb0ctrl, 0, sizeof(rxb0ctrl));
  rxb0ctrl.cnvrt.rxm  = MCP2515_RXB0CTRL_RXM_STDEXT_BIT;
  rxb0ctrl.cnvrt.bukt = MCP2515_RXB0CTRL_BUKT_BIT; //! Rollover enable.

  MCP2515_ModifyRegister(MCP2515_CTRL_REG_RXB0CTRL, MCP2515_CTRL_REG_RXB0CTRL_RXM_MASK | 
                                                    MCP2515_CTRL_REG_RXB0CTRL_BUKT_MASK,
                                                    rxb0ctrl.reg);

  memset(&rxb1ctrl, 0, sizeof(rxb1ctrl));
  rxb1ctrl.cnvrt.rxm  = MCP2515_RXB1CTRL_RXM_STDEXT_BIT;

  MCP2515_ModifyRegister(MCP2515_CTRL_REG_RXB1CTRL, MCP2515_CTRL_REG_RXB1CTRL_RXM_MASK, rxb1ctrl.reg);

  return retval;
}

/**************************************************************************/

static MCP2515_ERROR MCP2515_SetMode(const MCP2515_CANCTRL_REQOP_TYPE mode)
{
  MCP2515_CTRL_REG_CANCTRL_TYPE canctrl;
  MCP2515_CTRL_REG_CANSTAT_TYPE canstat;
  MCP2515_ERROR                 retval  = MCP2515_SUCCESS;
  uint32_t startTime, endTime;

  memset(&canctrl, 0, sizeof(canctrl));
  canctrl.cnvrt.reqop = mode;  //! TODO Check value modes.
  MCP2515_ModifyRegister(MCP2515_CTRL_REG_CANCTRL, MCP2515_REG_CANCTRL_REQOP_MASK, canctrl.reg);

  startTime = TIME_GetMs();
  //! Wait for the operation to complete.
  do
  {
    canstat.reg  = MCP2515_ReadRegister(MCP2515_CTRL_REG_CANSTAT);
    endTime = TIME_GetMs();
  } while((canstat.cnvrt.opmod != mode) && ((endTime - startTime) < MCP2515_MAX_TIMEOUT_MS));

  retval = (canstat.cnvrt.opmod == mode) ? MCP2515_SUCCESS : MCP2515_FAIL;

  return retval;
}

/**************************************************************************/

static MCP2515_ERROR MCP2515_SetBitrate(const CAN_BUS_SPEED speed, const MCP2515_CLOCK clock)
{
  MCP2515_ERROR retval = MCP2515_SUCCESS;
  uint8_t       cfg1, cfg2, cfg3;

  retval = MCP2515_SetMode(MCP2515_CANCTRL_REQOP_CONFIG_MODE);

  switch (clock)
  {
    case MCP2515_CLOCK_8MHZ:
         switch (speed)
         {
            case (CAN_SPEED_5KBPS): //! 5Kbps.
            cfg1 = MCP2515_8MHz_5kBPS_CFG1;
            cfg2 = MCP2515_8MHz_5kBPS_CFG2;
            cfg3 = MCP2515_8MHz_5kBPS_CFG3;
            break;

            case (CAN_SPEED_10KBPS): //! 10Kbps.
            cfg1 = MCP2515_8MHz_10kBPS_CFG1;
            cfg2 = MCP2515_8MHz_10kBPS_CFG2;
            cfg3 = MCP2515_8MHz_10kBPS_CFG3;
            break;

            case (CAN_SPEED_20KBPS): //! 20Kbps.
            cfg1 = MCP2515_8MHz_20kBPS_CFG1;
            cfg2 = MCP2515_8MHz_20kBPS_CFG2;
            cfg3 = MCP2515_8MHz_20kBPS_CFG3;
            break;

            case (CAN_SPEED_31K25BPS): //! 31.25Kbps.
            cfg1 = MCP2515_8MHz_31k25BPS_CFG1;
            cfg2 = MCP2515_8MHz_31k25BPS_CFG2;
            cfg3 = MCP2515_8MHz_31k25BPS_CFG3;
            break;

            case (CAN_SPEED_33KBPS): //! 33.33Kbps.
            cfg1 = MCP2515_8MHz_33k3BPS_CFG1;
            cfg2 = MCP2515_8MHz_33k3BPS_CFG2;
            cfg3 = MCP2515_8MHz_33k3BPS_CFG3;
            break;

            case (CAN_SPEED_40KBPS): //! 40Kbps.
            cfg1 = MCP2515_8MHz_40kBPS_CFG1;
            cfg2 = MCP2515_8MHz_40kBPS_CFG2;
            cfg3 = MCP2515_8MHz_40kBPS_CFG3;
            break;

            case (CAN_SPEED_50KBPS): //! 50Kbps.
            cfg1 = MCP2515_8MHz_50kBPS_CFG1;
            cfg2 = MCP2515_8MHz_50kBPS_CFG2;
            cfg3 = MCP2515_8MHz_50kBPS_CFG3;
            break;

            case (CAN_SPEED_80KBPS): //! 80Kbps.
            cfg1 = MCP2515_8MHz_80kBPS_CFG1;
            cfg2 = MCP2515_8MHz_80kBPS_CFG2;
            cfg3 = MCP2515_8MHz_80kBPS_CFG3;
            break;

            case (CAN_SPEED_100KBPS): //! 100Kbps.
            cfg1 = MCP2515_8MHz_100kBPS_CFG1;
            cfg2 = MCP2515_8MHz_100kBPS_CFG2;
            cfg3 = MCP2515_8MHz_100kBPS_CFG3;
            break;

            case (CAN_SPEED_125KBPS): //! 125Kbps.
            cfg1 = MCP2515_8MHz_125kBPS_CFG1;
            cfg2 = MCP2515_8MHz_125kBPS_CFG2;
            cfg3 = MCP2515_8MHz_125kBPS_CFG3;
            break;

            case (CAN_SPEED_200KBPS): //! 200Kbps.
            cfg1 = MCP2515_8MHz_200kBPS_CFG1;
            cfg2 = MCP2515_8MHz_200kBPS_CFG2;
            cfg3 = MCP2515_8MHz_200kBPS_CFG3;
            break;

            case (CAN_SPEED_250KBPS): //! 250Kbps.
            cfg1 = MCP2515_8MHz_250kBPS_CFG1;
            cfg2 = MCP2515_8MHz_250kBPS_CFG2;
            cfg3 = MCP2515_8MHz_250kBPS_CFG3;
            break;

            case (CAN_SPEED_500KBPS): //! 500Kbps.
            cfg1 = MCP2515_8MHz_500kBPS_CFG1;
            cfg2 = MCP2515_8MHz_500kBPS_CFG2;
            cfg3 = MCP2515_8MHz_500kBPS_CFG3;
            break;

            case (CAN_SPEED_1000KBPS): //! 1Mbps.
            cfg1 = MCP2515_8MHz_1000kBPS_CFG1;
            cfg2 = MCP2515_8MHz_1000kBPS_CFG2;
            cfg3 = MCP2515_8MHz_1000kBPS_CFG3;
            break;

            default:
            break;
        }
         break;

    case MCP2515_CLOCK_16MHZ:
        switch (speed)
        {
            case (CAN_SPEED_5KBPS): //! 5Kbps.
            cfg1 = MCP2515_16MHz_5kBPS_CFG1;
            cfg2 = MCP2515_16MHz_5kBPS_CFG2;
            cfg3 = MCP2515_16MHz_5kBPS_CFG3;
            break;

            case (CAN_SPEED_10KBPS): //! 10Kbps.
            cfg1 = MCP2515_16MHz_10kBPS_CFG1;
            cfg2 = MCP2515_16MHz_10kBPS_CFG2;
            cfg3 = MCP2515_16MHz_10kBPS_CFG3;
            break;

            case (CAN_SPEED_20KBPS): //! 20Kbps.
            cfg1 = MCP2515_16MHz_20kBPS_CFG1;
            cfg2 = MCP2515_16MHz_20kBPS_CFG2;
            cfg3 = MCP2515_16MHz_20kBPS_CFG3;
            break;

            case (CAN_SPEED_33KBPS): //! 33.33Kbps.
            cfg1 = MCP2515_16MHz_33k3BPS_CFG1;
            cfg2 = MCP2515_16MHz_33k3BPS_CFG2;
            cfg3 = MCP2515_16MHz_33k3BPS_CFG3;
            break;

            case (CAN_SPEED_40KBPS): //! 40Kbps.
            cfg1 = MCP2515_16MHz_40kBPS_CFG1;
            cfg2 = MCP2515_16MHz_40kBPS_CFG2;
            cfg3 = MCP2515_16MHz_40kBPS_CFG3;
            break;

            case (CAN_SPEED_50KBPS): //! 50Kbps.
            cfg1 = MCP2515_16MHz_50kBPS_CFG1;
            cfg2 = MCP2515_16MHz_50kBPS_CFG2;
            cfg3 = MCP2515_16MHz_50kBPS_CFG3;
            break;

            case (CAN_SPEED_80KBPS): //! 80Kbps.
            cfg1 = MCP2515_16MHz_80kBPS_CFG1;
            cfg2 = MCP2515_16MHz_80kBPS_CFG2;
            cfg3 = MCP2515_16MHz_80kBPS_CFG3;
            break;

            case (CAN_SPEED_83K3BPS): //! 83.33Kbps.
            cfg1 = MCP2515_16MHz_83k3BPS_CFG1;
            cfg2 = MCP2515_16MHz_83k3BPS_CFG2;
            cfg3 = MCP2515_16MHz_83k3BPS_CFG3;
            break; 

            case (CAN_SPEED_100KBPS): // 100Kbps.
            cfg1 = MCP2515_16MHz_100kBPS_CFG1;
            cfg2 = MCP2515_16MHz_100kBPS_CFG2;
            cfg3 = MCP2515_16MHz_100kBPS_CFG3;
            break;

            case (CAN_SPEED_125KBPS): //! 125Kbps.
            cfg1 = MCP2515_16MHz_125kBPS_CFG1;
            cfg2 = MCP2515_16MHz_125kBPS_CFG2;
            cfg3 = MCP2515_16MHz_125kBPS_CFG3;
            break;

            case (CAN_SPEED_200KBPS): //! 200Kbps.
            cfg1 = MCP2515_16MHz_200kBPS_CFG1;
            cfg2 = MCP2515_16MHz_200kBPS_CFG2;
            cfg3 = MCP2515_16MHz_200kBPS_CFG3;
            break;

            case (CAN_SPEED_250KBPS): //! 250Kbps.
            cfg1 = MCP2515_16MHz_250kBPS_CFG1;
            cfg2 = MCP2515_16MHz_250kBPS_CFG2;
            cfg3 = MCP2515_16MHz_250kBPS_CFG3;
            break;

            case (CAN_SPEED_500KBPS): //! 500Kbps.
            cfg1 = MCP2515_16MHz_500kBPS_CFG1;
            cfg2 = MCP2515_16MHz_500kBPS_CFG2;
            cfg3 = MCP2515_16MHz_500kBPS_CFG3;
            break;

            case (CAN_SPEED_1000KBPS): //! 1Mbps.
            cfg1 = MCP2515_16MHz_1000kBPS_CFG1;
            cfg2 = MCP2515_16MHz_1000kBPS_CFG2;
            cfg3 = MCP2515_16MHz_1000kBPS_CFG3;
            break;

            default:
            break;
        }
         break;

    case MCP2515_CLOCK_20MHZ:
        switch (speed)
        {
            case (CAN_SPEED_40KBPS): //! 40Kbps.
            cfg1 = MCP2515_20MHz_40kBPS_CFG1;
            cfg2 = MCP2515_20MHz_40kBPS_CFG2;
            cfg3 = MCP2515_20MHz_40kBPS_CFG3;
            break;

            case (CAN_SPEED_50KBPS): //! 50Kbps.
            cfg1 = MCP2515_20MHz_50kBPS_CFG1;
            cfg2 = MCP2515_20MHz_50kBPS_CFG2;
            cfg3 = MCP2515_20MHz_50kBPS_CFG3;
            break;

            case (CAN_SPEED_80KBPS): //! 80Kbps.
            cfg1 = MCP2515_20MHz_80kBPS_CFG1;
            cfg2 = MCP2515_20MHz_80kBPS_CFG2;
            cfg3 = MCP2515_20MHz_80kBPS_CFG3;
            break;

            case (CAN_SPEED_100KBPS): //! 100Kbps.
            cfg1 = MCP2515_20MHz_100kBPS_CFG1;
            cfg2 = MCP2515_20MHz_100kBPS_CFG2;
            cfg3 = MCP2515_20MHz_100kBPS_CFG3;
            break;

            case (CAN_SPEED_125KBPS): //! 125Kbps.
            cfg1 = MCP2515_20MHz_125kBPS_CFG1;
            cfg2 = MCP2515_20MHz_125kBPS_CFG2;
            cfg3 = MCP2515_20MHz_125kBPS_CFG3;
            break;

            case (CAN_SPEED_200KBPS): //! 200Kbps.
            cfg1 = MCP2515_20MHz_200kBPS_CFG1;
            cfg2 = MCP2515_20MHz_200kBPS_CFG2;
            cfg3 = MCP2515_20MHz_200kBPS_CFG3;
            break;

            case (CAN_SPEED_250KBPS): //! 250Kbps.
            cfg1 = MCP2515_20MHz_250kBPS_CFG1;
            cfg2 = MCP2515_20MHz_250kBPS_CFG2;
            cfg3 = MCP2515_20MHz_250kBPS_CFG3;
            break;

            case (CAN_SPEED_500KBPS): //! 500Kbps.
            cfg1 = MCP2515_20MHz_500kBPS_CFG1;
            cfg2 = MCP2515_20MHz_500kBPS_CFG2;
            cfg3 = MCP2515_20MHz_500kBPS_CFG3;
            break;

            case (CAN_SPEED_1000KBPS): //! 1Mbps.
            cfg1 = MCP2515_20MHz_1000kBPS_CFG1;
            cfg2 = MCP2515_20MHz_1000kBPS_CFG2;
            cfg3 = MCP2515_20MHz_1000kBPS_CFG3;
            break;

            default:
            break;
        }
         break;

  default:
         break;
  }
    MCP2515_SetRegister(MCP2515_CTRL_REG_CNF1, cfg1);
    MCP2515_SetRegister(MCP2515_CTRL_REG_CNF2, cfg2);
    MCP2515_SetRegister(MCP2515_CTRL_REG_CNF3, cfg3);

  return retval;
}

/**************************************************************************/

static MCP2515_ERROR MCP2515_GetFreeTxBuffer(uint8_t *pIndex)
{
  MCP2515_CTRL_REG_TXBnCNRL_TYPE ctrl;
  MCP2515_ERROR                  retval = MCP2515_TX_NO_FREE_BUFFER;
  PMCP2515_CTX                   pCtx   = (PMCP2515_CTX) &mcp2515_ctx;

  if (NULL != pIndex)
  {
    *pIndex = 0;
    for( uint8_t i = 0; i < MCP2515_TXB_NUM_OF_BUFFERS; i++ )
    {
        ctrl.reg = MCP2515_ReadRegister(pCtx->pTXBnArray[i][MCP2515_TXBnREG_CTRL]); //! Check TXBnCTRL registers.
        //! If the TX buffer is free.
        if ( !ctrl.cnvrt.txreg )
        {
            *pIndex = i;
            retval  = MCP2515_SUCCESS;
            break;
        }
    }
  }
  else
  {
    retval = MCP2515_FAIL;
  }

  return retval;
}

/**************************************************************************/

MCP2515_ERROR MCP2515_SendMessage(const PCAN_FRAME pFrame)
{
  PMCP2515_CTX  pCtx   = (PMCP2515_CTX) &mcp2515_ctx;
  MCP2515_ERROR retval = MCP2515_FAIL;
  uint32_t      startTime, endTime;
  uint8_t       index;

  if (pFrame != NULL)
  {
    if (pFrame->length <= CAN_FRAME_MAX_LEN) 
    {
      retval = MCP2515_GetFreeTxBuffer(&index);

      if (!retval) 
      {
         MCP2515_CTRL_REG_TXBnCNRL_TYPE ctrl;
         MCP2515_CTRL_REG_TXBnSIDH_TYPE sidh;
         MCP2515_CTRL_REG_TXBnSIDL_TYPE sidl;
         MCP2515_CTRL_REG_TXBnDLC_TYPE  dlc;

         memset(&sidh, 0, sizeof(sidh));
         sidh.cnvrt.sid = (pFrame->id >> 3) & 0xff;
         MCP2515_SetRegister(pCtx->pTXBnArray[index][MCP2515_TXBnREG_SIDH], sidh.reg);

         memset(&sidl, 0, sizeof(sidl));
         sidl.cnvrt.sid = pFrame->id & 0x07;
         MCP2515_SetRegister(pCtx->pTXBnArray[index][MCP2515_TXBnREG_SIDL], sidl.reg);

         memset(&dlc, 0, sizeof(dlc));
         dlc.cnvrt.dlc = pFrame->length & 0x0F;
         MCP2515_SetRegister(pCtx->pTXBnArray[index][MCP2515_TXBnREG_DLC], dlc.reg);

         MCP2515_SetSetOfRegisters(pCtx->pTXBnArray[index][MCP2515_TXBnREG_DATA], pFrame->data, dlc.cnvrt.dlc);  //! Data is transmitted.

         memset(&ctrl, 0, sizeof(ctrl));
         ctrl.cnvrt.txreg = MCP2515_TXBnCNRL_TXREG_BIT;
         MCP2515_ModifyRegister(pCtx->pTXBnArray[index][MCP2515_TXBnREG_CTRL], MMCP2515_REG_TXBnCNRL_TXREQ_MASK, ctrl.reg);

         startTime = TIME_GetMs();
         do
         {
            ctrl.reg = MCP2515_ReadRegister(pCtx->pTXBnArray[index][MCP2515_TXBnREG_CTRL]);
            endTime  = TIME_GetMs();
         }while (ctrl.cnvrt.txreg && ((endTime - startTime) < MCP2515_MAX_TIMEOUT_MS));

         if((endTime - startTime) >= MCP2515_MAX_TIMEOUT_MS) //! TODO Rewrite the check.
         {
           retval = MCP2515_TX_DATA_STILL_IN_BUFFER;
         }
         else
         {
           retval = MCP2515_SUCCESS;
         }
      }
    }
    else
    {
      retval = MCP2515_TX_FAIL_MAX_LEN;
    }
  }

  return retval;
}

/**************************************************************************/

MCP2515_ERROR MCP2515_ReadMessage(PCAN_FRAME pFrame)
{
  PMCP2515_CTX        pCtx   = (PMCP2515_CTX) &mcp2515_ctx;
  MCP2515_ERROR       retval = MCP2515_FAIL;
  uint8_t             index;

  if (pFrame != NULL)
  {
    retval = MCP2515_CheckRxBuffer(&index);
    if (!retval)
    {
      MCP2515_CTRL_REG_RXBnSIDH_TYPE sidh;
      MCP2515_CTRL_REG_RXBnSIDL_TYPE sidl;
      MCP2515_CTRL_REG_RXBnDLC_TYPE  dlc;
      MCP2515_CTRL_REG_CANINTF_TYPE  caintf;

      sidh.reg = MCP2515_ReadRegister(pCtx->pRXBnArray[index][MCP2515_RXBnREG_SIDH]);
      sidl.reg = MCP2515_ReadRegister(pCtx->pRXBnArray[index][MCP2515_RXBnREG_SIDL]);
      pFrame->id = (sidh.cnvrt.sid << 3) | sidl.cnvrt.sid;

      dlc.reg = MCP2515_ReadRegister(pCtx->pRXBnArray[index][MCP2515_RXBnREG_DLC]);
      pFrame->length = dlc.cnvrt.dlc; //! TODO check the length.

      MCP2515_ReadSetOfRegisters(pCtx->pRXBnArray[index][MCP2515_RXBnREG_DATA], pFrame->data, pFrame->length);

      memset(&caintf, 0, sizeof(caintf));

      if (index) //! TODO rewrite this check.
      {
        caintf.cnvrt.rx1if = MCP2515_CANINTF_RX1IF_BIT;
      }
      else
      {
        caintf.cnvrt.rx0if = MCP2515_CANINTF_RX0IF_BIT;
      }

      MCP2515_ModifyRegister(MCP2515_CTRL_REG_CANINTF ,caintf.reg, 0x00);
    }
  }

  return retval;
}

/**************************************************************************/

static MCP2515_ERROR MCP2515_CheckRxBuffer(uint8_t *pIndex)
{
  MCP2515_CTRL_REG_CANINTF_TYPE  caintf;
  MCP2515_ERROR                  retval = MCP2515_SUCCESS;

  if (NULL != pIndex)
  {
    *pIndex = 0;
    caintf.reg = MCP2515_ReadRegister(MCP2515_CTRL_REG_CANINTF);

    if (caintf.cnvrt.rx0if)
    {
      *pIndex = 0; //! TODO Change non definition names (numbers).
    } 
    else if (caintf.cnvrt.rx1if)
    {
      *pIndex = 1;
    }
    else
    {
      retval = MCP2515_RX_NO_MSG_DATA;
    }
  }
  else
  {
    retval = MCP2515_FAIL;
  }

  return retval;
}

/**************************************************************************/

void MCP2515_SetMask(MCP2515_RXMnREGISTERS rxm, uint16_t mask)
{
  MCP2515_CTRL_REG_RXMnSIDH_TYPE sidh;
  MCP2515_CTRL_REG_RXMnSIDL_TYPE sidl;

  MCP2515_SetMode(MCP2515_CANCTRL_REQOP_CONFIG_MODE);

  memset(&sidh, 0, sizeof(sidh));
  memset(&sidl, 0, sizeof(sidl));

  sidh.cnvrt.sid = (mask >> 3) & 0xff;
  sidl.cnvrt.sid = mask & 0x07;

  switch ( rxm )
  {
    case MCP2515_RXM0REGISTERS:
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXM0SIDH, sidh.reg);
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXM0SIDL, sidl.reg);
      break;

    case MCP2515_RXM1REGISTERS:
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXM1SIDH, sidh.reg);
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXM1SIDL, sidl.reg);
      break;

    default:
      break;
  }

  MCP2515_SetMode(MCP2515_CANCTRL_REQOP_NORMAL_MODE);
}

/**************************************************************************/

void MCP2515_SetFilter(MCP2515_RXFnREGISTERS rxf, uint16_t filter)
{
  MCP2515_CTRL_REG_RXFnSIDH_TYPE sidh;
  MCP2515_CTRL_REG_RXFnSIDL_TYPE sidl;

  MCP2515_SetMode(MCP2515_CANCTRL_REQOP_CONFIG_MODE);

  memset(&sidh, 0, sizeof(sidh));
  memset(&sidl, 0, sizeof(sidl));

  sidh.cnvrt.sid = (filter >> 3) & 0xff;
  sidl.cnvrt.sid = filter & 0x07;

  //! Extended Identifier Enable bit is set to 0. Filter is applied only to standard frames.

    switch ( rxf )
  {
    case MCP2515_RXF0REGISTERS:
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXF0SIDH, sidh.reg);
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXF0SIDL, sidl.reg);
      break;

    case MCP2515_RXF1REGISTERS:
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXF1SIDH, sidh.reg);
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXF1SIDL, sidl.reg);
      break;

    case MCP2515_RXF2REGISTERS:
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXF2SIDH, sidh.reg);
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXF2SIDL, sidl.reg);
      break;

    case MCP2515_RXF3REGISTERS:
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXF3SIDH, sidh.reg);
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXF3SIDL, sidl.reg);
      break;

    case MCP2515_RXF4REGISTERS:
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXF4SIDH, sidh.reg);
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXF4SIDL, sidl.reg);
      break;

    case MCP2515_RXF5REGISTERS:
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXF5SIDH, sidh.reg);
      MCP2515_SetRegister(MCP2515_CTRL_REG_RXF5SIDL, sidl.reg);
      break;

    default:
      break;
  }

  MCP2515_SetMode(MCP2515_CANCTRL_REQOP_NORMAL_MODE);
}