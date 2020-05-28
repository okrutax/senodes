/**************************************************************************
 * @file       snode_cnfg.h                                               *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      The configuration of the snode project.                    *
 *************************************************************************/

#ifndef _SNODE_CNFG_H_ // Sensor Node
#define _SNODE_CNFG_H_ 

#ifdef _cplusplus_
extern "C"
{
#endif

#include <stdint.h>

//#define CNFG_CAN_FRAME_SOURCE_ID     (203) //! TODO: Change to auto addresing.

typedef struct _SNODE_JOYXY
{
  uint16_t x;
  uint16_t y;
}SNODE_JOYXY, *PSNODE_JOYXY;

#ifdef _cplusplus
}
#endif

#endif // _SNODE_CNFG_H_