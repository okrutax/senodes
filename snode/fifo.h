/**************************************************************************
 * @file       fifo.h                                                     *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      The fifo library of the snode project.                     *
 *************************************************************************/

#ifndef _FIFO_H_ // Fifo
#define _FIFO_H_ 

#ifdef _cplusplus_
extern "C"
{
#endif

#include "mcp2515.h"

/**************************************************************************/

typedef enum _FIFO_ERROR
{
  FIFO_SUCCESS     = 0x00,
  FIFO_FAIL        = 0x01,
  FIFO_NO_ELEMENTS = 0x02,
  FIFO_OVERFLOW    = 0x03
}FIFO_ERROR, *PFIFO_ERROR;

/**************************************************************************/

typedef struct _FIFO_ELEMENT
{
  CAN_FRAME             frame; //! FIFO data element.
  struct _FIFO_ELEMENT *pNext; //! Next in the linked list.
  struct _FIFO_ELEMENT *pPrev; //! Previous in the linked list.
}FIFO_ELEMENT, *PFIFO_ELEMENT;

/**************************************************************************
 * Main FIFO structure.                                                   *
 **************************************************************************
 */
typedef struct _FIFO_HANDLE
{
  PFIFO_ELEMENT pHead;
  PFIFO_ELEMENT pTail;
}FIFO_HANDLE, *PFIFO_HANDLE;

/**************************************************************************
 * Global function declarations.                                          *
 **************************************************************************
 */
PFIFO_HANDLE FIFO_Create(void);
FIFO_ERROR   FIFO_Destroy(PFIFO_HANDLE pFifoHandle);
FIFO_ERROR FIFO_Put(PFIFO_HANDLE pFifoHandle, PCAN_FRAME pCanFrame);
FIFO_ERROR FIFO_Get(PFIFO_HANDLE pFifoHandle, PCAN_FRAME pCanFrame);

#ifdef _cplusplus
}
#endif

#endif // _FIFO_H_