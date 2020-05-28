/**************************************************************************
 * @file       fifo.h                                                     *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      The fifo library of the snode project.                     *
 *************************************************************************/

#include <stdlib.h>
#include "fifo.h"

/**************************************************************************
 * FIFO Static variables.                                                 *
 **************************************************************************
 */
PFIFO_HANDLE  pTxQueue;

/**************************************************************************/

PFIFO_HANDLE FIFO_Create(void)
{
  PFIFO_HANDLE pFifoHandle;

  pFifoHandle = (PFIFO_HANDLE) malloc(sizeof(*pFifoHandle));
  if ( NULL == pFifoHandle )
  {
    return NULL;
  }

  pFifoHandle->pHead = NULL;
  pFifoHandle->pTail = NULL;

  return pFifoHandle;
}

/**************************************************************************/

FIFO_ERROR FIFO_Put(PFIFO_HANDLE pFifoHandle, PCAN_FRAME pCanFrame)
{
  FIFO_ERROR    retval      = FIFO_FAIL;
  PFIFO_ELEMENT pFifoElement;

  if ( pFifoHandle && pCanFrame )
  {
    pFifoElement = (PFIFO_ELEMENT) malloc(sizeof(*pFifoElement));

    if ( pFifoElement ) //! If the FIFO element is allocated.
    {
      memcpy(&pFifoElement->frame, pCanFrame, sizeof(*pCanFrame));

      if ( NULL == pFifoHandle->pHead )
      {
        //! This is the first element in the FIFO.
        pFifoHandle->pHead = pFifoElement;
        pFifoHandle->pTail = pFifoElement;

        //! This new element does not have a previous and the next elements.
        pFifoElement->pNext = NULL;
        pFifoElement->pPrev = NULL;
      }
      else
      {
        //! We already have some elements in the FIFO.
        pFifoHandle->pTail->pPrev = pFifoElement;
        pFifoElement->pNext       = pFifoHandle->pTail->pPrev;
        pFifoElement->pPrev       = NULL;
        pFifoHandle->pTail        = pFifoElement; //! Append this element to the end of the FIFO.
      }

      retval = FIFO_SUCCESS;
    }
    else
    {
      retval = FIFO_OVERFLOW;
    }
  }

  return retval;
}

/**************************************************************************/

FIFO_ERROR FIFO_Get(PFIFO_HANDLE pFifoHandle, PCAN_FRAME pCanFrame)
{
  FIFO_ERROR    retval      = FIFO_FAIL;
  PFIFO_ELEMENT pFifoElement;

  if ( pFifoHandle && pCanFrame )
  {
    if ( pFifoHandle->pHead )
    {
      pFifoElement = pFifoHandle->pHead;

      memcpy(pCanFrame, &pFifoElement->frame, sizeof(*pCanFrame));

      pFifoHandle->pHead        = pFifoElement->pPrev;
      pFifoHandle->pHead->pNext = NULL;

      free(pFifoElement);

      retval = FIFO_SUCCESS;
    }
    else
    {
      //! There are no elements in FIFO.
      retval = FIFO_NO_ELEMENTS;
    }
  }

  return retval;
}

/**************************************************************************/

FIFO_ERROR FIFO_Destroy(PFIFO_HANDLE pFifoHandle)
{
  FIFO_ERROR    retval     = FIFO_FAIL;
  PFIFO_ELEMENT pCurrent, pNext;

  if ( pFifoHandle )
  {
    pCurrent = pFifoHandle->pHead;
    while (pCurrent)
    {
      pNext = pCurrent->pPrev;
      free(pCurrent);
      pCurrent = pNext;
    }
    free(pFifoHandle);
    retval = FIFO_SUCCESS;
  }
  return retval;
}