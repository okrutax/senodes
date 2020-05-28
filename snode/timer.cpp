/**************************************************************************
 * @file       timer.cpp                                                  *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      This timer library is a part of the snode project.         *
 *************************************************************************/

#include <string.h>
#include "timer.h"
#include "Arduino-hal/time_hal.h"

/**************************************************************************
 * Static variables.                                                      *
 **************************************************************************
 */
static TIMER_TASK_CTX timer_tasks_ctx[TIMER_MAX_TASKS];

/**************************************************************************
 * Static function declarations.                                          *
 **************************************************************************
 */
static PTIMER_TASK_CTX TIMER_GetNextTaskSlot(void);

/**************************************************************************/

void TIMER_Init(void)
{
  memset(timer_tasks_ctx, 0x00, sizeof(timer_tasks_ctx));
}

/**************************************************************************/

static PTIMER_TASK_CTX TIMER_GetNextTaskSlot(void)
{
  for ( uint8_t i = 0; i < TIMER_MAX_TASKS; i++ )
  {
    PTIMER_TASK_CTX pTask = &timer_tasks_ctx[i];
    if ( NULL == pTask->handler )
    {
      return pTask;
    }
  }
  return NULL;
}

/**************************************************************************/

bool TIMER_AddTask(uint32_t ms, TIMER_HANDLER handler, void * pParametr)
{
  PTIMER_TASK_CTX pTask;
  bool            bRetval = false;

  pTask = TIMER_GetNextTaskSlot();

  if ( NULL != pTask )
  {
    pTask->start   = TIME_GetMs();
    pTask->expires = ms;
    pTask->handler = handler;
    pTask->pParametr = pParametr;

    bRetval = true;
  }

  return bRetval;
}

/**************************************************************************/

void TIMER_Tick(void)
{
  uint32_t currentTime = TIME_GetMs();

  for ( uint8_t i = 0; i < TIMER_MAX_TASKS; i++ )
  {
    PTIMER_TASK_CTX pTask = &timer_tasks_ctx[i];

    if ( ( NULL != pTask ) && ( NULL != pTask->handler ) )
    {
      uint32_t duration = currentTime - pTask->start;
      if ( duration >= pTask->expires )
      {
        bool bRepeat = pTask->handler(pTask->pParametr);

        if ( bRepeat )
        {
          pTask->start = currentTime;
        }
        else //! Remove from ctx.
        {
          pTask->handler   = NULL;
          pTask->pParametr = NULL;
          pTask->start     = 0x00;
          pTask->expires   = 0x00;
        }
      }
    }
  }
}