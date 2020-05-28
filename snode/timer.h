/**************************************************************************
 * @file       timer.h                                                    *
 * @author     Oleksandr Kornuta (robot@boteon.com)                       *
 * @license    This project is released under the MIT License (MIT)       *
 * @copyright  Copyright (c) 2019 Boteon                                  *
 * @date       November 2019                                              *
 * @brief      This timer library is a part of the snode project.         *
 *************************************************************************/

#ifndef _TIMER_H_ // Timer
#define _TIMER_H_ 

#ifdef _cplusplus_
extern "C"
{
#endif

#include <stdint.h>

/**************************************************************************
 * User configuration defines.                                            *
 **************************************************************************
 */
#define TIMER_MAX_TASKS            (6)

/**************************************************************************/

typedef bool (* TIMER_HANDLER)(void *);

/**************************************************************************/

typedef struct _TIMER_TASK_CTX
{
  TIMER_HANDLER handler;
  void          *pParametr;
  uint32_t      start;
  uint32_t      expires;
}TIMER_TASK_CTX, *PTIMER_TASK_CTX;

/**************************************************************************
 * Global function declarations.                                          *
 **************************************************************************
 */
void TIMER_Init(void);
bool TIMER_AddTask(uint32_t ms, TIMER_HANDLER handler, void * pParametr);
void TIMER_Tick(void);

#ifdef _cplusplus
}
#endif

#endif // _TIMER_H_