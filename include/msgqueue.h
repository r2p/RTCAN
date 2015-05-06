/**
 * @file    msgqueue.h
 * @brief   RTCAN message queue functions.
 *
 * @addtogroup XXX
 * @{
 */

#ifndef _MSGQUEUE_H_
#define _MSGQUEUE_H_

#include "rtcan.h"

/*===========================================================================*/
/* Constants.                                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Pre-compile time settings.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Data structures and types.                                         	     */
/*===========================================================================*/

typedef struct
{
	rtcan_msg_t *next;
} msgqueue_t;

/*===========================================================================*/
/* Macros.                                                                   */
/*===========================================================================*/

#define msgqueue_isempty(queue_p) \
  ((void *)(queue_p) == (void *)(queue_p)->next)
/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
	void msgqueue_init(msgqueue_t* queuep);
	void msgqueue_insert(msgqueue_t* queuep, rtcan_msg_t* msgp);
	void msgqueue_remove(msgqueue_t* queuep, rtcan_msg_t* msgp);
	rtcan_msg_t* msgqueue_get(msgqueue_t* queue_p);
#ifdef __cplusplus
}
#endif

#endif /* _MSGQUEUE_H_ */

/** @} */
