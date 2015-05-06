/**
 * @file    hrt.h
 * @brief   RTCAN HRT messages handling.
 *
 * @addtogroup HRT
 * @{
 */

#ifndef _HRT_H_
#define _HRT_H_

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

/**
 * @brief   RTCAN Driver type forward declaration.
 */
typedef struct RTCANDriver RTCANDriver;

/**
 * @brief   RTCAN HRT message type forward declaration.
 */
typedef struct hrt_msg_t hrt_msg_t;

/**
 * @brief   RTCAN HRT message data update callback type.
 */
typedef void (*hrt_updatecb_t)(hrt_msg_t* msgp);

/**
 * @brief   RTCAN HRT message type forward declaration.
 */
struct __attribute__ ((__packed__)) hrt_msg_t
{
	hrt_msg_t * next;
	hrt_updatecb_t update; // TODO move to a shared location
	rtcan_msgstatus_t status;
	uint16_t id :16; // TODO move to a shared location
	uint16_t size; // TODO move to a shared location
	uint32_t slot;
	uint16_t period;
	uint8_t * data;
	uint8_t * ptr;
	uint8_t fragment :7;
};

/**
 * @brief   RTCAN HRT reservation calendar type.
 */
typedef struct {
	hrt_msg_t * next;
} hrt_calendar_t;

/*===========================================================================*/
/* Macros.                                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
void hrt_init(void);
void hrt_reserve(RTCANDriver * rtcanp, hrt_msg_t * new_msg_p);
bool_t hrt_reserved_slot(RTCANDriver * rtcanp);
void hrt_update(hrt_msg_t * msg_p);
void hrt_transmit(RTCANDriver * rtcanp);
#ifdef __cplusplus
}
#endif

#endif /* _HRT_H_ */

/** @} */


