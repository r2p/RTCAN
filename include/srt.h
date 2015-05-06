/**
 * @file    hrt.h
 * @brief   RTCAN SRT messages handling.
 *
 * @addtogroup SRT
 * @{
 */

#ifndef _SRT_H_
#define _SRT_H_

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
typedef struct srt_msg_t srt_msg_t;

/**
 * @brief   RTCAN SRT message transmission callback type.
 */
typedef void (*srt_txcb_t)(srt_msg_t* msgp);

/**
 * @brief   RTCAN HRT message type forward declaration.
 */
struct __attribute__ ((__packed__)) srt_msg_t
{
	srt_msg_t * next;
	srt_txcb_t callback; // TODO move to a shared location
	rtcan_msgstatus_t status;
	uint16_t id :16; // TODO move to a shared location
	uint16_t size; // TODO move to a shared location
	uint32_t deadline;
	uint8_t * data;
	uint8_t * ptr;
	uint8_t fragment :7;
};

/**
 * @brief   RTCAN SRT message queue type.
 */
// TODO

/*===========================================================================*/
/* Macros.                                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
void srt_init(void);
bool_t srt_transmit(RTCANDriver * rtcanp);
#ifdef __cplusplus
}
#endif

#endif /* _SRT_H_ */

/** @} */


