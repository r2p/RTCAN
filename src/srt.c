#include "ch.h"
#include "rtcan.h"

#include "msgqueue.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

uint8_t srt_laxity(rtcan_msg_t* msgp) {
	int32_t laxity;

	laxity = msgp->deadline - chTimeNow();

	if ((laxity < 0) || (laxity > 64)) {
		return 0x3F;
	}

	return laxity & 0x3F;
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/



bool_t srt_transmit(RTCANDriver * rtcanp) {
	rtcan_msg_t* msgp;
	rtcan_txframe_t txfp;

	if (msgqueue_isempty(&(rtcanp->srt_queue))) {
		return FALSE;
	}

	if (!rtcan_lld_can_txe(rtcanp)) {
		return FALSE;
	}

	if (rtcanp->onair[0]) { // XXX DEBUG
		return FALSE;
	}

	msgp = msgqueue_get(&(rtcanp->srt_queue));


	/* Check for deadline */
	/*
	if (msgp->deadline <= chTimeNow()) {
		msgp->status = RTCAN_MSG_TIMEOUT;
		return FALSE;
	}
    */

	txfp.id = ((srt_laxity(msgp) << 23) | ((msgp->id & 0xFFFF) << 7) | ((msgp->fragment) & 0x7F));

	if (msgp->fragment > 0) {
		txfp.len = RTCAN_FRAME_SIZE;
	} else {
		txfp.len = msgp->data + msgp->size - msgp->ptr;
	}

	txfp.data32[0] = *(uint32_t *)msgp->ptr;
	txfp.data32[1] = *((uint32_t *)msgp->ptr + 1);

	rtcan_lld_can_transmit(rtcanp, &txfp);
	rtcanp->onair[txfp.mbox] = msgp;
	msgp->status = RTCAN_MSG_ONAIR;

	return TRUE;
}
