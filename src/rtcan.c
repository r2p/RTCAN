#include "ch.h"
#include "hal.h"
#include "rtcan.h"

#include <math.h>

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

uint32_t last_sync_tim = 0;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

// TODO: restore old master arbitration protocol
uint8_t stm32_id8(void) {
	const unsigned long * uid = (const unsigned long *) 0x1FFFF7E8;

	return (uid[2] & 0xFF);
}

bool_t rtcan_ismaster(void) {
#ifdef RTCAN_ISMASTER
	return TRUE;
#else
	return FALSE;
#endif
}

#if RTCAN_USE_HRT
void rtcan_sync_transmit(RTCANDriver * rtcanp) {
	rtcan_txframe_t sync_frame;

	sync_frame.id = (0xFF & rtcanp->cycle) << 7;
	sync_frame.len = 8;
	sync_frame.data32[0] = rtcanp->reservation_mask[0];
	sync_frame.data32[1] = rtcanp->reservation_mask[1];

	rtcan_lld_can_transmit(rtcanp, &sync_frame);
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

bool_t rtcan_hrt_slot(RTCANDriver * rtcanp);

/**
 * @brief   Timer interrupt platform-independent code.
 *
 * @api
 */
void rtcan_tim_isr_code(RTCANDriver * rtcanp) {

	rtcanLockFromIsr()
	;

	switch (rtcanp->state) {
#if RTCAN_USE_HRT
	case RTCAN_MASTER:
	if (rtcanp->slot == (rtcanp->config->slots - 1)) {
		rtcanp->cycle++;
		rtcanp->slot = 0;
		rtcan_sync_transmit(rtcanp);
		chSysUnlockFromIsr();
		return;
	}
	break;
#endif
	case RTCAN_SLAVE:
		if (rtcanp->slot > rtcanp->config->slots) {
			rtcanp->state = RTCAN_ERROR;
		}
		break;
	case RTCAN_SYNCING:
		rtcanp->slot++;
		chSysUnlockFromIsr()
		;
		return;
		break;
	default:
		/* Should never happen. */
		rtcanp->state = RTCAN_ERROR;
		break;
	}

	rtcanp->slot++;

#if RTCAN_USE_HRT
	if (hrt_reserved_slot(rtcanp)) {
		hrt_transmit(rtcanp);
		chSysUnlockFromIsr();
		return;
	}
#endif

	srt_transmit(rtcanp);

	chSysUnlockFromIsr();
}

/**
 * @brief   Transmission completed interrupt platform-independent code.
 *
 * @api
 */
void rtcan_txok_isr_code(RTCANDriver * rtcanp, rtcan_mbox_t mbox) {
	rtcan_msg_t *msgp;

	chSysLockFromIsr()
	;

	msgp = rtcanp->onair[mbox];
	rtcanp->onair[mbox] = NULL;

	if (msgp->fragment > 0) {
		msgp->fragment--;
		msgp->ptr += RTCAN_FRAME_SIZE;
		msgqueue_insert(&(rtcanp->srt_queue), msgp);
		msgp->status = RTCAN_MSG_QUEUED;
	} else {
		msgp->status = RTCAN_MSG_READY;
		if (msgp->callback) {
			msgp->callback(msgp);
		}
	}

#if !(RTCAN_USE_HRT)
	srt_transmit(rtcanp);
#endif

	chSysUnlockFromIsr();
}

/**
 * @brief   Arbitration lost interrupt platform-independent code.
 *
 * @api
 */
void rtcan_alst_isr_code(RTCANDriver * rtcanp, rtcan_mbox_t mbox) {
	rtcan_msg_t* msgp;

	chSysLockFromIsr()
	;

	msgp = rtcanp->onair[mbox];
	rtcanp->onair[mbox] = NULL;

	if (msgp == NULL) {
		/* should never happen */
		while (1)
			;
		chSysUnlockFromIsr();
		return;
	}

	msgqueue_insert(&(rtcanp->srt_queue), msgp);
	msgp->status = RTCAN_MSG_QUEUED;

#if !(RTCAN_USE_HRT)
	srt_transmit(rtcanp);
#endif

	chSysUnlockFromIsr();
}

/**
 * @brief   Transmission error interrupt platform-independent code.
 *
 * @api
 */
void rtcan_terr_isr_code(RTCANDriver * rtcanp, rtcan_mbox_t mbox) {
	rtcan_msg_t* msgp;

	chSysLockFromIsr()
	;

	msgp = rtcanp->onair[mbox];
	rtcanp->onair[mbox] = NULL;

	if (msgp == NULL) {
		/* should never happen */
		while (1)
			;
		chSysUnlockFromIsr();
		return;
	}

	msgp->status = RTCAN_MSG_TIMEOUT;

	if (msgp->callback) {
		msgp->callback(msgp);
	}

#if !(RTCAN_USE_HRT)
	srt_transmit(rtcanp);
#endif

	chSysUnlockFromIsr();
}

/**
 * @brief   Receive interrupt platform-independent code.
 *
 * @api
 */
void rtcan_rx_isr_code(RTCANDriver * rtcanp) {
	rtcan_rxframe_t rxf;
	rtcan_msg_t * msgp;

	chSysLockFromIsr()
	;

	rtcan_lld_can_receive(rtcanp, &rxf);

#if RTCAN_USE_HRT
	if ((rxf.id & 0x7F8000) == 0) {
		switch (rtcanp->state) {
			case RTCAN_SLAVE:
			last_sync_tim = rtcan_lld_tim_get_counter(rtcanp);
			rtcanp->cycle++;
			rtcanp->slot = 0;
			// FIXME: should be measured.
			rtcan_lld_tim_set_counter(rtcanp, 158);
			break;

			case RTCAN_SYNCING:
			if (rtcanp->cycle == 0) {
				rtcan_lld_tim_start_timer(rtcanp);
			}

			if (rtcanp->cycle == rtcanp->config->clock) {
				rtcan_lld_tim_stop_timer(rtcanp);
				float tmp = (rtcanp->slot * 0xFFFF + rtcan_lld_tim_get_counter(rtcanp)) / (rtcanp->config->clock * rtcanp->config->slots);
				uint32_t interval = (uint32_t) (tmp + 0.5f);
				rtcanp->state = RTCAN_SLAVE;
				rtcanp->cycle = (rxf.id >> 7) & 0xFF;
				rtcanp->slot = 0;
				rtcan_lld_tim_set_interval(rtcanp, interval);
				rtcan_lld_tim_start_timer(rtcanp);
				// FIXME: should be measured in loopback as it was done in v1.
				rtcan_lld_tim_set_counter(rtcanp, 158);
				break;
			}

			rtcanp->cycle++;

			break;

			default:
			break;
		}

		chSysUnlockFromIsr();
		return;
	}
#endif /* RTCAN_USE_HRT */

	msgp = rtcanp->filters[rxf.filter];

	/* Should never happen. */
	if (msgp == NULL) {
		chSysUnlockFromIsr();
		while (1)
			;
		return;
	}

	if (msgp->status == RTCAN_MSG_READY) {
		msgp->status = RTCAN_MSG_ONAIR;
		msgp->ptr = (uint8_t *)msgp->data;
		msgp->id = (rxf.id >> 7) & 0xFFFF;

		/* Reset fragment counter. */
		if (msgp->size > RTCAN_FRAME_SIZE) {
			msgp->fragment = (msgp->size - 1) / RTCAN_FRAME_SIZE;
		} else {
			msgp->fragment = 0;
		}
	}

	if (msgp->status == RTCAN_MSG_ONAIR) {
		uint32_t i;

		/* check source (needed by mw v2). */
		uint8_t source = (rxf.id >> 7) & 0xFF;
		uint8_t prevsource = msgp->id & 0xFF;
		if (source != prevsource) {
//		if (msgp->id != ((rxf.id >> 7) & 0xFFFF)) {
			chSysUnlockFromIsr();
			return;
		}

		/* check fragment */
		uint8_t fragment = rxf.id & 0x7F;
		if (fragment != msgp->fragment) {
			msgp->status = RTCAN_MSG_READY;
			chSysUnlockFromIsr();
			return;
		}

		for (i = 0; i < rxf.len; i++) {
			*(msgp->ptr++) = rxf.data8[i];
		}

		if (msgp->fragment > 0) {
			msgp->fragment--;
		} else {
			if (msgp->callback) {
				msgp->status = RTCAN_MSG_BUSY;
				msgp->callback(msgp);
			}
		}
	}

	if (msgp->status == RTCAN_MSG_ERROR) {
		msgp->callback(msgp);
	}
	chSysUnlockFromIsr();
}

/**
 * @brief   RTCAN Driver initialization.
 *
 * @init
 */
void rtcanInit(void) {

#if RTCAN_USE_HRT
	rtcan_lld_tim_init();
#endif
	rtcan_lld_can_init();
}

/**
 * @brief   RTCAN Driver reset.
 *
 * @param[in] rtcanp     pointer to the @p RTCANDriver object
 *
 * @init
 */
void rtcanReset(RTCANDriver * rtcanp) {
	int i;

	rtcanp->state = RTCAN_STOP;
	rtcanp->config = NULL;

	msgqueue_init(&(rtcanp->srt_queue));

	for (i = 0; i < RTCAN_MBOX_NUM; i++) {
		rtcanp->onair[i] = NULL;
	}

	for (i = 0; i < RTCAN_FILTERS_NUM; i++) {
		rtcanp->filters[i] = NULL;
	}

#if RTCAN_USE_HRT
	hrt_reset(rtcanp);
#endif /* RTCAN_USE_HRT */

}

/**
 * @brief   Configures and activates the CAN peripheral.
 * @note    Activating the CAN bus can be a slow operation this this function
 *          is not atomic, it waits internally for the initialization to
 *          complete.
 *
 * @param[in] rtcanp    pointer to the @p RTCANDriver object
 * @param[in] config    pointer to the @p CANConfig object. Depending on
 *                      the implementation the value can be @p NULL.
 *
 * @api
 */
void rtcanStart(RTCANDriver *rtcanp, const RTCANConfig *config) {

	rtcanDbgCheck((rtcanp != NULL) || (config != NULL), "rtcanStart");

	chSysLock()
	;
	rtcanDbgAssert((rtcanp->state == RTCAN_STOP),
			"rtcanStart(), #1", "invalid state");

	if (rtcanp->state == RTCAN_STOP) {
		rtcanp->config = config;
		rtcan_lld_can_start(rtcanp);
		rtcanp->state = RTCAN_STARTING;
	}

#if RTCAN_USE_HRT
	rtcan_lld_tim_start(rtcanp);

	if (rtcan_ismaster()) {
		rtcanp->state = RTCAN_MASTER;
		// TODO: to be estimated (loopback).
		rtcan_lld_tim_set_interval(rtcanp, 166);
		rtcan_lld_tim_start_timer(rtcanp);
	} else {
		rtcanp->state = RTCAN_SYNCING;
		// TODO: calculate from config?.
		rtcan_lld_tim_set_interval(rtcanp, 0xFFFF);
		rtcan_filter_t filter;
		rtcan_lld_can_addfilter(rtcanp, 0, (0xFF00 << 7), &filter);
	}
#else
	rtcanp->state = RTCAN_SLAVE;
#endif /* RTCAN_USE_HRT */

	chSysUnlock();

	while (rtcanp->state == RTCAN_SYNCING) {
		rtcan_blinker();
	}
}

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] rtcanp      pointer to the @p RTCANDriver object
 *
 * @api
 */
void rtcanStop(RTCANDriver *rtcanp) {

	rtcanDbgCheck(rtcanp != NULL, "rtcanStop");

	chSysLock()
	;
	rtcanDbgAssert((rtcanp->state == RTCAN_MASTER) || (rtcanp->state == RTCAN_SLAVE),
			"rtcanStop(), #1", "invalid state");
	rtcan_lld_can_stop(rtcanp);
	rtcanp->state = RTCAN_STOP;
	chSysUnlock();
}

/**
 * @brief   XXX.
 *
 * @api
 */
void rtcanTransmit(RTCANDriver * rtcanp, rtcan_msg_t *msgp, uint32_t timeout) {

	rtcanLock()
	;
	rtcanTransmitI(rtcanp, msgp, timeout);
	rtcanUnlock();
}

/**
 * @brief   XXX.
 *
 * @api
 */
void rtcanTransmitI(RTCANDriver * rtcanp, rtcan_msg_t *msgp, uint32_t timeout) {

	/* Lock message */
	msgp->status = RTCAN_MSG_BUSY;

	/* Compute absolute deadline */
	msgp->deadline = chTimeNow() + timeout;

	/* Reset fragment counter. */
	if (msgp->size > RTCAN_FRAME_SIZE) {
		msgp->fragment = (msgp->size - 1) / RTCAN_FRAME_SIZE;
	} else {
		msgp->fragment = 0;
	}

	/* Reset data pointer. */
	msgp->ptr = (uint8_t *)msgp->data;

	msgqueue_insert(&(rtcanp->srt_queue), msgp);
	msgp->status = RTCAN_MSG_QUEUED;
#if !(RTCAN_USE_HRT)
	srt_transmit(rtcanp);
#endif
}

/**
 * @brief   XXX.
 *
 * @api
 */
void rtcanReceive(RTCANDriver * rtcanp, rtcan_msg_t *msgp) {
	rtcan_filter_t filter;

	rtcanLock();

	/* Add the hardware filter. */
	rtcan_lld_can_addfilter(rtcanp, (msgp->id & 0xFFFF) << 7, 0xFFFF << 7, &filter);
	rtcanp->filters[filter] = msgp;

	rtcanUnlock();
}

void rtcanReceiveMask(RTCANDriver * rtcanp, rtcan_msg_t *msgp, uint32_t mask) {
	rtcan_filter_t filter;

	rtcanLock();

	/* Add the hardware filter. */
	rtcan_lld_can_addfilter(rtcanp, (msgp->id & 0xFFFF) << 7, (mask & 0xFFFF) << 7, &filter);
	rtcanp->filters[filter] = msgp;

	rtcanUnlock();
}

/**
 * @brief   XXX.
 *
 * @api
 */

#if RTCAN_USE_HRT
void rtcanGetTime(RTCANDriver * rtcanp, rtcan_time_t * timep) {
	uint32_t cycle;
	uint32_t slot;
	rtcan_cnt_t cnt;
	long usecs;

	do {
		cycle = rtcanp->cycle;
		slot = rtcanp->slot;
		cnt = rtcan_lld_tim_get_counter(rtcanp);
	} while (slot != rtcanp->slot);

	usecs = ((cycle * rtcanp->config->slots) + rtcanp->slot) * (1000000 / rtcanp->config->clock / rtcanp->config->slots)
			+ cnt;

	timep->sec = usecs / 1000000;
	timep->nsec = (usecs % 1000000) * 1000;
}
#endif

void rtcan_blinker(void) {
#ifdef LED1
	switch (RTCAND1.state) {
	case RTCAN_MASTER:
		palClearPad(LED1_GPIO, LED1);
		chThdSleepMilliseconds(200);
		palSetPad(LED1_GPIO, LED1);
		chThdSleepMilliseconds(100);
		palClearPad(LED1_GPIO, LED1);
		chThdSleepMilliseconds(200);
		palSetPad(LED1_GPIO, LED1);
		chThdSleepMilliseconds(500);
		break;
	case RTCAN_SYNCING:
		palTogglePad(LED1_GPIO, LED1);
		chThdSleepMilliseconds(100);
		break;
	case RTCAN_SLAVE:
		palTogglePad(LED1_GPIO, LED1);
		chThdSleepMilliseconds(500);
		break;
	case RTCAN_ERROR:
		palTogglePad(LED4_GPIO, LED4);
		chThdSleepMilliseconds(200);
		break;
	default:
		chThdSleepMilliseconds(100);
		break;
	}
#endif
}

/** @} */
