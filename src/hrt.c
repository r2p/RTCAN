#include "rtcan.h"
#include "hrt.h"

#if RTCAN_USE_HRT

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#if RTCAN_HRT_TEST

/* Test static schedules for 3 nodes. */

void hrt_msg(hrt_msg_t * msgp, uint16_t id, uint8_t * data_p, uint16_t size, uint16_t period, uint16_t slot) {
	msgp->next = NULL;
	msgp->status = RTCAN_MSG_READY;
	msgp->id = id;
	msgp->slot = slot;
	msgp->period = period;
	msgp->data = data_p;
	msgp->ptr = data_p;
	msgp->size = size;

	if (msgp->size > RTCAN_FRAME_SIZE) {
		msgp->fragment = msgp->size / RTCAN_FRAME_SIZE;
	} else {
		msgp->fragment = 0;
	}
}

void hrt_test_1x100(RTCANDriver * rtcanp) {
	static hrt_msg_t msg1;
	uint8_t data1[4] = "AAAA";
	uint16_t id = stm32_id8();
	uint16_t offset;

	switch (id) {
	case 40:
		offset = 0;
		rtcanp->reservation_mask[0] = 0x0000000E;
		rtcanp->reservation_mask[1] = 0x00000000;
		break;
	case 82:
		offset = 1;
		break;
	case 54:
		offset = 2;
		break;
	default:
		return;
		break;
	}

	hrt_msg(&msg1, (1 << 8) | id, data1, sizeof(data1), 60, offset + 1);

	calendar_reserve(rtcanp, &msg1);
}

void hrt_test_1x100_big(RTCANDriver * rtcanp) {
	static hrt_msg_t msg1;
	uint8_t data1[20] = "AAAABBBBCCCCDDDDEEEE";
	uint16_t id = stm32_id8();
	uint16_t offset;

	switch (id) {
	case 40:
		offset = 0;
		rtcanp->reservation_mask[0] = 0x000003FE;
		rtcanp->reservation_mask[1] = 0x00000000;
		break;
	case 82:
		offset = 3;
		// XXX
		rtcanp->reservation_mask[0] = 0x000003FE;
		rtcanp->reservation_mask[1] = 0x00000000;
		break;
	case 54:
		offset = 6;
		break;
	default:
		return;
		break;
	}

	hrt_msg(&msg1, (1 << 8) | id, data1, sizeof(data1), 60, offset + 1);

	calendar_reserve(rtcanp, &msg1);
}

void hrt_test_1x1k_1x200_1x100(RTCANDriver * rtcanp) {
	static hrt_msg_t msg1;
	static hrt_msg_t msg2;
	static hrt_msg_t msg3;
	uint8_t data1[4] = "AAAA";
	uint8_t data2[4] = "BBBB";
	uint8_t data3[4] = "CCCC";
	uint16_t id = stm32_id8();
	uint16_t offset;

	switch (id) {
	case 40:
		offset = 0;
		rtcanp->reservation_mask[0] = 0x8e38ffff;
		rtcanp->reservation_mask[1] = 0x038e38ff;
		break;
	case 82:
		offset = 1;
		break;
	case 54:
		offset = 2;
		break;
	default:
		return;
		break;
	}

	hrt_msg(&msg1, (1 << 8) | id, data1, sizeof(data1), 6, offset + 1);
	hrt_msg(&msg2, (2 << 8) | id, data2, sizeof(data2), 30, offset + 4);
	hrt_msg(&msg3, (3 << 8) | id, data3, sizeof(data3), 60, offset + 10);

	calendar_reserve(rtcanp, &msg1);
	calendar_reserve(rtcanp, &msg2);
	calendar_reserve(rtcanp, &msg3);
}


void hrt_test_4x200_4x100(RTCANDriver * rtcanp) {
	static hrt_msg_t msg1;
	static hrt_msg_t msg2;
	static hrt_msg_t msg3;
	static hrt_msg_t msg4;
	static hrt_msg_t msg5;
	static hrt_msg_t msg6;
	static hrt_msg_t msg7;
	static hrt_msg_t msg8;
	uint8_t data1[8] = "AAAAAAAA";
	uint8_t data2[8] = "BBBBBBBB";
	uint8_t data3[4] = "CCCC";
	uint8_t data4[4] = "DDDD";
	uint16_t id = stm32_id8();
	uint16_t offset;

	switch (id) {
	case 40:
		offset = 0;
		rtcanp->reservation_mask[0] = 0x9FE7F9FE;
		rtcanp->reservation_mask[1] = 0x00781E07;
		break;
	case 82:
		offset = 10;
		break;
	case 54:
		offset = 20;
		break;
	default:
		return;
		break;
	}

	hrt_msg(&msg1, (1 << 8) | id, data1, sizeof(data1), 30, offset + 1);
	hrt_msg(&msg2, (2 << 8) | id, data2, sizeof(data2), 30, offset + 2);
	hrt_msg(&msg3, (3 << 8) | id, data3, sizeof(data3), 30, offset + 3);
	hrt_msg(&msg4, (4 << 8) | id, data4, sizeof(data4), 30, offset + 4);
	hrt_msg(&msg5, (5 << 8) | id, data1, sizeof(data1), 60, offset + 5);
	hrt_msg(&msg6, (6 << 8) | id, data2, sizeof(data2), 60, offset + 6);
	hrt_msg(&msg7, (7 << 8) | id, data3, sizeof(data3), 60, offset + 7);
	hrt_msg(&msg8, (8 << 8) | id, data4, sizeof(data4), 60, offset + 8);

	calendar_reserve(rtcanp, &msg1);
	calendar_reserve(rtcanp, &msg2);
	calendar_reserve(rtcanp, &msg3);
	calendar_reserve(rtcanp, &msg4);
	calendar_reserve(rtcanp, &msg5);
	calendar_reserve(rtcanp, &msg6);
	calendar_reserve(rtcanp, &msg7);
	calendar_reserve(rtcanp, &msg8);
}
#endif


#if HRT_CALENDAR_TEST
#include "ch.h"
#include "chprintf.h"

void hrt_calendar_dump(void) {
	hrt_msg_t * msgp = (hrt_msg_t *)&hrt_calendar;
	int i = 0;

	chprintf((BaseSequentialStream *)&SERIAL_DRIVER, "\r\nCALENDAR DUMP at %x\r\n", (uint32_t)&hrt_calendar);

	while (msgp->next != (hrt_msg_t *)&hrt_calendar) {
		msgp = msgp->next;
		chprintf((BaseSequentialStream *)&SERIAL_DRIVER, "hrt_calendar[%d] = %x %d %d %x\r\n", i, (uint32_t)msgp, msgp->period, msgp->slot, (uint32_t)msgp->next);
		i++;
	}
}
#endif /* HRT_CALENDAR_TEST */

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

void calendar_reset(hrt_calendar_t * calp) {
	calp->next = (hrt_msg_t *)calp;
}

void calendar_add(hrt_calendar_t * calp, hrt_msg_t * msgp) {
    hrt_msg_t * curr_msgp = (hrt_msg_t *)calp;

    msgp->slot = -1;

	while (curr_msgp->next != (hrt_msg_t *)calp) {
			if (msgp->period < (curr_msgp->next)->period) {
					break;
			}
			curr_msgp = curr_msgp->next;
	}

	msgp->next = curr_msgp->next;
	curr_msgp->next = msgp;
}

void calendar_reserve(RTCANDriver * rtcanp, hrt_msg_t * msgp) {
	hrt_calendar_t * calp = &(rtcanp->hrt_calendar);
    hrt_msg_t * curr_msgp = (hrt_msg_t *)calp;

	while (curr_msgp->next != (hrt_msg_t *)calp) {
			if (msgp->slot < (curr_msgp->next)->slot) {
					break;
			}
			curr_msgp = curr_msgp->next;
	}

	msgp->next = curr_msgp->next;
	curr_msgp->next = msgp;
}


bool_t calendar_check(hrt_calendar_t * calp, uint16_t slot, uint16_t period) {
    hrt_msg_t * msgp = (hrt_msg_t *)calp;
	uint16_t module = slot % period;

	while (msgp->next != (hrt_msg_t *)calp) {
		msgp = msgp->next;
		if (msgp->slot == 65535) {
			continue;
		}
		if ((period > msgp->period) && (period % msgp->period) != 0) {
			return FALSE;
		}
		if ((period < msgp->period) && (msgp->period % period) != 0) {
			return FALSE;
		}
		if (module % msgp->period == msgp->slot) {
			return FALSE;
		}
	}

	return TRUE;
}

bool_t calendar_set_slot(hrt_calendar_t * calp, hrt_msg_t * msgp) {
	hrt_msg_t * curr_msgp = (hrt_msg_t *)calp;
	int slot = 1;

	/* Empty calendar. */
	if (curr_msgp->next == (hrt_msg_t *)calp) {
		msgp->slot = slot;
		return TRUE;
	}

	while (slot < msgp->period) {
		if (calendar_check(calp, slot, msgp->period)) {
			msgp->slot = slot;
			return TRUE;
		}
		curr_msgp = curr_msgp->next;
		slot++;
	}

	msgp->slot = -1;

	return FALSE;
}

void calendar_prepare(hrt_calendar_t * calp) {
    hrt_msg_t * msgp = (hrt_msg_t *)calp;

	while (msgp->next != (hrt_msg_t *)calp) {
		msgp = msgp->next;
		calendar_set_slot(calp, msgp);
    }
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void hrt_reset(RTCANDriver * rtcanp) {

	calendar_reset(&(rtcanp->hrt_calendar));

	rtcanp->reservation_mask[0] = 0;
	rtcanp->reservation_mask[1] = 0;
}


bool_t hrt_reserved_slot(RTCANDriver * rtcanp) {
	// FIXME: should be independent from slot number configuration
	if (rtcanp->slot < 32) {
		return ((0x01 << rtcanp->slot) & rtcanp->reservation_mask[0]);
	}

	if (rtcanp->slot < 60) {
		return ((0x01 << (rtcanp->slot - 32)) & rtcanp->reservation_mask[1]);
	}

	/* Should never reach */
	return FALSE;
}

void hrt_transmit(RTCANDriver * rtcanp) {
	hrt_msg_t * msgp;
	rtcan_txframe_t txfp;
	uint32_t slot;

	msgp = (hrt_msg_t *)rtcanp->hrt_calendar.next;
	slot = (rtcanp->cycle * rtcanp->config->slots) + rtcanp->slot;

	/* Check for empty local HRT calendar. */
	if (msgp == (hrt_msg_t *)&(rtcanp->hrt_calendar)) {
		return;
	}

	/* Still not my turn. Slot reserved by another node. */
	if (msgp->slot > slot) {
		return;
	}

	/* My slot? */
	if ((slot >= msgp->slot) && (slot < (msgp->slot + msgp->size))) {
		txfp.id = 0x00 | (((msgp->id & 0xFFFF) << 7) | ((msgp->fragment) & 0x7F));
		txfp.data32[0] = *(uint32_t *)msgp->ptr;
		txfp.data32[1] = *((uint32_t *)msgp->ptr + 1);

		/* Handle fragmentation. */
		if (msgp->fragment > 0) {
			txfp.len = RTCAN_FRAME_SIZE;
			rtcan_lld_can_transmit(rtcanp, &txfp);
			msgp->fragment--;
			msgp->ptr += RTCAN_FRAME_SIZE;
			msgp->status = RTCAN_MSG_ONAIR;
			return;
		} else {
			txfp.len = msgp->data + msgp->size - msgp->ptr;
			rtcan_lld_can_transmit(rtcanp, &txfp);
			/* Reset data read pointer. */
			msgp->ptr = msgp->data;
			msgp->status = RTCAN_MSG_READY;
		}
	} else {
		/* Missed slot. */
		msgp->status = RTCAN_MSG_TIMEOUT;
	}

	/* Remove from calendar head. */
	rtcanp->hrt_calendar.next = msgp->next;

	/* Reset fragment counter. */
	if (msgp->size > RTCAN_FRAME_SIZE) {
		msgp->fragment = msgp->size / RTCAN_FRAME_SIZE;
	} else {
		msgp->fragment = 0;
	}

	/* Update deadline and insert back into calendar */
	// TODO: calendar_reserve() is not constant-time. Should be handled outside lock? Is it possible?.
	msgp->slot += msgp->period;
	calendar_reserve(rtcanp, msgp);

	return;
}

#endif /* RTCAN_USE_HRT */
