/**
 * @file    rtcan.c
 * @brief   RTCAN message queue code.
 *
 * @addtogroup RTCAN
 * @{
 */
#include "rtcan.h"
#include "msgqueue.h"

void msgqueue_init(msgqueue_t * queuep) {
	queuep->next = (rtcan_msg_t *)(void *)queuep;
}


void msgqueue_insert(msgqueue_t * queuep, rtcan_msg_t * msgp) {
	rtcan_msg_t * curr_msgp = (rtcan_msg_t *)queuep;

	rtcanDbgCheck((queuep != NULL) && (msgp != NULL), "msgqueue_insert");

	/* Deadline based list insert. */
	while (curr_msgp->next != (rtcan_msg_t *)queuep) {
		if (curr_msgp->deadline > msgp->deadline) {
			break;
		}
		curr_msgp = curr_msgp->next;
	}

	msgp->next = curr_msgp->next;
	curr_msgp->next = msgp;
}


void msgqueue_remove(msgqueue_t * queuep, rtcan_msg_t * msgp) {
	rtcan_msg_t * curr_msgp;

	rtcanDbgCheck((queuep != NULL) && (msgp != NULL), "msgqueue_remove");
	rtcanDbgAssert(!msgqueue_isempty((msgqueue_t *) queuep),
			"msgqueue_remove(), #1", "queue is empty");

	curr_msgp = (rtcan_msg_t *)queuep;

	while (curr_msgp->next != msgp) {
		curr_msgp = curr_msgp->next;
	}

	curr_msgp->next = msgp->next;
}

rtcan_msg_t * msgqueue_get(msgqueue_t * queuep) {
	rtcan_msg_t * msgp;

	rtcanDbgCheck(queuep != NULL, "msgqueue_get");
	rtcanDbgAssert(!msgqueue_isempty((msgqueue_t *) queuep),
			"msgqueue_get(), #1", "queue is empty");

	msgp = queuep->next;
	queuep->next = msgp->next;

	return msgp;
}
