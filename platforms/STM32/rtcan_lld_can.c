#include "rtcan.h"
#include "rtcan_lld_can.h"

//FIXME: remove ChibiOS dependency (IRQ numbers and priorities)
#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief RTCAN1 driver identifier.*/
#if RTCAN_STM32_USE_CAN1 || defined(__DOXYGEN__)
RTCANDriver RTCAND1;
#endif

/** @brief RTCAN2 driver identifier.*/
#if RTCAN_STM32_USE_CAN2 || defined(__DOXYGEN__)
RTCANDriver RTCAND2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Common TX ISR handler.
 *
 * @param[in] rtcanp      pointer to the @p RTCANDriver object
 *
 * @notapi
 */
static void rtcan_lld_can_tx_handler(RTCANDriver *rtcanp) {
	CAN_TypeDef * can = rtcanp->can;
	uint32_t tsr;

	tsr = can->TSR;

	/* No more events until a message is transmitted.*/
	can->TSR = CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2;

	if (tsr & CAN_TSR_TXOK0) {
		rtcan_txok_isr_code(rtcanp, 0);
	}

	if (tsr & CAN_TSR_TXOK1) {
		rtcan_txok_isr_code(rtcanp, 1);
	}

	if (tsr & CAN_TSR_TXOK2) {
		rtcan_txok_isr_code(rtcanp, 2);
	}

	if (tsr & CAN_TSR_ALST0) {
		rtcan_alst_isr_code(rtcanp, 0);
	}

	if (tsr & CAN_TSR_ALST1) {
		rtcan_alst_isr_code(rtcanp, 1);
	}

	if (tsr & CAN_TSR_ALST2) {
		rtcan_alst_isr_code(rtcanp, 2);
	}

	if (tsr & CAN_TSR_TERR0) {
		rtcan_terr_isr_code(rtcanp, 0);
	}

	if (tsr & CAN_TSR_TERR1) {
		rtcan_terr_isr_code(rtcanp, 1);
	}

	if (tsr & CAN_TSR_TERR2) {
		rtcan_terr_isr_code(rtcanp, 2);
	}
}

/**
 * @brief   Common RX0 ISR handler.
 *
 * @param[in] rtcanp      pointer to the @p RTCANDriver object
 *
 * @notapi
 */
static void rtcan_lld_can_rx0_handler(RTCANDriver *rtcanp) {
	uint32_t rf0r;

	rf0r = rtcanp->can->RF0R;
	if ((rf0r & CAN_RF0R_FMP0 )> 0){
		/* No more receive events until the queue 0 has been emptied.*/
		rtcanp->can->IER &= ~CAN_IER_FMPIE0;

		/* RTCAN RX0 ISR */
		rtcan_rx_isr_code(rtcanp);
	}
	if ((rf0r & CAN_RF0R_FOVR0 )> 0){
		/* Overflow events handling.*/
		rtcanp->can->RF0R = CAN_RF0R_FOVR0;

		/* RTCAN RX0 OVERFLOW ISR */
	}
}

/**
 * @brief   Common RX1 ISR handler.
 *
 * @param[in] rtcanp      pointer to the @p RTCANDriver object
 *
 * @notapi
 */
static void rtcan_lld_can_rx1_handler(RTCANDriver *rtcanp) {
	uint32_t rf1r;

	rf1r = rtcanp->can->RF1R;
	if ((rf1r & CAN_RF1R_FMP1 )> 0){
		/* No more receive events until the queue 0 has been emptied.*/
		rtcanp->can->IER &= ~CAN_IER_FMPIE1;

		/* RTCAN RX1 ISR */
	}
	if ((rf1r & CAN_RF1R_FOVR1 )> 0){
		/* Overflow events handling.*/
		rtcanp->can->RF1R = CAN_RF1R_FOVR1;

		/* RTCAN RX1 OVERFLOW ISR */
	}
}

/**
 * @brief   Common SCE ISR handler.
 *
 * @param[in] rtcanp      pointer to the @p RTCANDriver object
 *
 * @notapi
 */
static void rtcan_lld_can_sce_handler(RTCANDriver *rtcanp) {
	uint32_t msr;

	msr = rtcanp->can->MSR;
	rtcanp->can->MSR = CAN_MSR_ERRI | CAN_MSR_WKUI | CAN_MSR_SLAKI;

	/* Error event.*/
	if (msr & CAN_MSR_ERRI ) {
		uint32_t flags;
		uint32_t esr = rtcanp->can->ESR;

		rtcanp->can->ESR &= ~CAN_ESR_LEC;
		flags = (flagsmask_t)(esr & 7);
		if ((esr & CAN_ESR_LEC )> 0)flags
			|= CAN_FRAMING_ERROR;

		/* RTCAN ERROR ISR */
	}
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if RTCAN_STM32_USE_CAN1 || defined(__DOXYGEN__)
/**
 * @brief   CAN1 TX interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_CAN1_TX_HANDLER) {

	CH_IRQ_PROLOGUE();

	rtcan_lld_can_tx_handler(&RTCAND1);

	CH_IRQ_EPILOGUE();
}

/*
 * @brief   CAN1 RX0 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_CAN1_RX0_HANDLER) {

	CH_IRQ_PROLOGUE();

	rtcan_lld_can_rx0_handler(&RTCAND1);

	CH_IRQ_EPILOGUE();
}

/**
 * @brief   CAN1 RX1 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_CAN1_RX1_HANDLER) {

	CH_IRQ_PROLOGUE();

	rtcan_lld_can_rx1_handler(&RTCAND1);

	CH_IRQ_EPILOGUE();
}

/**
 * @brief   CAN1 SCE interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_CAN1_SCE_HANDLER) {

	CH_IRQ_PROLOGUE();

	rtcan_lld_can_sce_handler(&RTCAND1);

	CH_IRQ_EPILOGUE();
}
#endif /* RTCAN_STM32_USE_CAN1 */

#if RTCAN_STM32_USE_CAN2 || defined(__DOXYGEN__)
/**
 * @brief   CAN2 TX interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_CAN2_TX_HANDLER) {

	CH_IRQ_PROLOGUE();

	rtcan_lld_can_tx_handler(&RTCAND2);

	CH_IRQ_EPILOGUE();
}

/*
 * @brief   CAN2 RX0 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_CAN2_RX0_HANDLER) {

	CH_IRQ_PROLOGUE();

	rtcan_lld_can_rx0_handler(&RTCAND2);

	CH_IRQ_EPILOGUE();
}

/**
 * @brief   CAN2 RX1 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_CAN2_RX1_HANDLER) {

	CH_IRQ_PROLOGUE();

	rtcan_lld_can_rx1_handler(&RTCAND2);

	CH_IRQ_EPILOGUE();
}

/**
 * @brief   CAN2 SCE interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_CAN2_SCE_HANDLER) {

	CH_IRQ_PROLOGUE();

	rtcan_lld_can_sce_handler(&RTCAND2);

	CH_IRQ_EPILOGUE();
}
#endif /* RTCAN_STM32_USE_CAN2 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level CAN driver initialization.
 *
 * @notapi
 */
void rtcan_lld_can_init(void) {

#if RTCAN_STM32_USE_CAN1
	RTCAND1.can = CAN1;
	rtcanReset(&RTCAND1);
#endif
#if RTCAN_STM32_USE_CAN2
	RTCAND2.can = CAN2;
	rtcanReset(&RTCAND2);
#endif

}

/**
 * @brief   Configures and activates the CAN peripheral.
 *
 * @param[in] rtcanp      pointer to the @p RTCANDriver object
 *
 * @notapi
 */
void rtcan_lld_can_start(RTCANDriver *rtcanp) {

	/* Interrupts activation.*/
	nvicEnableVector(STM32_CAN1_TX_NUMBER,
			CORTEX_PRIORITY_MASK(STM32_CAN_CAN1_IRQ_PRIORITY));
	nvicEnableVector(STM32_CAN1_RX0_NUMBER,
			CORTEX_PRIORITY_MASK(STM32_CAN_CAN1_IRQ_PRIORITY));
	nvicEnableVector(STM32_CAN1_RX1_NUMBER,
			CORTEX_PRIORITY_MASK(STM32_CAN_CAN1_IRQ_PRIORITY));
	nvicEnableVector(STM32_CAN1_SCE_NUMBER,
			CORTEX_PRIORITY_MASK(STM32_CAN_CAN1_IRQ_PRIORITY));
	/* Clock activation.*/
	rccEnableCAN1(FALSE);

	/* Entering initialization mode. */
	rtcanp->state = RTCAN_STARTING;
	rtcanp->can->MCR = CAN_MCR_INRQ;
	while ((rtcanp->can->MSR & CAN_MSR_INAK )== 0);

	/* BTR initialization.*/
	/* TODO: to calculate from bitrate configuration */
#ifdef STM32F4XX
	rtcanp->can->BTR = CAN_BTR_SJW(0) | CAN_BTR_TS2(4) | CAN_BTR_TS1(7)
				| CAN_BTR_BRP(2);
#endif

#ifdef STM32F30X
	rtcanp->can->BTR = CAN_BTR_SJW(0) | CAN_BTR_TS2(2) | CAN_BTR_TS1(4)
				| CAN_BTR_BRP(3);
#endif

#ifdef STM32F10X_MD
	rtcanp->can->BTR = CAN_BTR_SJW(0) | CAN_BTR_TS2(2) | CAN_BTR_TS1(4)
				| CAN_BTR_BRP(3);
#endif

	/* MCR initialization.*/
	rtcanp->can->MCR = CAN_MCR_NART | CAN_MCR_TTCM;

	/* Interrupt sources initialization.*/
	rtcanp->can->IER = CAN_IER_TMEIE | CAN_IER_FMPIE0 | CAN_IER_FMPIE1
			| CAN_IER_WKUIE | CAN_IER_ERRIE | CAN_IER_LECIE | CAN_IER_BOFIE
			| CAN_IER_EPVIE | CAN_IER_EWGIE | CAN_IER_FOVIE0 | CAN_IER_FOVIE1;
}

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] rtcanp      pointer to the @p RTCANDriver object
 *
 * @notapi
 */
void rtcan_lld_can_stop(RTCANDriver *rtcanp) {

	/* If in ready state then disables the CAN peripheral.*/
	if ((rtcanp->state == RTCAN_MASTER) || (rtcanp->state == RTCAN_SLAVE)) {
#if RTCAN_STM32_USE_CAN1
		if (&RTCAND1 == rtcanp) {

#if RTCAN_STM32_USE_CAN2
			chDbgAssert(RTCAND2.state == CAN_STOP,
					"rtcan_lld_can_stop(), #1", "RTCAN2 must be stopped");
#endif

			CAN1->MCR = 0x00010002; /* Register reset value.    */
			CAN1->IER = 0x00000000; /* All sources disabled.    */
			nvicDisableVector(STM32_CAN1_TX_NUMBER);
			nvicDisableVector(STM32_CAN1_RX0_NUMBER);
			nvicDisableVector(STM32_CAN1_RX1_NUMBER);
			nvicDisableVector(STM32_CAN1_SCE_NUMBER);
			rccDisableCAN1(FALSE);
		}
#endif
#if RTCAN_STM32_USE_CAN2
		if (&RTCAND2 == rtcanp) {
			CAN2->MCR = 0x00010002; /* Register reset value.    */
			CAN2->IER = 0x00000000; /* All sources disabled.    */
			nvicDisableVector(STM32_CAN2_TX_NUMBER);
			nvicDisableVector(STM32_CAN2_RX0_NUMBER);
			nvicDisableVector(STM32_CAN2_RX1_NUMBER);
			nvicDisableVector(STM32_CAN2_SCE_NUMBER);
			rccDisableCAN2(FALSE);
		}
#endif
	}
}

/**
 * @brief   Determines whether a frame can be transmitted.
 *
 * @param[in] rtcanp      pointer to the @p RTCANDriver object
 *
 * @return              The queue space availability.
 * @retval FALSE        no space in the transmit queue.
 * @retval TRUE         transmit slot available.
 *
 * @notapi
 */
bool_t rtcan_lld_can_txe(RTCANDriver *rtcanp) {

//	return (rtcanp->can->TSR & CAN_TSR_TME ) != 0;
	return (rtcanp->can->TSR & CAN_TSR_TME ) == CAN_TSR_TME;  // XXX WORKAROUND
}

/**
 * @brief   Inserts a frame into the transmit queue.
 *
 * @param[in] rtcanp    pointer to the @p RTCANDriver object
 * @param[in] txfp      pointer to the CAN frame to be transmitted
 *
 * @notapi
 */
void rtcan_lld_can_transmit(RTCANDriver * rtcanp, rtcan_txframe_t * txfp) {
	CAN_TypeDef * can = rtcanp->can;
	CAN_TxMailBox_TypeDef *tmbp;

//	txfp->mbox = (rtcanp->can->TSR & CAN_TSR_CODE ) >> 24;
	txfp->mbox = 0; // XXX WORKAROUND
	tmbp = &can->sTxMailBox[txfp->mbox];

	tmbp->TIR =	(txfp->id << 3) | CAN_TI0R_IDE;
	tmbp->TDTR = txfp->len;

	tmbp->TDLR = txfp->data32[0];
	tmbp->TDHR = txfp->data32[1];
	tmbp->TIR |= CAN_TI0R_TXRQ;
}

/**
 * @brief   Determines whether a frame has been received.
 *
 * @param[in] rtcanp      pointer to the @p RTCANDriver object
 *
 * @return              The queue space availability.
 * @retval FALSE        no space in the transmit queue.
 * @retval TRUE         transmit slot available.
 *
 * @notapi
 */
bool_t rtcan_lld_can_rxne(RTCANDriver * rtcanp) {

	return ((rtcanp->can->RF0R & CAN_RF0R_FMP0 )!= 0 || (rtcanp->can->RF1R & CAN_RF1R_FMP1) != 0);
}

/**
 * @brief   Receives a frame from the input queue.
 *
 * @param[in] rtcanp      pointer to the @p RTCANDriver object
 * @param[out] rxfp       pointer to the buffer where the CAN frame is stored
 *
 * @notapi
 */
void rtcan_lld_can_receive(RTCANDriver *rtcanp, rtcan_rxframe_t *rxfp) {
	CAN_TypeDef * can = rtcanp->can;

	if ((can->RF0R & CAN_RF0R_FMP0 )!= 0){
		/* Fetches the frame.*/
		rxfp->id = can->sFIFOMailBox[0].RIR >> 3;
		rxfp->len = can->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC;
		rxfp->data32[0] = can->sFIFOMailBox[0].RDLR;
		rxfp->data32[1] = can->sFIFOMailBox[0].RDHR;
		rxfp->filter = (uint8_t)(can->sFIFOMailBox[0].RDTR >> 8);

		/* Releases the mailbox.*/
		can->RF0R = CAN_RF0R_RFOM0;

		/* If the queue is empty re-enables the interrupt in order to generate
		 events again.*/
		if ((can->RF0R & CAN_RF0R_FMP0 )== 0){
			can->IER |= CAN_IER_FMPIE0;
		}
	} else if ((can->RF1R & CAN_RF1R_FMP1 )!= 0){
		/* Fetches the message.*/
		rxfp->id = can->sFIFOMailBox[1].RIR >> 3;
		rxfp->len = can->sFIFOMailBox[1].RDTR & CAN_RDT0R_DLC;
		rxfp->data32[0] = can->sFIFOMailBox[1].RDLR;
		rxfp->data32[1] = can->sFIFOMailBox[1].RDHR;
		rxfp->filter = (uint8_t)(can->sFIFOMailBox[1].RDTR >> 8);

		/* Releases the mailbox.*/
		can->RF1R = CAN_RF1R_RFOM1;

		/* If the queue is empty re-enables the interrupt in order to generate
		 events again.*/
		if ((can->RF1R & CAN_RF1R_FMP1 )== 0){
			can->IER |= CAN_IER_FMPIE1;
		}
	}
}

/**
 * @brief   Adds a hardware filter to the CAN cotroller.
 *
 * @param[in] rtcanp      pointer to the @p RTCANDriver object
 * @param[in] id          the CAN ID to be accepted
 * @param[in] mask        the mask to apply to the filter
 * @param[out] filter     pointer to the buffer where the filter index is stored
 *
 * @return              The operation result.
 * @retval FALSE        no hardware filter available.
 * @retval TRUE         hardware filter added.

 * @notapi
 */
bool_t rtcan_lld_can_addfilter(RTCANDriver* rtcanp, uint32_t id, uint32_t mask, rtcan_filter_t * filter) {
	CAN_TypeDef * can = rtcanp->can;
	CAN_FilterRegister_TypeDef * cfp;
	static uint32_t next_fid = 0;
	uint32_t fmask;

	if (next_fid < RTCAN_STM32_CAN_FILTER_SLOTS) {
		cfp = &(can->sFilterRegister[next_fid
				/ RTCAN_STM32_CAN_FILTER_SLOTS_PER_BANK]);
		fmask = (1 << (next_fid / RTCAN_STM32_CAN_FILTER_SLOTS_PER_BANK));
		can->FMR |= CAN_FMR_FINIT;
		can->FA1R &= ~fmask;

		// XXX tutto da sistemare!!!
		if (RTCAN_STM32_CAN_FILTER_MODE == STM32_CAN_FILTER_MODE_ID_MASK) {
			can->FM1R &= ~fmask;
			if (RTCAN_STM32_CAN_FILTER_SCALE == STM32_CAN_FILTER_SCALE_16) {
				can->FS1R &= ~fmask;
				id = ((id & 0x1FFC0000) >> 13) | 0x08 | (id & 0x00038000) >> 15;
				mask = (mask & 0x0000FFFF) << 16;
				if (next_fid % 2)
					cfp->FR2 = id | mask;
				else
					cfp->FR1 = id | mask;
			} else {
				can->FS1R |= fmask;
				cfp->FR1 = id << 3;
				cfp->FR2 = mask << 3;
			}
		} else {
			can->FM1R |= fmask;
			if (RTCAN_STM32_CAN_FILTER_SCALE == STM32_CAN_FILTER_SCALE_16) {
				can->FS1R &= ~fmask;
				uint16_t *tmp16 = (uint16_t *) cfp;
				tmp16 = tmp16 + (next_fid % 4);
				*tmp16 = (uint16_t)(id & 0x0000FFFF);
			} else {
				can->FS1R |= fmask;
				if (next_fid % 2)
					cfp->FR2 = id;
				else
					cfp->FR1 = id;
			}
		}

		can->FA1R |= fmask;
		can->FMR &= ~CAN_FMR_FINIT;

		*filter = next_fid++;
		return TRUE;
	} else {
		return FALSE;
	}
}

/** @} */
