/**
 * @file    STM32/rtcan_lld_tim.c
 * @brief   STM32 low level timer driver source.
 *
 * @addtogroup TIM
 * @{
 */

#include "rtcan.h"
#include "rtcan_lld_tim.h"

//FIXME: remove ChibiOS dependency (IRQ numbers and priorities)
#include "hal.h"

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

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/**
 * @brief   TIM3 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(STM32_TIM3_HANDLER) {

  CH_IRQ_PROLOGUE();

  TIM3->SR = 0;
  rtcan_tim_isr_code(&RTCAND1);

  CH_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level timer driver initialization.
 *
 * @notapi
 */
void rtcan_lld_tim_init() {

#if RTCAN_STM32_USE_CAN1
	RTCAND1.tim = RTCAN_STM32_RTCAND1_TIM;
#endif

#if RTCAN_STM32_USE_CAN2
	RTCAND2.tim = RTCAN_STM32_RTCAND2_TIM;
#endif
}

/**
 * @brief   Configures and activates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void rtcan_lld_tim_start(RTCANDriver * rtcanp) {
	TIM_TypeDef * tim = rtcanp->tim;
	uint32_t clock;
	uint16_t psc;

	/* Interrupts activation.*/
	// FIXME: need a macro to get IRQ numbers from the used timer
#if RTCAN_STM32_USE_CAN1
	rccEnableTIM3(FALSE);
	rccResetTIM3();
	nvicEnableVector(STM32_TIM3_NUMBER, CORTEX_PRIORITY_MASK(STM32_GPT_TIM3_IRQ_PRIORITY));
#endif

#if RTCAN_STM32_USE_CAN2
	rccEnableTIM4(FALSE);
	rccResetTIM4();
	nvicEnableVector(STM32_TIM4_NUMBER, CORTEX_PRIORITY_MASK(STM32_GPT_TIM4_IRQ_PRIORITY));
#endif

#ifdef STM32F4XX
	// FIXME: need a macro
	clock = 42000000 * 2;
#else
	// FIXME: need a macro
	clock = 36000000 * 2;
#endif
	/* Prescaler value calculation.*/
	psc = (uint16_t)((clock / rtcanp->config->baudrate) - 1);

	/* Timer configuration.*/
	tim->CR1  = 0;
	tim->PSC  = psc;
	tim->DIER = 0;
}

/**
 * @brief   Deactivates the GPT peripheral.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void rtcan_lld_tim_stop(RTCANDriver * rtcanp) {
	TIM_TypeDef * tim = rtcanp->tim;

	tim->CR1  = 0;
	tim->DIER = 0;
	tim->SR   = 0;

	/* Interrupts deactivation.*/
	// FIXME: need a macro to get IRQ numbers from the used timer
#if RTCAN_STM32_USE_CAN1
	nvicDisableVector(STM32_TIM3_NUMBER);
	rccDisableTIM3(FALSE);
#endif

#if RTCAN_STM32_USE_CAN2
	nvicDisableVector(STM32_TIM4_NUMBER);
	rccDisableTIM4(FALSE);
#endif
}

/**
 * @brief   Starts the timer in continuous mode.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 * @param[in] interval  period in ticks
 *
 * @notapi
 */
void rtcan_lld_tim_start_timer(RTCANDriver * rtcanp) {
	TIM_TypeDef * tim = rtcanp->tim;
	tim->EGR  = TIM_EGR_UG;             /* Update event.                */
	tim->CNT  = 0;                      /* Reset counter.               */
	tim->SR   = 0;                      /* Clear pending IRQs (if any). */
	tim->DIER = TIM_DIER_UIE;           /* Update Event IRQ enabled.    */
	tim->CR1  = TIM_CR1_URS | TIM_CR1_CEN;
}

/**
 * @brief   Stops the timer.
 *
 * @param[in] gptp      pointer to the @p GPTDriver object
 *
 * @notapi
 */
void rtcan_lld_tim_stop_timer(RTCANDriver * rtcanp) {
	TIM_TypeDef * tim = rtcanp->tim;

	tim->CR1  = 0;
	tim->SR   = 0;
	tim->DIER = 0;
}


void rtcan_lld_tim_set_interval(RTCANDriver * rtcanp, rtcan_cnt_t interval) {
	TIM_TypeDef * tim = rtcanp->tim;

	tim->ARR = interval - 1;
}


rtcan_cnt_t rtcan_lld_tim_get_interval(RTCANDriver * rtcanp) {
	TIM_TypeDef * tim = rtcanp->tim;

	return tim->ARR + 1;
}

rtcan_cnt_t rtcan_lld_tim_get_counter(RTCANDriver * rtcanp) {
	TIM_TypeDef * tim = rtcanp->tim;

	return tim->CNT;
}

void rtcan_lld_tim_set_counter(RTCANDriver * rtcanp, rtcan_cnt_t cnt) {
	TIM_TypeDef * tim = rtcanp->tim;

	tim->CNT = cnt;
}


/** @} */
#endif
