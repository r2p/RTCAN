#ifndef _RTCAN_LLD_CAN_H_
#define _RTCAN_LLD_CAN_H_

#ifdef STM32F10X
#include "stm32f10x.h"
#endif

#ifdef STM32F30X
#include "stm32f30x.h"
#endif

#ifdef STM32F40X
#include "stm32f4xx.h"
#endif

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * The following macros from the ST header file are replaced with better
 * equivalents.
 */
#undef CAN_BTR_BRP
#undef CAN_BTR_TS1
#undef CAN_BTR_TS2
#undef CAN_BTR_SJW

/**
 * @brief   This implementation supports three transmit mailboxes.
 */
#define CAN_TX_MAILBOXES            3

/**
 * @brief   This implementation supports two receive mailboxes.
 */
#define CAN_RX_MAILBOXES            2



#define RTCAN_STM32_CAN_MAX_FILTERS 28

#define STM32_CAN_FILTER_MODE_ID_MASK     0     /**< @brief Identifier mask mode. */
#define STM32_CAN_FILTER_MODE_ID_LIST     1     /**< @brief Identifier list mode. */
#define STM32_CAN_FILTER_SCALE_16         0     /**< @brief Dual 16-bit scale.    */
#define STM32_CAN_FILTER_SCALE_32         1     /**< @brief Single 32-bit scale.  */
#define RTCAN_STM32_CAN_FILTER_SLOTS_PER_BANK ((1 + RTCAN_STM32_CAN_FILTER_MODE) * (2 - RTCAN_STM32_CAN_FILTER_SCALE))
#define RTCAN_STM32_CAN_FILTER_SLOTS (RTCAN_STM32_CAN_MAX_FILTERS * RTCAN_STM32_CAN_FILTER_SLOTS_PER_BANK)

#define RTCAN_STM32_CAN_FILTER_MODE		STM32_CAN_FILTER_MODE_ID_MASK
#define RTCAN_STM32_CAN_FILTER_SCALE	STM32_CAN_FILTER_SCALE_32


/**
 * @name    CAN registers helper macros
 * @{
 */
#define CAN_BTR_BRP(n)              (n)         /**< @brief BRP field macro.*/
#define CAN_BTR_TS1(n)              ((n) << 16) /**< @brief TS1 field macro.*/
#define CAN_BTR_TS2(n)              ((n) << 20) /**< @brief TS2 field macro.*/
#define CAN_BTR_SJW(n)              ((n) << 24) /**< @brief SJW field macro.*/

#define CAN_IDE_STD                 0           /**< @brief Standard id.    */
#define CAN_IDE_EXT                 1           /**< @brief Extended id.    */

#define CAN_RTR_DATA                0           /**< @brief Data frame.     */
#define CAN_RTR_REMOTE              1           /**< @brief Remote frame.   */
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Low level driver configuration options
 * @{
 */
/**
 * @brief   CAN1 driver enable switch.
 * @details If set to @p TRUE the support for CAN1 is included.
 */
#if !defined(RTCAN_STM32_CAN_USE_CAN1) || defined(__DOXYGEN__)
#define RTCAN_STM32_USE_CAN1                      TRUE
#endif

/**
 * @brief   CAN2 driver enable switch.
 * @details If set to @p TRUE the support for CAN2 is included.
 */
#if !defined(RTCAN_STM32_CAN_USE_CAN2) || defined(__DOXYGEN__)
#define RTCAN_STM32_USE_CAN2                      FALSE
#endif

/**
 * @brief   CAN1 interrupt priority level setting.
 */
#if !defined(RTCAN_STM32_CAN_CAN1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define RTCAN_STM32_CAN_CAN1_IRQ_PRIORITY         5
#endif
/** @} */

/**
 * @brief   CAN2 interrupt priority level setting.
 */
#if !defined(RTCAN_STM32_CAN_CAN2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define RTCAN_STM32_CAN_CAN2_IRQ_PRIORITY         5
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
#if RTCAN_STM32_USE_CAN1 && !defined(__DOXYGEN__)
extern RTCANDriver RTCAND1;
#endif

#if RTCAN_STM32_USE_CAN2 && !defined(__DOXYGEN__)
extern RTCANDriver RTCAND2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
void rtcan_lld_can_init(void);
void rtcan_lld_can_start(RTCANDriver *rtcanp);
void rtcan_lld_can_stop(RTCANDriver *rtcanp);
bool_t rtcan_lld_can_txe(RTCANDriver *rtcanp);
void rtcan_lld_can_transmit(RTCANDriver *rtcanp, rtcan_txframe_t *framep);
bool_t rtcan_lld_can_rxne(RTCANDriver *rtcanp);
void rtcan_lld_can_receive(RTCANDriver *rtcanp, rtcan_rxframe_t *framep);
bool_t rtcan_lld_can_addfilter(RTCANDriver* rtcanp, uint32_t id, uint32_t mask, rtcan_filter_t * filter);
#ifdef __cplusplus
}
#endif

#endif /* _RTCAN_LLD_CAN_H_ */

/** @} */
