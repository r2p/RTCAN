#ifndef _RTCAN_H_
#define _RTCAN_H_

#include <stdint.h>

#include "board.h"
#include "ch.h"

#ifdef STM32F10X_MD
#include "stm32f10x.h"
#endif

#ifdef STM32F30X
#include "stm32f30x.h"
#endif

#ifdef STM32F40_41xxx
#include "stm32f4xx.h"
#endif

//#define RTCAN_TEST

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/
//FIXME vanno nell'lld
#define RTCAN_FRAME_SIZE	8
#define RTCAN_MBOX_NUM		3
#define RTCAN_FILTERS_NUM	14

/**
 * @name    CAN status flags
 * @{
 */
/**
 * @brief   Errors rate warning.
 */
#define CAN_LIMIT_WARNING           1
/**
 * @brief   Errors rate error.
 */
#define CAN_LIMIT_ERROR             2
/**
 * @brief   Bus off condition reached.
 */
#define CAN_BUS_OFF_ERROR           4
/**
 * @brief   Framing error of some kind on the CAN bus.
 */
#define CAN_FRAMING_ERROR           8
/**
 * @brief   Overflow in receive queue.
 */
#define CAN_OVERFLOW_ERROR          16
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   RTCAN HRT messaging enable switch.
 * @details If set to @p TRUE the support for HRT messaging is included.
 */
#if !defined(RTCAN_USE_HRT) || defined(__DOXYGEN__)
#define RTCAN_USE_HRT                  0
#endif

/**
 * @brief   RTCAN SRT messaging enable switch.
 * @details If set to @p TRUE the support for SRT/NRT messaging is included.
 */
#if !defined(RTCAN_USE_SRT) || defined(__DOXYGEN__)
#define RTCAN_USE_SRT                  1
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   RTCAN message type forward declaration.
 */
typedef struct rtcan_msg_t rtcan_msg_t;

/**
 * @brief   RTCAN message callback type.
 */
typedef void (*rtcan_msgcallback_t)(rtcan_msg_t* msg_p);

/**
 * @brief   RTCAN state machine possible states.
 */
typedef enum {
	RTCAN_UNINIT = 0,
	RTCAN_STOP = 1,
	RTCAN_STARTING = 2,
	RTCAN_ERROR = 3,
	RTCAN_SYNCING = 4,
	RTCAN_SLAVE = 5,
	RTCAN_MASTER = 6,
} rtcanstate_t;

/**
 * @brief   RTCAN message status.
 */
typedef enum {
	RTCAN_MSG_UNINIT = 0,
	RTCAN_MSG_BUSY = 1,
	RTCAN_MSG_READY = 2,
	RTCAN_MSG_QUEUED = 3,
	RTCAN_MSG_ONAIR = 4,
	RTCAN_MSG_TIMEOUT = 5,
	RTCAN_MSG_ERROR = 6,
} rtcan_msgstatus_t;


/**
 * @brief   RTCAN message ID type.
 */
typedef uint16_t rtcan_id_t;

/**
 * @brief   CAN transmit mailbox type.
 */
typedef uint8_t rtcan_mbox_t;

/**
 * @brief   CAN receive filter type.
 */
typedef uint8_t rtcan_filter_t;

typedef uint16_t rtcan_cnt_t;

typedef struct {
	uint32_t sec;
	uint32_t nsec;
} rtcan_time_t;

#include "msgqueue.h"
#include "hrt.h"
#include "srt.h"

/**
 * @brief   CAN transmit frame type.
 */
typedef struct {
	uint32_t		id:29;
	uint8_t			len:4;
	uint8_t			mbox;
	union {
		uint8_t		data8[8];
		uint16_t	data16[4];
		uint32_t	data32[2];
	};
} rtcan_txframe_t;

/**
 * @brief   CAN receive frame type.
 */
typedef struct {
	uint32_t		id:29;
	uint8_t			len:4;
	uint8_t			filter;
	union {
		uint8_t		data8[8];
		uint16_t	data16[4];
		uint32_t	data32[2];
	};
} rtcan_rxframe_t;

/**
 * @brief   RTCAN SRT message type.
 */
struct __attribute__ ((__packed__)) rtcan_msg_t {
	rtcan_msg_t *next;
	rtcan_msgstatus_t status;
	rtcan_msgcallback_t callback;
	void * params;
	union {
		struct {
			uint32_t deadline;
		};
		struct {
			uint16_t slot;
			uint16_t period;
		};
	};
	rtcan_id_t id;
	uint16_t size;
	const uint8_t *data;
	uint8_t *ptr;
	uint8_t fragment :7;
};

/**
 * @brief   RTCAN configuration structure.
 */
typedef struct {
	/**
	 * @brief RTCAN baudrate.
	 */
	uint32_t baudrate;
	/**
	 * @brief RTCAN sync frequency.
	 */
	uint32_t clock;
	/**
	 * @brief Number of time-slots in each cycle.
	 */
	uint32_t slots;
} RTCANConfig;

/**
 * @brief   Structure representing a RTCAN driver.
 */
typedef struct RTCANDriver {
	/**
	 * @brief RTCAN state.
	 */
	rtcanstate_t state;
	/**
	 * @brief Time-slot counter.
	 */
	uint32_t slot;
	/**
	 * @brief Cycle counter.
	 */
	uint32_t cycle;
	/**
	 * @brief SRT message queue.
	 */
	msgqueue_t srt_queue;
	/**
	 * @brief On-air messages.
	 */
	rtcan_msg_t * onair[RTCAN_MBOX_NUM];
	/**
	 * @brief On-air messages.
	 */
	rtcan_msg_t * filters[RTCAN_FILTERS_NUM];
#if RTCAN_USE_HRT
	/**
	 * @brief HRT messages reservation mask.
	 */
	hrt_calendar_t hrt_calendar;
	/**
	 * @brief HRT messages reservation mask.
	 */
	uint32_t reservation_mask[2];
#endif /* RTCAN_USE_HRT */
	/**
	 * @brief Current configuration data.
	 */
	const RTCANConfig *config;
	/**
	 * @brief   Pointer to the TIM registers.
	 */
	/* TODO: typedef in lld */
	TIM_TypeDef *tim;
	/**
	 * @brief   Pointer to the CAN registers.
	 */
	/* TODO: typedef in lld */
	CAN_TypeDef *can;
} RTCANDriver;

#include "rtcan_lld_can.h"
#include "rtcan_lld_tim.h"


/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

#ifdef _CHIBIOS_RT_
#define rtcanLock			chSysLock
#define rtcanUnlock			chSysUnlock
#define rtcanLockFromIsr	chSysLockFromIsr
#define rtcanUnlockFromIsr	chSysUnlockFromIsr
#define rtcanDbgCheck		chDbgCheck
#define rtcanDbgAssert		chDbgAssert
#else
/* TODO: implement DbgCheck and DbgAssert */
#endif

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
uint8_t stm32_id8(void);
void rtcan_tim_isr_code(RTCANDriver * rtcanp);
void rtcan_txok_isr_code(RTCANDriver * rtcanp, rtcan_mbox_t mbox);
void rtcan_alst_isr_code(RTCANDriver * rtcanp, rtcan_mbox_t mbox);
void rtcan_terr_isr_code(RTCANDriver * rtcanp, rtcan_mbox_t mbox);
void rtcan_rx_isr_code(RTCANDriver * rtcanp);
void rtcanInit(void);
void rtcanReset(RTCANDriver * canp);
void rtcanStart(RTCANDriver * rtcanp, const RTCANConfig * config);
void rtcanStop(RTCANDriver * canp);
void rtcanTransmit(RTCANDriver * rtcanp, rtcan_msg_t *msgp, uint32_t timeout);
void rtcanTransmitI(RTCANDriver * rtcanp, rtcan_msg_t *msgp, uint32_t timeout);
void rtcanReceive(RTCANDriver * rtcanp, rtcan_msg_t *msgp);
void rtcanReceiveMask(RTCANDriver * rtcanp, rtcan_msg_t *msgp, uint32_t mask);
void rtcanGetTime(RTCANDriver * rtcanp, rtcan_time_t * timep);
void rtcan_blinker(void);
#ifdef __cplusplus
}
#endif

#endif /* _RTCAN_H_ */

/** @} */
