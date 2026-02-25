/*
 * MpptChg.h
 *
 *  Created on: Dec 8, 2025
 *      Author: Ruan de Jager
 */

#ifndef WORKER_MPPTCHG_MPPTCHG_H_
#define WORKER_MPPTCHG_MPPTCHG_H_

#include <stdint.h>

//! Charge state definition
typedef enum chg_mppt_state_t {
	CHG_SEL_MPPT_OFF=0,
	CHG_SEL_MPPT_15mA,
	CHG_SEL_MPPT_20mA,
	CHG_SEL_MPPT_25mA,
	CHG_SEL_MPPT_30mA,
	CHG_SEL_MPPT_35mA,
	CHG_SEL_MPPT_40mA
} chg_mppt_state_t;

typedef enum chg_mppt_bump_t {
	CHG_BUMP_MPPT_DOWN=0,
	CHG_BUMP_MPPT_UP,
} chg_mppt_bump_t;

//! State definitions for PG line
typedef enum chg_pg_state_t {
	CHG_PG_STATE_UNKNOWN=0,		// At startup
	CHG_PG_STATE_LOW,
	CHG_PG_STATE_HIGH,
	CHG_PG_STATE_UNSTABLE
} chg_pg_state_t;

void MPPTCHG_vInit(void);

void MPPTCHG_vPinChangeCallback(void);

void MPPTCHG_vIncMpptStateCounters(void);
uint32_t MPPTCHG_u32GetOffMpptCounter(void);
uint32_t MPPTCHG_u32Get15mAMpptCounter(void);
uint32_t MPPTCHG_u32Get20mAMpptCounter(void);
uint32_t MPPTCHG_u32Get25mAMpptCounter(void);
uint32_t MPPTCHG_u32Get30mAMpptCounter(void);
uint32_t MPPTCHG_u32Get35mAMpptCounter(void);
uint32_t MPPTCHG_u32Get40mAMpptCounter(void);
void MPPTCHG_vClearMpptStateCounters(void);

#endif /* WORKER_MPPTCHG_MPPTCHG_H_ */
