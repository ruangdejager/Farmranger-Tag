/*
 * MpptChg_Driver.h
 *
 *  Created on: Dec 8, 2025
 *      Author: Ruan de Jager
 */

#ifndef WORKER_MPPTCHG_MPPTCHG_DRIVER_H_
#define WORKER_MPPTCHG_MPPTCHG_DRIVER_H_

#include "hal_gpio.h"
#include "hal_bsp.h"
#include "hal_exti.h"
/*
 * Init GPIO pins for charger interface (PG)
 */
#define MPPTCHG_DRIVER_vInitGpio() \
{\
	HAL_GPIO_vInitIntPullup(BSP_CHG_nPOWER_GOOD_PORT, BSP_CHG_nPOWER_GOOD_PIN);\
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);\
}

/*
 * De-init GPIO pins for charger interface (PG)
 */
#define MPPTCHG_DRIVER_vDeinitGpio() \
{\
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);\
	HAL_GPIO_DeInit(BSP_CHG_nPOWER_GOOD_PORT, BSP_CHG_nPOWER_GOOD_PIN);\
}

#define MPPTCHG_DRIVER_vSetMppt5mA()                { \
		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_0_Port, BSP_CHG_CURRENT_SEL_0_Pin);\
		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_1_Port, BSP_CHG_CURRENT_SEL_1_Pin);\
		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_2_Port, BSP_CHG_CURRENT_SEL_2_Pin);\
}
#define MPPTCHG_DRIVER_vSetMppt10mA()                { \
		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_0_Port, BSP_CHG_CURRENT_SEL_0_Pin, GPIO_PIN_RESET);\
		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_1_Port, BSP_CHG_CURRENT_SEL_1_Pin);\
		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_2_Port, BSP_CHG_CURRENT_SEL_2_Pin);\
}
#define MPPTCHG_DRIVER_vSetMppt15mA()                { \
		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_0_Port, BSP_CHG_CURRENT_SEL_0_Pin);\
		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_1_Port, BSP_CHG_CURRENT_SEL_1_Pin, GPIO_PIN_RESET);\
		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_2_Port, BSP_CHG_CURRENT_SEL_2_Pin);\
}
#define MPPTCHG_DRIVER_vSetMppt20mA()                { \
		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_0_Port, BSP_CHG_CURRENT_SEL_0_Pin, GPIO_PIN_RESET);\
		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_1_Port, BSP_CHG_CURRENT_SEL_1_Pin, GPIO_PIN_RESET);\
		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_2_Port, BSP_CHG_CURRENT_SEL_2_Pin);\
}
//#define MPPTCHG_DRIVER_vSetMppt25mA()                { \
//		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_0_Port, BSP_CHG_CURRENT_SEL_0_Pin, GPIO_PIN_RESET);\
//		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_1_Port, BSP_CHG_CURRENT_SEL_1_Pin);\
//		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_2_Port, BSP_CHG_CURRENT_SEL_2_Pin);\
//}
#define MPPTCHG_DRIVER_vSetMppt25mA()                { \
		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_0_Port, BSP_CHG_CURRENT_SEL_0_Pin);\
		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_1_Port, BSP_CHG_CURRENT_SEL_1_Pin);\
		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_2_Port, BSP_CHG_CURRENT_SEL_2_Pin, GPIO_PIN_RESET);\
}
#define MPPTCHG_DRIVER_vSetMppt30mA()                { \
		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_0_Port, BSP_CHG_CURRENT_SEL_0_Pin, GPIO_PIN_RESET);\
		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_1_Port, BSP_CHG_CURRENT_SEL_1_Pin);\
		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_2_Port, BSP_CHG_CURRENT_SEL_2_Pin, GPIO_PIN_RESET);\
}
#define MPPTCHG_DRIVER_vSetMppt35mA()                { \
		HAL_GPIO_vInitAnalogNoPull(BSP_CHG_CURRENT_SEL_0_Port, BSP_CHG_CURRENT_SEL_0_Pin);\
		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_1_Port, BSP_CHG_CURRENT_SEL_1_Pin, GPIO_PIN_RESET);\
		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_2_Port, BSP_CHG_CURRENT_SEL_2_Pin, GPIO_PIN_RESET);\
}
#define MPPTCHG_DRIVER_vSetMppt40mA()                { \
		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_0_Port, BSP_CHG_CURRENT_SEL_0_Pin, GPIO_PIN_RESET);\
		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_1_Port, BSP_CHG_CURRENT_SEL_1_Pin, GPIO_PIN_RESET);\
		HAL_GPIO_vInitOutput(BSP_CHG_CURRENT_SEL_2_Port, BSP_CHG_CURRENT_SEL_2_Pin, GPIO_PIN_RESET);\
}

// Power good comes from charger, but on its own is not good enough
#define MPPTCHG_DRIVER_vInitPgPins()                    HAL_GPIO_vInitInPullup(BSP_CHG_nPOWER_GOOD_PORT, BSP_CHG_nPOWER_GOOD_PIN);
#define MPPTCHG_DRIVER_bGetPgPins()                     HAL_GPIO_ReadPin(BSP_CHG_nPOWER_GOOD_PORT, BSP_CHG_nPOWER_GOOD_PIN)
#define MPPTCHG_DRIVER_vDeinitPgPins()                  HAL_GPIO_vInitAnalogNoPull(BSP_CHG_nPOWER_GOOD_PORT, BSP_CHG_nPOWER_GOOD_PIN);

#define MPPTCHG_DRIVER_vRegisterCallback(func)          HAL_EXTI_vRegisterCallback(CHG_nPG_PIN, func)

#endif /* WORKER_MPPTCHG_MPPTCHG_DRIVER_H_ */
