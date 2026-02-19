/*
 * Power.c
 *
 *  Created on: Feb 19, 2026
 *      Author: Ruan de Jager
 */

#include "Power.h"
#include "Power_Config.h"
#include "platform.h"
#include "Battery.h"
#include "DeviceDiscovery.h"

#include <limits.h>

static EventGroupHandle_t gPowerEvents;

static TaskHandle_t xPowerStateManagerTaskHandle = NULL;

static void POWER_vStateManagerTask(void *arg);
static void POWER_vEnterRecovery(void);
static void POWER_vExitRecovery(void);

void POWER_vInit(void)
{
    gPowerEvents = xEventGroupCreate();

    /* Start in normal mode */
    POWER_vSetModeNormal();

    BaseType_t status;

	if (DEVICE_DISCOVERY_eGetDeviceRole() == DEVICE_ROLE_SECONDARY)
	{
	    status = xTaskCreate(POWER_vStateManagerTask,
	                "PowerStateManager",
					configMINIMAL_STACK_SIZE,
	                NULL,
					(configMAX_PRIORITIES - 4),
					&xPowerStateManagerTaskHandle);
	    configASSERT(status == pdPASS);
	}

}

void POWER_vSetModeNormal(void)
{
    xEventGroupSetBits(gPowerEvents,
                       POWER_CLASS_NORMAL |
                       POWER_CLASS_ALWAYS);

    xEventGroupClearBits(gPowerEvents,
                         POWER_CLASS_RECOVERY);

}

void POWER_vSetModeRecovery(void)
{
    xEventGroupSetBits(gPowerEvents,
                       POWER_CLASS_RECOVERY |
                       POWER_CLASS_ALWAYS);

    xEventGroupClearBits(gPowerEvents,
                         POWER_CLASS_NORMAL);
}

void POWER_vWaitForClass(EventBits_t classMask)
{
    xEventGroupWaitBits(gPowerEvents,
                        classMask,
                        pdFALSE,
                        pdFALSE,
                        portMAX_DELAY);
}

EventBits_t POWER_tGetState(void)
{
    return xEventGroupGetBits(gPowerEvents);
}

static void POWER_vStateManagerTask(void *arg)
{

	PLATFORM_bSubscribeToHeartbeat(xTaskGetCurrentTaskHandle(),
                        HB_ALLOW_IN_RECOVERY);

    for (;;)
    {
		// Wait 1s heartbeat
		xTaskNotifyWait(
			0x00,            // Don't clear bits on entry
			ULONG_MAX,       // Clear all bits on exit
			NULL,
			portMAX_DELAY
		);

        uint16_t batVoltage = BAT_u16GetVoltage();

        if ((POWER_tGetState() & POWER_CLASS_NORMAL) &&
        		batVoltage < ENTER_RECOVERY_MV)
        {
        	POWER_vEnterRecovery();
        }

        if (!(POWER_tGetState() & POWER_CLASS_NORMAL) &&
        		batVoltage > EXIT_RECOVERY_MV)
        {
        	POWER_vExitRecovery();
        }
    }
}

static void POWER_vEnterRecovery(void)
{
    POWER_vSetModeRecovery();
}

static void POWER_vExitRecovery(void)
{
	POWER_vSetModeNormal();
}

