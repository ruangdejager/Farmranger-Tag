/*
 * Farmranger.c
 *
 *  Created on: Nov 24, 2025
 *      Author: Ruan de Jager
 */

#include "Farmranger.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "dbg_log.h"

#include "str.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>


#define FR_RX_TASK_PRIORITY      (configMAX_PRIORITIES - 1) // Highest priority
#define FR_RX_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE)

#define FR_AT_HANDLER_TASK_PRIORITY      (configMAX_PRIORITIES - 2) // Lower priority than the RX task
#define FR_AT_HANDLER_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE*2)

// --- PRIVATE FREE_RTOS TASK HANDLEs ---
TaskHandle_t Farmranger_vRxTask_handle;
TaskHandle_t Farmranger_vATHandlerTask_handle;

// Buffer to build msgs before parsing
#define FR_RX_BUF_LEN 128
static char acFrRxBuf[FR_RX_BUF_LEN];
static uint8_t u8FrRxBufIdx = 0;

static SemaphoreHandle_t xLineReadySem;
static char acFrLineBuf[FR_RX_BUF_LEN];
static QueueHandle_t xATQueue;

static SemaphoreHandle_t xUartTxDoneSem;

bool bFRDeviceOn;

typedef BaseType_t (*ATParserFn)(const char *line, void *context);

typedef struct {
    const char *cmd;
    ATParserFn parser;
    void *context;
    char *out;
    size_t outLen;
    TaskHandle_t caller;
    TickType_t timeout;
} ATReq_t;

struct _farmranger_s
{
	hal_uart_t 	UartHandle;
	uint8_t		byte;
}farmranger;

BaseType_t FARMRANGER_tATSend(const char *cmd,
                   ATParserFn parser,
                   char *out,
                   size_t outLen,
                   void *context,
                   TickType_t timeout);
void FARMRANGER_vATHandlerTask(void *args);
BaseType_t FARMRANGER_tParseTimestamp(const char *line, void *ctx);
BaseType_t FARMRANGER_tParseLoggerReady(const char *line, void *ctx);
BaseType_t FARMRANGER_tParseOK(const char *line, void *ctx);
BaseType_t FARMRANGER_tParseRDY(const char *line, void *ctx);

void FARMRANGER_vInit(void)
{

	bFRDeviceOn = false;

	// Init UART
	FR_DRIVER_vInitFRDevice(&farmranger.UartHandle);
	// UART interface will be enabled/disabled at Farmranger device activation

	xLineReadySem = xSemaphoreCreateBinary();

	xUartTxDoneSem = xSemaphoreCreateBinary();
	configASSERT(xUartTxDoneSem != NULL);

	xATQueue = xQueueCreate(4, sizeof(ATReq_t));
	configASSERT(xATQueue != NULL);

    BaseType_t status;
    status = xTaskCreate(FARMRANGER_vRxTask,
            "FarmrangerRxTask",
            FR_RX_TASK_STACK_SIZE,
            NULL,
            FR_RX_TASK_PRIORITY,
			&Farmranger_vRxTask_handle);
    status = xTaskCreate(FARMRANGER_vATHandlerTask,
            "FarmrangerAtHandlerTask",
            FR_AT_HANDLER_TASK_STACK_SIZE,
            NULL,
            FR_AT_HANDLER_TASK_PRIORITY,
			&Farmranger_vATHandlerTask_handle);

    configASSERT(status == pdPASS);

}

void FARMRANGER_vUartOnWake(void)
{

	HAL_UART_vInit();
	// Init UART
	FR_DRIVER_vInitFRDevice(&farmranger.UartHandle);
	// UART interface will be enabled/disabled at Farmranger device activation

}

void FARMRANGER_vRxTask(void *parameters)
{
    uint8_t byte;

    for (;;)
    {
        // 1. Block until UART ISR notifies us (i.e. a byte arrived)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 2. Drain all available bytes from ring buffer
        while (UART_bReadByte(&farmranger.UartHandle, &byte))
        {
            if (u8FrRxBufIdx < FR_RX_BUF_LEN - 1)
            {
                acFrRxBuf[u8FrRxBufIdx++] = byte;
            }

            // For the ATcmd handler it notifies the end of a command
            if (byte == '\n')
            {
            	/* Transfer ownership of the completed line */
            	memcpy(acFrLineBuf, acFrRxBuf, u8FrRxBufIdx);
            	acFrLineBuf[u8FrRxBufIdx] = '\0';
            	u8FrRxBufIdx = 0;
            	memset(acFrRxBuf, 0, FR_RX_BUF_LEN);
            	xSemaphoreGive(xLineReadySem);
            }
        }
    }
}

void FARMRANGER_vNotifyOnRX(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (Farmranger_vRxTask_handle != NULL) {
    	vTaskNotifyGiveFromISR(Farmranger_vRxTask_handle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


bool FARMRANGER_bDeviceOn(void)
{
    /* Ensure RX task is running */
    vTaskResume(Farmranger_vRxTask_handle);

    if (bFRDeviceOn)
        return true;

    /* Clear any stale notifications */
    (void) ulTaskNotifyTake(pdTRUE, 0);

    taskENTER_CRITICAL();
    memset(acFrRxBuf, 0, FR_RX_BUF_LEN);
    memset(acFrLineBuf, 0, FR_RX_BUF_LEN);
    u8FrRxBufIdx = 0;
    xSemaphoreTake(xLineReadySem, 0); // drain
    taskEXIT_CRITICAL();

    /* Power up device first */
    FR_DRIVER_vEnableUart(&farmranger.UartHandle);
    FR_DRIVER_vIntEnable();

    char respBuf[32] = {0};
    memset(respBuf, 0, sizeof(respBuf));

    /* Now wait for RDY via AT handler */
    DBG("Wait for RDY...\r\n");
    if (FARMRANGER_tATSend(NULL,
                           FARMRANGER_tParseRDY,
                           respBuf,
                           sizeof(respBuf),
                           respBuf,
                           pdMS_TO_TICKS(5000)) != pdPASS)
    {
        DBG("RDY not received\r\n");
        return false;
    }

    bFRDeviceOn = true;
    DBG("Farmranger Ready.\r\n");
    return true;
}


void FARMRANGER_vDeviceOff(void)
{
	// Disable RX Task
	vTaskSuspend(Farmranger_vRxTask_handle);

	// Enable the uart peripheral
	FR_DRIVER_vDisableUart(&farmranger.UartHandle);
	// Drive the interrupt gpio high to activate farmranger
	FR_DRIVER_vIntDisable();

	bFRDeviceOn = false;
	DBG("Farmranger released.\r\n");

}

BaseType_t FARMRANGER_tATSend(const char *cmd,
                   ATParserFn parser,
                   char *out,
                   size_t outLen,
                   void *context,
                   TickType_t timeout)
{
    ATReq_t req = {
        .cmd = cmd,
        .parser = parser,
        .context = context,
        .out = out,
        .outLen = outLen,
        .caller = xTaskGetCurrentTaskHandle(),
        .timeout = timeout
    };

    (void) ulTaskNotifyTake(pdTRUE, 0);

    configASSERT(xATQueue != NULL);
    if (xQueueSend(xATQueue, &req, pdMS_TO_TICKS(100)) != pdPASS)
        return pdFAIL;

    uint32_t notifyValue = 0;
    if (xTaskNotifyWait(0, 0xFFFFFFFF, &notifyValue, timeout) != pdTRUE) return pdFAIL;

    return (notifyValue == 1) ? pdPASS : pdFAIL;
}

void FARMRANGER_vATHandlerTask(void *args)
{
    ATReq_t req;

    for (;;)
    {
        if (xQueueReceive(xATQueue, &req, portMAX_DELAY))
        {

        	if (req.cmd && strlen(req.cmd) > 0)
        	{
        		HAL_UART_vTxPutBuffer(&farmranger.UartHandle,
                                       (uint8_t*)req.cmd,
                                       strlen(req.cmd));
        	}

            TickType_t start = xTaskGetTickCount();
            BaseType_t notified = pdFALSE;

            while ((xTaskGetTickCount() - start) < req.timeout)
            {
                if (xSemaphoreTake(xLineReadySem, pdMS_TO_TICKS(50)) == pdTRUE)
                {
                    // Check line using parser
                	if (req.parser(acFrLineBuf, req.context))
                    {
                		xTaskNotify(req.caller,
                		            1,   // SUCCESS
                		            eSetValueWithOverwrite);
                        notified = pdTRUE;
                        break;
                    }

                    /* Clear line so we don’t re-parse junk */
                    memset(acFrLineBuf, 0, FR_RX_BUF_LEN);
                }
            }

            // Timeout → notify caller, then clear buffer
            if (!notified)
        		xTaskNotify(req.caller,
        		            2,   // TIMEOUT
        		            eSetValueWithOverwrite);

            memset(acFrRxBuf, 0, FR_RX_BUF_LEN);
            u8FrRxBufIdx = 0;
        }
    }
}


BaseType_t FARMRANGER_tParseTimestamp(const char *line, void *ctx)
{

	// Expect format: "1234567890\r\n"

	char *out = (char *)ctx;

	size_t len = strlen(line);

	// Expect: 10 digits + "\r\n"
	if (len == 12 && line[10] == '\r' && line[11] == '\n')
	{
		// Validate all 10 characters are digits
		for (int i = 0; i < 10; i++)
		{
			if (line[i] < '0' || line[i] > '9')
				return pdFALSE;
		}

		// Copy only the 10 digits
		memcpy(out, line, 10);
		out[10] = '\0';

		return pdTRUE;
	}

	return pdFALSE;
}


uint64_t FARMRANGER_u64RequestTimestamp(void)
{

    char tsStr[32] = {0};
    uint64_t tsValue = 0; // default if fail

    if (FARMRANGER_tATSend("AT+TSREQ\r\n",
    			FARMRANGER_tParseTimestamp,
                tsStr,
                sizeof(tsStr),
				tsStr,
                pdMS_TO_TICKS(2000)) == pdPASS)
    {
        // Convert ASCII timestamp (e.g., "1701122334") to uint64_t
        tsValue = strtoull(tsStr, NULL, 10);
    }

    return tsValue;

}

bool FARMRANGER_bLogData(MeshDiscoveredNeighbor_t *neighbors, uint16_t count)
{

    // 1. Build CSV-style payload
    static char logBuffer[4096];   // fits ~ 250 entries easily
    size_t pos = 0;

    for (uint16_t i = 0; i < count; i++)
    {
        int n = snprintf(&logBuffer[pos],
                         sizeof(logBuffer) - pos,
                         "%X,%u,%d,%u\t",
                         neighbors[i].u32DeviceId,
                         neighbors[i].u8HopCount,
                         neighbors[i].i16Rssi,
                         neighbors[i].u16BatMv);

        if (n <= 0 || n >= (int)(sizeof(logBuffer) - pos))
            return false;

        pos += n;
    }

    // 2. Send AT+LOG=<len>\r\n
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+LOG=%u\r\n", (unsigned)pos);

    char respBuf[32] = {0};

    if (FARMRANGER_tATSend(cmd,
                           FARMRANGER_tParseLoggerReady,
                           respBuf,
                           sizeof(respBuf),
                           respBuf,
                           pdMS_TO_TICKS(2000)) != pdPASS)
    {
        DBG("LogData: No 'Logger ready' received.\r\n");
        return false;
    }

    if (pos > 0)
    {
        // 3. Send the actual payload (CSV buffer)
    	HAL_UART_vTxPutBuffer(&farmranger.UartHandle,
                               (uint8_t*)logBuffer,
    						   pos);
        /* Wait until TX fully drained */
        if (xSemaphoreTake(xUartTxDoneSem, pdMS_TO_TICKS(3000)) != pdTRUE)
        {
            DBG("UART TX timeout\r\n");
            return false;
        }
    }

    // 4. Now wait for final OK
    memset(respBuf, 0, sizeof(respBuf));
    if (FARMRANGER_tATSend(NULL,
                           FARMRANGER_tParseOK,
                           respBuf,
                           sizeof(respBuf),
                           respBuf,
                           pdMS_TO_TICKS(2000)) != pdPASS)
    {
        DBG("LogData: No final OK received.\r\n");
        return false;
    }

    return true;
}


BaseType_t FARMRANGER_tParseLoggerReady(const char *line, void *ctx)
{
    if (strstr(line, "Logger ready\r\n") != NULL)
        return pdTRUE;

    return pdFALSE;
}

BaseType_t FARMRANGER_tParseOK(const char *line, void *ctx)
{
    if (!line) return pdFALSE;

    /* Accept RDY anywhere in the line */
    if (strstr(line, "OK") != NULL)
        return pdTRUE;

    return pdFALSE;
}

BaseType_t FARMRANGER_tParseRDY(const char *line, void *ctx)
{
    if (!line) return pdFALSE;

    /* Accept RDY anywhere in the line */
    if (strstr(line, "RDY") != NULL)
        return pdTRUE;

    return pdFALSE;
}


void HAL_UART_vTxCompleteISR(hal_uart_t *drv)
{
    if (drv == &farmranger.UartHandle)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xUartTxDoneSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}



