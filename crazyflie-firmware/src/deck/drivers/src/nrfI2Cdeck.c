#define DEBUG_MODULE "nrfI2CDeck"
// All includes
#include "debug.h"
#include "i2cdev.h"
#include "deck.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"

// Addresses, constants etc for nrf I2C dongle
#define nrfI2CAddr 0x02
#define nrfI2C_TASK_STACKSIZE    (2 * configMINIMAL_STACK_SIZE)
#define nrfI2C_TASK_PRI 3
#define nrfI2C_TASK_NAME "nrfI2CTask"

// Misc global variables
static bool isInit;
int rssi;
void nrfI2CTask(void* arg);

// Deck driver init function
static void nrfI2CInit()
{
  if (isInit)
    return;

  DEBUG_PRINT("Initializing nrf dongle as I2C (slave) device...\n");
  i2cdevInit(I2C1_DEV);

  xTaskCreate(nrfI2CTask, nrfI2C_TASK_NAME, nrfI2C_TASK_STACKSIZE, NULL, nrfI2C_TASK_PRI, NULL);

  isInit = true;
  DEBUG_PRINT("nrfI2Cdeck initialization complete!\n");
}

// Deck driver test function
static bool nrfI2CTest()
{
  DEBUG_PRINT("nrfI2CDeck test\n");
  return true;
}

void nrfI2CTask(void* arg)
{
  uint8_t intensitydata[2];

  systemWaitStart();
  TickType_t xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(10));
   
    // Read one byte from the sensor
    i2cdevRead(I2C1_DEV, (uint8_t)nrfI2CAddr, 1, &rssi);
    // DEBUG_PRINT("RSSI is: %f\n", rssi);
  }
}


static const DeckDriver nrfI2CDriver = {
  .name = "nrfI2CDeck",
  .init = nrfI2CInit,
  .test = nrfI2CTest,
};

DECK_DRIVER(nrfI2CDriver);
LOG_GROUP_START(nrfI2C)
LOG_ADD(LOG_INT8, rssi, &rssi)
LOG_GROUP_STOP(nrfI2C)
