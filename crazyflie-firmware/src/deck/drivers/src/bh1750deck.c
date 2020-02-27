#define DEBUG_MODULE "bh1750Deck"
// All includes
#include "debug.h"
#include "i2cdev.h"
#include "deck.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"

// Addresses, constants etc for BH1750
#define BH1750I2CAddr 0x23
#define ONE_TIME_HI_RES_MODE 0x20
#define ONE_TIME_HI_RES_MODE2 0x21
#define ONE_TIME_LOW_RES_MODE 0x23
#define CONT_HI_RES_MODE 0x10
#define CONT_HI_RES_MODE2 0x11
#define CONT_LOW_RES_MODE 0x13
float conv_factor = 1.2;
#define BH1750_TASK_STACKSIZE    (2 * configMINIMAL_STACK_SIZE)
#define BH1750_TASK_PRI 3
#define BH1750_TASK_NAME "BH1750"

// Misc global variables
static bool isInit;
float intensity;
void bh1750Task(void* arg);

// Deck driver init function
static void bh1750Init()
{
  if (isInit)
    return;

  DEBUG_PRINT("Initializing BH1750...\n");
  i2cdevInit(I2C1_DEV);

  xTaskCreate(bh1750Task, BH1750_TASK_NAME, BH1750_TASK_STACKSIZE, NULL, BH1750_TASK_PRI, NULL);

  isInit = true;
  DEBUG_PRINT("Initialization complete!\n");
}

// Deck driver test function
static bool bh1750Test()
{
  DEBUG_PRINT("BH1750Deck test\n");
  return true;
}

void bh1750Task(void* arg)
{
  uint8_t dataHB;
  uint8_t dataLB;
  bool ack;
  uint8_t intensitydata[2];

  systemWaitStart();
  TickType_t xLastWakeTime;

  // Write mode to sensor
  i2cdevWriteByte(I2C1_DEV, (uint8_t)BH1750I2CAddr, I2CDEV_NO_MEM_ADDR, (uint8_t)CONT_HI_RES_MODE);

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(120));
   
    // Read two bytes from the sensor
    i2cdevRead(I2C1_DEV, (uint8_t)BH1750I2CAddr, 2, &intensitydata);
    // Combine the two bytes to get data
    intensity = intensitydata[0] << 8 | intensitydata[1];
    intensity = intensity/conv_factor;
    //DEBUG_PRINT("Light reading is: %f\n", data);
  }
}


static const DeckDriver bh1750Driver = {
  .name = "bh1750Deck",
  .init = bh1750Init,
  .test = bh1750Test,
};

DECK_DRIVER(bh1750Driver);
LOG_GROUP_START(BH1750)
LOG_ADD(LOG_FLOAT, intensity, &intensity)
LOG_GROUP_STOP(BH1750)
