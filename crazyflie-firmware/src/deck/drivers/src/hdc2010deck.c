
#define DEBUG_MODULE "hdc2010Deck"
// All includes
#include "debug.h"
#include "i2cdev.h"
#include "deck.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "math.h"

// Addresses, constants etc for HDC2010
#define HDC2010I2CAddr 0x40  // for ADDR (or CS) pin low
// #define HDC2010I2CAddr 0x41  // for ADDR (or CS) pin high
// Registers
#define MeasConfReg 0x0F
#define TempLowByteReg 0x00
#define TempHighByteReg 0x01
#define HDC2010_TASK_STACKSIZE    (2 * configMINIMAL_STACK_SIZE)
#define HDC2010_TASK_PRI 3
#define HDC2010_TASK_NAME "HDC2010"

// Misc global variables
static bool isInit;
uint16_t raw_temp;
float temp;
void hdc2010Task(void* arg);

// Deck driver init function
static void hdc2010Init()
{
  if (isInit)
    return;

  DEBUG_PRINT("Initializing HDC2010...\n");
  i2cdevInit(I2C1_DEV);

  xTaskCreate(hdc2010Task, HDC2010_TASK_NAME, HDC2010_TASK_STACKSIZE, NULL, HDC2010_TASK_PRI, NULL);

  isInit = true;
  DEBUG_PRINT("HDC2010 initialization complete!\n");
}

// Deck driver test function
static bool hdc2010Test()
{
  DEBUG_PRINT("HDC2010Deck test\n");
  return true;
}

void hdc2010Task(void* arg)
{
  uint8_t dataHB;
  uint8_t dataLB;

  systemWaitStart();
  TickType_t xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(200));

    // Set measurement config to measure temperature only
    i2cdevWriteByte(I2C1_DEV, HDC2010I2CAddr, MeasConfReg, 0x03);

    // Read two bytes from the sensor
    i2cdevReadByte(I2C1_DEV, (uint8_t)HDC2010I2CAddr, (uint8_t)TempLowByteReg, &dataLB);
    i2cdevReadByte(I2C1_DEV, (uint8_t)HDC2010I2CAddr, (uint8_t)TempHighByteReg, &dataHB);
    // DEBUG_PRINT("Raw bytes received: %d, %d\n", dataHB, dataLB);
    // Combine the two bytes to get data    
    raw_temp = (dataHB<<8) | dataLB;
    // Convert raw data to Celcius as per datasheet
    if(dataHB!=0 && dataLB!=0) {
      temp = ( (raw_temp*165.0) / 65536.0 ) - 40.0;
    }
    // DEBUG_PRINT("Temperature reading is: %f\n", temp);
  }
}


static const DeckDriver hdc2010Driver = {
  .name = "hdc2010Deck",
  .init = hdc2010Init,
  .test = hdc2010Test,
};

DECK_DRIVER(hdc2010Driver);
LOG_GROUP_START(HDC2010)
LOG_ADD(LOG_FLOAT, temp, &temp)
LOG_GROUP_STOP(HDC2010)
