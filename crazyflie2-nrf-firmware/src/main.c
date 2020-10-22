/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 NRF Firmware
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */
#include <nrf.h>

#include <stdio.h>
#include <string.h>

#include "debug.h"
#include "platform.h"

#include "uart.h"
#include "esb.h"
#include "syslink.h"
#include "led.h"
#include "button.h"
#include "pm.h"
#include "pinout.h"
#include "systick.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

#include "memory.h"
#include "ownet.h"

#include "ble_crazyflies.h"

extern void  initialise_monitor_handles(void);
extern int ble_init(void);

#ifndef SEMIHOSTING
#define printf(...)
#endif

#ifndef DEFAULT_RADIO_RATE
  #define DEFAULT_RADIO_RATE  esbDatarate2M
#endif
#ifndef DEFAULT_RADIO_CHANNEL
  #define DEFAULT_RADIO_CHANNEL 80
#endif

static void mainloop(void);

#if BLE==0
#undef BLE
#endif

#ifdef BLE
int bleEnabled = 1;
#else
int bleEnabled = 0;
#endif

static bool boottedFromBootloader;

static void handleRadioCmd(struct esbPacket_s * packet);
static void handleBootloaderCmd(struct esbPacket_s *packet);
static void disableBle();

static int stmStartTime = 0;
int main()
{
  // Stop early if the platform is not supported
  if (platformInit() != 0) {
    while(1);
  }

  debugInit();
  systickInit();
  memoryInit();

  if (bleEnabled) {
    ble_init();
  } else {
    NRF_CLOCK->TASKS_HFCLKSTART = 1UL;
    while(!NRF_CLOCK->EVENTS_HFCLKSTARTED);
  }

#ifdef SEMIHOSTING
  initialise_monitor_handles();
#endif

  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSTAT_SRC_Synth;

  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;
  while(!NRF_CLOCK->EVENTS_LFCLKSTARTED);

  LED_INIT();
  if ((NRF_POWER->GPREGRET & 0x80) && ((NRF_POWER->GPREGRET&(0x3<<1))==0)) {
    buttonInit(buttonShortPress);
  } else {
    buttonInit(buttonIdle);
  }

  if  (NRF_POWER->GPREGRET & 0x20) {
    boottedFromBootloader = true;
    NRF_POWER->GPREGRET &= ~0x20;
  }

  pmInit();

  if ((NRF_POWER->GPREGRET&0x01) == 0) {
		  pmSetState(pmSysRunning);
  }

  LED_ON();


  NRF_GPIO->PIN_CNF[RADIO_PAEN_PIN] |= GPIO_PIN_CNF_DIR_Output | (GPIO_PIN_CNF_DRIVE_S0H1<<GPIO_PIN_CNF_DRIVE_Pos);

  if (!bleEnabled) {
    esbInit();

    esbSetDatarate(DEFAULT_RADIO_RATE);
    esbSetChannel(DEFAULT_RADIO_CHANNEL);
  }

  DEBUG_PRINT("Started\n");

  mainloop();

  // The main loop should never end
  // TODO see if we should shut-off the system there?
  while(1);

  return 0;
}

void mainloop()
{
  static struct syslinkPacket slRxPacket;
  static struct syslinkPacket slTxPacket;
  static EsbPacket esbRxPacket;
  bool esbReceived = false;
  bool slReceived;
  static int vbatSendTime;
  static int radioRSSISendTime;
  static uint8_t rssi;
  static uint8_t rssi_alt;  // custom for another bluetooth device
  static bool broadcast;
  static bool p2p;

  while(1)
  {

    if (bleEnabled) {
      if ((esbReceived == false) && bleCrazyfliesIsPacketReceived()) {
        EsbPacket* packet = bleCrazyfliesGetRxPacket();
        memcpy(esbRxPacket.data, packet->data, packet->size);
        esbRxPacket.size = packet->size;
        esbReceived = true;
        bleCrazyfliesReleaseRxPacket(packet);
      }
    }

#ifndef CONT_WAVE_TEST

    if ((esbReceived == false) && esbIsRxPacket())
    {
      EsbPacket* packet = esbGetRxPacket();
      //Store RSSI here so that we can send it to STM later
      // Todo investigate if we can not just simply link this to the packet itself or find a way to separate this due to P2P
      rssi = packet->rssi;
      // The received packet was a broadcast, if received on local address 1
      broadcast = packet->match == ESB_MULTICAST_ADDRESS_MATCH;
      // If the packet is a null packet with data[1] == 0x8*, it is a P2P packet
      if (packet->size >= 2 && (packet->data[0] & 0xf3) == 0xf3 && (packet->data[1] & 0xF0) == 0x80) {
        p2p = true;
        esbRxPacket.rssi = packet->rssi;
      } else {
        p2p = false;
      }
      memcpy(esbRxPacket.data, packet->data, packet->size);
      esbRxPacket.size = packet->size;
      esbReceived = true;
      esbReleaseRxPacket(packet);

      disableBle();
    }

    if (esbReceived)
    {
      EsbPacket* packet = &esbRxPacket;
      esbReceived = false;

      if((packet->size >= 4) && (packet->data[0]&0xf3) == 0xf3 && (packet->data[1]==0x03))
      {
        handleRadioCmd(packet);
      }
      else if ((packet->size >2) && (packet->data[0]&0xf3) == 0xf3 && (packet->data[1]==0xfe))
      {
        handleBootloaderCmd(packet);
      }
      else
      {
        if (p2p == false) {
          memcpy(slTxPacket.data, packet->data, packet->size);
          slTxPacket.length = packet->size;
          if (broadcast) {
            slTxPacket.type = SYSLINK_RADIO_RAW_BROADCAST;
          } else {
            slTxPacket.type = SYSLINK_RADIO_RAW;
          }
        } else {
          // The first byte sent is the P2P port
          slTxPacket.data[0] = packet->data[1] & 0x0F;
          slTxPacket.data[1] = packet->rssi; // Save RSSI between drones in packet
          memcpy(&slTxPacket.data[2], &packet->data[2], packet->size-2);
          slTxPacket.length = packet->size;
          if (broadcast) {
            slTxPacket.type = SYSLINK_RADIO_P2P_BROADCAST;
          } else {
            slTxPacket.type = SYSLINK_RADIO_P2P;
          }
        }
        syslinkSend(&slTxPacket);
      }
    }

    slReceived = syslinkReceive(&slRxPacket);
    if (slReceived)
    {
      switch (slRxPacket.type)
      {
        case SYSLINK_RADIO_RAW:
          if (esbCanTxPacket() && (slRxPacket.length < SYSLINK_MTU))
          {
            EsbPacket* packet = esbGetTxPacket();

            if (packet) {
              memcpy(packet->data, slRxPacket.data, slRxPacket.length);
              packet->size = slRxPacket.length;

              esbSendTxPacket(packet);
            }
            bzero(slRxPacket.data, SYSLINK_MTU);
          }

          if (bleEnabled) {
            if (slRxPacket.length < SYSLINK_MTU) {
              static EsbPacket pk;
              memcpy(pk.data,  slRxPacket.data, slRxPacket.length);
              pk.size = slRxPacket.length;
              bleCrazyfliesSendPacket(&pk);
            }
          }

          break;
        case SYSLINK_RADIO_CHANNEL:
          if(slRxPacket.length == 1)
          {
            esbSetChannel(slRxPacket.data[0]);

            slTxPacket.type = SYSLINK_RADIO_CHANNEL;
            slTxPacket.data[0] = slRxPacket.data[0];
            slTxPacket.length = 1;
            syslinkSend(&slTxPacket);
          }
          break;
        case SYSLINK_RADIO_DATARATE:
          if(slRxPacket.length == 1)
          {
            esbSetDatarate(slRxPacket.data[0]);

            slTxPacket.type = SYSLINK_RADIO_DATARATE;
            slTxPacket.data[0] = slRxPacket.data[0];
            slTxPacket.length = 1;
            syslinkSend(&slTxPacket);
          }
          break;
        case SYSLINK_RADIO_CONTWAVE:
          if(slRxPacket.length == 1) {
            esbSetContwave(slRxPacket.data[0]);

            slTxPacket.type = SYSLINK_RADIO_CONTWAVE;
            slTxPacket.data[0] = slRxPacket.data[0];
            slTxPacket.length = 1;
            syslinkSend(&slTxPacket);
          }
          break;
        case SYSLINK_RADIO_ADDRESS:
          if(slRxPacket.length == 5)
          {
            uint64_t address = 0;
            memcpy(&address, &slRxPacket.data[0], 5);
            esbSetAddress(address);

            slTxPacket.type = SYSLINK_RADIO_ADDRESS;
            memcpy(slTxPacket.data, slRxPacket.data, 5);
            slTxPacket.length = 5;
            syslinkSend(&slTxPacket);
          }
          break;
        case SYSLINK_RADIO_POWER:
          if(slRxPacket.length == 1)
          {
            esbSetTxPowerDbm((int8_t)slRxPacket.data[0]);

            slTxPacket.type = SYSLINK_RADIO_POWER;
            slTxPacket.data[0] = slRxPacket.data[0];
            slTxPacket.length = 1;
            syslinkSend(&slTxPacket);
          }
          break;
        case SYSLINK_PM_ONOFF_SWITCHOFF:
          pmSetState(pmAllOff);
          break;
        case SYSLINK_OW_GETINFO:
        case SYSLINK_OW_READ:
        case SYSLINK_OW_SCAN:
        case SYSLINK_OW_WRITE:
          if (memorySyslink(&slRxPacket)) {
            syslinkSend(&slRxPacket);
          }
          break;
        case SYSLINK_RADIO_P2P_BROADCAST:
          // Send the P2P packet immediately without buffer
          esbSendP2PPacket(slRxPacket.data[0],&slRxPacket.data[1],slRxPacket.length-1);
          break;

      }
    }

    // Wait a while to start pushing over the syslink since UART pins are used to launch STM32 i bootloader as well
    if (systickGetTick() > (stmStartTime + SYSLINK_STARTUP_DELAY_TIME_MS)) {
      // Send the battery voltage and state to the STM every SYSLINK_SEND_PERIOD_MS
      if (systickGetTick() >= vbatSendTime + SYSLINK_SEND_PERIOD_MS) {
        float fdata;
        uint8_t flags = 0;

        vbatSendTime = systickGetTick();
        slTxPacket.type = SYSLINK_PM_BATTERY_STATE;
        slTxPacket.length = 9;

        flags |= (pmIsCharging() == true)?0x01:0;
        flags |= (pmUSBPower() == true)?0x02:0;

        slTxPacket.data[0] = flags;

        fdata = pmGetVBAT();
        memcpy(slTxPacket.data+1, &fdata, sizeof(float));

        fdata = pmGetISET();
        memcpy(slTxPacket.data+1+4, &fdata, sizeof(float));

#ifdef PM_SYSLINK_INCLUDE_TEMP
        fdata = pmGetTemp();
        slTxPacket.length += 4;
        memcpy(slTxPacket.data+1+8, &fdata, sizeof(float));
#endif
        syslinkSend(&slTxPacket);
      }
      //Send an RSSI sample to the STM every 10ms(100Hz)

      if (systickGetTick() >= radioRSSISendTime + 10) {
        radioRSSISendTime = systickGetTick();
        slTxPacket.type = SYSLINK_RADIO_RSSI;
        //This message contains only the RSSI measurement which consist
        //of a single uint8_t
        slTxPacket.length = sizeof(uint8_t);
        memcpy(slTxPacket.data, &rssi, sizeof(uint8_t));

        syslinkSend(&slTxPacket);

	// custom for another bluetooth device
        // msDelay(5);
        rssi_alt = 10;  // custom for another bluetooth device
	slTxPacket.type = SYSLINK_ALT_RSSI;
        //This message contains only the RSSI measurement which consist
        //of a single uint8_t
        slTxPacket.length = sizeof(uint8_t);
        memcpy(slTxPacket.data, &rssi_alt, sizeof(uint8_t));

        syslinkSend(&slTxPacket);
      }
    }
#endif

    // Button event handling
    ButtonEvent be = buttonGetState();
    bool usbConnected = pmUSBPower();
    if ((pmGetState() != pmSysOff) && (be == buttonShortPress) && !usbConnected)
    {
      pmSetState(pmAllOff);
      /*swdInit();
      swdTest();*/
    }
    else if ((pmGetState() != pmSysOff) && (be == buttonShortPress)
                                        && usbConnected)
    {
    	//pmSetState(pmSysOff);
      pmSetState(pmAllOff);
        /*swdInit();
        swdTest();*/
    }
    else if ((pmGetState() == pmSysOff) && (be == buttonShortPress))
    {
      //Normal boot
      pmSysBootloader(false);
      pmSetState(pmSysRunning);
    }
    else if ((pmGetState() == pmSysOff) && boottedFromBootloader)
    {
      //Normal boot after bootloader
      pmSysBootloader(false);
      pmSetState(pmSysRunning);
    }
    else if ((pmGetState() == pmSysOff) && (be == buttonLongPress))
    {
      //stm bootloader
      pmSysBootloader(true);
      pmSetState(pmSysRunning);
    }
    boottedFromBootloader = false;

    // processes loop
    buttonProcess();
    pmProcess();
    //owRun();       //TODO!
  }
}

#define RADIO_CTRL_SET_CHANNEL 1
#define RADIO_CTRL_SET_DATARATE 2
#define RADIO_CTRL_SET_POWER 3

static void handleRadioCmd(struct esbPacket_s *packet)
{
  switch (packet->data[2]) {
    case RADIO_CTRL_SET_CHANNEL:
      esbSetChannel(packet->data[3]);
      break;
    case RADIO_CTRL_SET_DATARATE:
      esbSetDatarate(packet->data[3]);
      break;
    case RADIO_CTRL_SET_POWER:
      esbSetTxPower(packet->data[3]);
      break;
    default:
      break;
  }
}

#define BOOTLOADER_CMD_RESET_INIT 0xFF
#define BOOTLOADER_CMD_RESET      0xF0
#define BOOTLOADER_CMD_ALLOFF     0x01
#define BOOTLOADER_CMD_SYSOFF     0x02
#define BOOTLOADER_CMD_SYSON      0x03
#define BOOTLOADER_CMD_GETVBAT    0x04
#define BOOTLOADER_CMD_LED_ON     0x05
#define BOOTLOADER_CMD_LED_OFF    0x06

static void handleBootloaderCmd(struct esbPacket_s *packet)
{
  static bool resetInit = false;
  static struct esbPacket_s txpk;

  switch (packet->data[2]) {
    case BOOTLOADER_CMD_RESET_INIT:

      resetInit = true;

      txpk.data[0] = 0xff;
      txpk.data[1] = 0xfe;
      txpk.data[2] = BOOTLOADER_CMD_RESET_INIT;

      memcpy(&(txpk.data[3]), (uint32_t*)NRF_FICR->DEVICEADDR, 6);

      txpk.size = 9;
      if (bleEnabled) {
        bleCrazyfliesSendPacket(&txpk);
      }

      if (esbCanTxPacket()) {
        struct esbPacket_s *pk = esbGetTxPacket();
        memcpy(pk, &txpk, sizeof(struct esbPacket_s));
        esbSendTxPacket(pk);
      }

      break;
    case BOOTLOADER_CMD_RESET:
      if (resetInit && (packet->size == 4)) {
        msDelay(100);
        if (packet->data[3] == 0) {
          NRF_POWER->GPREGRET |= 0x40;
        } else {
          //Set bit 0x20 forces boot to firmware
          NRF_POWER->GPREGRET |= 0x20U;
        }
        if (bleEnabled) {
          sd_nvic_SystemReset();
        } else {
          NVIC_SystemReset();
        }
      }
      break;
    case BOOTLOADER_CMD_ALLOFF:
      pmSetState(pmAllOff);
      break;
    case BOOTLOADER_CMD_SYSOFF:
      pmSetState(pmSysOff);
      break;
    case BOOTLOADER_CMD_SYSON:
      pmSysBootloader(false);
      pmSetState(pmSysRunning);
      stmStartTime = systickGetTick();
      syslinkReset();

      break;
    case BOOTLOADER_CMD_GETVBAT:
      if (esbCanTxPacket()) {
        float vbat = pmGetVBAT();
        struct esbPacket_s *pk = esbGetTxPacket();

        pk->data[0] = 0xff;
        pk->data[1] = 0xfe;
        pk->data[2] = BOOTLOADER_CMD_GETVBAT;

        memcpy(&(pk->data[3]), &vbat, sizeof(float));
        pk->size = 3 + sizeof(float);

        esbSendTxPacket(pk);
      }
      break;
    case BOOTLOADER_CMD_LED_ON:
      LED_ON();
      break;
    case BOOTLOADER_CMD_LED_OFF:
      LED_OFF();
      break;
    default:
      break;
  }
}

static void disableBle() {
  if (bleEnabled) {
      sd_softdevice_disable();
      bleEnabled = 0;
      esbInit();
  }
}
