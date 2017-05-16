/*
 * Adapted from https://github.com/bitcraze/crazyflie-firmware/blob/376a1de0dbadf300411c25a04c6f15341259ea43/src/hal/interface/wifilink.h
 *
 * wifilink.c - UART link layer for WiFi communication
 */

#ifndef __WIFI_H__
#define __WIFI_H__

#include <stdint.h>
#include <stdbool.h>
#include "syslink.h"

void wifilinkInit(void);
bool wifilinkTest(void);
void wifilinkSetChannel(uint8_t channel);
void wifilinkSetDatarate(uint8_t datarate);
void wifilinkSetAddress(uint64_t address);
void wifilinkSyslinkDispatch(SyslinkPacket *slp);
struct crtpLinkOperations * wifilinkGetLink();


#endif //__WIFI_H__