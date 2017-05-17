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
#include "crtp.h"


//#define SYSLINK_MTU 32

#define CRTP_START_BYTE  0xAA
#define SYSLINK_START_BYTE1 0xBC
#define SYSLINK_START_BYTE2 0xCF

void wifilinkInit(void);
bool wifilinkTest(void);
struct crtpLinkOperations * wifilinkGetLink();


#endif //__WIFI_H__