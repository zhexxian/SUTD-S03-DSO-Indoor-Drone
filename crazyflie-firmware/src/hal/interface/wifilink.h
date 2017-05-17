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
#include "eprintf.h"

#define UART_TYPE       USART3
#define UART_PERIF      RCC_APB1Periph_USART3

#define UART_DMA_IRQ    DMA1_Channel2_IRQn
#define UART_DMA_IT_TC  DMA1_IT_TC2
#define UART_DMA_CH     DMA1_Channel2

#define UART_GPIO_PERIF RCC_APB2Periph_GPIOB
#define UART_GPIO_PORT  GPIOB
#define UART_GPIO_TX    GPIO_Pin_10
#define UART_GPIO_RX    GPIO_Pin_11

#define CRTP_START_BYTE  0xAA
#define SYSLINK_START_BYTE1 0xBC
#define SYSLINK_START_BYTE2 0xCF

void wifilinkInit(void);
bool wifilinkTest(void);
struct crtpLinkOperations * wifilinkGetLink();


#endif //__WIFI_H__