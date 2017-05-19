/** Adapted from https://github.com/bitcraze/crazyflie-firmware/blob/69b8d9394df464580395da291559c93a504b579d/src/hal/src/wifilink.c
 *
 *
 * wifilink.c - UART link layer for WiFi communication
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "queuemonitor.h"
#include "semphr.h"

#include "config.h"
#include "wifilink.h"
#include "syslink.h"
#include "uart2.h"
#include "crtp.h"
#include "configblock.h"
#include "log.h"
#include "led.h"
#include "ledseq.h"
#include "queuemonitor.h"
#include "pm.h"
#include "config.h"

#include "crtp.h"
#include "crtpservice.h"

#define WIFILINK_TX_QUEUE_SIZE (1)
#define UART_DATA_TIMEOUT_MS 1000
#define UART_DATA_TIMEOUT_TICKS (UART_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CRTP_START_BYTE 0xAA
#define CCR_ENABLE_SET  ((uint32_t)0x00000001)

static xQueueHandle crtpPacketDelivery;
static uint8_t sendBuffer[64];

static bool isInit;

static int wifilinkSendCRTPPacket(CRTPPacket *p);
static int wifilinkSetEnable(bool enable);
static int wifilinkReceiveCRTPPacket(CRTPPacket *p);

static xQueueHandle uartDataDelivery;

void uartRxTask(void *param);

static struct crtpLinkOperations wifilinkOp =
{
  .setEnable         = wifilinkSetEnable,
  .sendPacket        = wifilinkSendCRTPPacket,
  .receivePacket     = wifilinkReceiveCRTPPacket,
};

void wifilinkInit(void)
{
  if (isInit)
    return;

  xTaskCreate(uartRxTask, UART_RX_TASK_NAME,
              UART_RX_TASK_STACKSIZE, NULL, 5, NULL);

  crtpPacketDelivery = xQueueCreate(5, sizeof(CRTPPacket));
  uartDataDelivery = xQueueCreate(1024, sizeof(uint8_t));

  isInit = true;
}

void uartRxTask(void *param)
{
  enum {waitForFirstStart, waitForSecondStart,
        waitForPort, waitForSize, waitForData, waitForCRC } rxState;

  uint8_t c;
  uint8_t dataIndex = 0;
  uint8_t crc = 0;
  CRTPPacket p;
  rxState = waitForFirstStart;
  uint8_t counter = 0;
  while(1)
  {
    if (xQueueReceive(uartDataDelivery, &c, UART_DATA_TIMEOUT_TICKS) == pdTRUE)
    {
      counter++;
     /* if (counter > 4)
        ledSetRed(1);*/
      switch(rxState)
      {
        case waitForFirstStart:
          rxState = (c == CRTP_START_BYTE) ? waitForSecondStart : waitForFirstStart;
          break;
        case waitForSecondStart:
          rxState = (c == CRTP_START_BYTE) ? waitForPort : waitForFirstStart;
          break;
        case waitForPort:
          p.header = c;
          crc = c;
          rxState = waitForSize;
          break;
        case waitForSize:
          if (c < CRTP_MAX_DATA_SIZE)
          {
            p.size = c;
            crc = (crc + c) % 0xFF;
            dataIndex = 0;
            rxState = (c > 0) ? waitForData : waitForCRC;
          }
          else
          {
            rxState = waitForFirstStart;
          }
          break;
        case waitForData:
          p.data[dataIndex] = c;
          crc = (crc + c) % 0xFF;
          dataIndex++;
          if (dataIndex == p.size)
          {
            rxState = waitForCRC;
          }
          break;
        case waitForCRC:
          if (crc == c)
          {
            xQueueSend(crtpPacketDelivery, &p, 0);
          }
          rxState = waitForFirstStart;
          break;
        default:
          ASSERT(0);
          break;
      }
    }
    else
    {
      // Timeout
      rxState = waitForFirstStart;
    }
  }
}

static int wifilinkReceiveCRTPPacket(CRTPPacket *p)
{
  if (xQueueReceive(crtpPacketDelivery, p, M2T(100)) == pdTRUE)
  {
    return 0;
  }

  return -1;
}

static int wifilinkSendCRTPPacket(CRTPPacket *p)
{
  int dataSize;

  ASSERT(p->size < SYSLINK_MTU);

  sendBuffer[0] = p->header;

  if (p->size <= CRTP_MAX_DATA_SIZE)
  {
    memcpy(&sendBuffer[1], p->data, p->size);
  }

  dataSize = p->size + 1;


  ledseqRun(LINK_DOWN_LED, seq_linkup);


  uart2SendData(dataSize, sendBuffer);

  return 0;    
  
}



static int wifilinkSetEnable(bool enable)
{
  return 0;
}

bool wifilinkTest(void)
{
  return isInit;
}

struct crtpLinkOperations * wifilinkGetLink()
{
  return &wifilinkOp;
}

