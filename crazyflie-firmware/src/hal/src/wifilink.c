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
#include "crtp.h"
#include "configblock.h"
#include "log.h"
#include "led.h"
#include "ledseq.h"
#include "queuemonitor.h"
#include "pm.h"

#include "crtp.h"
#include "crtpservice.h"

#define WIFILINK_TX_QUEUE_SIZE (1)

static xQueueHandle crtpPacketDelivery;

static bool isInit;

static int wifilinkSendCRTPPacket(CRTPPacket *p);
static int wifilinkSetEnable(bool enable);
static int wifilinkReceiveCRTPPacket(CRTPPacket *p);


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

  // Initialize the USB peripheral
  usbInit();

  crtpPacketDelivery = xQueueCreate(5, sizeof(CRTPPacket));
  DEBUG_QUEUE_MONITOR_REGISTER(crtpPacketDelivery);

  ASSERT(crtpPacketDelivery);

  //syslinkInit();

  isInit = true;
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

  return usbSendData(dataSize, sendBuffer);
  /*
  static SyslinkPacket slp;

  ASSERT(p->size <= CRTP_MAX_DATA_SIZE);

  slp.type = SYSLINK_WIFI_RAW;
  slp.length = p->size + 1;
  memcpy(slp.data, &p->header, p->size + 1);

  if (xQueueSend(txQueue, &slp, M2T(100)) == pdTRUE)
  {
    return true;
  }

  return false;
  */
}



static int wifilinkSetEnable(bool enable)
{
  return 0;
}

bool wifilinkTest(void)
{
  return syslinkTest();
}

struct crtpLinkOperations * wifilinkGetLink()
{
  return &wifilinkOp;
}

