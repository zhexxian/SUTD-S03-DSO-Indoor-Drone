#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include <string.h>

#include "sdkconfig.h"

static char tag[] = "socketClient";

void socketClient() {
	ESP_LOGD(tag, "start");
	int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	ESP_LOGD(tag, "socket: rc: %d", sock);
	struct sockaddr_in serverAddress;
	serverAddress.sin_family = AF_INET;
	inet_pton(AF_INET, "192.168.0.103", &serverAddress.sin_addr.s_addr);
	serverAddress.sin_port = htons(9999);

	int rc = connect(sock, (struct sockaddr *)&serverAddress, sizeof(struct sockaddr_in));
	ESP_LOGD(tag, "connect rc: %d", rc);

	char *data = "Hello FireFly CrazyFlie ground control!";
	rc = send(sock, data, strlen(data), 0);
	ESP_LOGD(tag, "send: rc: %d", rc);

	rc = close(sock);
	ESP_LOGD(tag, "close: rc: %d", rc);

	vTaskDelete(NULL);
}

int app_main(void)
{

	socketClient();
	return 0;
}
