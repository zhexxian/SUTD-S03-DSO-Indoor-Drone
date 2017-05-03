#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include <esp_log.h>

static char tag[] = "test_intr"; 
static QueueHandle_t q1; 

#define TEST_GPIO (21) 
static void handler(void *args) {   
	gpio_num_t gpio;   
	gpio = TEST_GPIO;   
	xQueueSendToBackFromISR(q1, &gpio, NULL); 
} 

void test1_task() {   
	ESP_LOGD(tag, ">> test1_task");   
	gpio_num_t gpio;   
	q1 = xQueueCreate(10, sizeof(gpio_num_t));      

	gpio_config_t gpioConfig;   
	gpioConfig.pin_bit_mask = GPIO_SEL_25;   
	gpioConfig.mode = GPIO_MODE_INPUT;   
	gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;   
	gpioConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;   
	gpioConfig.intr_type = GPIO_INTR_POSEDGE;   
	gpio_config(&gpioConfig);   

	gpio_install_isr_service(0);   
	gpio_isr_handler_add(TEST_GPIO, handler, NULL);   
	while(1) {      
		ESP_LOGD(tag, "Waiting on queue");      
		BaseType_t rc = xQueueReceive(q1, &gpio, portMAX_DELAY);      
		ESP_LOGD(tag, "Woke from queue wait: %d", rc);   
	}   
	vTaskDelete(NULL); 
} 

void app_main(void)
{
    nvs_flash_init();

	test1_task();
}

