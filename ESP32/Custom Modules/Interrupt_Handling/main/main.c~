/*
 * Test interrupt handling on a GPIO.
 * In this fragment we watch for a change on the input signal
 * of GPIO 21.  When it goes high, an interrupt is raised which
 * adds a message to a queue which causes a task that is blocking
 * on the queue to wake up and process the interrupt.
 */

#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include <time.h>
#include <sys/time.h>



static char tag[] = "test_interrupt"; 
static QueueHandle_t q1; 
struct timeval tvalBefore, tvalAfter;
long int timeIntervalUs;

#define TEST_GPIO (21)
 
static void handler(void *args) {   
	gpio_num_t gpio;   
	gpio = TEST_GPIO;   
	xQueueSendToBackFromISR(q1, &gpio, NULL); 
} 

void test1_task(void *ignore) {   
	ESP_LOGD(tag, ">> test1_task"); 
	gpio_num_t gpio;   
	q1 = xQueueCreate(10, sizeof(gpio_num_t));      

	gpio_config_t gpioConfig;   
	gpioConfig.pin_bit_mask = GPIO_SEL_21;   
	gpioConfig.mode = GPIO_MODE_INPUT;   
	gpioConfig.pull_up_en = GPIO_PULLUP_ENABLE;   
	gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;   
	gpioConfig.intr_type = GPIO_INTR_POSEDGE;   
	gpio_config(&gpioConfig);   

	gpio_install_isr_service(0);   
	gpio_isr_handler_add(TEST_GPIO, handler, NULL);   
	while(1) {      
		ESP_LOGD(tag, "Waiting on queue");     
		BaseType_t rc = xQueueReceive(q1, &gpio, portMAX_DELAY);
		ESP_LOGD(tag, "Woke from queue wait: %d", rc);
		gettimeofday (&tvalAfter, NULL);
		timeIntervalUs = ((tvalAfter.tv_sec - tvalBefore.tv_sec)*1000000L
           +tvalAfter.tv_usec) - tvalBefore.tv_usec;
		printf("Time in microseconds: %ld microseconds\n",timeIntervalUs);
		printf("Frequency in Hz: %.2f Hz\n",1000000.0/(timeIntervalUs));
		gettimeofday (&tvalBefore, NULL);
	}   
	vTaskDelete(NULL); 
} 

void app_main(void)
{
    xTaskCreate(&test1_task, "test1_task", 2048, NULL, 5, NULL);
}

