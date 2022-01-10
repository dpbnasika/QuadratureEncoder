#include "stdio.h"
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "pthread.h"
#include "quadrature_encoder.h"

using namespace std;

extern "C"
{
void app_main();
}

QuadratureEncoder encoder((gpio_num_t)32,(gpio_num_t)33);

void forward(void *pvParameter){
	
	int direction = encoder.getDirection();
    	if(direction == 1){
    		cout << "Forward RPM:  " << encoder.getRPM() << endl;
    	}
    	vTaskDelay(10);
}
void forward_loop(void *pvParameter)
{
    while(true){
    	forward(pvParameter);
    }
}
void backward(void *pvParameter)
{
    while(true){
    	int direction = encoder.getDirection();
    	if(direction == -1){
    		cout << "Backward RPM: "<< (encoder.getRPM()*-1) << endl;
    	}
    	vTaskDelay(10);
    }
}
void idle(void *pvParameter)
{
    while(true){
    	int direction = encoder.getDirection();
    	if(direction == 0){
    		cout << "Idle RPM: "<< (encoder.getRPM()*0) << endl;
    	}
    	vTaskDelay(10);
    }
}

void app_main(void)
{
	cout << "testing encoder library" << endl;

	encoder.init();

	xTaskCreate(&forward, "forward_loop", 2048, NULL, 5, NULL);
	xTaskCreate(&backward, "backward_loop", 2048, NULL, 5, NULL);
	xTaskCreate(&idle, "ide_loop", 2048, NULL, 5, NULL);

}

