/*
 * 	Version 1.0 of Quadrature Encoder library with A B I pulses works with any ABI encoder.
 * 	Changes required if the resolution of encoder is different
 *
 * 	Functionalities Included:
 *
 * 	1. Provides Encoder Direction
 * 	2. Provides Encoder RPM
 * 	3. Provides Encoder Absolute Position ( 0° - 360° degrees )
 *
 *  References:
 *
 *  1. Quadrature Encoder Matrix: https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf
 *  2. Calculating RPM from Pulses: https://www.quantumdev.com/calculating-output-frequency-in-rotary-encoders/.
 *  3. Few parts of the functionality and structure is adapted(https://github.com/zacsketches/Encoder)and rest all are modified and extended based on our requirements and needs.
 *
 *  Created on: 19 July 2021
 *      Author: dp
 */

#ifndef QUADRATURE_H
#define QUADRATURE_H

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"


#define LOW             0
#define HIGH            1

//Define the resolution of the encoder here
//Change the resolution of the encoder here in case if the encoder is different from the below
#define ENCODER_A_PPR   2048
#define ENCODER_AB_PPR  (ENCODER_A_PPR * 2)

//Quadrature Encoder Matrix array to get the encoder direction
const int QEM[16] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};

//QuadratureEncoder class
class QuadratureEncoder
{
public:

	//Constructor
	QuadratureEncoder(gpio_num_t A, gpio_num_t B)
	{
		encoder_pinA = A;
		encoder_pinB = B;
	};

	void init();

	//Returns count of the Channel A and B pulses;
	long pulsecount()
	{
		return pulse_count;
	}

	int getDirection();

	int getRPM();

	int getAbsolutePosition();

	//Reverse the direction in case forward is backward and backward is forward
	void reverse()
	{
		rev = !rev;
	}

	double mapLinear(double x, double inMin, double inMax, double outMin, double outMax);

private:

	static volatile int get_enc_A_state;
	static volatile int get_enc_B_state;
	static volatile int out_val, old_reading, new_reading;
	static volatile long pulse_count;
	static long old_pulse_count;
	static bool rev;

	static long previous_milliseconds;
	static long current_milliseconds;

	static int time_interval;
	static volatile long encoder_A_pulses;
	static volatile long encoder_A_position_pulses;

	gpio_num_t encoder_pinA;
	gpio_num_t encoder_pinB;
	static int rpm;

	//Interrupt service routines
	static void isrChannelA(void* arg);
	static void isrChannelB(void* arg);

};

volatile int QuadratureEncoder::get_enc_A_state = 0;
volatile int QuadratureEncoder::get_enc_B_state = 0;
volatile int QuadratureEncoder::out_val = 0;
volatile int QuadratureEncoder::old_reading = 0;
volatile int QuadratureEncoder::new_reading = 0;
volatile long QuadratureEncoder::pulse_count = 0;
long QuadratureEncoder::old_pulse_count = 0;
bool QuadratureEncoder::rev = false;
long QuadratureEncoder::previous_milliseconds = 0;
long QuadratureEncoder::current_milliseconds = 0;
int QuadratureEncoder::time_interval = 1000; // unit: milliseconds
volatile long QuadratureEncoder::encoder_A_pulses = 0;
volatile long QuadratureEncoder::encoder_A_position_pulses = 0;
int QuadratureEncoder::rpm = 0;

/**
 * @breif      Initializes Quadrature Encoder channel A and B pins
 * @param      void
 * @return     void
 */
void QuadratureEncoder::init()
{

	gpio_pad_select_gpio(encoder_pinA);
	gpio_pad_select_gpio(encoder_pinB);

	gpio_set_direction(encoder_pinA, GPIO_MODE_INPUT);
	gpio_set_direction(encoder_pinB, GPIO_MODE_INPUT);

	gpio_set_intr_type(encoder_pinA, GPIO_INTR_ANYEDGE);
	gpio_set_intr_type(encoder_pinB, GPIO_INTR_ANYEDGE);

	gpio_install_isr_service(0);

	get_enc_A_state = gpio_get_level(encoder_pinA);
	get_enc_B_state = gpio_get_level(encoder_pinB);

	gpio_isr_handler_add(encoder_pinA, QuadratureEncoder::isrChannelA, NULL);
	gpio_isr_handler_add(encoder_pinB, QuadratureEncoder::isrChannelB, NULL);

	previous_milliseconds = esp_timer_get_time();
	encoder_A_pulses = 0;
	encoder_A_position_pulses = 0;
}

/**
 * @breif      Interrupt service routine for channel A
 * @param      void
 * @return     direction (+1.-1,0) (forward, reverse, stopped) respectively
 */
int QuadratureEncoder::getDirection()
{
	int direction=0;
	long new_pulse_count = pulsecount();
	long delta = new_pulse_count - old_pulse_count;
	if(delta > 0) {
		direction = 1;
	}
	else if (delta < 0){
		direction = -1;
	}
	else if (delta == 0){
		direction = 0;
	}

	old_pulse_count = new_pulse_count;

	return direction;
}

/**
 * @breif      Interrupt service routine for channel A
 * @param      void*
 * @return     void
 */
void IRAM_ATTR QuadratureEncoder::isrChannelA(void* arg)
{
	encoder_A_pulses++;

	old_reading = new_reading;
	get_enc_A_state = !get_enc_A_state;

	if(rev){
		new_reading = get_enc_A_state * 2 + get_enc_B_state;
	}
	else {
		new_reading = get_enc_B_state * 2 + get_enc_A_state;
	}

	out_val = QEM[old_reading * 4 + new_reading];

	switch(out_val){
	case 1:
		++pulse_count;
		break;
	case -1:
		--pulse_count;
		break;
	}
}

/**
 * @breif      Interrupt service routine for channel B
 * @param      void*
 * @return     void
 */
void IRAM_ATTR QuadratureEncoder::isrChannelB(void* arg)
{
	old_reading = new_reading;
	get_enc_B_state = !get_enc_B_state;

	if(rev){
		new_reading = get_enc_A_state * 2 + get_enc_B_state;
	}
	else {
		new_reading = get_enc_B_state * 2 + get_enc_A_state;
	}

	out_val = QEM[old_reading * 4 + new_reading];

	switch(out_val){
	case 1:
		++pulse_count;
		break;
	case -1:
		--pulse_count;
		break;
	}
}
/**
 * @breif      measures quadrature encoder RPM
 * 			   Revolutions per minute (RPM) = (total encoder pulse in 1s / motor encoder output) x 60s
 * @param      void
 * @return     quadrature encoder RPM
 */
int QuadratureEncoder::getRPM(){

	current_milliseconds = esp_timer_get_time();
	if (current_milliseconds - previous_milliseconds > time_interval)
	{
		previous_milliseconds = current_milliseconds;
		rpm = (float)(encoder_A_pulses * 60 / ENCODER_A_PPR);
		encoder_A_pulses = 0;
	}
	return rpm;
}

/**
 * @breif      measures quadrature encoder absolute position
 * @param      void
 * @return     quadrature encoder absolute position (0° degrees - 360° degrees)
 */
int QuadratureEncoder::getAbsolutePosition(){

	encoder_A_position_pulses = pulse_count;
	int degrees = (encoder_A_position_pulses * 360)/ENCODER_AB_PPR;

	if(pulse_count > ENCODER_AB_PPR){
		pulse_count = 0;
	}
	if(pulse_count < -ENCODER_AB_PPR){
		pulse_count = 0;
	}
	if(degrees < 0){
		degrees = mapLinear(degrees, -360, 0, 0, 360);
	}

	return degrees;
}

/**
 * @breif      Linear mapping function
 * @param      x      -  input value
 * @param      inMin  -  minimum input
 * @param      inMax  -  maximum input
 * @param      outMin -  minimum output
 * @param      outMax -  maximum output
 * @return     Linearly mapped value from the given inputs
 */
double QuadratureEncoder::mapLinear(double x, double inMin, double inMax, double outMin, double outMax)
{
	return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

#endif
