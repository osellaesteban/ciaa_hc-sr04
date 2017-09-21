/* Copyright 2017, Esteban Osella
 * Copyright 2017, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Date: 2017-08-25 */

#include "sapi_hc_sr04.h"         /* <= sAPI HC-S04 header */
#include "sapi_gpio.h"         	   /* <= sAPI I2C header */

HC_SR04_data_t  base_data;

/**
 * Function bool_t hc_sr04IsAlive( void )
 * Description: checks whether the HC-SR04 is operative or not
 * by trying to make a single data read
 */
bool_t hc_sr04IsAlive( void ){
      return (TRUE);
}

bool_t hc_sr04PrepareDefaultConfig( HC_SR04_config_t  * config ){

   config->HC_SR04_TRIG = GPIO0;
   config->HC_SR04_ECHO = GPIO1;
   config->HC_SR04_mode = HC_SR04_DEFAULT_mode;
   config->period = HC_SR04_DEFAULT_period;
   return (TRUE);
}

bool_t hc_sr04Config( HC_SR04_config_t  config ){
	bool_t result = FALSE;
	base_data->configuration->HC_SR04_ECHO = config->HC_SR04_ECHO;
	base_data->configuration->HC_SR04_TRIG = config->HC_SR04_TRIG;
	base_data->configuration->HC_SR04_mode = config->HC_SR04_mode;
	base_data->configuration->period = config->period;

	gpioConfig(config->HC_SR04_TRIG,GPIO_OUTPUT);
	gpioConfig(config->HC_SR04_ECHO	,GPIO_INPUT);
	if (config->HC_SR04_mode == HC_SR04_continuous_measurement)
	{
		/** configure a timer to call the hc_sr04Read() function every
		 * config->period us. Such function is responsible of the hw
		 * interface
		 * c
		 */
	}
	return result;
}

/**
 * First function that is called in order to get the operation started.
 */

bool_t hc_sr04Read(){
	bool_t result = FALSE;
	base_data->confiabillity = FALSE;
	base_data->listening = TRUE;
	base_data->t_beggins = 0; // Register here which is the listening mode begging time.

	//
	/**
	 * Tbd:
	 * Switch base_data->configuration->HC_SR04_ECHO gpio pin to TRUE for 10us and then
	 * back to false. Then, enable the interruption in base_data->configuration->HC_SR04_ECHO,
	 * and assign hc_sr04Listening() to such response.
	 * If after 23529us base_data->configuration->HC_SR04_ECHO has not received an
	 * interruption, then base_data->listening = FALSE;
	 */
	return result;
}
bool_t hc_sr04Listening()
{
	tickRead(base_data->t_beggins);
	/**
	 * Now, it has to be configured base_data->configuration->HC_SR04_ECHO gpio pin such that
	 *  hc_sr04Write() takes the control.
	 */

}
bool_t hc_sr04Write()
{
	/**
	 * This function is supposed to be responsible of writing the responses of the HC-SR04
	 * Is the one which should be activated when the hw interruption in
	 * base_data->configuration->HC_SR04_ECHO goes back to FALSE.
	 */
	tick_t actual_time;
	tickRead(actual_time);// this returns in us?? if not, must be converted to us in the
						  // assignment to meassurement
	bool_t result = FALSE;
	base_data->measurement = uint16_t(actual_time-base_data->t_beggins);
	base_data->dstmeassurement = base_data->measurement*0.34/2; // Time_high(us)*340m/s*1000mm/m*s/1e6us / 2
	if (base_data->measurement< HC_SR04_WINDOW_UPPER_LIMIT)
	{
		base_data->confiabillity = TRUE;
		result = TRUE;
	}
	return result;
}
