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
#include "sapi.h"         	   /* <= sAPI I2C header */


// Tbd: consider the actual clock rate.
#define UP_TIME        2040 // 10us at 204MHz are 2040 ticks
#define TIMER0_PRESCALER   1// OJO QUE ESTO CUENTA TICKS CORRIENDO A 204MHz.
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
   config->HC_SR04_ECHO = TIMER_CAPTURE2;
   config->HC_SR04_mode = HC_SR04_DEFAULT_mode;
   config->period = HC_SR04_DEFAULT_period;
   return (TRUE);
}

bool_t hc_sr04Configure( HC_SR04_data_t  * base_data, HC_SR04_config_t * config ){
	bool_t result = FALSE;
	base_data->configuration.HC_SR04_ECHO = config->HC_SR04_ECHO;
	base_data->configuration.HC_SR04_TRIG = config->HC_SR04_TRIG;
	base_data->configuration.HC_SR04_mode = config->HC_SR04_mode;
	base_data->configuration.period = config->period;

	gpioConfig(config->HC_SR04_TRIG,GPIO_OUTPUT);
	gpioConfig(config->HC_SR04_ECHO	,GPIO_INPUT);
	gpioWrite(base_data->configuration.HC_SR04_TRIG , OFF );
	gpioWrite(base_data->configuration.HC_SR04_ECHO , OFF );
	if (config->HC_SR04_mode == HC_SR04_continuous_measurement)
	{
		if(config->period > HC_SR04_min_period)
			timerTickerConfig( TIMER1, config->period );
		else
		{
			timerTickerConfig( TIMER1, HC_SR04_min_period );
			/** ToDo: set a string with a message*/
		}
		timerTickerSetTickEvent( TIMER1, hc_sr04Rise );
		timerSetPower( TIMER1, ON );
	}
	timerTickerConfig( TIMER1, base_data->configuration.period);
			timerTickerSetTickEvent( TIMER1, hc_sr04Rise);
			timerSetPower( TIMER1, ON );
	uartWriteString(UART_USB,"hc_sr04Configure call --> gone\r\n");
	return result;
}

/**
 * First function that is called in order to get the operation started.
 */

bool_t hc_sr04Rise(HC_SR04_data_t  * base_data){
	bool_t result = FALSE;
	uartWriteString(UART_USB,"hc_sr04Rise call --> entrada\r\n");
	/** ToDo:La caga aca!!
	 s
	 */
	base_data->confiabillity = FALSE;
	uartWriteString(UART_USB,"hc_sr04Rise call --> confiability en false\r\n");
	base_data->listening = TRUE;

	gpioWrite(base_data->configuration.HC_SR04_TRIG , ON );

	timerTickerConfig( TIMER1, UP_TIME ); // Reconfigure the timer to fall down the trigger signal after 2
	timerTickerSetTickEvent( TIMER1, hc_sr04Fall );
	timerSetPower( TIMER1, ON );
	uartWriteString(UART_USB,"hc_sr04Rise call --> termino de levantar el pulso y  configurar la caida\r\n");
	timerInputCaptureConfig( TIMER0, base_data->configuration.HC_SR04_ECHO,
	                            TIMER0_PRESCALER,
	                            TRUE, FALSE );
	timerInputCaptureSetCaptureEvent( TIMER0, base_data->configuration.HC_SR04_ECHO,
			hc_sr04Write );
	timerSetPower( TIMER0, ON );


	base_data->t_beggins = timerReadCapture( TIMER0, base_data->configuration.HC_SR04_ECHO ); // Register here which is the listening mode begging time.
	uartWriteString(UART_USB,"t_beggins:");
	uartWriteString(UART_USB,base_data->t_beggins);
	uartWriteString(UART_USB,"\r\n");
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
bool_t hc_sr04Fall(HC_SR04_data_t  * base_data){
	base_data->listening = TRUE;
	//base_data->t_beggins = 0; // Register here which is the listening mode begging time.
	gpioWrite(base_data->configuration.HC_SR04_TRIG , OFF );
	timerTickerConfig( TIMER1, HC_SR04_WINDOW_UPPER_LIMIT );
	timerTickerSetTickEvent( TIMER1, hc_sr04SwitchListeningOff);
	timerSetPower( TIMER1, ON );
}
/**
 * If nothing was listen, then  disable the listening and eventually, reconfigure the timer
 *  to trigger for a new measurement.
 */
bool_t hc_sr04SwitchListeningOff(HC_SR04_data_t  * base_data)
{
	bool_t result = FALSE;
	base_data->listening = FALSE;
		//base_data->t_beggins = 0; // Register here which is the listening mode begging time.
	//gpioWrite(base_data->configuration->HC_SR04_TRIG, OFF);
	timerSetPower( TIMER1, OFF );
	if(base_data->configuration.HC_SR04_mode == HC_SR04_continuous_measurement)
	{
		timerTickerConfig( TIMER1, base_data->configuration.period);
		timerTickerSetTickEvent( TIMER1, hc_sr04Rise);
		timerSetPower( TIMER1, ON );
	}
	return result;

}

/**
 * hc_sr04Write(HC_SR04_data_t  * base_data)
 * is used by the timer to eventually write the
 */
bool_t hc_sr04Write(HC_SR04_data_t  * base_data)
{
	bool_t result = FALSE;
	if (base_data->listening)
	{
		uint32_t actual_time = timerReadCapture( TIMER0, base_data->configuration.HC_SR04_ECHO ); // Register here which is the listening mode begging time.;
		base_data->measurement = actual_time-base_data->t_beggins;
		base_data->dstmeassurement = base_data->measurement*0.34/(2*204000000); // Time_high(us)*340m/s*1000mm/m*s/1e6us / 2
		//consolePrintString("Distancia en ticks: ");
		//consolePrintUInt(base_data->measurement);
		//consolePrintString("\r\n");
		if (base_data->measurement< HC_SR04_WINDOW_UPPER_LIMIT)
		{
			base_data->confiabillity = TRUE;
			result = TRUE;
		}
	}
	return result;
}
