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

#ifndef _SAPI_HC_SR04_H_
#define _SAPI_HC_SR04_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/




/*==================[typedef]================================================*/
#define HC_SR04_min_period 16666
#define HC_SR04_DEFAULT_period 16666 // in us
#define HC_SR04_WINDOW_UPPER_LIMIT 23529


typedef enum {
	HC_SR04_continuous_measurement = 0,
	HC_SR04_single_measurement = 1,
	HC_SR04_idle = 2,
	HC_SR04_DEFAULT_mode = HC_SR04_single_measurement
} HC_SR04_mode_t;

typedef struct{
	uint8_t HC_SR04_TRIG; //gpio output for triggering signal
	uint8_t HC_SR04_ECHO; //gpio input for echo signal
	HC_SR04_MODE_t HC_SR04_mode; // default: single measurement
	uint16_t period; // sampling period (if configured as continuous reading)
} HC_SR04_config_t;

typedef struct {
	HC_SR04_config_t configuration; // sampling configuration
	uint16_t measurement; //last read value. Used 16bits 'cos based on the
						  // dynamic range of the sensor (~1333 measurements)
	float32_t dstmeassurement; // the distance in mm
	bool_t confiabillity; // true if the measurement was correctly obtained, false otherwise
	bool_t listening;
	tick_t t_beggins; // time instant in which the trigger pulse is on;

} HC_SR04_data_t ;


/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

bool_t hc_sr04IsAlive(void);
bool_t hc_sr04PrepareDefaultConfig( HC_SR04_config_t  * config );
bool_t hc_sr04Config( HC_SR04_config_t  config );
bool_t hc_sr04Read();
bool_t hc_sr04Write();
bool_t hc_sr04Listening();

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef _SAPI_HMC5883L_H_ */
