#include "regler.h"

#include "FreeRTOS.h"
#include "task.h"

#include "minimu9v2.h"

#include "stm32f4xx.h"
#include "core_cm4.h"
#include "system_stm32f4xx.h"

#include "uart_et.h"

#include "PWM.h"
#include "dac.h"

#include <stdio.h>



//----- Kaskadenregler für Winkelgeschwindigkeit

void vRegler(void *pvParameters)
{
	const portTickType xDelay = 10 / portTICK_RATE_MS; //zahl ist die periode in ms
	portTickType xLastWakeTime;


	float soll = 0;

	float Kp = 0.05;	//Kp ... irgendwas 5
	float Kd = 0.00;	//Kd ... irgendwas 2
	float Ki = 0.02;	//Ki  			1

	float epsilon = 0.01; //Threshold

	float Ta = 0.01; 	//aufweckrate (für D-Anteil)

	float p_anteil, i_anteil, d_anteil;
	float error, error_integral, error_old;

	error_old = 0;

	regler_start = false;

	float pwm_x = 1720;

	//int pwm_ch1_offset = 72;
	//pwm_ch1 = 0;

	//int p = 5;   // änderungsrate für P-Teil , in % vom gesamt auschlag


	DAC_Initialisation();

	while(1)
    {
		xLastWakeTime = xTaskGetTickCount();


		if (regler_start)
		{
			//----- Abweichung

			error = soll - att.Yrate;

			//-----P Anteil
			p_anteil = error*Kp;

			//*-----I-Anteil
			if ((error*error)> (epsilon*epsilon))
			{
				error_integral = error_integral + error*Ta;
			}
			i_anteil = error_integral * Ki;
			//*/

			//-----D-Anteil
			d_anteil = (error_old-error)/Ta * Kd;
			error_old = error;


			//---- Zusammensetzen
			pwm_x -= p_anteil; // Minus wegen richtung
			pwm_x += i_anteil;
			pwm_x += d_anteil;

			if (pwm_x > pulsemax){
				pwm_x=pulsemax;
				error_integral -= error;
			}
			if (pwm_x < pulsemin	 ) {
				pwm_x=pulsemin;
				error_integral -= error;
			}

			//USART_send3values (att.Yrate, att.pitch, pwm_ch1);
			USART_send3values (p_anteil*100, i_anteil*100, d_anteil*100);
			pwm_ch1=(int) pwm_x;



					/*/-----P regelung
					if (error==0) pwm_ch1 = pwm_ch1_offset;
					if (error>0) {
						pwm_ch1 = pwm_ch1_offset-p;

						//DAC_set(pwm_ch1);
					}
					if (error<0) {
						pwm_ch1 = pwm_ch1_offset+p;

						//DAC_set(pwm_ch1);
					}

					//*/
					DAC_set(att.pitch);



		}
		vTaskDelayUntil( &xLastWakeTime, xDelay);
    }
}
