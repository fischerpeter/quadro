#include "stm32f4xx.h"

#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

#include "led.h"

#include "FreeRTOS.h"
#include "task.h"

#include "PWM.h"





void Timer1_PWMInit();


void vPWMTask(void *pvParameters)
{

	const portTickType xDelay = 20 / portTICK_RATE_MS; //zahl ist die periode in ms
	portTickType xLastWakeTime;

	Timer1_PWMInit();

	int mitte = pulsemin+(pulsemax-pulsemin)/2;

	pwm_ch1 = 0;
	pwm_ch2 = mitte;
	pwm_ch3 = mitte;
	pwm_ch4 = mitte;
	while(1)
    {
		//Board_LedOn(2);

		xLastWakeTime = xTaskGetTickCount();


		//pwm_ch1 = 0;

		/*/--- 0-100 %
		TIM2->CCR1 = pulsemin+((pulsemax-pulsemin)/100*pwm_ch1);  // pwm_ch in prozent, x/100*2500 = x*25  (2500 entspricht 2)
		TIM2->CCR2 = pulsemin+((pulsemax-pulsemin)/100*pwm_ch2);
		TIM2->CCR3 = pulsemin+((pulsemax-pulsemin)/100*pwm_ch3);
		TIM2->CCR4 = pulsemin+((pulsemax-pulsemin)/100*pwm_ch4);
		//*/
		//*/
		TIM2->CCR1 = pwm_ch1;
		TIM2->CCR2 = pwm_ch2;
		TIM2->CCR3 = pwm_ch3;
		TIM2->CCR4 = pwm_ch4;
		//*/


		//Board_LedOff(2);
		vTaskDelayUntil( &xLastWakeTime, xDelay);
    }
}




// STM32F107 PWM1 TIM1_CH3N(PE12) TIM1_CH3(PE13) - sourcer32@gmail.com



/**************************************************************************************/

void Timer1_setClocks(void)
{
  /* TIM1, GPIOE and AFIO clocks enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

/**************************************************************************************/

void Timer1_setGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOE Configuration: Channel 1-4  as alternate function push-pull */
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* TIM1 Full remapping pins (to PE pin bank) */ // Wenn man Port E benutzen wollen würde (Sind auf Leds)
  //GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);
}

/**************************************************************************************/

void Timer1_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 71 - 1; // 1MHz timebase from 72 MHz bus
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = periode - 1; // 50 Hz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  //TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Channel 3 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
  TIM_OCInitStructure.TIM_Pulse = pulsemax; // 25/75 for PE13, 75/25 for PE12

  TIM_OC1Init(TIM2, &TIM_OCInitStructure); // Channel 1
  TIM_OC2Init(TIM2, &TIM_OCInitStructure); // Channel 2
  TIM_OC3Init(TIM2, &TIM_OCInitStructure); // Channel 3
  TIM_OC4Init(TIM2, &TIM_OCInitStructure); // Channel 4

  /* TIM1 counter enable */
  TIM_Cmd(TIM2, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

/**************************************************************************************/
void Tim2_diller() {
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
	TIM_OCInitTypeDef TIM_OC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2 );

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBase_InitStructure.TIM_Period = periode;
	TIM_TimeBase_InitStructure.TIM_Prescaler = 84 -1;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBase_InitStructure);

	TIM_OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC_InitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC_InitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC_InitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OC_InitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_InitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OC_InitStructure.TIM_Pulse = pulsemax;
	TIM_OC1Init(TIM2, &TIM_OC_InitStructure);

	TIM_Cmd(TIM2, ENABLE);
}

void Timer1_PWMInit()
{
	Tim2_diller();
	//Timer1_setClocks();

	//Timer1_setGPIO();

	//Timer1_Configuration();

}

