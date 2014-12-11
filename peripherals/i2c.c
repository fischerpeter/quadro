#include "stm32f4xx.h"
#include "core_cm4.h"
#include "system_stm32f4xx.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"

//#include "minimu9v2.h"

#define I2C1_INTR_PRIO 0
#define I2C1_INTR_SUBPRIO 1

#define SLAVE_ADDRESS 0xD4

#define STM32F4xx

int i2c_started = 0;



void I2C1_Init(void){					// http://www.mikrocontroller.net/topic/323702


  GPIO_InitTypeDef GPIO_InitStruct;
  I2C_InitTypeDef I2C_InitStruct;


  I2C_DeInit(I2C1);


  #ifdef STM32F10x
    // enable APB1 peripheral clock for I2C1, SCL and SDA pins
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |  RCC_APB2Periph_AFIO,ENABLE);

    // Setup SCL und SDA pins
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_PinRemapConfig(GPIO_Remap_I2C1,DISABLE);

  #elif defined(STM32F4xx)
    // enable APB1 peripheral clock for I2C1, SCL and SDA pins
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // Setup SCL und SDA pins
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;   // 6,7  / 8,9
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    //GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;  // original
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOB, &GPIO_InitStruct);         // init GPIOB

    // Connect I2C1 pins to AF
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1); // SCL
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA
  #else
    #error "No MCU defined"
  #endif

    // configure I2C1
    I2C_InitStruct.I2C_ClockSpeed = 400000;     // Hz
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStruct);


    I2C_Cmd(I2C1, ENABLE);
    //i2c_status = i2c_ok;

    I2C_ClearFlag(I2C1, I2C_FLAG_AF);
    I2C_ClearFlag(I2C1, I2C_FLAG_ARLO);
    I2C_ClearFlag(I2C1, I2C_FLAG_BERR);


}
