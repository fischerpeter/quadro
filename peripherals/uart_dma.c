#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "core_cm4.h"
#include "system_stm32f4xx.h"


#define COM1 			USART1
#define COM1_GPIO		GPIOB
#define COM1_CLK		RCC_APB2Periph_USART1
#define COM1_GPIO_CLK	RCC_APB2Periph_GPIOB
#define TxPin			GPIO_Pin_6


void RCC_Configuration(void)
{
  /* Enable DMA clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  /* Enable GPIOC bank clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  /* Enable UART clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

  //remap pin, wegen board
  //GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
}

/******************************************************************************/

void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART Tx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = TxPin;  // PA.09 USART1.TX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure USART Rx as input floating */
  /*
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // PA.10 USART1.RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  */
}

/******************************************************************************/

void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;

  /* USART resources configuration (Clock, GPIO pins and USART registers) ----*/
  /* USART configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 57600;//115200
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx;

  /* USART configuration */
  USART_Init(USART1, &USART_InitStructure);

  /* Enable the USART1 */
  USART_Cmd(USART1, ENABLE);
}

/******************************************************************************/

void DMA_Configuration(void)
{
  // USART1 Channel 4, USART2 Channel 7, USART3 Channel 2

  DMA_DeInit(DMA1_Channel4);
  DMA_Cmd(DMA1_Channel4, DISABLE);
}

/******************************************************************************/

void DMA_USART_String(char *s)
{
  DMA_InitTypeDef DMA_InitStructure;

    /* Disable the DMA Controller/Channel */
    DMA_DeInit(DMA1_Channel4);

  /* USART1_Tx_DMA_Channel (triggered by USART1 Tx event) Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)s;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = strlen(s);
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  DMA_Init(DMA1_Channel4, &DMA_InitStructure);

    /* Enable the DMA Controller/Channel */
  DMA_Cmd(DMA1_Channel4, ENABLE);

  /* Enable USART1 DMA TX request */
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    /* Wait of DMA completion */
    while(DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET);

  /* Loop until the end of transmit */
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}


void SendString(char str[]) {
   DMA_USART_String(str);
}
/******************************************************************************/

void StartUART(void)
{

  RCC_Configuration();
  GPIO_Configuration();
  USART_Configuration();
  DMA_Configuration();

}


