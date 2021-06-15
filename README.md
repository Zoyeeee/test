# test
```c
#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include <math.h>
//define Internal RC frequencies
#define XTAL 16000000UL


#define AHB_PRE 1
#define APB1_PRE 2
#define APB2_PRE 1
#define SysTickClk 10000


#define SYSCLK 72000000
#define AHB SYSCLK/AHB_PRE
#define APB1 AHB/APB1_PRE
#define APB1_TIM APB1*2
#define APB2 AHB/APB2_PRE
#define APB2_TIM APB2*1
#define SysTicks AHB/SysTickClk


#define USART_BAUDRATE 19200
#define BUFFER_LENGTH 3
uint16_t buffer_size=0,current_send_byte;
uint8_t buffer[BUFFER_LENGTH];

#define pi 3.14

void clock_set()
{
FLASH->ACR|=FLASH_ACR_PRFTEN;
FLASH->ACR|=FLASH_ACR_LATENCY_2WS;//FLASH PREFETCH
RCC->CFGR&=~RCC_CFGR_HPRE_Msk;
RCC->CFGR|=RCC_CFGR_HPRE_DIV1;
RCC->CFGR&=~RCC_CFGR_PPRE1_Msk;
RCC->CFGR|=RCC_CFGR_PPRE1_DIV2;
RCC->CFGR&=~RCC_CFGR_PPRE2_Msk;
RCC->CFGR|=RCC_CFGR_PPRE2_DIV1;//set AHB APB1 APB2
RCC->PLLCFGR&=~RCC_PLLCFGR_PLLM_Msk;
RCC->PLLCFGR|=8<<RCC_PLLCFGR_PLLM_Pos;
RCC->PLLCFGR&=~RCC_PLLCFGR_PLLN_Msk;
RCC->PLLCFGR|=72<<RCC_PLLCFGR_PLLN_Pos;
RCC->PLLCFGR&=~RCC_PLLCFGR_PLLP_Msk; //SET PLLM PLLN PLLP
RCC->CR|=RCC_CR_PLLON;//OPEN PLL
while((RCC->CR&RCC_CR_PLLRDY)==0)//WAIT PLL START
{}
RCC->CFGR&=(uint32_t)((uint32_t)~(RCC_CFGR_SW));
RCC->CFGR|=RCC_CFGR_SW_PLL;
while((RCC->CFGR&RCC_CFGR_SWS)!=RCC_CFGR_SWS_PLL)
{}
}

void usart2_Init(uint32_t baud)
{
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN|
	              RCC_AHB1ENR_GPIOBEN;//open clock
GPIOA->MODER =0x000000A0;//pa2 pa3 	Multiple function mode
GPIOA->AFR[0]=0x00007700;//
GPIOB->MODER=0x00005555;//GPIOB
RCC->APB1ENR |= RCC_APB1ENR_USART2EN;//usart2 enable
USART2->BRR = APB1/baud;//Baudrate
USART2->CR1 = USART_CR1_UE|//usart2enable
	            USART_CR1_TE|//
	            USART_CR1_RE|//transmit receive enable
	            USART_CR1_RXNEIE;//Interrupt enable
NVIC_EnableIRQ(USART2_IRQn);//Configuration interrupt	
}

void SendByte(uint8_t dat)
{
	USART2->DR = dat;
	while((USART2->SR & USART_SR_TC) == 0);
}

void Send_buffer(uint8_t *buff,uint16_t count)
{
	int i=0;
for (i=0;i<count;i++)
	{
		SendByte(*buff);
		*buff++;
	}
}

void delay_ms(uint16_t time)     
{    
  uint16_t i=0;    
  while(time--)     
  {    
    i=12000;    
    while(i--);    
  }    
}
void USART2_IRQHandler(void)
{
	if(USART2->SR & USART_SR_RXNE)
	{
		buffer[buffer_size] = USART2->DR;
		buffer_size++;
		if(buffer[buffer_size-1] == 'k')
		{
			GPIOB->ODR &= ~(1 <<0); //LEN0 OFF
			buffer_size = 0;
			USART2->CR1 &= ~USART_CR1_RE;
		}
		else if(USART2->DR == 'l')
		{
			GPIOB->ODR |= 1 << 0; // LED0 ON
			buffer_size = 0;
			USART2->CR1 &= ~USART_CR1_RE;
		}
	}
}
void TIM1_BRK_TIM9_IRQHandler(void)
{
TIM9->SR &=~TIM_SR_UIF;
	if(!(GPIOB->IDR & GPIO_IDR_ID9))
	{
	GPIOB->BSRR |= GPIO_BSRR_BS1;
	GPIOB->BSRR |= GPIO_BSRR_BR0;
	}
	else
	{
	GPIOB->BSRR |= GPIO_BSRR_BS0;
	GPIOB->BSRR |= GPIO_BSRR_BR1;
	}
}
void TIM1_UP_TIM10_IRQHandler(void)
{
  static uint8_t i=0;
	double s=sin(i*2*pi/100);
	TIM10->SR &=~TIM_SR_UIF;
	if(++i==100) i=0;
	buffer[0]='A';
	buffer[1]=i;
	buffer[2]=(int8_t)(s*100);
	if(1)
	{
	Send_buffer(buffer,3);
	}
}
void Init_tim9(uint16_t arr,uint16_t psc)
{
	//OPEN TIM9
	RCC->APB2ENR |=1<<16;
	TIM9->CNT=0;//CLEAR 
	TIM9->PSC = psc;
	TIM9->ARR = arr;
/*Allow update interrupt enable*/
	TIM9->DIER |= 1<<0;
/*Enable basic timer interrupt*/
  NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
/*Turn on counter enable*/
  TIM9->CR1 |= 1<<0;
}
void Init_tim10(uint16_t arr,uint16_t psc)
{
	//OPEN TIM10
	RCC->APB2ENR |=1<<17;
	TIM10->CNT=0;//CLEAR 
	TIM10->PSC = psc;
	TIM10->ARR = arr;
/*Allow update interrupt enable*/
	TIM10->DIER |= 1<<0;
/*Enable basic timer interrupt*/
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
/*Turn on counter enable*/
  TIM10->CR1 |= 1<<0;
}

int main()
{
	clock_set();
	usart2_Init(USART_BAUDRATE);
	Init_tim9(1000,72-1);
	Init_tim10(10000,72-1);
while(1)
{}
}


```
