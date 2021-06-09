# test
```c
#include "stm32f4xx.h"


#define XTAl	
#define HSE_STARTUP_TIMEOUT ((uint16_t)0xFFFF)

#define	AHB_PRE			1
#define APB1_PRE		2
#define APB2_PRE		1
#define SysTicksClk 10000

#define	SYSCLK			72000000
#define	AHB					SYSCLK/AHB_PRE//72
#define APB1				AHB/APB1_PRE//36
#define APB1_TIM		APB1*2
#define APB2				AHB/APB2_PRE//72
#define APB2_TIM		APB2*1
#define SysTicks		AHB/SysTicksClk
#define USART2_MAX_RECV_LEN 400


void ChangeLED(unsigned counter);
static void setHSE(void);
void Init1(void);
void USART2_IRQHandler(void);
void Send_buffer(uint8_t *buff);
void SendByte(uint8_t dat);

uint8_t USART2_RX_BUF[USART2_MAX_RECV_LEN];
uint8_t Rx_data_counter = 0;


int main()
{
	//unsigned counter=0x0000;
	//__IO uint32_t StartUpCounter= 0,HSEStatus=0;
	
	//Enable HSE
	setHSE();
	Init1();
	//
	while(1)
	{
		if (!(GPIOB->IDR & GPIO_IDR_ID0))
		{
			Send_buffer("13");
			USART2->CR1 |= USART_CR1_RE;
			GPIOA->ODR |= (1 << 0);
			while((!(GPIOB->IDR & GPIO_IDR_ID0)));
			//GPIOA->ODR |= 1 << 0; // LED0 ON
			//ChangeLED(counter+1);
		}
		else if (!(GPIOB->IDR & GPIO_IDR_ID1))
		{
			Send_buffer("12");
			USART2->CR1 |= USART_CR1_RE;
			GPIOA->ODR &= (0 << 0); 
			while((!(GPIOB->IDR & GPIO_IDR_ID1)));
			//GPIOA->ODR &= ~(1 << 1); // LED0 Off
			//ChangeLED(counter);
		}		
	}
}

void ChangeLED(unsigned counter)
{
	GPIOA->ODR |= counter;
	GPIOA->ODR &= counter;
}


static void setHSE(void)
{
	if ((RCC->CR & RCC_CR_HSIRDY) != RESET) {
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |RCC_AHB1ENR_GPIOBEN |RCC_AHB1ENR_GPIODEN;
	//GPIOD->MODER |= 1 << 2 * 2;// Set PD2 to output mode++++++++
	// Enable FLASH Prefetch
	FLASH->ACR |= FLASH_ACR_PRFTEN;//Prefetch enable
	FLASH->ACR |= FLASH_ACR_LATENCY_2WS;//Latency
		//RCC clock configuration
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;//AHBdiv1=72
	RCC->CFGR &= ~RCC_CFGR_PPRE1_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;//AHB1div2=36
	RCC->CFGR &= ~RCC_CFGR_PPRE2_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;//AHB2div1=72
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Msk;
	RCC->PLLCFGR |= 8 << RCC_PLLCFGR_PLLM_Pos;//VCOinput=PLLinput/PLLM=16/8=2  BestValue,which is recommended value
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_Msk;
	RCC->PLLCFGR |= 72*2 << RCC_PLLCFGR_PLLN_Pos;//VCOoutput=VCOinput*PLLN=2*36=72;but need div2;	36*Latency=36*2=72
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP_Msk;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_0;//
	RCC->CR |= RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) {}
	// Set SysClk from PLL
	RCC->CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_SW));/*!< SW[1:0] bits (System clock Switch) */
	RCC->CFGR |= RCC_CFGR_SW_PLL;/*!< PLL selected as system clock */
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)
		{}
	//SysTick_Config(SysTicks);
	}
}

void Init1(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;/*Peripheral clock enable and enable uart2,P2,P3*/
	GPIOA->MODER |=0x00000005;
	GPIOA->MODER |= 1<<(2*2+1);//2
	GPIOA->MODER |= 1<<(2*3+1);//3
	//GPIOD->MODER |=1<<(2*2);
	GPIOA->AFR[0] |= 0x7700;
	//GPIOA->AFR[1] |= 0x00000770;
	USART2->BRR=APB1/19200; //USART_BOUNDRATE
	USART2->CR1 = USART_CR1_UE| //Enable UART1
	USART_CR1_TE | //Enable Transmitter
	USART_CR1_RE | //Enable Receiver
	USART_CR1_RXNEIE; //Enable interrupt in peripheral
	NVIC_EnableIRQ(USART2_IRQn); //Enable Global interrupt
}

void USART2_IRQHandler(void)
{
	if(USART2->SR & USART_SR_RXNE)
	{
		USART2_RX_BUF[Rx_data_counter] = USART2->DR;
		Rx_data_counter++;
		if(USART2_RX_BUF[Rx_data_counter-1] == '13')
		{
			//ChangeLED(counter+1);
			GPIOA->ODR |= (1 << 0); 
			Rx_data_counter = 0;
			USART2->CR1 &= ~USART_CR1_RE;
		}
		else if(USART2->DR == '12')
		{
			//ChangeLED(counter);
			GPIOA->ODR &= (0 << 0); 
			Rx_data_counter = 0;
			USART2->CR1 &= ~USART_CR1_RE;
		}
	}
}

void Send_buffer(uint8_t *buff)
{
	while(*buff != '\0')
	SendByte(*buff++);
}

void SendByte(uint8_t dat)
{
	USART2->DR = dat;
	while((USART2->SR & USART_SR_TC) == 0);
}



```
