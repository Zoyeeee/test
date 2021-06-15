# test
```c
#include "stm32f4xx.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_gpio.h"
#include "math.h"

//define UART Baudrate
#define USART_BAUDRATE		19200

// define busses presscalers High-speed APB (APB2) and low-speed APB (APB1)
#define AHB_PRE 		1				// 1, 2, 4, 8, 16, 64, 128, 256, 512
#define APB1_PRE		2				// 1, 2, 4, 8, 16
#define APB2_PRE		1				// 1, 2, 4, 8, 16
#define SysTicksClk		10000			// SysTick Freq = 10kHz

// calculate peripheral frequencies
#define SYSCLK			84000000
#define AHB				SYSCLK/AHB_PRE
#define APB1			AHB/APB1_PRE
#define APB1_TIM		APB1*2
#define APB2			AHB/APB2_PRE
#define APB2_TIM		APB2*1
#define SysTicks		AHB/SysTicksClk//4.6k 10k 0.1ms

static void SetSysClock(void);
void usart2_init(uint32_t pclk1,uint32_t bound);
void delay_ms(uint16_t time);
void TIM9_init(uint32_t pclk1, uint32_t freq);
void TIM10_init(uint32_t pclk1, uint32_t freq);
void Send_buffer(int8_t *buff, uint16_t count);
void TIM3_init(void);
void TIM4_init(void);
void TIM3_PA6_Duty_cycle(uint8_t duty);
void TIM3_PA7_Duty_cycle(uint8_t duty);
int8_t read_encoder(void);

enum PIN{
	PIN0, PIN1, PIN2, PIN3, 
	PIN4, PIN5, PIN6, PIN7, 
	PIN8, PIN9, PIN10, PIN11, 
	PIN12, PIN13, PIN14, PIN15
};

#define USART2_MAX_RECV_LEN		3	
int8_t USART2_RX_BUF[USART2_MAX_RECV_LEN]; 
int8_t Rx_data_counter = 0;

int main()
{
	SetSysClock();
	usart2_init(APB1, USART_BAUDRATE);
	TIM3_init();
	TIM4_init();
	TIM10_init(APB2_TIM, 100);
	while(1)
	{
	}
}

//#define USART_BAUDRATE 19200
#define BUFFER_LENGTH 3
uint16_t buffer_size, current_send_byte;
uint8_t buffer[BUFFER_LENGTH];

void USART2_IRQHandler(void)
{
	uint8_t Rotating_speed = 0;
	
	if (USART2->SR & USART_SR_TXE)
	{
		if(buffer_size > current_send_byte)
		{
			USART2->DR = buffer[current_send_byte];
			current_send_byte++;
		}
		else
		{
			USART2->CR2 &= ~USART_CR1_TXEIE;
			current_send_byte = 0;
			buffer_size = 0;
		}
	}

    if (USART2->SR & USART_SR_RXNE)
    {
        Rotating_speed = USART2->DR;
		
		if(Rotating_speed > 0x9B)
		{
			Rotating_speed = (0xff - Rotating_speed + 1);
			TIM3_PA7_Duty_cycle(0);
			TIM3_PA6_Duty_cycle(Rotating_speed);
			GPIOA->BSRR |= GPIO_BSRR_BR5;
			GPIOA->BSRR |= GPIO_BSRR_BS8;
		}
		else if(Rotating_speed == 0)
		{
			TIM3_PA7_Duty_cycle(0);
			TIM3_PA6_Duty_cycle(0);
			GPIOA->BSRR |= GPIO_BSRR_BR8;
			GPIOA->BSRR |= GPIO_BSRR_BR5;
		}
		else
		{
			TIM3_PA6_Duty_cycle(0);
			TIM3_PA7_Duty_cycle(Rotating_speed);
			GPIOA->BSRR |= GPIO_BSRR_BR8;
			GPIOA->BSRR |= GPIO_BSRR_BS5;
		}
    }
}

void Send_buffer(int8_t *buff, uint16_t count)
{
	for(uint16_t i = 0; i < count; i++)
	{
		buffer[buffer_size + i] = buff[i];
	}
	buffer_size += count;
	USART2->CR1 |= USART_CR1_TXEIE;
}
//SysTick Interrupt Handler
void SysTick_Handler(void)
{
	
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

void TIM1_BRK_TIM9_IRQHandler(void)
{
	TIM9->SR &= ~TIM_SR_UIF;
	
	if(!(GPIOB->IDR & GPIO_IDR_ID0))
	{
		GPIOA->BSRR |= GPIO_BSRR_BS9;
		GPIOA->BSRR |= GPIO_BSRR_BR10;
	}
	else
	{
		GPIOA->BSRR |= GPIO_BSRR_BS10;
		GPIOA->BSRR |= GPIO_BSRR_BR9;
	}
}

#define M_PI 3.14159265358979323846
void TIM1_UP_TIM10_IRQHandler(void)
{
	TIM10->SR &= ~TIM_SR_UIF;
	
	static uint8_t i = 0;
	
	if(++i == 100) i = 0;
	
	USART2_RX_BUF[0] = 'A';
	USART2_RX_BUF[1] = i;
	USART2_RX_BUF[2] = read_encoder();
	
	USART2->CR1 |= USART_CR1_TXEIE;
	Send_buffer(USART2_RX_BUF, BUFFER_LENGTH);
}

void TIM3_init(void)
{
	// init TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;		//TIM3 clock enabled
	GPIOA->MODER |= 1 << (PIN6*2 + 1) | 1 << (PIN7*2 + 1); ; // Reuse output mode
	GPIOA->MODER |= 1 << (PIN5*2) | 1 << (PIN8*2); // General output mode
	GPIOA->AFR[0] |= (uint32_t)0x22000000; // Reuse as TIM3
	TIM3->PSC = 83;			
	TIM3->ARR = 10000;		// T = 10ms
	TIM3->CCMR1 = 0x6060;
	
	TIM3->CCR1 = 0;		// Duty cycle
	TIM3->CCR2 = 0;		// Duty cycle 
	
	TIM3->CCER = 0x0011;	// Enable comparison output
	
	TIM3->CR1 |= TIM_CR1_CEN;					// Counter enable
}
void TIM3_PA6_Duty_cycle(uint8_t duty)
{
	TIM3->CCR1 = duty*100;		// Duty cycle 80%
}
void TIM3_PA7_Duty_cycle(uint8_t duty)
{
	TIM3->CCR2 = duty*100;		// Duty cycle 80%
}
void TIM4_init(void)
{
	// init TIM4
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;		//TIM4 clock enabled
	GPIOB->MODER |= 1 << (PIN6*2 + 1) | 1 << (PIN7*2 + 1); ; // Reuse output mode
	GPIOB->AFR[0] |= (uint32_t)0x22000000; // Reuse as TIM4
	TIM4->ARR = 255;
	// channel is configured as input,IC1 is mapped on TI1,IC2 is mapped on TI2
	TIM4->CCMR1 &= 0xFCFC;
	/*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0;
	TIM4->CCMR1 |= TIM_CCMR1_CC2S_0;

	TIM4->CCER |= TIM_CCER_CC1E;	// Enable comparison output
	TIM4->CCER |= TIM_CCER_CC2E;
	
	// Encoder mode 3 Counter counts up/down on both TI1FP1 and TI2FP2 edges 
	// depending on the level of the other input.
	TIM4->SMCR &= 0xFFF8;
	/*!<SMS[2:0] bits (Slave mode selection)    */
	TIM4->SMCR |= TIM_SMCR_SMS_0;
	TIM4->SMCR |= TIM_SMCR_SMS_1;

	TIM4->CNT = 0;

	TIM4->CR1 |= TIM_CR1_CEN;		// Counter enable
}

int8_t read_encoder(void)
{
	int8_t encoder_num = 0;
	encoder_num = (int8_t)TIM4 ->CNT;
	
	return encoder_num;
}

void TIM9_init(uint32_t pclk1, uint32_t freq)
{
	// init TIM9
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;			//TIM9 clock enabled
	TIM9->PSC &= ~TIM_PSC_PSC;
	TIM9->PSC = pclk1/(freq*1000) -1;			// Makes the counting frequency 1KHz	T = 1ms
	TIM9->CNT = 0;								// Counter clear
	TIM9->ARR = 1000;
	// TIM9->CR1 |= TIM_CR1_ARPE;				// Automatic reload preload enable
	TIM9->DIER |= TIM_DIER_UIE;					// Update interrupt enable
	
	NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
	
	TIM9->CR1 |= TIM_CR1_CEN;					// Counter enable
}

void TIM10_init(uint32_t pclk1, uint32_t freq)
{
	// init TIM10
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;		//TIM9 clock enabled
	TIM10->PSC &= ~TIM_PSC_PSC;
	TIM10->PSC = pclk1/(freq*1000) -1;			// Makes the counting frequency 1KHz	T = 1ms
	TIM10->CNT = 0;								// Counter clear
	TIM10->ARR = 1000;
	TIM10->DIER |= TIM_DIER_UIE;				// Update interrupt enable
	
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	
	TIM10->CR1 |= TIM_CR1_CEN;					// Counter enable
}
// Init USART2
// pclk1:PLL clock frequency
// bound:USART2 Baud rate
void usart2_init(uint32_t pclk1,uint32_t bound)
{  	 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	GPIOD->MODER |= 1 << (PIN2*2);
	// PA2->USART2_TX,PA3->USART2_RX Multiplex as serial port
	GPIOA->MODER |= 1 << (PIN2*2 + 1) | 1 << (PIN3*2 + 1); // Reuse output mode
	GPIOA->MODER |= 1 << (PIN9*2) | 1 << (PIN10*2); // General output mode
	GPIOA->AFR[0] = (uint32_t)0x7700; // Reuse as USART2
	
 	USART2->BRR=pclk1/bound;
	USART2->CR1 = USART_CR1_UE |	//Enable UART
				USART_CR1_TE |		//Enable Transmitter
				USART_CR1_RE |		//Enable Receiver
				USART_CR1_RXNEIE;	//Enable interrupt in peripheral
	
	NVIC_EnableIRQ(USART2_IRQn);		//Enable Global interrupt
}

/****************************************************************************/
/*            PLL (clocked by HSI) used as System clock source              */
/****************************************************************************/
static void SetSysClock(void)
{
	if ((RCC->CR & RCC_CR_HSIRDY) != RESET)
	{
		/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
		FLASH->ACR |= FLASH_ACR_PRFTEN;
		FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
		/* HCLK = SYSCLK / 1*/
		RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;  // AHBx=SYSCLK / AHB_PRE
		RCC->CFGR |= RCC_CFGR_HPRE_DIV1; 
		/* PCLK2 = HCLK / 2*/
		RCC->CFGR &= ~RCC_CFGR_PPRE1_Msk; // APB2 = AHBx / APB2_PRE
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
		/* PCLK1 = HCLK / 4*/
		RCC->CFGR &= ~RCC_CFGR_PPRE2_Msk; // APB1 = AHBx / APB1_PRE
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

		/* Configure the main PLL */
		// RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC;
		// RCC_PLLCFGR_PLLM=16, RCC_PLLCFGR_PLLN=46, RCC_PLLCFGR_PLLP=2,
		RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Msk;
		RCC->PLLCFGR |= 8 << RCC_PLLCFGR_PLLM_Pos;

		RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_Msk;
		RCC->PLLCFGR |= 168 << RCC_PLLCFGR_PLLN_Pos;

		RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP_Msk;
		RCC->PLLCFGR |= RCC_PLLCFGR_PLLP_0;

		/* Enable the main PLL */
		RCC->CR |= RCC_CR_PLLON;  

		/* Wait till the main PLL is ready */
		while ((RCC->CR & RCC_CR_PLLRDY) == 0) 
		{}

		/* Select the main PLL as system clock source */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW)); 
		RCC->CFGR |= RCC_CFGR_SW_PLL;

		/* Wait till the main PLL is used as system clock source */
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL)
		{}
	}
}



```
