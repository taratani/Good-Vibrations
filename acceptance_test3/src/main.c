

#include <stm32f4xx.h>
#include <misc.h>			 // I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>
#include "vl53l0x_api.h"
#include "tm_stm32f4_i2c.h"
#include "tm_stm32f4_gpio.h"
#include <malloc.h>

#define MAX_STRLEN 32 // this is the maximum string length of our string in characters
#define BUTTON_LIMIT 2500000
#define NUMBER_OF_DEVICES 5 // number of rangefinders connected
#define MAX_ACTIVE_MOTORS 3
volatile char received_string[MAX_STRLEN+1]; // this will hold the recieved string
#define SLAVE_ADDRESS 0x29 // the slave address (example)
#define PERC_10 2738
#define PERC_5 2615
uint8_t state = 1;
uint8_t received_data[2];

uint8_t bottom_row = 0x00;
uint8_t middle_row = 0x00;
uint8_t top_row = 0x00;
uint8_t patternCycle = 0x01;

char object[15];
uint8_t location;
uint8_t object_array[3];
uint8_t object_index = 0;
uint8_t isObject = 1;

uint8_t object_recognition = 0;


uint8_t below_10 = 0;
uint8_t below_5 = 0;
uint8_t reset = 0;

uint32_t counter = 0;

uint32_t measurement[NUMBER_OF_DEVICES];

uint8_t getImageData = 0;



void Delay(__IO uint32_t nCount) {
  while(nCount--) {
  }
}

void clearString()
{
	for(int i=0; i<15; i++)
	{
		object[i] = 0x0;
	}
}

// updates object array with patter
void updateObjects()
{
	if(!strcmp(object, "person"))
	{
		object_array[location] = 0x3;
	}
	else if(!strcmp(object, "chair"))
	{
		object_array[location] = 0x12;
	}
}


/* This funcion initializes the USART1 peripheral
 *
 * Arguments: baudrate --> the baudrate at which the USART is
 * 						   supposed to operate
 */
void init_USART1(uint32_t baudrate){

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

void Configure_PC4(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clock for GPIOC */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Tell system that you will use PD0 for EXTI_Line0 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);

    /* PD0 is connected to EXTI_Line0 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line4;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);

    ADC_InitTypeDef ADC_InitStruct;
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConvEdge =
        ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_NbrOfConversion = 1;
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    ADC_Init(ADC1, &ADC_InitStruct);
    ADC_Cmd(ADC1, ENABLE);

    // Select input channel for ADC1
    // ADC1 channel 9 is on PB1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1,
        ADC_SampleTime_84Cycles);
}

/* This function is used to transmit a string of characters via
 * the USART specified in USARTx.
 *
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 						   (volatile) char *s is the string you want to send
 *
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 *
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
 * */
void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}


void TM_LEDS_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* Alternating functions for pins */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource15, GPIO_AF_TIM4);

	/* Set pins */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void TM_TIMER_Init(void) {
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;

	/* Enable clock for TIM4 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
/*
	TIM4 is connected to APB1 bus, which has on F407 device 42MHz clock
	But, timer has internal PLL, which double this frequency for timer, up to 84MHz
	Remember: Not each timer is connected to APB1, there are also timers connected
	on APB2, which works at 84MHz by default, and internal PLL increase
	this to up to 168MHz
	Set timer prescaller
	Timer count frequency is set with
	timer_tick_frequency = Timer_default_frequency / (prescaller_set + 1)
	In our case, we want a max frequency for timer, so we set prescaller to 0
	And our timer will have tick frequency
	timer_tick_frequency = 84000000 / (0 + 1) = 84000000
*/
	TIM_BaseStruct.TIM_Prescaler = 0;
	/* Count up */
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
/*
	Set timer period when it have reset
	First you have to know max value for timer
	In our case it is 16bit = 65535
	To get your frequency for PWM, equation is simple
	PWM_frequency = timer_tick_frequency / (TIM_Period + 1)
	If you know your PWM frequency you want to have timer period set correct
	TIM_Period = timer_tick_frequency / PWM_frequency - 1
	In our case, for 10Khz PWM_frequency, set Period to
	TIM_Period = 84000000 / 10000 - 1 = 8399
	If you get TIM_Period larger than max timer value (in our case 65535),
	you have to choose larger prescaler and slow down timer tick frequency
*/
    TIM_BaseStruct.TIM_Period = 8399; /* 10kHz PWM */
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
	/* Initialize TIM4 */
    TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
	/* Start count on TIM4 */
    TIM_Cmd(TIM3, ENABLE);
}

void TM_PWM_Init(void) {
	TIM_OCInitTypeDef TIM_OCStruct;

	/* Common settings */

	/* PWM mode 2 = Clear on compare match */
	/* PWM mode 1 = Set on compare match */
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

/*
	To get proper duty cycle, you have simple equation
	pulse_length = ((TIM_Period + 1) * DutyCycle) / 100 - 1
	where DutyCycle is in percent, between 0 and 100%
	25% duty cycle: 	pulse_length = ((8399 + 1) * 25) / 100 - 1 = 2099
	50% duty cycle: 	pulse_length = ((8399 + 1) * 50) / 100 - 1 = 4199
	75% duty cycle: 	pulse_length = ((8399 + 1) * 75) / 100 - 1 = 6299
	100% duty cycle:	pulse_length = ((8399 + 1) * 100) / 100 - 1 = 8399
	Remember: if pulse_length is larger than TIM_Period, you will have output HIGH all the time
*/
	TIM_OCStruct.TIM_Pulse = 2099; /* 25% duty cycle */
	TIM_OC1Init(TIM3, &TIM_OCStruct);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OCStruct.TIM_Pulse = 4199; /* 50% duty cycle */
	TIM_OC2Init(TIM3, &TIM_OCStruct);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OCStruct.TIM_Pulse = 6299; /* 75% duty cycle */
	TIM_OC3Init(TIM3, &TIM_OCStruct);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_OCStruct.TIM_Pulse = 4199; /* 100% duty cycle */
	TIM_OC4Init(TIM3, &TIM_OCStruct);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void init_SPI2(void){

	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;

	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* configure pins used by SPI1
	 * PB10 = SCK
	 * PC3 = MOSI
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	// connect SPI1 pins to SPI alternate function
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI1);

	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* Configure the RCK pin as an output.
	 * The data is loaded as output on
	 * the rising edge of RCK.
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Configure the G_Bar pin as an output.
	 * This needs to be low for the shift
	 * registers to be enabled
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* Configure the G_Bar pin as an output.
	 * This needs to be low for the shift
	 * registers to be enabled
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	//GPIOA->BSRRL |= GPIO_Pin_5; // set PE7 high

	// enable peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* configure SPI1 in Mode 0
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_LSB;// data is transmitted MSB first
	SPI_Init(SPI2, &SPI_InitStruct);

	SPI_Cmd(SPI2, ENABLE); // enable SPI1
}

/* This function is used to transmit data
 * with SPI2
 * 			data --> data to be transmitted
 * 			returns received value
 */
uint8_t SPI2_send(uint8_t data){

	SPI2->DR = data; // write data to be transmitted to the SPI data register
	while( !(SPI2->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	//while( !(SPI1->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPI2->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SPI2->DR; // return received data from SPI data register
}

void TIM_INT_Init()
{
    // Enable clock for TIM2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    // TIM2 initialization overflow every 500ms
    // TIM2 by default has clock of 84MHz
    // Here, we must set value of prescaler and period,
    // so update event is 0.5Hz or 500ms
    // Update Event (Hz) = timer_clock / ((TIM_Prescaler + 1) *
    // (TIM_Period + 1))
    // Update Event (Hz) = 84MHz / ((4199 + 1) * (9999 + 1)) = 2 Hz
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 4199;
    TIM_TimeBaseInitStruct.TIM_Period = 9999;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;

    // TIM2 initialize
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);
    // Enable TIM2 interrupt
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    // Start TIM2
    TIM_Cmd(TIM5, ENABLE);

    // Nested vectored interrupt settings
    // TIM2 interrupt is most important (PreemptionPriority and
    // SubPriority = 0)
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

int vibrate(uint8_t value){
	  GPIOC->BSRRL |= GPIO_Pin_15; // set G_Bar to high (disable)
	  GPIOC->BSRRL |= GPIO_Pin_14; // set SR_CLR to high (enable)
	  init_SPI2();
	  GPIOC->BSRRH |= GPIO_Pin_15; // set G_Bar to low (enable)


	  GPIOA->BSRRH |= GPIO_Pin_5; // set RCK to low
	  SPI2_send(value);  // transmit first 8 bits (far)
	  GPIOA->BSRRL |= GPIO_Pin_5; // set RCK to high
	  //GPIOA->BSRRH |= GPIO_Pin_5; // set RCK to low
}


void ADC_Config(void)
{
    // Enable clock for ADC1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // Init GPIOB for ADC input
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Init ADC1
    ADC_InitTypeDef ADC_InitStruct;
    ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
    ADC_InitStruct.ADC_ExternalTrigConvEdge =
        ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_NbrOfConversion = 1;
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ScanConvMode = DISABLE;
    ADC_Init(ADC1, &ADC_InitStruct);
    ADC_Cmd(ADC1, ENABLE);

    // Select input channel for ADC1
    // ADC1 channel 9 is on PB1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1,
        ADC_SampleTime_84Cycles);
}

uint16_t ADC_Read(void)
{
    // Start ADC conversion
    ADC_SoftwareStartConv(ADC1);
    // Wait until conversion is finish
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

    return ADC_GetConversionValue(ADC1);
}

void init_I2C1(void){

	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* setup SCL and SDA pins
	 * You can connect I2C1 to two different
	 * pairs of pins:
	 * 1. SCL on PB6 and SDA on PB7
	 * 2. SCL on PB8 and SDA on PB9
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB

	// Connect I2C1 pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA

	// configure I2C1
	I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1

	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}



VL53L0X_Error rangeInit(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
        // StaticInit will set interrupt by default
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_PerformRefCalibration(pMyDevice,
        		&VhvSettings, &PhaseCal); // Device Initialization
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
        		&refSpadCount, &isApertureSpads); // Device Initialization
    }


    return Status;
}


void Start_Ranging(VL53L0X_Dev_t *MyDevices)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	for(int i=0; i<NUMBER_OF_DEVICES; i++){
		Status = VL53L0X_StartMeasurement(&(MyDevices[i]));
	}
}

void Get_Ranges(VL53L0X_Dev_t *MyDevices)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    VL53L0X_RangingMeasurementData_t   *pRangingMeasurementData    = &RangingMeasurementData;
	for(int i=0; i<NUMBER_OF_DEVICES; i++){
        Status = VL53L0X_GetRangingMeasurementData(&(MyDevices[i]), pRangingMeasurementData);
        measurement[i] = pRangingMeasurementData->RangeMilliMeter;
	}
}

void End_Ranging(VL53L0X_Dev_t *MyDevices)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	for(int i=0; i<NUMBER_OF_DEVICES; i++){
		Status = VL53L0X_StopMeasurement(&(MyDevices[i]));
	}
}

VL53L0X_Error Init_Rangefinder(VL53L0X_Dev_t *MyDevice, uint8_t address, uint16_t pin, uint8_t pinPrefix)
{

	VL53L0X_Error Status = VL53L0X_ERROR_NONE;

	if(pinPrefix == 1)
	{
		TM_GPIO_SetPinHigh(GPIOA, pin);
	}
	else if(pinPrefix == 2)
	{
		TM_GPIO_SetPinHigh(GPIOB, pin);
	}
	else
	{
		TM_GPIO_SetPinHigh(GPIOC, pin);
	}
	Delay(500);

	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_DataInit(MyDevice);

	Status = VL53L0X_SetDeviceAddress(MyDevice, address);
	MyDevice->I2cDevAddr      = address/2;

	if (Status == VL53L0X_ERROR_NONE)
		Status = rangeInit(MyDevice);

	if (Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_SetLinearityCorrectiveGain(MyDevice, 1000);

	if(Status == VL53L0X_ERROR_NONE)
		Status = VL53L0X_SetDeviceMode(MyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in single ranging mode

	return Status;
}

void Init_RangeArray(VL53L0X_Dev_t *MyDevices)
{
   VL53L0X_Error Status = VL53L0X_ERROR_NONE;
   uint8_t address;
   uint8_t baseAddress = 0x40;

   uint16_t a_pins[3] = {GPIO_PIN_12, GPIO_PIN_11, GPIO_PIN_8};
   uint16_t c_pins[3] = {GPIO_PIN_8, GPIO_PIN_7, GPIO_PIN_6};

   TM_GPIO_Init(GPIOA, (GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_8), TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
   TM_GPIO_SetPinLow(GPIOA, (GPIO_PIN_12 | GPIO_PIN_11 | GPIO_PIN_8));

   TM_GPIO_Init(GPIOC, (GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_6), TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
   TM_GPIO_SetPinLow(GPIOC, (GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_6));
   Delay(500);

   for(int i=0; i<2; i++)
   {
	   MyDevices[i].I2cDevAddr	= 0x29;
	   MyDevices[i].comms_type      =  1;
	   MyDevices[i].comms_speed_khz =  100;
	   address = baseAddress + 4*i;
	   Init_Rangefinder(&(MyDevices[i]), address, a_pins[i], 1);
	   if (Status != VL53L0X_ERROR_NONE)
		   break;
   }



   for(int i=2; i<5; i++)
   {
	   MyDevices[i].I2cDevAddr	= 0x29;
	   MyDevices[i].comms_type      =  1;
	   MyDevices[i].comms_speed_khz =  100;
	   address = baseAddress + 4*i;
	   Init_Rangefinder(&(MyDevices[i]), address, c_pins[i-2], 3);
	   if (Status != VL53L0X_ERROR_NONE)
		   break;
   }

}

void buzz(int time)
{
	TM_LEDS_Init();
	TM_TIMER_Init();
	TM_PWM_Init();
	Delay(time);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
}

void flagReset()
{
	below_10 = 0;
	below_5 = 0;
	reset = 0;
}

void UpdatePattern()
{
	uint8_t temp_bottom = bottom_row;
	uint8_t temp_middle = middle_row;
	uint8_t temp_top = top_row;


	if(!(object_array[2] & patternCycle) && (object_array[2] != 0x00))
	{
		temp_bottom = bottom_row & 0x1F;
		temp_middle = middle_row & 0x1F;
		temp_top = top_row & 0xF8;
	}
	if(!(object_array[1] & patternCycle) && (object_array[1] != 0x00))
	{
		temp_bottom = bottom_row & 0xE7;
		temp_middle = middle_row & 0xE7;
		temp_top = top_row & 0x18;
	}
	if(!(object_array[0] & patternCycle) && (object_array[0] != 0x00))
	{
		temp_bottom = bottom_row & 0xF8;
		temp_middle = middle_row & 0xF8;
		temp_top = top_row & 0x1F;
	}


	vibrate(temp_top);
	vibrate(temp_middle);
	vibrate(temp_bottom);
}

void UpdateMotors()
{
	// reset all motors
	bottom_row = 0x00;
	middle_row = 0x00;
	top_row = 0x00;

	// find minimum values
	for(int i=0; i<MAX_ACTIVE_MOTORS; i++)
	{
		uint16_t min = 4000;
		uint8_t index = 0;
		for(int j=0; j<NUMBER_OF_DEVICES; j++)
		{
			if(measurement[j] < min){
				min = measurement[j];
				index = j;
			}
		}

		measurement[index] = (7000+1);

		if(index < 3){
			index = 0x01 << (index+1);
		}
		else if(index == 3)
		{
			index = 0x03 << (index+1);
		}
		else
		{
			index = 0x01 << (index+2);
		}

		// check for closest distance
		if(min < 500){
			top_row = top_row | index;
		}
		else if((min > 500) && (min < 1000))
		{
			middle_row = middle_row | index;
		}
		else if(min < 4000)
		{
			bottom_row = bottom_row | index;
		}
	}
	if(getImageData){
		UpdatePattern();
	}
	else{
		vibrate(top_row);
		vibrate(middle_row);
		vibrate(bottom_row);
	}

}

int main(void) {

   SystemInit();


   int32_t status_int;
   int32_t init_done = 0;

   uint8_t data = 0;
   VL53L0X_Error Status = VL53L0X_ERROR_NONE;

   TM_I2C_Init(I2C1, TM_I2C_PinsPack_1, TM_I2C_CLOCK_STANDARD);

   VL53L0X_Dev_t MyDevices[NUMBER_OF_DEVICES];

   Init_RangeArray(&MyDevices);
   Start_Ranging(&MyDevices);


   init_USART1(9600); // initialize USART1 @ 9600 baud

   //ADC_Config();
   //Configure_PC4();

   //TIM_INT_Init();
   //object_array[1] = 0x3;
   vibrate(0x00);
   vibrate(0x00);
   vibrate(0x00);


   uint16_t adc_val = 0;


//
//  /* Init leds */
  //TM_LEDS_Init();
//  /* Init timer */
  //TM_TIMER_Init();
//  /* Init PWM */
  //TM_PWM_Init();

  // device wake up

  while (1){
    /*
     * You can do whatever you want in here
     */
	  //__WFI();
	  //VL53L0X_SetLinearityCorrectiveGain(pMyDevice, 1000);
	  //TM_I2C_ReadMulti(I2C1, 0x29, 0xC0, &data, 1);
	  //Status = rangingTest(pMyDevice);

	    Get_Ranges(&MyDevices);
		if(counter == 4)
		{
			counter = 0;
			if(patternCycle == 0x08)
			{
				patternCycle = 0x01;
			}
			else
			{
				patternCycle = patternCycle << 1;
			}
		}
		else
		{
			counter++;
		}
		UpdateMotors();
		Delay(50000);


	  /*
	  adc_val = ADC_Read();


	  if((adc_val < PERC_10) && (adc_val > PERC_5) && !(below_10)){
		  buzz(500000);
		  Delay(500000);
		  buzz(500000);
		  below_10 = 1;
		  reset = 1;
	  }
	  else if ((adc_val < PERC_5) && !(below_5)){
		  buzz(500000);
		  Delay(500000);
		  buzz(500000);
		  Delay(500000);
		  buzz(500000);
		  USART_puts(USART1, "sleep");
		  below_5 = 1;
		  reset = 1;
	  }
	  else if ((adc_val > 3751) && reset){
		  flagReset();
		  buzz(500000);
	  }
	*/

  }
}

// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void){

	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){

		static uint8_t cnt = 0; // this counter is used to determine the string length
		char t = USART1->DR; // the character from the USART1 data register is saved in t

		/* check if the received character is not the LF character (used to determine end of string)
		 * or the if the maximum string length has been been reached
		 */
		if( (t != '\n') && (cnt < MAX_STRLEN) ){
			received_string[cnt] = t;
			cnt++;

			// beginning of string parsing
			if(t != ',' && isObject)
			{
				object[object_index] = t;
				object_index++;
			}
			else if(t != ',' && !isObject)
			{
				location = (t - '0')-1;
				object_index = 0;
				updateObjects();
			}
			else
			{
				if(isObject)
				{
					isObject = 0;
				}
				else
				{
					isObject = 1;
					clearString();
				}
			}
			// end of string parsing

		}
		else if(t == '\n')
		{
			isObject = 1;
			clearString();
			object_index = 0;
			cnt = 0;
		}
		else{ // otherwise reset the character counter and print the received string
			cnt = 0;
		}
	}
}


void TIM5_IRQHandler()
{
    // Checks whether the TIM2 interrupt has occurred or not
    if (TIM_GetITStatus(TIM5, TIM_IT_Update))
    {
    	counter++;
        // Clears the TIM2 interrupt pending bit
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    }
}

void EXTI4_IRQHandler(void) {
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
        /* Do your stuff when PD0 is changed */
    	uint32_t time = 0;

    	while(!(GPIOC->IDR & GPIO_Pin_4) && (time < BUTTON_LIMIT)){
    		time = time + 1;
    	}

    	// if state is equal to 1
    	if(state == 1){
    		if(time < BUTTON_LIMIT){
    			USART_puts(USART1, "hello");
    			state = 2;

    			  buzz(500000);

    			  Delay(500000*3);

    			  buzz(500000);
    			  Delay(500000);
    			  buzz(500000);


    		}
    		else{
    			USART_puts(USART1, "quit");
    			state = 1;
    		}
    	}
    	else{
    		if(time < BUTTON_LIMIT){
    			USART_puts(USART1, "data");
    			state = 2;
    			getImageData = 1;
    		}
    		else{
    			USART_puts(USART1, "quit");
    			state = 1;

    			TM_LEDS_Init();
    			TM_TIMER_Init();
    			TM_PWM_Init();
    			Delay(500000);
    			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);

    			Delay(500000);

    			TM_LEDS_Init();
    			TM_TIMER_Init();
    			TM_PWM_Init();
    			Delay(500000);
    			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
    		}
    	}

    	while(!(GPIOC->IDR & GPIO_Pin_4)){}

        /* Clear interrupt flag */
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}
