/*
 * gbcam.c
 *
 *  Created on: 30/nov/2014
 *      Author: giacomo
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_uart.h"

#include "gbcam.h"

/*
 * Handlers
 */
ADC_HandleTypeDef		AdcHandle;
TIM_HandleTypeDef		htim;

/*
 * Global variables
 */
volatile uint8_t		image[IMAGE_SIZE];
volatile uint16_t		imageCursor;

volatile int			flag_captureGoing,
						flag_captureEnd;

volatile int			clockPhase;

volatile int			acquiring;
volatile int			skip;

/*
 * Pin macros
 */
#define RESET_low()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define RESET_high()	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

#define START_low()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define START_high()	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

#define LOAD_low()		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)
#define LOAD_high()	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)

#define SIN_put(what)	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, what)
#define SIN_low()		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)

#define READ()			HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)


/*
 * Private functions
 */

void CAM_GPIO_init(void)
{
	/* Enable GPIO peripheral clocks */
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();

	/* Variables */
	GPIO_InitTypeDef  GPIO_InitStruct;

	/*
	 * VOUT pin
	 * [ANALOG INPUT]
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*
	 * RESET pin
	 * [PUSH/PULL OUTPUT]
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*
	 * START pin
	 * [PUSH/PULL OUTPUT]
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*
	 * READ pin
	 * [INPUT]
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	//GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*
	 * LOAD pin
	 * [PUSH/PULL OUTPUT]
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*
	 * SIN pin
	 * [PUSH/PULL OUTPUT]
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*
	 * XCK pin
	 * [ALTERNATE FUNCTION]
	 * For PWM using TIM2 Channel 2
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void CAM_ADC_init(void)
{
	/*
	 * Enable ADC clock
	 */
	__ADC1_CLK_ENABLE();

	/* Variables */
	ADC_ChannelConfTypeDef sADCConfig;

	/*
	 * ADC Initialization
	 */
	AdcHandle.Instance = ADC1;
	AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	AdcHandle.Init.Resolution = ADC_RESOLUTION8b;
	AdcHandle.Init.ScanConvMode = ENABLE;
	AdcHandle.Init.ContinuousConvMode = ENABLE;
	AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle.Init.NbrOfDiscConversion = 0;
	AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.NbrOfConversion = 1;
	AdcHandle.Init.DMAContinuousRequests = ENABLE;
	AdcHandle.Init.EOCSelection = ENABLE;

	HAL_ADC_Init(&AdcHandle);

	/*
	 * Configure channel 0
	 */
	sADCConfig.Channel = ADC_CHANNEL_0;
	sADCConfig.Rank = 1;
	sADCConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	sADCConfig.Offset = 0;

	HAL_ADC_ConfigChannel(&AdcHandle, &sADCConfig);
}

void CAM_TIM_init(CLOCKSPEED speed)
{
	/*
	 * Enable TIM2 clock
	 */
	__TIM2_CLK_ENABLE();

	/* Variables */
	uint32_t prescaler, period;
	TIM_MasterConfigTypeDef sMasterConfig;

	/*
	 * Workout TIM parameters based on given clock speed
	 */
	if (speed == FAST) {
		prescaler = 84;
		period = 8;
	} else {
		prescaler = 84;
		period = 18;
	}

	/*
	 * Time Base configuration and initialization
	 */
	htim.Instance = TIM2;
	htim.Init.Period = period - 1;
	htim.Init.Prescaler = prescaler - 1;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.RepetitionCounter = 0x0;

	HAL_TIM_Base_Init(&htim);

	/*
	 * Configure TIM2 to trig ADC using the TRG0 internal trigger
	 */
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

	/*
	 * Initialize PWM
	 */
	HAL_TIM_PWM_Init(&htim);

	/*
	 * Configure PWM channel
	 */
	TIM_OC_InitTypeDef sPWMConfig;

	sPWMConfig.OCMode     = TIM_OCMODE_PWM1;
	sPWMConfig.Pulse      = period / 2;
	sPWMConfig.OCPolarity = TIM_OCPOLARITY_LOW;
	sPWMConfig.OCFastMode = TIM_OCFAST_DISABLE;

	HAL_TIM_PWM_ConfigChannel(&htim, &sPWMConfig, TIM_CHANNEL_2);
}

inline void CAM_waitClockRising(void) {
	while (!clockPhase);
}

inline void CAM_waitClockFalling(void) {
	while (clockPhase);
}

inline void CAM_dummyClock() {
	CAM_waitClockRising();
	CAM_waitClockFalling();
}

void CAM_changeClockSpeed(CLOCKSPEED speed)
{
	/* Wait for clock falling edge */
	CAM_waitClockFalling();

	/* Stop the timer */
	HAL_TIM_PWM_Stop_IT(&htim, TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(&htim);

	/* De-initialize the timer */
	HAL_TIM_PWM_DeInit(&htim);
	HAL_TIM_Base_DeInit(&htim);

	/* Re-initialize the timer */
	CAM_TIM_init(speed);

	/* And finally restart it along with PWM */
	HAL_TIM_Base_Start_IT(&htim);					// Providing PeriodElapsed Callback
	HAL_TIM_PWM_Start_IT(&htim, TIM_CHANNEL_2);	// Providing PulseFinished Callback
}

/*
 * Public functions
 */

void CAM_init()
{
	/*
	 * Initialize hardware
	 */
	CAM_GPIO_init();
	CAM_TIM_init(SLOW);
	CAM_ADC_init();

	/*
	 * Initial GPIO pins status
	 */
	RESET_high();
	START_low();
	LOAD_low();
	SIN_low();

	/*
	 * Configure NVIC (interrupts)
	 */
	HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);

	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	/*
	 * Initialize library global variables
	 */
	imageCursor = 0;

	flag_captureGoing = 0;
	flag_captureEnd = 0;

	clockPhase = 0;

	acquiring = 0;

	skip = PIXSKIP;

	/*
	 * Start ADC
	 */
	HAL_ADC_Start_IT(&AdcHandle);					// Providing ConvCplt Callback

	/*
	 * Start TIM Base and TIM PWM
	 */
	HAL_TIM_Base_Start_IT(&htim);					// Providing PeriodElapsed Callback
	HAL_TIM_PWM_Start_IT(&htim, TIM_CHANNEL_2);	// Providing PulseFinished Callback
}

void CAM_reset(void)
{
	/* Ensure XCK to be low when pulling down RESET */
	CAM_waitClockFalling();

	/* Reset command */
	RESET_low();
	CAM_waitClockRising();
	RESET_high();
	CAM_waitClockFalling();

	/* Dummy clock cycle */
	CAM_dummyClock();
}

void CAM_loadRegister(uint8_t address, uint8_t data)
{
	/* Variables */
	int i;

	/* Ensure XCK to be low before putting data on SIN */
	CAM_waitClockFalling();

	/* Serially shift out address (3 bits) */
	for (i = 2; i >= 0; i--) {
		/* Put bit on SIN */
		SIN_put(address & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET);

		/* Wait for XCK to raise so the bit is loaded */
		CAM_waitClockRising();

		/* Wait for XCK to get low */
		CAM_waitClockFalling();
	}

	/* Serially shift out data except the last bit (8 - 1 bits) */
	for (i = 7; i >= 1; i--) {
		/* Put bit on SIN */
		SIN_put(data & (1 << i) ? GPIO_PIN_SET : GPIO_PIN_RESET);

		/* Wait for XCK to raise so the bit is loaded */
		CAM_waitClockRising();

		/* Wait for XCK to get low */
		CAM_waitClockFalling();
	}

	/* Put last bit and put LOAD signal high */
	SIN_put(data & 0x01 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	LOAD_high();

	/* Wait for XCK to raise so the bit is loaded */
	CAM_waitClockRising();

	/* Reset SIN to default state (low) */
	SIN_low();

	/* Wait for XCK to fall so the LOAD signal is acquired */
	CAM_waitClockFalling();

	/* Reset LOAD signal to default state (low) */
	LOAD_low();
}

void CAM_startCapture(void)
{
	acquiring = 1;

	/* Ensure XCK to be low when sending the START signal */
	CAM_waitClockFalling();

	/* Start command */
	START_high();
	CAM_waitClockRising();
	START_low();
	CAM_waitClockFalling();

	/* Set fast clock speed */
	CAM_changeClockSpeed(FAST);
}

void CAM_pauseCapture(void)
{
	acquiring = 0;
}

void CAM_resumeCapture(void)
{
	acquiring = 1;
}

int CAM_imageAvailable(void)
{
	return flag_captureEnd;
}

uint8_t* CAM_readImage(void)
{
	flag_captureEnd = 0;

	return (uint8_t*) &image[0];
}

/*
 * HAL callbacks
 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	/* If not acquiring then exit ADC callback */
	if (!acquiring) {
		return;
	}

	/* Check for captureGoing flag, array overflow and skip variable*/
	if (flag_captureGoing && imageCursor < IMAGE_SIZE && !(skip--)) {
		image[imageCursor++] = (uint8_t) AdcHandle->Instance->DR;
		skip = PIXSKIP;
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
		clockPhase = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
		clockPhase = 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin != GPIO_PIN_1) return;

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET) {
		/* This block is run on the rising edge of the READ signal */

		/* If not acquiring then exit callback */
		if (!acquiring)
			return;

		flag_captureGoing = 1;
		imageCursor = 0;
		flag_captureEnd = 0;
	} else {
		/* This block is run on the falling edge of the READ signal */

		flag_captureGoing = 0;

		/* If not acquiring then exit callback */
		if (!acquiring)
			return;

		flag_captureEnd = 1;
	}
}

/*
 * Interrupt handlers
 */

void ADC_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&AdcHandle);
}

void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim);
}

void EXTI1_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}
