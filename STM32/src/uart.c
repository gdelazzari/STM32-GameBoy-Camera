/*
 * uart.c
 *
 *  Created on: 30/mag/2014
 *      Author: giacky98
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_gpio.h"

#include "uart.h"

UART_HandleTypeDef handle_UART;

void UART_init(int baud)
{
	__USART6_CLK_ENABLE();

	__GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	handle_UART.Instance = USART6;
	handle_UART.Init.BaudRate = baud;
	handle_UART.Init.WordLength = UART_WORDLENGTH_8B;
	handle_UART.Init.StopBits = UART_STOPBITS_1;
	handle_UART.Init.Parity = UART_PARITY_NONE;
	handle_UART.Init.Mode = UART_MODE_TX_RX;
	handle_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	handle_UART.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&handle_UART);
}

void UART_putc(uint8_t c)
{
	HAL_UART_Transmit(&handle_UART, &c, 1, 22000);
}

uint8_t UART_getc(void)
{
	uint8_t data;

	HAL_UART_Receive(&handle_UART, &data, 1, 22000);

	return data;
}

int UART_available(void)
{
	return 0;
}
