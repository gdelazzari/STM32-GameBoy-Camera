/*
 * uart.h
 *
 *  Created on: 30/mag/2014
 *      Author: giacky98
 */

#ifndef UART_H_
#define UART_H_


void UART_init(int baud);
void UART_putc(uint8_t c);
uint8_t UART_getc(void);
int UART_available(void);


#endif /* UART_H_ */
