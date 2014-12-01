#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_uart.h"

#include "uart.h"
#include "xprintf.h"

#include "gbcam.h"

extern UART_HandleTypeDef handle_UART;

int main()
{
	UART_init(115200);

	xdev_out(UART_putc);

	//xprintf("Initializing camera...\n");

	CAM_init();

	//xprintf("Resetting it...\n");

	CAM_reset();

	//xprintf("Loading registers...\n");

	CAM_loadRegister(0x00, 0x80);
	CAM_loadRegister(0x01, 0xD6);
	CAM_loadRegister(0x02, 0x08);
	CAM_loadRegister(0x03, 0x00);
	CAM_loadRegister(0x04, 0x01);
	CAM_loadRegister(0x05, 0x00);
	CAM_loadRegister(0x06, 0x01);
	CAM_loadRegister(0x07, 0x07);

	//xprintf("Done\n");

	CAM_startCapture();

	while (1) {
		if (CAM_imageAvailable()) {
			//xprintf("Acquired image\n");

			UART_putc(0x00);
			HAL_UART_Transmit(&handle_UART, CAM_readImage(), IMGSIZE, 22000);
		}
	}
}

