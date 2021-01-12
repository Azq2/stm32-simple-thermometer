// thrid party library from http://we.easyelectronics.ru/STM32/stm32-1-wire-dma-prodolzhenie.html
/*
 * onewire.h
 *
 *  Version 1.0.1
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_

// для разных процессоров потребуется проверить функцию OW_Init
// на предмет расположения ножек USART
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

// выбираем, на каком USART находится 1-wire
#define OW_USART USART3

// если нужно отдавать тики FreeRTOS, то раскомментировать
//#define OW_GIVE_TICK_RTOS

// первый параметр функции OW_Send
#define OW_SEND_RESET		1
#define OW_NO_RESET			2

// статус возврата функций
#define OW_OK			1
#define OW_ERROR		2
#define OW_NO_DEVICE	3

#define OW_NO_READ		0xff

#define OW_READ_SLOT	0xff

uint8_t OW_Init(void);
uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart);
void OW_EnableTxPin(uint8_t enable);

#endif /* ONEWIRE_H_ */
