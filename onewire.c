/*
 * onewire.c
 *
 *  Created on: 13.02.2012
 *      Author: di
 */

#include "onewire.h"

// Буфер для приема/передачи по 1-wire
static uint8_t ow_buf[8];

#define OW_0	0x00
#define OW_1	0xff
#define OW_R_1	0xff

//-----------------------------------------------------------------------------
// функция преобразует один байт в восемь, для передачи через USART
// ow_byte - байт, который надо преобразовать
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
static void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (ow_byte & 0x01) {
			*ow_bits = OW_1;
		} else {
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

//-----------------------------------------------------------------------------
// обратное преобразование - из того, что получено через USART опять собирается байт
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
static uint8_t OW_toByte(uint8_t *ow_bits) {
	uint8_t ow_byte, i;
	ow_byte = 0;
	for (i = 0; i < 8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}

	return ow_byte;
}

//-----------------------------------------------------------------------------
// инициализирует USART
//-----------------------------------------------------------------------------
uint8_t OW_Init(void) {
	OW_EnableTxPin(1);
	
	switch (OW_USART) {
		case USART1:
			rcc_periph_clock_enable(RCC_AFIO);
			rcc_periph_clock_enable(RCC_GPIOA);
			rcc_periph_clock_enable(RCC_USART1);
		break;
		
		case USART2:
			rcc_periph_clock_enable(RCC_AFIO);
			rcc_periph_clock_enable(RCC_GPIOA);
			rcc_periph_clock_enable(RCC_USART2);
		break;
		
		case USART3:
			rcc_periph_clock_enable(RCC_AFIO);
			rcc_periph_clock_enable(RCC_GPIOB);
			rcc_periph_clock_enable(RCC_USART3);
		break;
	}
	
	usart_set_baudrate(OW_USART, 115200);
	usart_set_databits(OW_USART, 8);
	usart_set_stopbits(OW_USART, USART_STOPBITS_1);
	usart_set_parity(OW_USART, USART_PARITY_NONE);
	usart_set_flow_control(OW_USART, USART_FLOWCONTROL_NONE);
	usart_set_mode(OW_USART, USART_MODE_TX_RX);
	
	// Здесь вставим разрешение работы USART в полудуплексном режиме
	USART_CR3(OW_USART) |= USART_CR3_HDSEL;
	
	usart_enable(OW_USART);
	
	return OW_OK;
}

//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------
static uint8_t OW_Reset(void) {
	uint8_t ow_presence;
	
	usart_disable(OW_USART);
	usart_set_baudrate(OW_USART, 9600);
	usart_enable(OW_USART);
	
	// отправляем 0xf0 на скорости 9600
	
	USART_SR(OW_USART) &= USART_FLAG_TC;
	usart_send(OW_USART, 0xf0);
	
	while (!(USART_SR(OW_USART) & USART_FLAG_TC)) {
#ifdef OW_GIVE_TICK_RTOS
		taskYIELD();
#endif
	}
	
	ow_presence = usart_recv(OW_USART);
	
	usart_disable(OW_USART);
	usart_set_baudrate(OW_USART, 115200);
	usart_enable(OW_USART);
	
	return ow_presence != 0xf0 ? OW_OK : OW_NO_DEVICE;
}


//-----------------------------------------------------------------------------
// процедура общения с шиной 1-wire
// sendReset - посылать RESET в начале общения.
// 		OW_SEND_RESET или OW_NO_RESET
// command - массив байт, отсылаемых в шину. Если нужно чтение - отправляем OW_READ_SLOTH
// cLen - длина буфера команд, столько байт отошлется в шину
// data - если требуется чтение, то ссылка на буфер для чтения
// dLen - длина буфера для чтения. Прочитается не более этой длины
// readStart - с какого символа передачи начинать чтение (нумеруются с 0)
//		можно указать OW_NO_READ, тогда можно не задавать data и dLen
//-----------------------------------------------------------------------------
uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen,
		uint8_t *data, uint8_t dLen, uint8_t readStart) {

	// если требуется сброс - сбрасываем и проверяем на наличие устройств
	if (sendReset == OW_SEND_RESET) {
		if (OW_Reset() == OW_NO_DEVICE) {
			return OW_NO_DEVICE;
		}
	}
	
	while (cLen > 0) {

		OW_toBits(*command, ow_buf);
		command++;
		cLen--;
		
		for (uint32_t i = 0; i < 8; ++i) {
			USART_SR(OW_USART) &= USART_FLAG_TC;
			usart_send(OW_USART, ow_buf[i]);
			
			while (!(USART_SR(OW_USART) & USART_FLAG_TC)) {
				#ifdef OW_GIVE_TICK_RTOS
					taskYIELD();
				#endif
			}
			
			ow_buf[i] = usart_recv(OW_USART);
		}
		
		// если прочитанные данные кому-то нужны - выкинем их в буфер
		if (readStart == 0 && dLen > 0) {
			*data = OW_toByte(ow_buf);
			data++;
			dLen--;
		} else {
			if (readStart != OW_NO_READ) {
				readStart--;
			}
		}
	}

	return OW_OK;
}

void OW_EnableTxPin(uint8_t enable) {
	switch (OW_USART) {
		case USART1:
			if (enable) {
				gpio_set_mode(GPIO_BANK_USART1_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_USART1_TX);
			} else {
				gpio_set_mode(GPIO_BANK_USART1_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
			}
		break;
		
		case USART2:
			if (enable) {
				gpio_set_mode(GPIO_BANK_USART2_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_USART2_TX);
			} else {
				gpio_set_mode(GPIO_BANK_USART2_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
			}
		break;
		
		case USART3:
			if (enable) {
				gpio_set_mode(GPIO_BANK_USART3_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_USART3_TX);
			} else {
				gpio_set_mode(GPIO_BANK_USART3_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);
			}
		break;
	}
}
