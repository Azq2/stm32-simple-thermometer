#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define __WFI() __asm__ volatile ("wfi")
#define __WFE() __asm__ volatile ("wfe")
#define COUNT_OF(array) (sizeof(array) / sizeof(array[0]))
#define gpio_write_bit(port, pins, value) ((value) ? gpio_set(port, pins) : gpio_clear(port, pins))

__attribute__((always_inline)) static inline void __enable_irq(void) {
	__asm__ volatile ("cpsie i" : : : "memory");
}

__attribute__((always_inline)) static inline uint32_t __disable_irq(void) {
	uint32_t result;
	__asm__ volatile ("MRS %0, primask\ncpsid i" : "=r" (result));
	return result;
}

int _write(int file, char *ptr, int len);
void pwr_enter_standby_mode(void);
void gpio_set_all_analog(void);
void set_lowpower_clock(void);
uint32_t adc_read_one_value(uint32_t adc, uint32_t channel);
void uart_simple_setup(uint32_t usart, uint32_t baudrate, bool printf);

void delay_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t us);

uint32_t crc32(const uint8_t *data, uint32_t size);
uint8_t crc8(const uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif
