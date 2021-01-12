#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/f1/bkp.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/dbgmcu.h>

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "utils.h"
#include "swtimer.h"
#include "onewire.h"

static uint32_t digits_state[3] = {};

struct digits_pin {
	uint32_t port;
	uint32_t pin;
};

static struct digits_pin digits_anodes[] = {
	{.port = GPIOB, .pin = GPIO14}, 
	{.port = GPIOB, .pin = GPIO13}, 
	{.port = GPIOB, .pin = GPIO12}, 
};

static struct digits_pin digits_cathodes[] = {
	{.port = GPIOA, .pin = GPIO0}, 
	{.port = GPIOA, .pin = GPIO1}, 
	{.port = GPIOA, .pin = GPIO2}, 
	{.port = GPIOA, .pin = GPIO3}, 
	{.port = GPIOA, .pin = GPIO4}, 
	{.port = GPIOA, .pin = GPIO5}, 
	{.port = GPIOA, .pin = GPIO6}, 
	{.port = GPIOA, .pin = GPIO7}, 
};

struct {
	swtimer_t *digits;
	swtimer_t *temperature_request;
	swtimer_t *temperature_read;
} tasks = {0};

static uint8_t symtab[0xFF] = {0};

static void main_hw_setup(void) {
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	
	for (uint32_t i = 0; i < COUNT_OF(digits_anodes); ++i) {
		gpio_set(digits_anodes[i].port, digits_anodes[i].pin);
		gpio_set_mode(digits_anodes[i].port, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, digits_anodes[i].pin);
	}
	
	for (uint32_t i = 0; i < COUNT_OF(digits_cathodes); ++i) {
		gpio_set(digits_cathodes[i].port, digits_cathodes[i].pin);
		gpio_set_mode(digits_cathodes[i].port, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, digits_cathodes[i].pin);
	}
	
	// Setup systick
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(rcc_ahb_frequency / 1000);
	systick_interrupt_enable();
	
	// Setup sleep mode
	SCB_SCR &= ~SCB_SCR_SLEEPDEEP;
	SCB_SCR &= ~SCB_SCR_SLEEPONEXIT;
//	DBGMCU_CR |= DBGMCU_CR_SLEEP;
}

static void debug_setup(void) {
	// UART for debug
	rcc_periph_clock_enable(RCC_USART1);
	uart_simple_setup(USART1, 115200, true);
	
	// PA9 & PA10 - UART
	gpio_set_mode(GPIO_BANK_USART1_TX, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	gpio_set_mode(GPIO_BANK_USART1_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	
	// PC13 - status led
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO13);
	gpio_write_bit(GPIOC, GPIO13, 1);
}

void exti0_isr(void) {
	exti_reset_request(EXTI0);
}

void sys_tick_handler(void) {
	swtimer_interrupt();
}

void swtimer_enable_hook(bool enable) {
	if (enable) {
		printf("[WAKE UP]\r\n");
		systick_counter_enable();
	} else {
		printf("[IDLE]\r\n");
		systick_counter_disable();
	}
}

void swtimer_idle_hook(void) {
	__WFI();
}

void hard_fault_handler() {
	// make pickoff
	printf("Hard fault!!!\r\n");
	
	while (true) {
		gpio_toggle(GPIOC, GPIO13);
		delay_ms(500);
	}
}

static void digits_next_frame(void) {
	static uint8_t digits_curr_frame = 0;
	
	uint8_t anode = digits_curr_frame % COUNT_OF(digits_anodes);
	
	for (uint32_t i = 0; i < COUNT_OF(digits_anodes); ++i) {
		if (i == anode) {
			gpio_clear(digits_anodes[i].port, digits_anodes[i].pin);
		} else {
			gpio_set(digits_anodes[i].port, digits_anodes[i].pin);
		}
	}
	
	for (uint32_t i = 0; i < COUNT_OF(digits_cathodes); ++i) {
		if ((digits_state[anode] >> i) & 1) {
			gpio_clear(digits_cathodes[i].port, digits_cathodes[i].pin);
		} else {
			gpio_set(digits_cathodes[i].port, digits_cathodes[i].pin);
		}
	}
	
	++digits_curr_frame;
}

static void digits_init_symtab(void) {
	const uint8_t S0 = (1 << 0);
	const uint8_t S1 = (1 << 1);
	const uint8_t S2 = (1 << 2);
	const uint8_t S3 = (1 << 3);
	const uint8_t S4 = (1 << 4);
	const uint8_t S5 = (1 << 5);
	const uint8_t S6 = (1 << 6);
	const uint8_t S7 = (1 << 7);
	
	symtab['0'] = S0 | S1 | S3 | S5 | S6 | S7;
	symtab['1'] = S3 | S5;
	symtab['2'] = S0 | S1 | S4 | S5 | S7;
	symtab['3'] = S1 | S3 | S4 | S5 | S7;
	symtab['4'] = S3 | S4 | S5 | S6;
	symtab['5'] = S1 | S3 | S4 | S6 | S7;
	symtab['6'] = S0 | S1 | S3 | S4 | S6 | S7;
	symtab['7'] = S3 | S5 | S7;
	symtab['8'] = S0 | S1 | S3 | S4 | S5 | S6 | S7;
	symtab['9'] = S1 | S3 | S4 | S5 | S6 | S7;
	symtab['-'] = S4;
}

static void digits_set(uint8_t index, char c, uint8_t point) {
	digits_state[index] = symtab[(uint8_t) c] | (point ? (1 << 2) : 0);
}

static void digits_task(void) {
	digits_next_frame();
}

static void temperature_request_task(void) {
	OW_EnableTxPin(1);
	
	uint8_t ret = OW_Send(OW_SEND_RESET, (uint8_t *) "\xcc\x44", 2, NULL, 0, OW_NO_READ);
	if (ret == OW_OK) {
		OW_EnableTxPin(0);
		swtimer_set_timeout(tasks.temperature_read, 1500, true);
	} else {
		digits_set(0, '-', 1);
		digits_set(0, '-', 1);
		digits_set(0, '0', 1);
		
		printf("[DS18B20] can't request temperature... (ret=%d)\r\n", ret);
		swtimer_set_timeout(tasks.temperature_request, 3000, true);
	}
}

static void temperature_read_task(void) {
	OW_EnableTxPin(1);
	
	uint8_t buf[9] = {0};
	OW_Send(OW_SEND_RESET, (uint8_t *) "\xcc\xbe", 2, NULL, 0, OW_NO_READ);
	OW_Send(OW_NO_RESET, (uint8_t *) "\xff\xff\xff\xff\xff\xff\xff\xff\xff", 9, buf, 9, 0);
	
	uint8_t real_crc = crc8(buf, 8);
	if (real_crc == buf[8]) {
		double temp = (double) ((buf[1] << 8) | buf[0]) * 0.0625d;
		printf("%.02f °C\r\n", temp);
		
		if (temp > -100 && temp < 100) {
			char str[8];
			sprintf(str, "%.02f", temp);
			
			uint32_t digit = 0;
			for (uint32_t i = 0; i < 8; ++i) {
				if (!str[i] || digit > 2)
					break;
				
				if (str[i] == '.')
					continue;
				
				digits_set(digit, str[i], str[i + 1] == '.' && digit != 2);
				++digit;
			}
			
			swtimer_set_timeout(tasks.temperature_request, 3000, true);
		} else {
			digits_set(0, '-', 1);
			digits_set(0, '-', 1);
			digits_set(0, '1', 1);
			
			printf("[DS18B20] invalid temp = %f\r\n", temp);
			swtimer_set_timeout(tasks.temperature_request, 3000, true);
		}
	} else {
		digits_set(0, '-', 1);
		digits_set(0, '-', 1);
		digits_set(0, '2', 1);
		
		printf("[DS18B20] crc error (real_crc=%02X != expected=%02X)\r\n", real_crc, buf[8]);
		swtimer_set_timeout(tasks.temperature_request, 3000, true);
	}
}

int main(void) {
	gpio_set_all_analog();
	delay_init();
	main_hw_setup();
	debug_setup();
	digits_init_symtab();
	
	OW_Init();
	
	digits_set(0, '-', 0);
	digits_set(1, '-', 0);
	digits_set(2, '-', 0);
	
	tasks.digits = swtimer_create(digits_task);
	tasks.temperature_request = swtimer_create(temperature_request_task);
	tasks.temperature_read = swtimer_create(temperature_read_task);
	
	swtimer_set_interval(tasks.digits, 5, true);
	swtimer_set_timeout(tasks.temperature_request, 0, true);
	swtimer_schedule();
	
	return 0;
}
