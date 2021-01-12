#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/dwt.h>

#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "utils.h"

static int32_t uart_for_prinf = -1;

void pwr_enter_standby_mode(void) {
	rcc_periph_clock_enable(RCC_PWR);
	pwr_enable_wakeup_pin();
	pwr_disable_power_voltage_detect();
	pwr_clear_wakeup_flag();
	pwr_set_standby_mode();
	SCB_SCR |= SCB_SCR_SLEEPDEEP;
	__WFI();      
}

uint32_t crc32(const uint8_t *data, uint32_t size) {
	CRC_CR |= CRC_CR_RESET;
	
	for (uint32_t i = 0; i < size; i += 4) {
		switch ((i + size) - size) {
			case 1:
				CRC_DR = (data[i + 1] << 16) | (data[i + 1] << 8) | data[i];
			break;
			
			case 2:
				CRC_DR = (data[i + 1] << 8) | data[i];
			break;
			
			case 3:
				CRC_DR = data[i];
			break;
			
			default:
				CRC_DR = *((uint32_t *) &data[i]);
			break;
		}
	}
	
	return CRC_DR;
}

void set_lowpower_clock(void) {
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);
	
	rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);
	
	size_t osc_freq = 8000000;
	
	// AHB
	size_t ahb_div = 64;
	rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_DIV64);
	
	// APB1
	size_t apb1_div = 1;
	rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_NODIV);
	
	// APB2
	size_t apb2_div = 1;
	rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);
	
	// ADC
	rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV2);
	
	rcc_ahb_frequency = osc_freq / ahb_div;
	rcc_apb1_frequency = rcc_ahb_frequency / apb1_div;
	rcc_apb2_frequency = rcc_ahb_frequency / apb2_div;
}

void gpio_set_all_analog(void) {
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
	
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO_ALL);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO_ALL);
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO_ALL);
	gpio_set_mode(GPIOD, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO_ALL);
	
	rcc_periph_clock_disable(RCC_GPIOA);
	rcc_periph_clock_disable(RCC_GPIOB);
	rcc_periph_clock_disable(RCC_GPIOC);
	rcc_periph_clock_disable(RCC_GPIOD);
}

uint32_t adc_read_one_value(uint32_t adc, uint32_t channel) {
	uint8_t channel_array[16];
	
	adc_power_off(adc);
	adc_disable_scan_mode(adc);
	adc_set_single_conversion_mode(adc);
	adc_disable_external_trigger_regular(adc);
	adc_set_right_aligned(adc);
	adc_set_sample_time_on_all_channels(adc, ADC_SMPR_SMP_28DOT5CYC);
	adc_power_on(adc);
	adc_reset_calibration(adc);
	adc_calibrate(adc);
	
	channel_array[0] = channel;
	adc_set_regular_sequence(adc, 1, channel_array);
	adc_start_conversion_direct(adc);
	while (!(ADC_SR(adc) & ADC_SR_EOC));
	
	uint32_t value = ADC_DR(adc);
	
	adc_power_off(adc);
	
	return value;
}

void uart_simple_setup(uint32_t usart, uint32_t baudrate, bool use_for_printf) {
	usart_set_baudrate(usart, baudrate);
	usart_set_databits(usart, 8);
	usart_set_stopbits(usart, USART_STOPBITS_1);
	usart_set_parity(usart, USART_PARITY_NONE);
	usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
	usart_set_mode(usart, USART_MODE_TX_RX);
	usart_enable(usart);
	
	if (use_for_printf)
		uart_for_prinf = USART1;
}

void delay_init(void) {
	dwt_enable_cycle_counter();
}

void delay_us(uint32_t us) {
	uint32_t cycles = dwt_read_cycle_counter() + (rcc_ahb_frequency / 1000000) * us;
	while (dwt_read_cycle_counter() <= cycles);
}

void delay_ms(uint32_t us) {
	uint32_t cycles = dwt_read_cycle_counter() + (rcc_ahb_frequency / 1000) * us;
	while (dwt_read_cycle_counter() <= cycles);
}

int _write(int file, char *ptr, int len) {
	int i;
	if (uart_for_prinf > -1 && (file == STDOUT_FILENO || file == STDERR_FILENO)) {
		for (i = 0; i < len; ++i)
			usart_send_blocking(USART1, ptr[i]);
		return i;
	}
	errno = EIO;
	return -1;
}

uint8_t crc8(const uint8_t *data, uint8_t len) {
	const static uint8_t crc8_table[] = {
		0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
		157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
		35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
		190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
		70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
		219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
		101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
		248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
		140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
		17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
		175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
		50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
		202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
		87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
		233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
		116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
	};
	
	uint8_t crc = 0;
	while (len--)
		crc = crc8_table[(crc ^ *data++)];
	
	return crc;
}
