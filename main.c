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

#define PWM_FREQ			70000
#define PWM_DUTY_CYCLE		50
#define DEBUG				1

static void pwm_setup(void) {
	uint32_t period = (uint32_t) round((float) rcc_apb1_frequency / (float) PWM_FREQ);
	uint32_t duty_cycle = (uint32_t) round((float) rcc_apb1_frequency / (float) PWM_FREQ / 100.0f * (float) PWM_DUTY_CYCLE);
	
	rcc_periph_reset_pulse(RST_TIM2);
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM2, 0);
	timer_set_period(TIM2, period);
	timer_continuous_mode(TIM2);
	
	timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM1);
	timer_enable_oc_preload(TIM2, TIM_OC3);
	
	timer_set_oc_polarity_high(TIM2, TIM_OC3);
	timer_enable_oc_output(TIM2, TIM_OC3);
	
	timer_set_oc_value(TIM2, TIM_OC3, duty_cycle);
	
	timer_enable_preload(TIM2);
	timer_enable_counter(TIM2);
}

static void main_hw_setup(void) {
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_TIM2);
	
	// PA0 - IRQ
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO1);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2);
	
	// PA1 - dc dc enable
	gpio_set_mode(GPIO_BANK_TIM2_CH3, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1);
	
	// PA2 - dc dc pwm
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM2_CH3);
	
	// Setup EXTI for IRQ
	nvic_enable_irq(NVIC_EXTI0_IRQ);
	exti_select_source(EXTI0, GPIOA);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI0);
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

void hard_fault_handler() {
	// make pickoff
	printf("Hard fault!!!\r\n");
	
	while (true) {
		gpio_toggle(GPIOC, GPIO13);
		delay_ms(500);
	}
}

int main(void) {
	gpio_set_all_analog();
	
	// Init delay functions
	delay_init();
	
	// Init main HW peripherals
	main_hw_setup();
	
	// Setup sleep mode
	SCB_SCR &= ~SCB_SCR_SLEEPDEEP;
	SCB_SCR &= ~SCB_SCR_SLEEPONEXIT;
//	DBGMCU_CR |= DBGMCU_CR_SLEEP;
	
	gpio_write_bit(GPIOA, GPIO1, 1);
	pwm_setup();
	
	debug_setup();
	while (1) {
		printf("ololo %d %d %d\r\n", gpio_get(GPIOA, GPIO0), gpio_get(GPIOA, GPIO1), gpio_get(GPIOA, GPIO2));
		
		gpio_write_bit(GPIOC, GPIO13, gpio_get(GPIOA, GPIO0) != 0 ? 0 : 1);
	}
	
	return 0;
}
