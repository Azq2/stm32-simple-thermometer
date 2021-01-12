#pragma once

#include <stdint.h>
#include <stdbool.h>

#define SWTIMER_MAX			32

typedef void (*swtimer_func_t)(void);

typedef struct {
	uint8_t id;
	bool run;
	bool loop;
	swtimer_func_t callback;
	uint32_t counter;
	uint32_t reload;
} swtimer_t;

uint32_t swtimer_counter(void);

swtimer_t *swtimer_create(swtimer_func_t callback);
void swtimer_delete(swtimer_t *timer);

void swtimer_run(swtimer_t *timer, uint32_t reload, bool loop, bool if_not_run);

static inline void swtimer_set_interval(swtimer_t *timer, uint32_t interval, bool if_not_run) {
	swtimer_run(timer, interval, true, if_not_run);
}

static inline void swtimer_set_timeout(swtimer_t *timer, uint32_t timeout, bool if_not_run) {
	swtimer_run(timer, timeout, false, if_not_run);
}

void swtimer_stop(swtimer_t *timer);

void swtimer_schedule(void);
void swtimer_interrupt(void);

void swtimer_enable_hook(bool enable);
void swtimer_idle_hook(void);
