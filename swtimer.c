#include <stdlib.h>

#include "swtimer.h"
#include "utils.h"

static swtimer_t tasks[SWTIMER_MAX];
static uint8_t tasks_count = 0;
static uint8_t run_tasks_count = 0;
static volatile uint32_t systicks = 0;

uint32_t swtimer_counter(void) {
	return systicks;
}

swtimer_t *swtimer_create(swtimer_func_t callback) {
	if (tasks_count < SWTIMER_MAX) {
		swtimer_t *task = &tasks[tasks_count];
		task->id = tasks_count;
		
		task->run = 0;
		task->loop = 0;
		
		task->counter = 0;
		task->reload = 0;
		task->callback = callback;
		
		++tasks_count;
		
		return task;
	}
	return NULL;
}

void swtimer_delete(swtimer_t *timer) {
	int was_masked = __disable_irq();
	
	if (timer->run)
		swtimer_stop(timer);
	
	if (tasks_count > 1)
		tasks[timer->id] = tasks[tasks_count - 1];
	
	--tasks_count;
	
	if (!was_masked)
		__enable_irq();
}

void swtimer_run(swtimer_t *timer, uint32_t reload, bool loop, bool if_not_run) {
	int was_masked = __disable_irq();
	
	if ((if_not_run && timer->run)) {
		if (!was_masked)
			__enable_irq();
		return;
	}
	
	timer->counter = systicks;
	timer->reload = reload;
	timer->loop = loop;
	timer->run = 1;
	
	if (!run_tasks_count)
		swtimer_enable_hook(true);
	
	++run_tasks_count;
	
	if (!was_masked)
		__enable_irq();
}

void swtimer_stop(swtimer_t *timer) {
	int was_masked = __disable_irq();
	
	if (!timer->run) {
		if (!was_masked)
			__enable_irq();
		return;
	}
	
	timer->run = 0;
	--run_tasks_count;
	
	if (!run_tasks_count)
		swtimer_enable_hook(false);
	
	if (!was_masked)
		__enable_irq();
}

void swtimer_interrupt(void) {
	++systicks;
}

void swtimer_schedule(void) {
	while (true) {
		bool idle = true;
		for (uint8_t i = 0; i < tasks_count; ++i) {
			swtimer_t *task = &tasks[i];
			if (task->run && systicks - task->counter >= task->reload) {
				task->counter = systicks;
				
				swtimer_func_t callback = task->callback;
				if (!task->loop)
					swtimer_stop(task);
				callback();
				idle = false;
			}
		}
		if (idle)
			swtimer_idle_hook();
	}
}
