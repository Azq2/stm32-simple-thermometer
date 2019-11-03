PROJECT = test
BUILD_DIR = bin

CFILES += main.c
CFILES += utils.c

LDLIBS += -lm

DEVICE=stm32f103xb
OOCD_FILE = board/stm32f1discovery.cfg

INCLUDES += $(patsubst %,-I%, .)
OPENCM3_DIR=libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk

install:
	echo -n "RAM: "; size "$(PROJECT).elf" | tail -1 | awk '{print $$2 + $$3}'
	echo -n "ROM: "; size "$(PROJECT).elf" | tail -1 | awk '{print $$1}'

	st-flash write "$(PROJECT).bin" 0x08000000
