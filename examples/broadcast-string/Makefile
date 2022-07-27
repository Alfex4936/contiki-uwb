CONTIKI_PROJECT = dev
all: $(CONTIKI_PROJECT)

permission:
	sudo chmod a+rw /dev/ttyACM0
	sudo chmod a+rw /dev/ttyACM1

run1:
	make TARGET=dwm1001 dev1_input.upload

run2:
	make TARGET=dwm1001 dev2.upload

run3:
	make TARGET=dwm1001 dev3.upload

con1:
	python py/acm0_serial.py
# make PORT=/dev/ttyACM0 login

con2:
	python py/acm1_serial.py
# make PORT=/dev/ttyACM1 login

con3:
	python py/acm2_serial.py

# enable softdevice
#NRF52_SOFTDEVICE = 132

# print over NRF RTT
#PRINTF_OVER_RTT = 1

# enable nRF log over RTT
NRF_LOG_OVER_RTT = 1

CFLAGS += -DNRF_SHOW_RESETREASON

DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

# Enable Rime stack
CONTIKI_WITH_IPV4 = 0
CONTIKI_WITH_IPV6 = 0
CONTIKI_WITH_RIME = 1

UWB_CONTIKI ?= ../..
include $(UWB_CONTIKI)/Makefile.uwb
