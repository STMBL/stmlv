OPT = -O3

#config size in kB
CONF_SIZE = 4
CPU = STM32F303RC
XTAL_FREQ = 16000000

CFLAGS += -DHAL_MAX_PINS=512
CFLAGS += -DHAL_MAX_COMPS=32
CFLAGS += -DHAL_MAX_CTX=4096
LDFLAGS += -lc -specs=nosys.specs

LIBS = ST_HAL
LIBS += ST_USB
LIBS += CMSIS
LIBS += HAL
LIBS += STMBLTALK
LIBS += MISC
LIBS += CONF
LIBS += CMD

PERIPH = TIM
PERIPH += UART
PERIPH += ADC
PERIPH += DAC
PERIPH += OPAMP
PERIPH += COMP
PERIPH += CRC
PERIPH += RCC
PERIPH += GPIO
PERIPH += PWR
PERIPH += FLASH
PERIPH += DMA
PERIPH += USB
PERIPH += SPI

SOURCES += $(wildcard src/*.c)

# INCDIRS += ../shared/inc/
# SHAREDSOURCES += $(wildcard ../shared/src/*.c)

SRC_COMPS += $(wildcard src/comps/*.c)
# SHARED_COMPS += $(wildcard ../shared/src/comps/*.c)
LIB_COMPS += $(wildcard ../../framework/comps/curpid/*.c)
LIB_COMPS += $(wildcard ../../framework/comps/*.c)

CONFIG_TEMPLATES += $(wildcard conf/template/*.txt)

# Include the base rules
#
include ../../framework/base.mak