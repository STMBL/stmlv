OPT = -O1

#config size in kB
CONF_SIZE = 4
CPU = STM32F303RC
XTAL_FREQ = 16000000

LDFLAGS += -lc -specs=nosys.specs

LIBS = ST_HAL
LIBS += CMSIS

#PERIPH = TIM

SOURCES += $(wildcard src/*.c)

# INCDIRS += ../shared/inc/
# SHAREDSOURCES += $(wildcard ../shared/src/*.c)

# SRC_COMPS += $(wildcard src/comps/*.c)
# SHARED_COMPS += $(wildcard ../shared/src/comps/*.c)
# LIB_COMPS += $(wildcard ../../framework/comps/curpid/*.c)
# LIB_COMPS += $(wildcard ../../framework/comps/*.c)

# CONFIG_TEMPLATES += $(wildcard conf/template/*.txt)

# Include the base rules
#
include ../../framework/base.mak