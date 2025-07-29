##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
# -Wdouble-promotion -fno-omit-frame-pointer

ifndef PLATFORM
$(error PLATFORM should be defined : MINICAN or MICROCAN)
endif

ifeq "$(PLATFORM)" "MINICAN"
PLATFORM_MINICAN=1
PLATFORM_MICROCAN=0
else ifeq "$(PLATFORM)" "MICROCAN"
PLATFORM_MINICAN=0
PLATFORM_MICROCAN=1
else
$(error PLATFORM should be MINICAN or MICROCAN)
endif

DEBUG := 1
OPT_SPEED := 2
OPT_SIZE := 3

EXECMODE := $(DEBUG)
#EXECMODE := $(OPT_SPEED)
#EXECMODE := $(OPT_SIZE)

GCCVERSIONGTEQ10 := $(shell expr `arm-none-eabi-gcc -dumpversion | cut -f1 -d.` \>= 10)
GCC_DIAG =  -Werror -Wno-error=unused-variable -Wno-error=format \
	    -Wno-error=cpp -Wno-error=type-limits \
            -Wno-error=unused-function  \
            -Wunused -Wpointer-arith \
            -Werror=sign-compare \
            -Wshadow -Wparentheses -fmax-errors=5 \
            -ftrack-macro-expansion=2 -Wno-error=strict-overflow -Wstrict-overflow=2 \
            -Wvla-larger-than=128 -Wduplicated-branches -Wdangling-else \
	    -Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches \
            -Wlogical-op -Wformat-overflow=2 -fdiagnostics-color=always

G++_DIAG =   -Wnon-virtual-dtor -Woverloaded-virtual


ifeq "$(GCCVERSIONGTEQ10)" "1"
    GCC_DIAG += -Wno-error=volatile 
    G++_DIAG += -Wno-volatile -Wno-error=deprecated-declarations
endif

UNUSED_DIAGS = -Wcast-align -Wsign-conversion -Wconversion

ifeq ($(EXECMODE),$(DEBUG)) 
  USE_OPT =  -Os -ggdb3  -Wall -Wextra \
	    -falign-functions=16 -fomit-frame-pointer \
	    $(GCC_DIAG) 
  USE_LTO = no
endif

ifeq ($(EXECMODE),$(OPT_SPEED)) 
  USE_OPT =  -Ofast -ggdb3 -flto -Wall -Wextra \
         -falign-functions=16 -fomit-frame-pointer \
         -DCH_DBG_SYSTEM_STATE_CHECK=0 -DCH_DBG_ENABLE_CHECKS=0 \
         -DCH_DBG_ENABLE_ASSERTS=0 -DCH_DBG_ENABLE_STACK_CHECK=0 \
         -DCH_DBG_FILL_THREADS=0 \
          $(GCC_DIAG)
  USE_LTO = yes
endif

ifeq ($(EXECMODE),$(OPT_SIZE)) 
  USE_OPT =  -Os -flto  -Wall -Wextra \
	    -falign-functions=16 -fomit-frame-pointer \
            -fipa-pta --specs=nano.specs \
            -DCH_DBG_SYSTEM_STATE_CHECK=0 -DCH_DBG_ENABLE_CHECKS=0 \
            -DCH_DBG_ENABLE_ASSERTS=0 -DCH_DBG_ENABLE_STACK_CHECK=0 \
            -DCH_DBG_FILL_THREADS=0 \
	    $(GCC_DIAG)
	USE_LTO = yes
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = -std=gnu23  
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -std=gnu++26 -fno-rtti -fno-exceptions -fno-threadsafe-statics  $(G++_DIAG)
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO).
ifeq ($(USE_LTO),)
  USE_LTO = no
endif


# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x1000
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x600
endif

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# FPU-related options.
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, target, sources and paths
#

# Define project name here
PROJECT = $(PLATFORM)

# Target settings.
MCU  = cortex-m4

# Imported source files and paths.
MY_DIRNAME=../../../../ChibiOS_21.11_stable
ifneq "$(wildcard $(MY_DIRNAME) )" ""
   RELATIVE=../../../..
else
  RELATIVE=../../..
endif

CHIBIOS  := $(RELATIVE)/$(notdir $(MY_DIRNAME))
CONFDIR  := ./cfg
BUILDDIR := ./build_$(PLATFORM)
DEPDIR   := ./.dep
STMSRC = $(RELATIVE)/COMMON/stm
VARIOUS = $(RELATIVE)/COMMON/various
ETL_LIB = ../../../../../etl/include
FROZEN_LIB := ../../../../../frozen/include
COMMON := ../COMMON/source
COMMON_CFG := $(COMMON)/../cfg
SRCDIR = ./source

# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32g4xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32G4xx/platform.mk
include cfg/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk
# Auto-build files in ./source recursively.
include $(CHIBIOS)/tools/mk/autobuild.mk
# Other files (optional).
include $(CHIBIOS)/os/hal/lib/complex/xsnor/hal_xsnor.mk
include $(CHIBIOS)/os/hal/lib/complex/mfs/hal_mfs.mk
include $(CHIBIOS)/os/test/test.mk
include $(CHIBIOS)/test/mfs/mfs_test.mk



# Define linker script file here
#LDSCRIPT= $(STARTUPLD)/STM32G491xE.ld
LDSCRIPT= $(CONFDIR)/STM32G491xE.ld

DSDL := $(RELATIVE)/../UAVCAN/DSDL
DSDLC := $(RELATIVE)/../UAVCAN/DSDLC
DSDL_ROOTS_PART := $(sort $(dir $(wildcard $(DSDL)/*/)))
DSDL_ROOTS := $(filter-out $(DSDL)/tests/,$(DSDL_ROOTS_PART))
DSDLC_CSRC := $(wildcard $(DSDLC)/src/*.c)

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       $(TESTSRC) \
       $(CHIBIOS)/os/various/syscalls.c \
       $(VARIOUS)/stdutil.c \
       $(VARIOUS)/printf.c \
       $(VARIOUS)/microrl/microrlShell.c \
       $(VARIOUS)/microrl/microrl.c \
       $(VARIOUS)/hal_stm32_dma.c \
       $(VARIOUS)/M95P/hal_xsnor_stm_m95p.c \
       $(RELATIVE)/../UAVCAN/libcanard/canard.c \
       $(VARIOUS)/inputCapture.c \
       $(VARIOUS)/esc_dshot.c \
       $(VARIOUS)/timerDmaCache.c \
       $(VARIOUS)/dshot_erps.c \
       $(VARIOUS)/dshot_rpmCapture.c \
       $(VARIOUS)/futabaSbusUart.c \
       $(VARIOUS)/hal_stm32_crc_v1.c



CSRC += $(DSDLC_CSRC) 


# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC) \
        $(VARIOUS)/pprzLink.cpp \
        $(VARIOUS)/UAVCAN/pubSub.cpp \
	$(VARIOUS)/M95P/eeprom_stm_m95p.cpp \
        $(VARIOUS)/UAVCAN/persistantParam.cpp \
        $(VARIOUS)/UAVCAN/persistantStorage.cpp \
        $(VARIOUS)/smart_servos/smart_servo.cpp \
        $(VARIOUS)/smart_servos/STS3032.cpp \
	$(COMMON)/adcSurvey.cpp \
	$(COMMON)/dynamicPinConfig.cpp \
	$(COMMON)/servoRole.cpp \
	$(COMMON)/servoPwm.cpp \
	$(COMMON)/servoSmart.cpp \
	$(COMMON)/escDshotRole.cpp \
	$(COMMON)/sbusTunnelRole.cpp \
	$(COMMON)/telemetryTunnelRole.cpp \
	$(COMMON)/baro_MPL3115A2_Role.cpp \
	$(COMMON)/deviceRessource.cpp \
	$(COMMON)/MFS.cpp \
	$(COMMON)/firmwareUpdate.cpp \
	$(COMMON)/rgbLeds.cpp

# List ASM source files here.
ASMSRC = $(ALLASMSRC)

# List ASM with preprocessor source files here.
ASMXSRC = $(ALLXASMSRC)

# Inclusion directories.
INCDIR = $(CONFDIR) $(ALLINC) $(CHIBIOS)/os/various $(VARIOUS) \
	 $(VARIOUS)/M95P $(EXTLIB) $(ETL_LIB) $(FROZEN_LIB) $(COMMON) \
         $(RELATIVE)/../UAVCAN/libcanard $(DSDLC)/include ../canard_chutils \
	 $(TESTINC) $(COMMON_CFG)
 

# Define C warning options here.
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DCANARD_ENABLE_CANFD=true -DDRONECAN_CXX_WRAPPERS=true -DR1M8M \
        -DFROZEN_NO_EXCEPTIONS=1 \
        -DPLATFORM_MINICAN=$(PLATFORM_MINICAN) -DPLATFORM_MICROCAN=$(PLATFORM_MICROCAN)
ifeq ($(EXECMODE),$(DEBUG))
UDEFS += -DTRACE
endif
#
# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user defines
##############################################################################

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk
$(OBJS): $(CONFDIR)/$(PLATFORM).h

LD = $(TRGT)g++

PY := python3

.PHONY: build_dsdlc

build_dsdlc:
	@echo Generating dsdl code from $(DSDL_ROOTS) to $(DSDLC)
	$(PY) $(RELATIVE)/../UAVCAN/dronecan_dsdlc/dronecan_dsdlc.py -O $(DSDLC) $(DSDL_ROOTS)
	@echo Done

$(CONFDIR)/$(PLATFORM).h: $(CONFDIR)/$(PLATFORM).cfg
	boardGen.pl --no-pp-line --no-adcp-in 	$<  $@


# stflash: all
# 	@echo write $(BUILDDIR)/$(PROJECT).bin to flash memory
# 	/bin/st-flash write  $(BUILDDIR)/$(PROJECT).bin 0x08000000
# 	@echo Done

flash: all
	@echo write $(BUILDDIR)/$(PROJECT).elf to flash memory
	bmpflash -dev /dev/bmp_gdb_mini $(BUILDDIR)/$(PROJECT).elf
	@echo Done

# flash: all
# 	@echo write $(BUILDDIR)/$(PROJECT).elf to flash memory
# 	bmpflash  $(BUILDDIR)/$(PROJECT).elf
# 	@echo Done
