.PHONY: all bootloader firmwares minican microcan clean flashmini flashmicro uartflashmini uartflashmicro

# if the bootloader is compiled in debug mode, update BOOTLOADER_SIZE to 64k else 11k should be enough
BOOTLOADER_SIZE := 16k
BOOTLOADER_LD := bootloader/cfg/STM32G491xE.ld
MINICAN_LD := minican/cfg/STM32G491xE.ld

# Optional build-time overrides forwarded to sub-makes.
PROPAGATE_VARS := CAN_BITRATE PRECISE_FDCAN_TIMINGS RELEASE
PROPAGATE_FLAGS := $(foreach v,$(PROPAGATE_VARS),$(if $(value $(v)),$v=$(value $(v))))

# STM32CubeProgrammer (UART ROM bootloader) initial flashing helpers.
# Usage (BOOT0 tied to VCC, power-cycle into ROM bootloader):
#   make uartflashmini              # uses DEV=/dev/ttyUSB0 by default
#   make uartflashmicro DEV=/dev/ttyUSB1
CUBECLI ?= /usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI
DEV ?= /dev/ttyUSB0
CUBECLI_BR ?= 115200
CUBECLI_PARITY ?= even

define CUBECLI_WRITE
$(CUBECLI) -c port=$(DEV) br=$(CUBECLI_BR) parity=$(CUBECLI_PARITY) -w $(1) -v
endef



# Build MINICAN+MICROCAN sequentially to avoid races on `minican/cfg/board.h`
# when running a parallel top-level make.
all: check-bootloader-size bootloader firmwares
	@cp  minican/build*/LAST*.bin /tmp

bootloader:
	$(MAKE) -C bootloader

firmwares:
	$(MAKE) -C minican PLATFORM=MINICAN $(PROPAGATE_FLAGS) firmware
	$(MAKE) -C minican PLATFORM=MICROCAN $(PROPAGATE_FLAGS) firmware

minican:
	$(MAKE) -C minican PLATFORM=MINICAN $(PROPAGATE_FLAGS) firmware

microcan:
	$(MAKE) -C minican PLATFORM=MICROCAN $(PROPAGATE_FLAGS) firmware

clean:
	@rm -f minican/cfg/board.h minican/cfg/.boardgen_*.stamp
	$(MAKE) -C bootloader $(PROPAGATE_FLAGS) clean
	$(MAKE) -C minican PLATFORM=MICROCAN $(PROPAGATE_FLAGS) clean
	$(MAKE) -C minican PLATFORM=MINICAN $(PROPAGATE_FLAGS) clean


check-bootloader-size:
	./set_bootloader_size.pl $(BOOTLOADER_SIZE) $(BOOTLOADER_LD) $(MINICAN_LD)

flashmini:
	$(MAKE) -C bootloader $(PROPAGATE_FLAGS) flash
	$(MAKE) -C minican PLATFORM=MINICAN $(PROPAGATE_FLAGS) flash

flashmicro:
	$(MAKE) -C bootloader $(PROPAGATE_FLAGS) flash
	$(MAKE) -C minican PLATFORM=MICROCAN $(PROPAGATE_FLAGS) flash

# Initial flashing via STM32 ROM UART bootloader (CubeProgrammer CLI).
uartflashmini: check-bootloader-size bootloader minican
	$(call CUBECLI_WRITE,bootloader/build/bootloader.elf)
	$(call CUBECLI_WRITE,minican/build_MINICAN/MINICAN.elf)

uartflashmicro: check-bootloader-size bootloader microcan
	$(call CUBECLI_WRITE,bootloader/build/bootloader.elf)
	$(call CUBECLI_WRITE,minican/build_MICROCAN/MICROCAN.elf)
