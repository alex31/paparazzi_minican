.PHONY: all bootloader minican microcan clean

# if the bootloader is compiled in debug mode, update BOOTLOADER_SIZE to 64k else 11k should be enough
BOOTLOADER_SIZE := 16k
BOOTLOADER_LD := bootloader/cfg/STM32G491xE.ld
MINICAN_LD := minican/cfg/STM32G491xE.ld

# Optional build-time overrides forwarded to sub-makes.
PROPAGATE_VARS := CAN_BITRATE PRECISE_FDCAN_TIMINGS RELEASE
PROPAGATE_FLAGS := $(foreach v,$(PROPAGATE_VARS),$(if $(value $(v)),$v=$(value $(v))))



#all: minican
all: check-bootloader-size bootloader minican microcan

bootloader:
	$(MAKE) -C bootloader $(PROPAGATE_FLAGS)

minican:
	@rm -f minican/cfg/board.h
	$(MAKE) -C minican PLATFORM=MINICAN $(PROPAGATE_FLAGS) firmware

microcan: minican
	@rm -f minican/cfg/board.h
	$(MAKE) -C minican PLATFORM=MICROCAN $(PROPAGATE_FLAGS) firmware

clean:
	@rm -f minican/cfg/board.h
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
