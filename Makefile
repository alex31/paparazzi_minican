.PHONY: all bootloader minican microcan clean

# if the bootloader is compiled in debug mode, update BOOTLOADER_SIZE to 64k else 11k should be enough
BOOTLOADER_SIZE := 16k
BOOTLOADER_LD := bootloader/cfg/STM32G491xE.ld
MINICAN_LD := minican/cfg/STM32G491xE.ld




#all: minican
all: check-bootloader-size bootloader minican microcan

bootloader:
	$(MAKE) -C bootloader

minican:
	@rm -f minican/cfg/board.h
	$(MAKE) -C minican PLATFORM=MINICAN

microcan: minican
	@rm -f minican/cfg/board.h
	$(MAKE) -C minican PLATFORM=MICROCAN

clean:
	@rm -f minican/cfg/board.h
	$(MAKE) -C bootloader clean
	$(MAKE) -C minican PLATFORM=MICROCAN clean
	$(MAKE) -C minican PLATFORM=MINICAN clean

check-bootloader-size:
	./set_bootloader_size.pl $(BOOTLOADER_SIZE) $(BOOTLOADER_LD) $(MINICAN_LD)

flash:
	$(MAKE) -C bootloader flash
	$(MAKE) -C minican PLATFORM=MINICAN flash
