.PHONY: all bootloader minican clean

# if the bootloader is compiled in debug mode, update BOOTLOADER_SIZE to 64k
BOOTLOADER_SIZE := 12k
BOOTLOADER_LD := bootloader/cfg/STM32G491xE.ld
MINICAN_LD := minican/cfg/STM32G491xE.ld




#all: minican
all: check-bootloader-size bootloader minican microcan

bootloader:
	$(MAKE) -C bootloader

minican:
	$(MAKE) -C minican PLATFORM=MINICAN

microcan:
	$(MAKE) -C minican PLATFORM=MICROCAN

clean:
	$(MAKE) -C bootloader clean
	$(MAKE) -C minican clean

check-bootloader-size:
	./set_bootloader_size.pl $(BOOTLOADER_SIZE) $(BOOTLOADER_LD) $(MINICAN_LD)

flash:
	$(MAKE) -C bootloader flash
	$(MAKE) -C minican flash
