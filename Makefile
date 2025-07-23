.PHONY: all bootloader minican clean

BOOTLOADER_SIZE := 64k
BOOTLOADER_LD := bootloader/cfg/STM32G474xE.ld
MINICAN_LD := minican/cfg/STM32G474xE.ld




all: minican
#all: check-bootloader-size bootloader minican

bootloader:
	$(MAKE) -C bootloader

minican:
	$(MAKE) -C minican

clean:
	$(MAKE) -C bootloader clean
	$(MAKE) -C minican clean

check-bootloader-size:
	./set_bootloader_size.pl $(BOOTLOADER_SIZE) $(BOOTLOADER_LD) $(MINICAN_LD)

flash:
	$(MAKE) -C bootloader flash
	$(MAKE) -C minican flash
