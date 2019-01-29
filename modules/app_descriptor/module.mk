APP_DESCRIPTOR_MODULE_DIR := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

.PHONY: INSERT_CRC
POST_MAKE_ALL_RULE_HOOK: $(BUILDDIR)/$(PROJECT)-crc.bin
$(BUILDDIR)/$(PROJECT)-crc.bin: $(BUILDDIR)/$(PROJECT).bin
	python $(APP_DESCRIPTOR_MODULE_DIR)/crc_binary.py $< $@

ifneq (,$(wildcard $(BOARD_DIR)/bootloader.bin))
POST_MAKE_ALL_RULE_HOOK: $(BUILDDIR)/$(PROJECT)-combined.bin
$(BUILDDIR)/$(PROJECT)-combined.bin: $(BUILDDIR)/$(PROJECT)-crc.bin
	dd if=/dev/zero bs=$(BOOTLOADER_SIZE) count=1 | tr "\000" "\377" > $@
	dd if=$(BOARD_DIR)/bootloader.bin of=$@ conv=notrunc
	dd oflag=seek_bytes if=$(BUILDDIR)/$(PROJECT)-crc.bin of=$@ seek=$(BOOTLOADER_SIZE)
endif
