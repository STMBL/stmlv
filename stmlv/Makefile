#OPT = -O1

HWVERSION = V1

F3_ADDRESS = 0x08004000

#build f3 firmware
f3: force_look
	$(MAKE) -C f3/ ADDRESS=$(F3_ADDRESS)

#build f3 bootloader
f3_boot: force_look
	$(MAKE) -C f3_boot/ ADDRESS=0x08000000 APP_ADDRESS=$(F3_ADDRESS)

#flash f3 bootloader using stlink
f3_boot_flash: force_look
	$(MAKE) -C f3_boot/ ADDRESS=0x08000000 APP_ADDRESS=$(F3_ADDRESS) flash 

#flash f3 bootloader using df-util
f3_boot_btburn: force_look
	$(MAKE) -C f3_boot/ ADDRESS=0x08000000 APP_ADDRESS=$(F3_ADDRESS) btburn

#flash f3 firmware using stlink
flash: force_look
	$(MAKE) -C f3/ ADDRESS=$(F3_ADDRESS) flash

#flash f3 firmware using df-util
btburn: force_look
	$(MAKE) -C f3/ ADDRESS=$(F3_ADDRESS) btburn

all_btburn: force_look
	$(MAKE) -C f3/ ADDRESS=$(F3_ADDRESS) all_btburn

all_flash: force_look
	$(MAKE) -C f3/ ADDRESS=$(F3_ADDRESS) all_flash

all: f3 f3_boot

# Target: clean project
#
clean: force_look
	@echo Cleaning project:
	rm -rf build
	@$(MAKE) -C f3_boot/ clean
	@$(MAKE) -C f3/ clean

force_look:
	@true
