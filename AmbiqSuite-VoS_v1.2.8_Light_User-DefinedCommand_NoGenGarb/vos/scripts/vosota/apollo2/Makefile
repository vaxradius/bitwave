#*******************************************************************************
#
# Simple Makefile to prepare binaries for VoS OTA.
#
#*******************************************************************************

FLAG_ADDR_APOLLO2 = 0x6000
LOAD_ADDR_APOLLO2 = 0x8000
APPBIN_APOLLO2_BLUE_PROD =../../../boards/apollo2_blue_evb/examples/vos/iar/bin/VoS_Light_OTA.bin
BOOTBIN_APOLLO2_BLUE_PROD =../../../boards/apollo2_blue_evb/examples/multi_boot/keil/bin/multi_boot.bin
UPDATEBIN_APOLLO2_BLUE =../../../boards/apollo2_blue_evb/examples/vos/iar/bin/VoS_Light_OTA.bin

all: $(APPBIN_APOLLO2_BLUE_PROD) $(BOOTBIN_APOLLO2_BLUE_PROD) $(UPDATEBIN_APOLLO2_BLUE)
	python bootloader_binary_combiner.py --bootbin "${BOOTBIN_APOLLO2_BLUE_PROD}" --appbin "${APPBIN_APOLLO2_BLUE_PROD}" --flag-addr ${FLAG_ADDR_APOLLO2} --load-address ${LOAD_ADDR_APOLLO2} -o starter_binary_apollo2_vos
	python ota_binary_converter.py --appbin "${UPDATEBIN_APOLLO2_BLUE}" --load-address ${LOAD_ADDR_APOLLO2} -o update_binary_apollo2_vos


#$(APPBIN_APOLLO2_BLUE_PROD):
#	$(MAKE) -C ../../../boards/apollo2_blue_evb/examples/key_word_detection/iar/ $(MAKECMDGOALS)

#$(BOOTBIN_APOLLO2_BLUE_PROD):
#	$(MAKE) -C ../../../boards/apollo2_blue_evb/examples/multi_boot/iar/ $(MAKECMDGOALS)

#$(UPDATEBIN_APOLLO2_BLUE):
#	$(MAKE) -C ../../../boards/apollo2_blue_evb/examples/key_word_detection/iar/ $(MAKECMDGOALS)

#clean:
#	rm -rf starter_binary_apollo2_vos.bin update_binary_apollo2_vos.bin
#	$(MAKE) -C ../../../boards/apollo2_blue_evb/examples/multi_boot/iar/ $(MAKECMDGOALS)
#	$(MAKE) -C ../../../boards/apollo2_blue_evb/examples/key_word_detection/iar/ $(MAKECMDGOALS)
#	$(MAKE) -C ../../../boards/apollo2_blue_evb/examples/key_word_detection/iar/ $(MAKECMDGOALS)
	