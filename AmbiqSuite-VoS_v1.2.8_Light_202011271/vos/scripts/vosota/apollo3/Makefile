#*******************************************************************************
#
# Simple Makefile to prepare binaries for AMOTA for Apollo3.
#
#*******************************************************************************


# Apollo3-BLUE EVB
#UPDATEBIN_APOLLO3_BLUE =../../../boards/apollo3_maya/examples/freertos_amota/iar/bin/freertos_amota.bin
#APPBIN_APOLLO3_BLUE =../../../boards/apollo3_maya/examples/freertos_amota/iar/bin/freertos_amota.bin
UPDATEBIN_APOLLO3_BLUE =../../../boards/apollo3_maya/examples/vos/iar/bin/VoS_DSPC_THF.bin
APPBIN_APOLLO3_BLUE =../../../boards/apollo3_maya/examples/vos/iar/bin/VoS_DSPC_THF.bin

all: $(APPBIN_APOLLO3_BLUE) $(UPDATEBIN_APOLLO3_BLUE) $(UPDATEBIN_APOLLO3_BLUE_ETHERMIND) $(APPBIN_APOLLO3_BLUE_ETHERMIND)
	# Apollo3 Cordio
	cp $(APPBIN_APOLLO3_BLUE) starter_binary_apollo3_vos.bin
	cp apollo3_scripts/keys_info0.py apollo3_scripts/keys_info.py
#	python3 ../../apollo3_scripts/create_cust_image_blob.py --bin $(APPBIN_APOLLO3_BLUE) --load-address 0xc000 --magic-num 0xcb -o ../../apollo3_scripts/temp_main_nosecure_ota --version 0x0
#	python3 ota_binary_converter.py --appbin ../../apollo3_scripts/temp_main_nosecure_ota.bin -o update_binary_apollo3_blue
	python apollo3_scripts/create_cust_image_blob.py --bin $(APPBIN_APOLLO3_BLUE) --load-address 0xc000 --magic-num 0xcb -o apollo3_scripts/temp_main_nosecure_ota --version 0x0
	python ota_binary_converter.py --appbin apollo3_scripts/temp_main_nosecure_ota.bin -o update_binary_apollo3_vos
	@rm -rf apollo3_scripts/temp_main_nosecure_ota.bin

$(APPBIN_APOLLO3_BLUE):
#	$(MAKE) -C ../../../boards/apollo3_maya/freertos_amota/iar/ $(MAKECMDGOALS
	$(MAKE) -C ../../../boards/apollo3_maya/example/vos/iar/ $(MAKECMDGOALS)

clean:
	rm -rf *.bin
	rm -rf $(APPBIN_APOLLO3_BLUE)
	