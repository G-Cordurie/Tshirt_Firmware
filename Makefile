.PHONY: default help

APP_SUBDIRS := app/$(BUILD_PROJECT)/s132/armgcc
BTL_SUBDIRS := btl/diagW_s132_ble/armgcc
OUTPUT_DIRECTORY := _build

HW_REV = $(shell cat $(APP_SUBDIRS)/../config/app_config.h | grep "define HW_REV_STR" | awk '{print $$NF}' | sed 's/"//g' | sed 's/\r//g')
SW_REV = $(shell cat $(APP_SUBDIRS)/../../../modules/ble/app_ble_gap.h | grep "define SW_REV_STR" | awk '{print $$NF}' | sed 's/"//g' | sed 's/\r//g')
GIT_SHA1_ID = $(shell git rev-parse --short HEAD)
ifdef DEBUG
SUFFIX = debug
else
SUFFIX = release
endif

# Define protoc compiler
PROTOC = protoc
PROTOC_PLUGIN = --plugin=protoc-gen-nanopb=$(SDK_ROOT)/external/nano-pb/generator/protoc-gen-nanopb

proto:
	chmod -R 777 $(SDK_ROOT)/external/nano-pb/generator
	$(PROTOC) $(PROTOC_PLUGIN) --nanopb_out=. app/modules/storage/md.proto

# Default target - first one defined
default: all

# Print all targets that can be built
help:
	$(MAKE) -C $(APP_SUBDIRS) help

all:
	@echo ============================== Building application: $(APP_SUBDIRS) ==============================
	$(MAKE) -C $(APP_SUBDIRS)
	@echo ============================== Building bootloader: $(BTL_SUBDIRS) ==============================
	$(MAKE) -C $(BTL_SUBDIRS)
	@echo ============================== Generating bootloader settings ==============================
	make settings
	@echo ============================== Merging APP + BTL_SETTINGS + BTL + SD ==============================
	make merge
	@echo ============================== Generating DFU package ==============================
	make dfu_pkg
	@echo ============================== Building finished ==============================

settings:
	nrfutil settings generate --family NRF52 --application $(APP_SUBDIRS)/$(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 $(APP_SUBDIRS)/$(OUTPUT_DIRECTORY)/settings.hex

merge:
	mergehex --merge $(BTL_SUBDIRS)/$(OUTPUT_DIRECTORY)/nrf52832_xxaa_s132.hex $(APP_SUBDIRS)/$(OUTPUT_DIRECTORY)/settings.hex $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_7.3.0_softdevice.hex $(APP_SUBDIRS)/$(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex --output $(APP_SUBDIRS)/$(OUTPUT_DIRECTORY)/$(BUILD_PROJECT)_v$(SW_REV)_$(GIT_SHA1_ID).hex

dfu_pkg:
	nrfutil pkg generate --debug-mode --hw-version 52 --application-version 1 --application $(APP_SUBDIRS)/$(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex --sd-req 0x9D,0x0101,0x0124 --bootloader $(BTL_SUBDIRS)/$(OUTPUT_DIRECTORY)/nrf52832_xxaa_s132.hex --bootloader-version 1 --softdevice $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_7.3.0_softdevice.hex --sd-id 0x0124 --key-file $(BTL_SUBDIRS)/../../priv.pem $(APP_SUBDIRS)/$(OUTPUT_DIRECTORY)/DiagW-$(HW_REV)_v$(SW_REV)_$(GIT_SHA1_ID)_dfu_$(SUFFIX).zip

flash: default
	$(MAKE) -C $(APP_SUBDIRS) flash

flash_softdevice:
	$(MAKE) -C $(APP_SUBDIRS) flash_softdevice

flash_merge: $(APP_SUBDIRS)/$(OUTPUT_DIRECTORY)/$(BUILD_PROJECT)_v$(SW_REV)_$(GIT_SHA1_ID).hex
	make all
	$(MAKE) -C $(APP_SUBDIRS) erase
	nrfjprog -f nrf52 --program $< --sectorerase
	nrfjprog -f nrf52 --reset

sdk_config:
	$(MAKE) -C $(APP_SUBDIRS) sdk_config

erase:
	$(MAKE) -C $(APP_SUBDIRS) erase

clean:
	$(MAKE) -C $(APP_SUBDIRS) clean
	$(MAKE) -C $(BTL_SUBDIRS) clean

docker_build:
	-docker rmi -f diagw-fw:1.0
	docker build --file .devcontainer/Dockerfile --build-arg=PWD=$$(pwd) -t diagw-fw:1.0 .

docker_run:
	xhost + && docker run -it --rm --privileged \
		-e DISPLAY \
		-e SDK_ROOT \
		-e BUILD_PROJECT \
		-e DEBUG \
		-v $$(pwd):$$(pwd) \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /etc/localtime:/etc/localtime:ro \
		-v /dev/bus/usb/:/dev/bus/usb \
		-w $$(pwd) diagw-fw:1.0

docker_run_flash:
	xhost + && docker run -it --rm --privileged \
		-e DISPLAY \
		-e SDK_ROOT \
		-e BUILD_PROJECT \
		-e DEBUG \
		-v $$(pwd):$$(pwd) \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /etc/localtime:/etc/localtime:ro \
		-v /dev/bus/usb/:/dev/bus/usb \
		-w $$(pwd) diagw-fw:1.0	\
		/bin/bash -c "make clean; make all; make flash_merge"	

docker_run_ses:
	xhost + && docker run -it --rm --privileged \
		-e DISPLAY \
		-e SDK_ROOT \
		-e BUILD_PROJECT \
		-e DEBUG \
		-v $$(pwd):$$(pwd) \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /etc/localtime:/etc/localtime:ro \
		-v /dev/bus/usb/:/dev/bus/usb \
		-w $$(pwd) diagw-fw:1.0	\
		/usr/local/segger_embedded_studio/bin/emStudio -D SDK_ROOT=$(SDK_ROOT) $(PWD)/app/$(BUILD_PROJECT)/s132/ses/$(BUILD_PROJECT).emProject

docker_run_rtt:
	xhost + && docker run -it --rm --privileged \
		-e DISPLAY \
		-e SDK_ROOT \
		-e BUILD_PROJECT \
		-e DEBUG \
		-v $$(pwd):$$(pwd) \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /etc/localtime:/etc/localtime:ro \
		-v /dev/bus/usb/:/dev/bus/usb \
		-w $$(pwd) diagw-fw:1.0	\
		/bin/bash -c "JLinkRTTViewer --device nRF52832_xxAA --connection usb --interface swd --autoconnect"