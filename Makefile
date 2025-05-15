
#BOARD=esp32doit-devkit-v1
#BOARD=heltec_wifi_kit_32
BOARD=esp32
#VERBOSE=1

GIT_VERSION := "$(shell git describe --abbrev=4 --dirty --always --tags)"
CHIP=esp32
OTA_ADDR=192.168.43.222
IGNORE_STATE=1
EXCLUDE_DIRS=/home/jim/Arduino/libraries/lvgl/|/home/jim/Arduino/libraries/LovyanGFX
BUILD_EXTRA_FLAGS += -DGIT_VERSION=\"$(GIT_VERSION)\"
BUILD_MEMORY_TYPE = qio_qspi

include ${HOME}/Arduino/libraries/makeEspArduino/makeEspArduino.mk

fixtty:
	stty -F ${UPLOAD_PORT} -hupcl -crtscts -echo raw 921600	

cat:	fixtty
	cat ${UPLOAD_PORT}

backtrace:
	tr ' ' '\n' | /home/jim/.arduino15/packages/esp32/tools/xtensa-esp32-elf-gcc/*/bin/xtensa-esp32-elf-addr2line -f -i -e $(BUILD_DIR)/*.elf


autotrim_ubuntu:	
	g++  -x c++ -g autotrim.ino -o autotrim_ubuntu -DESP32 -DCSIM -DUBUNTU -I./ -I${HOME}/Arduino/libraries/jimlib/src 
# add -pg to profile 
	

socat:	
	socat udp-recv:9000 - 
