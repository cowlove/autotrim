
#BOARD=esp32doit-devkit-v1
#BOARD=heltec_wifi_kit_32
#BOARD=nodemcu-32s
VERBOSE=1

CHIP=esp32
OTA_ADDR=192.168.43.222
IGNORE_STATE=1

include ${HOME}/Arduino/makeEspArduino/makeEspArduino.mk

fixtty:
	stty -F ${UPLOAD_PORT} -hupcl -crtscts -echo raw 921600	

cat:	fixtty
	cat ${UPLOAD_PORT}

backtrace:
	tr ' ' '\n' | /home/jim/.arduino15/packages/esp32/tools/xtensa-esp32-elf-gcc/1.22.0-80-g6c4433a-5.2.0/bin/xtensa-esp32-elf-addr2line -f -i -e $(BUILD_DIR)/*.elf
