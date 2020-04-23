
#BOARD=esp32doit-devkit-v1
#BOARD=heltec_wifi_kit_32
#BOARD=nodemcu-32s
VERBOSE=1

CHIP=esp32
OTA_ADDR=192.168.43.222
IGNORE_STATE=1

include ${HOME}/Arduino/makeEspArduino/makeEspArduino.mk

fixtty:
	stty -F ${UPLOAD_PORT} -hupcl -crtscts -echo raw 9600

cat:	fixtty
	cat ${UPLOAD_PORT}
