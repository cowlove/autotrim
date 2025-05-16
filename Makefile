BOARD ?= esp32s3
PORT ?= /dev/ttyACM0
CHIP ?= esp32
VERBOSE=1
EXCLUDE_DIRS=${HOME}/Arduino/libraries/lvgl|${HOME}/Arduino/libraries/LovyanGFX
PART_FILE=${ESP_ROOT}/tools/partitions/min_spiffs.csv
GIT_VERSION := "$(shell git describe --abbrev=6 --dirty --always)"

ifeq (${BOARD},esp32s3)
	CDC_ON_BOOT=1
	BUILD_MEMORY_TYPE=qio_opi
endif
ifeq (${BOARD},esp32)
	BUILD_MEMORY_TYPE=qio_qspi
endif

BUILD_EXTRA_FLAGS += -DGIT_VERSION=\"$(GIT_VERSION)\" 
	
#sed  's|^\(.*/srmodels.bin\)|#\1|g' -i ~/.arduino15/packages/esp32/hardware/esp32/3.2.0/boards.txt  



UPLOAD_PORT ?= /dev/ttyACM0

ifeq ($(BOARD),csim)
SKETCH_NAME=$(shell basename `pwd`)
CSIM_BUILD_DIR=./build/csim
CSIM_LIBS=Arduino_CRC32 ArduinoJson Adafruit_HX711 esp32jimlib
CSIM_LIBS+=esp32csim
CSIM_SRC_DIRS=$(foreach L,$(CSIM_LIBS),${HOME}/Arduino/libraries/${L}/src)
CSIM_SRC_DIRS+=$(foreach L,$(CSIM_LIBS),${HOME}/Arduino/libraries/${L})
CSIM_SRC_DIRS+=$(foreach L,$(CSIM_LIBS),${HOME}/Arduino/libraries/${L}/src/csim_include)
CSIM_SRCS=$(foreach DIR,$(CSIM_SRC_DIRS),$(wildcard $(DIR)/*.cpp)) 
CSIM_SRC_WITHOUT_PATH = $(notdir $(CSIM_SRCS))
CSIM_OBJS=$(CSIM_SRC_WITHOUT_PATH:%.cpp=${CSIM_BUILD_DIR}/%.o)
CSIM_INC=$(foreach DIR,$(CSIM_SRC_DIRS),-I${DIR})

CSIM_CFLAGS+=-g -ffunction-sections -Wl,--gc-sections -MMD -fpermissive -DGIT_VERSION=\"${GIT_VERSION}\" -DESP32 -DCSIM -DUBUNTU 
#CSIM_CFLAGS+=-DGPROF=1 -pg
#CSIM_CFLAGS+=-O2

${CSIM_BUILD_DIR}/%.o: %.cpp 
	echo $@
	${CCACHE} g++ ${CSIM_CFLAGS} -x c++ -c ${CSIM_INC} $< -o $@

${CSIM_BUILD_DIR}/%.o: %.ino
	echo $@
	${CCACHE} g++ ${CSIM_CFLAGS} -x c++ -c ${CSIM_INC} $< -o $@

${SKETCH_NAME}_csim: ${CSIM_BUILD_DIR} ${CSIM_OBJS} ${CSIM_BUILD_DIR}/${SKETCH_NAME}.o
	echo $@
	g++ -g ${CSIM_CFLAGS} ${CSIM_OBJS} ${CSIM_BUILD_DIR}/${SKETCH_NAME}.o -o $@         

csim: ${SKETCH_NAME}_csim 
	cp $< $@

${CSIM_BUILD_DIR}:
	mkdir -p ${CSIM_BUILD_DIR}

VPATH = $(sort $(dir $(CSIM_SRCS)))

.PHONY: csim-clean
csim-clean:
	rm -f ${CSIM_BUILD_DIR}/*.[od] ${SKETCH_NAME}_csim csim

-include ${CSIM_BUILD_DIR}/*.d
else
	include ~/Arduino/libraries/makeEspArduino/makeEspArduino.mk
endif

cat:    
	while sleep .01; do if [ -c ${PORT} ]; then stty -F ${PORT} -echo raw 115200 && cat ${PORT}; fi; done  | tee ./cat.`basename ${PORT}`.out
socat:  
	socat udp-recvfrom:9000,fork - 
mocat:
	mosquitto_sub -h rp1.local -t "${MAIN_NAME}/#" -F "%I %t %p"   
uc:
	${MAKE} upload && ${MAKE} cat

backtrace:
	tr ' ' '\n' | addr2line -f -i -e ./build/${BOARD}/*.elf


