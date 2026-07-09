#ifdef CSIM
#include "esp32csim.h"
#endif
#include "driver/twai.h"
	
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <istream>
#include <fstream>

#ifdef CSIM
#undef ARDUINO
#define ARDUINO 10819
#endif
#include "TinyGPS++.h"
#include "jimlib.h"
#include "dataTools.h"
// The historical ILS simulator lives in the winglevlr nav helpers.
#include "../winglevlr/WaypointNav.h"
#include "espNowMux.h"
#include "reliableStream.h"
#include "synchTools.h"
#include "sl30.h"
#include "buttonTools.h"
#include "TTGO_TS.h"
#include "SensorFusion.h"

using namespace WaypointNav;
//#define LED_PIN 2

//WiFiMulti wifi;
//WiFiUDP udpG90;
struct NavFixState {
	double lat, lon;
	float track;
	float altMeters;
	float hvelKnots;
	uint32_t timestamp;
	bool updated, valid, hasTrack, hasSpeed, hasAltitude;
};

static NavFixState navFix;
TinyGPSPlus nmeaGps;
static const uint32_t GPS_FIX_STALE_MS = 2000;

static SensorFusion sensorFusion;

static float g5KnobValues[6];
static double hd = -2, vd = -2; // cdi deflections 

#ifdef XDISPLAY
namespace Display {
	JDisplay jd;
	int y = 0;
	const int c2x = 70;
	JDisplayItem<const char *>  ip(&jd,10,y,"WIFI:", "%s ");
	JDisplayItem<float>  sec(&jd,10,y+=10," SEC:", "%.02f ");
}
#endif


//std::queue<struct CanPacket> qFull;
//std::queue<uint8_t *> qEmpty; 

// PINOUT Garmin G5 DB9 connection, G5 manual Appendix A.1.1 p. 156
// Looking at front of male connector
// 
//  1 2 3 4 5		1 CAN-H   	2 CAN-L  	3 UNIT-ID	4 RX	6 TX
//   6 7 8 9              6 Sig GND 	7. 12V 		8 12V	9 GND


// Pinout for custom board - DB9 looking at front of male 
// 	1 2 3 4 5		1 CAN-H		2 CAN-L		3 ADC IN	4 TX	5 Relay Common
//   6 7 8 9 				6 Relay1	7. 12V		8 Relay2  9 GND 

// Pinout for TTGO-TS looking at back of board connector left top
//
//               VN  VP
//  ADC (green)	 33	 RST
//  TX (yellow)  27  32  (yellow)   CAN 
//               GND 26  (purple)   CAN
//               00  GND (black)    GND to CAN
//     (black)   GND 3.3 (red)      Power to CAN
//               RX  21  (yellow)   relay2
//               TX  22  (orange)   relay1
//               VB  5.0 (red) 		Power in
//
//


// esp32s3 board
const struct { 
	int canTx = 9; /* yellow */
	int canRx = 8; /* green */
	int serialTx = 7; // blue
	int serialRx = 44; /* brown */
} pins;

#if 0
#include "ulp.h"
#include "soc/rtc_io_reg.h"
#include "driver/rtc_io.h"
void ulp_go() { 
	int pin = rtc_io_number_get(GPIO_NUM_2);
	rtc_gpio_init(GPIO_NUM_2);
	rtc_gpio_init(GPIO_NUM_2);
	rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
	Serial.printf("rtc_io %d\n", pin);
	const ulp_insn_t program[] = {
		M_LABEL(1),
		I_WR_REG(RTC_GPIO_OUT_REG, 
				pin + RTC_GPIO_OUT_DATA_S, 
				pin + RTC_GPIO_OUT_DATA_S, 1),  // LED ON
		I_MOVI(R0, 100),
		M_LABEL(2),
		I_DELAY(5000),
		I_SUBI(R0, R0, 1),
		M_BGE(2, 1),
		I_WR_REG(RTC_GPIO_OUT_REG, 
			pin + RTC_GPIO_OUT_DATA_S, 
			pin + RTC_GPIO_OUT_DATA_S, 0),  // LED OFF
		I_MOVI(R0, 100),
		M_LABEL(3),
		I_DELAY(50000),
		I_SUBI(R0, R0, 1),
		M_BGE(3, 1),
		M_BX(1),
	};

	size_t load_addr = 0;
	size_t size = sizeof(program)/sizeof(ulp_insn_t);
	ulp_process_macros_and_load(load_addr, program, &size);
	ulp_run(load_addr);
}
#endif

//WiFiUDP udp;
//const char *udpHost = "192.168.4.100";
//const char *udpHost = "255.255.255.255";

// port schema:  7890 - g5 in 
//               7891 - g5 out
//               7892 - ifly in
//               7893 - ifly out
//				 7894 - debug can data out


// OBS knob toggle mode settings
// 0-3: HDG mode
// 4: NAV mode
// 5: ILS simulation
// 6: CDI needle test movement

SL30 sl30;

bool otaInProgress = false;
int udpCanOut = 0; // debug output, send all CAN data out over udp port
static int cmdCount = 0;
EggTimer blinkTimer(500), screenTimer(200);
EggTimer pidTimer(250);
static uint8_t buf[1024];

float setPoint = -1;
int lastCmdTime = 0, lastCmdMs = 0, lastCmdPin = 0;
float startTrimPos = 0;

//RollingAverage<long int,1000> loopTimeAvg;

struct DummyLoopTime { 
	float average() { return -1; }
	float min() { return -1; }
	float max() { return -1; }
	void add(float) {}
} loopTimeAvg;
uint64_t lastLoopTime = -1;
EggTimer serialReportTimer(500), displayTimer(1000);
EggTimer canReportTimer(100), canResetTimer(5000), sl30Heartbeat(1000), sdCardFlush(2000), ilsWaitReportTimer(2000);

uint64_t nextCmdTime = 0;
static bool debugMoveNeedles = false;

static bool first = true;
ReliableStreamESPNow espnow("G5", true/*alwaysBroadcast*/);
static int espnowPackets = 0;

// send udp packet multiple times on broadcast address and all addresses in normal EchoUAT wifi range 
void superSend(const char *b) { 
	espnow.write(string(b));
	espnowPackets++;
	//Serial.print(b);
}

struct IsrData {
	float pitch, roll, knobVal = 0, magHdg, magTrack, ias, tas, palt, slip;
	int knobSel, mode;
	bool forceSend, hasPalt;
	uint32_t timestamp;
	uint8_t *bufIn, *bufOut; 
	int exceptions;
	int udpSent;
	int udpErrs;
} isrData, lastSent;

void resetIlsSimulator() {
	if (ils != NULL) {
		delete ils;
		ils = NULL;
	}
}

void setMode(int newMode) {
	if (isrData.mode == newMode)
		return;

	bool touchesIlsMode = isrData.mode == 5 || newMode == 5;
	isrData.mode = newMode;
	// Entering mode 5 starts fresh; leaving mode 5 tears down the active simulation.
	if (touchesIlsMode)
		resetIlsSimulator();
}

float knobValues[10];
int canSerial = 0;
void canToText(const uint8_t *ibuf, int lastId, int mpSize, uint32_t ts, char *obuf, int obufsz);

class CanWrapper {
	//Mutex canMutex;
	struct CanPacket { 
		uint8_t buf[8];
		uint64_t timestamp;
		int len;
		int id;
		int maxLen;
	};

	CircularBoundedQueue<CanPacket> pktQueue = CircularBoundedQueue<CanPacket>(256);
	//void onCanPacket(int id, int len, int timestamp, const uint8_t *buf) {}
	void (*onCanPacket)(int, int, uint32_t, const uint8_t *) = NULL;
	
public:
	int getQueueLen() { return pktQueue.getCount(); } 
	int isrCount = 0, pktCount = 0, dropped = 0;
	void onReceive(	void (*f)(int, int, uint32_t, const uint8_t *)){ onCanPacket = f; }
	vector<uint32_t> filters;
	void begin() {
		twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL, 
			.tx_io = (gpio_num_t) pins.canTx, 
			.rx_io = (gpio_num_t) pins.canRx, \
			.clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED,      \
			.tx_queue_len = 1, .rx_queue_len = 32,       \
			.alerts_enabled = TWAI_ALERT_NONE,  
			.clkout_divider = 0,        \
			.intr_flags = ESP_INTR_FLAG_LEVEL1
		};
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
		twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();

		twai_driver_install(&g_config, &t_config, &f_config);
		twai_start();
		//Serial.println("CAN OPENED");
		instancePtr = this;
	}
	void end() { 
	}
	void reset() { 
	}
	void run(int timeout, int maxPkts = -1) { 
		char obuf[128];
		twai_message_t cp;
		while(maxPkts != 0 && twai_receive(&cp, timeout) == ESP_OK) {
			isrCount++;
			if (0) { 
				printf("CAN %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " %1d",
					micros(), cp.flags, cp.identifier, cp.data_length_code);
				if (cp.rtr == 0) {
					for (int n = 0; n < cp.data_length_code; n++) 
					printf(" %02x", cp.data[n]);
				}
				printf("\n");
			}
			uint32_t timestamp = millis();
			if (maxPkts > 0) 
				maxPkts--;

			if (canSerial) {
				//canToText(cp.buf, cp.id, cp.len, cp.timestamp, obuf, sizeof(obuf));
				//Serial.print("C "); 
				//Serial.print(obuf);
			}

			bool filtMatch = false;
			for(auto f : filters) { 
				if (cp.identifier == f)
					filtMatch = true;
			}
			if (filtMatch || filters.size() == 0) { 
				pktCount++;
				if (onCanPacket != NULL)
					onCanPacket(cp.identifier, cp.data_length_code, timestamp, cp.data);
			}
		}
	}
	static CanWrapper *instancePtr;
};

CanWrapper *CanWrapper::instancePtr = NULL;

CanWrapper *can = NULL;
uint8_t canDebugBuf[1024];
//PidControl pid(2);
//static RollingAverage<int,1000> trimPosAvg;

struct CanChannel {
	uint32_t channel;
	uint32_t chanMask;
	uint32_t header;
	uint32_t headerMask;
	uint8_t buf[256];
	uint32_t lastPktTs = 0;
	int len = 0;
	CanChannel(uint32_t c, uint32_t m = 0xffffffff, uint32_t h = 0xffffffff, uint32_t hm = 0xffffffff)
	 : channel(c), chanMask(m), header(h), headerMask(hm) {}

	bool available(uint32_t chan, const uint8_t *ibuf, int l, uint32_t ts) { 
		if (ts - lastPktTs > 2000 && len > 0) 
			return true;
		if (len == 0 || (channel & chanMask) != (chan & chanMask))
			return false;
		if (l >= 4) { 
			uint32_t firstWord = (ibuf[0] << 24) | (ibuf[1] << 16) | (ibuf[2] << 8) | ibuf[3];
			if ((firstWord & headerMask) == (header & headerMask))
				return true;
		}
		return false;
	}
	bool add(uint32_t chan, const uint8_t *data, int l, uint32_t ts) { 
		if ((channel & chanMask) != (chan & chanMask))
			return false;
		for(int i = 0; i < l; i++) {
			if (len + i < sizeof(buf))
				buf[len + i] = data[i];			
		}
		len = min((int)sizeof(buf), len + l);
		lastPktTs = ts;
		return true;
	}
};

vector<CanChannel> channels;
void canParse(int id, int len, uint32_t timestamp, const uint8_t *ibuf);

void onPacket(int id, uint32_t timestamp, vector<CanChannel>::iterator i) { 
	canParse(id, i->len, timestamp, i->buf);
	i->len = 0;
}
void canSort(int id, int len, uint32_t timestamp, const uint8_t *ibuf) { 
	// The timestamp coming in doesn´t seem to compare well with micros(),
	// maybe the two cores have different micros() 
	//Serial.printf("0x%08x 0x%08x\n", timestamp, micros());
	//timestamp = micros(); 

	// channels timed out? 
	for(auto i = channels.begin(); i != channels.end(); i++) { 
		if (i->available(id, ibuf, len, timestamp)) 
			onPacket(i->channel, timestamp, i);
	}
	for(auto i = channels.begin(); i != channels.end(); i++) { 
		if (i->add(id, ibuf, len, timestamp))
			break;
	}
}


void sendDebugData() { 
	char sbuf[160];				
	//snprintf(sbuf, sizeof(sbuf), "%d %f %s DEBUG\n", 
	//	(int)can->pktCount, (float)trimPosAvg.average(), WiFi.localIP().toString().c_str());
	//superSend(sbuf);
	//Serial.printf(sbuf);
}

void sendData(const char *format, ...) { 
    va_list args;
    va_start(args, format);
	char buf[128];
	vsnprintf(buf, sizeof(buf), format, args);
	superSend(buf);
    va_end(args);
}

void sendCanData() { 
	char sbuf[256];				
	int age = millis() - isrData.timestamp;
	snprintf(sbuf, sizeof(sbuf),
		"P=%.2f R=%.2f HDG=%.2f TRK=%.2f IAS=%.1f TAS=%.1f PALT=%.1f MODE=%d "
		"K=%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
		isrData.pitch * 180 / M_PI, isrData.roll * 180 / M_PI, 
		isrData.magHdg * 180 / M_PI, isrData.magTrack * 180 / M_PI, 
		isrData.ias / MPS_PER_KNOT, isrData.tas / MPS_PER_KNOT, 
		isrData.palt, isrData.mode, 
		knobValues[0], knobValues[1], knobValues[2],
		knobValues[3], knobValues[4], knobValues[5]);
	//snprintf(sbuf, sizeof(sbuf), "%+06.3f %+06.3f %+06.3f %+06.3f %+06.3f %+06.3f %+06.3f %d %+06.3f %d %d CAN\n", 
	//	isrData.pitch, isrData.roll, isrData.magHdg, isrData.magTrack, isrData.ias, isrData.tas, 
	//	isrData.palt, isrData.knobSel, isrData.knobVal, age, isrData.mode);
	superSend(sbuf);
	lastSent = isrData;
	//Serial.printf("%d\t%d\t%s", isrData.udpSent, isrData.count, sbuf);
}


float floatFromBinary(const uint8_t *b) { 
	char buf[4];
	for (int n = 0; n < 4; n++) {
		buf[n] = b[n];
	}
	float rv = 0;
	try {
		rv = *(float *)&buf[0];
	} catch(...) { 
		rv = 0;
	}
	return rv;
}

void canToText(const uint8_t *ibuf, int lastId, int mpSize, uint32_t ts, char *obuf, int obufsz) { 
	snprintf(obuf, obufsz, "%08x %08x %02d ", ts, lastId, mpSize);
	int outlen = strlen(obuf);
	for(int n = 0; n < mpSize; n++) {
		if (n > 0 && (n % 8) == 0) { 
			snprintf(obuf + outlen, obufsz - outlen, " ");
			outlen += 1;
		} 
		snprintf(obuf + outlen, obufsz - outlen, "%02x", ibuf[n]);
		outlen += 2;
	}
	obuf[outlen++] = '\n';
	obuf[outlen] = 0;				
}
int serialBytesIn = 0;

uint32_t lastSendMs = 0; // rate gate only Pitch/Roll sends  
static const int sendMinMs = 50;

void canParse(int id, int len, uint32_t timestamp, const uint8_t *ibuf) { 
	int lastId = id;
	int mpSize = len;
	static char obuf[1024];
	uint32_t nowMs = millis();
	if (canSerial) {
		canToText(ibuf, id, len, timestamp, obuf, sizeof(obuf));
		Serial.printf("P "); 
		Serial.print(obuf);
	}

#if 0 	
	for (int i = 0; i < len / 4; i++) { 
		float f = floatFromBinary(&ibuf[i * 4]);
		if (f >= -100 & f <= -50  ) { 
			Serial.printf("found float %f between 1000-2000 at byte %d, packet:\n", f, i * 4);
			canToText(ibuf, lastId, mpSize, millis(), obuf, sizeof(obuf));
			Serial.print(obuf);
		}
	}
#endif


#if 0 
	// maybe is GPS altitude? 
	if ((lastId == 0x18882100 || lastId == 0x188c2100) && 
		ibuf[0] == 0xc1 && ibuf[1] == 0x0b && mpSize == 84) {
		float thresh = 0.1;
		try {
			float palt = floatFromBinary(&ibuf[64]);
			if (abs(palt - lastSent.palt) > thresh && palt > 200) { 
				sendData("PALT=%.5f\n", palt);
				lastSent.palt = palt;
				isrData.palt = palt;
			}
		} catch(...) {
			//isrData.palt = 0;
		}
	} 
#endif

	if ((lastId == 0x18882100 || lastId == 0x188c2100) 
		&& ibuf[0] == 0xdd && ibuf[1] == 0x00 && mpSize >= 52 && ibuf[15] & 0x40) {
		float thresh = 0.01;
		try {
			float magHdg = floatFromBinary(&ibuf[16]);
			isrData.magHdg = magHdg;
		} catch(...) {
			isrData.magHdg = 0;
		}
	} 
	if ((lastId == 0x18882100 || lastId == 0x188c2100) 
	&& ibuf[0] == 0xdd && ibuf[1] == 0x00 && mpSize >= 52  
	&& ibuf[15] & 0x20) {
		float thresh = 0.01;
		try {
			float magTrack = floatFromBinary(&ibuf[16]);
			isrData.magTrack = magTrack;
	
		} catch(...) {
			isrData.magTrack = 0;
		}
	} 	
	if (lastId == 0x18882100 
		&& ibuf[0] == 0xdd && ibuf[1] == 0x00 && mpSize == 60) {
		float thresh = 0.0005;
		try {
			float pitch = floatFromBinary(&ibuf[20]);
			float roll = floatFromBinary(&ibuf[24]);
			isrData.timestamp = millis();
			isrData.pitch = pitch;
			isrData.roll = roll;

		} catch(...) {
			isrData.pitch = isrData.roll = 0; 
			isrData.exceptions++;
		}
		//sendCanData(false);
	}
#if 0 
	if (lastId == 0x18882100 && ibuf[0] == 0xdc && ibuf[1] == 0x02 && mpSize >= 16) {
		float thresh = .01;
		try {
			float slip = floatFromBinary(&ibuf[12]); 
			if (slip != lastSent.slip) {
				sendData("SL=%f", slip * 180/M_PI);
				lastSent.slip = slip;
			}
			isrData.slip = slip;
		} catch(...) {
			isrData.slip = 0; 
			isrData.exceptions++;
		}
	}
#endif
	/*
	if (lastId == 0x18882100 && ibuf[0] == 0xdd && ibuf[1] == 0x0a && mpSize >= 32) {
		try {
			float palt = floatFromBinary(&ibuf[28]);
			isrData.palt = palt;
			sendData("PALT=%f", palt);
			Serial.printf("PALT=%f\n", palt);
		} catch(...) {
			isrData.ias = isrData.tas = isrData.palt = 0; 
			isrData.exceptions++;
		}
	}
	*/
	if (lastId == 0x18882100 && ibuf[0] == 0xdd && ibuf[1] == 0x0a && mpSize == 52) {
		float thresh = 0.1;
		try {
			float ias = floatFromBinary(&ibuf[12]); 
			float tas = floatFromBinary(&ibuf[16]);
			float palt = floatFromBinary(&ibuf[28]);
			isrData.timestamp = millis();
			isrData.ias = ias; 
			isrData.palt = palt;
			isrData.hasPalt = true;
			isrData.tas = tas;
		} catch(...) {
			isrData.ias = isrData.tas = isrData.palt = 0; 
			isrData.hasPalt = false;
			isrData.exceptions++;
		}
		//sendCanData(false);
	}
	if ((lastId == 0x10882200 || lastId == 0x108c2200) 
		&& ibuf[0] == 0xe4 && ibuf[1] == 0x65 && mpSize == 7) {
		try {
			double knobVal = floatFromBinary(&ibuf[3]);
			int knobSel = ibuf[2];					
			isrData.knobVal = knobVal;
			isrData.knobSel = knobSel;
			if (knobSel >= 0 && knobSel < sizeof(knobValues)/sizeof(knobValues[0]))
				knobValues[knobSel] = knobVal;
			if (knobSel > 0 && knobSel < sizeof(g5KnobValues)/sizeof(g5KnobValues[0]))
				g5KnobValues[knobSel] = knobVal;
		} catch(...) {
			isrData.knobSel = isrData.knobVal = 0;
			isrData.exceptions++; 
		}
		Serial.printf("Knob %d val %f\n", (int)isrData.knobSel, isrData.knobVal);
		isrData.forceSend = true;
		//sendCanData(true);
	}

	static float lastKnobVal = 0;
	static uint64_t lastKnobMillis = 0;
	if ((isrData.knobSel == 1/*hdg*/ || isrData.knobSel == 5)  && isrData.knobVal != lastKnobVal) { 
		if (millis() - lastKnobMillis > 2500) 
			setMode(0);
		lastKnobMillis = millis();
		double delta = isrData.knobVal - lastKnobVal;
		bool oneDegree = abs(delta) < 1.5/180*M_PI && abs(delta) > 0.5/180*M_PI;
		bool evenMode = (isrData.mode & 0x1) == 0;
		if (false && isrData.knobSel == 0) { // for debugging, use altimeter adjustment for knob input too 
			oneDegree = abs(delta) < 35 && abs(delta) > 30;
		}
		if (oneDegree && ((delta > 0 && evenMode) || (delta < 0 && !evenMode))) {
			setMode(isrData.mode + 1);
		} else if (isrData.mode != 0) { 
			setMode(0);
		}
		lastKnobVal = isrData.knobVal;
	}	
}

bool wifiTryConnectNO(const char *ap, const char *pw, int seconds = 15) { 
	if (WiFi.status() != WL_CONNECTED) {
		WiFi.begin(ap, pw);
		for(int d = 0; d < seconds * 10 && WiFi.status() != WL_CONNECTED; d++) {
			delay(100);
		}
	}
	return WiFi.status() == WL_CONNECTED;
}

void updateFixFromNmea() {
	if (!nmeaGps.location.isValid() || nmeaGps.location.age() > GPS_FIX_STALE_MS)
		return;
	navFix.lat = nmeaGps.location.lat();
	navFix.lon = nmeaGps.location.lng();
	if (nmeaGps.course.isValid() && nmeaGps.course.age() <= GPS_FIX_STALE_MS) {
		navFix.track = nmeaGps.course.deg();
		navFix.hasTrack = true;
	} else {
		navFix.hasTrack = false;
	}
	if (nmeaGps.altitude.isValid() && nmeaGps.altitude.age() <= GPS_FIX_STALE_MS) {
		navFix.altMeters = nmeaGps.altitude.meters();
		navFix.hasAltitude = true;
		sensorFusion.updateGpsAltitude(
			navFix.altMeters,
			millis() - nmeaGps.altitude.age(),
			isrData.palt,
			isrData.hasPalt);
	} else {
		navFix.hasAltitude = false;
	}
	if (nmeaGps.speed.isValid() && nmeaGps.speed.age() <= GPS_FIX_STALE_MS) {
		navFix.hvelKnots = nmeaGps.speed.knots();
		navFix.hasSpeed = true;
	} else {
		navFix.hasSpeed = false;
	}
	navFix.timestamp = millis() - nmeaGps.location.age();
	if (navFix.hasTrack && navFix.hasSpeed) {
		sensorFusion.updateGpsPosition(
			navFix.lat,
			navFix.lon,
			navFix.track,
			navFix.hvelKnots,
			navFix.timestamp);
	}
	navFix.updated = true;
	navFix.valid = true;
}

bool hasFreshGpsFix() {
	return navFix.valid && (uint32_t)(millis() - navFix.timestamp) <= GPS_FIX_STALE_MS;
}

float fusedGpsAltitudeMeters() {
	return sensorFusion.hasFusedAltitude()
		? sensorFusion.fusedAltitudeMeters(navFix.altMeters, isrData.palt, isrData.hasPalt)
		: navFix.altMeters;
}

void expireStaleGpsFix() {
	if (navFix.valid && !hasFreshGpsFix()) {
		navFix.valid = false;
		navFix.updated = false;
		navFix.hasTrack = false;
		navFix.hasSpeed = false;
		navFix.hasAltitude = false;
	}
}

// Serial and ESP-NOW command paths can both provide NMEA, so parse by line here.
void parseNmea(const char *buf, int n) {
	for (int i = 0; i < n; i++) {
		nmeaGps.encode(buf[i]);
		if (buf[i] == '\r' || buf[i] == '\n')
			updateFixFromNmea();
	}
}

void parseNmeaLine(const char *line) {
	parseNmea(line, strlen(line));
	parseNmea("\n", 1);
}
	
void processCommand(const char *buf, int r) {
//static PinPulse pp[] = { PinPulse(pins.relay1), PinPulse(pins.relay2) };
		static LineBuffer line;
		for (int i = 0; i < r; i++) {
			int ll = line.add(buf[i]);
			if (ll) {
				cmdCount++;
				Serial.printf("LINE: %s", line.line);
				int ms, val, pin, seq;
				float f;
				static int lastSeq = 0;
				if (sscanf(line.line, "trim %f %d", &f, &seq) == 2 ) {
					setPoint = f;
					lastSeq = seq;
				}
				if (sscanf(line.line, "cdi %f", &f) == 1 ) {
					debugMoveNeedles = f;
				}
				if (sscanf(line.line, "canudp %f", &f) == 1 ) {
					udpCanOut = f;
				}
				if (sscanf(line.line, "canserial %f", &f) == 1 ) {
					canSerial = f;
				}
				if (strstr(line.line, "NMEA=") == line.line) {
					parseNmeaLine(line.line + 5);
				}
				if (line.line[0] == '$' || line.line[0] == '!') {
					parseNmeaLine(line.line);
				}
				if (strstr(line.line, "PMRRV")  != NULL) { 
					Serial2.write((uint8_t *)line.line, strlen(line.line));
					//Serial.printf("G5: %s", line.line);
				}
			}
		}
}

void setup() {
	wdtInit(15);
	//pinMode(pins.led, OUTPUT);
	//digitalWrite(pins.led, 1);
	Serial.begin(921600);
	Serial.setTimeout(10);
	while(0)  { 
		int p = pins.serialRx;
		Serial.printf("AUTOTRIM %d\n", p);
		pinMode(p, OUTPUT);
		digitalWrite(p, 1);
		delay(1000);
		digitalWrite(p, 0);
		delay(1000);
		wdtReset();
	}

	Serial2.begin(9600, SERIAL_8N1, pins.serialRx, pins.serialTx);
	Serial2.setTimeout(10);
	
	//pinMode(pins.button, INPUT_PULLUP);
//	pinMode(3,INPUT);

	//adcAttachPin(pins.ADC);
	//analogSetCycles(255);
	//pid.setGains(4, 0, 0);
	//pid.finalGain = 1;

	can = new CanWrapper();
	can->onReceive(canSort);
	//can->filters.push_back(0x18882100);
	//can->filters.push_back(0x10882200);
	//can->filters.push_back(0x108c2200);

	channels.push_back(CanChannel(0x18882100, -1, 0xdd000000, 0xfff00000));
	channels.push_back(CanChannel(0x188c2100, -1, 0xdd000000, 0xfff00000));
	channels.push_back(CanChannel(0x18882200, -1, 0xdd000000, 0xfff00000));
	channels.push_back(CanChannel(0x188c2200, -1, 0xdd000000, 0xfff00000));
	//channels.push_back(CanChannel(0x10882200, -1, 0xe4650000, 0xffff0000));
	//channels.push_back(CanChannel(0x108c2200, -1, 0xe4650000, 0xffff0000));
	channels.push_back(CanChannel(0x10882200, -1, 0x0, 0x0));
	channels.push_back(CanChannel(0x108c2200, -1, 0x0, 0x0));
	channels.push_back(CanChannel(0, 0));
	can->begin();
}
EggTimer report(2000);

void loop() {
	uint64_t now = micros();
	wdtReset();

	ArduinoOTA.handle();
	if (otaInProgress) {
		wdtInit(220);
		can->reset();
		return;
	}
	can->run(pdMS_TO_TICKS(5), 100);

	
	while (Serial.available()) { 
		static char buf[512];
		int n = Serial.read(buf, sizeof(buf));
		processCommand(buf, n);
	}

	while (Serial2.available()) { 
		static LineBuffer lb;
		static char buf[512];
		serialBytesIn++;
		int n = Serial2.read(buf, sizeof(buf));
		//Serial.printf("Serial read %d bytes\n", n);
		lb.add(buf, n, [](const char *line) {
			parseNmeaLine(line);
			sendData("NMEA=%s", line);
			//Serial.printf("NMEA: %s\n", line);
		});

	}
	if (lastLoopTime != -1) 
		loopTimeAvg.add(now - lastLoopTime);
	lastLoopTime = now;

	if (canResetTimer.tick()) {
		can->reset();
	}
	
	string s = espnow.read();
	if (s.length()) { 
		processCommand(s.c_str(), s.length());
	}

	if (canReportTimer.tick()/* || isrData.forceSend*/) { 
		isrData.forceSend = false;
		//Serial.printf("%08d %d\n", (int)millis(), isrData.mode);
		sendCanData();
	}

	if (serialReportTimer.tick()) {
		//digitalWrite(pins.led, !digitalRead(pins.led));
		char buf[256]; 
		snprintf(buf, sizeof(buf), "L: %08.3f %05.3f/%05.3f/%05.3f "
		"m:%d appkt: %d can:%d drop:%d qlen:%d serIn:%d "
		"%d\n", 
		millis() / 1000.0, 
		loopTimeAvg.average()/1000.0, loopTimeAvg.min()/1000.0, loopTimeAvg.max()/1000.0, 
		isrData.mode, espnowPackets, can->isrCount, can->dropped, can->getQueueLen(), serialBytesIn,
		0);
		Serial.print(buf);

#if 0 
		twai_message_t p;
		static bool alternate = 0;
		alternate = !alternate;

		p.flags = 0x1; 
		//p.identifier = 0x108a2222; // net err conflict
		//p.identifier = 0x108e2222; // no conflict 
		//p.identifier = 0x108c2200; // no conflict
		//p.identifier = 0x10882200; // network err conflict 
		p.identifier = 0x108c2222;
		p.data_length_code = 7;
		p.data[0] = 0xe4;
		p.data[1] = 0x65;
		p.data[2] = 0x00;
		if (alternate) { 
			p.data[3] = 0x28;
			p.data[4] = 0x17;
			p.data[5] = 0xc6;
			p.data[6] = 0x47;
		} else { 
			p.data[3] = 0x17;
			p.data[4] = 0x28;
			p.data[5] = 0xc6;
			p.data[6] = 0x47;
		}
		int r = twai_transmit(&p, pdMS_TO_TICKS(100));
		printf("twai_transmit returned %d\n", r);
#endif
	}

	static int startupPeriod = 20000;
	if (millis() > startupPeriod)
		startupPeriod = 0;
	
	if (sl30Heartbeat.tick()) {
#if 0
		if (0) { 
			//ScopedMutex lock(canMutex);  // panics ISR when ISR blocks too long 
			CAN.beginExtendedPacket(0x10882200);
			static bool alternate = 0;
			alternate = !alternate;
			if (alternate) { 				
				CAN.write(0xe4);
				CAN.write(0x65);
				CAN.write(0x02);
				CAN.write(0x67);
				CAN.write(0x66);
				CAN.write(0x98);
				CAN.write(0x43);
			} else { 
				CAN.write(0xe4);
				CAN.write(0x65);
				CAN.write(0x02);
				CAN.write(0xf6);
				CAN.write(0x28);
				CAN.write(0x89);
				CAN.write(0x43);
			}			
			CAN.endPacket();
			// alt bug change		
			//e4650267669843
			//e46502f6288943

			// hdg bug change
			//e465011fc92940
			//e4650113e72a40
		}	
#endif // #if 0 
		std::string s = sl30.pmrrv("301234E"); // send arbitary NAV software version packet as heartbeat
		Serial2.print(s.c_str());
		//Serial.print(s.c_str());
	}
	
	static EggTimer g5Timer(100);

	if (g5Timer.tick()) { 
		expireStaleGpsFix();
		if (debugMoveNeedles || isrData.mode == 6 || startupPeriod > 0) { 
			hd += .05;
			vd += .075;
			if (hd > 2) hd = -2;
			if (vd > 2) vd = -2;
			if (millis() - isrData.timestamp > 1000) 
				hd = vd = -2;
			std::string s = sl30.setCDI(hd, vd);
			Serial2.print(s.c_str());
			//Serial.print(s.c_str());
		}
		if (isrData.mode == 5 && hasFreshGpsFix() && navFix.hasTrack && navFix.hasSpeed && navFix.hasAltitude) {
			SensorFusion::Position fusedPos = sensorFusion.fusedPosition(millis());
			LatLon now(fusedPos.lat, fusedPos.lon);
			float altMeters = fusedGpsAltitudeMeters();
			if (ils == NULL) {
				float vlocTrk = g5KnobValues[4] * 180/M_PI;
				float altBug = g5KnobValues[2];
				// Historical mode-5 behavior has two ILS entry paths:
				// - VLOC/OBS course set: synthesize a fictional ILS ahead of the current GPS track.
				// - VLOC/OBS course zero: choose the nearest compatible approach from the built-in list.
				// The choice happens only when ils is created; leave and re-enter mode 5 to switch paths.
				if (vlocTrk != 0) {
					const float gs = 3.0;
					float tdze = altBug - 200 / 3.281;
					// Put the final approach intercept 3 km ahead, then project to the touchdown point
					// using the selected course and a 3 degree glideslope from the current altitude.
					LatLon facIntercept = locationBearingDistance(now, navFix.track, 3000);
					float facDist = (altMeters - tdze) / tan(gs * M_PI/180);
					LatLon tdz = locationBearingDistance(facIntercept, magToTrue(vlocTrk), facDist);
					// The simulator uses TDZ for glide slope, and a projected localizer antenna
					// DEFAULT_RUNWAY_LENGTH_FT + 1000 ft past TDZ for lateral CDI sensitivity.
					ils = new IlsSimulator(tdz, tdze, magToTrue(vlocTrk), gs);
				} else {
					Approach *a = findBestApproach(now);
					if (a != NULL)
						ils = new IlsSimulator(LatLon(a->lat, a->lon), a->tdze / 3.281, magToTrue(a->fac), a->gs);
				}
				if (ils != NULL) {
					Serial.print("Started ILS, ");
					Serial.println(ils->toString().c_str());
				}
			}
			if (ils != NULL) {
				ils->setCurrentLocation(now, altMeters);
				hd = ils->cdiPercent() * 2.0;
				vd = ils->gsPercent() * 2.0;
				std::string s = sl30.setCDI(hd, vd);
				Serial2.print(s.c_str());
			}
		} else if (isrData.mode == 5 && (!navFix.valid || !navFix.hasTrack || !navFix.hasSpeed || !navFix.hasAltitude) && ilsWaitReportTimer.tick()) {
			if (!navFix.valid)
				Serial.println("ILS mode waiting for GPS fix");
			else if (!navFix.hasTrack)
				Serial.println("ILS mode waiting for GPS track");
			else if (!navFix.hasSpeed)
				Serial.println("ILS mode waiting for GPS speed");
			else
				Serial.println("ILS mode waiting for GPS altitude");
		} else if (ils != NULL) {
			resetIlsSimulator();
		}
	}
	delay(1);
	yield();
}


#ifdef CSIM
////////////////////////////////////////////////////////////////////////////////////////
// the rest of the file is simulation support code 


class ESP32sim_autotrim : Csim_Module { 
	std::ifstream trackSimFile;
	WaypointNav::WaypointSequencer tSim = WaypointNav::WaypointSequencer(trackSimFile);
	int last_us = 0;	
	bool makeKml = false;
	IntervalTimer hz100 = IntervalTimer(100/*msec*/);
	int loopcount = 0;
	bool trackSimActive = false;
	int lastTrackSimReportMs = -1000;

	void parseArg(char **&a, char **la) override {
		printf("at parse arg\n");
		if (strcmp(*a, "--kml") == 0) makeKml = true;
		if (strcmp(*a, "--canfile") == 0) CAN.setSimFile(*(++a)); 
		if (strcmp(*a, "--canserial") == 0) canSerial = 1;
		if (strcmp(*a, "--tracksim") == 0) {
			trackSimFile = std::ifstream(*(++a), std::ios_base::in | std::ios_base::binary);
			trackSimActive = true;
		}
	}
	void setup() override {}

	void updateInputs() {
		if (tSim.inputs.find("MODE") != tSim.inputs.end())
			setMode(tSim.inputs["MODE"]);
		if (tSim.inputs.find("HDGBUG") != tSim.inputs.end())
			g5KnobValues[1] = DEG2RAD(tSim.inputs["HDGBUG"]);
		if (tSim.inputs.find("ALTBUG") != tSim.inputs.end())
			g5KnobValues[2] = tSim.inputs["ALTBUG"];
		if (tSim.inputs.find("VLOCBUG") != tSim.inputs.end())
			g5KnobValues[4] = DEG2RAD(tSim.inputs["VLOCBUG"]);
	}

	void publishTrackSimState() {
		LatLonAlt p = tSim.wptTracker.curPos;
		if (!p.valid)
			return;

		float actualTrack = tSim.wptTracker.commandTrack;
		if (tSim.wptTracker.prevPos.valid)
			actualTrack = bearing(tSim.wptTracker.prevPos.loc, p.loc);

		navFix.lat = p.loc.lat;
		navFix.lon = p.loc.lon;
		navFix.altMeters = p.alt;
		navFix.track = actualTrack;
		navFix.hvelKnots = tSim.wptTracker.speed;
		navFix.timestamp = millis();
		navFix.updated = true;
		navFix.valid = true;
		navFix.hasTrack = true;
		navFix.hasSpeed = true;
		navFix.hasAltitude = true;

		isrData.palt = p.alt * FEET_PER_METER;
		isrData.hasPalt = true;
		isrData.magTrack = DEG2RAD(trueToMag(navFix.track));
		isrData.timestamp = millis();
		sensorFusion.updateGpsPosition(
			navFix.lat,
			navFix.lon,
			navFix.track,
			navFix.hvelKnots,
			navFix.timestamp);
		sensorFusion.updateGpsAltitude(navFix.altMeters, navFix.timestamp, isrData.palt, isrData.hasPalt);
	}

	bool flyIlsNeedles(float sec) {
		if (!tSim.autoPilotOn || isrData.mode != 5 || ils == NULL || !tSim.wptTracker.curPos.valid)
			return false;
		if (abs(hd) >= 2.0)
			return false;

		float speedMps = tSim.wptTracker.speed * .51444f;

		tSim.wptTracker.commandTrack = constrain360(ils->faCrs + hd * 22.5f);
		tSim.wptTracker.prevPos = tSim.wptTracker.curPos;
		tSim.wptTracker.curPos.loc = locationBearingDistance(
			tSim.wptTracker.curPos.loc,
			tSim.wptTracker.commandTrack,
			speedMps * sec);

		float vvelMps = 0.0f;
		if (abs(vd) < 2.0) {
			float nominalDescent = speedMps * tan(DEG2RAD(ils->gs));
			vvelMps = -nominalDescent - vd * 1.5f;
		}
		tSim.wptTracker.curPos.alt = max(0.0f, tSim.wptTracker.curPos.alt + vvelMps * sec);
		tSim.wptTracker.vvel = vvelMps * 196.85f;
		tSim.wptTracker.xte = crossTrackErr(ils->tdzLocation, ils->locAntennaLocation, tSim.wptTracker.curPos.loc);
		return true;
	}

	void runTrackSim(float sec) {
		tSim.run(sec);
		updateInputs();

		if (!flyIlsNeedles(sec)) {
			if (tSim.autoPilotOn)
				tSim.wptTracker.setCDI(hd, vd, tSim.decisionHeight);
			if (tSim.wptTracker.curPos.valid && !tSim.wptTracker.waypointPassed)
				tSim.wptTracker.sim(sec);
		}

		publishTrackSimState();

		if (millis() - lastTrackSimReportMs >= 1000) {
			lastTrackSimReportMs = millis();
			LatLonAlt p = tSim.wptTracker.curPos;
			float range = p.valid && tSim.wptTracker.activeWaypoint.valid
				? distance(p.loc, tSim.wptTracker.activeWaypoint.loc)
				: -1;
			printf("TSIM t %.1f mode %d hd %.3f vd %.3f lat %.7f lon %.7f alt %.1f trk %.1f range %.1f xte %.1f\n",
				millis() / 1000.0, isrData.mode, hd, vd, navFix.lat, navFix.lon,
				(float)navFix.altMeters, navFix.track, range, tSim.wptTracker.xte);
		}
	}
			
	void loop() override {
		CAN.run();
		if (!hz100.tick(millis())) 
			return;
		if (trackSimActive)
			runTrackSim(hz100.interval / 1000.0);
	}  
		
	void done() override{
		//printf("gdl90 msgs %d errors %d\n", gdl90.msgCount, gdl90.errCount);
		exit(0);
	} 
} autotrim;
#endif
