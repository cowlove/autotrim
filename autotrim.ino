#ifdef UBUNTU
#include "ESP32sim_ubuntu.h"
#else
#include <CAN.h>
#include <TTGO_TS.h>
#endif 
	
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <istream>

#include "jimlib.h"
//#include "GDL90Parser.h"
//#include "WaypointNav.h"
#include "espNowMux.h"
#include "reliableStream.h"

//using namespace WaypointNav;
//#define LED_PIN 2

//WiFiMulti wifi;
//WiFiUDP udpG90;
//GDL90Parser gdl90;
//GDL90Parser::State state;

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

// TTGO Schema 
const struct PinAssignments { 
	int ADC = 33;
	int canTx = 32; /* yellow */
	int canRx = 26; /* purple */
	int relay1 = 21;
	int relay2 = 22;
	int serialTx = 27; 
	int serialRx = 0; /* unused */
	int button = 39;
} pins;

WiFiUDP udp;
//const char *udpHost = "192.168.4.100";
const char *udpHost = "255.255.255.255";

// port schema:  7890 - g5 in 
//               7891 - g5 out
//               7892 - ifly in
//               7893 - ifly out
//				 7894 - debug can data out


// OBS knob toggle mode settings
// 0-3: HDG mode
// 4: NAV mode
// 5: CDI needle test movement
// 6: CAN data udp dump (port 7894) 

int udpPortIn = 7892;

SL30 sl30;

File fd;
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
EggTimer serialReportTimer(1000), displayTimer(1000);
EggTimer pinReportTimer(500), canResetTimer(5000), sl30Heartbeat(1000), sdCardFlush(2000);

uint64_t nextCmdTime = 0;
static bool debugMoveNeedles = false;

static bool first = true;
ReliableStreamESPNow autopilot("G5");
static int autopilotPackets = 0;

// send udp packet multiple times on broadcast address and all addresses in normal EchoUAT wifi range 
void superSend(const char *b) { 
#if 0 
   for (int repeat = 0; repeat < 1; repeat++) { 
		udp.beginPacket("255.255.255.255", 7891);
		udp.write((uint8_t *)b, strlen(b));
		udp.endPacket();
		for (int n = 100; n < 103; n++) {
			if (WiFi.localIP()[3] != n) { 
				char ip[32];
				snprintf(ip, sizeof(ip), "192.168.4.%d", n);
				udp.beginPacket(ip, 7891);
				udp.write((const uint8_t *)b, strlen(b));
				udp.endPacket();
			}
		}
	}
#endif
	autopilot.write(string(b));
	autopilotPackets++;
	Serial.print(b);
}

struct IsrData {
	float pitch, roll, knobVal = 0, magHdg, magTrack, ias, tas, palt, slip;
	int knobSel, mode;
	bool forceSend;
	uint64_t timestamp;
	uint8_t *bufIn, *bufOut; 
	int exceptions;
	int udpSent;
	int udpErrs;
} isrData, lastSent;

float knobValues[10];

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
	uint8_t ibuf[1024];
	int mpSize = 0, lastId = 0;
	//void onCanPacket(int id, int len, int timestamp, const uint8_t *buf) {}
	void (*onCanPacket)(int, int, int, const uint8_t *) = NULL;
	
public:
	int getQueueLen() { return pktQueue.getCount(); } 
	int isrCount = 0, pktCount = 0, dropped = 0;
	void onReceive(	void (*f)(int, int, int, const uint8_t *)){ onCanPacket = f; }
	vector<uint32_t> filters;
	void begin() { 
		CAN.setPins(pins.canRx, pins.canTx);     
		if (!CAN.begin(1000E3)) {
			Serial.println("Starting CAN failed!");
			while (1) {}
		}
		Serial.println("CAN OPENED");
		CAN.filter(0,0);
		instancePtr = this;
		CAN.onReceive([](int len) { CanWrapper::instancePtr->isr(len); });
		mpSize = 0;
		lastId = -1;
	}
	void end() { 
		CAN.onReceive(NULL);
		CAN.end();
	}
	void reset() { CAN.filter(0,0); }
	void run(int timeout, int maxPkts = -1) { 
		CanPacket *p;
		while(maxPkts != 0 && (p = pktQueue.peekTail(timeout)) != NULL) { 
			CanPacket cp = *p;
			pktQueue.freeTail();
			if (maxPkts > 0) 
				maxPkts--;

			bool filtMatch = false;
			for(auto f : filters) { 
				if (cp.id == f)
					filtMatch = true;
			}
			if (filtMatch || filters.size() == 0) { 
				if (cp.id != lastId || mpSize >= sizeof(ibuf)) {
					pktCount++;
					if (onCanPacket != NULL && mpSize > 0) {
						onCanPacket((int)lastId, (int)mpSize, (int)cp.timestamp, ibuf);
					}
					mpSize = 0;
					lastId = cp.id;
				}
				for(int n = 0; n < cp.len && mpSize < sizeof(ibuf); n++) {
					ibuf[mpSize++] = cp.buf[n];
				}
			}
		}
	}
	void isr(int packetSize) {
		if (packetSize) {
			CanPacket *pkt = pktQueue.peekHead(0);
			if (pkt != NULL) {
				pkt->timestamp = millis();				
				pkt->id = CAN.packetId();
				pkt->len = packetSize;
				if (!CAN.packetRtr()) {
					for (int n = 0; n < min(packetSize, (int)sizeof(pkt->buf)); n++) {
						pkt->buf[n] = CAN.read();
					}
				}
				pktQueue.postHead();
			} else { 
				dropped++;
			}
		}
		isrCount++;
	}
	static CanWrapper *instancePtr;
};

CanWrapper *CanWrapper::instancePtr = NULL;

CanWrapper *can = NULL;
uint8_t canDebugBuf[1024];
//PidControl pid(2);
//static RollingAverage<int,1000> trimPosAvg;
int canSerial = 0;

void sendDebugData() { 
	char sbuf[160];				
	//snprintf(sbuf, sizeof(sbuf), "%d %f %s DEBUG\n", 
	//	(int)can->pktCount, (float)trimPosAvg.average(), WiFi.localIP().toString().c_str());
	//superSend(sbuf);
	//Serial.printf(sbuf);
}

void sendUdpCan(const char *format, ...) { 
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
		"P=%f R=%f HDG=%f TRK=%f IAS=%f TAS=%f PALT=%f MODE=%d "
		"KNOB0=%f KNOB1=%f KNOB2=%f\n",
		isrData.pitch * 180 / M_PI, isrData.roll * 180 / M_PI, 
		isrData.magHdg * 180 / M_PI, isrData.magTrack * 180 / M_PI, 
		isrData.ias / MPS_PER_KNOT, isrData.tas / MPS_PER_KNOT, 
		isrData.palt, isrData.mode, knobValues[0], knobValues[1], knobValues[2]);
	//snprintf(sbuf, sizeof(sbuf), "%+06.3f %+06.3f %+06.3f %+06.3f %+06.3f %+06.3f %+06.3f %d %+06.3f %d %d CAN\n", 
	//	isrData.pitch, isrData.roll, isrData.magHdg, isrData.magTrack, isrData.ias, isrData.tas, 
	//	isrData.palt, isrData.knobSel, isrData.knobVal, age, isrData.mode);
	superSend(sbuf);
	lastSent = isrData;
	fd.write(sbuf, strlen(sbuf));
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

void openLogFile(const char *filename) {
#ifndef UBUNTU
	open_TTGOTS_SD(); 
	int fileVer = 1;
	char fname[20];
	if (strchr(filename, '%')) { // if filename contains '%', scan for existing files  
		for(fileVer = 1; fileVer < 999; fileVer++) {
			snprintf(fname, sizeof(fname), filename, fileVer);
			if (!(fd = SD.open(fname, (F_READ)))) {
				fd.close();
				fd = SD.open(fname, (F_READ | F_WRITE | F_CREAT));
				Serial.printf("Opened %s\n", fname);
				return;
			}
		} 
	}
#endif

}

void canToText(const uint8_t *ibuf, int lastId, int mpSize, uint64_t ts, char *obuf, int obufsz) { 
	snprintf(obuf, obufsz, " (%.3f) can0 %08x [%02d] ", ts / 1000.0, lastId, mpSize);
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

void canParse(int id, int len, int timestamp, const uint8_t *ibuf) { 
	int lastId = id;
	int mpSize = len;
	static char obuf[1024];

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
	if (canSerial) {
		canToText(ibuf, lastId, mpSize, millis(), obuf, sizeof(obuf));
		Serial.print(obuf);
	}


#if 0 
	// maybe is GPS altitude? 
	if ((lastId == 0x18882100 || lastId == 0x188c2100) && 
		ibuf[0] == 0xc1 && ibuf[1] == 0x0b && mpSize == 84) {
		float thresh = 0.1;
		try {
			float palt = floatFromBinary(&ibuf[64]);
			if (abs(palt - lastSent.palt) > thresh && palt > 200) { 
				sendUdpCan("PALT=%.5f\n", palt);
				lastSent.palt = palt;
				isrData.palt = palt;
			}
		} catch(...) {
			//isrData.palt = 0;
		}
	} 
#endif

	if ((lastId == 0x18882100 || lastId == 0x188c2100) && ibuf[0] == 0xdd && ibuf[1] == 0x00 && mpSize == 60 && ibuf[15] & 0x40) {
		float thresh = 0.001;
		try {
			float magHdg = floatFromBinary(&ibuf[16]);
			if (abs(magHdg - lastSent.magHdg) > thresh) { 
				sendUdpCan("HDG=%.5f\n", magHdg * 180/M_PI);
				lastSent.magHdg = magHdg;
			}
			isrData.magHdg = magHdg;
		} catch(...) {
			isrData.magHdg = 0;
		}
	} 
	if ((lastId == 0x18882100 || lastId == 0x188c2100) && ibuf[0] == 0xdd && ibuf[1] == 0x00 && mpSize == 60 && ibuf[15] & 0x20) {
		float thresh = 0.001;
		try {
			float magTrack = floatFromBinary(&ibuf[16]);
			if (abs(magTrack - lastSent.magTrack) > thresh) {   
				sendUdpCan("TRK=%.5f\n", magTrack * 180/M_PI);
				lastSent.magTrack = magTrack;
			}
			isrData.magTrack = magTrack;
	
		} catch(...) {
			isrData.magTrack = 0;
		}
	} 	
	if (lastId == 0x18882100 && ibuf[0] == 0xdd && ibuf[1] == 0x00 && mpSize == 60) {
		float thresh = 0.0001;
		try {
			float pitch = floatFromBinary(&ibuf[20]);
			float roll = floatFromBinary(&ibuf[24]);
			isrData.timestamp = millis();
			if (abs(pitch - lastSent.pitch) > thresh || 
				abs(roll - lastSent.roll) > thresh) {
				sendUdpCan("P=%f R=%f\n", pitch * 180/M_PI, roll * 180/M_PI);
				lastSent.pitch = pitch; 
				lastSent.roll = roll;
			}
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
				sendUdpCan("SL=%f", slip * 180/M_PI);
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
			sendUdpCan("PALT=%f", palt);
			Serial.printf("PALT=%f\n", palt);
		} catch(...) {
			isrData.ias = isrData.tas = isrData.palt = 0; 
			isrData.exceptions++;
		}
	}
	*/
	if (lastId == 0x18882100 && ibuf[0] == 0xdd && ibuf[1] == 0x0a && mpSize >= 40) {
		float thresh = 0.0;
		try {
			float ias = floatFromBinary(&ibuf[12]); 
			float tas = floatFromBinary(&ibuf[16]);
			float palt = floatFromBinary(&ibuf[28]);
			isrData.timestamp = millis();
			if (abs(ias - lastSent.ias) > thresh || 
				abs(tas - lastSent.tas) > thresh || 
				abs(palt - lastSent.palt) > thresh) { 
				sendUdpCan("IAS=%f TAS=%f PALT=%f\n", ias / MPS_PER_KNOT, 
				tas / MPS_PER_KNOT, palt);
				lastSent.ias = ias;
				lastSent.tas = tas;
				lastSent.palt = palt;
			}
			isrData.ias = ias; 
			isrData.palt = palt;
			isrData.tas = tas;
		} catch(...) {
			isrData.ias = isrData.tas = isrData.palt = 0; 
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
			Serial.printf("KNOB%d=%f\n", knobSel, knobVal);
			sendUdpCan("KNOB%d=%f\n", knobSel, knobVal);
			if (knobSel > 0 && knobSel < sizeof(g5KnobValues)/sizeof(g5KnobValues[0]))
				g5KnobValues[knobSel] = knobVal;
		} catch(...) {
			isrData.knobSel = isrData.knobVal = 0;
			isrData.exceptions++; 
		}
		//Serial.printf("Knob %d val %f\n", (int)isrData.knobSel, isrData.knobVal);
		isrData.forceSend = true;
		//sendCanData(true);
	}

	static float lastKnobVal = 0;
	static uint64_t lastKnobMillis = 0;
	if ((isrData.knobSel == 1/*hdg*/)  && isrData.knobVal != lastKnobVal) { 
		if (millis() - lastKnobMillis > 500) 
			isrData.mode = 0;
		lastKnobMillis = millis();
		double delta = isrData.knobVal - lastKnobVal;
		bool oneDegree = abs(delta) < 1.5/180*M_PI && abs(delta) > 0.5/180*M_PI;
		bool evenMode = (isrData.mode & 0x1) == 0;
		if (false && isrData.knobSel == 0) { // for debugging, use altimeter adjustment for knob input too 
			oneDegree = abs(delta) < 35 && abs(delta) > 30;
		}
		if (oneDegree && ((delta > 0 && evenMode) || (delta < 0 && !evenMode))) {
			isrData.mode++;
			sendUdpCan("MODE=%d\n", isrData.mode);
		} else if (isrData.mode != 0) { 
			isrData.mode = 0;
			sendUdpCan("MODE=%d\n", isrData.mode);
		}
		lastKnobVal = isrData.knobVal;
	}	

	if (fd == true) { 
		canToText(ibuf, id, len, timestamp, obuf, sizeof(obuf));
		fd.write(obuf, strlen(obuf));
		if (sdCardFlush.tick()) 
			fd.flush();
	}

	if (udpCanOut) { 
		if (udp.beginPacket("255.255.255.255", 7894) == 0) 
			isrData.udpErrs++;
		udp.write((uint8_t *)&id, sizeof(id));
		udp.write((uint8_t *)&len, sizeof(len));
		udp.write((const uint8_t*)ibuf, len);
		if (udp.endPacket() == 0) 
			isrData.udpErrs++;
		isrData.udpSent++;
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
				if (strstr(line.line, "PMRRV")  != NULL) { 
					Serial2.write((uint8_t *)line.line, strlen(line.line));
					//Serial.printf("G5: %s", line.line);
				}
			}
		}
}

void setup() {
	esp_task_wdt_init(5, true);
	esp_task_wdt_add(NULL);

	Serial.begin(921600, SERIAL_8N1);
	Serial.setTimeout(10);
	//while(1)  { 
		Serial.printf("AUTOTRIM\n");
		delay(100);
	//}

	Serial2.begin(9600, SERIAL_8N1, pins.serialRx, pins.serialTx);
	Serial2.setTimeout(10);
	
	pinMode(pins.button, INPUT_PULLUP);
//	pinMode(3,INPUT);

	adcAttachPin(pins.ADC);
	//analogSetCycles(255);
	//pid.setGains(4, 0, 0);
	//pid.finalGain = 1;

#if 0 	
	espNowMux.registerReadCallback("g5", 
        [](const uint8_t *mac, const uint8_t *data, int len){
			processCommand((char *)data, len);
    });
#endif

	can = new CanWrapper();
	can->onReceive(canParse);
	//can->filters.push_back(0x18882100);
	//can->filters.push_back(0x10882200);
	//can->filters.push_back(0x108c2200);
	can->begin();
}
EggTimer report(2000);

void loop() {
	uint64_t now = micros();
	esp_task_wdt_reset();
	ArduinoOTA.handle();
	if (otaInProgress) {
		esp_task_wdt_init(220, true);
		can->reset();
		return;
	}
	can->run(pdMS_TO_TICKS(5), 5);

	
	while (Serial.available()) { 
		static char buf[512];
		int n = Serial.read(buf, sizeof(buf));
		processCommand(buf, n);
	}

	while (Serial2.available()) { 
		static LineBuffer lb;
		static char buf[512];
		int n = Serial2.read(buf, sizeof(buf));
		Serial.printf("Serial read %d bytes\n", n);
		lb.add(buf, n, [](const char *line) {
			sendUdpCan("NMEA=%s", line);
			Serial.printf("NMEA: %s\n", line);
		});

	}
	if (lastLoopTime != -1) 
		loopTimeAvg.add(now - lastLoopTime);
	lastLoopTime = now;

	if (canResetTimer.tick()) {
		can->reset();
	}
	
	string s = autopilot.read();
	if (s.length()) { 
		processCommand(s.c_str(), s.length());
	}

	//trimPosAvg.add(analogRead(33));
	if (pinReportTimer.tick()/* || isrData.forceSend*/) { 
		isrData.forceSend = false;
		//Serial.printf("%08d %d\n", (int)millis(), isrData.mode);
		sendCanData();
	}
	if (1 && serialReportTimer.tick()) {
		//sendDebugData();
		char buf[256]; 
		snprintf(buf, sizeof(buf), "L: %05.3f/%05.3f/%05.3f m:%d app: %d can:%d drop:%d qlen:%d\n", loopTimeAvg.average()/1000.0, loopTimeAvg.min()/1000.0, loopTimeAvg.max()/1000.0, 
			isrData.mode, autopilotPackets, can->pktCount, can->dropped, can->getQueueLen());
		Serial.print(buf);
		fd.print(buf);
	}
	
	if (Serial.available()) {
		static LineBuffer line;
		int l = Serial.readBytes(buf, sizeof(buf));
		for (int i = 0; i < l; i++) {
			int ll = line.add(buf[i]);
			if (ll) {
				udp.beginPacket("255.255.255.255", 7891);
				udp.write((uint8_t *)line.line, ll);
				udp.endPacket();
			}
		}
	}	
	
	int avail;
	if (0 && (avail = udp.parsePacket()) > 0) {
		int r = udp.read(buf, min(avail, (int)sizeof(buf)));
		processCommand((char *)buf, r);
	}

	static int startupPeriod = 20000;
	if (millis() > startupPeriod)
		startupPeriod = 0;
	
	if (sl30Heartbeat.tick()) {
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
		std::string s = sl30.pmrrv("301234E"); // send arbitary NAV software version packet as heartbeat
		Serial2.print(s.c_str());
		//Serial.print(s.c_str());
	}
	
	static EggTimer g5Timer(100);

	if (g5Timer.tick()) { 
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
	}
	if (isrData.mode == 7) 
		canSerial = udpCanOut = 1;
	if (isrData.mode == 1) 
		canSerial = udpCanOut = 0;

	//canSerial = udpCanOut = (isrData.mode == 7);
	delayMicroseconds(1);
}


#ifdef UBUNTU
////////////////////////////////////////////////////////////////////////////////////////
// the rest of the file is simulation support code 


class ESP32sim_autotrim : ESP32sim_Module { 
#ifdef DISCARD
	ifstream gdl90file; 
	ifstream trackSimFile;
	WaypointNav::WaypointSequencer tSim = WaypointNav::WaypointSequencer(trackSimFile);
#endif
	int last_us = 0;	
	bool makeKml = false;
	IntervalTimer hz100 = IntervalTimer(100/*msec*/);
	int loopcount = 0;

	void parseArg(char **&a, char **la) override {
		printf("at parse arg\n");
		if (strcmp(*a, "--kml") == 0) makeKml = true;
		if (strcmp(*a, "--canfile") == 0) CAN.setSimFile(*(++a)); 
		if (strcmp(*a, "--canserial") == 0) canSerial = 1;
	}
	void setup() override {}
			
	void loop() override {
		CAN.run();
		if (!hz100.tick(millis())) 
			return;
	}  
		
	void done() override{
		//printf("gdl90 msgs %d errors %d\n", gdl90.msgCount, gdl90.errCount);
		exit(0);
	} 
} autotrim;
#endif



