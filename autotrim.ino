#ifdef UBUNTU
#include "ESP32sim_ubuntu.h"
#else
#include <CAN.h>
#endif 
	
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <istream>

#include "RollingLeastSquares.h"
#include "PidControl.h"
#include "jimlib.h"
#include "GDL90Parser.h"
#include "WaypointNav.h"

using namespace WaypointNav;
//#define LED_PIN 2

WiFiMulti wifi;
WiFiUDP udpG90;
GDL90Parser gdl90;
GDL90Parser::State state;

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

RollingAverage<long int,1000> loopTimeAvg;
uint64_t lastLoopTime = -1;
EggTimer serialReportTimer(1000), displayTimer(1000);
EggTimer pinReportTimer(200), canResetTimer(5000), sl30Heartbeat(1000), sdCardFlush(2000);

uint64_t nextCmdTime = 0;
static bool debugMoveNeedles = false;

static bool first = true;

// send udp packet multiple times on broadcast address and all addresses in normal EchoUAT wifi range 
void superSend(const char *b) { 
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
} isrData;

class ScopedUnlockMutex {
	Mutex &mut;
public:
	ScopedUnlockMutex(Mutex &m) : mut(m) { mut.unlock(); }
	~ScopedUnlockMutex() { mut.lock(); }
}; 

class CanWrapper {
	Mutex canMutex;
	struct CanPacket { 
		uint8_t buf[8];
		uint64_t timestamp;
		int len;
		int id;
		int maxLen;
	};

	CircularBoundedQueue<CanPacket> pktQueue = CircularBoundedQueue<CanPacket>(2048);
	char ibuf[1024];
	int mpSize = 0, lastId = 0;
	//void onCanPacket(int id, int len, int timestamp, const char *buf) {}
	void (*onCanPacket)(int, int, int, const char *) = NULL;
	
public:
	int getQueueLen() { return pktQueue.getCount(); } 
	int isrCount = 0, pktCount = 0, dropped = 0;
	void onReceive(	void (*f)(int, int, int, const char *)){ onCanPacket = f; }
	
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
			if (cp.id != lastId || mpSize >= sizeof(ibuf)) {
				pktCount++;
				if (onCanPacket != NULL) {
					onCanPacket(lastId, mpSize, cp.timestamp, ibuf);
				}
				mpSize = 0;
				lastId = cp.id;
			}
			for(int n = 0; n < cp.len && mpSize < sizeof(ibuf); n++) {
				ibuf[mpSize++] = cp.buf[n];
			}
		}
	}
	void isr(int packetSize) {
		ScopedMutex lock(canMutex);
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
} can;

CanWrapper *CanWrapper::instancePtr = NULL;


uint8_t canDebugBuf[1024];
PidControl pid(2);
static RollingAverage<int,1000> trimPosAvg;
int canSerial = 0;

void sendDebugData() { 
	char sbuf[160];				
	snprintf(sbuf, sizeof(sbuf), "%d %f %s DEBUG\n", 
		(int)can.pktCount, (float)trimPosAvg.average(), WiFi.localIP().toString().c_str());
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
	char sbuf[160];				
	int age = millis() - isrData.timestamp;
	snprintf(sbuf, sizeof(sbuf), "%+06.3f %+06.3f %+06.3f %+06.3f %+06.3f %+06.3f %+06.3f %d %+06.3f %d %d CAN\n", 
		isrData.pitch, isrData.roll, isrData.magHdg, isrData.magTrack, isrData.ias, isrData.tas, 
		isrData.palt, isrData.knobSel, isrData.knobVal, age, isrData.mode);
	superSend(sbuf);
	fd.write(sbuf, strlen(sbuf));
	//Serial.printf("%d\t%d\t%s", isrData.udpSent, isrData.count, sbuf);
}


float floatFromBinary(const char *b) { 
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

void canToText(const char *ibuf, int lastId, int mpSize, uint64_t ts, char *obuf, int obufsz) { 
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

void canParse(int id, int len, int timestamp, const char *ibuf) { 
	int lastId = id;
	int mpSize = len;
	static char obuf[1024];
	
	if (canSerial) {
		canToText(ibuf, lastId, mpSize, millis(), obuf, sizeof(obuf));
		Serial.print(obuf);
	}
	
	if ((lastId == 0x18882100 || lastId == 0x188c2100) && ibuf[0] == 0xdd && ibuf[1] == 0x00 && mpSize == 60 && ibuf[15] & 0x40) {
		try {
			isrData.magHdg = floatFromBinary(&ibuf[16]);
			sendUdpCan("HDG=%.5f\n", isrData.magHdg * 180/M_PI);
		} catch(...) {
			isrData.magHdg = 0;
		}
	} 
	if ((lastId == 0x18882100 || lastId == 0x188c2100) && ibuf[0] == 0xdd && ibuf[1] == 0x00 && mpSize == 60 && ibuf[15] & 0x20) {
		try {
			isrData.magTrack = floatFromBinary(&ibuf[16]);
			sendUdpCan("TRK=%.5f\n", isrData.magTrack * 180/M_PI);
		} catch(...) {
			isrData.magTrack = 0;
		}
	} 	
	if (lastId == 0x18882100 && ibuf[0] == 0xdd && ibuf[1] == 0x00 && mpSize == 60) {
		try {
			isrData.pitch = floatFromBinary(&ibuf[20]);
			isrData.roll = floatFromBinary(&ibuf[24]);
			isrData.forceSend = true;
			isrData.timestamp = millis();
			sendUdpCan("P=%f R=%f\n", isrData.pitch* 180/M_PI, isrData.roll* 180/M_PI);
		} catch(...) {
			isrData.pitch = isrData.roll = 0; 
			isrData.exceptions++;
		}
		//sendCanData(false);
	}
	if (lastId == 0x18882100 && ibuf[0] == 0xdc && ibuf[1] == 0x02 && mpSize >= 16) {
		try {
			isrData.slip = floatFromBinary(&ibuf[12]); 
			sendUdpCan("SL=%f", isrData.slip * 180/M_PI);
		} catch(...) {
			isrData.slip = 0; 
			isrData.exceptions++;
		}
	}
	if (lastId == 0x18882100 && ibuf[0] == 0xdd && ibuf[1] == 0x0a && mpSize >= 44) {
		try {
			isrData.ias = floatFromBinary(&ibuf[12]); 
			isrData.tas = floatFromBinary(&ibuf[16]);
			isrData.palt = floatFromBinary(&ibuf[24]);
			isrData.timestamp = millis();
			isrData.forceSend = true;
			sendUdpCan("IAS=%f TAS=%f PALT=%f", isrData.ias / 0.5144, isrData.tas / 0.5144, isrData.palt / 3.2808);
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
			sendUdpCan("KSEL=%d KVAL=%f\n", knobSel, knobVal);
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
	if ((isrData.knobSel == 1/*hdg*/ || isrData.knobSel == 0/*altimeter*/)  && isrData.knobVal != lastKnobVal) { 
		if (millis() - lastKnobMillis > 500) 
			isrData.mode = 0;
		lastKnobMillis = millis();
		double delta = isrData.knobVal - lastKnobVal;
		bool oneDegree = abs(delta) < 1.5/180*M_PI && abs(delta) > 0.5/180*M_PI;
		bool evenMode = (isrData.mode & 0x1) == 0;
		if (isrData.knobSel == 0) { // for debugging, use altimeter adjustment for knob input too 
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
		canToText((char *)ibuf, id, len, timestamp, obuf, sizeof(obuf));
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

bool wifiTryConnect(const char *ap, const char *pw, int seconds = 15) { 
	if (WiFi.status() != WL_CONNECTED) {
		WiFi.begin(ap, pw);
		for(int d = 0; d < seconds * 10 && WiFi.status() != WL_CONNECTED; d++) {
			delay(100);
		}
	}
	return WiFi.status() == WL_CONNECTED;
}
	
static PinPulse pp[] = { PinPulse(pins.relay1), PinPulse(pins.relay2) };


void setup() {
	esp_task_wdt_init(20, true);
	esp_task_wdt_add(NULL);

	Serial.begin(921600, SERIAL_8N1);
	Serial.setTimeout(10);
	Serial.printf("AUTOTRIM\n");

	Serial2.begin(9600, SERIAL_8N1, pins.serialRx, pins.serialTx);
	Serial2.setTimeout(10);
	
	pinMode(pins.button, INPUT_PULLUP);
//	pinMode(3,INPUT);

#ifdef XDISPLAY
	Display::jd.begin();
	Display::jd.clear();
	Display::jd.forceUpdate();
#endif

	wifi.addAP("Ping-582B", "");
	wifi.addAP("Flora_2GEXT", "maynards");
	wifi.addAP("Team America", "51a52b5354");
	wifi.addAP("ChloeNet", "niftyprairie7");
	uint64_t startms = millis();
	Serial.printf("Waiting for wifi...\n");
	while (digitalRead(pins.button) != 0 && WiFi.status() != WL_CONNECTED) {
		wifi.run();
		delay(10);
	}

	if (WiFi.status() == WL_CONNECTED) {
		Serial.printf("connected\n");
		udp.begin(udpPortIn);
		udpG90.begin(4000);
		ArduinoOTA.begin();
		ArduinoOTA.onStart([]() {
			esp_task_wdt_delete(NULL);
			can.reset();
			Serial.println("Start");
			otaInProgress = true;
		});
		openLogFile("CAN%03d.TXT");
	} 
	
	adcAttachPin(pins.ADC);
	//analogSetCycles(255);
	pid.setGains(4, 0, 0);
	pid.finalGain = 1;
	
	can.onReceive(canParse);
	can.begin();
}
GDL90Parser::State currentState;
static EggTimer report(2000);


void fakeApproach(LatLon now, float vlocTrk, float altBug) { 
	const float gs = 3.0;
	float tdze = altBug - 200 / 3.281;
	LatLon facIntercept = locationBearingDistance(now, currentState.track, 1600);
	float facDist = 1600 + (currentState.alt - tdze) / tan(gs * M_PI/180);
	LatLon tdz = locationBearingDistance(facIntercept, magToTrue(vlocTrk), facDist);
	ils = new IlsSimulator(tdz, tdze, magToTrue(vlocTrk), gs);
}

void loop() {
	uint64_t now = micros();
	esp_task_wdt_reset();
	ArduinoOTA.handle();
	if (otaInProgress) {
		esp_task_wdt_init(220, true);
		can.reset();
		return;
	}
#ifdef XDISPLAY
	if (displayTimer.tick()) { 
		Display::sec = millis() / 1000.0;
		Display::jd.update(false, false);
	}	
#endif
	if (first || digitalRead(pins.button) == 0) { 
		first = false;
		pp[0].pulse(1, 1000);
		pp[1].pulse(1, 2000);
	} 
	can.run(pdMS_TO_TICKS(5), 20);
	
	if (lastLoopTime != -1) 
		loopTimeAvg.add(now - lastLoopTime);
	lastLoopTime = now;
	
	if (canResetTimer.tick()) {
		can.reset();
	}
	
	if (report.tick()) { 
		WiFiUDP &udp = udpG90;
		udp.beginPacket("255.255.255.255", 9000);
		char b[128];
		snprintf(b, sizeof(b), "%d %s    " __BASE_FILE__ "   " __DATE__ "   " __TIME__ "   0x%08x\n", 
				(int)(millis() / 1000), WiFi.localIP().toString().c_str(), /*(int)ESP.getEfuseMac()*/0);
		udp.write((const uint8_t *)b, strlen(b));
		udp.endPacket();
	}
                        

	
	for (int n =0; n < sizeof(pp)/sizeof(pp[0]); n++) { 
		pp[n].run();
	}

	trimPosAvg.add(analogRead(33));
	if (pinReportTimer.tick()/* || isrData.forceSend*/) { 
		isrData.forceSend = false;
		//Serial.printf("%08d %d\n", (int)millis(), isrData.mode);
		sendCanData();
	}
	if (1 && serialReportTimer.tick()) {
		//sendDebugData();
		char buf[256]; 
		snprintf(buf, sizeof(buf), "L: %05.3f/%05.3f/%05.3f m:%d err:%d can:%d drop:%d qlen:%d\n", loopTimeAvg.average()/1000.0, loopTimeAvg.min()/1000.0, loopTimeAvg.max()/1000.0, 
			isrData.mode, isrData.udpErrs, can.pktCount, can.dropped, can.getQueueLen());
		Serial.print(buf);
		fd.print(buf);
	}

	if (millis() > nextCmdTime && setPoint != -1) { 
		float c = pid.add((setPoint - trimPosAvg.average()), trimPosAvg.average(), millis() / 1000.0);
		if (abs(c) > 200) {
			c = 200 * c/abs(c);
		}
		if (abs(c) > 10) {
			if (c < 0) {
				pp[0].pulse(1, abs(c));
			} else { 
				pp[1].pulse(1, abs(c));
			}
		}
		nextCmdTime = millis() + (int)abs(c) + 50;  // schedule time for PID to run. 
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
	if ((avail = udp.parsePacket()) > 0) {
		static LineBuffer line;
		int r = udp.read(buf, min(avail, (int)sizeof(buf)));
		for (int i = 0; i < r; i++) {
			int ll = line.add(buf[i]);
			if (ll) {
				cmdCount++;
				//Serial.printf("LINE: %s", line.line);
				int ms, val, pin, seq;
				float f;
				static int lastSeq = 0;
				if (sscanf(line.line, "pin %d %d %d %d", &pin, &val, &ms, &seq) == 4) { 
					cmdCount += 100;
					if (pin >= 0 && pin < sizeof(pp)/sizeof(pp[0]) && seq != lastSeq && ms > 5 && ms <= 500) {
						pp[pin].pulse(val, ms);
					}
					lastSeq = seq;
					lastCmdTime = micros();
					lastCmdMs = ms;
					lastCmdPin = pin;
					startTrimPos = trimPosAvg.average();
				}
				if (sscanf(line.line, "trim %f %d", &f, &seq) == 2 ) {
					setPoint = f;
					lastSeq = seq;
				}
				if (sscanf(line.line, "gain %f %d", &f, &seq) == 2 ) {
					pid.gain.p = f;
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
	

	avail = udpG90.parsePacket();
	if (avail > 0) {
		//Serial.printf("UDP: %d bytes avail\n", avail);
				static int count = 0;
		unsigned char buf[1024]; 
		int recsize = udpG90.read(buf, min(avail,(int)sizeof(buf)));
		for (int i = 0; i < recsize; i++) {  
			gdl90.add(buf[i]);
			GDL90Parser::State s = gdl90.getState();
			if (s.valid && s.updated) {
				currentState = s;
				int vvel = (signed short)(s.vvel * 64);
				float brg = 0, range = 0, hat = 0;
				if (ils != NULL) { 
					LatLon target = (ils != NULL) ? ils->tdzLocation : LatLon(47.90558443637955, -122.10252512643517);
					LatLon here(s.lat, s.lon);
					brg = bearing(here, target);
					range = distance(here, target);
					hat = s.alt - ils->tdze;
				}
				count++;
				Serial.printf("%08.2f pos %+11.6f %+11.6f track %6.1f, ils-brg %6.1f ils-range %5.0f ils-hat %5.0f palt %5d:, galt %5d, vvel %+4d, hvel %3d CDI:%+.1f GS:%+.1f\n", 
						millis()/1000.0, s.lat, s.lon, s.track, brg, range, hat, (s.palt * 25) - 1000, (int) (s.alt * FEET_PER_METER), 
					vvel, s.hvel, hd, vd);		
			}
		}
		if (fd == true) { 
			char obuf[1024];
			sprintf(obuf, "%s", "GDL90:");
			int maxB = min((int)(sizeof(obuf) - 8)/2, recsize);
			for (int i = 0; i < maxB; i++) { 
				sprintf(obuf + 6 + i * 2, "%02x", buf[i]);
			}
			sprintf(obuf + 6 + maxB * 2, "\n");
			fd.write(obuf, strlen(obuf));
			if (sdCardFlush.tick()) 
				fd.flush();
			//Serial.printf("%08d ", millis());	
			//Serial.print(obuf);	

		}
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
		if (isrData.mode == 5) {
			LatLon now(currentState.lat, currentState.lon); 
			if (ils == NULL) {
				float vlocTrk = g5KnobValues[4] * 180/M_PI;
				float altBug = g5KnobValues[2];
				if (vlocTrk != 0) { 
					fakeApproach(now, vlocTrk, altBug);
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
				ils->setCurrentLocation(now, currentState.alt);
				hd = ils->cdiPercent() * 2.0;
				vd = ils->gsPercent() * 2.0;
				std::string s = sl30.setCDI(hd, vd);
				Serial2.print(s.c_str());
				//Serial.print(s.c_str());
			}
		} else if (ils != NULL) {
			delete ils;
			ils = NULL;
		}
	}

	canSerial = udpCanOut = (isrData.mode == 7);
	delayMicroseconds(1);
}


#ifdef UBUNTU
////////////////////////////////////////////////////////////////////////////////////////
// the rest of the file is simulation support code 


class ESP32sim_autotrim : ESP32sim_Module { 
	ifstream gdl90file; 
	ifstream trackSimFile;
	WaypointNav::WaypointSequencer tSim = WaypointNav::WaypointSequencer(trackSimFile);
	int last_us = 0;	
	bool makeKml = false;
	IntervalTimer hz100 = IntervalTimer(100/*msec*/);
	int loopcount = 0;

	void parseArg(char **&a, char **la) override {
		printf("at parse arg\n");
		if (strcmp(*a, "--kml") == 0) makeKml = true;
		if (strcmp(*a, "--gdltest") == 0) {
			ifstream f = ifstream(*(++a), ios_base::in | ios_base::binary);
			while(f) { 
				gdl90.add(f.get());
			}
			done();
			exit(0);
		}
		if (strcmp(*a, "--gdl") == 0) 
			gdl90file = ifstream(*(++a), ios_base::in | ios_base::binary);
		if (strcmp(*a, "--gdlseek") == 0) {
			int pos = 0;
			sscanf(*(++a), "%d", &pos);
			gdl90file.seekg(pos);
		}
		if (strcmp(*a, "--tracksim") == 0) 
			trackSimFile = ifstream(*(++a), ios_base::in | ios_base::binary);
	}
	void setup() override {}
			
	void loop() override {
		if (!hz100.tick(millis())) 
			return;
		if (gdl90file) {
			std::vector<unsigned char> data(300);
			gdl90file.read((char *)data.data(), data.size());	
			int n = gdl90file.gcount();
			if (gdl90file && n > 0) { 
				ESP32sim_udpInput(4000, data);
			}
		}	
		if (trackSimFile || trackSimFile.eof()) {
			tSim.run(hz100.interval / 1000.0);
			if (tSim.autoPilotOn) { 
				tSim.wptTracker.setCDI(hd, vd, tSim.decisionHeight);
			}
			if (tSim.inputs.find("MODE") != tSim.inputs.end());
				isrData.mode = tSim.inputs["MODE"];
			g5KnobValues[1] = tSim.inputs["HDGBUG"] * M_PI/180;
			g5KnobValues[2] = tSim.inputs["ALTBUG"];
			g5KnobValues[4] = tSim.inputs["VLOCBUG"] * M_PI/180;
			LatLonAlt p = tSim.wptTracker.curPos;
				GDL90Parser::State s;
			if (p.valid) { 
				s.lat = p.loc.lat;
				s.lon = p.loc.lon;
				s.alt = p.alt;
				s.track = tSim.wptTracker.steerHdg;
				s.vvel = tSim.wptTracker.vvel;
				s.hvel = tSim.wptTracker.speed;
				s.palt = (p.alt + 1000) / 25;
				loopcount++;

				WiFiUDP::InputData buf;
				buf.resize(128);
				buf.resize(gdl90.packMsg11(buf.data(), buf.size(), s));
				ESP32sim_udpInput(4000, buf);
				buf.resize(128);
				buf.resize(gdl90.packMsg10(buf.data(), buf.size(), s));
				ESP32sim_udpInput(4000, buf);

			}
			/*printf("%3d %d %d %+4.1f %+4.1f %+11.6f, %+11.6f a:%4.0f t:%4.1f r:%4.0f\n", tSim.sim.wayPointCount, tSim.sim.waypointPassed, isrData.mode,
				hd, vd, s.lat, s.lon, (float)s.alt, s.track, 
				distance(p.loc, tSim.sim.activeWaypoint.loc));
			*/
		}	
	}  
		
	void done() override{
		printf("gdl90 msgs %d errors %d\n", gdl90.msgCount, gdl90.errCount);
		exit(0);
	} 
} autotrim;
#endif



