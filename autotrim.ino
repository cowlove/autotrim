//#include <SPIFFS.h>
//#include "Update.h"

//  
// ./parse_sweep ./sweep6_unidirectional_battery_power.txt | grep PR | gnuplot -e 'p "-" u 2:3; pause 10;'



#ifndef UBUNTU
#include "Update.h"				
#include "WebServer.h"
#include "DNSServer.h"
#include "ESPmDNS.h"
#include <esp_task_wdt.h>
#include "mySD.h"
#include <HardwareSerial.h>
#include "SPI.h"
#include <CAN.h>
//#include "FS.h"
#include "ArduinoOTA.h"
#include "WiFiUdp.h"
#include "Wire.h"
#include "WiFiMulti.h"

#else
#include "ESP32sim_ubuntu.h"
#endif


#include "RunningLeastSquares.h"
#include "PidControl.h"
#define LED_PIN 2

WiFiMulti wifi;
	
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include "jimlib.h"

#include "G90Parser.h"

WiFiUDP udpG90;
GDL90Parser gdl90;
GDL90Parser::State state;


float g5KnobValues[6];

namespace Display {
	JDisplay jd;
	int y = 0;
	const int c2x = 70;
	JDisplayItem<const char *>  ip(&jd,10,y,"WIFI:", "%s ");
	JDisplayItem<float>  sec(&jd,10,y+=10," SEC:", "%.02f ");
}



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
const struct { 
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

msdFile fd;
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
   for (int repeat = 0; repeat < 2; repeat++) { 
		udp.beginPacket("255.255.255.255", 7891);
		udp.write((uint8_t *)b, strlen(b));
		udp.endPacket();
		for (int n = 100; n < 103; n++) { 
				char ip[32];
				snprintf(ip, sizeof(ip), "192.168.4.%d", n);
				udp.beginPacket(ip, 7891);
				udp.write((const uint8_t *)b, strlen(b));
				udp.endPacket();
		}
	}
}

struct {
	float pitch, roll, knobVal = 0, magHdg, magTrack, ias, tas, palt;
	int knobSel, mode;
	bool forceSend;
	uint64_t timestamp;
	uint8_t *bufIn, *bufOut; 
	int exceptions;
	int udpSent;
	int udpErrs;
} isrData;


class CanWrapper {
	Mutex canMutex;
	struct CanPacket { 
		uint8_t buf[8];
		uint64_t timestamp;
		int len;
		int id;
		int maxLen;
	};

	CircularBoundedQueue<CanPacket> pktQueue = CircularBoundedQueue<CanPacket>(256);
	char ibuf[1024];
	int mpSize = 0, lastId = 0;
	//void onCanPacket(int id, int len, int timestamp, const char *buf) {}
	void (*onCanPacket)(int, int, int, const char *) = NULL;
	
public:
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
	void run(int timeout) { 
		CanPacket *pkt;
		while((pkt = pktQueue.peekTail(timeout)) != NULL) { 
			static char obuf[1024];
			if (pkt->id != lastId || mpSize >= sizeof(ibuf)) {
				pktCount++;
				if (onCanPacket != NULL) 
					onCanPacket(lastId, mpSize, pkt->timestamp, ibuf);
				mpSize = 0;
				lastId = pkt->id;
			}
			for(int n = 0; n < pkt->len && mpSize < sizeof(ibuf); n++) {
				ibuf[mpSize++] = pkt->buf[n];
			}
			pktQueue.freeTail();
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
			digitalWrite(LED_PIN, d % 2);
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
	
	pinMode(LED_PIN, OUTPUT);
	pinMode(pins.button, INPUT_PULLUP);
//	pinMode(3,INPUT);

	Display::jd.begin();
	Display::jd.clear();
	Display::jd.forceUpdate();
	
	wifi.addAP("Ping-582B", "");
	wifi.addAP("Flora_2GEXT", "maynards");
	wifi.addAP("Team America", "51a52b5354");
	wifi.addAP("ChloeNet", "niftyprairie7");
	uint64_t startms = millis();
	Serial.printf("Waiting for wifi...\n");
	while (digitalRead(pins.button) != 0 && WiFi.status() != WL_CONNECTED) {
		wifi.run();
		delay(10);
		if (millis() - startms > 21000)
						break;
	}

	digitalWrite(LED_PIN, 1);

	if (WiFi.status() == WL_CONNECTED) {
		udp.begin(udpPortIn);
		ArduinoOTA.begin();
		ArduinoOTA.onStart([]() {
			esp_task_wdt_delete(NULL);
			can.reset();
			Serial.println("Start");
			otaInProgress = true;
		});
		openLogFile("CAN%03d.TXT");
	}
	
	udpG90.begin(4000);
	
	adcAttachPin(pins.ADC);
	//analogSetCycles(255);
	pid.setGains(4, 0, 0);
	pid.finalGain = 1;
	
	can.onReceive(canParse);
	can.begin();
}

struct LatLon {
	LatLon(double la, double lo) : lat(la), lon(lo) {}
	LatLon() {} 
	double lat;
	double lon; 
	LatLon toRadians() { 
		return LatLon(lat * M_PI/180, lon * M_PI/180); 
	} 
	LatLon toDegrees() { 
		return LatLon(lat * 180/M_PI, lon *180/M_PI); 
	} 
};

double bearing(LatLon a, LatLon b) { // returns relative brg in degrees
	a = a.toRadians();
	b = b.toRadians();
	double dL = b.lon - a.lon;
	double x = cos(b.lat) * sin(dL);
	double y = cos(a.lat) * sin(b.lat) - sin(a.lat) * 	cos(b.lat)* cos(dL);	
	double brg = atan2(x, y) * 180/M_PI;
	if (brg < 0) brg += 360;
	return brg;
}
	
double distance(LatLon p1, LatLon p2) { // returns distance in meters 
	p1 = p1.toRadians();
	p2 = p2.toRadians();

	const double R = 6371e3; // metres
	double dlat = p2.lat - p1.lat;
	double dlon = p2.lon - p1.lon;
	
	double a = sin(dlat/2) * sin(dlat/2) +
          cos(p1.lat) * cos(p2.lat) *
          sin(dlon/2) * sin(dlon/2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	return c * R;
}

LatLon locationBearingDistance(LatLon p, double brng, double d) { 
	p = p.toRadians();
	brng *= M_PI/180;

	LatLon p2;
	const double R = 6371e3; // metres
	p2.lat = asin(sin(p.lat) * cos(d/R) + cos(p.lat) * sin(d/R) * cos(brng));
		
	p2.lon = p.lon + 
				atan2(sin(brng) * sin(d/R) * cos(p.lat),
					cos(d/R) - sin(p.lat) * sin(p2.lat));

	return p2.toDegrees();
}

double glideslope(float distance, float alt1, float alt2) { 
		return atan2(alt1 - alt2, distance) * 180/M_PI; 
}

bool ESP32sim_makeKml = false;

class IlsSimulator {
	const LatLon tdzLocation; // lat/long of touchdown point 
	const float tdze;  // TDZE elevation of runway 
	const float faCrs; // final approach course
	const float gs; // glideslope in degrees
	float currentAlt;
	LatLon currentLoc;
public:
	IlsSimulator(LatLon l, float elev, float crs, float g) : tdzLocation(l), tdze(elev), faCrs(crs), gs(g) {}
	void setCurrentLocation(LatLon now, double alt) { 
		currentLoc = now;
		currentAlt = alt;
	}
	double glideSlopeError() {
		return glideslope(distance(currentLoc, tdzLocation), currentAlt, tdze) - gs;
	}
	double courseErr() { 
		return faCrs - bearing(currentLoc, tdzLocation);
	}
	double cdiPercent() { 
		return max(-1.0, min(1.0, courseErr() / 2.5));
	}
	double gsPercent() { 
		return max(-1.0, min(1.0, glideSlopeError() / 0.7));
	}
} *ils = NULL;

struct  Approach{
	const char *name;
	double lat, lon, fac, tdze, gs;
};

Approach approaches[] = {
	{"KBFI 14R", 47.53789999126084, -122.30912681366567, 135.0, 18, 3.00}, 
	{"KBFI 32L", 47.52145193515430, -122.29521847198963, 315.0, 18, 3.00}, 
	{"S43 15",   47.90660383843286, -122.10299154204903, 150.0, 22, 4.00},
	{"S43 33",   47.90250761678649, -122.10136258707182, 328.0, 22, 4.00},
	{"2S1 17",   47.46106431485166, -122.47652028295599, 170.0, 300, 4.00}, 
	{"2S1 35",   47.45619317486062, -122.47745151953475, 350.0, 300, 4.00}, 
	{"WN14 02",  47.46276277164776, -122.56987914788057, 040.0, 300, 4.50},	
};
	
	
float angularDiff(float a, float b) { 
	float d = a - b;
	if (d > +180) d -= 360;
	if (d < -180) d += 360;
	return d;
}

float magToTrue(float ang) { 
	return ang + 15.5;		;
}

float trueToMag(float ang) { 
	return ang - 15.5;		;
}

Approach *findBestApproach(LatLon p) { 
	Approach *best = NULL;
	for(int n = 0; n < sizeof(approaches)/sizeof(approaches[0]); n++) {
		Approach &a = approaches[n];
		LatLon tdz = LatLon(a.lat, a.lon); 
		float brg = bearing(p, LatLon(a.lat, a.lon));
		float dist = distance(p, tdz);
		if (abs(angularDiff(brg, a.fac)) >= 60 || dist > 20000)
			continue;
		if (best == NULL || 
			distance(p, tdz) < distance(p, LatLon(best->lat, best->lon)))
			best = &approaches[n]; 
	}
#ifdef UBUNTU
	if (best != NULL) { 
		printf("Chose '%s', bearing %.1f, dist %.1f\n", best->name, 
			bearing(p, LatLon(best->lat, best->lon)), distance(p, LatLon(best->lat, best->lon)));
	} else { 
		printf("No best approach?\n");
	}
#endif
	return best;
}



GDL90Parser::State currentState;

void loop() {
	uint64_t now = micros();
	esp_task_wdt_reset();
	ArduinoOTA.handle();
	if (otaInProgress) {
		esp_task_wdt_init(220, true);
		can.reset();
		return;
	}
	if (displayTimer.tick()) { 
		Display::sec = millis() / 1000.0;
		Display::jd.update(false, false);
	}	
	if (first || digitalRead(pins.button) == 0) { 
		first = false;
		pp[0].pulse(1, 1000);
		pp[1].pulse(1, 2000);
	} 
	can.run(pdMS_TO_TICKS(5));
	
	if (lastLoopTime != -1) 
		loopTimeAvg.add(now - lastLoopTime);
	lastLoopTime = now;
	
	if (canResetTimer.tick()) {
		can.reset();
	}
	
	for (int n =0; n < sizeof(pp)/sizeof(pp[0]); n++) { 
		pp[n].run();
	}

	trimPosAvg.add(analogRead(33));
	if (pinReportTimer.tick()/* || isrData.forceSend*/) { 
		isrData.forceSend = false;
		//Serial.printf("p%d %d %.2f %.2f m %d pins %d %d\n", lastCmdPin,  (int)(micros() - lastCmdTime), 
		//	trimPosAvg.average(), startTrimPos, lastCmdMs, digitalRead(pp[0].pin), digitalRead(pp[1].pin));
		sendCanData();
			}
	if (0 && serialReportTimer.tick()) {
		//sendDebugData(); 
		fd.printf("L: %05.3f/%05.3f/%05.3f err:%d can:%d drop:%d\n", loopTimeAvg.average()/1000.0, loopTimeAvg.min()/1000.0, loopTimeAvg.max()/1000.0, 
			isrData.udpErrs, can.pktCount, can.dropped);
		Serial.printf("%06.2f L: %05.3f/%05.3f/%05.3f err:%d can:%d drop:%d\n", millis() / 1000.0, loopTimeAvg.average()/1000.0, loopTimeAvg.min()/1000.0, loopTimeAvg.max()/1000.0, 
			isrData.udpErrs, can.pktCount, can.dropped);
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
		sl30.pmrrv("301234E"); // send arbitary NAV software version packet as heartbeat
	}
	
	static double hd = 0, vd = 0; // cdi deflections 

	avail = udpG90.parsePacket();
	while(avail > 0) {
		unsigned char buf[1024]; 
		int recsize = udpG90.read(buf, min(avail,(int)sizeof(buf)));
		if (recsize <= 0)
				break;
		avail -= recsize;
		for (int i = 0; i < recsize; i++) {  
			gdl90.add(buf[i]);
			GDL90Parser::State s = gdl90.getState();
			if (s.valid && s.updated) {
				currentState = s;
				if (ESP32sim_makeKml) {
					printf("%f,%f\n", s.lat, s.lon);
				}

				int vvel = (signed short)(s.vvel * 64);
				static const LatLon target(47.90558443637955, -122.10252512643517);
				LatLon here(s.lat, s.lon);
				float brg = bearing(here, target);
				float range = distance(here, target);
				Serial.printf("%08d pos %+11.6f,%+11.6f track %6.1f, brg %6.1f range %5.0f palt %5d, galt %5d, vvel %+4d, hvel %3d CDI:%+.1f GS:%+.1f\n", 
						(int)millis(), s.lat, s.lon, s.track, brg, range, (s.palt * 25) - 1000, (int) (s.alt * 3.321), 
					vvel, s.hvel, hd, vd);	
						
			}
		}
		char obuf[1024];
		sprintf(obuf, "%s", "GDL90:");
		int maxB = min((int)(sizeof(obuf) - 8)/2, recsize);
		for (int i = 0; i < maxB; i++) { 
			sprintf(obuf + 6 + i * 2, "%02x", buf[i]);
		}
		sprintf(obuf + 6 + maxB * 2, "\n");
		fd.write(obuf, strlen(obuf));
		Serial.printf("%08d ", millis());	
		Serial.println(obuf);	
	}
	
	static EggTimer g5Timer(100);
#ifdef UBUNTU
	if (millis() >276000) {
		isrData.mode = 5;
		g5KnobValues[2] = 200;
		g5KnobValues[4] = 10 * M_PI/180;
	}
#endif

	if (g5Timer.tick()) { 
		if (debugMoveNeedles || isrData.mode == 6 || startupPeriod > 0) { 
			hd += .1;
			vd += .15;
			if (hd > 2) hd = -2;
			if (vd > 2) vd = -2;
			if (millis() - isrData.timestamp > 1000) 
				hd = vd = -2;
			sl30.setCDI(hd, vd);
		}
		if (isrData.mode == 5) {
			LatLon now(currentState.lat, currentState.lon); 
			if (ils == NULL) {
				float vlocTrk = g5KnobValues[4] * 180/M_PI;
				float altBug = g5KnobValues[2];
				if (vlocTrk != 0) { 
					const float gs = 3.0;
					float tdze = altBug - 200 / 3.281;
					LatLon facIntercept = locationBearingDistance(now, currentState.track, 1000);
					float facDist = (currentState.alt - tdze) / tan(gs * M_PI/180);
					LatLon tdz = locationBearingDistance(facIntercept, magToTrue(vlocTrk), facDist);
					ils = new IlsSimulator(tdz, tdze, magToTrue(vlocTrk), gs);
					printf("Set ILS\n");
				} else { 
					Approach *a = findBestApproach(now);
					if (a != NULL)
						ils = new IlsSimulator(LatLon(a->lat, a->lon), a->tdze / 3.281, magToTrue(a->fac), a->gs);
				}
			}
			if (ils != NULL) { 
				ils->setCurrentLocation(now, currentState.alt);
				hd = ils->cdiPercent() * 2.0;
				vd = ils->gsPercent() * 2.0;
				sl30.setCDI(hd, vd);
			}
		} else if (ils != NULL) {
			delete ils;
			ils = NULL;
		}
	}


	canSerial = udpCanOut = (isrData.mode == 6);
}


#ifdef UBUNTU

ifstream gdl90file; 

void ESP32sim_setDebug(char const *) {}
void ESP32sim_parseArg(char **&a, char **endA) {
	if (strcmp(*a, "--kml") == 0) ESP32sim_makeKml = true;
	if (strcmp(*a, "--gdltest") == 0) {
		ifstream f = ifstream(*(++a), ios_base::in | ios_base::binary);
		while(f) { 
			gdl90.add(f.get());
		}
		ESP32sim_done();
		exit(0);
	}
	if (strcmp(*a, "--gdl") == 0) {
		gdl90file = ifstream(*(++a), ios_base::in | ios_base::binary);
	}
	if (strcmp(*a, "--gdlseek") == 0) {
		int pos = 0;
		sscanf(*(++a), "%d", &pos);
		gdl90file.seekg(pos);
	}
}

void ESP32sim_setup() {
	if (ESP32sim_makeKml) 
		printf("<Placemark><name>Untitled Path</name><LineString><tessellate>1</tessellate><altitudeMode>relativeToGround</altitudeMode><coordinates>\n");
}
int last_us = 0;	

class IntervalTimer {
public: 
	double last, interval;
	IntervalTimer(double i) : interval(i) {}
	bool tick(double now) {
		bool rval = (floor(now / interval) != floor(last / interval));
		last = now;
		return rval;
	}
};
				
IntervalTimer hz100(100/*msec*/);

void ESP32sim_loop() {
	if (hz100.tick(millis()) && gdl90file) {
		std::vector<unsigned char> data(300);
		gdl90file.read((char *)data.data(), data.size());	
		int n = gdl90file.gcount();
		if (gdl90file && n > 0) { 
			ESP32sim_udpInput(4000, data);
		}
	}		
}  
	
void ESP32sim_done() {
	if (ESP32sim_makeKml) 
		printf("</coordinates></LineString></Placemark>");
	printf("gdl90 msgs %d errors %d\n", gdl90.msgCount, gdl90.errCount);
} 
void ESP32sim_JDisplay_forceUpdate	() {}
#endif
