#include <HardwareSerial.h>
#include "SPI.h"
#include <CAN.h>
//#include "Update.h"

//  
// ./parse_sweep ./sweep6_unidirectional_battery_power.txt | grep PR | gnuplot -e 'p "-" u 2:3; pause 10;'


#ifdef ESP32
#include "Update.h"
#include "WebServer.h"
#include "DNSServer.h"
#include "ESPmDNS.h"
#include <esp_task_wdt.h>

#else
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#endif

#include "mySD.h"
#include "FS.h"
#include "ArduinoOTA.h"
#include "WiFiUdp.h"
#include "Wire.h"

#include "RunningLeastSquares.h"
#include "PidControl.h"
#define LED_PIN 2

#ifdef ESP32
#include "WiFiMulti.h"
WiFiMulti wifi;

#define SCREEN
#ifdef SCREEN
#include <U8g2lib.h>
#include <U8x8lib.h>
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
#endif

#endif

#include <queue>

struct CanPacket { 
	uint8_t *buf;
	uint64_t timestamp;
	int len;
	int id;
};

std::queue<struct CanPacket> qFull;
std::queue<uint8_t *> qEmpty; 

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
	int button = 34;
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

// From data format described in web search "SL30 Installation Manual PDF" 
class SL30 {
public:
        std::string twoenc(unsigned char x) {
                char r[3];
                r[0] = (((x & 0xf0) >> 4) + 0x30);
                r[1] = (x & 0xf) + 0x30;
                r[2] = 0;
                return std::string(r);
        }
        int chksum(const std::string& r) {
                int sum = 0;
                const char* s = r.c_str();
                while (*s)
                        sum += *s++;
                return sum & 0xff;
        }
        void open() {
        }
        void pmrrv(const std::string& r) {
                std::string s = std::string("$PMRRV") + r + twoenc(chksum(r)) + "\r\n";
                Serial2.write(s.c_str());
				//Serial.printf("G5: %s", s.c_str());
                //Serial.write(s.c_str());
        }
        void setCDI(double hd, double vd) {
                int flags = 0b11111010;
                hd *= 127 / 3;
                vd *= 127 / 3;
                pmrrv(std::string("21") + twoenc(hd) + twoenc(vd) + twoenc(flags));
        }
};

SL30 sl30;
msdFile fd;

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
	int count;
	float pitch, roll, knobVal = 0, magHdg, magTrack, ias, tas, palt;
	int knobSel, mode;
	bool forceSend;
	int lastId, lastSize, exceptions;
	uint64_t timestamp;
	uint8_t *bufIn, *bufOut; 
	int debugDropped;
	int udpSent;
	int udpErrs;
} isrData;

uint8_t canDebugBuf[1024];
PidControl pid(2);
static RollingAverage<int,1000> trimPosAvg;
int canSerial = 0;

void sendDebugData() { 
	char sbuf[160];				
	snprintf(sbuf, sizeof(sbuf), "%d %f %s DEBUG\n", 
		(int)isrData.count, (float)trimPosAvg.average(), WiFi.localIP().toString().c_str());
	//superSend(sbuf);
	//Serial.printf(sbuf);
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

void canPrint(int packetSize) { 
	Serial.printf(" (%f) can0 %08x [%02d] ", micros()/1000000.0, CAN.packetId(), packetSize);
	if (!CAN.packetRtr()) {
		for (int n = 0; n < packetSize && CAN.available(); n++) {
			Serial.printf("%02x", (int)CAN.read());
		}
	}
	Serial.printf("\n");
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

int udpCanOut = 0; // debug output, send all CAN data out over udp port


void open_TTGOTS_SD() { 
	for (int retries = 0; retries < 2; retries++) { 	
		Serial.print("Initializing SD card...");
		if (SD.begin(13, 15, 2, 14)) { 
			Serial.println("initialization done.");
			return;
		}
		Serial.println("initialization failed!");
		delay(100);
	}
	Serial.println("giving up");
}

void openLogFile(const char *filename) {
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

void canParse(int packetSize) {
	static char ibuf[1024];
	static int mpSize = 0;
	static int lastId = 0;
	// try to parse packet
	if (packetSize == 0) 
		Serial.print("0 length packet\n");
	if (packetSize) {
		if (CAN.packetId() != lastId || mpSize >= sizeof(ibuf)) {
			isrData.count++;
			isrData.lastId = lastId;
			isrData.lastSize = mpSize;
			
			// TODO: move all this complicated debugging output out from the ISR
			if (fd != NULL) { 
				if (!qEmpty.empty()) {
					CanPacket pkt;
					pkt.buf = qEmpty.front();
					qEmpty.pop();	
					pkt.timestamp = millis();				
					pkt.id = lastId;
					pkt.len = mpSize;
					memcpy(pkt.buf, ibuf, mpSize);
					qFull.push(pkt);
				} else { 
					isrData.debugDropped++;
				}
			}		

			if (canSerial) {
				static char obuf[1024];
				canToText(ibuf, lastId, mpSize, millis(), obuf, sizeof(obuf));
				Serial.print(obuf);
			}
			
			if ((lastId == 0x18882100 || lastId == 0x188c2100) && ibuf[0] == 0xdd && ibuf[1] == 0x00 && mpSize == 60 && ibuf[15] & 0x40) {
				try {
					isrData.magHdg = floatFromBinary(&ibuf[16]);
				} catch(...) {
					isrData.magHdg = 0;
				}
			} 
			if ((lastId == 0x18882100 || lastId == 0x188c2100) && ibuf[0] == 0xdd && ibuf[1] == 0x00 && mpSize == 60 && ibuf[15] & 0x20) {
				try {
					isrData.magTrack = floatFromBinary(&ibuf[16]);
				} catch(...) {
					isrData.magTrack = 0;
				}
			} 	
			if (lastId == 0x18882100 && ibuf[0] == 0xdd && ibuf[1] == 0x00 && mpSize == 60) {
				// TODO: consider 0x188c? 
				//Serial.printf("AHRS PACKET\n"); 
				try {
					isrData.pitch = floatFromBinary(&ibuf[20]);
					isrData.roll = floatFromBinary(&ibuf[24]);
					isrData.forceSend = true;
					isrData.timestamp = millis();
				} catch(...) {
					isrData.pitch = isrData.roll = 0; 
					isrData.exceptions++;
				}
				//sendCanData(false);
			}
			if (lastId == 0x18882100 && ibuf[0] == 0xdd && ibuf[1] == 0x0a && mpSize >= 44) {
				// TODO: consider 0x188c? 
				//Serial.printf("PS PACKET\n"); 
				try {
					isrData.ias = floatFromBinary(&ibuf[12]); 
					isrData.tas = floatFromBinary(&ibuf[16]);
					isrData.palt = floatFromBinary(&ibuf[24]);
					isrData.timestamp = millis();
					isrData.forceSend = true;
				} catch(...) {
					isrData.ias = isrData.tas = isrData.palt = 0; 
					isrData.exceptions++;
				}
				//sendCanData(false);
			}
			if ((lastId == 0x10882200 || lastId == 0x108c2200) 
				&& ibuf[0] == 0xe4 && ibuf[1] == 0x65 && mpSize == 7) {
				if (0) {
					Serial.printf(" (%f) can0 %08x [%02d] ", micros()/1000000.0, lastId, mpSize);
					for(int n = 0; n < mpSize; n++) {
						if (n > 0 && (n % 8) == 0) { 
							Serial.printf(" "); } 
						Serial.printf("%02x", ibuf[n]);
					}
					Serial.printf("\n");
				}
				try {
					double knobVal = floatFromBinary(&ibuf[3]);
					int knobSel = ibuf[2];					
					isrData.knobVal = knobVal;
					isrData.knobSel = knobSel;
				} catch(...) {
					isrData.knobSel = isrData.knobVal = 0;
					isrData.exceptions++; 
				}
				isrData.forceSend = true;
				//sendCanData(true);
			}

			// reset for next packet
			lastId = CAN.packetId();
			mpSize = 0;
		}
		if (!CAN.packetRtr()) {
			for (int n = 0; n < packetSize; n++) {
				ibuf[mpSize + n] = CAN.read();
			}
			mpSize += packetSize;
		}
	}
}


void canInit() {
	CAN.setPins(pins.canRx, pins.canTx);     
	if (!CAN.begin(1000E3)) {
		Serial.println("Starting CAN failed!");
		while (1) {}
	}
	Serial.println("CAN OPENED");
	CAN.filter(0,0);
	CAN.onReceive(canParse);
}


class EggTimer {
	uint64_t last;
	int interval; 
public:
	EggTimer(int ms) : interval(ms), last(0) {}
	bool tick() { 
		uint64_t now = millis();
		if (now - last > interval) { 
			last = now;
			return true;
		} 
		return false;
	}
};


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
	

bool otaInProgress = false;


class PinPulse { 
public:
	int pin;
	uint64_t toggleTime = 0;
	PinPulse(int p, int initval = 0) : pin(p) { pinMode(p, OUTPUT); digitalWrite(p, initval); } 
	void  pulse(int v, int ms) { 
		toggleTime = ms > 0 ? millis() + ms: 0;
		//Serial.printf("PIN %d VALUE %d\n", pin, v);
		pinMode(pin, OUTPUT);
		digitalWrite(pin, v);
	}
	void run() { 
		if (toggleTime > 0 && millis() >= toggleTime) {
			toggleTime = 0;
			//Serial.printf("PIN %d TOGGLE\n", pin);
			pinMode(pin, OUTPUT);
			digitalWrite(pin, !digitalRead(pin));
		}
	}
};

static int cmdCount = 0;
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
#ifdef SCREEN
	u8g2.begin();
	u8g2.clearBuffer();					// clear the internal memory
	u8g2.setFont(u8g2_font_courB08_tr);	// choose a suitable font
	u8g2.setCursor(0,10);				// set write position
	u8g2.println("Searching for WiFi");
	u8g2.sendBuffer();
#endif
	
	wifi.addAP("Ping-582B", "");
	wifi.addAP("Flora_2GEXT", "maynards");
	wifi.addAP("Team America", "51a52b5354");
	wifi.addAP("ChloeNet", "niftyprairie7");
	uint64_t startms = millis();
	Serial.printf("Waiting for wifi...\n");
	while (WiFi.status() != WL_CONNECTED) {
		wifi.run();
		delay(10);
		if (millis() - startms > 21000)
						break;
	}


#ifdef SCREEN
	u8g2.clearBuffer();					// clear the internal memory
	u8g2.setFont(u8g2_font_courB08_tr);	// choose a suitable font
	u8g2.setCursor(0,10);				// set write position
	u8g2.println(WiFi.localIP());
	u8g2.sendBuffer();
#endif
	digitalWrite(LED_PIN, 1);

	udp.begin(udpPortIn);
	
	adcAttachPin(pins.ADC);
	analogSetCycles(255);
	pid.setGains(4, 0, 0);
	pid.finalGain = 1;

	ArduinoOTA.begin();
	ArduinoOTA.onStart([]() {
		esp_task_wdt_delete(NULL);
		CAN.onReceive(NULL);
		CAN.end();
		Serial.println("Start");
		otaInProgress = true;
	});
	
	openLogFile("CAN%03d.TXT");
	
	for (int n = 0; n < 30; n++) {
		qEmpty.push((uint8_t *)malloc(1024));
	}
	canInit();
}

class LineBuffer {
public:
	char line[1024];
	char len;
	int add(char c) {
		int r = 0; 
		if (c != '\r' && c != '\n');
			line[len++] = c;
		if (len >= sizeof(line) || c == '\n') {
			r = len;
				line[len] = '\0';
			len = 0;
		}
		return r;
	}
};

EggTimer blinkTimer(500), screenTimer(200);
EggTimer pidTimer(250);
static uint8_t buf[1024];


float setPoint = -1;
int lastCmdTime = 0, lastCmdMs = 0, lastCmdPin = 0;
float startTrimPos = 0;

RollingAverage<long int,1000> loopTimeAvg;
uint64_t lastLoopTime = -1;
EggTimer serialReportTimer(1000);
EggTimer pinReportTimer(200), canResetTimer(5000), sl30Heartbeat(1000), sdCardFlush(2000);

uint64_t nextCmdTime = 0;
static bool debugMoveNeedles = false;

static bool first = true;
void loop() {
	esp_task_wdt_reset();
	ArduinoOTA.handle();
	if (otaInProgress) {
		esp_task_wdt_init(220, true);
		CAN.onReceive(NULL);
		CAN.end();
		return;
	}
	if (qFull.empty() == false) {
		static char obuf[1024];
		struct CanPacket pkt = qFull.front();
		qFull.pop();
		
		canToText((char *)pkt.buf, pkt.id, pkt.len, pkt.timestamp, obuf, sizeof(obuf));
		fd.write(obuf, strlen(obuf));
		if (sdCardFlush.tick()) 
			fd.flush();
		
		if (udpCanOut) { 
			if (udp.beginPacket("255.255.255.255", 7894) == 0) 
				isrData.udpErrs++;
			udp.write((uint8_t *)&pkt.id, sizeof(pkt.id));
			udp.write((uint8_t *)&pkt.len, sizeof(pkt.len));
			udp.write(pkt.buf, pkt.len);
			if (udp.endPacket() == 0) 
				isrData.udpErrs++;
		}
		isrData.udpSent++;
		qEmpty.push(pkt.buf);
	}
	if (first || digitalRead(pins.button) == 0) { 
		first = false;
		digitalWrite(pp[0].pin, 1);
		delay(200);
		digitalWrite(pp[0].pin, 0);
		delay(200);
		digitalWrite(pp[1].pin, 1);
		delay(200);
		digitalWrite(pp[1].pin, 0);
	}
	
	uint64_t now = micros();
	if (lastLoopTime != -1) 
		loopTimeAvg.add(now - lastLoopTime);
	lastLoopTime = now;
	//delayMicroseconds(10);
	
	if (canResetTimer.tick()) {
		CAN.filter(0,0);
	}
#ifdef SCREEN
	if (0 && pp[0].toggleTime == 0 && pp[1].toggleTime == 0 && screenTimer.tick()) { 
		u8g2.setCursor(0,20);
		u8g2.printf("CMDS: %03d TSET: %03d   ", cmdCount, (int)setPoint);
		//u8g2.setCursor(0,30);
		//u8g2.printf("LOOP: %05.2f/%05.2f/%05.2f     ", loopTimeAvg.average()/1000.0, loopTimeAvg.min()/1000.0, loopTimeAvg.max()/1000.0);
		u8g2.sendBuffer();
	}
#endif
	
	for (int n =0; n < sizeof(pp)/sizeof(pp[0]); n++) { 
		pp[n].run();
	}

	trimPosAvg.add(analogRead(33));
	if (pinReportTimer.tick()/* || isrData.forceSend*/) { 
		isrData.forceSend = false;
		//Serial.printf("p%d %d %.2f %.2f m %d\n", lastCmdPin,  (int)(micros() - lastCmdTime), 
		//	trimPosAvg.average(), startTrimPos, lastCmdMs);
		sendCanData();
	}
	if (1 && serialReportTimer.tick()) {
		//sendDebugData(); 
		fd.printf("L: %05.3f/%05.3f/%05.3f err:%d can:%d drop:%d\n", loopTimeAvg.average()/1000.0, loopTimeAvg.min()/1000.0, loopTimeAvg.max()/1000.0, 
			isrData.udpErrs, isrData.count, isrData.debugDropped);
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
	
	unsigned int avail;
	if ((avail = udp.parsePacket()) > 0) {
		static LineBuffer line;
		int r = udp.read(buf, min(avail, sizeof(buf)));
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

	static double hd = 0, vd = 0;
	static EggTimer g5Timer(100);
	if (g5Timer.tick() && (debugMoveNeedles || isrData.mode == 5 || startupPeriod > 0)) { 
		hd += .1;
		vd += .15;
		if (hd > 2) hd = -2;
		if (vd > 2) vd = -2;
		if (millis() - isrData.timestamp > 100) 
			hd = vd = -2;
		sl30.setCDI(hd, vd);
	}
	
	if (sl30Heartbeat.tick()) { 
		sl30.pmrrv("301234E"); // send arbitary NAV software version packet as heartbeat
	}

	static float lastKnobVal = 0;
	static uint64_t lastKnobMillis = 0;
	if (isrData.knobVal != lastKnobVal) { 
		if (millis() - lastKnobMillis > 500) 
			isrData.mode = 0;
		lastKnobMillis = millis();
		double delta = isrData.knobVal - lastKnobVal;
		bool oneDegree = abs(delta) < 1.5/180*M_PI && abs(delta) > 0.5/180*M_PI;
		bool evenMode = (isrData.mode & 0x1) == 0;
		if (/*knobSel == isrData.knobSel &&*/ oneDegree && ((delta > 0 && evenMode) || (delta < 0 && !evenMode))) {
			isrData.mode++;
		} else {
			isrData.mode = 0;
		}
		lastKnobVal = isrData.knobVal;
	}	
	udpCanOut = (isrData.mode == 6);
}
