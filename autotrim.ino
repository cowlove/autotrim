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

#include "FS.h"
#include "ArduinoOTA.h"
#include "WiFiUdp.h"
#include "Wire.h"

#include "RunningLeastSquares.h"
#include "PidControl.h"
#define LED_PIN 2
#define BUTTON_PIN 0

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

WiFiUDP udp;
//const char *udpHost = "192.168.4.100";
const char *udpHost = "255.255.255.255";

// port schema:  7890 - g5 in 
//               7891 - g5 out
//               7892 - ifly in
//               7893 - ifly out

int udpPortIn = 7892;

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
	float pitch, roll, knobVal, magHdg;
	int knobSel;
	bool forceSend;
	int lastId, lastSize, exceptions;
} isrData;

PidControl pid(2);
static RollingAverage<int,1000> trimPosAvg;

void sendDebugData() { 
	char sbuf[160];				
	snprintf(sbuf, sizeof(sbuf), "%d %f %s DEBUG\n", 
		(int)isrData.count, (float)trimPosAvg.average(), WiFi.localIP().toString().c_str());
	superSend(sbuf);
	Serial.printf(sbuf);
}

void sendCanData() { 
	char sbuf[160];				
	snprintf(sbuf, sizeof(sbuf), "%+06.3f %+06.3f %+06.3f %d %+06.3f CAN\n", 
		isrData.pitch, isrData.roll, isrData.magHdg, isrData.knobSel, isrData.knobVal);
	superSend(sbuf);
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
	return *(float *)&buf[0];
}

void canParse(int packetSize) {
	static char buf[1024];
	static int mpSize = 0;
	static int lastId = 0;
	// try to parse packet
	if (packetSize) {
		if (CAN.packetId() != lastId || mpSize >= sizeof(buf)) {
			if (0) {
				Serial.printf(" (%f) can0 %08x [%02d] ", micros()/1000000.0, lastId, mpSize);
				for(int n = 0; n < mpSize; n++) {
					if (n > 0 && (n % 8) == 0) { 
						Serial.printf(" "); } 
					Serial.printf("%02x", buf[n]);
				}
				Serial.printf("\n");
			}
			isrData.lastId = lastId;
			isrData.lastSize = mpSize;
			if (lastId == 0x18882100 && buf[0] == 0xdd && buf[1] == 0x00 && mpSize == 60) {
				// TODO: consider 0x188c? 
				//Serial.printf("AHRS PACKET\n"); 
				try {
					isrData.magHdg = floatFromBinary(&buf[16]); 
					isrData.pitch = floatFromBinary(&buf[20]);
					isrData.roll = floatFromBinary(&buf[24]);
				} catch(...) {
					isrData.pitch = isrData.roll = 0; 
					isrData.exceptions++;
				}
				//sendCanData(false);
			}
			if ((lastId == 0x10882200 || lastId == 0x108c2200) 
				&& buf[0] == 0xe4 && buf[1] == 0x65 && mpSize == 7) {
				if (0) {
					Serial.printf(" (%f) can0 %08x [%02d] ", micros()/1000000.0, lastId, mpSize);
					for(int n = 0; n < mpSize; n++) {
						if (n > 0 && (n % 8) == 0) { 
							Serial.printf(" "); } 
						Serial.printf("%02x", buf[n]);
					}
					Serial.printf("\n");
				}
				try {
					 
					isrData.knobVal = floatFromBinary(&buf[3]);
					isrData.knobSel = buf[2];
				} catch(...) {
					isrData.knobSel = isrData.knobVal = 0;
					isrData.exceptions++; 
				}
				isrData.forceSend = true;
				//sendCanData(true);
			}
			lastId = CAN.packetId();
			mpSize = 0;
		}
		if (!CAN.packetRtr()) {
			for (int n = 0; n < packetSize; n++) {
				buf[mpSize + n] = CAN.read();
				isrData.count++;
			}
			mpSize += packetSize;
		}
	}
}


void canInit() {
	CAN.setPins(35/*rx*/,32/*tx*/);    // 25 and 26 on heltec 
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
	


void setup() {
	esp_task_wdt_init(20, true);
	esp_err_t err = esp_task_wdt_add(NULL);

	Serial.begin(921600, SERIAL_8N1);
	Serial.setTimeout(10);
	Serial.printf("AUTOTRIM\n");

	pinMode(LED_PIN, OUTPUT);
	pinMode(BUTTON_PIN, INPUT_PULLUP);
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
	
	
	adcAttachPin(33);
	analogSetCycles(255);
	pid.setGains(4, 0, 0);
	pid.finalGain = 1;
	ArduinoOTA.begin();
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

class PinPulse { 
public:
	int pin;
	uint64_t toggleTime = 0;
	PinPulse(int p, int initval = 1) : pin(p) { pinMode(p, OUTPUT); digitalWrite(p, initval); } 
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
static PinPulse pp[] = { PinPulse(27), PinPulse(14) };
static uint8_t buf[1024];


float setPoint = -1;
int lastCmdTime = 0, lastCmdMs = 0, lastCmdPin = 0;
float startTrimPos = 0;

RollingAverage<long int,1000> loopTimeAvg;
uint64_t lastLoopTime = -1;
EggTimer serialReportTimer(1000);
EggTimer pinReportTimer(200), canResetTimer(5000);
uint64_t nextCmdTime = 0;


void loop() {
	esp_task_wdt_reset();
	ArduinoOTA.handle();
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
	if (pinReportTimer.tick() || isrData.forceSend) { 
		isrData.forceSend = false;
		Serial.printf("p%d %d %.2f %.2f m %d\n", lastCmdPin,  (int)(micros() - lastCmdTime), 
			trimPosAvg.average(), startTrimPos, lastCmdMs);
		sendCanData();
	}
	if (serialReportTimer.tick()) {
		sendDebugData(); 
		Serial.printf("L: %05.3f/%05.3f/%05.3f\n", loopTimeAvg.average()/1000.0, loopTimeAvg.min()/1000.0, loopTimeAvg.max()/1000.0);
	}

	if (millis() > nextCmdTime && setPoint != -1) { 
		float c = pid.add((setPoint - trimPosAvg.average()), trimPosAvg.average(), millis() / 1000.0);
		if (abs(c) > 200) {
			c = 200 * c/abs(c);
		}
		if (abs(c) > 10) {
			if (c < 0) {
				pp[0].pulse(0, abs(c));
			} else { 
				pp[1].pulse(0, abs(c));
			}
		}
		nextCmdTime = millis() + (int)abs(c) + 50;  // schedule time for PID to run. 
	}
	if (digitalRead(BUTTON_PIN) == 0) { 
		digitalWrite(pp[0].pin, 0);
		delay(1000);
		digitalWrite(pp[0].pin, 1);
		delay(100);
		digitalWrite(pp[1].pin, 0);
		delay(2000);
		digitalWrite(pp[1].pin, 1);
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
				//Serial.println(line.line);
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
			}
		}
	}
}
