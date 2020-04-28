#include <HardwareSerial.h>
#include "SPI.h"
//#include "Update.h"

//  
// ./parse_sweep ./sweep6_unidirectional_battery_power.txt | grep PR | gnuplot -e 'p "-" u 2:3; pause 10;'


#ifdef ESP32
#include "Update.h"
#include "WebServer.h"
#include "DNSServer.h"
#include "ESPmDNS.h"
#else
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#endif

#include "FS.h"
//#include "ArduinoOTA.h"
#include "WiFiManager.h"
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
#include <U8g2lib.h>
#include <U8x8lib.h>
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

#endif

WiFiUDP udp;
//const char *udpHost = "192.168.4.100";
const char *udpHost = "255.255.255.255";

// port schema:  7890 - g5 in 
//               7891 - g5 out
//               7892 - ifly in
//               7893 - ifly out

int udpPortIn = 7892;



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

PidControl pid(2);
static RollingAverage<int,1000> trimPosAvg;
static EggTimer pidTimer(250);

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
	
	//WiFi.mode(WIFI_STA);
	WiFi.setSleep(false);
#if 0
	//WiFi.setSleep(false);
	
	wifi.addAP("Ping-582B", "");
	wifi.addAP("ChloeNet", "niftyprairie7");
	wifi.addAP("Team America", "51a52b5354");
	while (WiFi.status() != WL_CONNECTED) {
		wifi.run();
	}
#else
	//WiFi.begin("ChloeNet", "niftyprairie7");
	wifiTryConnect("Ping-582B", "");
	wifiTryConnect("ChloeNet", "niftyprairie7");
	u8g2.setCursor(0,20);				// set write position
	u8g2.println("NO");
	u8g2.sendBuffer();
	wifiTryConnect("Ping-582B", "");
	wifiTryConnect("ChloeNet", "niftyprairie7");
	wifiTryConnect("Ping-582B", "");
	wifiTryConnect("ChloeNet", "niftyprairie7");
//	WiFi.begin("Ping-582B", "");
#endif 

	u8g2.clearBuffer();					// clear the internal memory
	u8g2.setFont(u8g2_font_courB08_tr);	// choose a suitable font
	u8g2.setCursor(0,10);				// set write position
	u8g2.println(WiFi.localIP());
	u8g2.sendBuffer();
	digitalWrite(LED_PIN, 1);
	while (WiFi.status() != WL_CONNECTED) {
#ifdef WIFI_MULTI
		wifi.run();
#endif
		delay(1);
	}

	udp.begin(udpPortIn);
	
	Serial.begin(115200, SERIAL_8N1);
	Serial.setTimeout(10);
	
	adcAttachPin(33);
	analogSetCycles(255);
	pid.setGains(.45, 0, 0);
	pid.finalGain = 5;
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
EggTimer pinReportTimer(3);
void loop() {
	uint64_t now = micros();
	if (lastLoopTime != -1) 
		loopTimeAvg.add(now - lastLoopTime);
	lastLoopTime = now;
	
	
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
	if (pinReportTimer.tick()) { 
		Serial.printf("p%d %d %.2f %.2f m %d\n", lastCmdPin,  (int)(micros() - lastCmdTime), 
			trimPosAvg.average(), startTrimPos, lastCmdMs);
	}
	if (serialReportTimer.tick())  
		Serial.printf("L: %05.3f/%05.3f/%05.3f\n", loopTimeAvg.average()/1000.0, loopTimeAvg.min()/1000.0, loopTimeAvg.max()/1000.0);

	if (pidTimer.tick() && setPoint != -1) { 
		float c = pid.add((setPoint - trimPosAvg.average()), trimPosAvg.average(), millis() / 1000.0);
		if (abs(c) > 200) {
			c = 200 * c/abs(c);
		}
		if (abs(c) > 5) {
			if (c < 0) {
				pp[0].pulse(0, abs(c));
			} else { 
				pp[1].pulse(0, abs(c));
			}
		}
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
				Serial.println(line.line);
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
					if (seq != lastSeq)
						setPoint = f;
					lastSeq = seq;
				}
			}
		}
	}
}
