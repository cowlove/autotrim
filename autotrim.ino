#include <HardwareSerial.h>
#include "SPI.h"
//#include "Update.h"

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
#include "mavlink.h"

#include "mavlink.h"
#include "Wire.h"

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
	pinMode(3,INPUT);
#ifdef SCREEN
	u8g2.begin();
	u8g2.clearBuffer();					// clear the internal memory
	u8g2.setFont(u8g2_font_courB08_tr);	// choose a suitable font
	u8g2.setCursor(0,10);				// set write position
	u8g2.println("Searching for WiFi");
	u8g2.sendBuffer();
#endif
	
	WiFi.mode(WIFI_STA);
#ifdef ESP32
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
	wifiTryConnect("Ping-582B", "");
	wifiTryConnect("ChloeNet", "niftyprairie7");
	wifiTryConnect("Ping-582B", "");
	wifiTryConnect("ChloeNet", "niftyprairie7");
//	WiFi.begin("Ping-582B", "");
#endif 

	u8g2.setCursor(0,20);				// set write position
	u8g2.println("CONNECTED");
	u8g2.sendBuffer();
	digitalWrite(LED_PIN, 1);
	while (WiFi.status() != WL_CONNECTED) {
#ifdef WIFI_MULTI
		wifi.run();
#endif
		delay(1);
	}

	udp.begin(udpPortIn);
	
	Serial.begin(57600, SERIAL_8N1);
	Serial.setTimeout(10);
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
static PinPulse pp[] = { PinPulse(12), PinPulse(13) };
		
void loop() {
#ifdef SCREEN
	if (pp[0].toggleTime == 0 && pp[1].toggleTime == 0 && screenTimer.tick()) { 
		u8g2.clearBuffer();					// clear the internal memory
		u8g2.setFont(u8g2_font_courB08_tr);	// choose a suitable font
		u8g2.setCursor(0,10);				// set write position
		u8g2.println(WiFi.localIP());
		u8g2.setCursor(0,20);
		u8g2.printf("CMDS: %03d   CRS: %03d", cmdCount, 180);
		u8g2.sendBuffer();
	}
#endif
	
	for (int n =0; n < sizeof(pp)/sizeof(pp[0]); n++) { 
		pp[n].run();
	}
	
	unsigned int avail;
	if ((avail = udp.parsePacket()) > 0) {
		static uint8_t buf[1024];
		static LineBuffer line;
		int r = udp.read(buf, min(avail, sizeof(buf)));
		for (int i = 0; i < r; i++) {
			int ll = line.add(buf[i]);
			if (ll) {
				cmdCount++;
				Serial.println(line.line);
				int ms, val, pin, seq;
				static int lastSeq = 0;
				if (sscanf(line.line, "pin %d %d %d %d", &pin, &val, &ms, &seq) == 4) { 
					cmdCount += 10;
					if (pin >= 0 && pin < sizeof(pp)/sizeof(pp[0]) && seq != lastSeq && ms > 5 && ms <= 500) {
						pp[pin].pulse(val, ms);
					}
					lastSeq = seq;
				}
			}
		}
	}

	delay(1);
}
