#include <NeoSWSerial.h>
#include <SPI.h>

NeoSWSerial gpsPort(4, 5);

//#define TEST
#ifdef TEST
#define DEBUG(arg...) Serial.print(arg)
#define DEBUG_LN(arg...) Serial.println(arg)
#else
#define DEBUG(arg...) {}
#define DEBUG_LN(arg...) {}
#endif

#define BL_PIN 8
#define LE_PIN 9
#define HVE_PIN 3
#define LDR_PIN A2
#define ZERO_PIN 6
#define UNIT_PIN 7

struct VTGInfo {
	char fix;
	int speed;
};

VTGInfo vtgInfo;
int ledState = 1;
byte gps_set_success = 0;

#define NUM_BRIGHTNESS_LEVELS 5
#define MSG_RATE 200

#define NAV5_DYN 1
#define NAV5_MIN_EL 2
#define NAV5_POS_FIX_MODE 4
#define NAV5_DR_LIM 8
#define NAV5_POS_MASK 16
#define NAV5_TIME_MASK 32
#define NAV5_STATIC_HOLD_MASK 64
#define NAV5_DGPS_MASK 128
#define NAV5_CNO_THRESHOLD 1
#define NAV5_UTC 4
#define NAV5_ALL 0xFF

void printHex(uint8_t val) {
	DEBUG(val >> 4, HEX);
	DEBUG(val & 0xf, HEX);
}

void setChecksum(uint8_t *MSG, uint8_t len) {
	MSG[len - 2] = MSG[len - 1] = 0;

	for (uint8_t i = 2; i < len - 2; i++) {
		MSG[len - 2] += MSG[i];
		MSG[len - 1] += MSG[len - 2];
	}
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
	setChecksum(MSG, len);

	for (int i = 0; i < len; i++) {
		gpsPort.write(MSG[i]);
		printHex(MSG[i]);
		DEBUG(',');
	}
	DEBUG_LN();
}

boolean dumpResponse(uint8_t len) {
	uint8_t bytesRead = 0;
	unsigned long startTime = millis();

	while (bytesRead < len) {
		// Timeout if no valid response in 3 seconds
		if (millis() - startTime > 3000) {
			DEBUG_LN(" No response");
			return false;
		}

		// Make sure data is available to read
		if (gpsPort.available()) {
			printHex(gpsPort.read());
			DEBUG(',');
			bytesRead++;
		}
	}

	DEBUG_LN();
	return true;
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  DEBUG(" * Reading ACK response: ");

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  setChecksum(ackPacket, sizeof(ackPacket)/sizeof(uint8_t));

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      DEBUG_LN(" (SUCCESS!)");
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      DEBUG_LN(" (FAILED!)");
      return false;
    }

    // Make sure data is available to read
    if (gpsPort.available()) {
      b = gpsPort.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
        printHex(b);
      }
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
    }
  }
}

void getCurrentNavMode() {
#ifdef TEST
	uint8_t getNav[] = {
		0xB5, 0x62,	// Header
		0x06, 		// Class
		0x24,		// Id
		0x00,		// Length LSB (0)
		0x00,		// Length MSB
		0x00,		// CHKSUM_A
		0x00		// CHKSUM_B
	};

	DEBUG_LN("Requesting uBlox nav mode: ");
	sendUBX(getNav, sizeof(getNav) / sizeof(uint8_t));
	DEBUG_LN("Reading uBlox nav mode: ");
	dumpResponse(44);
	getUBX_ACK(getNav);
#endif
}

void setNavMode() {
	DEBUG_LN("Setting uBlox nav mode: ");
	uint8_t setNav[] = {
		0xB5, 0x62,	// Header
		0x06, 		// Class
		0x24,		// Id
		0x24,		// Length LSB (36)
		0x00,		// Length MSB
		NAV5_DYN | NAV5_POS_FIX_MODE, 0x00, 	// Mask LSB, MSB
		0x04, 		// dyn model - automotive (default = 0)
		0x03, 		// fix mode - auto
		0x00, 0x00, 0x00, 0x00,	// fix altitude (aka sea level) (default = 0)
		0x10, 0x27, 0x00, 0x00,	// fix altitude variance (default = 10,000)
		0x05, 		// minElev - 5 deg
		0x00, 		// drLimit - 0
		0xFA, 0x00,	// pDop - default = 250
		0xFA, 0x00,	// tDop - default = 250
		0x64, 0x00,	// pAcc - default = 100
		0x5E, 0x01,	// tAcc - default = 350
		0x00,		// staticHoldThresh
		0x3C,		// dgnssTimeout (default 60s)
		0x00,		// cnoThreshNumSVs - 0
		0x00,		// cnoThresh - 0
		0x00, 0x00,	// reserved1
		0x00, 0x00,	// staticHoldMaxDist
		0x00,		// utcStandard - automatic
		0x00, 0x00, 0x00, 0x00, 0x00,	// reserved2
		0x00,		// CHKSUM_A
		0x00		// CHKSUM_B
	};

	while (!gps_set_success) {
		sendUBX(setNav, sizeof(setNav) / sizeof(uint8_t));
		gps_set_success = getUBX_ACK(setNav);
	}
	gps_set_success = 0;
}

void setOutputRateMs(int rate) {
	uint8_t setRate[] = {
		0xB5, 0x62,	// Header
		0x06,		// Class
		0x08,		// Id
		0x06,		// Length LSB
		0x00,		// Length MSB
		200,0x00,	// Measurement rate (ms) LSB,MSB
		0x01,0x00,	// Ratio of measurement rate to navigation rate
		0x01,0x00,	// Time ref - 1=GPS time
		0x00,		// CHKSUM_A
		0x00		// CHKSUM_B
	};

	setRate[6] = rate % 256;
	setRate[7] = rate / 256;

	DEBUG_LN("Setting update rate");

	while (!gps_set_success) {
		sendUBX(setRate, sizeof(setRate) / sizeof(uint8_t));
		gps_set_success = getUBX_ACK(setRate);
	}
	gps_set_success = 0;
}

void setup() {
#ifdef TEST
	Serial.begin(115200);
#endif
	DEBUG_LN("Nixie Speedometer");

	vtgInfo.fix = 'N';
	vtgInfo.speed = -1;
	ledState = 1;

	pinMode(BL_PIN, OUTPUT);
	pinMode(LE_PIN, OUTPUT);
	pinMode(HVE_PIN, OUTPUT);

	pinMode(ZERO_PIN, INPUT);
	pinMode(UNIT_PIN, INPUT);

	SPI.begin();

	getNormalizedBrightness(NUM_BRIGHTNESS_LEVELS);
	updateDisplay();

	digitalWrite(BL_PIN, HIGH);
	digitalWrite(HVE_PIN, LOW);

	gpsPort.begin(9600);

	// Get current NAV5 config
	getCurrentNavMode();
	// This command sets automotive mode and confirms it
	setNavMode();
	// Get current NAV5 config
	getCurrentNavMode();

	// Disable the NMEA messages we don't want (everything but VTG)
	gpsPort.print("$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n");
	gpsPort.print("$PUBX,40,ZDA,0,0,0,0,0,0*44\r\n");
//	gpsPort.print("$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n");
	gpsPort.print("$PUBX,40,GSV,0,0,0,0,0,0*59\r\n");
	gpsPort.print("$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n");
	gpsPort.print("$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n");
	gpsPort.print("$PUBX,40,RMC,0,0,0,0,0,0*47\r\n");

	setOutputRateMs(MSG_RATE);

	// Timer0 is already used for millis() - we'll just interrupt somewhere
	// in the middle and call the "Compare A" function below
	OCR0A = 0xAF;
	TIMSK0 |= _BV(OCIE0A);
}

unsigned long now = 0;

const double maxLDR = 1023;
const double minLDR = 0;
const unsigned long sampleTime = 50;

volatile byte brightness = 2;

void getNormalizedBrightness(int maxValue) {
	static const double sensorSmoothCountLDR = 40;
	static double sensorLDRSmoothed = (maxLDR - minLDR)/2;
	static double adjustedLDR = sensorLDRSmoothed;
	static unsigned long lastUpdate = 0;

	if (now - lastUpdate > sampleTime) {
		lastUpdate = now;

		int adc = analogRead(LDR_PIN);

		double sensorDiff = adc - sensorLDRSmoothed;
		sensorLDRSmoothed += (sensorDiff / sensorSmoothCountLDR);
		sensorLDRSmoothed = constrain(sensorLDRSmoothed, minLDR, maxLDR);

		adjustedLDR = sensorLDRSmoothed * sensorLDRSmoothed / maxLDR;

	}

	brightness = map(adjustedLDR, minLDR, maxLDR, 0, maxValue);
}

unsigned long lastNixieChangems = 0;

uint32_t DIGIT_MASK[] { 0x200, 0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x100, 0x0 };

union {
	uint32_t l;
	uint8_t b[4];
} data_;

// Interrupt is called once a millisecond,
SIGNAL(TIMER0_COMPA_vect) {
	static int cycle = 0;

	cycle = (cycle + 1) % NUM_BRIGHTNESS_LEVELS;

	if (brightness < cycle) {
		digitalWrite(BL_PIN, LOW);
	} else {
		digitalWrite(BL_PIN, HIGH);
	}
}

void updateDisplay() {
	static int lastNixieDigit = -2;
	static int lastLEDState =  -1;
	static int nixieDigit = 0;

#ifdef TEST
	if (vtgInfo.speed >= 0) {
#endif
		nixieDigit = vtgInfo.speed;
#ifdef TEST
	}
#endif

#ifdef TEST
	if (now - lastNixieChangems >= 100) {
		lastNixieChangems = now;

//		DEBUG("Brightness=");DEBUG_LN(brightness);

		if (vtgInfo.speed < 0) {
			nixieDigit = (nixieDigit + 1) % 1000;
		}
	}
#endif
	if (lastNixieDigit != nixieDigit || lastLEDState != ledState) {
		lastNixieDigit = nixieDigit;
		lastLEDState = ledState;

		data_.l = ((uint32_t)ledState) << 30;

		if (nixieDigit >= 0) {
			int showZero = digitalRead(ZERO_PIN);

			data_.l |= (DIGIT_MASK[(nixieDigit % 10)] << 20);
			if (nixieDigit >= 10 || showZero) {
				data_.l |= DIGIT_MASK[((nixieDigit / 10) % 10)] << 10;
			}
			if (nixieDigit >= 100 || showZero) {
				data_.l |= DIGIT_MASK[(nixieDigit / 100) % 10];
			}
		}

		SPI.beginTransaction(SPISettings(10000000, LSBFIRST, SPI_MODE1));
		SPI.transfer(data_.b, 4);
		SPI.endTransaction();
		digitalWrite(LE_PIN, HIGH);
		digitalWrite(LE_PIN, LOW);
	}
}

unsigned long lastGPSPollms = 0;

void parseGPS(const char *pBuf) {
#define NUM_NMEA_VALUES 10
#define NMEA_VERB 0
#define VTG_KNOTS 5
#define VTG_KMH 7
#define VTG_FIX 9
	static char *nmeaValues[NUM_NMEA_VALUES];
	char *nmea = pBuf;

	if (strlen(nmea) > 0) {
		DEBUG(nmea);
#ifdef TEST
		Serial.flush();
#endif

		char *tok = strsep(&nmea, ",");
		for (int i = 0; i < NUM_NMEA_VALUES; i++, tok = strsep(&nmea, ",")) {
			if (tok) {
				nmeaValues[i] = tok;
			} else {
				nmeaValues[i] = 0;
			}
		}

		if (nmeaValues[NMEA_VERB] && strstr(nmeaValues[NMEA_VERB], "VTG")) {
			// get the FIX field. This is the last field, so there should be a '*' at position 1
			char *fixValue = nmeaValues[VTG_FIX];
			if (fixValue && strlen(fixValue) == 6) {
				vtgInfo.fix = *fixValue;
			}

			if (digitalRead(UNIT_PIN)) {
				char *kmh = nmeaValues[VTG_KMH];
				if (!kmh || strlen(kmh) < 1 || *kmh < '0' || *kmh > '9') {
					vtgInfo.speed = -1;	// Not a number
				} else {
					vtgInfo.speed = atof(kmh) + 0.5;	// Round, don't truncate
				}
			} else {
				char *knots = nmeaValues[VTG_KNOTS];
				if (!knots || strlen(knots) < 1 || *knots < '0' || *knots > '9') {
					vtgInfo.speed = -1;	// Not a number
				} else {
					vtgInfo.speed = ((int)(atof(knots) * 115) + 50) / 100;	// Round, don't truncate
				}
			}
//			DEBUG("fix="); DEBUG(vtgInfo.fix);
//			DEBUG(",speed="); DEBUG(vtgInfo.speed);
//			DEBUG_LN();
		}
	}
}

void pollGPS() {
#define NMEA_BUF_SIZE 256
	static char nmeaBuf[NMEA_BUF_SIZE];
	static int lastIndex = 0;

	lastGPSPollms = now;
	int i = lastIndex;
	while (gpsPort.available()) {
		int val = gpsPort.read();
		if (i < NMEA_BUF_SIZE - 1) {
			nmeaBuf[i++] = (char) val;
		}
		if (val == '\n') {
			nmeaBuf[i] = 0;	// Terminate string after newline
			parseGPS(nmeaBuf);
			i = 0;
		}
	}

	lastIndex = i;	// non-zero if we only read a partial line
}

void updateFixIndicator() {
	static unsigned long lastLEDChange = 0;
	static unsigned long halfPeriod = 2000;

	switch (vtgInfo.fix) {
	case 'A':
	case 'D':
		ledState = 0;
		lastLEDChange = now;	// Never turn on
		break;
	case 'E':
		halfPeriod = 500;
		break;
	case 'N':
	default:
		halfPeriod = 1000;
		break;
	}

	if (now - lastLEDChange > halfPeriod) {
		lastLEDChange = now;
//		DEBUG_LN(ledState);
		ledState ^= 1;
	}
}

void loop() {
	now = millis();

	pollGPS();
	updateFixIndicator();
	getNormalizedBrightness(NUM_BRIGHTNESS_LEVELS);
	updateDisplay();
}
