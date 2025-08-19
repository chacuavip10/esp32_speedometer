// TODO
// - Change UBX Hex via ucenter to update hex command

#define myTime +7 // <<< adjust you time accourding to UTC time
// #define mms_to_kmh 276.581
#define mms_to_kmh 0.0036 // 1 mm/s = 0.0036 km/h
#define BAUD_38400 38400  // Default baud rate of NEO-M10
#define BAUD_115200 115200

#define button 32

#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// ON ESP32: SCL	GPIO 22, SDA	GPIO 21
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // See datasheet for Address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial GPS(2);

#define RXD2 16 // Uart2 on ESP32
#define TXD2 17 // Uart2 on ESP32

// NEED TO CHECK THIS WITH UCENTER
// HEX COMMAND NEMA
const unsigned char GxGGA_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x23, // GxGGA off
};

const unsigned char NMEA_GxGLL_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A, // GxGLL off
};

const unsigned char NMEA_GxGSA_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31, // GxGSA off
};

const unsigned char NMEA_GxGSV_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38, // GxGSV off
};

const unsigned char NMEA_GxRMC_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, // GxRMC off
};

const unsigned char NMEA_GxVTG_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46, // GxVTG off
};

// HEX COMMAND UBX
const unsigned char UBX_NAV_PVT_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC, // NAV-PVT off
};

const unsigned char UBX_NAV_POSLLH_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, // NAV-POSLLH off
};

const unsigned char UBX_NAV_STATUS_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, // NAV-STATUS off
};

const unsigned char UBX_NAV_VELNED_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x12,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22, 0x29, // NAV-VELNED off
};

const unsigned char UBX_NAV_PVT_ON[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1, // NAV-PVT on
};

// HEX COMMAND RATE
const unsigned char RATE_10Hz[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64,
    0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12, //(10Hz)
};

const unsigned char RATE_5Hz[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8,
    0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
};

const unsigned char RATE_1Hz[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8,
    0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39, //(1Hz)
};

// Add the UBX command for setting the baud rate to 115200
const unsigned char SET_BAUD_RATE_115200[] PROGMEM = {
    // Baud Rate 115200
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
    0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x23, 0x00,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x5E, // Baud Rate 115200
};

// TODO ADD OTHER CFG CHANGE: GNSS GPS+BEIDOU, disable other...
//
//

const unsigned char UBX_HEADER[] = {0xB5, 0x62};

struct NAV_PVT
{
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW; // GPS time of week of the navigation epoch (ms)

  unsigned short year;     // Year (UTC)
  unsigned char month;     // Month, range 1..12 (UTC)
  unsigned char day;       // Day of month, range 1..31 (UTC)
  unsigned char hour;      // Hour of day, range 0..23 (UTC)
  unsigned char minute;    // Minute of hour, range 0..59 (UTC)
  unsigned char second;    // Seconds of minute, range 0..60 (UTC)
  char valid;              // Validity Flags
  unsigned long tAcc;      // Time accuracy estimate (ns)
  long nano;               // Fraction of second (ns)
  unsigned char fixType;   // GNSSfix Type, range 0..5
  char flags;              // Fix Status Flags
  unsigned char reserved1; // reserved
  unsigned char numSV;     // Number of satellites used in Nav Solution

  long lon;           // Longitude (deg)
  long lat;           // Latitude (deg)
  long height;        // Height above Ellipsoid (mm)
  long hMSL;          // Height above mean sea level (mm)
  unsigned long hAcc; // Horizontal Accuracy Estimate (mm)
  unsigned long vAcc; // Vertical Accuracy Estimate (mm)

  long velN;                // NED north velocity (mm/s)
  long velE;                // NED east velocity (mm/s)
  long velD;                // NED down velocity (mm/s)
  long gSpeed;              // Ground Speed (2-D) (mm/s)
  long heading;             // Heading of motion 2-D (deg)
  unsigned long sAcc;       // Speed Accuracy Estimate
  unsigned long headingAcc; // Heading Accuracy Estimate
  unsigned short pDOP;      // Position dilution of precision
  short reserved2;          // Reserved
  unsigned long reserved3;  // Reserved

  unsigned long buffer1; // To Pass Checksum check
  unsigned long buffer2; // To Pass Checksum check
};

NAV_PVT pvt;

// Variable setting for update screen

bool isPM = false;
unsigned char hour = 0;
unsigned char minute = 0;
unsigned char second = 0;

int gSpeed_calibration = 0; // Offset to add to the gSpeed for calibration
int buttonPoll = 0;
// float TEST_SPEED = 0;
long gSpeed = 0;
long sAcc = 0;
float sAcc_float;
int numSV = 0;
int fixType = 0;
unsigned long lastScreenUpdate = 0;
// char speedBuf[16];
// char satsBuf[16];
float speedCalc;
String speedoText = "N/A";
unsigned long looptime = 0;
unsigned long max_loop = 0;

char *spinner = "/-\\|";
byte screenRefreshSpinnerPos = 0;
byte gpsUpdateSpinnerPos = 0;

#define sat_logo_HEIGHT 20
#define sat_logo_WIDTH 20
const unsigned char sat_logo[] = {
    0x00, 0x01, 0x00, 0x80, 0x07, 0x00, 0xc0, 0x06, 0x00, 0x60, 0x30, 0x00,
    0x60, 0x78, 0x00, 0xc0, 0xfc, 0x00, 0x00, 0xfe, 0x01, 0x00, 0xff, 0x01,
    0x80, 0xff, 0x00, 0xc0, 0x7f, 0x06, 0xc0, 0x3f, 0x06, 0x80, 0x1f, 0x0c,
    0x80, 0x4f, 0x06, 0x19, 0xc6, 0x03, 0x1b, 0x80, 0x01, 0x73, 0x00, 0x00,
    0x66, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x70, 0x00, 0x00}; // Nếu thiếu RAM -> static const unsigned char PROGMEM sat_logo[]

void calcChecksum(unsigned char *CK)
{
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++)
  {
    CK[0] += ((unsigned char *)(&pvt))[i];
    CK[1] += CK[0];
  }
}

long numGPSMessagesReceived = 0;

bool processGPS()
{
  static int fpos = 0;
  static unsigned char checksum[2];
  const int payloadSize = sizeof(NAV_PVT);

  while (GPS.available())
  {
    byte c = GPS.read();
    if (fpos < 2)
    {
      if (c == UBX_HEADER[fpos])
        fpos++;
      else
        fpos = 0;
    }
    else
    {
      if ((fpos - 2) < payloadSize)
        ((unsigned char *)(&pvt))[fpos - 2] = c;

      fpos++;

      if (fpos == (payloadSize + 2))
      {
        calcChecksum(checksum);
      }
      else if (fpos == (payloadSize + 3))
      {
        if (c != checksum[0])
          fpos = 0;
      }
      else if (fpos == (payloadSize + 4))
      {
        fpos = 0;
        if (c == checksum[1])
        {
          return true;
        }
      }
      else if (fpos > (payloadSize + 4))
      {
        fpos = 0;
      }
    }
  }
  return false;
}

void updateScreen()
{
  // TODO: print gSpeed_cali on screen
  speedoText = "Km/h";
  speedCalc = (gSpeed * mms_to_kmh) + gSpeed_calibration; // kmh
  sAcc_float = sAcc * mms_to_kmh;
  display.clearDisplay();
  display.setTextColor(WHITE);

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(spinner[screenRefreshSpinnerPos]);

  display.setTextSize(2);
  display.setCursor(15, 0);
  display.print(spinner[gpsUpdateSpinnerPos]);

  // Display satelites
  display.drawXBitmap(30, 0, sat_logo, sat_logo_WIDTH, sat_logo_HEIGHT, 1);
  display.setTextSize(1);
  display.setCursor(48, 0);
  display.print(numSV);

  // Speed accuracy estimate display
  display.setTextSize(1);
  display.setCursor(60, 13);
  display.print("sAcc: "); // Km/h or mph
  display.setTextSize(1);
  display.setCursor(93, 13);
  display.print(sAcc_float, 1);

  if (pvt.valid & 0x04) // 00000100 -> flag for UTC Time is valid!
  {
    display.setTextSize(1);
    display.setCursor(70, 2);
    display.print(hour); // Print Hour
    display.print(":");
    if (minute < 10)
      display.print("0");  // Add leading zero if needed
    display.print(minute); // Print Minute
    display.print(":");
    if (second < 10)
      display.print("0");  // Add leading zero if needed
    display.print(second); // Print Second

    // Display AM/PM
    display.setCursor(115, 2);
    if (isPM)
    {
      display.print("PM");
    }
    else
    {
      display.print("AM");
    }
  }
  else
  {
    // Display "No Fix" if UTC time is not valid
    display.setTextSize(1);
    display.setCursor(81, 2);
    display.print("NO DATA");
  }

  display.setTextSize(4);
  display.setCursor(0, 28);
  if (fixType != 3)
    display.print("---"); // if no 3D-FIX
  // display.print("TEST_SPEED", 1);
  else
    display.print(speedCalc, 1);

  // Display speed calibration setting
  display.setTextSize(1);
  display.setCursor(84, 56);
  display.print("+");
  display.setCursor(91, 56);
  display.print(gSpeed_calibration);
  // Display max looptime
  display.setTextSize(1);
  display.setCursor(110, 40);
  display.print(max_loop);

  display.setTextSize(1);
  display.setCursor(100, 56);
  display.print(speedoText); // Km/h or mph

  display.display();
}

//========================================================================//
//                            NOW THE MAIN PART                           //
//========================================================================//
void setup()
{
  GPS.begin(BAUD_38400, SERIAL_8N1, RXD2, TXD2);
  Serial.begin(BAUD_115200);
  Serial.println("Initialize OLED display");
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.display();
  pinMode(button, INPUT_PULLDOWN);
  // Initialize OLED display
  //---------------------------------------------------//
  // ENABLE THIS IF GPS MODULE DON'T HAVE FLASH MEMORY //
  //---------------------------------------------------//
  // Serial.println("send configuration data:");
  // // send configuration data in UBX protocol
  // Serial.println("NMEA_GxGLL_OFF");
  // for (unsigned int i = 0; i < sizeof(NMEA_GxGLL_OFF); i++)
  // {
  //   GPS.write(pgm_read_byte(NMEA_GxGLL_OFF + i));
  //   delay(5);
  // }

  // Serial.println("NMEA_GxGSA_OFF");
  // for (unsigned int i = 0; i < sizeof(NMEA_GxGSA_OFF); i++)
  // {
  //   GPS.write(pgm_read_byte(NMEA_GxGSA_OFF + i));
  //   delay(5);
  // }

  // Serial.println("NMEA_GxGSV_OFF");
  // for (unsigned int i = 0; i < sizeof(NMEA_GxGSV_OFF); i++)
  // {
  //   GPS.write(pgm_read_byte(NMEA_GxGSV_OFF + i));
  //   delay(5);
  // }

  // Serial.println("NMEA_GxRMC_OFF");
  // for (unsigned int i = 0; i < sizeof(NMEA_GxRMC_OFF); i++)
  // {
  //   GPS.write(pgm_read_byte(NMEA_GxRMC_OFF + i));
  //   delay(5);
  // }

  // Serial.println("NMEA_GxRMC_OFF");
  // for (unsigned int i = 0; i < sizeof(NMEA_GxRMC_OFF); i++)
  // {
  //   GPS.write(pgm_read_byte(NMEA_GxRMC_OFF + i));
  //   delay(5);
  // }

  // Serial.println("NMEA_GxVTG_OFF");
  // for (unsigned int i = 0; i < sizeof(NMEA_GxVTG_OFF); i++)
  // {
  //   GPS.write(pgm_read_byte(NMEA_GxVTG_OFF + i));
  //   delay(5);
  // }

  // Serial.println("UBX_NAV_PVT_OFF");
  // for (unsigned int i = 0; i < sizeof(UBX_NAV_PVT_OFF); i++)
  // {
  //   GPS.write(pgm_read_byte(UBX_NAV_PVT_OFF + i));
  //   delay(5);
  // }

  // Serial.println("UBX_NAV_POSLLH_OFF");
  // for (unsigned int i = 0; i < sizeof(UBX_NAV_POSLLH_OFF); i++)
  // {
  //   GPS.write(pgm_read_byte(UBX_NAV_POSLLH_OFF + i));
  //   delay(5);
  // }

  // Serial.println("UBX_NAV_STATUS_OFF");
  // for (unsigned int i = 0; i < sizeof(UBX_NAV_STATUS_OFF); i++)
  // {
  //   GPS.write(pgm_read_byte(UBX_NAV_STATUS_OFF + i));
  //   delay(5);
  // }

  // Serial.println("UBX_NAV_VELNED_OFF");
  // for (unsigned int i = 0; i < sizeof(UBX_NAV_VELNED_OFF); i++)
  // {
  //   GPS.write(pgm_read_byte(UBX_NAV_VELNED_OFF + i));
  //   delay(5);
  // }

  // Serial.println("UBX_NAV_PVT_ON");
  // for (unsigned int i = 0; i < sizeof(UBX_NAV_PVT_ON); i++)
  // {
  //   GPS.write(pgm_read_byte(UBX_NAV_PVT_ON + i));
  //   delay(5);
  // }

  // Serial.println("Set update rate to 10hz");
  // for (unsigned int i = 0; i < sizeof(RATE_10Hz); i++)
  // {
  //   GPS.write(pgm_read_byte(RATE_10Hz + i));
  //   delay(5);
  // }

  // Serial.println("Set GNSS Constellations: TODO");
  // // TODO
  // // Note that changes to any items within this group will trigger a reset to
  // // the GNSS subsystem. The
  // //  reset takes some time, so wait first for the acknowledgement from the
  // //  receiver and then 0.5 seconds before sending the next command.
  // Serial.println("WAIT MODULE TO REBOOT");
  // delay(1000); // Wait module to reboot

  // Serial.println("SET GPS BAUD RATE TO 115200");
  // // Optional: Change baud rate to 115200
  // for (unsigned int i = 0; i < sizeof(SET_BAUD_RATE_115200); i++)
  // {
  //   GPS.write(pgm_read_byte(SET_BAUD_RATE_115200 + i));
  //   delay(5);
  // }
  //---------------------------------------------------//
  //              CONFIGURE END                        //
  //---------------------------------------------------//

  GPS.end(); // End the 9600 baud communication

  // Serial.println("Increase buffer size to 256 bytes");
  // GPS.setTxBufferSize(256);
  // GPS.setRxBufferSize(256); // Change to 1024 if seeing drop, Must call
  // before begin()
  Serial.println("Reinitialize GPS at 115200");

  GPS.begin(BAUD_115200); // Start communication at 115200 baud
  Serial.println("Setup complete. Speedometer should start now!");
}

void loop()
{
  looptime = millis();
  // Check if button is pressed!
  buttonPoll = digitalRead(button);
  if (buttonPoll == HIGH)
  {
    delay(200);
    buttonPoll = digitalRead(button);
    if (buttonPoll == LOW)
    {
      gSpeed_calibration =
          (gSpeed_calibration + 1) % 6; // if gSpeed_calibration = 6 -> 0
      // TEST_SPEED = 5.1 + TEST_SPEED;
    }
  }

  // Now process GPS
  if (processGPS())
  {
    hour = (pvt.hour myTime) %
           24; // Add timezone hours and ensure it wraps around after 23
    minute = pvt.minute;
    second = pvt.second;

    // Convert to 12-hour format
    if (hour >= 12)
    {
      isPM = true;
    }
    else
    {
      isPM = false;
    }

    if (hour > 12)
    {
      hour -= 12;
    }
    else if (hour == 0)
    {
      hour = 12; // Midnight case (12 AM)
    }

    numSV = pvt.numSV;
    fixType = pvt.fixType;
    gSpeed = pvt.gSpeed;
    sAcc = pvt.sAcc;
    gpsUpdateSpinnerPos = (gpsUpdateSpinnerPos + 1) % 4;
  }

  unsigned long now = millis();
  if (now - lastScreenUpdate > 50) // update screen at 20Hz (The SSD1306 hardware I2C @ 400kHz means 23ms. i.e. 40 FPS is a practical limit.)
  {
    // looptime = millis() - looptime + 26;
    updateScreen(); // took 25-26ms to complete
    // Serial.println("updateScreen() took " + String(millis() - then) + "ms");
    lastScreenUpdate = now;
    screenRefreshSpinnerPos = (screenRefreshSpinnerPos + 1) % 4;
  }
  looptime = millis() - looptime;
  if (looptime > max_loop)
  {
    max_loop = looptime;
  }
  // Serial.println("looptime: " + String(millis() - start_loop));
}
