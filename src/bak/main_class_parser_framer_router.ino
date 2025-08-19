/**
 * ESP32 + SSD1306 + u-blox UBX NAV-PVT Speedometer (PlatformIO / Arduino)
 * - UBX Framer + Router: parse multiple UBX messages on one stream
 * - NAV-PVT mapped to UbxNavPvt struct
 * - sendUbxWithAck(): legacy style, read 10-byte ACK/NAK directly from UART
 * - Checksum verify (ACK + PVT via framer)
 * - Spinner as const char* "/-\\|" + modulo % 4
 * - 24-hour clamp for local time display
 * - Global maxLoopMs; DEBUG_PROFILE: always Serial-print maxLoopMs each loop
 * - 2-space formatting, English comments
 */

#define DEBUG_PROFILE 1 // enable Serial log for maxLoopMs (always prints when
                        // enabled)
#define UBX_ACK_DEBUG 1 // 1 = log ACK checksum / class-id mismatches

#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>
#include <pgmspace.h>

// =============== Hardware configuration ===============
static const int kTimeOffsetHours = 7;   // UTC+7
static const uint32_t BAUD_IN = 38400;   // u-blox default baud
static const uint32_t BAUD_OUT = 115200; // desired baud after config
static const int PIN_BUTTON = 32;        // calibration button
static const uint8_t PIN_RXD2 = 16;      // GPS TX -> ESP32 RXD2
static const uint8_t PIN_TXD2 = 17;      // GPS RX -> ESP32 TXD2

// SSD1306 I2C (ESP32 default: SDA=21, SCL=22)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HardwareSerial GPS(2); // UART2 for GPS

#pragma pack(push, 1) // exact binary layout, no padding
struct UbxNavPvt
{
  uint32_t iTOW;    // U4: GPS time of week of the navigation epoch.
  uint16_t year;    // U2:  Year (UTC)
  uint8_t month;    // U1: Month, range 1..12 (UTC)
  uint8_t day;      // U1: Day of month, range 1..31 (UTC)
  uint8_t hour;     // U1:  Hour of day, range 0..23 (UTC)
  uint8_t minute;   // U1: Minute of hour, range 0..59 (UTC)
  uint8_t second;   // U1:  Seconds of minute, range 0..60 (UTC)
  uint8_t valid;    // X1:  Validity flags  bit 0  U:1 validDate-- 1 = valid UTC Date (see section Time validity in the integration manual for details); bit 1  U:1 validTime-- 1 = valid UTC time of day (see section Time validity in the integration manual for details);  bit 2  U:1 fullyResolved-- 1 = UTC time of day has been fully resolved (no seconds uncertainty). Cannot be used to check if time is completely solved.;  bit 3  U:1 validMag-- 1 = valid magnetic declination
  uint32_t tAcc;    // U4: Time accuracy estimate (UTC
  int32_t nano;     // I4:  Fraction of second, range -1e9 .. 1e9 (UTC)
  uint8_t fixType;  // U1: GNSSfix Type: • 0 = no fix • 1 = dead reckoning only • 2 = 2D-fix • 3 = 3D-fix • 4 = GNSS + dead reckoning combined • 5 = time only fix
  uint8_t flags;    // X1: Fix status flags
  uint8_t flags2;   // X1: Additional flags
  uint8_t numSV;    // U1: Number of satellites used in Nav Solution
  int32_t lon;      // I4: Longitude
  int32_t lat;      // I4: Latitude
  int32_t height;   // I4: Height above ellipsoid mm
  int32_t hMSL;     // I4: Height above mean sea level mm
  uint32_t hAcc;    // U4: Horizontal accuracy estimate mm
  uint32_t vAcc;    // U4: Vertical accuracy estimate mm
  int32_t velN;     // I4: NED north velocity mm/s
  int32_t velE;     // I4: NED east velocity mm/s
  int32_t velD;     // I4: NED down velocity mm/s
  int32_t gSpeed;   // I4: m/s (ground speed 2D)
  int32_t headMot;  // I4: Heading of motion (2-D) 1e-5 deg
  uint32_t sAcc;    // U4: Speed accuracy estimate mm/s
  uint32_t headAcc; // U4: Heading accuracy estimate (both motion and vehicle) 1e-5 deg
  uint16_t pDOP;    // U2: Position DOP 0.01
  uint8_t reserved1[6];
  int32_t headVeh; // I4: Heading of vehicle (2-D) 1e-5 deg
  int16_t magDec;  // I2: Magnetic declination 1e-2 deg
  uint16_t magAcc; // U2: Magnetic declination accuracy 1e-2 deg
};
#pragma pack(pop)

// latest NAV-PVT
static UbxNavPvt pvt;

// =============== Spinner (keep const char*) ===============
static const char spinner[] = "/-\\|";
static uint8_t spinnerScreenPos = 0; // 0..3
static uint8_t spinnerGpsPos = 0;    // 0..3

// =============== Satellite icon ===============
#define sat_logo_HEIGHT 20
#define sat_logo_WIDTH 20
static const uint8_t sat_logo[] PROGMEM = {
    0x00, 0x01, 0x00, 0x80, 0x07, 0x00, 0xC0, 0x06, 0x00, 0x60, 0x30, 0x00,
    0x60, 0x78, 0x00, 0xC0, 0xFC, 0x00, 0x00, 0xFE, 0x01, 0x00, 0xFF, 0x01,
    0x80, 0xFF, 0x00, 0xC0, 0x7F, 0x06, 0xC0, 0x3F, 0x06, 0x80, 0x1F, 0x0C,
    0x80, 0x4F, 0x06, 0x19, 0xC6, 0x03, 0x1B, 0x80, 0x01, 0x73, 0x00, 0x00,
    0x66, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x70, 0x00, 0x00};

// GPS Process Rate
static uint16_t navPvtCountPerSec = 0;
static uint16_t navPvtCountLastSec = 0;
static uint32_t lastNavPvtTick = 0;

// =============== UBX commands (precomputed checksums) ===============
#define PROGMEM_READ_U8(p, i) \
  (uint8_t)pgm_read_byte(((const uint8_t *)(p)) + (i))

static const uint8_t NMEA_GxGGA_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00,
    0, 0, 0, 0, 0, 0, 0xFF, 0x23};
static const uint8_t NMEA_GxGLL_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01,
    0, 0, 0, 0, 0, 0, 0x00, 0x2A};
static const uint8_t NMEA_GxGSA_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02,
    0, 0, 0, 0, 0, 0, 0x01, 0x31};
static const uint8_t NMEA_GxGSV_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03,
    0, 0, 0, 0, 0, 0, 0x02, 0x38};
static const uint8_t NMEA_GxRMC_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04,
    0, 0, 0, 0, 0, 0, 0x03, 0x3F};
static const uint8_t NMEA_GxVTG_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05,
    0, 0, 0, 0, 0, 0, 0x04, 0x46};
static const uint8_t UBX_NAV_PVT_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07,
    0, 0, 0, 0, 0, 0, 0x17, 0xDC};
static const uint8_t UBX_NAV_POSLLH_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02,
    0, 0, 0, 0, 0, 0, 0x12, 0xB9};
static const uint8_t UBX_NAV_STATUS_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03,
    0, 0, 0, 0, 0, 0, 0x13, 0xC0};
static const uint8_t UBX_NAV_VELNED_OFF[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x12,
    0, 0, 0, 0, 0, 0, 0x22, 0x29};
static const uint8_t UBX_NAV_PVT_ON[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1};
static const uint8_t RATE_10Hz[] PROGMEM = {0xB5, 0x62, 0x06, 0x08, 0x06,
                                            0x00, 0x64, 0x00, 0x01, 0x00,
                                            0x01, 0x00, 0x7A, 0x12};
static const uint8_t SET_BAUD_115200[] PROGMEM = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
    0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x23, 0x00,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x5E};

// =============== UBX send helpers ===============
static void sendUbx(const uint8_t *cmd)
{
  const uint8_t len = PROGMEM_READ_U8(cmd, 4) | (PROGMEM_READ_U8(cmd, 5) << 8);
  const uint16_t total = 6 + len + 2; // header(6) + payload + CK(2)
  static uint8_t buf[128];
  if (total > sizeof(buf))
    return;
  for (uint16_t i = 0; i < total; ++i)
    buf[i] = PROGMEM_READ_U8(cmd, i);
  GPS.write(buf, total);
  GPS.flush();
}

static void calcUbxChecksum(const uint8_t *data, uint16_t len, uint8_t &ckA,
                            uint8_t &ckB)
{
  ckA = 0;
  ckB = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    ckA += data[i];
    ckB += ckA;
  }
}

// =============== Framer + Router (multi-message) ===============
static const uint8_t UBX_SYNC1 = 0xB5;
static const uint8_t UBX_SYNC2 = 0x62;
static const uint16_t UBX_FRAMER_BUF = 512; // big enough for most UBX payloads

struct UbxFrame
{
  uint8_t cls = 0;
  uint8_t id = 0;
  uint16_t len = 0;
  const uint8_t *payload = nullptr; // points to framer's internal buffer
};

class UbxFramer
{
public:
  UbxFramer() { reset(); }
  bool poll(Stream &s, UbxFrame &out)
  {
    while (s.available())
    {
      uint8_t c = s.read();
      if (step(c, out))
        return true;
    }
    return false;
  }
  void reset()
  {
    st_ = 0;
    payIdx_ = 0;
    payloadLen_ = 0;
    ckA_ = ckB_ = 0;
  }

private:
  uint8_t st_ = 0;
  uint8_t cls_ = 0, id_ = 0;
  uint16_t payloadLen_ = 0, payIdx_ = 0;
  uint8_t ckA_ = 0, ckB_ = 0;
  uint8_t buf_[UBX_FRAMER_BUF];

  static inline void ckAdd(uint8_t b, uint8_t &A, uint8_t &B)
  {
    A += b;
    B += A;
  }

  bool step(uint8_t c, UbxFrame &out)
  {
    switch (st_)
    {
    case 0:
      if (c == UBX_SYNC1)
        st_ = 1;
      return false;
    case 1:
      if (c == UBX_SYNC2)
        st_ = 2;
      else
        st_ = 0;
      return false;
    case 2:
      cls_ = c;
      ckA_ = c;
      ckB_ = ckA_;
      st_ = 3;
      return false;
    case 3:
      id_ = c;
      ckAdd(c, ckA_, ckB_);
      st_ = 4;
      return false;
    case 4:
      payloadLen_ = c;
      ckAdd(c, ckA_, ckB_);
      st_ = 5;
      return false;
    case 5:
      payloadLen_ |= (uint16_t)c << 8;
      ckAdd(c, ckA_, ckB_);
      if (payloadLen_ > (UBX_FRAMER_BUF - 2))
      {
        st_ = 0;
        return false;
      }
      payIdx_ = 0;
      st_ = (payloadLen_ == 0) ? 7 : 6;
      return false;
    case 6:
      buf_[payIdx_++] = c;
      ckAdd(c, ckA_, ckB_);
      if (payIdx_ == payloadLen_)
        st_ = 7;
      return false;
    case 7:
      if (c != ckA_)
      {
        st_ = 0;
        return false;
      }
      st_ = 8;
      return false;
    case 8:
      st_ = 0;
      if (c != ckB_)
        return false;
      out.cls = cls_;
      out.id = id_;
      out.len = payloadLen_;
      out.payload = buf_;
      return true;
    }
    return false;
  }
};

static UbxFramer FR;

// Only NAV-PVT handler for now; add more if needed
static inline void handleNavPvt(const UbxFrame &f)
{
  if (f.len == sizeof(UbxNavPvt))
  {
    memcpy(&pvt, f.payload, sizeof(UbxNavPvt));
    // Moved here: advance GPS spinner only on valid PVT
    spinnerGpsPos = (spinnerGpsPos + 1) % 4;
    navPvtCountPerSec++; // count valid PVT
  }
}

static inline void routeUbxFrame(const UbxFrame &f)
{
  uint16_t key = ((uint16_t)f.cls << 8) | f.id;
  switch (key)
  {
  case 0x0107: // NAV-PVT
    handleNavPvt(f);
    break;
  default:
    break;
  }
}

// =============== Send UBX and wait for ACK/NAK (read 10 bytes directly)
// ===============
static bool sendUbxWithAck(const uint8_t *cmd, uint8_t expectCls,
                           uint8_t expectId, uint16_t timeoutMs)
{
  sendUbx(cmd);
  delay(10); // short gap

  uint32_t start = millis();
  uint8_t ackBuf[10];
  uint8_t idx = 0;

  while (millis() - start < timeoutMs)
  {
    if (!GPS.available())
    {
      delay(1);
      continue;
    }
    uint8_t b = GPS.read();
    if (idx < sizeof(ackBuf))
      ackBuf[idx++] = b;

    if (idx >= 10)
    {
      // Expect: B5 62 05 {01=ACK | 00=NAK} 02 00 class id ckA ckB
      if (ackBuf[0] == 0xB5 && ackBuf[1] == 0x62 && ackBuf[2] == 0x05)
      {
        uint8_t ackType = ackBuf[3]; // 0x01=ACK, 0x00=NAK
        uint16_t payloadLen = (uint16_t)ackBuf[4] | ((uint16_t)ackBuf[5] << 8);
        if (payloadLen == 2)
        {
          uint8_t ckA, ckB;
          calcUbxChecksum(&ackBuf[2], 4 + payloadLen, ckA, ckB);
          if (ckA == ackBuf[8] && ckB == ackBuf[9])
          {
            uint8_t cls = ackBuf[6];
            uint8_t id = ackBuf[7];
            if (cls == expectCls && id == expectId)
            {
              return (ackType == 0x01); // true: ACK-ACK, false: ACK-NAK
            }
            else
            {
#if UBX_ACK_DEBUG
              Serial.printf(
                  "[UBX][ACK] Class/ID mismatch: got cls=0x%02X id=0x%02X, "
                  "expect cls=0x%02X id=0x%02X\n",
                  cls, id, expectCls, expectId);
#endif
            }
          }
          else
          {
#if UBX_ACK_DEBUG
            Serial.printf(
                "[UBX][ACK] Checksum mismatch: rx=(%02X %02X) calc=(%02X "
                "%02X), type=%s\n",
                ackBuf[8], ackBuf[9], ckA, ckB,
                (ackType == 0x01 ? "ACK" : "NAK"));
#endif
          }
        }
        else
        {
#if UBX_ACK_DEBUG
          Serial.printf(
              "[UBX][ACK] Unexpected payload length: %u (expected 2)\n",
              payloadLen);
#endif
        }
      }
      idx = 0; // not the ACK we want → keep scanning
    }
  }
#if UBX_ACK_DEBUG
  Serial.println(F("[UBX][ACK] Timeout waiting for ACK"));
#endif
  return false;
}

// =============== Unit conversion ===============
static inline float mmps_to_kmh(int32_t mmps) { return mmps * 0.0036f; }

// =============== App variables ===============
int speedCalibrationOffset = 0; // km/h offset (0..5)
unsigned long lastScreenUpdateMs = 0;
unsigned long maxLoopMs = 0; // GLOBAL: always updated, shown on screen
String speedoText = "Km/h";  // keep as requested

// =============== Display render ===============
static void updateScreen()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Screen spinner
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(spinner[spinnerScreenPos]);

  // GPS spinner
  display.setTextSize(2);
  display.setCursor(15, 0);
  display.print(spinner[spinnerGpsPos]);

  // Satellites
  display.drawXBitmap(30, 0, sat_logo, sat_logo_WIDTH, sat_logo_HEIGHT, 1);
  display.setTextSize(1);
  display.setCursor(48, 0);
  display.print(pvt.numSV);

  // sAcc in km/h (1 decimal)
  display.setCursor(60, 13);
  display.print("sAcc: ");
  display.setCursor(93, 13);
  display.print(mmps_to_kmh(pvt.sAcc), 1);

  // Time (if UTC valid)
  if (pvt.valid & 0x04)
  {
    int h24 = (int)pvt.hour + kTimeOffsetHours;
    if (h24 >= 24)
      h24 -= 24;
    else if (h24 < 0)
      h24 += 24;

    bool isPM = (h24 >= 12);
    int h12 = h24 % 12;
    if (h12 == 0)
      h12 = 12;

    display.setCursor(70, 2);
    display.print((unsigned)h12);
    display.print(":");
    if (pvt.minute < 10)
      display.print("0");
    display.print(pvt.minute);
    display.print(":");
    if (pvt.second < 10)
      display.print("0");
    display.print(pvt.second);

    display.setCursor(115, 2);
    display.print(isPM ? "PM" : "AM");
  }
  else
  {
    display.setCursor(81, 2);
    display.print("NO DATA");
  }

  // Speed (km/h + calibration offset)
  display.setTextSize(4);
  display.setCursor(0, 28);
  if (pvt.fixType != 3)
  {
    display.print("---");
  }
  else
  {
    display.print(mmps_to_kmh(pvt.gSpeed) + speedCalibrationOffset, 1);
  }

  // Calibration, max loop time, unit label
  display.setTextSize(1);
  display.setCursor(84, 56);
  display.print("+");
  display.setCursor(91, 56);
  display.print(speedCalibrationOffset);

  display.setTextSize(1);
  display.setCursor(110, 40);
  display.print(maxLoopMs);

  display.setTextSize(1);
  display.setCursor(100, 56);
  display.print(speedoText);

  display.display();
}

// =============== Setup ===============
void setup()
{
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    while (1)
      delay(1000);
  }
  display.clearDisplay();
  display.display();

  pinMode(PIN_BUTTON, INPUT_PULLDOWN);

  // GPS UART2 at default baud
  GPS.begin(BAUD_IN, SERIAL_8N1, PIN_RXD2, PIN_TXD2);

  //-------------------------------------------//
  // SEND UBX COMMAND (IF MODULE HAS NO FLASH) //
  //-------------------------------------------//
  // Serial.println(F("Send configuration (ACK via 10-byte read):"));
  // auto sendWithAckLog = [&](const uint8_t *cmd, uint8_t cls, uint8_t id,
  //                           const char *name, uint16_t timeout)
  // {
  //   bool ok = sendUbxWithAck(cmd, cls, id, timeout);
  //   Serial.printf("%s: %s\n", name, ok ? "ACK OK" : "ACK FAIL/Timeout");
  //   delay(20);
  //   return ok;
  // };

  // // NMEA sentences OFF
  // sendWithAckLog(NMEA_GxGGA_OFF, 0x06, 0x01, "NMEA_GxGGA_OFF", 600);
  // sendWithAckLog(NMEA_GxGLL_OFF, 0x06, 0x01, "NMEA_GxGLL_OFF", 600);
  // sendWithAckLog(NMEA_GxGSA_OFF, 0x06, 0x01, "NMEA_GxGSA_OFF", 600);
  // sendWithAckLog(NMEA_GxGSV_OFF, 0x06, 0x01, "NMEA_GxGSV_OFF", 600);
  // sendWithAckLog(NMEA_GxRMC_OFF, 0x06, 0x01, "NMEA_GxRMC_OFF", 600);
  // sendWithAckLog(NMEA_GxVTG_OFF, 0x06, 0x01, "NMEA_GxVTG_OFF", 600);

  // // Older NAV messages OFF
  // sendWithAckLog(UBX_NAV_PVT_OFF, 0x06, 0x01, "UBX_NAV_PVT_OFF", 600);
  // sendWithAckLog(UBX_NAV_POSLLH_OFF, 0x06, 0x01, "UBX_NAV_POSLLH_OFF", 600);
  // sendWithAckLog(UBX_NAV_STATUS_OFF, 0x06, 0x01, "UBX_NAV_STATUS_OFF", 600);
  // sendWithAckLog(UBX_NAV_VELNED_OFF, 0x06, 0x01, "UBX_NAV_VELNED_OFF", 600);

  // // NAV-PVT ON
  // sendWithAckLog(UBX_NAV_PVT_ON, 0x06, 0x01, "UBX_NAV_PVT_ON", 800);

  // // 10 Hz rate
  // sendWithAckLog(RATE_10Hz, 0x06, 0x08, "CFG-RATE 10Hz", 800);

  // // Change baud to 115200 (CFG-PRT): send command, then ALWAYS reinit UART
  // sendWithAckLog(SET_BAUD_115200, 0x06, 0x00, "CFG-PRT 115200", 1000);

  //-------------------------------------------//
  // END UBX COMMAND (IF MODULE HAS NO FLASH)  //
  //-------------------------------------------//
  GPS.flush();
  GPS.end();
  delay(80);
  GPS.begin(BAUD_OUT, SERIAL_8N1, PIN_RXD2, PIN_TXD2);

  Serial.println(F("Setup complete. Speedometer should start now!"));
}

// =============== Loop ===============
void loop()
{
  unsigned long t0 = millis();

  // Simple button (step calibration 0..5 km/h)
  int buttonState = digitalRead(PIN_BUTTON);
  if (buttonState == HIGH)
  {
    delay(200); // simple debounce; can be replaced with non-blocking if needed
    if (digitalRead(PIN_BUTTON) == LOW)
    {
      speedCalibrationOffset = (speedCalibrationOffset + 1) % 6;
    }
  }

  // Poll UBX frames and route them
  UbxFrame f;
  while (FR.poll(GPS, f))
  {
    routeUbxFrame(f);
  }

  // Update screen ~20 Hz
  unsigned long now = millis();
  if (now - lastScreenUpdateMs > 50)
  {
    updateScreen();
    lastScreenUpdateMs = now;
    spinnerScreenPos = (spinnerScreenPos + 1) % 4;
  }

  if (now - lastNavPvtTick >= 1000)
  {
    lastNavPvtTick = now;
    navPvtCountLastSec = navPvtCountPerSec;
    navPvtCountPerSec = 0;
  }
#ifdef DEBUG_PROFILE
  Serial.printf("[PVT] %u valid NAV-PVT msgs/sec\n", navPvtCountLastSec);
#endif

  // Update maxLoopMs and always print when DEBUG_PROFILE is enabled
  unsigned long loopTime = millis() - t0;
  if (loopTime > maxLoopMs)
  {
    maxLoopMs = loopTime;
  }
#ifdef DEBUG_PROFILE
  Serial.print(F("maxLoopMs: "));
  Serial.println(maxLoopMs);
#endif
}
