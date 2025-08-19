/**
 * ESP32 + SSD1306 + u-blox UBX NAV-PVT Speedometer (PlatformIO / Arduino)
 * - UBX Framer + Router: parse multiple UBX messages on one stream
 * - NAV-PVT mapped to UbxNavPvt struct
 * - sendUbxWithAck(): legacy style, read 10-byte ACK/NAK directly from UART
 * - Checksum verify (ACK + PVT via framer)
 * - Spinner as const char* "/-\\|" + modulo % 4
 * - 24-hour clamp for local time display
 */

#define DEBUG_PROFILE 1 // enable Serial log for maxLoopMs_last5s (always prints when
                        // enabled)
#define UBX_ACK_DEBUG 1 // 1 = log ACK checksum / class-id mismatches
#define SHOW_CLOCK 0

#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>
#include <pgmspace.h>
#include <HardwareSerial.h>

// =============== Hardware configuration ===============
static const int kTimeOffsetHours = 7;      // UTC+7
static const uint32_t TARGET_BAUD = 115200; // desired baud after config
static const int PIN_BUTTON = 32;           // calibration button
static const uint8_t PIN_RXD2 = 16;         // GPS TX -> ESP32 RXD2
static const uint8_t PIN_TXD2 = 17;         // GPS RX -> ESP32 TXD2

// SSD1306 I2C (ESP32 default: SDA=21, SCL=22)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
HardwareSerial GPS(2); // UART2 for GPS

// Detect baudrate
static int CURRENT_BAUD_RATE;
int COMMON_BAUD_RATE[] = {9600, 38400, 115200};
// ===== Helpers =====
bool readByteWithTimeout(HardwareSerial &s, uint8_t &b, uint32_t ms)
{
  uint32_t t0 = millis();
  while (millis() - t0 < ms)
  {
    if (s.available())
    {
      b = s.read();
      return true;
    }
  }
  return false;
}

bool syncToUBX(HardwareSerial &s, uint32_t ms)
{
  enum
  {
    WAIT_B5,
    WAIT_62
  } st = WAIT_B5;
  uint32_t t0 = millis();
  while (millis() - t0 < ms)
  {
    if (!s.available())
      continue;
    uint8_t b = s.read();
    if (st == WAIT_B5)
      st = (b == 0xB5) ? WAIT_62 : WAIT_B5;
    else
    {
      if (b == 0x62)
        return true;
      st = (b == 0xB5) ? WAIT_62 : WAIT_B5;
    }
  }
  return false;
}

// Đọc UBX đầy đủ + verify checksum; trả true nếu hợp lệ và trả ra cls/id/len
bool readUBXValid(HardwareSerial &s, uint8_t &cls, uint8_t &id, uint16_t &len,
                  uint32_t firstSyncTimeoutMs = 600)
{
  if (!syncToUBX(s, firstSyncTimeoutMs))
    return false;

  uint8_t lenL, lenH, b;
  uint8_t ckA = 0, ckB = 0;
  auto upd = [&](uint8_t v)
  { ckA += v; ckB += ckA; };

  if (!readByteWithTimeout(s, cls, 150))
    return false;
  upd(cls);
  if (!readByteWithTimeout(s, id, 150))
    return false;
  upd(id);
  if (!readByteWithTimeout(s, lenL, 150))
    return false;
  upd(lenL);
  if (!readByteWithTimeout(s, lenH, 150))
    return false;
  upd(lenH);
  len = (uint16_t)lenL | ((uint16_t)lenH << 8);

  for (uint16_t i = 0; i < len; i++)
  {
    if (!readByteWithTimeout(s, b, 200))
      return false;
    upd(b);
  }
  uint8_t inA, inB;
  if (!readByteWithTimeout(s, inA, 150))
    return false;
  if (!readByteWithTimeout(s, inB, 150))
    return false;

  return (ckA == inA) && (ckB == inB);
}

uint8_t hex2nibble(char c)
{
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return 0xFF;
}

// Đọc 1 câu NMEA hợp lệ: $...*HH\r\n; trả về head (vd: GNGGA/GPRMC)
bool readNMEAValid(HardwareSerial &s, String &head, uint32_t timeoutMs = 1200)
{
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs)
  {
    if (!s.available())
      continue;
    char c = s.read();
    if (c != '$')
      continue;

    uint8_t xsum = 0;
    String body;
    uint32_t t1 = millis();
    while (millis() - t1 < 900)
    {
      if (!s.available())
        continue;
      char d = s.read();
      if (d == '*')
      {
        char h1 = 0, h2 = 0;
        uint32_t t2 = millis();
        while (millis() - t2 < 200 && !s.available())
          ;
        if (!s.available())
          break;
        h1 = s.read();
        t2 = millis();
        while (millis() - t2 < 200 && !s.available())
          ;
        if (!s.available())
          break;
        h2 = s.read();
        uint8_t hi = hex2nibble(h1), lo = hex2nibble(h2);
        if (hi == 0xFF || lo == 0xFF)
          break;
        uint8_t want = (hi << 4) | lo;

        // bỏ CR/LF
        uint32_t t3 = millis();
        while (millis() - t3 < 300 && s.available())
        {
          char e = s.read();
          if (e == '\n')
            break;
        }

        if (xsum == want)
        {
          int comma = body.indexOf(',');
          head = (comma > 0) ? body.substring(0, comma) : body; // ví dụ "GNGGA"
          return true;
        }
        break; // checksum sai
      }
      if (d == '\r' || d == '\n')
        break;
      xsum ^= (uint8_t)d;
      body += d;
    }
  }
  return false;
}

// ===== Hàm chính: dò, IN STATUS, và TRẢ VỀ baudrate =====
int detectGPSBaudrateWithStatus()
{
  const int n = sizeof(COMMON_BAUD_RATE) / sizeof(COMMON_BAUD_RATE[0]);
  for (int i = 0; i < n; i++)
  {
    int br = COMMON_BAUD_RATE[i];
    Serial.printf("Trying %d...\n", br);
    GPS.begin(br, SERIAL_8N1, PIN_RXD2, PIN_TXD2);
    GPS.flush();
    delay(80);

    bool found = false;
    uint32_t window = 1800;
    uint32_t t0 = millis();
    while (millis() - t0 < window)
    {
      uint8_t ucls = 0, uid = 0;
      uint16_t ulen = 0;
      if (readUBXValid(GPS, ucls, uid, ulen, 250))
      {
        Serial.printf("  UBX OK @ %d | cls=0x%02X id=0x%02X len=%u\n", br, ucls, uid, ulen);
        GPS.flush();
        return br;
      }
      String head;
      if (readNMEAValid(GPS, head, 300))
      {
        Serial.printf("  NMEA OK @ %d | %s\n", br, head.c_str());
        GPS.flush();
        return br;
      }
    }
    GPS.end();
    Serial.printf("  No valid frame @ %d\n", br);
  }
  return -1;
}
//
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
float speedCalc;

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
static bool gpsLost = true;

// =============== UBX commands (precomputed checksums) ===============
#define PROGMEM_READ_U8(p, i) \
  (uint8_t)pgm_read_byte(((const uint8_t *)(p)) + (i))

// CFG-MSGOUT, disable all default, only enable UBX_NAV_PVT on uart1
static const uint8_t NMEA_ID_GGA_I2C_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xBA, 0x00, 0x91, 0x20, 0x00, 0x05, 0xC6};
static const uint8_t NMEA_ID_GGA_SPI_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xBE, 0x00, 0x91, 0x20, 0x00, 0x09, 0xDA};
static const uint8_t NMEA_ID_GGA_UART1_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xBB, 0x00, 0x91, 0x20, 0x00, 0x06, 0xCB};
static const uint8_t NMEA_ID_GLL_I2C_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xC9, 0x00, 0x91, 0x20, 0x00, 0x14, 0x11};
static const uint8_t NMEA_ID_GLL_SPI_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xCD, 0x00, 0x91, 0x20, 0x00, 0x18, 0x25};
static const uint8_t NMEA_ID_GLL_UART1_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xCA, 0x00, 0x91, 0x20, 0x00, 0x15, 0x16};
static const uint8_t NMEA_ID_GSA_I2C_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xBF, 0x00, 0x91, 0x20, 0x00, 0x0A, 0xDF};
static const uint8_t NMEA_ID_GSA_SPI_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xC3, 0x00, 0x91, 0x20, 0x00, 0x0E, 0xF3};
static const uint8_t NMEA_ID_GSA_UART1_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xC0, 0x00, 0x91, 0x20, 0x00, 0x0B, 0xE4};
static const uint8_t NMEA_ID_GSV_I2C_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xC4, 0x00, 0x91, 0x20, 0x00, 0x0F, 0xF8};
static const uint8_t NMEA_ID_GSV_SPI_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xC8, 0x00, 0x91, 0x20, 0x00, 0x13, 0x0C};
static const uint8_t NMEA_ID_GSV_UART1_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xC5, 0x00, 0x91, 0x20, 0x00, 0x10, 0xFD};
static const uint8_t NMEA_ID_RMC_I2C_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xAB, 0x00, 0x91, 0x20, 0x00, 0xF6, 0x7B};
static const uint8_t NMEA_ID_RMC_SPI_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xAF, 0x00, 0x91, 0x20, 0x00, 0xFA, 0x8F};
static const uint8_t NMEA_ID_RMC_UART1_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xAC, 0x00, 0x91, 0x20, 0x00, 0xF7, 0x80};
static const uint8_t NMEA_ID_VTG_I2C_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xB0, 0x00, 0x91, 0x20, 0x00, 0xFB, 0x94};
static const uint8_t NMEA_ID_VTG_SPI_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xB4, 0x00, 0x91, 0x20, 0x00, 0xFF, 0xA8};
static const uint8_t NMEA_ID_VTG_UART1_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0xB1, 0x00, 0x91, 0x20, 0x00, 0xFC, 0x99};
static const uint8_t UBX_NAV_PVT_UART1_ON[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x07, 0x00, 0x91, 0x20, 0x01, 0x53, 0x48};
// DYNMODEL Set to automotive
static const uint8_t CFG_NAVSPG_DYNMODEL_AUTOMOTIVE[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x21, 0x00, 0x11, 0x20, 0x04, 0xF0, 0x4D};
// CFG_RATE
static const uint8_t CFG_RATE_MEAS_10Hz[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0x64, 0x00, 0x51, 0xB9}; // Nominal time between GNSS measurements -- E.g. 100 ms results in 10 Hz measurement rate, 1000 ms = 1 Hz measurement rate.\
// GNSS CFG_SIGNAL
static const uint8_t GAL_ENA_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x21, 0x00, 0x31, 0x10, 0x00, 0xFC, 0x89};

static const uint8_t GAL_E1_ENA_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x07, 0x00, 0x31, 0x10, 0x00, 0xE2, 0x07};

// BAUD RATE
static const uint8_t CFG_UART1_BAUDRATE_115200[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00, 0xF3, 0xA5};
static const uint8_t CFG_UART1_BAUDRATE_38400[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x52, 0x40, 0x00, 0x96, 0x00, 0x00, 0xC6, 0x1F};

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
    lastNavPvtTick = millis();
    gpsLost = false;
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
static bool sendUbxWithAck(const uint8_t *cmd, uint8_t expectCls, uint8_t expectId, uint16_t timeoutMs)
{
  sendUbx(cmd);
  delay(10);

  enum
  {
    SY1,
    SY2,
    CLASS,
    ID,
    LENL,
    LENH,
    PAY0,
    PAY1,
    CKA,
    CKB
  } st = SY1;
  uint8_t t[10]; // B5 62 05 xx 02 00 cls id ckA ckB
  uint32_t start = millis();
  while (millis() - start < timeoutMs)
  {
    if (!GPS.available())
    {
      delay(1);
      continue;
    }
    uint8_t b = GPS.read();
    switch (st)
    {
    case SY1:
      st = (b == 0xB5) ? SY2 : SY1;
      break;
    case SY2:
      st = (b == 0x62) ? CLASS : SY1;
      break;
    case CLASS:
      t[2] = b;
      st = (b == 0x05) ? ID : SY1;
      break;
    case ID:
      t[3] = b;
      st = LENL;
      break;
    case LENL:
      t[4] = b;
      st = LENH;
      break;
    case LENH:
      t[5] = b;
      if (((uint16_t)t[4] | ((uint16_t)b << 8)) == 2)
        st = PAY0;
      else
        st = SY1;
      break;
    case PAY0:
      t[6] = b;
      st = PAY1;
      break;
    case PAY1:
      t[7] = b;
      st = CKA;
      break;
    case CKA:
      t[8] = b;
      st = CKB;
      break;
    case CKB:
    {
      t[9] = b;
      uint8_t ckA, ckB;
      uint8_t sum[6] = {0x05, t[3], t[4], t[5], t[6], t[7]};
      calcUbxChecksum(sum, 6, ckA, ckB);
      if (ckA == t[8] && ckB == t[9])
      {
        uint8_t ackType = t[3]; // 0x01=ACK, 0x00=NAK
        if (t[6] == expectCls && t[7] == expectId)
          return (ackType == 0x01);
#if UBX_ACK_DEBUG
        Serial.printf("[UBX][ACK] Class/ID mismatch: got 0x%02X/0x%02X expect 0x%02X/0x%02X\n",
                      t[6], t[7], expectCls, expectId);
#endif
      }
      st = SY1; // tiếp tục dò (sliding)
    }
    break;
    }
  }
#if UBX_ACK_DEBUG
  Serial.println(F("[UBX][ACK] Timeout waiting for ACK"));
#endif
  return false;
}

// =============== Unit conversion ===============
static inline float mmps_to_kmh(int32_t mmps) { return mmps * 0.0036f; }

int getEffectiveCalibrationOffset(float speedKmh, int rawOffset)
{
  int eff = rawOffset;
  if (speedKmh >= 60)
    eff = rawOffset;
  else if (speedKmh >= 40 && speedKmh < 60)
  {
    eff -= 1;
  }
  else if (speedKmh >= 20 && speedKmh < 40)
  {
    eff -= 2;
  }
  else if (speedKmh < 20)
    eff -= 3;

  if (eff < 0)
    eff = 0;
  if (eff > 5)
    eff = 5;
  return eff;
}

// =============== App variables ===============
int speedCalibrationOffset = 0; // km/h offset (0..5)
unsigned long lastScreenUpdateMs = 0;
unsigned long maxLoopMs_last5s = 0;
unsigned long lastMaxLoopResetMs = 0; // NEW: mốc reset maxLoopMs_last5s mỗi 5s
String speedoText = "kmh";

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
  display.setTextSize(2);
  display.setCursor(55, 0);
  if (gpsLost)
    display.print("x");
  else
    display.print(pvt.numSV);

  // // sAcc in km/h (1 decimal)
  if ((float)pvt.sAcc > 0.05f * (float)pvt.gSpeed)
  {
    display.setTextSize(1);
    display.setCursor(114, 55);
    display.print("x");
    // display.setCursor(93, 26);
    // display.print(mmps_to_kmh(pvt.sAcc), 1);
    // display.setTextSize(1);
  }

  if (SHOW_CLOCK)
  {
    // Time (if UTC valid)
    if (gpsLost)
    {
      display.setTextSize(1);
      display.setCursor(81, 2);
      display.print("NO DATA");
    }
    else if (pvt.valid & 0x04)
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
      display.setTextSize(1);
      display.setCursor(83, 2);
      display.print((unsigned)h12);
      display.print(":");
      if (pvt.minute < 10)
        display.print("0");
      display.print(pvt.minute);
      // display.print(":");
      // if (pvt.second < 10)
      //   display.print("0");
      // display.print(pvt.second);

      display.setCursor(115, 2);
      display.print(isPM ? "PM" : "AM");

      display.setTextSize(1);
      display.setCursor(100, 13);
      display.print(speedoText);

      display.setTextSize(1);
      display.setCursor(84, 13);
      display.print("+");
      display.setCursor(91, 13);
      display.print(speedCalibrationOffset);
    }
    else
    {
      display.setTextSize(1);
      display.setCursor(81, 2);
      display.print("NO DATA");
    }
  }
  else
  {
    display.setTextSize(1);
    display.setCursor(88, 2);
    display.print("+");
    display.setCursor(95, 2);
    display.print(speedCalibrationOffset);
    display.setCursor(104, 2);
    display.print(speedoText);
  }

  // Speed (km/h + calibration offset)
  display.setTextSize(4);
  if (gpsLost)
    display.print("LOST");
  else if (pvt.fixType < 2)
    display.print("---");
  else if ((float)pvt.sAcc > 0.2f * (float)pvt.gSpeed)
    display.print("..."); // Speed not reliable
  else
  {
    speedCalc = mmps_to_kmh(pvt.gSpeed);
    if (speedCalc < 1.0f)
      speedCalc = 0.0f;
    int effOffset = getEffectiveCalibrationOffset(speedCalc, speedCalibrationOffset);
    float print_speed = speedCalc + effOffset;
    if (print_speed >= 100.0f) // Speed có 3 chữ số, in ngay sát lề
    {
      display.setCursor(0, 28);
      display.print(print_speed, 0);
    }
    else if (print_speed >= 10.0f) // Speed có 2 chữ số, lùi vào 1 số
    {
      display.setCursor(30, 28);
      display.print(print_speed, 0);
    }
    else // Speed < 10, in cả 1 chữ số thập phân, có làm tròn
    {
      display.setCursor(0, 28);
      display.print(print_speed, 1);
    }
    // TODO: Apply smoothing here - Accuracy-gated Zero Filter - Deadband Filter + timeout
  }

  display.setTextSize(1);
  display.setCursor(110, 40);
  display.print(maxLoopMs_last5s);

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
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(4);
  display.setCursor(0, 14);
  display.print("SETUP");
  display.display();

  pinMode(PIN_BUTTON, INPUT_PULLDOWN);
  // DETECT CURRENT BAUDRATE
  Serial.println("\n[Auto-Baud Detect | UBX checksum or NMEA]");
  CURRENT_BAUD_RATE = detectGPSBaudrateWithStatus();
  if (CURRENT_BAUD_RATE > 0)
    Serial.printf(">>> Current GPS baudrate: %d\n", CURRENT_BAUD_RATE);
  else
    Serial.println("!!! Cannot detect GPS baudrate (no valid UBX/NMEA on known rates).");
  if (CURRENT_BAUD_RATE == 115200)
  {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.print("Send CMD");
    display.display();
    Serial.println("Looklike the GPS Module has factory setting!. Setting up the module now!).");
    // GPS UART2 at default baud
    GPS.begin(CURRENT_BAUD_RATE, SERIAL_8N1, PIN_RXD2, PIN_TXD2);

    //-------------------------------------------//
    // SEND UBX COMMAND (IF MODULE revert to factory setting) //
    // Default 38400 rate //
    //-------------------------------------------//
    Serial.println(F("Send configuration (ACK via 10-byte read):"));

    auto drawAckToOLED = [&](const char *name, bool ok)
    {
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(1);
      display.setCursor(0, 14); // dòng 1
      display.print(name);
      display.setCursor(0, 36); // dòng 2
      display.print(ok ? "OK" : "NOK");
      display.display();
    };

    auto sendWithAckLog = [&](const uint8_t *cmd, uint8_t cls, uint8_t id, const char *name, uint16_t timeout)
    {
      bool ok = sendUbxWithAck(cmd, cls, id, timeout);
      Serial.printf("%s: %s\n", name, ok ? "ACK OK" : "ACK FAIL/Timeout");
      drawAckToOLED(name, ok); // NEW: hiển thị lên OLED
      delay(20);
      return ok;
    };
    // class id : 0x06, msgID : 0x8a->138 : CFG - VALSET

    // DISABLE NMEA MSG
    sendWithAckLog(NMEA_ID_GGA_I2C_OFF, 0x06, 0x8a, "NMEA_ID_GGA_I2C_OFF", 1000);
    sendWithAckLog(NMEA_ID_GGA_SPI_OFF, 0x06, 0x8a, "NMEA_ID_GGA_SPI_OFF", 1000);
    sendWithAckLog(NMEA_ID_GGA_UART1_OFF, 0x06, 0x8a, "NMEA_ID_GGA_UART1_OFF", 1000);
    sendWithAckLog(NMEA_ID_GLL_I2C_OFF, 0x06, 0x8a, "NMEA_ID_GLL_I2C_OFF", 1000);
    sendWithAckLog(NMEA_ID_GLL_SPI_OFF, 0x06, 0x8a, "NMEA_ID_GLL_SPI_OFF", 1000);
    sendWithAckLog(NMEA_ID_GLL_UART1_OFF, 0x06, 0x8a, "NMEA_ID_GLL_UART1_OFF", 1000);
    sendWithAckLog(NMEA_ID_GSA_I2C_OFF, 0x06, 0x8a, "NMEA_ID_GSA_I2C_OFF", 1000);
    sendWithAckLog(NMEA_ID_GSA_SPI_OFF, 0x06, 0x8a, "NMEA_ID_GSA_SPI_OFF", 1000);
    sendWithAckLog(NMEA_ID_GSA_UART1_OFF, 0x06, 0x8a, "NMEA_ID_GSA_UART1_OFF", 1000);
    sendWithAckLog(NMEA_ID_GSV_I2C_OFF, 0x06, 0x8a, "NMEA_ID_GSV_I2C_OFF", 1000);
    sendWithAckLog(NMEA_ID_GSV_SPI_OFF, 0x06, 0x8a, "NMEA_ID_GSV_SPI_OFF", 1000);
    sendWithAckLog(NMEA_ID_GSV_UART1_OFF, 0x06, 0x8a, "NMEA_ID_GSV_UART1_OFF", 1000);
    sendWithAckLog(NMEA_ID_RMC_I2C_OFF, 0x06, 0x8a, "NMEA_ID_RMC_I2C_OFF", 1000);
    sendWithAckLog(NMEA_ID_RMC_SPI_OFF, 0x06, 0x8a, "NMEA_ID_RMC_SPI_OFF", 1000);
    sendWithAckLog(NMEA_ID_RMC_UART1_OFF, 0x06, 0x8a, "NMEA_ID_RMC_UART1_OFF", 1000);
    sendWithAckLog(NMEA_ID_VTG_I2C_OFF, 0x06, 0x8a, "NMEA_ID_VTG_I2C_OFF", 1000);
    sendWithAckLog(NMEA_ID_VTG_SPI_OFF, 0x06, 0x8a, "NMEA_ID_VTG_SPI_OFF", 1000);
    sendWithAckLog(NMEA_ID_VTG_UART1_OFF, 0x06, 0x8a, "NMEA_ID_VTG_UART1_OFF", 1000);

    // ENABLE NAV_PVT
    sendWithAckLog(UBX_NAV_PVT_UART1_ON, 0x06, 0x8a, "UBX_NAV_PVT_UART1_ON", 1000);

    sendWithAckLog(CFG_NAVSPG_DYNMODEL_AUTOMOTIVE, 0x06, 0x8a, "DYNMODEL_AUTOMOTIVE", 1000);

    // 10 Hz rate
    sendWithAckLog(CFG_RATE_MEAS_10Hz, 0x06, 0x8a, "CFG_RATE_MEAS_10Hz", 1000);

    // Disable GNSS GAL ==> GPS L1CA + SBAS + BEIDOU B1 + QZSS, need to wait atleast 500ms to GNSS subsystem to restart
    sendWithAckLog(GAL_ENA_OFF, 0x06, 0x8a, "GAL_ENA_OFF", 1000);
    delay(1000);
    sendWithAckLog(GAL_E1_ENA_OFF, 0x06, 0x8a, "GAL_E1_ENA_OFF", 1000);
    delay(1000);

    // Change baud to 115200 (CFG-PRT): send command, then ALWAYS reinit UART
    // sendWithAckLog(CFG_UART1_BAUDRATE_38400, 0x06, 0x8a, "CFG_UART1_BAUDRATE_38400", 1000);
    sendWithAckLog(CFG_UART1_BAUDRATE_115200, 0x06, 0x8a, "CFG_UART1_BAUDRATE_115200", 1000);
    //-------------------------------------------//
    // END UBX COMMAND (IF MODULE revert to factory setting)  //
    //-------------------------------------------//
    GPS.flush();
    GPS.end();
    delay(80);
  }
  GPS.begin(TARGET_BAUD, SERIAL_8N1, PIN_RXD2, PIN_TXD2);

  Serial.println(F("Setup complete. Speedometer should start now!"));
  Serial.println(F("===================================================="));
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(4);
  display.setCursor(0, 14);
  display.print("START");
  display.display();
  delay(500);
  lastMaxLoopResetMs = millis();
}

// =============== Loop ===============
void loop()
{
  unsigned long t0 = millis();
  static uint32_t lastRateUpdate = 0;
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

  if (now - lastRateUpdate >= 1000)
  {
    lastRateUpdate = now;
    navPvtCountLastSec = navPvtCountPerSec;
    navPvtCountPerSec = 0;
#ifdef DEBUG_PROFILE
    Serial.print(F("maxLoopMs_last5s: "));
    Serial.println(maxLoopMs_last5s);
    Serial.printf("[PVT] %u valid NAV-PVT msgs/sec\n", navPvtCountLastSec);
    Serial.println("---------");
#endif
  }

  if (now - lastNavPvtTick > 5000)
  {
    if (!gpsLost)
    {
      gpsLost = true;
    }
  }

  if (now - lastScreenUpdateMs > 50)
  {
    updateScreen();
    lastScreenUpdateMs = now;
    spinnerScreenPos = (spinnerScreenPos + 1) % 4;
  }

  if (now - lastMaxLoopResetMs >= 5000)
  {
    maxLoopMs_last5s = 0;
    lastMaxLoopResetMs = now;
  }
  // Update maxLoopMs_last5s and always print when DEBUG_PROFILE is enabled
  unsigned long loopTime = millis() - t0;
  if (loopTime > maxLoopMs_last5s)
  {
    maxLoopMs_last5s = loopTime;
  }
}
