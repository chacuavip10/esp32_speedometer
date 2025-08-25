/**
 * ESP32 + SSD1306 + u-blox NAV‑PVT Speedometer (PlatformIO / Arduino)
 * Architecture:
 *   - Dual core: core1 = GPS I/O + parsing, core0 = UI + OLED
 *   - Dual snapshots: REALTIME (NAV‑PVT updates), COUNTER (1 Hz stats)
 *   - Auto-baud detect (UBX or NMEA), UBX config sender to 115200 + 10 Hz PVT
 * Performance:
 *   - UART RX 4KB, UBX framer buf 2KB, batch read for locality
 *   - Redraw only on content change or ≥200ms
 *
 * NOTE: This file is reorganized by functional blocks with clearer comments.
 *       No behavior changes; only ordering and documentation.
 */

/////////////////////
// 1) INCLUDES & GLOBAL CONFIG
/////////////////////

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

#define DEBUG_PROFILE 1 // 1 = enable periodic serial stats (1 Hz)
#define DEBUG_CPU 1     // 1 = enable CPU/RAM usage logs

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Desired GPS baud after configuration
static const uint32_t TARGET_BAUD = 115200;

/////////////////////
// 2) PINS & RUNTIME IDs
/////////////////////

static const uint8_t PIN_RXD2 = 26; // GPS TX -> RXD2
static const uint8_t PIN_TXD2 = 27; // GPS RX -> TXD2
static const int PIN_BUTTON = 32;

static int coreIdGps = -1;
static int coreIdUi = -1;

/////////////////////
// 3) CPU/RAM PROFILING (optional)
/////////////////////
#if DEBUG_CPU
#include "esp_freertos_hooks.h"
#include "esp_timer.h"

// Idle counters for per-core load measurement
static volatile uint32_t s_idleCnt[2] = {0, 0};
static uint32_t s_idleCntLast[2] = {0, 0};
static uint32_t s_idleBaseline[2] = {0, 0};

static bool idle_hook_core0(void)
{
  s_idleCnt[0]++;
  return true;
}
static bool idle_hook_core1(void)
{
  s_idleCnt[1]++;
  return true;
}

// Compute CPU load every 1s (0..100%)
static void computeCpuLoad1s(float outLoad[2])
{
  static uint32_t warmup = 2;  // learn baseline for first 2s
  static float decay = 0.001f; // 0.1%/s baseline decay (keep baseline fresh)

  uint32_t d0 = s_idleCnt[0] - s_idleCntLast[0];
  uint32_t d1 = s_idleCnt[1] - s_idleCntLast[1];
  s_idleCntLast[0] = s_idleCnt[0];
  s_idleCntLast[1] = s_idleCnt[1];

  if (warmup)
  {
    if (d0 > s_idleBaseline[0])
      s_idleBaseline[0] = d0;
    if (d1 > s_idleBaseline[1])
      s_idleBaseline[1] = d1;
    if (--warmup == 0)
    {
      if (!s_idleBaseline[0])
        s_idleBaseline[0] = d0 ? d0 : 1;
      if (!s_idleBaseline[1])
        s_idleBaseline[1] = d1 ? d1 : 1;
    }
  }
  else
  {
    s_idleBaseline[0] = (uint32_t)max((float)s_idleBaseline[0] * (1.0f - decay), (float)d0);
    s_idleBaseline[1] = (uint32_t)max((float)s_idleBaseline[1] * (1.0f - decay), (float)d1);
  }

  float l0 = 1.0f - (float)d0 / (float)(s_idleBaseline[0] ? s_idleBaseline[0] : 1);
  float l1 = 1.0f - (float)d1 / (float)(s_idleBaseline[1] ? s_idleBaseline[1] : 1);
  if (l0 < 0)
    l0 = 0;
  if (l0 > 1)
    l0 = 1;
  if (l1 < 0)
    l1 = 0;
  if (l1 > 1)
    l1 = 1;
  outLoad[0] = l0 * 100.0f;
  outLoad[1] = l1 * 100.0f;
}

// Heap/PSRAM usage in %
static void getMemoryUsage(float &heapPct, float &psramPct, bool &hasPsram)
{
  uint32_t heapTotal = ESP.getHeapSize();
  uint32_t heapFree = ESP.getFreeHeap();
  heapPct = heapTotal ? (100.0f * (float)(heapTotal - heapFree) / (float)heapTotal) : 0.0f;
  hasPsram = psramFound();
  if (hasPsram)
  {
    uint32_t psTotal = ESP.getPsramSize();
    uint32_t psFree = ESP.getFreePsram();
    psramPct = psTotal ? (100.0f * (float)(psTotal - psFree) / (float)psTotal) : 0.0f;
  }
  else
  {
    psramPct = 0.0f;
  }
}
#endif // DEBUG_CPU

/////////////////////
// 4) UBX NAV‑PVT STRUCT (binary layout)
/////////////////////

#pragma pack(push, 1)
struct UbxNavPvt
{
  uint32_t iTOW;
  uint16_t year;
  uint8_t month, day, hour, minute, second, valid;
  uint32_t tAcc;
  int32_t nano;
  uint8_t fixType, flags, flags2, numSV;
  int32_t lon, lat, height, hMSL;
  uint32_t hAcc, vAcc;
  int32_t velN, velE, velD, gSpeed; // mm/s
  int32_t headMot;
  uint32_t sAcc;
  uint32_t headAcc;
  uint16_t pDOP;
  uint8_t reserved1[6];
  int32_t headVeh;
  int16_t magDec;
  uint16_t magAcc;
};
#pragma pack(pop)

/////////////////////
// 5) GLOBAL STATE & FORWARD DECLS
/////////////////////
#define GPS_LOST_TIMEOUT_MS 2200
// Runtime flags
static volatile bool g_running = true;

// Last PVT & loss detection
static UbxNavPvt g_lastPvt{};
static uint32_t lastNavPvtTick = 0;
static bool gpsLost = true;

// Per-second counters (produced on core1, read on core0)
volatile uint16_t ubxSeenPerSec = 0, ubxSeenLastSec = 0;
volatile uint16_t pvtSeenPerSec = 0, pvtSeenLastSec = 0;
volatile uint16_t pvtValidPerSec = 0, pvtValidLastSec = 0;
volatile uint16_t nmeaPerSec = 0, nmeaLastSec = 0;

// NAV‑PVT version counter (increments on each valid PVT)
static volatile uint32_t g_pvtVer = 0;

// Forward decls for snapshot helpers
static void rt_write(const UbxNavPvt &pvt, bool lost, uint32_t ver);
static void cnt_write(uint32_t ubx, uint32_t pvt, uint32_t valid, uint32_t nmea);

/////////////////////
// 6) ASSETS (icons, spinners)
/////////////////////

// 20x20 monochrome satellite icon
#define sat_logo_HEIGHT 20
#define sat_logo_WIDTH 20
static const uint8_t sat_logo[] PROGMEM = {
    0x00, 0x01, 0x00, 0x80, 0x07, 0x00, 0xC0, 0x06, 0x00, 0x60, 0x30, 0x00,
    0x60, 0x78, 0x00, 0xC0, 0xFC, 0x00, 0x00, 0xFE, 0x01, 0x00, 0xFF, 0x01,
    0x80, 0xFF, 0x00, 0xC0, 0x7F, 0x06, 0xC0, 0x3F, 0x06, 0x80, 0x1F, 0x0C,
    0x80, 0x4F, 0x06, 0x19, 0xC6, 0x03, 0x1B, 0x80, 0x01, 0x73, 0x00, 0x00,
    0x66, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x70, 0x00, 0x00};

// Spinners & calibration
static const char spinner[] = "/-\\|";
static int speedCalibrationOffset = 0; // 0..5 km/h (UI button cycles)

/////////////////////
// 7) HARDWARE INSTANCES
/////////////////////

HardwareSerial GPS(2);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/////////////////////
// 8) AUTODETECT BAUD (UBX/NMEA)
/////////////////////

// Common baudrates to probe (order matters)
static int COMMON_BAUD_RATE[] = {38400, 115200, 230400, 460800, 9600};

// Byte read with timeout
static bool readByteWithTimeout(HardwareSerial &s, uint8_t &b, uint32_t ms)
{
  uint32_t t0 = millis();
  while (millis() - t0 < ms)
  {
    if (s.available())
    {
      b = s.read();
      return true;
    }
    yield();
  }
  return false;
}

// Seek UBX sync (0xB5 0x62)
static bool syncToUBX(HardwareSerial &s, uint32_t ms)
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
    {
      yield();
      continue;
    }
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

// Read a UBX frame header+payload and verify checksum quickly
static bool readUBXValid(HardwareSerial &s, uint8_t &cls, uint8_t &id, uint16_t &len, uint32_t firstSyncTimeoutMs = 600)
{
  if (!syncToUBX(s, firstSyncTimeoutMs))
    return false;
  uint8_t lenL, lenH, b;
  uint8_t ckA = 0, ckB = 0;
  auto upd = [&](uint8_t v)
  { ckA+=v; ckB+=ckA; };
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

// Hex helpers for NMEA validation
static uint8_t hex2nibble(char c)
{
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return 0xFF;
}

// NMEA checksum over body (between '$' and '*')
static inline uint8_t nmeaChecksumBody(const String &body)
{
  uint8_t cs = 0;
  for (size_t i = 0; i < body.length(); ++i)
    cs ^= (uint8_t)body[i];
  return cs;
}

static inline bool nmeaParseHexByte(char h1, char h2, uint8_t &out)
{
  uint8_t hi = hex2nibble(h1), lo = hex2nibble(h2);
  if (hi == 0xFF || lo == 0xFF)
    return false;
  out = (uint8_t)((hi << 4) | lo);
  return true;
}

// Blocking-NMEA validator (used at setup for baud probing)
static bool readNMEAValid(HardwareSerial &s, String &head, uint32_t timeoutMs = 1200)
{
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs)
  {
    if (!s.available())
    {
      yield();
      continue;
    }
    char c = s.read();
    if (c != '$')
      continue; // seek '$'
    String body;
    uint32_t tBody = millis();
    while (millis() - tBody < 900)
    {
      if (!s.available())
      {
        yield();
        continue;
      }
      char d = s.read();
      if (d == '*')
      {
        char h1 = 0, h2 = 0;
        uint32_t tw = millis();
        while (millis() - tw < 200 && !s.available())
          yield();
        if (!s.available())
          break;
        h1 = s.read();
        tw = millis();
        while (millis() - tw < 200 && !s.available())
          yield();
        if (!s.available())
          break;
        h2 = s.read();
        uint8_t want = 0;
        if (!nmeaParseHexByte(h1, h2, want))
          break;
        while (s.available())
        {
          char e = s.read();
          if (e == '\n')
            break;
        }
        uint8_t got = nmeaChecksumBody(body);
        if (got == want)
        {
          int comma = body.indexOf(',');
          head = (comma > 0) ? body.substring(0, comma) : body;
          return true;
        }
        break;
      }
      if (d == '\r' || d == '\n')
        break; // malformed
      body += d;
    }
  }
  return false;
}

// Try a list of common baudrates and return the first that yields valid UBX/NMEA
static int detectGPSBaudrateWithStatus()
{
  const int n = sizeof(COMMON_BAUD_RATE) / sizeof(COMMON_BAUD_RATE[0]);
  for (int i = 0; i < n; i++)
  {
    int br = COMMON_BAUD_RATE[i];
    Serial.printf("[SETUP] Trying %d...\n", br);
    GPS.begin(br, SERIAL_8N1, PIN_RXD2, PIN_TXD2);
    GPS.flush();
    delay(60);
    uint32_t window = 1600;
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
    delay(100);
    Serial.printf("  No valid frame @ %d\n", br);
  }
  return -1;
}

/////////////////////
// 9) UBX COMMAND BLOBS & SENDER
/////////////////////

#include <pgmspace.h>
#define PROGMEM_READ_U8(p, i) (uint8_t)pgm_read_byte(((const uint8_t *)(p)) + (i))

// NMEA OFF (I2C/SPI/UART1)
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

// UBX NAV‑PVT enable on UART1
static const uint8_t UBX_NAV_PVT_UART1_ON[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x07, 0x00, 0x91, 0x20, 0x01, 0x53, 0x48};

// Rate 10Hz, dyn model, GNSS masks (only used subset)
static const uint8_t CFG_RATE_MEAS_10Hz[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0x64, 0x00, 0x51, 0xB9};
static const uint8_t CFG_NAVSPG_DYNMODEL_AUTOMOTIVE[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x21, 0x00, 0x11, 0x20, 0x04, 0xF0, 0x4D};
static const uint8_t GAL_ENA_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x21, 0x00, 0x31, 0x10, 0x00, 0xFC, 0x89};
static const uint8_t GAL_E1_ENA_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x07, 0x00, 0x31, 0x10, 0x00, 0xE2, 0x07};

// UART baud change to 115200
static const uint8_t CFG_UART1_BAUDRATE_115200[] PROGMEM = {0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00, 0xF3, 0xA5};

// Send UBX blob from PROGMEM (small scratch buffer)
static void sendUbx(const uint8_t *cmd)
{
  const uint8_t len = PROGMEM_READ_U8(cmd, 4) | (PROGMEM_READ_U8(cmd, 5) << 8);
  const uint16_t total = 6 + len + 2;
  static uint8_t buf[128];
  if (total > sizeof(buf))
    return;
  for (uint16_t i = 0; i < total; i++)
    buf[i] = PROGMEM_READ_U8(cmd, i);
  GPS.write(buf, total);
  GPS.flush();
}

// Calculate UBX checksum over given data
static void calcUbxChecksum(const uint8_t *data, uint16_t len, uint8_t &ckA, uint8_t &ckB)
{
  ckA = ckB = 0;
  for (uint16_t i = 0; i < len; i++)
  {
    ckA += data[i];
    ckB += ckA;
  }
}

// Send UBX and read 10‑byte ACK/NAK
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
  uint8_t t[10];
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
      st = ((((uint16_t)t[4]) | ((uint16_t)b << 8)) == 2) ? PAY0 : SY1;
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
        uint8_t ackType = t[3]; // 0x01=ACK
        if (t[6] == expectCls && t[7] == expectId)
          return (ackType == 0x01);
      }
      st = SY1;
    }
    break;
    }
  }
  Serial.println(F("[UBX][ACK] Timeout"));
  return false;
}

/////////////////////
// 10) UBX FRAMER (stream → frames)
/////////////////////

static const uint8_t UBX_SYNC1 = 0xB5;
static const uint8_t UBX_SYNC2 = 0x62;
static const uint16_t UBX_FRAMER_BUF = 2048;

struct UbxFrame
{
  uint8_t cls = 0, id = 0;
  uint16_t len = 0;
  const uint8_t *payload = nullptr;
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
  bool feed(uint8_t c, UbxFrame &out) { return step(c, out); }
  void reset()
  {
    st_ = 0;
    payIdx_ = 0;
    payloadLen_ = 0;
    ckA_ = ckB_ = 0;
  }

private:
  uint8_t st_ = 0, cls_ = 0, id_ = 0;
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
      st_ = (c == UBX_SYNC2) ? 2 : 0;
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
      payloadLen_ |= ((uint16_t)c << 8);
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

/////////////////////
// 11) FAST NMEA COUNTER (non‑alloc, per‑byte)
/////////////////////

enum : uint8_t
{
  NMEA_IDLE = 0,
  NMEA_BODY,
  NMEA_STAR1,
  NMEA_STAR2,
  NMEA_AFTERCS,
  NMEA_CR
};

static struct
{
  uint8_t st, xsum, h1, h2, haveValid; // state, running XOR, hex digits, valid flag
  uint16_t cnt;                        // body length guard
} s_nmea = {NMEA_IDLE, 0u, 0u, 0u, 0u, 0u};

static inline void nmea_reset() { s_nmea = {NMEA_IDLE, 0, 0, 0, 0, 0}; }
static inline void nmea_finalize_if_valid()
{
  if (s_nmea.haveValid)
    nmeaPerSec++;
  nmea_reset();
}

// Per‑byte feed (counts a sentence only when "*hh" matches)
static inline void nmeaFeedByte(uint8_t c)
{
  if (c == '$')
  {
    if (s_nmea.st != NMEA_IDLE)
      nmea_finalize_if_valid();
    s_nmea.st = NMEA_BODY;
    s_nmea.xsum = 0;
    s_nmea.cnt = 0;
    s_nmea.haveValid = 0;
    return;
  }
  switch (s_nmea.st)
  {
  case NMEA_IDLE:
    break;
  case NMEA_BODY:
    if (c == '*')
      s_nmea.st = NMEA_STAR1;
    else if (c == '\r')
      s_nmea.st = NMEA_CR;
    else if (c == '\n')
      nmea_reset();
    else
    {
      s_nmea.xsum ^= c;
      if (++s_nmea.cnt > 110)
        nmea_reset();
    }
    break;
  case NMEA_STAR1:
  {
    uint8_t hi = hex2nibble((char)c);
    if (hi == 0xFF)
    {
      nmea_reset();
      break;
    }
    s_nmea.h1 = c;
    s_nmea.st = NMEA_STAR2;
  }
  break;
  case NMEA_STAR2:
  {
    uint8_t lo = hex2nibble((char)c);
    if (lo == 0xFF)
    {
      nmea_reset();
      break;
    }
    uint8_t hi = hex2nibble((char)s_nmea.h1);
    uint8_t want = (uint8_t)((hi << 4) | lo);
    s_nmea.haveValid = (want == s_nmea.xsum) ? 1u : 0u;
    s_nmea.st = NMEA_AFTERCS;
  }
  break;
  case NMEA_AFTERCS:
    if (c == '\r')
      s_nmea.st = NMEA_CR;
    else if (c == '\n')
      nmea_finalize_if_valid();
    break;
  case NMEA_CR:
    if (c == '\n')
      nmea_finalize_if_valid();
    break;
  }
}

/////////////////////
// 12) SNAPSHOTS (REALTIME + COUNTER)
/////////////////////

struct RtSnapshot
{
  volatile uint32_t seq;
  UbxNavPvt pvt;
  bool gpsLost;
  uint32_t pvtVer; // increments on each valid NAV‑PVT
};
struct CntSnapshot
{
  volatile uint32_t seq;
  uint16_t ubxPerSec, pvtSeenPerSec, pvtValidPerSec, nmeaPerSec;
};

static RtSnapshot g_rtSnap;
static CntSnapshot g_cntSnap;
static portMUX_TYPE g_rtMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE g_cntMux = portMUX_INITIALIZER_UNLOCKED;

// Lock-free style double‑read (seq even => stable)
static inline void rt_write(const UbxNavPvt &p, bool lost, uint32_t ver)
{
  portENTER_CRITICAL(&g_rtMux);
  g_rtSnap.seq++;
  g_rtSnap.pvt = p;
  g_rtSnap.gpsLost = lost;
  g_rtSnap.pvtVer = ver;
  g_rtSnap.seq++;
  portEXIT_CRITICAL(&g_rtMux);
}
static inline void rt_read(RtSnapshot &out)
{
  uint32_t s1, s2;
  do
  {
    s1 = g_rtSnap.seq;
    if (s1 & 1)
      continue;
    out = g_rtSnap;
    s2 = g_rtSnap.seq;
  } while (s1 != s2 || (s2 & 1));
}
static inline void cnt_write(uint16_t ubx, uint16_t seen, uint16_t valid, uint16_t nmea)
{
  portENTER_CRITICAL(&g_cntMux);
  g_cntSnap.seq++;
  g_cntSnap.ubxPerSec = ubx;
  g_cntSnap.pvtSeenPerSec = seen;
  g_cntSnap.pvtValidPerSec = valid;
  g_cntSnap.nmeaPerSec = nmea;
  g_cntSnap.seq++;
  portEXIT_CRITICAL(&g_cntMux);
}
static inline void cnt_read(CntSnapshot &out)
{
  uint32_t s1, s2;
  do
  {
    s1 = g_cntSnap.seq;
    if (s1 & 1)
      continue;
    out = g_cntSnap;
    s2 = g_cntSnap.seq;
  } while (s1 != s2 || (s2 & 1));
}

/////////////////////
// 13) RENDERING (OLED)
/////////////////////

struct RenderState
{
  bool gpsLost;
  uint8_t fixType, numSV, calOffset;
  bool sAccBad;
  float print_speed; // speed in kmh
  uint8_t gpsSpin;   // spinner advanced when new PVT arrives
  uint8_t scrSpin;   // spinner advanced on each redraw
};
static RenderState lastRS{true, 0xFF, 0, 0xFF, false, 0xFFFF, 0, 0};

// Large, single pass draw (no allocations)
static void drawScreen(const RenderState &rs)
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Screen spinner
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(spinner[rs.scrSpin]);
  // GPS spinner
  display.setCursor(15, 0);
  display.print(spinner[rs.gpsSpin]);

  // Calibration offset
  display.setTextSize(1);
  display.setCursor(88, 2);
  display.print("+");
  display.setCursor(95, 2);
  display.print(rs.calOffset);
  display.setCursor(104, 2);
  display.print("kmh");

  // Satellite icon + number of sats (or 'x' when lost)
  display.drawXBitmap(30, 0, sat_logo, sat_logo_WIDTH, sat_logo_HEIGHT, 1);
  display.setTextSize(2);
  display.setCursor(55, 0);
  if (rs.gpsLost)
    display.print("x");
  else
    display.print(rs.numSV);

  // Accuracy warning (bottom-right 'x')
  if (rs.sAccBad)
  {
    display.setTextSize(1);
    display.setCursor(114, 55);
    display.print("x");
  }

  // Speed (LOST / --- / value)
  display.setTextSize(4);
  display.setCursor(0, 28);
  if (rs.gpsLost)
    display.print("LOST");
  else if (rs.fixType < 2)
    display.print("---");
  else
  {
    if (rs.print_speed >= 100.0f)
    {
      display.setCursor(0, 28);
      display.print((int)lroundf(rs.print_speed));
    }
    else if (rs.print_speed >= 10.0f)
    {
      display.setCursor(30, 28);
      display.print((int)lroundf(rs.print_speed));
    }
    else
    {
      display.setCursor(0, 28);
      display.print(rs.print_speed, 1);
    }
  }
  display.display();
}

/////////////////////
// 14) GPS BYTE PIPELINE (core1)
/////////////////////

// Process a batch: feed both NMEA counter and UBX framer
static inline IRAM_ATTR void processGpsBytesBatch(const uint8_t *buf, size_t n)
{
  UbxFrame f;
  for (size_t i = 0; i < n; ++i)
  {
    const uint8_t c = buf[i];

    // NMEA fast-path (stateful)
    if (c == '$' || s_nmea.st != NMEA_IDLE)
      nmeaFeedByte(c);

    // UBX framer (cheap unless sync seen)
    if (FR.feed(c, f))
    {
      ubxSeenPerSec++;
      if (f.cls == 0x01 && f.id == 0x07 && f.len == sizeof(UbxNavPvt))
      {
        pvtSeenPerSec++;
        memcpy((void *)&g_lastPvt, f.payload, sizeof(UbxNavPvt));
        lastNavPvtTick = millis();
        gpsLost = false;
        g_pvtVer++;
        rt_write(g_lastPvt, gpsLost, g_pvtVer);
        if (g_lastPvt.fixType >= 2)
          pvtValidPerSec++;
      }
    }
  }
}

// Task on core1: read UART in batches, update counters & lost state
static void gps_task(void *)
{
  coreIdGps = xPortGetCoreID();
  Serial.printf("[gps_task] running on core %d\n", coreIdGps);

  static uint8_t batch[512];
  uint32_t tRate = millis();

  while (g_running)
  {
    int avail = GPS.available();
    if (avail > 0)
    {
      int toRead = (avail > (int)sizeof(batch)) ? (int)sizeof(batch) : avail;
      int n = GPS.readBytes((char *)batch, toRead);
      if (n > 0)
        processGpsBytesBatch(batch, (size_t)n);
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(1)); // light yield
    }

    // LOST detection (no PVT for >2.2s)
    uint32_t now = millis();
    if (now - lastNavPvtTick > GPS_LOST_TIMEOUT_MS)
    {
      if (!gpsLost)
      {
        gpsLost = true;
        rt_write(g_lastPvt, gpsLost, g_pvtVer);
      }
    }

    // Publish counters every 1s
    if (now - tRate >= 1000)
    {
      tRate = now;
      ubxSeenLastSec = ubxSeenPerSec;
      ubxSeenPerSec = 0;
      pvtSeenLastSec = pvtSeenPerSec;
      pvtSeenPerSec = 0;
      pvtValidLastSec = pvtValidPerSec;
      pvtValidPerSec = 0;
      nmeaLastSec = nmeaPerSec;
      nmeaPerSec = 0;
      cnt_write(ubxSeenLastSec, pvtSeenLastSec, pvtValidLastSec, nmeaLastSec);
    }
  }
  vTaskDelete(nullptr);
}

/////////////////////
// 15) UI HELPERS & TASK (core0)
/////////////////////

// Piecewise calibration based on actual speed
static inline int getEffectiveCalibrationOffset(int32_t g_mmps, int rawOffset)
{
  int eff = rawOffset;
  if (g_mmps < 5556)
    eff = rawOffset - 3; // <20 km/h
  else if (g_mmps < 11112)
    eff = rawOffset - 2; // 20..40
  else if (g_mmps < 16668)
    eff = rawOffset - 1; // 40..60
  // else = rawOffset
  if (eff < 0)
    eff = 0;
  if (eff > 5)
    eff = 5;
  return eff;
}

// UI task: reads snapshots, decides redraws, prints debug
static void ui_task(void *)
{
  coreIdUi = xPortGetCoreID();
  Serial.printf("[ui_task]  running on core %d\n", coreIdUi);
  Serial.println("====================================================");

  uint32_t lastDrawMs = 0, lastStatMs = 0, lastRedrawUs = 0, lastPvtVer = 0;
  uint32_t redrawCount = 0, redrawsPerSec = 0;

  uint8_t gpsSpin = 0;
  RtSnapshot rt{};
  CntSnapshot cnt{};

  for (;;)
  {
    // Button: short press to cycle calibration offset (0..5)
    if (digitalRead(PIN_BUTTON) == HIGH)
    {
      vTaskDelay(pdMS_TO_TICKS(200));
      if (digitalRead(PIN_BUTTON) == LOW)
        speedCalibrationOffset = (speedCalibrationOffset + 1) % 6;
    }

    // Read realtime snapshot frequently
    rt_read(rt);

    // Build render state from snapshot
    RenderState rs;
    rs.gpsLost = rt.gpsLost;
    rs.fixType = rt.pvt.fixType;
    rs.numSV = rt.pvt.numSV;
    rs.calOffset = (uint8_t)speedCalibrationOffset;

    // sAcc bad when: <20km/h → >1km/h sAcc; ≥20km/h → sAcc > 5% gSpeed
    rs.sAccBad = ((rt.pvt.gSpeed <= 5555) && (rt.pvt.sAcc > 278)) ||
                 ((rt.pvt.gSpeed > 5555) && ((float)rt.pvt.sAcc > 0.05f * (float)rt.pvt.gSpeed));

    // GPS spinner advances when a new PVT arrives
    rs.gpsSpin = lastRS.gpsSpin;
    if (rt.pvtVer != lastPvtVer)
      rs.gpsSpin = (uint8_t)((rs.gpsSpin + 1) % 4);

    // Speed formatting (+ calibration)
    float speedKmh = rt.gpsLost ? 0.0f : (rt.pvt.gSpeed * 0.0036f);
    if (speedKmh < 1.0f)
      speedKmh = 0.0f;
    int effOffset = getEffectiveCalibrationOffset(rt.pvt.gSpeed, speedCalibrationOffset);
    float print_speed = rs.gpsLost ? 0.0f : (speedKmh + effOffset);
    rs.print_speed = print_speed;

    // Redraw policy: on new PVT / lost change / offset change, or ≥200ms
    uint32_t now = millis();
    bool changed = (rt.pvtVer != lastPvtVer) || (rs.gpsLost != lastRS.gpsLost) || (rs.calOffset != lastRS.calOffset);
    bool needDraw = changed || (now - lastDrawMs > 200);
    if (needDraw)
    {
      uint32_t t0 = millis();
      rs.scrSpin = (uint8_t)((lastRS.scrSpin + 1) % 4);
      drawScreen(rs);
      lastRedrawUs = millis() - t0;
      lastDrawMs = now;
      lastRS = rs;
      lastPvtVer = rt.pvtVer;
      redrawCount++;
    }

    // 1 Hz debug print
    if (now - lastStatMs >= 1000)
    {
      lastStatMs = now;
      cnt_read(cnt);
      redrawsPerSec = redrawCount;
      redrawCount = 0;

#if DEBUG_PROFILE
#if DEBUG_CPU
      float load[2] = {0, 0};
      computeCpuLoad1s(load);
      float heapPct = 0.0f, psramPct = 0.0f;
      bool hasPsram = false;
      getMemoryUsage(heapPct, psramPct, hasPsram);
      Serial.printf("CPU load         : core0=%.2f%% core1=%.2f%%\n", load[0], load[1]);
      if (hasPsram)
        Serial.printf("Mem usage        : Heap=%.2f%%  PSRAM=%.2f%%\n", heapPct, psramPct);
      else
        Serial.printf("Mem usage        : Heap=%.2f%%\n", heapPct);
#endif
      Serial.printf("Packet/s         : %u (pps)\n", cnt.ubxPerSec + cnt.nmeaPerSec);
      Serial.printf("UBX/s            : %u (pps)\n", cnt.ubxPerSec);
      Serial.printf("PVT_seen/s       : %u (pps)\n", cnt.pvtSeenPerSec);
      Serial.printf("PVT_valid/s      : %u (pps)\n", cnt.pvtValidPerSec);
      Serial.printf("NMEA/s           : %u (pps)\n", cnt.nmeaPerSec);
      Serial.printf("redraw_ms        : %u (ms)\n", (unsigned long)lastRedrawUs);
      Serial.printf("redraw/s (FPS)   : %u (fps)\n", redrawsPerSec);
      Serial.println("----------------------------------------------");
#endif
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/////////////////////
// 16) SETUP & LOOP
/////////////////////

void setup()
{
  // Max CPU frequency (also set in platformio.ini)
  setCpuFrequencyMhz(240);
  Serial.begin(115200);

  // Disable radios & free memory early
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  esp_wifi_deinit();
  btStop();
  esp_bt_controller_disable();
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
  esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

  // I2C (SSD1306)
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Serial.printf("[ESP32] CPU frequency = %u MHz\n", getCpuFrequencyMhz());

  // OLED splash
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("[SETUP] SSD1306 allocation failed"));
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

  // Auto-baud detect (accepts UBX or NMEA)
  Serial.println("\n[SETUP] [Auto-Baud Detect | UBX checksum or NMEA]");
  static int CURRENT_BAUD_RATE = detectGPSBaudrateWithStatus();
  if (CURRENT_BAUD_RATE > 0)
    Serial.printf("[SETUP] >>> Current GPS baudrate: %d\n", CURRENT_BAUD_RATE);
  else
    Serial.println("[SETUP] !!! Cannot detect GPS baudrate (no valid UBX/NMEA on known rates).");

  // If not at target baud → push configuration at detected baud, then verify
  if (CURRENT_BAUD_RATE != (int)TARGET_BAUD)
  {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.print("[SETUP] Send CMD");
    display.display();

    Serial.println("[SETUP] Looks like GPS module is at factory setting. Reconfiguring...");
    GPS.begin(CURRENT_BAUD_RATE, SERIAL_8N1, PIN_RXD2, PIN_TXD2);

    auto drawAckToOLED = [&](const char *name, bool ok)
    {
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(1);
      display.setCursor(0, 14);
      display.print(name);
      display.setCursor(0, 36);
      display.print(ok ? "OK" : "NOK");
      display.display();
    };
    auto sendWithAckLog = [&](const uint8_t *cmd, uint8_t cls, uint8_t id, const char *name, uint16_t timeout)
    {
      bool ok = sendUbxWithAck(cmd, cls, id, timeout);
      Serial.printf("%s: %s\n", name, ok ? "ACK OK" : "ACK FAIL/Timeout");
      drawAckToOLED(name, ok);
      delay(20);
      return ok;
    };

    // Disable NMEA on all ports, enable NAV‑PVT, set 10 Hz, automotive, set UART 115200
    sendWithAckLog(NMEA_ID_GGA_I2C_OFF, 0x06, 0x8a, "NMEA GGA I2C OFF", 1000);
    sendWithAckLog(NMEA_ID_GGA_SPI_OFF, 0x06, 0x8a, "NMEA GGA SPI OFF", 1000);
    sendWithAckLog(NMEA_ID_GGA_UART1_OFF, 0x06, 0x8a, "NMEA GGA UART1 OFF", 1000);
    sendWithAckLog(NMEA_ID_GLL_I2C_OFF, 0x06, 0x8a, "NMEA GLL I2C OFF", 1000);
    sendWithAckLog(NMEA_ID_GLL_SPI_OFF, 0x06, 0x8a, "NMEA GLL SPI OFF", 1000);
    sendWithAckLog(NMEA_ID_GLL_UART1_OFF, 0x06, 0x8a, "NMEA GLL UART1 OFF", 1000);
    sendWithAckLog(NMEA_ID_GSA_I2C_OFF, 0x06, 0x8a, "NMEA GSA I2C OFF", 1000);
    sendWithAckLog(NMEA_ID_GSA_SPI_OFF, 0x06, 0x8a, "NMEA GSA SPI OFF", 1000);
    sendWithAckLog(NMEA_ID_GSA_UART1_OFF, 0x06, 0x8a, "NMEA GSA UART1 OFF", 1000);
    sendWithAckLog(NMEA_ID_GSV_I2C_OFF, 0x06, 0x8a, "NMEA GSV I2C OFF", 1000);
    sendWithAckLog(NMEA_ID_GSV_SPI_OFF, 0x06, 0x8a, "NMEA GSV SPI OFF", 1000);
    sendWithAckLog(NMEA_ID_GSV_UART1_OFF, 0x06, 0x8a, "NMEA GSV UART1 OFF", 1000);
    sendWithAckLog(NMEA_ID_RMC_I2C_OFF, 0x06, 0x8a, "NMEA RMC I2C OFF", 1000);
    sendWithAckLog(NMEA_ID_RMC_SPI_OFF, 0x06, 0x8a, "NMEA RMC SPI OFF", 1000);
    sendWithAckLog(NMEA_ID_RMC_UART1_OFF, 0x06, 0x8a, "NMEA RMC UART1 OFF", 1000);
    sendWithAckLog(NMEA_ID_VTG_I2C_OFF, 0x06, 0x8a, "NMEA VTG I2C OFF", 1000);
    sendWithAckLog(NMEA_ID_VTG_SPI_OFF, 0x06, 0x8a, "NMEA VTG SPI OFF", 1000);
    sendWithAckLog(NMEA_ID_VTG_UART1_OFF, 0x06, 0x8a, "NMEA VTG UART1 OFF", 1000);

    sendWithAckLog(UBX_NAV_PVT_UART1_ON, 0x06, 0x8a, "UBX NAV-PVT UART1 ON", 1000);
    sendWithAckLog(CFG_NAVSPG_DYNMODEL_AUTOMOTIVE, 0x06, 0x8a, "DYNMODEL AUTOMOTIVE", 1000);
    sendWithAckLog(CFG_RATE_MEAS_10Hz, 0x06, 0x8a, "RATE 10Hz", 1000);
    sendWithAckLog(GAL_ENA_OFF, 0x06, 0x8a, "GAL ENA OFF", 1000);
    delay(1000);
    sendWithAckLog(GAL_E1_ENA_OFF, 0x06, 0x8a, "GAL E1 OFF", 1000);
    delay(1000);
    sendWithAckLog(CFG_UART1_BAUDRATE_115200, 0x06, 0x8a, "UART1 115200", 1000);

    GPS.flush();
    GPS.end();
    delay(80);

    // Verify baud switched successfully
    int rebaud = detectGPSBaudrateWithStatus();
    if (rebaud != (int)TARGET_BAUD)
    {
      Serial.printf("[SETUP] !!! GPS baud mismatch: detected %d, expected %u. Halting.\n", rebaud, TARGET_BAUD);
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(2);
      display.setCursor(0, 14);
      display.print("BAUD FAIL");
      display.setCursor(0, 36);
      display.print(rebaud);
      display.display();
      while (1)
      {
        delay(1000);
      }
    }
  }

  // Final UART configuration
  GPS.end();
  delay(50);
  Serial.println(F("[SETUP] Set GPS rx buffer size to 4096 bytes"));
  GPS.setRxBufferSize(4096);
  GPS.begin(TARGET_BAUD, SERIAL_8N1, PIN_RXD2, PIN_TXD2);

  Serial.println(F("[SETUP] Setup complete. Speedometer should start now!"));
  Serial.println(F("===================================================="));
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(4);
  display.setCursor(0, 14);
  display.print("READY");
  display.display();
  delay(300);

#if DEBUG_CPU
  // Register idle hooks for load measurement
  esp_register_freertos_idle_hook_for_cpu(idle_hook_core0, 0);
  esp_register_freertos_idle_hook_for_cpu(idle_hook_core1, 1);
#endif

  // Start tasks (core1: GPS, core0: UI)
  xTaskCreatePinnedToCore(gps_task, "gps", 6144, nullptr, 9, nullptr, 1);
  xTaskCreatePinnedToCore(ui_task, "ui", 6144, nullptr, 4, nullptr, 0);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
