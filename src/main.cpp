// main.cpp
// ProForm TDF v2 Controller
//
// Features:
// - RS-485 (UART1) interface to ProForm lower control board (status/cadence + resistance control)
// - BLE FTMS server (Zwift-compatible) using NimBLE
// - LVGL UI on Waveshare ESP32-S3 Touch LCD 7"
// - Virtual rear gearing (14 gears)
// - Stream-based RS-485 frame parser for low-latency cadence updates
//
// Button behavior (active-low with INPUT_PULLUP):
// - Short press UP/DOWN: shift gear up/down
// - Long press UP/DOWN: increase/decrease base resistance (manual/offline trim)
//

#include <Arduino.h>
#include <NimBLEDevice.h>

#include <lvgl.h>
#include "ESP_Panel_Conf.h"
#include "esp_display_panel.hpp"
#include "lvgl_v8_port.h"

// =======================================================
// Hardware configuration
// =======================================================

// RS-485 UART (bike bus)
static const int RS485_RX = 15;
static const int RS485_TX = 16;
HardwareSerial BikeSerial(1);

// Buttons (GPIO inputs, active-low)
static const int BTN_GEAR_UP   = 43;
static const int BTN_GEAR_DOWN = 44;

// =======================================================
// ProForm protocol / polling
// =======================================================

static const uint8_t POLL_FRAME[] = {
  0xBA, 0xB5, 0xB1, 0xB0, 0xB3, 0xB0, 0xB0, 0xB0, 0xB2,
  0xB0, 0xB0, 0xB0, 0xB0, 0xC1, 0xC1, 0x8D, 0x8A
};

static bool looksLikeStatusFrame(const uint8_t* f, size_t n) {
  // BA B5 ... 8D 8A
  return (n >= 19 && f[0] == 0xBA && f[1] == 0xB5 && f[n - 2] == 0x8D && f[n - 1] == 0x8A);
}

// =======================================================
// Kevin nibble mapping (reverse-engineered encoding)
// =======================================================

static int calcValueKevin(uint8_t b) {
  if (b < 0xB0) return -1;
  if (b <= 0xB9) return (b & 0x0F);                    // 0..9
  if (b >= 0xC1 && b <= 0xC6) return (b & 0x0F) + 10;  // 10..15
  return -1;
}

static uint16_t decodePairKevin(uint8_t hi, uint8_t lo) {
  int a = calcValueKevin(hi);
  int b = calcValueKevin(lo);
  if (a < 0 || b < 0) return 0;
  return (uint16_t)(a * 16 + b);
}

static bool kevinPairValid(uint8_t hi, uint8_t lo) {
  return (calcValueKevin(hi) >= 0 && calcValueKevin(lo) >= 0);
}

// If the selected cadence field is in half-RPM units, set true.
static const bool CADENCE_NEEDS_DIV2 = false;

// Cadence field location in BA B5 status frame (byte index for hi nibble, lo at +1)
static const int CADENCE_BYTE_INDEX = 13;

// =======================================================
// Resistance gear tables (Kevin)
// =======================================================

static uint8_t gearCommand1[130];
static uint8_t gearCommand2[130];
static uint8_t gearCommand3[130];
static uint8_t gearCommand4[130];

static void InitializeGears() {
  uint8_t code[]      = { 0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xC1,0xC2,0xC3,0xC4,0xC5,0xC6 };
  uint8_t oneMask[]   = { 0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
  uint8_t threeMask[] = { 0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1 };
  int initialOffset2 = 15;
  int initialOffset3 = 8;
  int initialOffset4 = 11;

  int countloop = 0;
  int maxp = 8;
  for (int p = 0; p < maxp; p++) {
    for (unsigned int k = 0; k < 16; k++) {
      int one = oneMask[k];
      gearCommand1[countloop] = code[one + p];

      int two = (k + initialOffset2) % 16;
      gearCommand2[countloop] = code[two];

      int three = (initialOffset3 - threeMask[k]);
      gearCommand3[countloop] = code[three - (p % maxp)];

      int four = (16 - initialOffset4 - k) % 16;
      gearCommand4[countloop] = code[four];

      countloop++;
    }
  }
}

static uint32_t lastRs485TxMs = 0;
static const uint32_t RS485_TX_GUARD_MS = 180;

// Send resistance to bike (0..80)
static void sendResistance(uint8_t level0to80) {
  if (level0to80 > 80) level0to80 = 80;

  // map 0..80 -> 0..127 (table index)
  uint8_t idx = (uint8_t)((level0to80 * 127U) / 80U);

  uint8_t pkt[] = {
    0xBA, 0xB6, 0xB1, 0xB0, 0xB6, 0xB0, 0xB0, 0xB0,
    0xB5, 0xB0, 0xB0,
    gearCommand1[idx],
    gearCommand2[idx],
    gearCommand3[idx],
    gearCommand4[idx],
    0x8D, 0x8A
  };

  BikeSerial.write(pkt, sizeof(pkt));
  BikeSerial.flush();
  delay(25);
  lastRs485TxMs = millis();
}

// =======================================================
// Virtual gearing + manual base resistance
// =======================================================

static const int NUM_GEARS = 14;
static volatile int gearIndex = 5;  // 0..13 (default ~middle)

// Rebalanced gearing: less unusably-low range, more high-end
static const float gearRatio[NUM_GEARS] = {
  0.90f, 1.00f, 1.10f, 1.22f, 1.35f, 1.50f,
  1.65f, 1.82f, 2.00f, 2.20f, 2.42f, 2.66f,
  2.92f, 3.20f
};

static volatile uint8_t baseResistance = 0; // 0..80 from grade or manual
static volatile uint8_t resistance     = 0; // 0..80 after gearing

static uint8_t applyGearing(uint8_t baseRes, int gIdx) {
  if (gIdx < 0) gIdx = 0;
  if (gIdx >= NUM_GEARS) gIdx = NUM_GEARS - 1;
  float eff = (float)baseRes * gearRatio[gIdx];
  int r = (int)lroundf(eff);
  if (r < 0) r = 0;
  if (r > 80) r = 80;
  return (uint8_t)r;
}

// =======================================================
// Buttons (short press + long press)
// =======================================================

struct PressButton {
  int pin;
  bool last;
  uint32_t tDown;
  bool longFired;

  void begin(int p) {
    pin = p;
    pinMode(pin, INPUT_PULLUP);
    last = false;
    tDown = 0;
    longFired = false;
  }

  // return: 0 none, 1 short press, 2 long press (once)
  int update(uint32_t longMs = 500) {
    bool pressed = (digitalRead(pin) == LOW);
    int ev = 0;

    if (pressed && !last) {
      tDown = millis();
      longFired = false;
    }
    if (pressed && !longFired && (millis() - tDown) >= longMs) {
      longFired = true;
      ev = 2;
    }
    if (!pressed && last) {
      if (!longFired) ev = 1;
    }

    last = pressed;
    return ev;
  }
};

static PressButton btnUp;
static PressButton btnDn;

// =======================================================
// Cadence
// =======================================================

static volatile uint16_t cadence_rpm = 0;
static float cad_ema = 0.0f;
static uint32_t lastNonZeroCadMs = 0;

static volatile uint32_t lastStatusFrameMs = 0;

static const uint32_t CAD_ZERO_TIMEOUT_MS = 900;
static const uint32_t STATUS_FRAME_TIMEOUT_MS = 900;

// Smoothing parameters (single cadence channel)
static const float CAD_ALPHA_UP   = 0.70f;
static const float CAD_ALPHA_DOWN = 0.30f;

// Decode cadence from a BA B5 status frame (selected bytes)
static uint16_t decodeCadence(const uint8_t* f, size_t n) {
  if (CADENCE_BYTE_INDEX + 1 >= (int)n) return 0;
  if (!kevinPairValid(f[CADENCE_BYTE_INDEX], f[CADENCE_BYTE_INDEX + 1])) return 0;
  uint16_t v = decodePairKevin(f[CADENCE_BYTE_INDEX], f[CADENCE_BYTE_INDEX + 1]);
  if (CADENCE_NEEDS_DIV2) v = (uint16_t)(v / 2);
  return v;
}

// =======================================================
// Power estimate (apply -15% scale)
// =======================================================

static volatile int16_t power_w = 0;
static volatile uint16_t speed_x100_kph = 0; // 0.01 km/h for FTMS

// Shape constants
static float a = 1900.0f;
static float b = -0.3f;
static float c = 0.5f;
static float d = 0.0f;

// Apply a fixed 15% reduction to bring estimates closer to perceived effort
static const float POWER_SCALE = 0.85f;

static int16_t estimatePower(uint16_t rpm, uint8_t res) {
  float r = (res * 127.0f) / 80.0f;
  float p = (rpm * r * r / a) + (rpm * c) + d + (r * b);
  if (p < 0) p = 0;
  p *= POWER_SCALE;
  if (p > 2000) p = 2000;
  return (int16_t)lroundf(p);
}

// =======================================================
// Speed/Distance/Elapsed (derived from cadence + virtual gearing)
// =======================================================
//
// Use gearRatio[] as a drivetrain ratio: wheel_rpm = cadence_rpm * gearRatio[gearIndex]
// Assume a 700c road wheel circumference and integrate distance over time.
// Elapsed time starts when the user first pedals and never stops thereafter.
//

static const float WHEEL_CIRCUMFERENCE_M = 2.105f;       // ~700x25c (meters/rev)
static const float KM_TO_MILES = 0.621371192f;

static volatile float speed_mph = 0.0f;
static volatile float wheel_rpm = 0.0f;

static float distance_km = 0.0f;
static float distance_miles = 0.0f;

static bool timerStarted = false;
static uint32_t pedalStartMs = 0;
static uint32_t lastDistMs = 0;

static uint32_t elapsedSeconds() {
  if (!timerStarted) return 0;
  return (millis() - pedalStartMs) / 1000U;
}

static void updateSpeedDistanceElapsed() {
  uint32_t now = millis();

  // Start timer on first detected pedaling (cadence > 0)
  if (!timerStarted && cadence_rpm > 0) {
    timerStarted = true;
    pedalStartMs = now;
    lastDistMs = now;
  }

  float ratio = gearRatio[gearIndex];
  wheel_rpm = (float)cadence_rpm * ratio;

  // speed (kph) = wheel_rpm * circumference(m) * 60 / 1000
  float speed_kph_f = wheel_rpm * WHEEL_CIRCUMFERENCE_M * (60.0f / 1000.0f);
  if (speed_kph_f < 0) speed_kph_f = 0;

  // Requested tweak: double speed (and therefore distance)
  speed_kph_f *= 2.0f;

  speed_mph = speed_kph_f * KM_TO_MILES;

  // FTMS expects 0.01 km/h
  float sx100 = speed_kph_f * 100.0f;
  if (sx100 < 0) sx100 = 0;
  if (sx100 > 65535.0f) sx100 = 65535.0f;
  speed_x100_kph = (uint16_t)lroundf(sx100);

  // Integrate distance only after timer starts
  if (timerStarted) {
    float dt_s = (float)(now - lastDistMs) / 1000.0f;
    if (dt_s < 0) dt_s = 0;
    lastDistMs = now;

    // distance_km += speed_kph * hours
    distance_km += (speed_kph_f * (dt_s / 3600.0f));
    if (distance_km < 0) distance_km = 0;
    distance_miles = distance_km * KM_TO_MILES;
  } else {
    distance_km = 0.0f;
    distance_miles = 0.0f;
  }
}

// =======================================================
// BLE FTMS (Zwift)
// =======================================================

static float grade_pct = 0.0f;
static bool bleConnected = false;
static String systemStatus = "booting";

static bool simMode = false;
static uint32_t lastSimMs = 0;

// Target + slew state for SIM base resistance
static volatile uint8_t targetBaseResistance = 0;
static float baseResF = 0.0f;

static uint8_t gradeToResistance(float gradePercent) {
  float g = gradePercent;

  // Tuneables
  const float FLAT_BASE  = 24.0f;  // base at 0% grade
  const float UP_SLOPE   = 2.8f;   // per +1% grade
  const float DOWN_SLOPE = 1.4f;   // per -1% grade (gentler)
  const float MIN_BASE   = 12.0f;  // never go below this in SIM
  const float MAX_BASE   = 80.0f;

  float r = FLAT_BASE + (g >= 0.0f ? (g * UP_SLOPE) : (g * DOWN_SLOPE));

  if (r < MIN_BASE) r = MIN_BASE;
  if (r > MAX_BASE) r = MAX_BASE;
  return (uint8_t)lroundf(r);
}

static void updateBaseResistanceSlew() {
  if (!simMode) return;

  static uint32_t lastMs = 0;
  uint32_t now = millis();
  uint32_t dt = now - lastMs;
  if (dt < 40) return; // ~25 Hz
  lastMs = now;

  // Ramp rates in levels/sec (tune to taste)
  const float UP_PER_SEC   = 6.0f;
  const float DOWN_PER_SEC = 10.0f;

  float target = (float)targetBaseResistance;
  float rate = (target > baseResF) ? UP_PER_SEC : DOWN_PER_SEC;
  float maxStep = rate * ((float)dt / 1000.0f);

  float diff = target - baseResF;
  if (fabsf(diff) <= maxStep) baseResF = target;
  else baseResF += (diff > 0 ? maxStep : -maxStep);

  baseResistance = (uint8_t)lroundf(baseResF);
}

static NimBLEUUID UUID_FTMS_SVC((uint16_t)0x1826);
static NimBLEUUID UUID_FTMS_FEATURE((uint16_t)0x2ACC);
static NimBLEUUID UUID_INDOOR_BIKE_DATA((uint16_t)0x2AD2);
static NimBLEUUID UUID_CONTROL_POINT((uint16_t)0x2AD9);
static NimBLEUUID UUID_STATUS((uint16_t)0x2ADA);

static NimBLEServer* srv = nullptr;
static NimBLECharacteristic* chFeature = nullptr;
static NimBLECharacteristic* chBikeData = nullptr;
static NimBLECharacteristic* chCtrlPt = nullptr;
static NimBLECharacteristic* chStatus = nullptr;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*) override {
    bleConnected = true;
    systemStatus = "BLE connected";
  }
  void onDisconnect(NimBLEServer*) override {
    bleConnected = false;
    systemStatus = "BLE advertising";
    NimBLEDevice::startAdvertising();
  }
};

static void ctrlPointIndicateResponse(uint8_t reqOpcode, uint8_t resultCode) {
  // FTMS Response Code: [0]=0x80, [1]=req opcode, [2]=result
  uint8_t resp[3] = { 0x80, reqOpcode, resultCode };
  chCtrlPt->setValue(resp, sizeof(resp));
  chCtrlPt->indicate();
}

class CtrlPointCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) override {
    std::string v = c->getValue();
    if (v.empty()) return;
    uint8_t opcode = (uint8_t)v[0];

    switch (opcode) {
      case 0x00: // Request Control
        systemStatus = "FTMS: control granted";
        ctrlPointIndicateResponse(opcode, 0x01);
        break;

      case 0x07: // Start/Resume
        systemStatus = "FTMS: start";
        ctrlPointIndicateResponse(opcode, 0x01);
        break;

      case 0x08: // Stop/Pause
        systemStatus = "FTMS: stop";
        simMode = false;
        ctrlPointIndicateResponse(opcode, 0x01);
        break;

      case 0x04: // Set Target Resistance Level (manual base)
        if (v.size() >= 2) {
          uint8_t lvl = (uint8_t)v[1];
          if (lvl > 80) lvl = 80;

          // If SIM mode is active (recent 0x11), ignore 0x04 to avoid control fights
          if (simMode && (millis() - lastSimMs) < 3000) {
            systemStatus = "FTMS: ignore 0x04 (SIM)";
            ctrlPointIndicateResponse(opcode, 0x01);
            break;
          }

          simMode = false;
          baseResistance = lvl;
          baseResF = (float)baseResistance;
          systemStatus = "FTMS: manual base";
          ctrlPointIndicateResponse(opcode, 0x01);
        } else {
          ctrlPointIndicateResponse(opcode, 0x04); // invalid parameter
        }
        break;

      case 0x11: { // Indoor Bike Simulation Parameters (grade)
        if (v.size() >= 7) {
          int16_t gr = (int16_t)((uint8_t)v[3] | ((uint8_t)v[4] << 8));

          // Grade scaling correction (proven in your testing)
          grade_pct = (gr / 100.0f) * 2.0f;

          targetBaseResistance = gradeToResistance(grade_pct);

          // On transition into SIM, start slew from current base resistance
          if (!simMode) baseResF = (float)baseResistance;

          simMode = true;
          lastSimMs = millis();
          systemStatus = "FTMS: sim grade";
          ctrlPointIndicateResponse(opcode, 0x01);
        } else {
          ctrlPointIndicateResponse(opcode, 0x04);
        }
        break;
      }

      default:
        ctrlPointIndicateResponse(opcode, 0x02); // opcode not supported
        break;
    }
  }
};

static void notifyIndoorBikeData() {
  if (!bleConnected) return;

  // Include: speed (uint16 0.01 km/h), cadence (uint16 0.5 rpm), power (int16)
  uint16_t flags = 0;
  flags |= (1 << 2); // cadence present
  flags |= (1 << 6); // power present

  uint8_t p[2 + 2 + 2 + 2];
  size_t o = 0;

  p[o++] = (uint8_t)(flags & 0xFF);
  p[o++] = (uint8_t)((flags >> 8) & 0xFF);

  uint16_t sp = speed_x100_kph;
  p[o++] = (uint8_t)(sp & 0xFF);
  p[o++] = (uint8_t)((sp >> 8) & 0xFF);

  uint16_t cad = (uint16_t)(cadence_rpm * 2); // 0.5 rpm units
  p[o++] = (uint8_t)(cad & 0xFF);
  p[o++] = (uint8_t)((cad >> 8) & 0xFF);

  int16_t pw = power_w;
  p[o++] = (uint8_t)(pw & 0xFF);
  p[o++] = (uint8_t)((pw >> 8) & 0xFF);

  chBikeData->setValue(p, o);
  chBikeData->notify();
}

// =======================================================
// LVGL UI (ProForm-style)
// =======================================================

#if LV_FONT_MONTSERRAT_64
extern const lv_font_t lv_font_montserrat_64;
#define FONT_NUM  (&lv_font_montserrat_64)
#elif LV_FONT_MONTSERRAT_48
extern const lv_font_t lv_font_montserrat_48;
#define FONT_NUM  (&lv_font_montserrat_48)
#elif LV_FONT_MONTSERRAT_36
extern const lv_font_t lv_font_montserrat_36;
#define FONT_NUM  (&lv_font_montserrat_36)
#elif LV_FONT_MONTSERRAT_28
extern const lv_font_t lv_font_montserrat_28;
#define FONT_NUM  (&lv_font_montserrat_28)
#else
#define FONT_NUM  LV_FONT_DEFAULT
#endif

#if LV_FONT_MONTSERRAT_28
extern const lv_font_t lv_font_montserrat_28;
#define FONT_LABEL (&lv_font_montserrat_28)
#elif LV_FONT_MONTSERRAT_20
extern const lv_font_t lv_font_montserrat_20;
#define FONT_LABEL (&lv_font_montserrat_20)
#else
#define FONT_LABEL LV_FONT_DEFAULT
#endif

#if LV_FONT_MONTSERRAT_20
extern const lv_font_t lv_font_montserrat_20;
#define FONT_SM   (&lv_font_montserrat_20)
#else
#define FONT_SM   LV_FONT_DEFAULT
#endif

static lv_obj_t* lblStatus = nullptr;

// Top tiles (4 equal boxes)
static lv_obj_t* tileGear = nullptr;
static lv_obj_t* tileDist = nullptr;
static lv_obj_t* tileTime = nullptr;
static lv_obj_t* tileSpeed = nullptr;

static lv_obj_t* lblDistVal = nullptr;
static lv_obj_t* lblDistUnit = nullptr;

static lv_obj_t* lblTimeVal = nullptr;
static lv_obj_t* lblTimeUnit = nullptr;

static lv_obj_t* lblGearVal = nullptr;
static lv_obj_t* lblGearUnit = nullptr;

static lv_obj_t* lblSpeedVal = nullptr;
static lv_obj_t* lblSpeedUnit = nullptr;

// Gauges
static lv_obj_t* gSpeed = nullptr;
static lv_obj_t* gCad = nullptr;
static lv_obj_t* gWatts = nullptr;
static lv_obj_t* gGrade = nullptr;

static lv_obj_t* gLblSpeedVal = nullptr;
static lv_obj_t* gLblCadVal = nullptr;
static lv_obj_t* gLblWattsVal = nullptr;
static lv_obj_t* gLblGradeVal = nullptr;

static lv_meter_indicator_t* indSpeed = nullptr;
static lv_meter_indicator_t* indCad = nullptr;
static lv_meter_indicator_t* indWatts = nullptr;
static lv_meter_indicator_t* indGrade = nullptr;

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void fmtElapsed(char* out, size_t out_sz, uint32_t sec) {
  uint32_t h = sec / 3600U;
  uint32_t m = (sec % 3600U) / 60U;
  uint32_t s = sec % 60U;
  if (h > 0) snprintf(out, out_sz, "%u:%02u:%02u", (unsigned)h, (unsigned)m, (unsigned)s);
  else       snprintf(out, out_sz, "%u:%02u", (unsigned)m, (unsigned)s);
}

static lv_obj_t* createTile(lv_obj_t* parent, lv_color_t bg, int w, int h) {
  lv_obj_t* t = lv_obj_create(parent);
  lv_obj_set_size(t, w, h);
  lv_obj_set_style_bg_color(t, bg, 0);
  lv_obj_set_style_border_width(t, 0, 0);
  lv_obj_set_style_outline_width(t, 0, 0);
  lv_obj_set_style_shadow_width(t, 0, 0);
  lv_obj_set_style_radius(t, 0, 0);
  lv_obj_set_style_pad_all(t, 10, 0);
  lv_obj_clear_flag(t, LV_OBJ_FLAG_SCROLLABLE);
  return t;
}

static lv_obj_t* createGauge(lv_obj_t* parent,
                            const char* unit,
                            int32_t vmin, int32_t vmax,
                            lv_color_t arcColor,
                            lv_meter_indicator_t** outInd,
                            lv_obj_t** outValLbl) {
  lv_obj_t* cont = lv_obj_create(parent);
  lv_obj_set_size(cont, 180, 220);
  lv_obj_set_style_radius(cont, 90, 0);
  lv_obj_set_style_border_width(cont, 0, 0);
  lv_obj_set_style_outline_width(cont, 0, 0);
  lv_obj_set_style_shadow_width(cont, 0, 0);
  lv_obj_set_style_pad_all(cont, 10, 0);
  lv_obj_set_style_bg_color(cont, lv_color_hex(0xD4D7DE), 0); // light grey
  lv_obj_set_style_bg_opa(cont, LV_OPA_COVER, 0);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* m = lv_meter_create(cont);
  lv_obj_set_size(m, 170, 170);
  lv_obj_align(m, LV_ALIGN_TOP_MID, 0, -2);
  lv_obj_set_style_bg_color(m, lv_color_hex(0x111111), 0); // dark face
  lv_obj_set_style_bg_opa(m, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(m, 0, 0);
  lv_obj_set_style_outline_width(m, 0, 0);
  lv_obj_set_style_shadow_width(m, 0, 0);
  lv_obj_clear_flag(m, LV_OBJ_FLAG_SCROLLABLE);

  lv_meter_scale_t* sc = lv_meter_add_scale(m);
  lv_meter_set_scale_range(m, sc, vmin, vmax, 180, 180); // top semi-arc
  lv_meter_set_scale_ticks(m, sc, 0, 0, 0, lv_color_black()); // hide ticks

  // Background arc
  lv_meter_indicator_t* bg = lv_meter_add_arc(m, sc, 12, lv_color_hex(0x3A3A3A), 0);
  lv_meter_set_indicator_start_value(m, bg, vmin);
  lv_meter_set_indicator_end_value(m, bg, vmax);

  // Value arc (fill)
  lv_meter_indicator_t* fg = lv_meter_add_arc(m, sc, 12, arcColor, 0);
  lv_meter_set_indicator_start_value(m, fg, vmin);
  lv_meter_set_indicator_end_value(m, fg, vmin);
  *outInd = fg;

  // Value label
  lv_obj_t* v = lv_label_create(cont);
  lv_obj_set_style_text_font(v, FONT_NUM, 0);
  lv_obj_set_style_text_color(v, lv_color_white(), 0);
  lv_label_set_text(v, "0");
  lv_obj_align(v, LV_ALIGN_CENTER, 0, 10);
  *outValLbl = v;

  // Unit label (bottom)
  lv_obj_t* u = lv_label_create(cont);
  lv_obj_set_style_text_font(u, FONT_SM, 0);
  lv_obj_set_style_text_color(u, lv_color_hex(0x2A2A2A), 0);
  lv_label_set_text(u, unit);
  lv_obj_align(u, LV_ALIGN_BOTTOM_MID, 0, -8);

  return m; // meter object
}

static void uiCreate() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_set_style_pad_all(scr, 0, 0);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x6FA9D6), 0); // blue-ish
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(scr, 0, 0);
  lv_obj_set_style_outline_width(scr, 0, 0);
  lv_obj_set_style_shadow_width(scr, 0, 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

  // Top bar container
  lv_obj_t* top = lv_obj_create(scr);
  lv_obj_set_size(top, lv_pct(100), 118);
  lv_obj_align(top, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(top, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(top, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(top, 0, 0);
  lv_obj_set_style_outline_width(top, 0, 0);
  lv_obj_set_style_shadow_width(top, 0, 0);
  lv_obj_set_style_pad_all(top, 0, 0);
  lv_obj_set_style_pad_column(top, 0, 0);
  lv_obj_set_style_pad_row(top, 0, 0);
  lv_obj_clear_flag(top, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(top, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(top, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  // 4 equal tiles, 200px each (800px total)
  tileGear  = createTile(top, lv_color_hex(0x3B3B3B), 200, 118);
  tileDist  = createTile(top, lv_color_hex(0xD13B36), 200, 118);
  tileTime  = createTile(top, lv_color_hex(0xD13B36), 200, 118);
  tileSpeed = createTile(top, lv_color_hex(0x3B3B3B), 200, 118);

  // Gear tile
  lblGearUnit = lv_label_create(tileGear);
  lv_obj_set_style_text_font(lblGearUnit, FONT_LABEL, 0);
  lv_obj_set_style_text_color(lblGearUnit, lv_color_white(), 0);
  lv_label_set_text(lblGearUnit, "Gear");
  lv_obj_align(lblGearUnit, LV_ALIGN_TOP_LEFT, 0, -2);

  lblGearVal = lv_label_create(tileGear);
  lv_obj_set_style_text_font(lblGearVal, FONT_NUM, 0);
  lv_obj_set_style_text_color(lblGearVal, lv_color_white(), 0);
  lv_label_set_text(lblGearVal, "1");
  lv_obj_align(lblGearVal, LV_ALIGN_BOTTOM_LEFT, 0, -10);

  // Distance tile
  lblDistUnit = lv_label_create(tileDist);
  lv_obj_set_style_text_font(lblDistUnit, FONT_LABEL, 0);
  lv_obj_set_style_text_color(lblDistUnit, lv_color_white(), 0);
  lv_label_set_text(lblDistUnit, "Distance");
  lv_obj_align(lblDistUnit, LV_ALIGN_TOP_LEFT, 0, -2);

  lblDistVal = lv_label_create(tileDist);
  lv_obj_set_style_text_font(lblDistVal, FONT_NUM, 0);
  lv_obj_set_style_text_color(lblDistVal, lv_color_white(), 0);
  lv_label_set_text(lblDistVal, "0.00");
  lv_obj_align(lblDistVal, LV_ALIGN_BOTTOM_LEFT, 0, -10);

  // Time tile
  lblTimeUnit = lv_label_create(tileTime);
  lv_obj_set_style_text_font(lblTimeUnit, FONT_LABEL, 0);
  lv_obj_set_style_text_color(lblTimeUnit, lv_color_white(), 0);
  lv_label_set_text(lblTimeUnit, "Session time");
  lv_obj_align(lblTimeUnit, LV_ALIGN_TOP_LEFT, 0, -2);

  lblTimeVal = lv_label_create(tileTime);
  lv_obj_set_style_text_font(lblTimeVal, FONT_NUM, 0);
  lv_obj_set_style_text_color(lblTimeVal, lv_color_white(), 0);
  lv_label_set_text(lblTimeVal, "0:00");
  lv_obj_align(lblTimeVal, LV_ALIGN_BOTTOM_LEFT, 0, -10);

  // Speed tile
  lblSpeedUnit = lv_label_create(tileSpeed);
  lv_obj_set_style_text_font(lblSpeedUnit, FONT_LABEL, 0);
  lv_obj_set_style_text_color(lblSpeedUnit, lv_color_white(), 0);
  lv_label_set_text(lblSpeedUnit, "Speed");
  lv_obj_align(lblSpeedUnit, LV_ALIGN_TOP_LEFT, 0, -2);

  lblSpeedVal = lv_label_create(tileSpeed);
  lv_obj_set_style_text_font(lblSpeedVal, FONT_NUM, 0);
  lv_obj_set_style_text_color(lblSpeedVal, lv_color_white(), 0);
  lv_label_set_text(lblSpeedVal, "0.0");
  lv_obj_align(lblSpeedVal, LV_ALIGN_BOTTOM_LEFT, 0, -10);

  // Gauges row (middle)
  lv_obj_t* gRow = lv_obj_create(scr);
  lv_obj_set_size(gRow, lv_pct(100), 260);
  lv_obj_align(gRow, LV_ALIGN_CENTER, 0, 44);
  lv_obj_set_style_bg_opa(gRow, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(gRow, 0, 0);
  lv_obj_set_style_outline_width(gRow, 0, 0);
  lv_obj_set_style_shadow_width(gRow, 0, 0);
  lv_obj_set_style_pad_all(gRow, 12, 0);
  lv_obj_clear_flag(gRow, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_flex_flow(gRow, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(gRow, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  gSpeed = createGauge(gRow, "Speed",      0, 40,   lv_palette_main(LV_PALETTE_PURPLE), &indSpeed, &gLblSpeedVal);
  gCad   = createGauge(gRow, "cadence",    0, 140,  lv_palette_main(LV_PALETTE_CYAN),   &indCad,   &gLblCadVal);
  gWatts = createGauge(gRow, "watts",      0, 1000, lv_palette_main(LV_PALETTE_ORANGE), &indWatts, &gLblWattsVal);
  gGrade = createGauge(gRow, "grade (%)", -10, 20,  lv_palette_main(LV_PALETTE_GREEN),  &indGrade, &gLblGradeVal);

  // Bottom status (no divider/lines)
  lblStatus = lv_label_create(scr);
  lv_obj_set_style_text_font(lblStatus, FONT_SM, 0);
  lv_obj_set_style_text_color(lblStatus, lv_color_white(), 0);
  lv_label_set_text(lblStatus, "Status: booting");
  lv_obj_align(lblStatus, LV_ALIGN_BOTTOM_LEFT, 10, -8);
}

static void uiUpdate() {
  // Top values
  char distBuf[24];
  snprintf(distBuf, sizeof(distBuf), "%.2f", (double)distance_miles);
  lv_label_set_text(lblDistVal, distBuf);

  char tBuf[24];
  fmtElapsed(tBuf, sizeof(tBuf), elapsedSeconds());
  lv_label_set_text(lblTimeVal, tBuf);

  char spBuf[24];
  snprintf(spBuf, sizeof(spBuf), "%.1f", (double)speed_mph);
  lv_label_set_text(lblSpeedVal, spBuf);

  char gBuf[24];
  snprintf(gBuf, sizeof(gBuf), "%d", gearIndex + 1);
  lv_label_set_text(lblGearVal, gBuf);

  // Gauges
  {
    int32_t v = (int32_t)lroundf(speed_mph);
    v = clamp_i32(v, 0, 40);
    lv_meter_set_indicator_end_value(gSpeed, indSpeed, v);
    char b[16];
    snprintf(b, sizeof(b), "%.1f", (double)speed_mph);
    lv_label_set_text(gLblSpeedVal, b);
  }

  {
    int32_t v = (int32_t)cadence_rpm;
    v = clamp_i32(v, 0, 140);
    lv_meter_set_indicator_end_value(gCad, indCad, v);
    char b[16];
    snprintf(b, sizeof(b), "%u", (unsigned)cadence_rpm);
    lv_label_set_text(gLblCadVal, b);
  }

  {
    int32_t v = (int32_t)power_w;
    v = clamp_i32(v, 0, 1000);
    lv_meter_set_indicator_end_value(gWatts, indWatts, v);
    char b[16];
    snprintf(b, sizeof(b), "%d", (int)power_w);
    lv_label_set_text(gLblWattsVal, b);
  }

  // Grade gauge: display-only x2 (no calculation changes elsewhere)
  {
    float grade_disp = grade_pct * 2.0f;
    int32_t v = (int32_t)lroundf(grade_disp);
    v = clamp_i32(v, -10, 20);
    lv_meter_set_indicator_end_value(gGrade, indGrade, v);
    char b[16];
    snprintf(b, sizeof(b), "%.1f", (double)grade_disp);
    lv_label_set_text(gLblGradeVal, b);
  }

  // Status line: enforce booting -> BLE advertising -> BLE connected progression
  String shown = systemStatus;
  if (!bleConnected) {
    if (shown != "booting" && !shown.endsWith("FAIL")) {
      shown = "BLE advertising";
    }
  }

  String s = "Status: " + shown;
  lv_label_set_text(lblStatus, s.c_str());
}

// =======================================================
// Stream-based RS-485 frame parser
// =======================================================

static uint8_t rxFrame[64];
static size_t rxLen = 0;
static bool rxInFrame = false;

static void handleStatusFrame(const uint8_t* f, size_t n) {
  lastStatusFrameMs = millis();

  uint16_t rpm_raw = decodeCadence(f, n);

  // Zero timeout based on observed non-zero cadence reception
  if (rpm_raw > 0) lastNonZeroCadMs = millis();
  if (millis() - lastNonZeroCadMs > CAD_ZERO_TIMEOUT_MS) rpm_raw = 0;

  // Single-channel EMA for cadence
  float alpha = (rpm_raw > cad_ema) ? CAD_ALPHA_UP : CAD_ALPHA_DOWN;
  cad_ema = cad_ema + alpha * ((float)rpm_raw - cad_ema);
  if (cad_ema < 2.0f) cad_ema = 0.0f;
  cadence_rpm = (uint16_t)lroundf(cad_ema);

  power_w = estimatePower(cadence_rpm, resistance);
}

static void processIncomingByte(uint8_t b) {
  if (!rxInFrame) {
    if (b == 0xBA) {
      rxInFrame = true;
      rxLen = 0;
      rxFrame[rxLen++] = b;
    }
    return;
  }

  // In-frame accumulation
  if (rxLen < sizeof(rxFrame)) {
    rxFrame[rxLen++] = b;
  } else {
    // Overflow protection
    rxInFrame = false;
    rxLen = 0;
    return;
  }

  // End marker detection
  if (rxLen >= 2 && rxFrame[rxLen - 2] == 0x8D && rxFrame[rxLen - 1] == 0x8A) {
    rxInFrame = false;

    if (looksLikeStatusFrame(rxFrame, rxLen)) {
      handleStatusFrame(rxFrame, rxLen);
    }

    rxLen = 0;
  }
}

// =======================================================
// Buttons handling
// =======================================================

static void handleButtons() {
  int upEv = btnUp.update(500);
  int dnEv = btnDn.update(500);

  // Short press: shift gear
  if (upEv == 1) {
    if (gearIndex < NUM_GEARS - 1) gearIndex++;
    systemStatus = "Shift UP";
  }
  if (dnEv == 1) {
    if (gearIndex > 0) gearIndex--;
    systemStatus = "Shift DOWN";
  }

  // Long press: base resistance trim (offline/manual)
  if (upEv == 2) {
    if (baseResistance < 80) baseResistance++;
    simMode = false;
    baseResF = (float)baseResistance;
    systemStatus = "BaseRes +";
  }
  if (dnEv == 2) {
    if (baseResistance > 0) baseResistance--;
    simMode = false;
    baseResF = (float)baseResistance;
    systemStatus = "BaseRes -";
  }
}

// =======================================================
// Setup / Loop
// =======================================================

static esp_panel::board::Board* g_board = nullptr;

void setup() {
  Serial.begin(115200);
  delay(200);

  systemStatus = "booting";

  // Buttons
  btnUp.begin(BTN_GEAR_UP);
  btnDn.begin(BTN_GEAR_DOWN);

  // RS-485 UART
  BikeSerial.setRxBufferSize(2048);
  BikeSerial.begin(38400, SERIAL_8N1, RS485_RX, RS485_TX);

  // Resistance lookup tables
  InitializeGears();

  // Display init
  g_board = new esp_panel::board::Board();
  if (!g_board->init() || !g_board->begin()) {
    systemStatus = "panel FAIL";
    while (true) delay(1000);
  }

  auto* lcd = g_board->getLCD();
  auto* tp  = g_board->getTouch();

  if (!lvgl_port_init(lcd, tp)) {
    systemStatus = "lvgl FAIL";
    while (true) delay(1000);
  }

  lvgl_port_lock(-1);
  uiCreate();
  lvgl_port_unlock();

  // BLE FTMS init
  NimBLEDevice::init("Proform TDF v2");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  srv = NimBLEDevice::createServer();
  srv->setCallbacks(new ServerCallbacks());

  NimBLEService* ftms = srv->createService(UUID_FTMS_SVC);

  chFeature = ftms->createCharacteristic(UUID_FTMS_FEATURE, NIMBLE_PROPERTY::READ);
  {
    // Non-zero placeholder feature set (keeps clients satisfied)
    uint8_t feat[8] = { 0x44, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00 };
    chFeature->setValue(feat, sizeof(feat));
  }

  chBikeData = ftms->createCharacteristic(UUID_INDOOR_BIKE_DATA, NIMBLE_PROPERTY::NOTIFY);

  chCtrlPt = ftms->createCharacteristic(UUID_CONTROL_POINT,
                                        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE);
  chCtrlPt->setCallbacks(new CtrlPointCallbacks());

  chStatus = ftms->createCharacteristic(UUID_STATUS, NIMBLE_PROPERTY::NOTIFY);

  ftms->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(UUID_FTMS_SVC);
  adv->setScanResponse(true);
  adv->start();

  baseResF = (float)baseResistance;
  targetBaseResistance = baseResistance;

  systemStatus = "BLE advertising";
}

void loop() {
  // Buttons
  handleButtons();

  // SIM slew update (ramps baseResistance toward targetBaseResistance)
  updateBaseResistanceSlew();

  // Effective resistance after gearing
  resistance = applyGearing(baseResistance, gearIndex);

  // Apply resistance to bike (rate limited)
  static uint8_t lastResSent = 255;
  static uint32_t lastResMs = 0;
  if (resistance != lastResSent && millis() - lastResMs > 250) {
    lastResMs = millis();
    lastResSent = resistance;
    systemStatus = "send resistance";
    sendResistance(resistance);
    power_w = estimatePower(cadence_rpm, resistance);
  }

  // Drain RS-485 UART continuously (stream parser)
  while (BikeSerial.available()) {
    processIncomingByte((uint8_t)BikeSerial.read());
  }

  // Periodic status poll (stimulates traffic; parser handles responses asynchronously)
  static uint32_t lastPoll = 0;
  if (millis() - lastPoll >= 180) {
    lastPoll = millis();
    if (millis() - lastRs485TxMs >= RS485_TX_GUARD_MS) {
      BikeSerial.write(POLL_FRAME, sizeof(POLL_FRAME));
      BikeSerial.flush();
    }
  }

  // If status frames stop arriving, decay cadence quickly to avoid stale display/power
  if (lastStatusFrameMs != 0 && (millis() - lastStatusFrameMs) > STATUS_FRAME_TIMEOUT_MS) {
    cad_ema *= 0.55f;
    if (cad_ema < 2.0f) cad_ema = 0.0f;
    cadence_rpm = (uint16_t)lroundf(cad_ema);
    power_w = estimatePower(cadence_rpm, resistance);
  }

  // Update derived speed/distance/elapsed (cadence + gearRatio -> wheel RPM -> speed + distance)
  updateSpeedDistanceElapsed();

  // BLE notify
  static uint32_t lastBle = 0;
  if (bleConnected && millis() - lastBle >= 250) {
    lastBle = millis();
    notifyIndoorBikeData();
  }

  // UI update
  static uint32_t lastUi = 0;
  if (millis() - lastUi >= 250) {
    lastUi = millis();
    if (lvgl_port_lock(0)) {
      uiUpdate();
      lvgl_port_unlock();
    }
  }

  delay(2);
}
