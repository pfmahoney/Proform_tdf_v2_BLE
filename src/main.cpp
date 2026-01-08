// main.cpp
// ProForm TDF v2 Controller
//
// Features:
// - RS-485 (UART1) interface to ProForm lower control board (status/cadence + resistance control)
// - BLE FTMS server (Zwift-compatible) using NimBLE
// - LVGL UI on Waveshare ESP32-S3 Touch LCD 7"
// - Virtual rear gearing (14 gears) + manual base resistance trim (offline capable)
// - Stream-based RS-485 frame parser for low-latency cadence updates
// - FrameAge display to confirm status-frame freshness
//
// Button behavior (active-low with INPUT_PULLUP):
// - Short press UP/DOWN: shift gear up/down
// - Long press UP/DOWN: increase/decrease base resistance (manual/offline trim)
//
// Notes:
// - A buzzer is not confirmed to exist on all Waveshare variants. An external passive piezo on BUZZER_PIN to GND
//   is recommended if no onboard buzzer is present.

#include <Arduino.h>
#include <NimBLEDevice.h>

#include <lvgl.h>
#include "esp_panel_board_supported_conf.h"
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

// External buzzer / piezo (recommended)
static const int BUZZER_PIN = 6;

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
static volatile int gearIndex = 6;  // 0..13 (default ~middle)

static const float gearRatio[NUM_GEARS] = {
  0.35f, 0.42f, 0.50f, 0.60f, 0.72f, 0.85f,
  1.00f, 1.15f, 1.32f, 1.52f, 1.75f, 2.00f,
  2.25f, 2.50f
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
// Buzzer (Arduino-ESP32 core 3.x LEDC API)
// =======================================================

static bool beepActive = false;
static uint32_t beepEndMs = 0;

static void buzzerInit() {
  pinMode(BUZZER_PIN, OUTPUT);
  ledcAttach(BUZZER_PIN, 2000 /*Hz*/, 8 /*bits*/);
  ledcWrite(BUZZER_PIN, 0);
  ledcWriteTone(BUZZER_PIN, 0);
}

static void startBeep(uint16_t freqHz = 2200, uint16_t ms = 45) {
  ledcWriteTone(BUZZER_PIN, freqHz);
  ledcWrite(BUZZER_PIN, 128);
  beepActive = true;
  beepEndMs = millis() + ms;
}

static void buzzerUpdate() {
  if (beepActive && (int32_t)(millis() - beepEndMs) >= 0) {
    ledcWrite(BUZZER_PIN, 0);
    ledcWriteTone(BUZZER_PIN, 0);
    beepActive = false;
  }
}

// =======================================================
// Cadence + FrameAge
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
static volatile uint16_t speed_x100_kph = 0; // placeholder

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
// BLE FTMS (Zwift)
// =======================================================

static float grade_pct = 0.0f;
static bool bleConnected = false;
static String systemStatus = "booting";

static bool simMode = false;
static uint32_t lastSimMs = 0;

static uint8_t gradeToResistance(float gradePercent) {
  // clamp -5..15% to 0..80
  float g = gradePercent;
  if (g < -5) g = -5;
  if (g > 15) g = 15;
  float t = (g + 5.0f) / 20.0f;  // 0..1
  int r = (int)lroundf(t * 80.0f);
  if (r < 0) r = 0;
  if (r > 80) r = 80;
  return (uint8_t)r;
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
          systemStatus = "FTMS: manual base";
          ctrlPointIndicateResponse(opcode, 0x01);
        } else {
          ctrlPointIndicateResponse(opcode, 0x04); // invalid parameter
        }
        break;

      case 0x11: { // Indoor Bike Simulation Parameters (grade)
        // v layout (little endian):
        // [1..2] wind speed (0.001 m/s)  int16
        // [3..4] grade (0.01 %)         int16
        // [5]    crr (0.0001)          uint8
        // [6]    cw  (0.01 kg/m)       uint8
        if (v.size() >= 7) {
          int16_t gr = (int16_t)((uint8_t)v[3] | ((uint8_t)v[4] << 8));

          // Grade scaling correction (proven in your testing)
          grade_pct = (gr / 100.0f) * 2.0f;

          baseResistance = gradeToResistance(grade_pct);
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
// LVGL UI
// =======================================================

static lv_obj_t* lblMain = nullptr;
static lv_obj_t* lblStatus = nullptr;

static void uiCreate() {
  lv_obj_t* scr = lv_scr_act();
  lv_obj_set_style_pad_all(scr, 14, 0);

  lblMain = lv_label_create(scr);
  lv_obj_align(lblMain, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_label_set_text(lblMain, "Starting...");

  lblStatus = lv_label_create(scr);
  lv_obj_align(lblStatus, LV_ALIGN_BOTTOM_LEFT, 0, 0);
  lv_label_set_text(lblStatus, "Status: booting");
}

static void uiUpdate() {
  const uint32_t now = millis();
  const uint32_t frameAge = (lastStatusFrameMs == 0) ? 0 : (now - lastStatusFrameMs);

  char a[260];
  snprintf(a, sizeof(a),
           "Cadence: %u rpm\n"
           "Grade: %.1f%%\n"
           "BaseRes: %u  Gear: %d/%d (x%.2f)\n"
           "EffRes:  %u\n"
           "Power:   %d W\n"
           "BLE: %s  Mode: %s\n"
           "FrameAge: %ums",
           (unsigned)cadence_rpm,
           grade_pct,
           (unsigned)baseResistance,
           gearIndex + 1, NUM_GEARS, gearRatio[gearIndex],
           (unsigned)resistance,
           (int)power_w,
           bleConnected ? "conn" : "adv",
           simMode ? "SIM" : "MAN/OFF",
           (unsigned)frameAge);

  lv_label_set_text(lblMain, a);

  String s = "Status: " + systemStatus;
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
    startBeep(2600, 45);
  }
  if (dnEv == 1) {
    if (gearIndex > 0) gearIndex--;
    systemStatus = "Shift DOWN";
    startBeep(1800, 45);
  }

  // Long press: base resistance trim (offline/manual)
  if (upEv == 2) {
    if (baseResistance < 80) baseResistance++;
    simMode = false;
    systemStatus = "BaseRes +";
    startBeep(3000, 35);
  }
  if (dnEv == 2) {
    if (baseResistance > 0) baseResistance--;
    simMode = false;
    systemStatus = "BaseRes -";
    startBeep(1400, 35);
  }
}

// =======================================================
// Setup / Loop
// =======================================================

static esp_panel::board::Board* g_board = nullptr;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Buttons and buzzer
  btnUp.begin(BTN_GEAR_UP);
  btnDn.begin(BTN_GEAR_DOWN);
  buzzerInit();

  // RS-485 UART
  BikeSerial.setRxBufferSize(2048);
  BikeSerial.begin(38400, SERIAL_8N1, RS485_RX, RS485_TX);

  // Resistance lookup tables
  InitializeGears();

  // Display init
  systemStatus = "init panel";
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
  systemStatus = "init BLE";
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

  systemStatus = "ready (adv)";
}

void loop() {
  // Buttons + buzzer
  handleButtons();
  buzzerUpdate();

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
