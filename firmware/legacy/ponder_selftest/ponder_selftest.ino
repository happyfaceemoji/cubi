// CUBI - bench + final self-test for the XIAO ESP32S3 Sense
// -------------------------------------------------------------------
// Flash this before anything goes in the shell (build plan phase 4) and again after closing it (phase 8).
// Arduino IDE: board "XIAO_ESP32S3", PSRAM "OPI PSRAM", USB CDC On Boot "Enabled". ESP32 core 3.0.x.
// Library: "VL53L0X" by Pololu (Library Manager).
// Open Serial Monitor at 115200. It tests every part, prints PASS/FAIL, then takes single-letter commands.
//
// Pin map (matches the wiring view and PRINT_NOTES.txt):
//   D0  GPIO1  touch pad            D4 GPIO5  SDA (VL53L0X)      D6  GPIO43 amp BCLK
//   D1  GPIO2  servo FL             D5 GPIO6  SCL (VL53L0X)      D7  GPIO44 amp LRC
//   D3  GPIO4  servo FR             D2 GPIO3  battery / 2        D10 GPIO9  amp DIN
//   D8  GPIO7  servo RL             camera + PDM mic: inside the Sense board (GPIO 10-18, 38-42, 47, 48)
//   D9  GPIO8  servo RR             SD slot: leave EMPTY (shares D8-D10)
//
// Resource plan (this is what makes everything work at the same time):
//   LEDC  timer 0 / channel 0 = camera clock.  Servos use channels 4-7 (timers 2-3), all 50 Hz.
//   I2S0  = PDM mic (only I2S0 can do PDM on the S3, so the mic is started FIRST).  I2S1 = amp.
//   I2C0  = VL53L0X (Wire).  The camera's SCCB bus uses I2C1 internally.
//   Serial = USB CDC, so GPIO43/44 are free for the amp.
// Power: servos + amp run straight off the switched 1S battery (3.6-4.2 V; GH-S37D is rated 3.6-4.8 V).
//   Below 3.6 V the firmware refuses to walk: charge on the left USB-C with the switch ON.

#include <Wire.h>
#include <VL53L0X.h>
#include <ESP_I2S.h>
#include "esp_camera.h"
#include "esp_system.h"
#include "cubi_gait.h"     // gait + reflex trained in simulation (python cubi_train.py export)

// ---------- pins ----------
const int PIN_TOUCH = 1;
const int PIN_BAT = 3;
const int PIN_SDA = 5, PIN_SCL = 6;
const int PIN_BCLK = 43, PIN_LRC = 44, PIN_DIN = 9;
const int PIN_MIC_CLK = 42, PIN_MIC_DATA = 41;
// horn_deg: where each leg points with its servo centred. Front legs are pressed onto the spline ~30 deg FORWARD,
// rear legs ~30 deg BACK, so the 90 deg servo covers walking (+-12) and folding flat (75). trim_us fine-tunes: send
// 'c' (legs straight down) and adjust until each leg is vertical (1 deg ~ 11 us).
struct Leg { const char *name; int pin; int ch; int trim_us; float horn_deg; };
Leg legs[4] = { {"FL", 2, 4, 0, 30}, {"FR", 4, 5, 0, 30}, {"RL", 7, 6, 0, -30}, {"RR", 8, 7, 0, -30} };

// GH-S37D: sold as 1000-2000 us = 90 deg, 1500 us = centre. MEASURE YOURS with the 'r' command (servo on the bench,
// no leg): the pulse widths where it stops turning and the angle between those two ends. Many micro servos turn
// further than their spec; more travel lets the shutdown tuck fold her legs further up (180 = vertical).
const float SERVO_TRAVEL_DEG = 90.0f;                  // measured end-to-end angle
const int SERVO_MIN_US = 1000, SERVO_MAX_US = 2000;    // measured pulse widths at the two ends
const int SERVO_CENTRE_US = 1500;
const float US_PER_DEG = (SERVO_MAX_US - SERVO_MIN_US) / SERVO_TRAVEL_DEG;
const int GAIT_LIMIT_DEG = 12;        // front and rear legs touch at 17 deg
const float FOLD_DEG = 75.0f;         // lie down: front legs 75 deg forward, rear 75 back -> resting flat on the bottom plate
// shutdown tuck: legs keep folding up beside her, as far as the servo reaches (180 = pointing straight up)
const float TUCK_DEG = fminf(180.0f, 30.0f + SERVO_TRAVEL_DEG / 2);
const float BATT_STOP_V = 3.6f;       // below this: no walking (servo rating + battery health)

VL53L0X tof;
I2SClass mic, amp;
uint16_t floorMm = 0;                 // flat-desk reading, taken at start-up
bool okCam = false, okTof = false, okMic = false, okAmp = false, okServo = false;

// ---------- servos ----------
uint32_t usToDuty(int us) { return (uint32_t)((uint64_t)us * 16384 / 20000); }   // 50 Hz, 14-bit
void servoUs(int i, int us) {
  us = constrain(us + legs[i].trim_us, SERVO_MIN_US, SERVO_MAX_US);
  ledcWrite(legs[i].pin, usToDuty(us));
}
void servoDeg(int i, float deg, bool fold = false) {      // + = leg swings forward (leg angle from vertical)
  if (!fold) deg = constrain(deg, -GAIT_LIMIT_DEG, GAIT_LIMIT_DEG);   // walking/play: legs never meet
  float servo = constrain(deg - legs[i].horn_deg, -SERVO_TRAVEL_DEG / 2, SERVO_TRAVEL_DEG / 2);   // the servo's own travel
  float sign = (i % 2 == 0) ? 1 : -1;  // left and right servos face opposite ways
  servoUs(i, SERVO_CENTRE_US + (int)(sign * servo * US_PER_DEG));
}
void servosOff() { for (int i = 0; i < 4; i++) ledcWrite(legs[i].pin, 0); }   // no pulses = limp, saves power

bool initServos() {
  bool ok = true;
  for (int i = 0; i < 4; i++) ok &= ledcAttachChannel(legs[i].pin, 50, 14, legs[i].ch);
  for (int i = 0; i < 4; i++) servoDeg(i, 0);            // legs straight down (NOT servo centre: horns are offset)
  return ok;
}

// ---------- camera (XIAO ESP32S3 Sense pinout) ----------
bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = 15; c.pin_d1 = 17; c.pin_d2 = 18; c.pin_d3 = 16; c.pin_d4 = 14; c.pin_d5 = 12; c.pin_d6 = 11; c.pin_d7 = 48;
  c.pin_xclk = 10; c.pin_pclk = 13; c.pin_vsync = 38; c.pin_href = 47;
  c.pin_sccb_sda = 40; c.pin_sccb_scl = 39; c.pin_pwdn = -1; c.pin_reset = -1;
  c.xclk_freq_hz = 20000000; c.pixel_format = PIXFORMAT_JPEG; c.frame_size = FRAMESIZE_VGA;
  c.jpeg_quality = 12; c.fb_count = 1; c.fb_location = CAMERA_FB_IN_PSRAM; c.grab_mode = CAMERA_GRAB_LATEST;
  if (!psramFound()) { Serial.println("  ! no PSRAM: set Tools > PSRAM > OPI PSRAM"); return false; }
  if (esp_camera_init(&c) != ESP_OK) return false;
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;
  Serial.printf("  photo %ux%u, %u bytes JPEG\n", fb->width, fb->height, fb->len);
  esp_camera_fb_return(fb);
  return true;
}

// ---------- distance sensor ----------
bool initTof() {
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  tof.setTimeout(100);
  if (!tof.init()) return false;
  tof.setMeasurementTimingBudget(20000);   // 20 ms -> 50 Hz edge reflex
  tof.startContinuous();
  uint32_t sum = 0; int n = 0;
  for (int i = 0; i < 10; i++) { uint16_t d = tof.readRangeContinuousMillimeters(); if (!tof.timeoutOccurred() && d < 2000) { sum += d; n++; } }
  if (n < 5) return false;
  floorMm = sum / n;
  Serial.printf("  desk baseline %u mm (expect ~100-115 mm inside the shell on a flat desk)\n", floorMm);
  return true;
}
// Edge/wall reflex (thresholds trained in simulation): > CUBI_EDGE_RATIO x baseline or no return = edge,
// < CUBI_WALL_RATIO x baseline = obstacle. Run at 50 Hz in firmware, never via the agent.
int tofVerdict(uint16_t d, bool timeout) {
  if (timeout || d >= 8000 || d > floorMm * CUBI_EDGE_RATIO) return 1;   // edge
  if (d < floorMm * CUBI_WALL_RATIO) return 2;                            // wall
  return 0;
}

// ---------- audio ----------
bool initMic() {   // FIRST, so it gets I2S0
  mic.setPinsPdmRx(PIN_MIC_CLK, PIN_MIC_DATA);
  return mic.begin(I2S_MODE_PDM_RX, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
}
int micLevel(int ms) {
  int16_t buf[256]; int peak = 0; uint32_t t0 = millis();
  while (millis() - t0 < (uint32_t)ms) {
    size_t n = mic.readBytes((char *)buf, sizeof(buf)) / 2;
    for (size_t i = 0; i < n; i++) peak = max(peak, abs((int)buf[i]));
  }
  return peak;
}
bool initAmp() {
  amp.setPins(PIN_BCLK, PIN_LRC, PIN_DIN);
  return amp.begin(I2S_MODE_STD, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
}
void tone_(int hz, int ms, float vol = 0.35f) {   // keep vol <= 0.7 (speaker limit)
  int16_t buf[160];
  int total = 16 * ms; float ph = 0, step = 2 * PI * hz / 16000.0f;
  while (total > 0) {
    int n = min(total, 160);
    for (int i = 0; i < n; i++) { buf[i] = (int16_t)(sinf(ph) * 32767 * vol); ph += step; }
    amp.write((uint8_t *)buf, n * 2); total -= n;
  }
}

// ---------- battery + touch ----------
float batteryV() { uint32_t mv = 0; for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_BAT); return mv / 8 * 2 / 1000.0f; }

// ---------- stick-slip walk (trained in the physics sim: cubi_train.py) ----------
#ifndef CUBI_SMOOTH
#define CUBI_SMOOTH 0.0f
#endif
static float cubiEase(float u) {          // straight ramp blended with a minimum-jerk curve: soft reversals
  return (1 - CUBI_SMOOTH) * u + CUBI_SMOOTH * u * u * u * (10 - 15 * u + 6 * u * u);
}
float cubiSaw(float ph) {   // fast swing forward for CUBI_FAST of the cycle, slow push back for the rest
  ph -= floorf(ph);
  return ph < CUBI_FAST ? -1 + 2 * cubiEase(ph / CUBI_FAST) : 1 - 2 * cubiEase((ph - CUBI_FAST) / (1 - CUBI_FAST));
}
// Soft start / stop (sim: start rocking 40 -> 24 deg/s, stop 38 -> 27): stride eases in, and at the end she keeps
// stepping while the stride shrinks to nothing. An edge or wall still stops her instantly.
static const float WALK_EASE_IN = 0.5f, WALK_EASE_OUT = 0.6f;
static float easeStep(float x) { x = constrain(x, 0.0f, 1.0f); return x * x * (3 - 2 * x); }
float batteryV();
void walk(float seconds, int dir) {   // dir +1 forward, -1 back. legs[]: FL, FR, RL, RR. 50 Hz loop, like the sim.
  if (batteryV() < BATT_STOP_V) { Serial.println("  battery below 3.6 V: not walking. Charge me."); return; }
  const float ph[4] = {CUBI_PH_LF, CUBI_PH_RF, CUBI_PH_LB, CUBI_PH_RB};
  const float trim[4] = {CUBI_TRIM_F, CUBI_TRIM_F, CUBI_TRIM_B, CUBI_TRIM_B};
  uint32_t t0 = millis();
  const float total = seconds + WALK_EASE_OUT;
  while (millis() - t0 < (uint32_t)(total * 1000)) {
    uint32_t tick = millis();
    float t = (tick - t0) / 1000.0f;
    float env = easeStep(t / WALK_EASE_IN) * (1 - easeStep((t - seconds) / WALK_EASE_OUT));
    for (int i = 0; i < 4; i++) servoDeg(i, env * (dir * CUBI_AMP_DEG * cubiSaw(CUBI_FREQ_HZ * t + ph[i]) + trim[i]));
    uint16_t d = tof.readRangeContinuousMillimeters();
    int v = tofVerdict(d, tof.timeoutOccurred());
    if (v && dir > 0) { Serial.println(v == 1 ? "  EDGE! backing up" : "  wall - stop"); if (v == 1) walk(CUBI_BACKUP_S, -1); break; }
    while (millis() - tick < 20) delay(1);
  }
  for (int i = 0; i < 4; i++) servoDeg(i, 0);
  delay(300); servosOff();
}

void report(const char *what, bool ok) { Serial.printf("[%s] %s\n", ok ? "PASS" : "FAIL", what); }

#include "cubi_play.h"      // puppy mode: tricks, mood engine, sounds (tested in cubi_play.py)
#include "cubi_mind.h"      // mind mode: traits, drives, learning (tested in cubi_mind.py)

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis(); while (!Serial && millis() - t0 < 4000) delay(10);
  Serial.println("\n=== CUBI SELF-TEST ===");
  esp_reset_reason_t rr = esp_reset_reason();
  if (rr == ESP_RST_BROWNOUT) Serial.println("!!! Last reset was a BROWNOUT: battery low, bulk cap missing/backwards, or a bad ground.");

  float vb = batteryV();
  Serial.printf("  battery %.2f V (D2 divider)\n", vb);
  report("battery sense (3.3-4.25 V; ~0 V = divider or switch)", vb > 3.3f && vb < 4.25f);
  if (vb > 0.5f && vb < BATT_STOP_V) Serial.println("  ! battery low: charge on the left USB-C with the switch ON");

  okMic = initMic();     report("PDM mic on I2S0", okMic);
  okAmp = initAmp();     report("MAX98357A amp on I2S1", okAmp);
  okCam = initCamera();  report("camera (OV2640/OV3660, VGA JPEG)", okCam);
  okTof = initTof();     report("VL53L0X distance sensor", okTof);
  okServo = initServos(); report("4 servo PWM channels (LEDC 4-7)", okServo);

  int idle = touchRead(PIN_TOUCH);
  Serial.printf("  touch idle %d (touch the head: it should change by >20%%)\n", idle);

  if (okAmp) { tone_(880, 120); tone_(1320, 160); }
  if (okMic) { int q = micLevel(300); Serial.printf("  mic quiet level %d - clap in the next 2 s...\n", q);
               int l = micLevel(2000); report("mic hears a clap", l > q * 4 && l > 2000); }

  if (okServo) {
    Serial.println("  servos: each leg wiggles +-10 deg in turn (FL, FR, RL, RR)");
    for (int i = 0; i < 4; i++) { servoDeg(i, 10); delay(250); servoDeg(i, -10); delay(250); servoDeg(i, 0); delay(250); }
  }

  // Everything at once: the worst case for the power supply.
  if (okServo && okCam && okAmp) {
    Serial.println("  STRESS: all legs sweep + photos + sound for 5 s...");
    uint32_t s0 = millis(); int frames = 0;
    while (millis() - s0 < 5000) {
      float a = 12 * sinf((millis() - s0) / 1000.0f * 2 * PI * 1.5f);
      for (int i = 0; i < 4; i++) servoDeg(i, (i == 0 || i == 3) ? a : -a);
      camera_fb_t *fb = esp_camera_fb_get(); if (fb) { frames++; esp_camera_fb_return(fb); }
      tone_(440, 20, 0.2f);
    }
    for (int i = 0; i < 4; i++) servoDeg(i, 0);
    delay(300); servosOff();
    Serial.printf("  %d photos during stress, battery now %.2f V\n", frames, batteryV());
    report("stress test survived without a reset", true);   // a brownout would have rebooted us before this line
  }
  Serial.println("\nCommands: w=walk 3 s  b=walk back  c=centre legs  o=legs off  d=distance  p=photo  t=tone  m=mic  v=battery  h=touch  L=lie down  U=get up  Z=goodnight  r=servo range  P=PUPPY MODE  M=MIND MODE (her own ideas)  I=her personality  W=new personality  G=golden retriever");
}

void loop() {
  if (!Serial.available()) return;
  char k = Serial.read();
  switch (k) {
    case 'w': walk(3, 1); break;
    case 'b': walk(3, -1); break;
    case 'c': for (int i = 0; i < 4; i++) servoDeg(i, 0); Serial.println("  legs centred: any leg not vertical -> set its trim_us"); break;
    case 'o': servosOff(); break;
    case 'd': { uint16_t d = tof.readRangeContinuousMillimeters(); int v = tofVerdict(d, tof.timeoutOccurred());
                Serial.printf("  %u mm  %s\n", d, v == 1 ? "EDGE" : v == 2 ? "WALL" : "desk"); } break;
    case 'p': { camera_fb_t *fb = okCam ? esp_camera_fb_get() : nullptr; if (fb) { Serial.printf("  photo %u bytes\n", fb->len); esp_camera_fb_return(fb); } } break;
    case 't': tone_(660, 300); break;
    case 'm': Serial.printf("  mic peak %d\n", micLevel(500)); break;
    case 'v': Serial.printf("  battery %.2f V\n", batteryV()); break;
    case 'h': Serial.printf("  touch %d\n", touchRead(PIN_TOUCH)); break;
    case 'P': puppyMode(); break;
    case 'M': mindMode(); break;
    case 'I': mindLoad(); mindPrint(); break;
    case 'W': mindLoad(true); mindPrint(); break;
    case 'G': makeGolden(); mindPrint(); break;
    case 'L': playTrick(FLOP); break;
    case 'Z': goodnight(); break;
    case 'r': rangeFinder(); break;
    case 'U': playTrick(UNTUCK); playTrick(GETUP); break;
  }
}
