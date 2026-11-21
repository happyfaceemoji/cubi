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
#include <Preferences.h>
// enums up front: the Arduino IDE auto-generates function prototypes above the first function,
// so any type used in a function signature has to be declared before that.
enum Trick { WIGGLE, PAW_PAT, BOUNCE, CHASE, ZOOMIES, SNIFF, STARTLE, STRETCH, SETTLE, FLOP, GETUP, TUCK, UNTUCK, N_TRICKS };
enum Act { A_WALK, A_TRICK, A_CALL, A_REST, A_IDLE };
enum Choice { C_EXPLORE, C_CALL, C_PLAY, C_REST, C_LOOK, C_IDLE, N_CHOICE };
// ===== inlined cubi_remote.h (phone remote: join Wi-Fi "Cubi", open http://192.168.4.1) =====
// cubi_remote.h — phone remote for Cubi (XIAO ESP32S3, Arduino ESP32 core 3.0.7)
//
// Cubi hosts a small web app. Join her Wi-Fi from your phone and open
// http://192.168.4.1. Each button sends one of the serial command letters.
//
// Hooking it up (3 small edits in cubi_fw.ino):
//   1. Near the top:            #include "cubi_remote.h"
//   2. At the end of setup():   remoteBegin();
//   3. Wherever the sketch reads a command from USB, swap
//        Serial.available()  ->  cmdAvailable()
//        Serial.read()       ->  cmdRead()
//        Serial.peek()       ->  cmdPeek()
//      (include any spot inside mind/puppy/follow mode that checks for 'x' to stop).
//
// The web server runs in its own task, so buttons still get through while
// loop() is busy in a walk or a trick. Commands queue up and the sketch
// picks them up the next time it checks for input.

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// ---- settings -------------------------------------------------------------
// Private Wi-Fi settings: copy cubi_secrets.example.h to cubi_secrets.h beside
// this sketch and fill it in. cubi_secrets.h is git-ignored. Without it the
// defaults below are used (change CUBI_AP_PASS before running her in public).
#if __has_include("cubi_secrets.h")
#include "cubi_secrets.h"
#endif
#ifndef CUBI_AP_SSID
#define CUBI_AP_SSID "Cubi"
#endif
#ifndef CUBI_AP_PASS
#define CUBI_AP_PASS "cubi1234"      // at least 8 characters
#endif
// To join your home Wi-Fi instead, define both of these before the #include.
// If it can't connect within 8 s, she falls back to her own network.
#ifndef CUBI_HOME_SSID
#define CUBI_HOME_SSID ""
#endif
#ifndef CUBI_HOME_PASS
#define CUBI_HOME_PASS ""
#endif
#ifndef CUBI_BATT_PIN
#define CUBI_BATT_PIN D2              // middle of the 2 x 100k divider
#endif

// ---- command queue --------------------------------------------------------
static QueueHandle_t   _cubiQ = nullptr;
static WebServer       _cubiSrv(80);
static char     _cubiLast = 0;
static uint32_t _cubiCount = 0;

inline bool remoteAvailable() {
  return _cubiQ && uxQueueMessagesWaiting(_cubiQ) > 0;
}
inline int remoteRead() {
  char c;
  if (_cubiQ && xQueueReceive(_cubiQ, &c, 0) == pdTRUE) return (uint8_t)c;
  return -1;
}
// Drop-in replacements for Serial.available()/Serial.read(): USB first, then phone.
inline int cmdAvailable() { return Serial.available() + (remoteAvailable() ? 1 : 0); }
inline int cmdRead() {
  if (Serial.available()) return Serial.read();
  return remoteRead();
}
inline int cmdPeek() {
  if (Serial.available()) return Serial.peek();
  char c;
  if (_cubiQ && xQueuePeek(_cubiQ, &c, 0) == pdTRUE) return (uint8_t)c;
  return -1;
}

static float _cubiBattV() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(CUBI_BATT_PIN);
  return (mv / 8.0f) * 2.0f / 1000.0f;
}

// ---- the phone app --------------------------------------------------------
static const char CUBI_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="theme-color" content="#111">
<link rel="manifest" href="/manifest.json">
<title>Cubi</title>
<style>
:root{--bg:#111;--card:#1c1c1e;--line:#2c2c2e;--txt:#f2f2f2;--dim:#8e8e93;--acc:#ff6b4a;--ok:#34c759;--warn:#ff9f0a;--bad:#ff3b30}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;user-select:none;-webkit-user-select:none}
body{margin:0;background:var(--bg);color:var(--txt);font:15px/1.3 -apple-system,system-ui,sans-serif;padding:calc(env(safe-area-inset-top) + 10px) 14px calc(env(safe-area-inset-bottom) + 90px)}
header{display:flex;align-items:center;justify-content:space-between;margin-bottom:12px}
h1{font-size:26px;margin:0;letter-spacing:-.5px}
.pill{display:flex;gap:10px;align-items:center;font-size:13px;color:var(--dim)}
.dot{width:9px;height:9px;border-radius:50%;background:var(--bad);display:inline-block;margin-right:5px}
.dot.on{background:var(--ok)}
h2{font-size:12px;text-transform:uppercase;letter-spacing:1px;color:var(--dim);margin:18px 2px 8px;font-weight:600}
.grid{display:grid;gap:8px}
.g2{grid-template-columns:repeat(2,1fr)}.g3{grid-template-columns:repeat(3,1fr)}.g4{grid-template-columns:repeat(4,1fr)}
button{background:var(--card);color:var(--txt);border:1px solid var(--line);border-radius:14px;padding:14px 6px;font:inherit;font-weight:600;min-height:54px;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:2px;transition:transform .06s,background .06s}
button small{font-weight:400;color:var(--dim);font-size:11px}
button:active,button.hit{transform:scale(.95);background:#2c2c2e}
button.big{min-height:76px;font-size:17px}
button.acc{background:var(--acc);border-color:var(--acc)}
button.acc small{color:#ffe3dc}
.dpad{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
.dpad button{min-height:70px;font-size:22px}
#stop{position:fixed;left:14px;right:14px;bottom:calc(env(safe-area-inset-bottom) + 12px);background:var(--bad);border:0;min-height:62px;font-size:19px;border-radius:18px;box-shadow:0 6px 24px #0008;z-index:5}
#toast{position:fixed;left:50%;top:calc(env(safe-area-inset-top) + 8px);transform:translateX(-50%);background:#2c2c2ecc;backdrop-filter:blur(10px);padding:7px 14px;border-radius:20px;font-size:13px;opacity:0;transition:opacity .2s;pointer-events:none;z-index:9}
#toast.show{opacity:1}
details{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:4px 12px;margin-top:18px}
summary{padding:10px 0;font-weight:600;color:var(--dim);list-style:none}
summary::after{content:" ▸"}details[open] summary::after{content:" ▾"}
details .grid{margin-bottom:12px}
details button{background:#242426}
.row{display:flex;gap:8px;margin-top:8px}
.row input{flex:1;background:#242426;border:1px solid var(--line);color:var(--txt);border-radius:12px;padding:12px;font:inherit;user-select:text;-webkit-user-select:text}
.row button{min-height:0;padding:0 18px}
</style></head><body>
<header><h1>Cubi</h1>
<div class="pill"><span><span class="dot" id="dot"></span><span id="link">…</span></span><span id="batt">– V</span></div></header>

<h2>Mode</h2>
<div class="grid g3">
<button class="big acc" data-k="M">Mind<small>her own way</small></button>
<button class="big" data-k="P">Puppy<small>taps &amp; hands</small></button>
<button class="big" data-k="F">Follow<small>watches you</small></button>
</div>

<h2>Move</h2>
<div class="dpad">
<button data-k="U">Up<small>get up</small></button>
<button data-k="w">▲<small>walk 3 s</small></button>
<button data-k="L">Down<small>lie down</small></button>
<button data-k="c">Centre<small>legs</small></button>
<button data-k="b">▼<small>back 3 s</small></button>
<button data-k="Z">Night<small>goodnight</small></button>
</div>

<h2>Tricks</h2>
<div class="grid g3" id="tricks"></div>
<div class="grid g2" style="margin-top:8px">
<button data-k="G">Golden retriever</button>
<button data-k="I">Personality<small>prints to USB</small></button>
</div>

<h2>Walk straight</h2>
<div class="grid g3">
<button data-k="(">◀ Steer<small>if she drifts right</small></button>
<button data-k="w">Test<small>walk 3 s</small></button>
<button data-k=")">Steer ▶<small>if she drifts left</small></button>
</div>

<details><summary>Walk tuning</summary>
<div class="grid g4">
<button data-k="-">Amp −</button><button data-k="+">Amp +</button>
<button data-k="[">Freq −</button><button data-k="]">Freq +</button>
<button data-k="k">Fast −</button><button data-k="K">Fast +</button>
<button data-k="y">Lean −</button><button data-k="Y">Lean +</button>
</div>
<div class="grid g3">
<button data-k="n">Next gait</button><button data-k="g">Show gait</button><button data-k="T">Auto-tune</button>
</div></details>

<details><summary>Sensors &amp; setup</summary>
<div class="grid g3">
<button data-k="D">Desk baseline</button><button data-k="R">Reflex on/off</button><button data-k="e">Eyes test</button>
<button data-k="d">Distance</button><button data-k="v">Battery</button><button data-k="t">Tone</button>
<button data-k="m">Mic</button><button data-k="h">Touch</button><button data-k="f">Leg dir test</button>
</div>
<div class="grid g2"><button data-k="o">Legs off</button><button data-k="W">New personality<small>forgets learning</small></button></div>
<div class="row"><input id="raw" maxlength="8" placeholder="Any command letters" autocapitalize="off" autocorrect="off"><button id="send">Send</button></div>
</details>

<button id="stop" data-k="x">STOP</button>
<div id="toast"></div>

<script>
const $=s=>document.querySelector(s);
const tricks=$('#tricks');
for(let i=1;i<=9;i++){const b=document.createElement('button');b.dataset.k=String(i);b.textContent='Trick '+i;tricks.appendChild(b);}
const LABEL={W:'New personality',Z:'Goodnight',x:'Stop'};
const CONFIRM={W:'Roll a new personality? She forgets what she learned.',T:'Auto-tune walks her for a while. Clear desk and smooth surface?'};
let tt;function toast(t){const e=$('#toast');e.textContent=t;e.classList.add('show');clearTimeout(tt);tt=setTimeout(()=>e.classList.remove('show'),1100);}
async function send(k,label){
  try{navigator.vibrate&&navigator.vibrate(12);}catch(e){}
  try{const r=await fetch('/k?c='+encodeURIComponent(k),{cache:'no-store'});
    if(!r.ok)throw 0;toast('Sent: '+(label||k));}catch(e){toast('Not connected');setLink(false);}
}
document.body.addEventListener('click',e=>{
  const b=e.target.closest('button[data-k]');if(!b)return;
  const k=b.dataset.k;if(CONFIRM[k]&&!confirm(CONFIRM[k]))return;
  b.classList.add('hit');setTimeout(()=>b.classList.remove('hit'),120);
  send(k,LABEL[k]||b.firstChild.textContent.trim());
});
$('#send').onclick=()=>{const v=$('#raw').value;if(v){send(v,v);$('#raw').value='';}};
function setLink(on,txt){$('#dot').classList.toggle('on',on);$('#link').textContent=on?(txt||'online'):'offline';}
async function poll(){
  try{const r=await fetch('/status',{cache:'no-store'});const s=await r.json();
    setLink(true,s.rssi?(s.rssi+' dBm'):'online');
    const v=s.batt;const e=$('#batt');e.textContent=v.toFixed(2)+' V';
    e.style.color=v<3.6?'var(--bad)':v<3.75?'var(--warn)':'var(--ok)';
  }catch(e){setLink(false);}
}
poll();setInterval(poll,4000);
</script></body></html>)HTML";

static const char CUBI_MANIFEST[] PROGMEM =
  "{\"name\":\"Cubi\",\"short_name\":\"Cubi\",\"start_url\":\"/\",\"display\":\"standalone\","
  "\"background_color\":\"#111111\",\"theme_color\":\"#111111\"}";

static void _cubiRoutes() {
  _cubiSrv.on("/", HTTP_GET, [] {
    _cubiSrv.sendHeader("Cache-Control", "no-cache");
    _cubiSrv.send_P(200, "text/html", CUBI_PAGE);
  });
  _cubiSrv.on("/manifest.json", HTTP_GET, [] {
    _cubiSrv.send_P(200, "application/manifest+json", CUBI_MANIFEST);
  });
  _cubiSrv.on("/k", HTTP_GET, [] {
    String c = _cubiSrv.arg("c");
    if (c.length() == 0 || c.length() > 8) { _cubiSrv.send(400, "text/plain", "bad"); return; }
    for (size_t i = 0; i < c.length(); i++) {
      char ch = c[i];
      xQueueSend(_cubiQ, &ch, 0);          // drop if full rather than block
      _cubiLast = ch; _cubiCount++;
    }
    Serial.printf("[remote] %s\n", c.c_str());
    _cubiSrv.send(200, "text/plain", "ok");
  });
  _cubiSrv.on("/status", HTTP_GET, [] {
    char buf[128];
    int rssi = (WiFi.getMode() & WIFI_STA) && WiFi.isConnected() ? WiFi.RSSI() : 0;
    snprintf(buf, sizeof buf, "{\"batt\":%.2f,\"up\":%lu,\"rssi\":%d,\"cmds\":%lu}",
             _cubiBattV(), (unsigned long)(millis() / 1000), rssi, (unsigned long)_cubiCount);
    _cubiSrv.send(200, "application/json", buf);
  });
  _cubiSrv.onNotFound([] { _cubiSrv.sendHeader("Location", "/"); _cubiSrv.send(302); });
}

static void _cubiTask(void*) {
  for (;;) { _cubiSrv.handleClient(); vTaskDelay(pdMS_TO_TICKS(5)); }
}

inline void remoteBegin() {
  _cubiQ = xQueueCreate(32, sizeof(char));
  bool home = false;
  if (strlen(CUBI_HOME_SSID)) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(CUBI_HOME_SSID, CUBI_HOME_PASS);
    uint32_t t0 = millis();
    while (!WiFi.isConnected() && millis() - t0 < 8000) delay(100);
    home = WiFi.isConnected();
  }
  if (home) {
    MDNS.begin("cubi");
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[remote] on home Wi-Fi: http://cubi.local  or  http://%s\n",
                  WiFi.localIP().toString().c_str());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(CUBI_AP_SSID, CUBI_AP_PASS);
    Serial.printf("[remote] join Wi-Fi \"%s\" (pw %s), open http://%s\n",
                  CUBI_AP_SSID, CUBI_AP_PASS, WiFi.softAPIP().toString().c_str());
  }
  _cubiRoutes();
  _cubiSrv.begin();
  // Core 1 alongside loop(); face detection keeps core 0.
  xTaskCreatePinnedToCore(_cubiTask, "cubi_web", 6144, nullptr, 2, nullptr, 1);
}
// ===== end cubi_remote =====
// ===== inlined cubi_gait.h =====
// Cubi gait + reflex, trained in simulation by cubi_train.py. Re-run `python cubi_train.py export` after training.

float CUBI_AMP_DEG = 12.0f;    // swing amplitude (+-), live-tunable with + / -
float CUBI_FREQ_HZ = 2.2f;    // step cycles per second, live-tunable with [ / ]
float CUBI_STEER = 0.0f;  // walk-straight trim: + = right legs push harder = curves LEFT. ( and ) live, saved in flash
float CUBI_FAST = 0.50f;   // was 0.348 (sim). Even swing walks on paper, 17 Nov auto-tune: +19/+90 mm. k / K live
#define CUBI_PH_LF     0.000f
#define CUBI_PH_LB     0.410f   // leg phase offsets (cycles)
#define CUBI_PH_RF     0.500f
#define CUBI_PH_RB     -0.088f
#define CUBI_TRIM_F    1.593f   // front / back leg centre offsets (deg)
#define CUBI_TRIM_B    -1.634f
#define CUBI_SMOOTH    0.000f   // rounds off leg reversals (0 = sharp sawtooth, 1 = minimum-jerk)
#define CUBI_EDGE_RATIO 1.35f  // ToF > ratio x flat-desk reading = edge
#define CUBI_WALL_RATIO 0.75f
#define CUBI_BACKUP_S   1.50f

// ===== end cubi_gait.h =====


// ---------- pins ----------
const int PIN_TOUCH = 1;
const int PIN_BAT = 3;
const int PIN_SDA = 5, PIN_SCL = 6;
const int PIN_BCLK = 43, PIN_LRC = 44, PIN_DIN = 9;
const int PIN_MIC_CLK = 42, PIN_MIC_DATA = 41;
// horn_deg: where each leg points with its servo centred. Front legs are pressed onto the spline ~60 deg FORWARD,
// rear legs ~60 deg BACK, so the 180 deg servo covers walking (+-12), folding flat (75) and the tuck (150). trim_us fine-tunes: send
// 'c' (legs straight down) and adjust until each leg is vertical (1 deg ~ 12 us).
struct Leg { const char *name; int pin; int ch; int trim_us; float horn_deg; };
const float HORN_OFFSET_DEG = 60.0f;   // 180 deg servos: legs pressed on 60 deg off vertical (front forward, rear back)
Leg legs[4] = { {"FL", 2, 4, 0, HORN_OFFSET_DEG}, {"FR", 4, 5, 0, HORN_OFFSET_DEG}, {"RL", 7, 6, 0, -HORN_OFFSET_DEG}, {"RR", 8, 7, 0, -HORN_OFFSET_DEG} };

// GH-S37D: sold as 1000-2000 us = 90 deg, 1500 us = centre. MEASURE YOURS with the 'r' command (servo on the bench,
// no leg): the pulse widths where it stops turning and the angle between those two ends. Many micro servos turn
// further than their spec; more travel lets the shutdown tuck fold her legs further up (180 = vertical).
const float SERVO_TRAVEL_DEG = 180.0f;                 // measured 16 Nov 2026: all four GH-S37D, 180 deg, no buzz
const int SERVO_MIN_US = 425, SERVO_MAX_US = 2575;     // measured pulse widths at the two ends
const int SERVO_CENTRE_US = 1500;
const float US_PER_DEG = (SERVO_MAX_US - SERVO_MIN_US) / SERVO_TRAVEL_DEG;
float stanceOff = 0;            // deg added to every leg while walking: - = feet behind the hips (raked back), y / Y
int GAIT_LIMIT_DEG = 15;       // front and rear legs touch at 17 deg        // front and rear legs touch at 17 deg
const float FOLD_DEG = 75.0f;         // lie down: front legs 75 deg forward, rear 75 back -> resting flat on the bottom plate
// shutdown tuck: legs keep folding up beside her, as far as the servo reaches (180 = pointing straight up)
const float TUCK_DEG = fminf(180.0f, HORN_OFFSET_DEG + SERVO_TRAVEL_DEG / 2);   // 150: legs fold up to 30 deg off vertical
const float BATT_STOP_V = 3.6f;       // below this: no walking (servo rating + battery health)

VL53L0X tof;
I2SClass mic, amp;
uint16_t floorMm = 0;
const int REFLEX_CONFIRM = 3;          // readings in a row (60 ms, ~2 mm of travel) before the reflex fires
bool reflexOn = true;                  // R toggles the edge/wall reflex during w/b (testing only)                 // flat-desk reading, taken at start-up
bool okCam = false, okTof = false, okMic = false, okAmp = false, okServo = false;

// ---------- servos ----------
uint32_t usToDuty(int us) { return (uint32_t)((uint64_t)us * 16384 / 20000); }   // 50 Hz, 14-bit
void servoUs(int i, int us) {
  us = constrain(us + legs[i].trim_us, SERVO_MIN_US, SERVO_MAX_US);
  ledcWrite(legs[i].pin, usToDuty(us));
}
void servoDeg(int i, float deg, bool fold = false) {      // + = leg swings forward (leg angle from vertical)
  if (!fold) deg = constrain(deg, -GAIT_LIMIT_DEG, GAIT_LIMIT_DEG) + stanceOff;   // walking/play: legs never meet; stanceOff leans ALL legs (gap unchanged)
  float servo = constrain(deg - legs[i].horn_deg, -SERVO_TRAVEL_DEG / 2, SERVO_TRAVEL_DEG / 2);   // the servo's own travel
  float sign = (i % 2 == 0) ? -1 : 1;  // left and right servos face opposite ways (flipped 17 Nov: f test showed all legs reversed)
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
  c.xclk_freq_hz = 20000000; c.pixel_format = PIXFORMAT_RGB565; c.frame_size = FRAMESIZE_QVGA;   // raw pixels for the face detector
  c.jpeg_quality = 12; c.fb_count = 2; c.fb_location = CAMERA_FB_IN_PSRAM; c.grab_mode = CAMERA_GRAB_LATEST;
  if (!psramFound()) { Serial.println("  ! no PSRAM: set Tools > PSRAM > OPI PSRAM"); return false; }
  esp_err_t ce = esp_camera_init(&c);
  if (ce != ESP_OK) { Serial.printf("  ! camera init err 0x%x (0x105/0x20001 = sensor not found: Sense board or ribbon)\n", ce); return false; }
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { Serial.println("  ! camera found but no frame (ribbon data lines / PSRAM)"); return false; }
  Serial.printf("  photo %ux%u, %u bytes RGB565\n", fb->width, fb->height, fb->len);
  esp_camera_fb_return(fb);
  return true;
}


// ---------- eyes: face detection on the chip's second core (cubi_eyes) ----------
// esp-dl two-stage face detector (ships with the ESP32 core 3.0.7 for the S3). Runs on core 0 while
// eyesOn is true; walking, reflexes and serial stay on core 1. Publishes where the biggest face is.
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"
static const float CAM_HFOV_DEG = 60.0f;        // approx horizontal field of view; only scales turn size
static const float FACE_DEADBAND_DEG = 9.0f;    // "looking at you" if the face is within this of centre
volatile bool eyesOn = false;
volatile float faceX = 0, faceW = 0;            // x: -1 = her far left .. +1 = her far right; w = face width / frame width
volatile uint32_t faceMs = 0;                   // millis() of the last sighting, 0 = never
volatile uint32_t faceArrivals = 0;             // +1 each time a face appears after 3 s with none
volatile uint32_t eyesFrames = 0, faceHits = 0;
volatile float eyesFps = 0;
uint8_t eyeRot = 0;                             // software rotation, x 90 deg clockwise (E finds it)
bool eyeFlip = false;                           // swap left/right (j)
static uint16_t *rotBuf = nullptr;

static void eyesSave() { Preferences p; p.begin("eyes", false); p.putUChar("rot", eyeRot); p.putBool("flip", eyeFlip); p.end(); }
static void eyesLoad() { Preferences p; p.begin("eyes", true); eyeRot = p.getUChar("rot", 0) & 3; eyeFlip = p.getBool("flip", false); p.end(); }
static bool faceFresh(uint32_t ms) { uint32_t f = faceMs; return eyesOn && f && millis() - f < ms; }
static float faceErrDeg() { return faceX * CAM_HFOV_DEG / 2; }   // + = you are to her right

static void rotate565(const uint16_t *in, uint16_t *out, int w, int h, int k) {
  for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) {
    uint16_t v = in[y * w + x];
    if (k == 1) out[x * h + (h - 1 - y)] = v;                     // 90 cw: new size h x w
    else if (k == 2) out[(h - 1 - y) * w + (w - 1 - x)] = v;
    else out[(w - 1 - x) * h + y] = v;                            // 270 cw
  }
}
static void eyesLoop(void *) {
  HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
  HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);
  uint32_t n = 0, t0 = millis(), lastSeen = 0;
  for (;;) {
    if (!eyesOn) { vTaskDelay(pdMS_TO_TICKS(100)); n = 0; t0 = millis(); continue; }
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    int w = fb->width, h = fb->height; uint16_t *img = (uint16_t *)fb->buf;
    uint8_t rot = eyeRot;
    if (rot && rotBuf) { rotate565(img, rotBuf, w, h, rot); img = rotBuf; if (rot & 1) { int t = w; w = h; h = t; } esp_camera_fb_return(fb); fb = nullptr; }
    std::list<dl::detect::result_t> &cand = s1.infer(img, {h, w, 3});
    std::list<dl::detect::result_t> &res = s2.infer(img, {h, w, 3}, cand);
    int best = 0; float bx = 0;
    for (auto &r : res) {
      int bw = r.box[2] - r.box[0];
      if (bw > best) { best = bw; bx = (r.box[0] + r.box[2]) * 0.5f / w * 2 - 1; }
    }
    if (fb) esp_camera_fb_return(fb);
    eyesFrames++;
    if (best > 0) {
      uint32_t now = millis();
      if (!lastSeen || now - lastSeen > 3000) faceArrivals++;
      lastSeen = now;
      faceX = constrain(eyeFlip ? -bx : bx, -1.0f, 1.0f); faceW = (float)best / w; faceMs = now; faceHits++;
    }
    if (++n && millis() - t0 > 2000) { eyesFps = n * 1000.0f / (millis() - t0); n = 0; t0 = millis(); }
    vTaskDelay(1);
  }
}
static void eyesBegin() {
  eyesLoad();
  rotBuf = (uint16_t *)ps_malloc(320 * 240 * 2);
  xTaskCreatePinnedToCore(eyesLoop, "eyes", 32768, nullptr, 1, nullptr, 0);
  Serial.printf("  eyes: face detection ready on core 0 (rotation %d deg%s)\n", eyeRot * 90, eyeFlip ? ", left/right swapped" : "");
}
static void eyesTest() {                          // e: where does she see you?
  if (!okCam) { Serial.println("  no camera"); return; }
  Serial.println("  EYES TEST 15 s: face her from 30-80 cm, good light. Then move to HER left and right.");
  eyesOn = true; uint32_t t0 = millis();
  while (millis() - t0 < 15000) {
    delay(400);
    if (faceFresh(500)) {
      float e = faceErrDeg();
      Serial.printf("  face %s  %+.0f deg  size %.2f  (%.1f fps)\n", fabsf(e) < FACE_DEADBAND_DEG ? "CENTRE" : e < 0 ? "her LEFT " : "her RIGHT", e, faceW, eyesFps);
    } else Serial.printf("  no face  (%.1f fps)\n", eyesFps);
    if (cmdAvailable() && cmdPeek() == 'x') { cmdRead(); break; }
  }
  eyesOn = false;
  Serial.println("  never finds you? send E. Then F: if she turns AWAY from you, send j.");
}
static void eyesOrient() {                        // E: try all 4 rotations while you look at her
  if (!okCam) { Serial.println("  no camera"); return; }
  Serial.println("  FIND ORIENTATION: hold your face 40-60 cm in front of her eyes, still, good light, 16 s...");
  eyesOn = true; uint32_t hits[4]; uint8_t keep = eyeRot;
  for (int k = 0; k < 4; k++) {
    eyeRot = k; delay(600);                       // let the old frame clear
    uint32_t h0 = faceHits, f0 = eyesFrames; delay(3400);
    hits[k] = faceHits - h0;
    Serial.printf("  rotation %3d deg: face in %u of %u frames\n", k * 90, hits[k], eyesFrames - f0);
  }
  eyesOn = false;
  int b = 0; for (int k = 1; k < 4; k++) if (hits[k] > hits[b]) b = k;
  if (hits[b] == 0) { eyeRot = keep; Serial.println("  no face in any rotation: more light, 40-60 cm, lens clean? Not saved."); return; }
  eyeRot = b; eyesSave();
  Serial.printf("  saved: rotation %d deg. Now e to check left/right.\n", b * 90);
}
static void followMode() {                        // F: she turns to keep you in view
  if (!okCam) { Serial.println("  no camera"); return; }
  Serial.println("FOLLOW: she turns to keep watching you. Walk slowly around her. x = stop.");
  eyesOn = true;
  while (!(cmdAvailable() && cmdRead() == 'x')) {
    if (faceFresh(500) && fabsf(faceErrDeg()) > FACE_DEADBAND_DEG) faceSpin(2.0f);
    else { servosOff(); delay(60); }
  }
  eyesOn = false; servosOff();
  Serial.println("follow off (turns away from you? send j)");
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
// Gait patterns (leg phase offsets, FL FR RL RR). 0 = the simulation-trained one. n cycles, T auto-tunes.
const char *GAIT_NAME[] = {"trained", "trot", "bound", "pace", "crawl"};
const float GAIT_PH[5][4] = {
  {CUBI_PH_LF, CUBI_PH_RF, CUBI_PH_LB, CUBI_PH_RB},
  {0.00f, 0.50f, 0.50f, 0.00f},     // trot: diagonal pairs together
  {0.00f, 0.00f, 0.50f, 0.50f},     // bound: front pair, then rear pair
  {0.00f, 0.50f, 0.00f, 0.50f},     // pace: left side, then right side
  {0.00f, 0.50f, 0.75f, 0.25f}};    // crawl: one leg at a time
int gaitIdx = 0;
float cubiSaw(float ph) {   // fast swing forward for CUBI_FAST of the cycle, slow push back for the rest
  ph -= floorf(ph);
  return ph < CUBI_FAST ? -1 + 2 * cubiEase(ph / CUBI_FAST) : 1 - 2 * cubiEase((ph - CUBI_FAST) / (1 - CUBI_FAST));
}
// Soft start / stop (sim: start rocking 40 -> 24 deg/s, stop 38 -> 27): stride eases in, and at the end she keeps
// stepping while the stride shrinks to nothing. An edge or wall still stops her instantly.
static const float WALK_EASE_IN = 0.5f, WALK_EASE_OUT = 0.6f;
static float easeStep(float x) { x = constrain(x, 0.0f, 1.0f); return x * x * (3 - 2 * x); }
float batteryV();
// Steering trim: scales each side's stride. Left legs FL/RL are i = 0, 2.
static float steerMul(int i) { return (i == 0 || i == 2) ? 1 - CUBI_STEER : 1 + CUBI_STEER; }
static void steerLoad() { Preferences p; p.begin("gait", true); CUBI_STEER = p.getFloat("steer", 0); p.end(); }
static void steerSet(float v) {
  CUBI_STEER = constrain(roundf(v * 100) / 100, -0.4f, 0.4f);
  Preferences p; p.begin("gait", false); p.putFloat("steer", CUBI_STEER); p.end();
  Serial.printf("  steer %+.2f (%s), saved. Left legs x%.2f, right legs x%.2f\n", CUBI_STEER,
                CUBI_STEER > 0 ? "pulls LEFT" : CUBI_STEER < 0 ? "pulls RIGHT" : "neutral", 1 - CUBI_STEER, 1 + CUBI_STEER);
}
void walk(float seconds, int dir) {   // dir +1 forward, -1 back. legs[]: FL, FR, RL, RR. 50 Hz loop, like the sim.
  if (batteryV() < BATT_STOP_V) { Serial.println("  battery below 3.6 V: not walking. Charge me."); return; }
  const float *ph = GAIT_PH[gaitIdx];
  const float trim[4] = {CUBI_TRIM_F, CUBI_TRIM_F, CUBI_TRIM_B, CUBI_TRIM_B};
  uint32_t t0 = millis();
  const float total = seconds + WALK_EASE_OUT;
  while (millis() - t0 < (uint32_t)(total * 1000)) {
    uint32_t tick = millis();
    float t = (tick - t0) / 1000.0f;
    float env = easeStep(t / WALK_EASE_IN) * (1 - easeStep((t - seconds) / WALK_EASE_OUT));
    for (int i = 0; i < 4; i++) servoDeg(i, env * (dir * CUBI_AMP_DEG * steerMul(i) * cubiSaw(CUBI_FREQ_HZ * t + ph[i]) + trim[i]));
    uint16_t d = tof.readRangeContinuousMillimeters();
    int raw = tofVerdict(d, tof.timeoutOccurred());
    if (raw && dir > 0 && reflexOn) Serial.printf("  %u mm -> %s\n", d, raw == 1 ? "edge" : "wall");
    static int streak = 0, last = 0;                 // a real edge/wall shows up on several readings in a row;
    streak = (raw && raw == last) ? streak + 1 : (raw ? 1 : 0);   // one dropout (8191) or a body wobble does not
    last = raw;
    int v = (streak >= REFLEX_CONFIRM) ? raw : 0;
    if (!reflexOn) v = 0;
    if (v && dir > 0) { streak = 0; last = 0; }
    if (v && dir > 0) { Serial.println(v == 1 ? "  EDGE! backing up" : "  wall - stop"); if (v == 1) walk(CUBI_BACKUP_S, -1); break; }
    while (millis() - tick < 20) delay(1);
  }
  for (int i = 0; i < 4; i++) servoDeg(i, 0);
  delay(300); servosOff();
}

// ---------- auto-tune: she measures her own progress against a box in front of her ----------
float avgTof(int n) {
  uint32_t sum = 0; int k = 0;
  for (int i = 0; i < n * 2 && k < n; i++) { uint16_t d = tof.readRangeContinuousMillimeters(); if (!tof.timeoutOccurred() && d < 1500) { sum += d; k++; } }
  return k >= n / 2 ? (float)sum / k : -1;
}
float tryGait(int g, float amp, float freq, float fast, float st) {   // returns mm moved toward the box (+ = forward), or NAN
  gaitIdx = g; CUBI_AMP_DEG = amp; CUBI_FREQ_HZ = freq; CUBI_FAST = fast; stanceOff = st;
  for (int i = 0; i < 4; i++) servoDeg(i, 0); delay(400); servosOff(); delay(200);
  float d0 = avgTof(15); if (d0 < 0) return NAN;
  if (d0 < 70) { Serial.println("  too close to the box: move her back to ~20 cm and send T again"); return NAN; }
  bool r = reflexOn; reflexOn = false; walk(3, 1); reflexOn = r;
  delay(300);
  float d1 = avgTof(15); if (d1 < 0) return NAN;
  float moved = d0 - d1;
  Serial.printf("  %-7s amp %2.0f  freq %.1f  fast %.2f  stance %+3.0f  ->  %+5.0f mm\n", GAIT_NAME[g], amp, freq, fast, st, moved);
  return moved;
}
void autoTune() {
  Serial.println("AUTO-TUNE v3: leaning the legs. Box or book (5+ cm tall) ~20 cm in front of her eyes, cable slack.");
  Serial.println("  She kept drifting BACKWARD, so v3 rakes all four legs back or forward (like bristles) and measures.");
  Serial.println("  Each setting runs twice. ~3 minutes. Nudge her back in line between runs if she turns. Starting in 5 s...");
  delay(5000);
  float best = -1e9; int bg = 0; float bk = 0.5f, bs = 0;
  const int gs[] = {0, 3, 1};                  // trained, pace, trot
  const float ks[] = {0.50f, 0.80f, 0.50f};
  const float sts[] = {-10.0f, -5.0f, 0.0f, 5.0f, 10.0f};
  for (int gi = 0; gi < 3; gi++) for (int si = 0; si < 5; si++) {
    float a = tryGait(gs[gi], 12, 2.2f, ks[gi], sts[si]); if (isnan(a)) goto done;
    float b = tryGait(gs[gi], 12, 2.2f, ks[gi], sts[si]); if (isnan(b)) goto done;
    float m = (a + b) / 2;
    Serial.printf("    avg %+.0f mm   (both runs %s)\n", m, (a > 10 && b > 10) ? "FORWARD" : (a < -10 && b < -10) ? "BACKWARD" : "mixed");
    if (m > best && a > 0 && b > 0) { best = m; bg = gs[gi]; bk = ks[gi]; bs = sts[si]; }
  }
done:
  gaitIdx = bg; CUBI_AMP_DEG = 12; CUBI_FREQ_HZ = 2.2f; CUBI_FAST = bk; stanceOff = bs;
  if (best < -1e8) Serial.println("BEST: nothing went forward on both runs. Paste me the list.");
  else Serial.printf("BEST (forward on both runs): %s  fast %.2f  stance %+.0f  ->  %+.0f mm in 3 s. Now active; w to walk.\n", GAIT_NAME[bg], bk, bs, best);
}

void report(const char *what, bool ok) { Serial.printf("[%s] %s\n", ok ? "PASS" : "FAIL", what); }

// ===== inlined cubi_play.h =====
// CUBI - puppy mode. Same tricks, mood rules and sounds as cubi_play.py (tested in the physics sim:
// 16 x 60 s of random taps / claps / hands near a desk edge, 0 falls). Included by ponder_selftest.ino after
// the servo, ToF, mic and amp helpers. Serial: P = puppy mode on, x = off, 1-9 = play one trick, L = lie down, U = get up.

// ---------------- sounds (16 kHz synth, no audio files) ----------------
static void chirp(float f0, float f1, int ms, float vol = 0.5f, float vib = 0, float noise = 0) {
  if (!okAmp) return;
  const int n = 16 * ms; int16_t buf[160]; float ph = 0; int i = 0;
  vol *= 0.7f;                                         // speaker limit
  while (i < n) {
    int k = 0;
    for (; k < 160 && i < n; k++, i++) {
      float x = (float)i / n;
      float f = f0 + (f1 - f0) * x + vib * sinf(2 * PI * 7 * i / 16000.0f) * f0 * 0.04f;
      ph += 2 * PI * f / 16000.0f;
      float env = fminf(1.0f, x * 12) * powf(1 - x, 1.5f);
      float s = sinf(ph) * (1 - noise) + noise * ((esp_random() & 0xFFFF) / 32768.0f - 1);
      buf[k] = (int16_t)(vol * env * s * 32767);
    }
    amp.write((uint8_t *)buf, k * 2);
  }
}
static void rest(int ms) { if (okAmp) { int16_t z[160] = {0}; for (int i = 0; i < ms / 10; i++) amp.write((uint8_t *)z, 320); } }
static void say(const char *s) {
  if (!s) return;
  if (!strcmp(s, "yip")) chirp(900, 1700, 90);
  else if (!strcmp(s, "yip2")) { chirp(900, 1700, 80); rest(60); chirp(1000, 1900, 80); }
  else if (!strcmp(s, "whine")) chirp(1300, 700, 450, 0.4f, 1.0f);
  else if (!strcmp(s, "sigh")) chirp(420, 260, 650, 0.35f, 0, 0.6f);
  else if (!strcmp(s, "trill")) for (int i = 0; i < 8; i++) chirp(i % 2 ? 1300 : 1700, i % 2 ? 1500 : 1900, 32, 0.4f);
  else if (!strcmp(s, "ruff")) { chirp(520, 380, 110, 0.6f, 0, 0.35f); rest(70); chirp(560, 400, 90, 0.55f, 0, 0.35f); }
}

// ---------------- tricks: leg angles in firmware order FL, FR, RL, RR (+ = forward) ----------------
static float envl(float t, float dur, float ramp = 0.12f) { return fmaxf(0, fminf(1, fminf(t / ramp, (dur - t) / ramp))); }
static void walkDeg(float t, float dir, float sideL, float sideR, float q[4]) {
  const float ph[4] = {CUBI_PH_LF, CUBI_PH_RF, CUBI_PH_LB, CUBI_PH_RB};
  const float trim[4] = {CUBI_TRIM_F, CUBI_TRIM_F, CUBI_TRIM_B, CUBI_TRIM_B};
  for (int i = 0; i < 4; i++) {
    float a = constrain(CUBI_AMP_DEG * steerMul(i) * cubiSaw(CUBI_FREQ_HZ * t + ph[i]) + trim[i], -12.0f, 12.0f);
    q[i] = a * dir * ((i == 0 || i == 2) ? sideL : sideR);
  }
}
struct TrickInfo { const char *name; float dur; bool forward; const char *sound; };
static const TrickInfo TRICK[N_TRICKS] = {
  {"wiggle", 1.0f, false, "trill"}, {"paw pat", 1.2f, false, "yip2"}, {"bounce", 1.0f, false, "yip"},
  {"chase", 1.2f, true, "yip"}, {"zoomies", 3.5f, true, "ruff"}, {"sniff", 2.6f, false, nullptr},
  {"startle", 0.9f, false, "whine"}, {"stretch", 4.5f, true, "sigh"}, {"settle", 2.4f, false, "sigh"},
  {"lie down", 1.8f, false, "sigh"}, {"get up", 1.8f, false, "yip"}, {"tuck", 1.8f, false, nullptr}, {"untuck", 1.8f, false, nullptr}};
static bool lying = false, tucked = false;
const float BOW_DEG = 72.0f, KICK_DEG = 60.0f;   // stretch uses the folding reach (front -15..75, rear -75..15)
static float smoothstep(float x) { x = constrain(x, 0.0f, 1.0f); return x * x * (3 - 2 * x); }

static void trickPose(Trick k, float t, float q[4]) {
  float a;
  switch (k) {
    case WIGGLE:  a = 6 * envl(t, 1.0f) * sinf(2 * PI * 6 * t); q[0] = a; q[2] = a; q[1] = -a; q[3] = -a; break;   // L vs R shimmy
    case PAW_PAT: a = 10 * envl(t, 1.2f) * sinf(2 * PI * 5 * t); q[0] = a; q[1] = -a; q[2] = q[3] = -6 * envl(t, 1.2f); break;
    case BOUNCE:  a = 6 * envl(t, 1.0f) * sinf(2 * PI * 4 * t); q[0] = q[1] = q[2] = q[3] = a; break;        // 6 Hz resonates: don't
    case CHASE:   walkDeg(t, 1, 1, 1, q); break;
    case ZOOMIES: if (t < 2.0f) walkDeg(t, 1, 1, 1, q); else walkDeg(t, 1, 1, -1, q); break;
    case SNIFF:   if (t < 0.9f) walkDeg(t, 1, 0, 1, q); else if (t < 1.3f || t >= 2.2f) q[0] = q[1] = q[2] = q[3] = 0;
                  else walkDeg(t, 1, 1, 0, q); break;
    case STARTLE: if (t < 0.45f) walkDeg(t, -1, 1, 1, q); else q[0] = q[1] = q[2] = q[3] = 0; break;
    case STRETCH:                       // play bow (front paws 72 out, bum bob), then each back leg 60 back in turn
      if (t < 2.2f) {
        a = smoothstep(t / 0.8f) * (1 - smoothstep((t - 1.6f) / 0.6f)); q[0] = q[1] = BOW_DEG * a;
        q[2] = q[3] = (t > 0.8f && t < 1.6f) ? 3 * sinf(2 * PI * 3 * (t - 0.8f)) : 0;
      } else {
        q[0] = q[1] = 0;
        float u = t - 2.3f; q[2] = -KICK_DEG * smoothstep(u / 0.35f) * (1 - smoothstep((u - 0.65f) / 0.35f));
        u = t - 3.4f;       q[3] = -KICK_DEG * smoothstep(u / 0.35f) * (1 - smoothstep((u - 0.65f) / 0.35f));
      }
      break;
    case FLOP:    a = FOLD_DEG * smoothstep(t / 1.4f); q[0] = q[1] = a; q[2] = q[3] = -a; break;                // fold flat
    case GETUP:   a = FOLD_DEG * (1 - smoothstep(t / 1.4f)); q[0] = q[1] = a; q[2] = q[3] = -a; break;
    case TUCK:    a = FOLD_DEG + (TUCK_DEG - FOLD_DEG) * smoothstep(t / 1.4f); q[0] = q[1] = a; q[2] = q[3] = -a; break;
    case UNTUCK:  a = TUCK_DEG + (FOLD_DEG - TUCK_DEG) * smoothstep(t / 1.4f); q[0] = q[1] = a; q[2] = q[3] = -a; break;
    default:      a = 3 * fmaxf(0, 1 - t / 2.0f) * sinf(2 * PI * 0.8f * t); q[0] = q[1] = q[2] = q[3] = a; break;     // SETTLE
  }
}

// ---------------- senses ----------------
static int touchIdle = 0;
static uint32_t lastTap = 0, handCloseAt = 0;
static bool handWasClose = false;
static int micQuiet = 500;
static bool edgeAhead() {
  if (!okTof) return false;
  uint16_t d = tof.readRangeContinuousMillimeters();
  return tofVerdict(d, tof.timeoutOccurred()) == 1;
}
static bool tapped() {
  int v = touchRead(PIN_TOUCH);                 // ESP32-S3 touch values RISE when touched
  if (touchIdle && v > touchIdle * 1.15f && millis() - lastTap > 400) { lastTap = millis(); return true; }
  return false;
}
static bool micDeaf = false;                     // mic stuck at full scale: never report a clap (else nonstop fake claps)
static bool clapped() {
  if (!okMic || micDeaf) return false;
  int l = micLevel(15);
  bool c = l > max(micQuiet * 5, 3000);
  if (!c) micQuiet = (micQuiet * 31 + l) / 32;
  return c;
}
// 0 = nothing, 1 = hand close in front, 2 = hand just pulled away (chase it!)
static int handState() {
  if (!okTof) return 0;
  uint16_t d = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) return 0;
  if (d < floorMm * CUBI_WALL_RATIO) { if (!handWasClose) handCloseAt = millis(); handWasClose = true; return 1; }
  if (handWasClose && d > floorMm * 0.9f) { handWasClose = false; return millis() - handCloseAt < 3000 ? 2 : 0; }
  return 0;
}

// ---------------- play one trick (edge reflex always wins) ----------------
static bool playTrick(Trick k, float speed = 1.0f) {
  if (tucked && k != UNTUCK) { tucked = false; playTrick(UNTUCK); }            // legs down first
  if (lying && k != GETUP && k != FLOP && k != TUCK && k != UNTUCK) { lying = false; playTrick(GETUP); }   // stand up before playing
  if (k == FLOP && lying) return true;
  if (k == GETUP && !lying) return true;
  if ((k == TUCK && (!lying || TUCK_DEG < FOLD_DEG + 1)) || (k == UNTUCK && !tucked)) return true;
  const bool fold = (k == FLOP || k == GETUP || k == TUCK || k == UNTUCK || k == STRETCH);   // full servo reach
  say(TRICK[k].sound);
  const float dur = TRICK[k].dur / speed;
  uint32_t t0 = millis();
  while (millis() - t0 < dur * 1000) {
    uint32_t tick = millis();
    float t = (tick - t0) / 1000.0f * speed, q[4];
    if (TRICK[k].forward && edgeAhead()) {                  // stop, back off, turn away
      Serial.println("  edge! backing off");
      say("whine");
      uint32_t b0 = millis();
      while (millis() - b0 < CUBI_BACKUP_S * 1000) { walkDeg((millis() - b0) / 1000.0f, -1, 1, 1, q); for (int i = 0; i < 4; i++) servoDeg(i, q[i]); delay(20); }
      b0 = millis();
      while (millis() - b0 < 1600) { walkDeg((millis() - b0) / 1000.0f, 1, 1, -1, q); for (int i = 0; i < 4; i++) servoDeg(i, q[i]); delay(20); }
      return false;
    }
    trickPose(k, t, q);
    for (int i = 0; i < 4; i++) servoDeg(i, q[i], fold);
    while (millis() - tick < 20) delay(1);
  }
  if (k == FLOP) { lying = true; delay(300); servosOff(); }          // resting on her belly: servos can relax
  if (k == GETUP) lying = false;
  if (k == TUCK) { tucked = true; delay(300); servosOff(); }
  if (k == UNTUCK) tucked = false;
  return true;
}

// Goodnight: settle, lie down, fold the legs up as far as the servos reach, relax. Then it's safe to flip the switch.
// (The switch cuts the battery instantly, so run this first: type Z, or puppy mode does it on a flat battery.)
static void goodnight() {
  playTrick(SETTLE); playTrick(FLOP); playTrick(TUCK);
  servosOff();
  Serial.printf("  goodnight: lying down, legs folded to %.0f deg. Safe to switch off.\n", TUCK_DEG);
}

// Servo range finder (bench, NO leg on the servo): < and > step the FL servo 25 us; note where it stops turning.
static void rangeFinder() {
  Serial.println("RANGE FINDER (FL servo, no leg fitted): < and > step 25 us, x = done. Stop at the first pulse where it");
  Serial.println("stops turning or buzzes. Then measure the angle between the two ends and set SERVO_TRAVEL_DEG / MIN_US / MAX_US.");
  int us = SERVO_CENTRE_US;
  ledcWrite(legs[0].pin, usToDuty(us));
  while (true) {
    if (!cmdAvailable()) { delay(10); continue; }
    char c = cmdRead();
    if (c == 'x') break;
    if (c == '<') us = max(400, us - 25);
    if (c == '>') us = min(2600, us + 25);
    ledcWrite(legs[0].pin, usToDuty(us));
    Serial.printf("  %d us\n", us);
  }
  ledcWrite(legs[0].pin, usToDuty(SERVO_CENTRE_US)); delay(400); servosOff();
}

// ---------------- mood engine ----------------
struct Mood { float energy = 0.8f, excite = 0.3f, idle = 0; uint32_t lastZoom = 0; int last = -1; } mood;
static void stimulus(float amt) { mood.excite = fminf(1, mood.excite + amt); mood.idle = 0; }
static int pickRaw(int hand) {
  float r = (esp_random() & 0xFFFF) / 65536.0f, e = mood.excite;
  if (mood.idle > 300) return -2;                                  // sleep
  if (hand == 1) return PAW_PAT;
  if (hand == 2 && e > 0.3f) return CHASE;
  if (e > 0.7f) {
    if (mood.energy > 0.5f && millis() - mood.lastZoom > 60000 && r < 0.35f) { mood.lastZoom = millis(); return ZOOMIES; }
    return r < 0.7f ? BOUNCE : WIGGLE;
  }
  if (e > 0.4f) return r < 0.33f ? WIGGLE : r < 0.66f ? PAW_PAT : BOUNCE;
  if (e > 0.15f) return r < 0.6f ? SNIFF : CHASE;                  // explore = a short chase-walk
  return r < 0.5f ? SETTLE : -1;                                   // -1 = sit still
}
static int pickTrick(int hand) {
  int a = pickRaw(hand);
  if (a == mood.last && hand == 0 && (a == BOUNCE || a == WIGGLE || a == PAW_PAT || a == SNIFF)) a = pickRaw(hand);
  mood.last = a;
  return a;
}

static void puppyMode() {
  Serial.println("PUPPY MODE: tap her head, clap, wave a hand in front of her eyes. x = stop, 1-9 = a trick, L = lie down, U = get up.");
  touchIdle = touchRead(PIN_TOUCH);
  say("yip2");
  bool asleep = false;
  uint32_t next = millis() + 800, lastT = millis();
  while (true) {
    if (cmdAvailable()) {
      char c = cmdRead();
      if (c == 'x') break;
      if (c >= '1' && c <= '9') { playTrick((Trick)(c - '1')); continue; }
      if (c == 'L') { playTrick(FLOP); continue; }
      if (c == 'U') { playTrick(GETUP); continue; }
      if (c == 'Z') { goodnight(); asleep = true; continue; }
    }
    float dt = (millis() - lastT) / 1000.0f; lastT = millis();
    mood.excite *= expf(-dt / 20.0f);
    mood.idle += dt;
    float vb = batteryV();
    if (vb < BATT_STOP_V) { if (!asleep) { say("sigh"); goodnight(); asleep = true; Serial.println("  tired (battery low): charge me"); } delay(200); continue; }
    // senses (react within one 20 ms tick)
    bool tap = tapped(), clap = clapped();
    int hand = handState();
    if (asleep) {
      if (tap || clap) { asleep = false; mood.idle = 0; stimulus(0.4f); playTrick(UNTUCK); playTrick(GETUP); playTrick(STRETCH); }
      else { delay(50); continue; }
    }
    if (tap) { stimulus(0.5f); playTrick(WIGGLE, 0.9f + 0.2f * (esp_random() & 0xFF) / 255.0f); next = millis() + 600; continue; }
    if (clap) { stimulus(0.3f); playTrick(mood.excite < 0.4f ? STARTLE : BOUNCE); next = millis() + 600; continue; }
    if (hand) stimulus(0.25f);
    if (millis() < next && !hand) { delay(10); continue; }
    int a = pickTrick(hand);
    if (a == -2) { playTrick(SETTLE); playTrick(FLOP); asleep = true; Serial.println("  zzz, lying down (tap to wake)"); continue; }
    if (a >= 0) {
      Serial.printf("  %s  (excite %.2f, energy %.2f)\n", TRICK[a].name, mood.excite, mood.energy);
      playTrick((Trick)a, 0.85f + 0.3f * (esp_random() & 0xFF) / 255.0f);       // never the same speed twice
      mood.energy = fmaxf(0, mood.energy - 0.02f * TRICK[a].dur);
    } else {
      servosOff();
      mood.energy = fminf(1, mood.energy + 0.01f);
    }
    next = millis() + 500 + (esp_random() % 2500);                             // 0.5-3 s between ideas
  }
  for (int i = 0; i < 4; i++) servoDeg(i, 0);
  delay(300); servosOff();
  Serial.println("puppy mode off");
}

// ===== end cubi_play.h =====

// ===== inlined cubi_mind.h =====
// CUBI - a mind of her own. Mirrors cubi_mind.py (tested in the physics sim on a finite 50 x 36 cm desk with a
// simulated person who comes and goes). Included after cubi_play.h: reuses its tricks, sounds, senses and the
// edge reflex. Serial: M = mind mode, x = stop, I = inner state, W = new personality (forgets what she learned).
//
//   traits    bold, social, playful, lazy (0..1): rolled on first boot, saved in flash. Every Cubi is different.
//   drives    energy, curiosity, social, boredom (0..1) build up / drain; excitement spikes on a tap.
//   choice    each drive scores a behaviour (explore, call you, play, nap, look around, sit), scaled by her
//             traits; she picks with a little randomness so she's consistent but never a loop.
//   learning  a tap within 4 s of a trick is praise: that trick gets likelier. Ignored ones fade. Saved.
//   reflexes  edge reflex on every step and bow, taps / claps / hands interrupt whatever she's doing.

#include <Preferences.h>

static const char *TRAIT_NAME[4] = {"bold", "social", "playful", "lazy"};
static const Trick PLAY[6] = {WIGGLE, PAW_PAT, BOUNCE, SNIFF, STRETCH, ZOOMIES};
static const float SPIN_DEG_S = 22.6f;          // measured in the sim; turn time = degrees / this
static const float PRAISE_S = 4.0f, TEMP = 0.12f;
// While exploring she glances 25 deg left and right every 2 s of walking. The distance sensor only looks ahead:
// walking within ~20 deg of parallel to a desk edge, her side legs go over before the beam sees it (sim).
static const float GLANCE_DEG = 25.0f, GLANCE_EVERY = 2.0f;

struct Mind {
  float trait[4];                                // bold, social, playful, lazy
  float pref[6];                                 // learned liking per PLAY trick
  uint8_t breed = 0;                             // 0 = her own random self, 1 = golden retriever (G)
  float energy = 0.8f, curiosity = 0.4f, social = 0.3f, boredom = 0.3f, excite = 0.2f;
  int last = -1, praiseFor = -1; uint32_t praiseUntil = 0, lastContact = 0;
} mind;
// Breed habits (cubi_mind.py BREEDS). Golden retriever: lives for people, learns fast from praise and keeps it,
// a clap is a party not a threat, greets you like you've been gone a year, wags while sitting, calls you with a
// happy yip-bounce instead of a whine.
struct Habits { float praise, fade; bool startles, greet, wag, yipCall; };
static const Habits HABITS[2] = {{0.35f, 0.10f, true, false, false, false}, {0.60f, 0.05f, false, true, true, true}};
static const float GOLDEN_TRAIT[4] = {0.6f, 0.95f, 0.85f, 0.35f};
static const uint32_t GREET_AFTER_MS = 60000;
#define H HABITS[mind.breed]
static Preferences nvs;

static float frand() { return (esp_random() & 0xFFFFFF) / 16777216.0f; }
static float lerpTrait(float v) { return 0.4f + 1.2f * v; }

static void mindSave() {
  nvs.begin("cubi", false);
  nvs.putBytes("trait", mind.trait, sizeof(mind.trait));
  nvs.putBytes("pref", mind.pref, sizeof(mind.pref));
  nvs.putUChar("breed", mind.breed);
  nvs.end();
}
static void mindLoad(bool reroll = false) {
  nvs.begin("cubi", true);
  bool have = !reroll && nvs.getBytesLength("trait") == sizeof(mind.trait);
  if (have) { nvs.getBytes("trait", mind.trait, sizeof(mind.trait)); nvs.getBytes("pref", mind.pref, sizeof(mind.pref)); mind.breed = nvs.getUChar("breed", 0) ? 1 : 0; }
  nvs.end();
  if (!have) {                                   // first boot (or W): who is she?
    for (int i = 0; i < 4; i++) mind.trait[i] = 0.1f + 0.8f * frand();
    for (int i = 0; i < 6; i++) mind.pref[i] = 0;
    mind.breed = 0;
    mindSave();
    Serial.println("  a new Cubi is born");
  }
}
static void makeGolden() {                       // G: golden retriever traits + habits, keeps what she has learned
  mindLoad();
  for (int i = 0; i < 4; i++) mind.trait[i] = GOLDEN_TRAIT[i];
  mind.breed = 1;
  mindSave();
  Serial.println("  she's a golden retriever now");
}
static void mindPrint() {
  Serial.printf("  %s\n", mind.breed ? "golden retriever" : "her own random self");
  Serial.printf("  traits: bold %.2f  social %.2f  playful %.2f  lazy %.2f\n", mind.trait[0], mind.trait[1], mind.trait[2], mind.trait[3]);
  Serial.printf("  drives: energy %.2f  curiosity %.2f  social %.2f  boredom %.2f  excite %.2f\n", mind.energy, mind.curiosity, mind.social, mind.boredom, mind.excite);
  Serial.print("  likes: ");
  for (int i = 0; i < 6; i++) Serial.printf("%s %+.2f  ", TRICK[PLAY[i]].name, mind.pref[i]);
  Serial.println();
}

// ---- drives ----
static void mindTick(float dt, Act a) {
  const bool active = (a == A_WALK || a == A_TRICK || a == A_CALL);
  if (a == A_REST) mind.energy = fminf(1, mind.energy + 0.012f * dt);
  else mind.energy = fmaxf(0, mind.energy - (active ? 0.0028f * (0.7f + 0.6f * mind.trait[3]) : 0.0004f) * dt);
  if (a == A_WALK) mind.curiosity = fmaxf(0, mind.curiosity - 0.025f * dt);
  else mind.curiosity = fminf(1, mind.curiosity + dt / 150.0f * lerpTrait(mind.trait[0]));
  mind.social = fminf(1, mind.social + dt / 200.0f * lerpTrait(mind.trait[1]));
  if (a != A_TRICK) mind.boredom = fminf(1, mind.boredom + dt / 90.0f * lerpTrait(mind.trait[2]));
  mind.excite *= expf(-dt / 20.0f);
}
static void mindStimulus(int kind) {               // 0 tap, 1 clap, 2 hand
  const float ex[3] = {0.5f, 0.3f, 0.25f}, so[3] = {0.15f, 0.6f, 0.5f};
  mind.excite = fminf(1, mind.excite + ex[kind]);
  mind.social *= so[kind];
  if (kind == 0 && mind.praiseFor >= 0 && millis() <= mind.praiseUntil) {
    mind.pref[mind.praiseFor] = fminf(2, mind.pref[mind.praiseFor] + H.praise);
    Serial.printf("  praised for %s -> likes it %+.2f\n", TRICK[PLAY[mind.praiseFor]].name, mind.pref[mind.praiseFor]);
    mind.praiseFor = -1;
    mindSave();
  }
}
static void mindPlayed(int idx) {
  if (mind.praiseFor >= 0) mind.pref[mind.praiseFor] = fmaxf(-1, mind.pref[mind.praiseFor] - H.fade);   // went unnoticed
  mind.praiseFor = idx; mind.praiseUntil = millis() + (uint32_t)(PRAISE_S * 1000);
  mind.boredom = fmaxf(0, mind.boredom - 0.15f);
}

// ---- choice ----
static const char *CHOICE_NAME[N_CHOICE] = {"curious: exploring", "lonely: calling you", "playful", "sleepy: napping", "looking around", "content: sitting"};
static int mindChoose() {
  if (mind.energy < 0.12f) return C_REST;
  const float *T = mind.trait, e = mind.energy;
  float u[N_CHOICE] = {
    mind.curiosity * (0.4f + 0.6f * e) * lerpTrait(T[0]),
    mind.social * mind.social * lerpTrait(T[1]) * 1.3f,
    (mind.boredom * (0.3f + 0.7f * e) + 0.5f * mind.excite) * lerpTrait(T[2]),
    powf(fmaxf(0, 0.55f - e) / 0.55f, 1.5f) * lerpTrait(T[3]) * 2.0f,
    0.18f,
    0.10f + 0.15f * T[3]};
  if (mind.last == C_LOOK || mind.last == C_IDLE || mind.last == C_CALL) u[mind.last] *= 0.5f;
  float mx = 0, w[N_CHOICE], sum = 0;
  for (int i = 0; i < N_CHOICE; i++) mx = fmaxf(mx, u[i]);
  for (int i = 0; i < N_CHOICE; i++) { w[i] = expf((u[i] - mx) / TEMP); sum += w[i]; }
  float r = frand() * sum;
  int c = N_CHOICE - 1;
  for (int i = 0; i < N_CHOICE; i++) { r -= w[i]; if (r <= 0) { c = i; break; } }
  mind.last = c;
  return c;
}
static int mindPickTrick() {
  float w[6], sum = 0, mx = -10;
  for (int i = 0; i < 6; i++) mx = fmaxf(mx, mind.pref[i]);
  for (int i = 0; i < 6; i++) {
    bool ok = PLAY[i] != ZOOMIES || (mind.energy > 0.5f && mind.excite > 0.5f);
    w[i] = ok ? expf((mind.pref[i] - mx) / 0.5f) : 0; sum += w[i];
  }
  float r = frand() * sum;
  for (int i = 0; i < 6; i++) { r -= w[i]; if (r <= 0 && w[i] > 0) return i; }
  return 0;
}

// ---- body: moves that keep the reflexes live ----
static uint32_t faceSeenArr = 0, faceNoticedMs = 0;
static int senseEvent(bool faces = true) {         // -1 nothing, 0 tap, 1 clap, 2 hand close, 3 hand pulled away, 4 sees you
  if (tapped()) return 0;
  if (faces && eyesOn && faceArrivals != faceSeenArr) {           // a face after 3 s of none
    faceSeenArr = faceArrivals;
    if (millis() - faceNoticedMs > 15000) { faceNoticedMs = millis(); return 4; }
  }
  if (clapped()) return 1;
  int h = handState();
  return h == 1 ? 2 : h == 2 ? 3 : -1;
}
static void escapeEdge() {
  Serial.println("  edge! backing off");
  say("whine");
  float q[4]; uint32_t b0 = millis();
  while (millis() - b0 < CUBI_BACKUP_S * 1000) { walkDeg((millis() - b0) / 1000.0f, -1, 1, 1, q); for (int i = 0; i < 4; i++) servoDeg(i, q[i]); delay(20); }
  static const float esc[6] = {-160, -130, -100, 100, 130, 160};   // same as cubi_mind.py
  float deg = esc[esp_random() % 6];
  b0 = millis();
  while (millis() - b0 < fabsf(deg) / SPIN_DEG_S * 1000) {
    walkDeg((millis() - b0) / 1000.0f, 1, deg < 0 ? 1 : -1, deg < 0 ? -1 : 1, q); for (int i = 0; i < 4; i++) servoDeg(i, q[i]); delay(20);
  }
}
static int turnBy(float deg, bool guarded = false) {   // returns a sense event if one interrupted, -2 = edge
  float q[4]; uint32_t t0 = millis();
  while (millis() - t0 < fabsf(deg) / SPIN_DEG_S * 1000) {
    if (guarded && edgeAhead()) { escapeEdge(); return -2; }
    walkDeg((millis() - t0) / 1000.0f, 1, deg < 0 ? 1 : -1, deg < 0 ? -1 : 1, q);   // (1,-1) spins right
    for (int i = 0; i < 4; i++) servoDeg(i, q[i]);
    int ev = senseEvent(); if (ev >= 0) return ev;
    delay(20);
  }
  return -1;
}
static int walkFor(float secs) {                   // forward, edge-guarded; -2 = edge, else sense event / -1
  float q[4]; uint32_t t0 = millis();
  while (millis() - t0 < (secs + 0.3f) * 1000) {     // short 0.3 s ease in / out: she stops to glance every 2 s
    if (edgeAhead()) { escapeEdge(); return -2; }
    float t = (millis() - t0) / 1000.0f, env = easeStep(t / 0.3f) * (1 - easeStep((t - secs) / 0.3f));
    walkDeg(t, 1, 1, 1, q);
    for (int i = 0; i < 4; i++) servoDeg(i, env * q[i]);
    int ev = senseEvent(); if (ev >= 0) return ev;
    delay(20);
  }
  return -1;
}
static int sitFor(float secs) {
  servosOff();
  uint32_t t0 = millis();
  while (millis() - t0 < secs * 1000) { int ev = senseEvent(); if (ev >= 0) return ev; delay(20); }
  return -1;
}

// ---- eyes in mind mode ----
void faceSpin(float maxS) {                        // spin on the spot toward the face; stops centred or when lost
  float q[4]; uint32_t t0 = millis();
  while (millis() - t0 < maxS * 1000 && faceFresh(600)) {
    float e = faceErrDeg();
    if (fabsf(e) < FACE_DEADBAND_DEG) break;
    walkDeg((millis() - t0) / 1000.0f, 1, e > 0 ? 1 : -1, e > 0 ? -1 : 1, q);   // (1,-1) spins right
    for (int i = 0; i < 4; i++) servoDeg(i, q[i]);
    delay(20);
  }
  servosOff();
}
static int watchYou(float secs) {                  // turn to you and keep watching; ends early if you leave
  Serial.println("    watching you");
  uint32_t t0 = millis(), lastT = millis();
  while (millis() - t0 < secs * 1000 && faceFresh(2500)) {
    if (faceFresh(500) && fabsf(faceErrDeg()) > FACE_DEADBAND_DEG) faceSpin(1.5f);
    int ev = senseEvent(false); if (ev >= 0) return ev;
    float dt = (millis() - lastT) / 1000.0f; lastT = millis();
    mind.social = fmaxf(0, mind.social - 0.06f * dt);              // company fills her up
    delay(40);
  }
  return -1;
}
static bool searchForYou() {                       // lonely: turn around in steps, looking for a face
  for (int k = 0; k < 6; k++) {
    uint32_t p0 = millis();
    while (millis() - p0 < 700) { if (faceFresh(400)) return true; delay(30); }
    if (turnBy(60) >= 0) return faceFresh(1000);   // a tap etc. interrupts
  }
  return faceFresh(500);
}
static void react(int ev) {
  if (ev < 0) return;
  if (H.greet && ev != 1 && millis() - mind.lastContact > GREET_AFTER_MS) {      // you're back!
    Serial.println("  you're back!");
    mind.excite = 1; mind.social = 0;
    say("yip2"); playTrick(WIGGLE); playTrick(BOUNCE); playTrick(WIGGLE);
  }
  mind.lastContact = millis();
  if (ev == 4) {                                   // she notices you
    Serial.println("  sees you!");
    mind.excite = fminf(1, mind.excite + 0.25f); mind.social *= 0.5f;
    say("yip"); int e2 = watchYou(4 + 5 * mind.trait[1]);
    if (e2 >= 0 && e2 != 4) react(e2);
    return;
  }
  if (ev == 0) { mindStimulus(0); playTrick(WIGGLE, 0.9f + 0.2f * frand()); }
  else if (ev == 1) { mindStimulus(1); playTrick(mind.excite <= 0.4f && H.startles ? STARTLE : BOUNCE); }
  else { mindStimulus(2); playTrick(ev == 2 ? PAW_PAT : CHASE); }
}

static void mindMode() {
  mindLoad();
  Serial.println("MIND MODE: she does what she wants. Tap her head, clap, wave a hand. x = stop, I = what's she thinking.");
  mindPrint();
  touchIdle = touchRead(PIN_TOUCH);
  eyesOn = okCam; faceSeenArr = faceArrivals;
  say("yip2");
  uint32_t lastT = millis();
  while (true) {
    if (cmdAvailable()) {
      char c = cmdRead();
      if (c == 'x') break;
      if (c == 'I') mindPrint();
    }
    if (batteryV() < BATT_STOP_V) { say("sigh"); goodnight(); Serial.println("  tired (battery low): charge me"); break; }
    int c = mindChoose();
    Serial.printf("  %s  (energy %.2f curious %.2f social %.2f bored %.2f)\n", CHOICE_NAME[c], mind.energy, mind.curiosity, mind.social, mind.boredom);
    uint32_t t0 = millis(); int ev = -1; Act a = A_IDLE;
    switch (c) {
      case C_EXPLORE: {
        static const float opts[8] = {-135, -90, -45, 0, 45, 90, 135, 180};      // same as cubi_mind.py
        float deg = opts[esp_random() % 8];
        if (deg != 0) ev = turnBy(deg);
        float left = 2.5f + 4.0f * mind.trait[0] + 1.5f * frand();
        while (left > 0 && ev == -1) {
          const float g[3] = {-GLANCE_DEG, 2 * GLANCE_DEG, -GLANCE_DEG};    // sweep the beam left and right
          for (int k = 0; k < 3 && ev == -1; k++) ev = turnBy(g[k], true);
          if (ev == -1) ev = walkFor(fminf(GLANCE_EVERY, left));
          left -= GLANCE_EVERY;
        }
        if (ev == -2) ev = -1;
        a = A_WALK; break;
      }
      case C_CALL:
        if (okCam && searchForYou()) {             // found you: happy, then watch you
          Serial.println("    found you!");
          say("yip2"); playTrick(WIGGLE); mind.social *= 0.3f; mind.lastContact = millis();
          ev = watchYou(6 + 6 * mind.trait[1]);
          a = A_CALL; break;
        }
        ev = turnBy(frand() < 0.5f ? -60 : 60);
        if (H.yipCall) { say("yip"); playTrick(BOUNCE); playTrick(WIGGLE); }   // golden: happy "where are you?"
        else { say("whine"); playTrick(WIGGLE); }
        if (ev < 0) ev = sitFor(4);                // give you a moment to answer
        a = A_CALL; break;
      case C_PLAY: {
        int i = mindPickTrick();
        Serial.printf("    %s (likes it %+.2f)\n", TRICK[PLAY[i]].name, mind.pref[i]);
        playTrick(PLAY[i], 0.85f + 0.3f * frand());
        mindPlayed(i);
        ev = sitFor(0.5f + frand());
        a = A_TRICK; break;
      }
      case C_REST:
        playTrick(SETTLE); playTrick(FLOP);
        while (mind.energy < 0.85f) {
          ev = sitFor(5);
          mindTick(5, A_REST);
          if (ev == 0 || (ev == 1 && mind.energy > 0.4f)) break;
          ev = -1;
          if (cmdAvailable() && cmdPeek() == 'x') break;
        }
        playTrick(GETUP); playTrick(STRETCH);
        lastT = millis();
        a = A_IDLE; break;                          // nap time already counted
      case C_LOOK:
        if (faceFresh(1500)) ev = watchYou(5 + 5 * mind.trait[1]);
        else playTrick(SNIFF);
        a = A_IDLE; break;
      default: if (H.wag) playTrick(WIGGLE); ev = sitFor(3 + 5 * frand()); a = A_IDLE; break;
    }
    float dt = (millis() - lastT) / 1000.0f; lastT = millis();
    mindTick(dt, a);
    react(ev);
    (void)t0;
  }
  mindSave();
  eyesOn = false;
  for (int i = 0; i < 4; i++) servoDeg(i, 0);
  delay(300); servosOff();
  Serial.println("mind mode off");
}

// ===== end cubi_mind.h =====


void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis(); while (!Serial && millis() - t0 < 4000) delay(10);
  const bool host = (bool)Serial;                 // false = switched on with no computer: straight to mind mode
  Serial.println("\n=== CUBI SELF-TEST ===");
  esp_reset_reason_t rr = esp_reset_reason();
  if (rr == ESP_RST_BROWNOUT) Serial.println("!!! Last reset was a BROWNOUT: battery low, bulk cap missing/backwards, or a bad ground.");

  float vb = batteryV();
  Serial.printf("  battery %.2f V (D2 divider)\n", vb);
  report("battery sense (3.3-4.25 V; ~0 V = divider or switch)", vb > 3.3f && vb < 4.25f);
  if (vb > 0.5f && vb < BATT_STOP_V) Serial.println("  ! battery low: charge on the left USB-C with the switch ON");

  okMic = initMic();     report("PDM mic on I2S0", okMic);
  okAmp = initAmp();     report("MAX98357A amp on I2S1", okAmp);
  okCam = initCamera();  report("camera (OV2640/OV3660, QVGA RGB565)", okCam);
  if (okCam) eyesBegin();
  okTof = initTof();     report("VL53L0X distance sensor", okTof);
  okServo = initServos(); report("4 servo PWM channels (LEDC 4-7)", okServo);

  int idle = touchRead(PIN_TOUCH);
  Serial.printf("  touch idle %d (touch the head: it should change by >20%%)\n", idle);

  if (okAmp) { tone_(880, 120); tone_(1320, 160); }
  if (okMic) { int q = micLevel(300); Serial.printf("  mic quiet level %d - clap in the next 2 s...\n", q);
               if (q > 16000) { micDeaf = true; Serial.println("  ! mic stuck near full scale: clap detection OFF"); }
               if (host) { int l = micLevel(2000); report("mic hears a clap", l > q * 4 && l > 2000); } }

  if (okServo) {
    Serial.println("  servos: each leg wiggles +-10 deg in turn (FL, FR, RL, RR)");
    for (int i = 0; i < 4; i++) { servoDeg(i, 10); delay(250); servoDeg(i, -10); delay(250); servoDeg(i, 0); delay(250); }
  }

  // Everything at once: the worst case for the power supply.
  if (host && okServo && okCam && okAmp) {
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
  steerLoad(); if (CUBI_STEER != 0) Serial.printf("  steer trim %+.2f (from flash)\n", CUBI_STEER);
  remoteBegin();
  Serial.println("\nCommands: w=walk 3 s  b=walk back  (/)=steer left/right  c=centre legs  C=servo centre 1500us  D=re-measure desk  R=reflex on/off  f=leg direction test  +/-=step size  [/]=step speed  k/K=snap  g=show gait  n=next gait pattern  y/Y=lean legs back/forward  T=AUTO-TUNE  o=legs off  d=distance  p=photo  t=tone  m=mic  v=battery  h=touch  L=lie down  U=get up  Z=goodnight  r=servo range  P=PUPPY MODE  M=MIND MODE (her own ideas)  e=eyes test  E=find camera orientation  j=swap eyes left/right  F=FOLLOW you  I=her personality  W=new personality  G=golden retriever");
  // Switched on without USB -> mind mode right away. With the Serial Monitor open: 5 s to send any key
  // (or press a phone button) to stay in command mode instead.
  if (!host) { delay(1000); mindMode(); }
  else {
    Serial.println("\nMIND MODE in 5 s - send any key to stay in command mode");
    uint32_t w = millis(); while (millis() - w < 5000 && !cmdAvailable()) delay(20);
    if (cmdAvailable()) { while (cmdAvailable()) cmdRead(); Serial.println("  command mode"); }
    else mindMode();
  }
}

void loop() {
  if (!cmdAvailable()) { delay(2); return; }   // idle: give Wi-Fi time
  char k = cmdRead();
  switch (k) {
    case '(': steerSet(CUBI_STEER + 0.05f); break;   // drifts RIGHT -> (
    case ')': steerSet(CUBI_STEER - 0.05f); break;   // drifts LEFT  -> )
    case 'w': walk(3, 1); break;
    case 'b': walk(3, -1); break;
    case 'c': for (int i = 0; i < 4; i++) servoDeg(i, 0); Serial.println("  legs centred: any leg not vertical -> set its trim_us"); break;
    case 'o': servosOff(); break;
    case 'f': Serial.println("  ALL legs 15 deg FORWARD (toward the eyes) for 2 s, then 15 deg BACK for 2 s. Any leg going the wrong way = flipped.");
      for (int i = 0; i < 4; i++) servoDeg(i, 15); delay(2000);
      for (int i = 0; i < 4; i++) servoDeg(i, -15); delay(2000);
      for (int i = 0; i < 4; i++) servoDeg(i, 0); delay(300); servosOff(); break;
    case '+': CUBI_AMP_DEG = min(15.0f, CUBI_AMP_DEG + 1); Serial.printf("  amp %.0f deg  freq %.1f Hz\n", CUBI_AMP_DEG, CUBI_FREQ_HZ); break;
    case '-': CUBI_AMP_DEG = max(4.0f, CUBI_AMP_DEG - 1); Serial.printf("  amp %.0f deg  freq %.1f Hz\n", CUBI_AMP_DEG, CUBI_FREQ_HZ); break;
    case ']': CUBI_FREQ_HZ = min(4.0f, CUBI_FREQ_HZ + 0.2f); Serial.printf("  amp %.0f deg  freq %.1f Hz\n", CUBI_AMP_DEG, CUBI_FREQ_HZ); break;
    case '[': CUBI_FREQ_HZ = max(0.6f, CUBI_FREQ_HZ - 0.2f); Serial.printf("  amp %.0f deg  freq %.1f Hz\n", CUBI_AMP_DEG, CUBI_FREQ_HZ); break;
    case 'D': { uint32_t sum = 0; int n = 0;
      for (int i = 0; i < 20; i++) { uint16_t d = tof.readRangeContinuousMillimeters(); if (!tof.timeoutOccurred() && d < 2000) { sum += d; n++; } }
      if (n >= 10) { floorMm = sum / n; Serial.printf("  new desk baseline %u mm (edge > %u, wall < %u)\n", floorMm, (unsigned)(floorMm * CUBI_EDGE_RATIO), (unsigned)(floorMm * CUBI_WALL_RATIO)); }
      else Serial.println("  no steady reading: stand her on a flat desk, nothing in front, and try again");
      break; }
    case 'R': reflexOn = !reflexOn; Serial.printf("  edge/wall reflex %s\n", reflexOn ? "ON" : "OFF - she WILL walk off edges, keep a hand ready"); break;
    case 'k': CUBI_FAST = max(0.10f, CUBI_FAST - 0.05f); Serial.printf("  fast-swing fraction %.2f (smaller = snappier forward swing)\n", CUBI_FAST); break;
    case 'K': CUBI_FAST = min(0.85f, CUBI_FAST + 0.05f); Serial.printf("  fast-swing fraction %.2f\n", CUBI_FAST); break;
    case 'g': Serial.printf("  gait: %s, amp %.0f deg, freq %.1f Hz, fast %.2f, stance %+.1f\n", GAIT_NAME[gaitIdx], CUBI_AMP_DEG, CUBI_FREQ_HZ, CUBI_FAST, stanceOff); break;
    case 'n': gaitIdx = (gaitIdx + 1) % 5; Serial.printf("  gait pattern: %s\n", GAIT_NAME[gaitIdx]); break;
    case 'T': autoTune(); break;
    case 'y': stanceOff = max(-15.0f, stanceOff - 2.5f); Serial.printf("  stance %+.1f deg (- = feet raked back)\n", stanceOff); break;
    case 'Y': stanceOff = min(15.0f, stanceOff + 2.5f); Serial.printf("  stance %+.1f deg\n", stanceOff); break;
    case 'C': for (int i = 0; i < 4; i++) ledcWrite(legs[i].pin, usToDuty(1500)); Serial.println("  all 4 servos at 1500 us (true centre): press legs on now, then send o"); break;
    case 'd': { uint16_t d = tof.readRangeContinuousMillimeters(); int v = tofVerdict(d, tof.timeoutOccurred());
                Serial.printf("  %u mm  %s\n", d, v == 1 ? "EDGE" : v == 2 ? "WALL" : "desk"); } break;
    case 'p': { camera_fb_t *fb = okCam ? esp_camera_fb_get() : nullptr; if (fb) { Serial.printf("  photo %u bytes\n", fb->len); esp_camera_fb_return(fb); } } break;
    case 't': tone_(660, 300); break;
    case 'm': { int16_t mb[256]; int lo = 32767, hi = -32768; long sum = 0, cnt = 0; uint32_t m0 = millis();
      while (millis() - m0 < 500) { size_t n = mic.readBytes((char *)mb, sizeof(mb)) / 2;
        for (size_t i = 0; i < n; i++) { lo = min(lo, (int)mb[i]); hi = max(hi, (int)mb[i]); sum += mb[i]; cnt++; } }
      Serial.printf("  mic min %d  max %d  avg %ld  samples %ld  (stuck line: min=max or all at +-32767; healthy: small swing round 0)\n", lo, hi, cnt ? sum / cnt : 0, cnt);
    } break;
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
    case 'e': eyesTest(); break;
    case 'E': eyesOrient(); break;
    case 'j': eyeFlip = !eyeFlip; eyesSave(); Serial.printf("  eyes left/right %s (saved)\n", eyeFlip ? "SWAPPED" : "normal"); break;
    case 'F': followMode(); break;
    case 'U': playTrick(UNTUCK); playTrick(GETUP); break;
  }
}
