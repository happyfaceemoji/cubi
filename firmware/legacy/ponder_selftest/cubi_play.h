// CUBI - puppy mode. Same tricks, mood rules and sounds as cubi_play.py (tested in the physics sim:
// 16 x 60 s of random taps / claps / hands near a desk edge, 0 falls). Included by ponder_selftest.ino after
// the servo, ToF, mic and amp helpers. Serial: P = puppy mode on, x = off, 1-9 = play one trick, L = lie down, U = get up.
#pragma once

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
    float a = constrain(CUBI_AMP_DEG * cubiSaw(CUBI_FREQ_HZ * t + ph[i]) + trim[i], -12.0f, 12.0f);
    q[i] = a * dir * ((i == 0 || i == 2) ? sideL : sideR);
  }
}
enum Trick { WIGGLE, PAW_PAT, BOUNCE, CHASE, ZOOMIES, SNIFF, STARTLE, STRETCH, SETTLE, FLOP, GETUP, TUCK, UNTUCK, N_TRICKS };
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
static bool clapped() {
  if (!okMic) return false;
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
    if (!Serial.available()) { delay(10); continue; }
    char c = Serial.read();
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
    if (Serial.available()) {
      char c = Serial.read();
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
