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
#pragma once
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
enum Act { A_WALK, A_TRICK, A_CALL, A_REST, A_IDLE };
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
enum Choice { C_EXPLORE, C_CALL, C_PLAY, C_REST, C_LOOK, C_IDLE, N_CHOICE };
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
static int senseEvent() {                          // -1 nothing, 0 tap, 1 clap, 2 hand close, 3 hand pulled away
  if (tapped()) return 0;
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
static void react(int ev) {
  if (ev < 0) return;
  if (H.greet && ev != 1 && millis() - mind.lastContact > GREET_AFTER_MS) {      // you're back!
    Serial.println("  you're back!");
    mind.excite = 1; mind.social = 0;
    say("yip2"); playTrick(WIGGLE); playTrick(BOUNCE); playTrick(WIGGLE);
  }
  mind.lastContact = millis();
  if (ev == 0) { mindStimulus(0); playTrick(WIGGLE, 0.9f + 0.2f * frand()); }
  else if (ev == 1) { mindStimulus(1); playTrick(mind.excite <= 0.4f && H.startles ? STARTLE : BOUNCE); }
  else { mindStimulus(2); playTrick(ev == 2 ? PAW_PAT : CHASE); }
}

static void mindMode() {
  mindLoad();
  Serial.println("MIND MODE: she does what she wants. Tap her head, clap, wave a hand. x = stop, I = what's she thinking.");
  mindPrint();
  touchIdle = touchRead(PIN_TOUCH);
  say("yip2");
  uint32_t lastT = millis();
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
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
          if (Serial.available() && Serial.peek() == 'x') break;
        }
        playTrick(GETUP); playTrick(STRETCH);
        lastT = millis();
        a = A_IDLE; break;                          // nap time already counted
      case C_LOOK: playTrick(SNIFF); a = A_IDLE; break;
      default: if (H.wag) playTrick(WIGGLE); ev = sitFor(3 + 5 * frand()); a = A_IDLE; break;
    }
    float dt = (millis() - lastT) / 1000.0f; lastT = millis();
    mindTick(dt, a);
    react(ev);
    (void)t0;
  }
  mindSave();
  for (int i = 0; i < 4; i++) servoDeg(i, 0);
  delay(300); servosOff();
  Serial.println("mind mode off");
}
