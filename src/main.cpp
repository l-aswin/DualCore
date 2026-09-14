// NitroWorks ECU - DualCore sprint - STEP 4: mutex-protected RobotState + snapshot
//                                            (pairing state machine core 0 / OLED core 1 unchanged)
//
// Step 3 made both cores do their real job (pairing state machine on core 0,
// OLED + RING-command emit on core 1) but shared the result as a handful of
// loose `volatile` scalars, no lock - deliberately naive. Each individual field
// was already safe (native-width scalar, one writer core, one reader core - the
// ESP32 doesn't tear those), but nothing stopped uiTask from combining fields
// read a few instructions apart into one inconsistent frame: it could read
// `linkState` from one btTask iteration and `connectedAtMs` from the next.
//
// Step 4 closes that gap without changing behaviour:
//
//   core 0 / btTask  - unchanged pairing state machine, but now writes into a
//     core-0-only `RobotState g_pending` as it goes (no lock needed - only
//     btTask ever touches it), then does ONE spinlock-protected struct copy
//     (`publishState()`) into the shared `g_state` at the tail of each loop
//     iteration. One critical section per iteration, not one per field.
//
//   core 1 / uiTask  - takes ONE spinlock-protected copy (`readState()`) at the
//     top of each frame into a local `RobotState snap`, then renders the whole
//     frame off `snap`. Every field in that frame is guaranteed to come from
//     the same btTask iteration.
//
// The lock is a `portMUX_TYPE` spinlock (`portENTER_CRITICAL` /
// `portEXIT_CRITICAL`), not a FreeRTOS semaphore/mutex - the protected section
// is just a fixed-size struct copy (a handful of scalars), so a spinlock avoids
// the scheduler overhead of a semaphore for something that fast, and ESP-IDF's
// portMUX_TYPE is specifically the primitive for a short, code-only critical
// section shared between the two cores.
//
//     Test rig for step 4 is OLED + the two buttons only - no ring, no TCU. The
//     `RING …` / `BUZZ …` bytes still go out UART1 TX (GPIO17) and are mirrored
//     to USB as `tx>`, but nothing receives them; RX bytes print as `rx<`
//     (jumper GPIO17->GPIO16 for a loopback check). That path is code, not
//     something this step verifies.
//
// powerState / S0 Battery Critical / the warning strip are NOT in this step -
// battery sensing is the BMS's (ECU-SPEC-001 rev 5) and the simulated
// powerState input lands in step 5. The status strip renders its `normal` state
// only; BATT_PCT_STUB stands in for a state-of-charge the ECU never actually
// receives.
//
// Verify (see docs/STEPS.md) - OLED + buttons only, behaviour identical to step 3:
//   1. boot bonded -> S1 Search (glyph + waves animating), ui on core 1
//   2. Pair button -> ~600 ms header-only blank -> S2 Pair, Mode LED off
//   3. controller connects -> S3 Connected (~1.5 s) -> S4 stick-check HUD;
//      wiggle a stick -> ACC / REV / TURN, ~400 ms linger back to the Idle glyph
//   4. disconnect -> back to S1 Search; bt heartbeat uninterrupted
//   5. Reset button -> S5 toast (~1 s) -> S2 Pair; NVS bond cleared
//   6. btTask / uiTask stack high-water recorded, both with headroom
//   7. lock overhead: bt poll (~5 ms) / ui frame (~166 ms) cadence unaffected

#include <Arduino.h>
#include <string.h>
#include <Preferences.h>
#include <Bluepad32.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// Set to 1 to make uiTask burn 3 s of core 1 every ~10 frames - btTask's
// BP32.update() must stay responsive through it. Ship at 0.
#define ISOLATION_TEST 1

// ---- pins --------------------------------------------------------------------
// Buttons keep the BluetoothPairing test-rig wiring (active-low, internal
// pull-up). ECU-SPEC-001 §2 puts Reset on GPIO14 and Pair on an input-only pin
// on the final board; the perfboard rig this sprint runs on uses 15 / 13.
constexpr int PIN_MODE_LED   = 2;    // HIGH = connected, LOW = searching/pairing - binary only, no blink pattern coded
constexpr int PIN_PAIR_BTN   = 15;   // Pair button   (HIGH -> LOW edge = press)
constexpr int PIN_RESET_BTN  = 13;   // Reset button  (clears the stored bond)
constexpr int PIN_I2C_SDA    = 21;   // SH1106 OLED
constexpr int PIN_I2C_SCL    = 22;
constexpr int PIN_UART1_RX   = 16;   // UART1 <- TCU  (ECU-SPEC-001 §2)
constexpr int PIN_UART1_TX   = 17;   // UART1 -> TCU  (RING / BUZZ commands)

// ---- tunables --------------------------------------------------------------
constexpr int32_t  AXIS_MAX        = 512;   // Bluepad32 stick full-scale
constexpr int32_t  S4_DEADZONE     = 60;    // ~12 % of AXIS_MAX (ECU-SPEC-002 §7.6 - tune on hw)
constexpr uint32_t S4_LINGER_MS    = 400;   // active axis re-centred -> hold before Idle
constexpr uint32_t BT_POLL_MS      = 5;     // BP32.update() cadence in btTask
constexpr uint32_t BT_HEARTBEAT_MS = 1000;
constexpr uint32_t UI_FRAME_MS     = 166;   // ~6 Hz (ECU-ADR-004)
constexpr uint32_t S3_HOLD_MS      = 1500;  // Connected screen dwell before S4
constexpr uint32_t RESET_TOAST_MS  = 1000;  // S5 dwell
constexpr uint32_t PAIR_BLANK_MS       = 600;  // header-only feedback frame after a Pair press
constexpr uint32_t PAIR_SETUP_DEFER_MS = 200;  // hold the BT key-wipe until that feedback frame is flushed
constexpr uint32_t BTN_DEBOUNCE_MS = 40;
constexpr int      BATT_PCT_STUB   = 82;    // no real SoC on the ECU (BMS owns it) - placeholder only

// ---- NVS (which controller this car is bonded to - ECU-SPEC-001 §8) --------
constexpr char NVS_NS[]         = "nitro-ecu";
constexpr char NVS_KEY_BONDED[] = "bonded";

// ---- link state ----------------------------------------------------------
enum LinkState : uint8_t { LINK_SEARCH = 0, LINK_PAIR = 1, LINK_CONNECTED = 2 };
static const char* LINK_NAME[] = {"SEARCH", "PAIR", "CONNECTED"};

// RING command token per link state (ECU-SPEC-001 §4 grammar).
static const char* ringCmdFor(LinkState s) {
  switch (s) {
    case LINK_SEARCH:    return "SEARCH_BLINK";
    case LINK_PAIR:      return "PAIR_BLINK";
    case LINK_CONNECTED: return "CONNECTED";
  }
  return "OFF";
}

// ---- cross-core shared state (core 0 writes, core 1 reads, mutex-guarded) ---
struct RobotState {
  LinkState linkState     = LINK_PAIR;
  bool      hasBond       = false;
  bool      connected     = false;
  int32_t   throttle      = 0;   // +forward / -reverse, ~AXIS_MAX scale
  int32_t   steer         = 0;   // +right / -left
  uint32_t  connectedAtMs  = 0;  // millis() of the last 0->1 connect edge
  uint32_t  resetToastAtMs = 0;  // millis() of the last Reset press (0 = none)
  uint32_t  pairBlankAtMs  = 0;  // millis() of the last Pair press (0 = none) - drives the S2 feedback frame
};

RobotState   g_state;                                   // the shared copy - touch only under g_stateMux
portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;  // spinlock guarding g_state

// core-0-only working copy - btTask reads/writes this freely with no locking
// (nothing else ever touches it), then flushes the whole thing into g_state
// under the spinlock once per loop iteration via publishState(). This keeps
// the critical section to a single struct copy instead of one per field.
RobotState g_pending;

void publishState() {
  portENTER_CRITICAL(&g_stateMux);
  g_state = g_pending;
  portEXIT_CRITICAL(&g_stateMux);
}

// core-1-only: one locked copy of g_state per uiTask frame. uiTask renders the
// whole frame off the returned snapshot, so every field it reads comes from
// the same btTask iteration.
RobotState readState() {
  RobotState snap;
  portENTER_CRITICAL(&g_stateMux);
  snap = g_state;
  portEXIT_CRITICAL(&g_stateMux);
  return snap;
}

// ---- Bluepad32 + pairing (core 0) ------------------------------------------
ControllerPtr controllers[BP32_MAX_GAMEPADS];

Preferences prefs;
bool     hasBondedController = false;   // mirror of the NVS flag, loaded in setup()
bool     pairingMode         = false;   // true = pairing window open (accept any controller)
uint32_t g_pairSetupPendingAt = 0;     // core-0 only: when to run the deferred forget-keys / enable-discovery

struct Button {
  int pin;
  int stable;
  int lastReading;
  uint32_t lastChangeMs;
};
Button pairBtn  = {PIN_PAIR_BTN,  HIGH, HIGH, 0};
Button resetBtn = {PIN_RESET_BTN, HIGH, HIGH, 0};

static void logStack(const char* who) {
  Serial.printf("%s  stack high-water: %u bytes free\n",
                who, (unsigned)uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t));
}

void initButton(Button& b) {
  pinMode(b.pin, INPUT_PULLUP);
  b.stable = b.lastReading = digitalRead(b.pin);
  b.lastChangeMs = millis();
}

// Returns true exactly once per press (debounced HIGH -> LOW edge).
bool buttonPressed(Button& b) {
  int reading = digitalRead(b.pin);
  if (reading != b.lastReading) {
    b.lastReading = reading;
    b.lastChangeMs = millis();
  }
  if (millis() - b.lastChangeMs >= BTN_DEBOUNCE_MS && reading != b.stable) {
    b.stable = reading;
    if (b.stable == LOW) return true;
  }
  return false;
}

bool pairingWindowOpen() { return pairingMode; }

void persistBonded(bool v) {
  size_t n = prefs.putBool(NVS_KEY_BONDED, v);
  hasBondedController = v;
  g_pending.hasBond = v;
  Serial.printf("persistBonded(%d) -> %u bytes written\n", v, (unsigned)n);
}

bool anyControllerConnected() {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controllers[i] && controllers[i]->isConnected()) return true;
  }
  return false;
}

void openPairingWindow() {
  // Flip the pending state FIRST so it wins over computeLinkState()'s result
  // later this same btTask iteration - both land in g_state together at the
  // single publishState() call at the loop tail, so core 1 still sees PAIR on
  // its very next frame; nothing here waits for computeLinkState().
  pairingMode = true;
  g_pending.linkState = LINK_PAIR;

  // Disconnect the active controller and (soon) forget the stored key, then open
  // up for one new controller. forgetBluetoothKeys() is all-or-nothing, so it
  // has to happen while the old key is the only one. hasBondedController is left
  // as-is - only cleared once a new controller actually bonds - so an abandoned
  // attempt still boots back into Search.
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controllers[i]) {
      controllers[i]->disconnect();
      controllers[i] = nullptr;
    }
  }
  digitalWrite(PIN_MODE_LED, LOW);

  // Defer forgetBluetoothKeys() + enableNewBluetoothConnections(true): both post
  // to the BT thread, whose NVS key-wipe (flash-sector erases) disables the
  // flash cache and freezes uiTask (GFX runs from flash, no PSRAM) for a few
  // hundred ms. Running them PAIR_SETUP_DEFER_MS later lets the S2 feedback
  // frame reach the panel first (see btTask). A real controller takes seconds
  // to pair, so entering discovery ~200 ms late is invisible.
  g_pairSetupPendingAt = millis();
  Serial.println("Pairing window open - waiting for a new controller.");
}

void closePairingWindow(const char* why) {
  BP32.enableNewBluetoothConnections(false);
  pairingMode = false;
  Serial.printf("Pairing window closed: %s\n", why);
}

// Reset: clear the stored bond FIRST (durable even if a later BP32 call blocks),
// then drop into an open pairing window. Also fires the S5 toast on core 1.
void resetBondedController() {
  Serial.println("Reset button - clearing stored bond");
  persistBonded(false);
  g_pending.resetToastAtMs = millis();
  openPairingWindow();
}

void pollButtons() {
  if (buttonPressed(pairBtn)) {
    Serial.println("Pair button pressed");
    // Intentional press feedback: blank everything below the header for
    // PAIR_BLANK_MS. Only when entering PAIR from SCAN / CONNECTED - a re-press
    // while already pairing just restarts discovery, no visual blank.
    // g_pending.linkState still holds the last PUBLISHED value here (this
    // iteration hasn't called publishState() yet), so this compares against
    // what core 1 actually last saw - same as reading g_state would.
    if (g_pending.linkState != LINK_PAIR) g_pending.pairBlankAtMs = millis();
    openPairingWindow();
  }
  if (buttonPressed(resetBtn)) {
    resetBondedController();
  }
}

void onConnectedController(ControllerPtr ctl) {
  bool bondedBefore = hasBondedController;
  int slot = -1;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controllers[i] == nullptr) { controllers[i] = ctl; slot = i; break; }
  }
  if (slot < 0) {
    Serial.println("Controller connected but no empty slot");
    return;
  }

  // Guard: enableNewBluetoothConnections(false) does not always reject unbonded
  // controllers on the BT-Classic path. A connect with no bond and no open
  // window should be impossible - drop it.
  if (!bondedBefore && !pairingWindowOpen()) {
    Serial.println("Rejecting unexpected controller (no bond, not pairing)");
    controllers[slot] = nullptr;
    ctl->disconnect();
    return;
  }

  Serial.printf("Controller connected, slot %d\n", slot);
  if (pairingWindowOpen()) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      if (i != slot && controllers[i]) {
        Serial.printf("Dropping stray controller in slot %d\n", i);
        controllers[i]->disconnect();
        controllers[i] = nullptr;
      }
    }
    persistBonded(true);   // authoritative-connect rule, ECU-SPEC-001 §8
    Serial.println("Bonded to this controller");
    closePairingWindow("new controller bonded");
  }
  digitalWrite(PIN_MODE_LED, HIGH);
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controllers[i] == ctl) {
      Serial.printf("Controller disconnected, slot %d\n", i);
      controllers[i] = nullptr;
      digitalWrite(PIN_MODE_LED, LOW);
      break;
    }
  }
}

// Reduce the whole gamepad to the two axes the S4 stick-check HUD cares about:
// throttle = left stick vertical (up = forward, so negate axisY), steer = axisX.
void publishSticks() {
  int32_t thr = 0, str = 0;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    ControllerPtr ctl = controllers[i];
    if (ctl && ctl->isConnected()) { thr = -ctl->axisY(); str = ctl->axisX(); break; }
  }
  g_pending.throttle = thr;
  g_pending.steer    = str;
}

LinkState computeLinkState() {
  if (anyControllerConnected())               return LINK_CONNECTED;
  if (pairingMode || !hasBondedController)     return LINK_PAIR;
  return LINK_SEARCH;
}

// ---- core 0: Bluetooth / pairing task -----------------------------------
void btTask(void*) {
  Serial.printf("bt  started on core %d\n", xPortGetCoreID());
  logStack("bt ");

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);

  if (hasBondedController) {
    BP32.enableNewBluetoothConnections(false);   // Search: reconnect the bonded pad only
    Serial.println("bt  booted bonded -> Search");
  } else {
    Serial.println("bt  booted unpaired -> Pair");
    openPairingWindow();
  }
  Serial.printf("bt  Bluepad32 setup done on core %d (fw %s)\n",
                xPortGetCoreID(), BP32.firmwareVersion());
  logStack("bt ");

  uint32_t beats = 0, lastHeartbeat = 0;
  bool wasConnected = false;

  for (;;) {
    pollButtons();

    bool dataUpdated = BP32.update();
    (void)dataUpdated;
    publishSticks();

    // Deferred pairing-window setup (see openPairingWindow): run the BT-thread
    // key-wipe + discovery-enable only once the S2 feedback frame has had time
    // to reach the panel.
    if (g_pairSetupPendingAt &&
        millis() - g_pairSetupPendingAt >= PAIR_SETUP_DEFER_MS) {
      g_pairSetupPendingAt = 0;
      BP32.forgetBluetoothKeys();
      BP32.enableNewBluetoothConnections(true);
      Serial.println("Pairing window: keys forgotten, accepting new connections");
    }

    bool nowConnected = anyControllerConnected();
    if (nowConnected && !wasConnected) g_pending.connectedAtMs = millis();
    wasConnected = nowConnected;
    g_pending.connected  = nowConnected;
    g_pending.linkState  = computeLinkState();

    // One critical-section struct copy for this whole iteration's changes -
    // see the RobotState comment above.
    publishState();

    uint32_t now = millis();
    if (now - lastHeartbeat >= BT_HEARTBEAT_MS) {
      lastHeartbeat = now;
      Serial.printf("bt  alive on core %d  link=%s  (beat %lu)\n",
                    xPortGetCoreID(), LINK_NAME[g_pending.linkState], (unsigned long)beats);
      if ((beats % 10) == 9) logStack("bt ");
      beats++;
    }

#if ISOLATION_TEST
    static bool spun = false;
    if (!spun && beats >= 7) {
      spun = true;
      Serial.println("bt  >>> busy-spinning 3 s (uiTask must keep rendering) <<<");
      uint32_t t0 = millis();
      while (millis() - t0 < 3000) { /* hog core 0 */ }
      Serial.println("bt  <<< spin done");
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(BT_POLL_MS));
  }
}

// ===========================================================================
// core 1: OLED rendering (ECU-SPEC-002) + RING/BUZZ command emit
// ===========================================================================
Adafruit_SH1106G display(128, 64, &Wire, -1);
bool g_oledOk = false;

// The OLED draw helpers below are a VERBATIM port of
// 06_Code_Sprints/BluetoothUIMockup/src/main.cpp - the pixel-accurate reference
// for ECU-SPEC-002, tuned on the real 1.3" SH1106. Do NOT re-derive them from
// the spec text; if a screen changes, change it in the mockup first, then copy
// it here. DualCore only supplies the live inputs: the per-frame RobotState
// snapshot's linkState picks the screen and the real gamepad sticks (throttle /
// steer) drive S4 via s4PickAxis().
#define W SH110X_WHITE

// Only two strip states: normal and warning. Critical takes the whole screen
// (S0) - step 5. Step 3 only ever passes BATT_NORMAL.
enum BattState { BATT_NORMAL, BATT_WARNING };

// ---- shared animation clocks (mockup) ------------------------------------
int  animWaves() { return (millis() / 300) % 4; }        // searching arcs: 0,1,2,3
bool animBlink() { return (millis() / 480) % 2 == 0; }   // "toggle every 3 frames"

// ---- draw helpers (verbatim from BluetoothUIMockup) --------------------------

// The Bluetooth glyph: 5 drawLine segments (stem, two shoulders, two crossing
// diagonals), centred at (cx,cy) with half-height h and half-width wd. Drawn
// twice at a 1px offset when bold (S3 variant); optional filled "connected" dot.
void drawBtGlyph(int cx, int cy, int h, int wd, bool bold, bool dot) {
  for (int o = 0; o < (bold ? 2 : 1); o++) {
    int d = o;  // 1px x-offset on the bold pass
    display.drawLine(cx + d, cy - h, cx + d, cy + h, W);            // stem
    display.drawLine(cx + d, cy - h, cx + wd + d, cy - h / 2, W);   // upper shoulder
    display.drawLine(cx + d, cy + h, cx + wd + d, cy + h / 2, W);   // lower shoulder
    display.drawLine(cx + wd + d, cy - h / 2, cx - wd + d, cy + h / 2, W);  // diagonal
    display.drawLine(cx + wd + d, cy + h / 2, cx - wd + d, cy - h / 2, W);  // diagonal
  }
  if (dot) display.fillCircle(cx, cy, 2, W);
}

// Right-opening "searching" arcs at r = step / 2*step / 3*step about a centre
// just right of the glyph. n arcs drawn; double-drawn (r and r+1) when solid (S3).
void drawWaves(int cx, int cy, int n, bool solid, int step) {
  for (int i = 1; i <= 3 && i <= n; i++) {
    int r = i * step;
    display.drawCircleHelper(cx, cy, r, 0x6, W);  // 0x6 = right half
    if (solid) display.drawCircleHelper(cx, cy, r + 1, 0x6, W);
  }
}

// One shared status-strip routine for S1 / S2 / S4. Ends by drawing the
// full-width divider at y = 20 (2px edge margin -> x 2..125).
void drawStatusStrip(BattState state, int pct, bool hudBtIcon) {
  display.setTextSize(2);
  if (state == BATT_NORMAL) {
    display.drawRect(2, 4, 32, 14, W);            // battery body
    display.fillRect(34, 8, 3, 6, W);             // nub
    int fw = (int)((pct / 100.0f) * 28);
    display.fillRect(4, 6, fw, 10, W);            // proportional charge
    display.setCursor(44, 3);
    display.print(pct);
    display.print("%");
  } else {  // BATT_WARNING
    display.drawRect(2, 3, 40, 16, W);            // bigger empty outline (solid)
    display.fillRect(42, 8, 3, 6, W);             // nub
    if (animBlink()) {                            // exclamation, centred in the outline
      display.fillRect(20, 6, 3, 6, W);           //   stem
      display.fillRect(20, 14, 3, 2, W);          //   dot
    }
    display.setCursor(48, 3);
    display.print("15%");                         // constant
  }

  if (hudBtIcon) drawBtGlyph(115, 10, 9, 6, true, true);  // connected icon (glyph + dot), right edge

  display.drawLine(2, 20, 125, 20, W);           // divider
}

// State word: static, size 3, x6, vertically centred in Area 2.
void drawWord(const char* word) {
  display.setTextSize(3);
  display.setCursor(6, 31);
  display.print(word);
}

// S1 / S1b / S2 / S2b share this: only word + strip state + pct differ.
void renderSearchLike(const char* word, BattState batt, int pct) {
  drawStatusStrip(batt, pct, false);
  drawWord(word);                                  // static, size 3, x6
  drawBtGlyph(97, 41, 17, 8, false, false);        // pushed right (leftmost ~x89)
  drawWaves(108, 41, animWaves(), false, 5);       // r 5/10/15 waves reach ~x123
}

// S3 Connected: READY + the "connected" counterpart of the searching mark.
void renderConnected() {
  display.setTextSize(2);
  display.setCursor(34, 6);
  display.print("READY");
  drawBtGlyph(55, 40, 17, 8, true, true);         // dot centred at (55,40)
  drawWaves(66, 40, 3, true, 5);
}

// small solid triangle, w wide / h tall, apex at the right (pointRight) or left.
void fillTri(int x, int y, int w, int h, bool pointRight) {
  for (int i = 0; i < w; i++) {
    int t  = pointRight ? (w - 1 - i) : i;          // tall end
    int hh = 1 + (h - 1) * t / (w - 1);
    display.drawFastVLine(x + i, y + (h - hh) / 2, hh, W);
  }
}

// ---- S4 axis selection - DualCore-specific (real sticks, not the mockup timer) --
// One axis owns Area 2; ~400 ms linger back to Idle (ECU-SPEC-002 §4).
enum S4Axis : uint8_t { S4_IDLE, S4_ACC, S4_REV, S4_TURN_L, S4_TURN_R };

S4Axis s4PickAxis(int32_t thr, int32_t str) {
  static S4Axis   held     = S4_IDLE;
  static uint32_t lingerAt = 0;

  bool thrActive = abs(thr) >= S4_DEADZONE;
  bool strActive = abs(str) >= S4_DEADZONE;

  S4Axis want = held;
  if (thrActive || strActive) {
    // Both crossing together from Idle: throttle wins (§4).
    if (thrActive && (!strActive || held == S4_ACC || held == S4_REV || held == S4_IDLE))
      want = (thr > 0) ? S4_ACC : S4_REV;
    else
      want = (str > 0) ? S4_TURN_R : S4_TURN_L;
    lingerAt = 0;
  } else if (held != S4_IDLE) {
    if (lingerAt == 0) lingerAt = millis();
    if (millis() - lingerAt >= S4_LINGER_MS) { want = S4_IDLE; lingerAt = 0; }
  }
  held = want;
  return held;
}

// S4 Stick-check HUD - mockup geometry (BX/BY/BW/BH, label + fillTri arrow,
// right-aligned value, ACC-left / REV-right / TURN centre-out, idle gamepad
// glyph). The state comes from s4PickAxis() and the magnitude `m` from the live
// axis rescaled past the deadzone, replacing the mockup's synthetic sine.
// thr/str come from this frame's RobotState snapshot (see uiTask).
void drawS4(int32_t thr, int32_t str) {
  drawStatusStrip(BATT_NORMAL, BATT_PCT_STUB, true);
  S4Axis axis = s4PickAxis(thr, str);

  if (axis == S4_IDLE) {                                    // idle gamepad glyph
    display.drawRoundRect(40, 32, 48, 20, 8, W);            // gamepad body
    display.fillRect(48, 37, 4, 10, W);                     // d-pad "+", left-justified,
    display.fillRect(45, 40, 10, 4, W);                     //   centred on the body midline
    display.fillCircle(80, 38, 2, W);                       // face buttons (vertical pair)
    display.fillCircle(80, 46, 2, W);
    return;
  }

  const int BX = 4, BY = 40, BW = 120, BH = 16;
  int32_t raw = (axis == S4_ACC || axis == S4_REV) ? thr : str;
  float m = (float)(abs(raw) - S4_DEADZONE) / (float)(AXIS_MAX - S4_DEADZONE);
  m = constrain(m, 0.0f, 1.0f);

  const char* label = (axis == S4_ACC) ? "ACC" : (axis == S4_REV) ? "REV"
                    : (axis == S4_TURN_R) ? "TURN R" : "TURN L";

  display.setTextSize(2);
  display.setCursor(4, 23);
  display.print(label);
  if (axis == S4_ACC) fillTri(4 + (int)strlen(label) * 12 + 2, 25, 7, 9, true);
  if (axis == S4_REV) fillTri(4 + (int)strlen(label) * 12 + 2, 25, 7, 9, false);

  char val[6];
  snprintf(val, sizeof(val), "%d%%", (int)(m * 100));
  display.setCursor(124 - (int)strlen(val) * 12, 23);
  display.print(val);

  display.drawRect(BX, BY, BW, BH, W);
  if (axis == S4_ACC) {                             // ACC - anchored left
    display.fillRect(BX + 2, BY + 2, (int)((BW - 4) * m), BH - 4, W);
  } else if (axis == S4_REV) {                      // REV - anchored right
    int w = (int)((BW - 4) * m);
    display.fillRect(BX + BW - 2 - w, BY + 2, w, BH - 4, W);
  } else {                                          // TURN - centre-out from the zero tick
    int cx = BX + BW / 2;
    display.drawFastVLine(cx, BY - 2, BH + 4, W);
    int w = (int)((BW / 2 - 4) * m);
    if (axis == S4_TURN_R) display.fillRect(cx + 2, BY + 2, w, BH - 4, W);
    else                   display.fillRect(cx - 2 - w, BY + 2, w, BH - 4, W);
  }
}

// S5 Reset toast (ECU-SPEC-002 §4) - no mockup entry; growing dots per §3.3.
void drawResetToast() {
  int dots = (millis() / 300) % 3 + 1;
  char buf[10] = "RESET";
  for (int i = 0; i < dots; i++) strcat(buf, ".");
  display.setTextSize(2);
  display.setCursor(18, 24);
  display.print(buf);
}

// ---- UART1 command emit (loopback / serial-print stub) --------------------
void emitCmd(const char* cmd) {
  Serial1.println(cmd);
  Serial.printf("tx> %s\n", cmd);
}

void pumpUart1Rx() {
  static char line[64];
  static size_t n = 0;
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') {
      if (n) { line[n] = 0; Serial.printf("rx< %s\n", line); n = 0; }
    } else if (n < sizeof(line) - 1) {
      line[n++] = c;
    }
  }
}

// ---- core 1: OLED + command task ----------------------------------------
void uiTask(void*) {
  Serial.printf("ui  started on core %d\n", xPortGetCoreID());
  logStack("ui ");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  g_oledOk = display.begin(0x3C, true);
  Serial.printf("ui  SH1106 begin: %s\n", g_oledOk ? "ok" : "FAILED (check wiring/addr)");
  if (g_oledOk) {
    display.setTextColor(SH110X_WHITE);
    display.clearDisplay();
    display.display();
  }

  LinkState lastEmitted = (LinkState)255;   // force a first emit
  uint32_t frames = 0;

  for (;;) {
    // One locked copy for the whole frame - every field below comes from the
    // same btTask iteration (see publishState()/readState()).
    RobotState snap = readState();

    // ---- pick the screen (ECU-SPEC-002 §5), powerState omitted (step 5) ----
    LinkState link   = snap.linkState;
    bool connected   = snap.connected;
    uint32_t connAt  = snap.connectedAtMs;
    uint32_t resetAt = snap.resetToastAtMs;
    uint32_t pairAt  = snap.pairBlankAtMs;
    uint32_t now     = millis();

    bool showReset = resetAt != 0 && (now - resetAt) < RESET_TOAST_MS;
    bool pairBlank = pairAt  != 0 && (now - pairAt)  < PAIR_BLANK_MS;

    // ---- RING / BUZZ emit on link-state change (this sprint: uiTask owns TX) --
    if (link != lastEmitted) {
      char cmd[24];
      snprintf(cmd, sizeof(cmd), "RING %s", ringCmdFor(link));
      emitCmd(cmd);
      if (link == LINK_CONNECTED) emitCmd("BUZZ CONNECT");
      lastEmitted = link;
    }
    pumpUart1Rx();

    // ---- render ----
    if (g_oledOk) {
      display.clearDisplay();
      if (showReset) {
        drawResetToast();
      } else if (link == LINK_CONNECTED) {
        if (now - connAt < S3_HOLD_MS) renderConnected();                    // S3
        else                           drawS4(snap.throttle, snap.steer);    // S4 stick-check HUD
      } else if (link == LINK_PAIR) {
        if (pairBlank)
          drawStatusStrip(BATT_NORMAL, BATT_PCT_STUB, false);   // header only - Pair-press feedback
        else
          renderSearchLike("PAIR", BATT_NORMAL, BATT_PCT_STUB); // S2
      } else {
        renderSearchLike("SCAN", BATT_NORMAL, BATT_PCT_STUB);   // S1
      }
      display.display();
    }
    (void)connected;

    if ((frames % 30) == 29) logStack("ui ");

#if ISOLATION_TEST
    if ((frames % 60) == 30) {
      Serial.println("ui  >>> busy-spinning 3 s (btTask must keep polling) <<<");
      uint32_t t0 = millis();
      while (millis() - t0 < 3000) { /* hog core 1 */ }
      Serial.println("ui  <<< spin done");
    }
#endif

    frames++;
    vTaskDelay(pdMS_TO_TICKS(UI_FRAME_MS));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_MODE_LED, OUTPUT);
  digitalWrite(PIN_MODE_LED, LOW);
  initButton(pairBtn);
  initButton(resetBtn);

  // UART1 to the TCU - RX 16 / TX 17 (ECU-SPEC-001 §2). Jumper 17->16 for loopback.
  Serial1.begin(115200, SERIAL_8N1, PIN_UART1_RX, PIN_UART1_TX);

  prefs.begin(NVS_NS, false);
#ifdef NITRO_QA_RESET_NVS
  prefs.clear();
  Serial.println("NITRO_QA_RESET_NVS: cleared stored bond");
#endif
  hasBondedController = prefs.getBool(NVS_KEY_BONDED, false);
  g_pending.hasBond   = hasBondedController;
  g_pending.linkState = hasBondedController ? LINK_SEARCH : LINK_PAIR;
  publishState();   // both tasks are created below, but seed g_state before they run either way

  Serial.println();
  Serial.println("NitroWorks ECU - DualCore sprint - STEP 4 (mutex-protected RobotState + snapshot)");
  Serial.printf("setup() runs on core %d\n", xPortGetCoreID());
  Serial.printf("Boot: NVS bonded flag = %d\n", hasBondedController);
  Serial.printf("free heap before tasks: %u B  (largest block %u B)\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap());

  xTaskCreatePinnedToCore(btTask, "bt", 8192, nullptr, 3, nullptr, 0);  // core 0
  xTaskCreatePinnedToCore(uiTask, "ui", 4096, nullptr, 1, nullptr, 1);  // core 1
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));   // all work is in the two pinned tasks
}
