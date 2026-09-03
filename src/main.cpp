// NitroWorks ECU - DualCore sprint - STEP 2: Bluepad32 owned by the core-0 task
//
// Step 1 stood up two pinned tasks (btTask/core 0, uiTask/core 1) with nothing
// but heartbeats. Step 2 brings the real Bluepad32 stack up *inside btTask on
// core 0* and reads a gamepad. No pairing state machine yet (step 3): btTask
// just accepts any controller and dumps its input. uiTask is still the step-1
// frame-counter stub.
//
// Why init BT from btTask and not setup(): setup()/loop() are the Arduino
// loopTask, which runs on core 1. ECU-ADR-004 pins the BT stack to core 0, so
// BP32.setup() has to run once btTask is already executing on core 0.
//
// Verify (see docs/STEPS.md):
//   1. Bluepad32 init logs from btTask on core 0, no watchdog reset, ui keeps ticking
//   2. controller connects -> Mode LED solid, dumpGamepad streams on stick input
//   3. ui frame cadence unaffected by connect / disconnect / active input
//   4. disconnect handled cleanly, btTask heartbeat uninterrupted
//   5. record bt stack high-water with the BT stack up
//   6. ISOLATION_TEST 1 -> neither core's spin stalls the other

#include <Arduino.h>
#include <Bluepad32.h>

// Set to 1 to make uiTask burn the CPU for 3 s every ~10 s (isolation test).
// btTask's BP32.update() must stay responsive through the spin. Ship at 0.
#define ISOLATION_TEST 1

constexpr int PIN_BT_HEARTBEAT_LED = 2;   // btTask toggles this ~1 Hz (GPIO2 Mode LED)

constexpr int32_t STICK_DEADZONE = 24;    // ignore idle axis jitter / rest offsets

constexpr uint32_t BT_POLL_MS = 5;        // BP32.update() cadence in btTask
constexpr uint32_t BT_HEARTBEAT_MS = 1000;

ControllerPtr controllers[BP32_MAX_GAMEPADS];

static void logStack(const char* who) {
  // High-water mark = smallest free stack seen so far, in words (4 bytes each).
  Serial.printf("%s  stack high-water: %u bytes free\n",
                who, (unsigned)uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t));
}

// ---- Bluepad32 callbacks (run on the BP32 task) --------------------------
// Step 2 has no pairing window / NVS bond - accept any controller into the
// first free slot. That logic comes back in step 3.
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controllers[i] == nullptr) {
      controllers[i] = ctl;
      Serial.printf("Controller connected, slot %d\n", i);
      digitalWrite(PIN_BT_HEARTBEAT_LED, HIGH);   // solid = connected
      return;
    }
  }
  Serial.println("Controller connected but no empty slot");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (controllers[i] == ctl) {
      Serial.printf("Controller disconnected, slot %d\n", i);
      controllers[i] = nullptr;
      return;
    }
  }
}

// True while any control is off its neutral position: a stick/trigger past the
// deadzone, or any button / d-pad pressed. Idle sticks (including the ~-4 LY/RY
// zero offset) stay under the deadzone and read as inactive.
bool inputActive(ControllerPtr ctl) {
  if (ctl->buttons() || ctl->miscButtons() || ctl->dpad()) return true;
  return abs(ctl->axisX())    > STICK_DEADZONE ||
         abs(ctl->axisY())    > STICK_DEADZONE ||
         abs(ctl->axisRX())   > STICK_DEADZONE ||
         abs(ctl->axisRY())   > STICK_DEADZONE ||
         abs(ctl->brake())    > STICK_DEADZONE ||
         abs(ctl->throttle()) > STICK_DEADZONE;
}

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "axes: LX=%4d LY=%4d RX=%4d RY=%4d | brake=%4d throttle=%4d | "
    "buttons=0x%04x dpad=0x%02x\n",
    ctl->axisX(), ctl->axisY(), ctl->axisRX(), ctl->axisRY(),
    ctl->brake(), ctl->throttle(),
    ctl->buttons(), ctl->dpad()
  );
}

// ---- core 0: Bluetooth / gamepad task ------------------------------------
void btTask(void*) {
  Serial.printf("bt  started on core %d\n", xPortGetCoreID());
  logStack("bt ");

  // BT stack init MUST happen here (core 0), not in setup() (core 1).
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);
  BP32.enableNewBluetoothConnections(true);   // step 2: accept anyone
  Serial.printf("bt  Bluepad32 setup done on core %d (fw %s)\n",
                xPortGetCoreID(), BP32.firmwareVersion());
  logStack("bt ");

  uint32_t beats = 0;
  uint32_t lastHeartbeat = 0;
#if ISOLATION_TEST
  bool spun = false;
#endif

  for (;;) {
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
      for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        ControllerPtr ctl = controllers[i];
        if (ctl && ctl->isConnected() && ctl->hasData() && inputActive(ctl)) {
          dumpGamepad(ctl);
        }
      }
    }

    uint32_t now = millis();
    if (now - lastHeartbeat >= BT_HEARTBEAT_MS) {
      lastHeartbeat = now;
      Serial.printf("bt  alive on core %d  (beat %lu)\n",
                    xPortGetCoreID(), (unsigned long)beats);
      if ((beats % 10) == 9) logStack("bt ");
      beats++;
    }

#if ISOLATION_TEST
    // Flip the spin to this task to check that a hog on core 0 doesn't stall ui.
    if (!spun && beats >= 7) {
      spun = true;
      Serial.println("bt  >>> busy-spinning 3 s (uiTask must keep counting) <<<");
      uint32_t t0 = millis();
      while (millis() - t0 < 3000) { /* hog core 0 */ }
      Serial.println("bt  <<< spin done");
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(BT_POLL_MS));
  }
}

// ---- core 1: OLED / LED status task (still the step-1 stub) ---------------
void uiTask(void*) {
  Serial.printf("ui  started on core %d\n", xPortGetCoreID());
  logStack("ui ");

  uint32_t frames = 0;
  for (;;) {
    Serial.printf("ui  alive on core %d  (frame %lu)\n",
                  xPortGetCoreID(), (unsigned long)frames);
    if ((frames % 10) == 9) logStack("ui ");

#if ISOLATION_TEST
    if ((frames % 10) == 5) {
      Serial.println("ui  >>> busy-spinning 3 s (btTask must keep polling) <<<");
      uint32_t t0 = millis();
      while (millis() - t0 < 3000) { /* hog core 1 */ }
      Serial.println("ui  <<< spin done");
    }
#endif

    frames++;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_BT_HEARTBEAT_LED, OUTPUT);
  digitalWrite(PIN_BT_HEARTBEAT_LED, LOW);

  Serial.println();
  Serial.println("NitroWorks ECU - DualCore sprint - STEP 2 (Bluepad32 on the core-0 task)");
  Serial.printf("setup() runs on core %d\n", xPortGetCoreID());

  Serial.printf("free heap before tasks: %u B  (largest block %u B, min ever %u B)\n",
                (unsigned)ESP.getFreeHeap(),
                (unsigned)ESP.getMaxAllocHeap(),
                (unsigned)ESP.getMinFreeHeap());

  xTaskCreatePinnedToCore(btTask, "bt", 8192, nullptr, 3, nullptr, 0);  // core 0
  xTaskCreatePinnedToCore(uiTask, "ui", 4096, nullptr, 1, nullptr, 1);  // core 1
}

void loop() {
  // Everything runs in the two pinned tasks; keep the Arduino loopTask idle.
  // Min 10 ms delay - without it core 1 sits at 100% and trips the watchdog.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
