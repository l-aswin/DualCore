// NitroWorks ECU - DualCore sprint - STEP 1: two tasks, one per core
//
// Goal of this step: the skeleton only. setup() creates two FreeRTOS tasks
// pinned to opposite cores and returns; loop() does nothing. Each task prints
// which core it is running on plus a heartbeat, so the split is visible in the
// serial monitor.
//
//   btTask -> core 0  (will own Bluepad32 + the gamepad from step 2 on)
//   uiTask -> core 1  (will own the OLED + LED status pattern from step 3 on)
//
// The user's "Core 1 / Core 2" == the ESP32's core 0 / core 1. ECU-ADR-004 pins
// Bluepad32 to core 0 and all application tasks to core 1; here btTask sits on
// core 0 with the (future) BT stack and uiTask takes core 1.
//
// Verify (see docs/STEPS.md):
//   1. monitor shows "bt  alive on core 0" and "ui  alive on core 1" steadily
//   2. the stack high-water marks printed at boot leave comfortable headroom
//   3. ISOLATION_TEST 1 -> uiTask busy-spins 3 s on a timer; btTask's heartbeat
//      must keep ticking straight through it (and swap the flag to test the
//      other direction)

#include <Arduino.h>

// Set to 1 to make uiTask burn the CPU for 3 s every ~10 s (step-1 isolation
// test). btTask must stay responsive through the spin. Ship at 0.
#define ISOLATION_TEST 0

constexpr int PIN_BT_HEARTBEAT_LED = 2;   // btTask toggles this ~1 Hz (GPIO2 Mode LED)

static void logStack(const char* who) {
  // High-water mark = smallest free stack seen so far, in words (4 bytes each).
  Serial.printf("%s  stack high-water: %u words free\n",
                who, (unsigned)uxTaskGetStackHighWaterMark(nullptr));
}

// ---- core 0: will become the Bluetooth / gamepad task ----------------------
void btTask(void*) {
  Serial.printf("bt  started on core %d\n", xPortGetCoreID());
  logStack("bt ");

  uint32_t beats = 0;
  for (;;) {
    digitalWrite(PIN_BT_HEARTBEAT_LED, beats & 1);
    Serial.printf("bt  alive on core %d  (beat %lu)\n",
                  xPortGetCoreID(), (unsigned long)beats);
    if ((beats % 10) == 9) logStack("bt ");
    beats++;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ---- core 1: will become the OLED / LED status task -----------------------
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
      Serial.println("ui  >>> busy-spinning 3 s (btTask must keep beating) <<<");
      uint32_t t0 = millis();
      while (millis() - t0 < 3000) { /* hog the core */ }
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
  Serial.println("NitroWorks ECU - DualCore sprint - STEP 1 (two tasks, one per core)");
  Serial.printf("setup() runs on core %d\n", xPortGetCoreID());

  xTaskCreatePinnedToCore(btTask, "bt", 8192, nullptr, 3, nullptr, 0);  // core 0
  xTaskCreatePinnedToCore(uiTask, "ui", 4096, nullptr, 1, nullptr, 1);  // core 1
}

void loop() {
  // Everything runs in the two pinned tasks; keep the Arduino loopTask idle.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
