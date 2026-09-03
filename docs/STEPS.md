# DualCore — staged build log

This sprint splits the ECU firmware across the ESP32's two cores (ECU-ADR-004):
**core 0** = Bluepad32 + gamepad (`btTask`), **core 1** = OLED + `led_pattern`
command emit (`uiTask`). It clones `BluetoothPairing` and adds the core split, then
wires the OLED to real Bluetooth events and real stick input, then adds the
`powerState` input and the CRITICAL response.

> **Rev 5 note (2026-09-01):** the WS2812B ring + buzzer moved to the TCU
> and battery sensing moved to the BMS (ECU-SPEC-001 rev 5). So `uiTask` no longer
> *animates* LEDs — it emits `RING …` / `BUZZ …` commands over UART1 — and there is
> no on-ECU `battTask`. Steps 3 and 5 below reflect that.

The user's "Core 1 / Core 2" = the ESP32's core 0 / core 1.

Each step lives on its own branch (`step-N-...`), branched off the previous
step's merged tip. Many commits per branch is fine. Merge to `main` when the
step's verification passes, then tag the merge `step-N`.

The sprint's real deliverable is the **G4 verdict** (ECU-SPEC-001 §9): does the
core split hold timing with the OLED redrawing? Recorded at the bottom.

---

## Step 1 — two tasks, one per core  · branch `step-1-two-cores`

**What changed**

- New PlatformIO project, `platformio.ini` cloned from `BluetoothPairing` (Bluepad32
  custom core + the short `core_dir` / `build_dir` paths for the Windows
  command-line-length limit) with the Adafruit SH110X / GFX libs added for the OLED.
- `src/main.cpp`: `setup()` creates two pinned FreeRTOS tasks and returns.
  - `btTask` — core 0, priority 3, 8 KB stack. Prints its core + a heartbeat
    every 1 s, toggles GPIO2.
  - `uiTask` — core 1, priority 1, 4 KB stack. Prints its core + a frame counter
    every 1 s.
  - `loop()` just `vTaskDelay`s — all work is in the two tasks.
- `ISOLATION_TEST` compile flag: when 1, `uiTask` busy-spins 3 s every ~10 s so
  you can watch `btTask` keep beating through it.

**How to verify**

```
$env:PATH += ";$env:USERPROFILE\.platformio\penv\Scripts"; pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor
```

1. Monitor shows `bt  alive on core 0` and `ui  alive on core 1` interleaved,
   once a second each, steadily.
2. Boot lines print each task's stack high-water mark — record the numbers below
   as the baseline; both should have comfortable headroom (thousands of words).
3. Rebuild with `-D ISOLATION_TEST=1` (or flip the `#define`). During `uiTask`'s
   3 s spin, `btTask`'s `beat N` lines must keep incrementing once a second with
   no stall. Flip the spin into `btTask` to check the other direction, then set
   the flag back to 0.

**Results** _(fill in after running on hardware)_

- [ ] both tasks report the expected cores - yes
- [ ] stack high-water: bt = 1667 words, ui = 624 words
- [ ] isolation test: btTask kept beating through uiTask's spin — yes
- notes:
---

## Step 2 — Bluepad32 owned by the core-0 task  · branch `step-2-bluepad32`

Branched off the `step-1` tag. Bring the real Bluepad32 stack up **inside `btTask`
on core 0** and read a gamepad. No pairing state machine yet (step 3); `uiTask`
stays the step-1 stub. The point of the step is to show the BT stack initialises
and runs on the pinned core-0 task without disturbing `uiTask` on core 1.

**What changed**

- `platformio.ini`: unchanged — the Bluepad32 custom core was already in place from
  step 1.
- `src/main.cpp`:
  - `#include <Bluepad32.h>`; `controllers[BP32_MAX_GAMEPADS]`, the
    `onConnectedController` / `onDisconnectedController` handlers, and
    `inputActive()` / `dumpGamepad()` / `STICK_DEADZONE` copied from
    `BluetoothPairing` without the NVS / pairing-window code.
  - `BP32.setup()` moved out of `setup()` and into `btTask`: `setup()` runs on the
    Arduino loopTask (core 1), and ECU-ADR-004 pins the BT stack to core 0, so init
    must happen once `btTask` is running. `btTask` then calls
    `enableVirtualDevice(false)` + `enableNewBluetoothConnections(true)` and drops
    into a fast loop of `BP32.update()`, printing `dumpGamepad()` on
    `hasData() && inputActive(ctl)`.
  - btTask loop delay 1000 ms → ~5 ms so `BP32.update()` is serviced promptly; the
    heartbeat LED + stack/core log move behind a 1 s `millis()` check.
  - `uiTask` and `loop()` untouched. `ISOLATION_TEST` kept.

**How to verify**

```
$env:PATH += ";$env:USERPROFILE\.platformio\penv\Scripts"; pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor
```

1. Boot: Bluepad32 init logs from `btTask` on core 0, no `Guru Meditation` /
   watchdog reset, `ui  alive on core 1` keeps ticking.
2. Connect a controller — `Controller connected, slot 0`, Mode LED (GPIO2) solid,
   `dumpGamepad` streams while a stick moves and stops on release.
3. `ui  alive on core 1  (frame N)` holds its steady 1 Hz cadence through connect,
   disconnect and active gamepad input — no skipped or bunched frames.
4. Power the controller off — `Controller disconnected, slot 0`, Mode LED drops,
   `btTask` heartbeat uninterrupted.
5. Record the `bt` stack high-water with the BT stack up; confirm headroom.
6. `-D ISOLATION_TEST=1`: `uiTask`'s 3 s spin must not stall `BP32.update()`, and
   flipping the spin into `btTask` must not stall `uiTask`. Set the flag back to 0.

**Results** _(fill in after running on hardware)_

- [ yes] Bluepad32 init runs on core 0 from `btTask`, no watchdog reset
- [ yes] controller connects, `dumpGamepad` streams on input, stops on release
- [ yes] `uiTask` frame cadence unaffected by connect / disconnect / input
- [ yes] disconnect handled cleanly, heartbeat uninterrupted
- [ yes] stack high-water: bt = 1602 words, ui = 616 words
- [ yes] isolation test: BT poll and `uiTask` each ride through the other's spin
- notes:

## Step 3 — pairing state machine (core 0) + OLED / ring-command emit (core 1)  · branch `step-3-pairing-state-machine+OLED-display`

Branched off the `step-2` tag. core 0 runs the full pairing state machine; core 1
renders the OLED screens (ECU-SPEC-002) **and** emits the matching `RING …`
command on each `linkState` change (no local pixels — the ring is on the TCU).
For this sprint the TCU end is a serial-print / loopback stub.

**What changed**

- `platformio.ini`: unchanged — `Adafruit SH110X` / `GFX` were already in from step 1,
  `Preferences` + `Wire` are framework libs.
- `src/main.cpp`:
  - **core 0 / `btTask`** — the pairing state machine cloned from the
    `BluetoothPairing` sprint: NVS `nitro-ecu` / `bonded` flag, debounced Pair
    (GPIO15) + Reset (GPIO13) buttons, `openPairingWindow()` /
    `closePairingWindow()`, and the "reject unexpected controller (no bond, not
    pairing)" guard in `onConnectedController`. `setup()`'s boot branch (bonded →
    Search / `enableNewBluetoothConnections(false)`; unbonded → auto Pair) moved
    into `btTask` since the BT stack must come up on core 0.
  - `btTask` each loop polls the buttons, calls `BP32.update()`, then reduces the
    gamepad to `computeLinkState()` (`SEARCH` / `PAIR` / `CONNECTED`) + the two
    S4 axes (`throttle` = −axisY, `steer` = axisX) and publishes them.
  - **Pair-press transition** — `openPairingWindow()` flips `linkState` to `PAIR`
    immediately but **defers** `forgetBluetoothKeys()` +
    `enableNewBluetoothConnections(true)` by `PAIR_SETUP_DEFER_MS` (200 ms): the
    BT thread's NVS key-wipe stalls core 1's flash reads, so the deferral lets
    the OLED paint first. Pressing Pair (from SCAN/CONNECTED) also stamps
    `g_pairBlankAtMs`, and `uiTask` renders a **header-only frame** (battery
    strip + divider, blank below) for `PAIR_BLANK_MS` (600 ms) as deliberate
    press feedback before S2 PAIR. A re-press while already pairing shows no
    blank.
  - **Cross-core sharing is naive on purpose** — a handful of `volatile` scalars
    (`g_linkState`, `g_hasBond`, `g_connected`, `g_throttle`, `g_steer`,
    `g_connectedAtMs`, `g_resetToastAtMs`, `g_pairBlankAtMs`), core 0 writes,
    core 1 reads, no lock.
    Step 4a promotes this to a struct; 4b adds the mutex + snapshot.
  - **core 1 / `uiTask`** — brings up the SH1106 (`Wire` on 21/22, addr `0x3C`)
    and renders at ~6 Hz (`UI_FRAME_MS` 166): **S1 SCAN** / **S2 PAIR** (shared
    layout, size-3 word + large BT glyph & cycling searching waves), **S3 READY**
    (bold glyph, all waves solid + dot, held `S3_HOLD_MS` 1.5 s), **S4** stick-check
    HUD (status strip + connected icon; Idle gamepad glyph / `ACC ▶` / `REV ◀` /
    `TURN L` / `TURN R` with a rescaled % bar; most-recently-active axis owns
    Area 2 with a 400 ms linger back to Idle), **S5 RESET…** toast (~1 s after the
    Reset button, then falls through to S2). Status strip renders its **normal**
    state only — `powerState` / warning / **S0** are step 5, and `BATT_PCT_STUB`
    stands in for a state-of-charge the ECU never actually receives (BMS owns it).
  - The OLED draw helpers (`drawBtGlyph` / `drawWaves` / `drawStatusStrip` /
    `drawWord` / `renderSearchLike` / `renderConnected` / `fillTri` / the S4
    body) are a **verbatim port of `BluetoothUIMockup/src/main.cpp`** — the
    pixel-accurate reference for ECU-SPEC-002, tuned on the real panel. DualCore
    supplies only the live inputs: `g_linkState` picks the screen and the real
    gamepad sticks drive S4 via `s4PickAxis()` (the mockup cycles S4 on a timer
    because it has no joystick).
  - `uiTask` also owns the UART1 TX for this sprint: on every `linkState` change
    it emits `RING SEARCH_BLINK` / `RING PAIR_BLINK` / `RING CONNECTED` (+ `BUZZ
    CONNECT` on connect) via `Serial1` (RX 16 / TX 17), mirrored to USB as `tx>`;
    bytes coming back on UART1 RX print as `rx<` (jumper 17→16 to see the round trip).
  - `ISOLATION_TEST` kept, default 0; the uiTask spin now runs against real rendering.

**How to verify**

```
$env:PATH += ";$env:USERPROFILE\.platformio\penv\Scripts"; pio run -e esp32dev
pio run -e esp32dev -t upload
pio device monitor
```

1. Boot **bonded**: OLED shows **S1 SCAN** with cycling waves, monitor prints
   `tx> RING SEARCH_BLINK`, `ui started on core 1` / `bt … link=SEARCH`.
2. Press **Pair**: OLED → **S2 PAIR**, `tx> RING PAIR_BLINK`, Mode LED fast-blink.
3. Connect a controller: `tx> RING CONNECTED` + `tx> BUZZ CONNECT`, OLED shows
   **S3 READY** ~1.5 s then the **S4** HUD. Wiggle the left stick → `ACC` / `REV`
   / `TURN L` / `TURN R` with the bar tracking, returns to the Idle gamepad glyph
   ~400 ms after re-centre.
4. Power the controller off: OLED → **S1 SCAN**, `tx> RING SEARCH_BLINK`,
   `bt` heartbeat uninterrupted.
5. Press **Reset**: **S5 RESET…** toast ~1 s → **S2 PAIR**; `Boot: NVS bonded
   flag = 0` on the next power cycle.
6. Record `bt` / `ui` stack high-water (printed every ~10 beats / ~30 frames);
   both must keep headroom with the OLED redrawing.
7. `-D ISOLATION_TEST=1`: uiTask's 3 s spin must not stall `BP32.update()`, and
   btTask's spin must not freeze the OLED. Set back to 0.

**Results** _(fill in after running on hardware)_

- [ ] boot bonded → S1, `RING SEARCH_BLINK` emitted once
- [ ] Pair → S2, `RING PAIR_BLINK`; Reset → S5 → S2, bond cleared
- [ ] connect → S3 (~1.5 s) → S4; sticks drive ACC / REV / TURN with linger
- [ ] disconnect → S1, heartbeat uninterrupted
- [ ] stack high-water: bt = ____ words, ui = ____ words
- [ ] isolation test: OLED redraw and BT poll each ride through the other's spin
- notes:

## Step 4a — naive `volatile` shared struct  · branch `step-4a-volatile`

_not started_

## Step 4b — mutex-protected `RobotState` + snapshot  · branch `step-4b-mutex-snapshot`

_not started_

## Step 5 — `powerState` input + CRITICAL response  · branch `step-5-powerstate`

_not started._ Battery *sensing* is the BMS's now (ECU-SPEC-001 rev 5), so this step
no longer reads a divider. It feeds a simulated `powerState` (button or timer →
`WARNING` → `CRITICAL`) into `RobotState` and exercises the ECU's reaction under the
core split: `motor_control` zeros + holds, the BLE path tears down, `oled_ui` shows S0,
`led_pattern` emits `RING OFF` + `BUZZ S0`. The G4-relevant question is whether the
split holds timing while a CRITICAL event forces a screen change + motor stop at once.

---

## G4 verdict

_Pending. After step 5, run with everything active (gamepad connected, stick
moving, OLED redrawing S4, `RING …` / `BUZZ …` commands going out on UART1, a
simulated `powerState` CRITICAL forcing S0 + motor stop) and log the `btTask`
loop-interval min/max/mean over 60 s. "Split holds" feeds the initial commit of
`07_Codebase_Repository`; "needs amendment" goes back into ECU-ADR-004 /
ECU-SPEC-001 §9._
