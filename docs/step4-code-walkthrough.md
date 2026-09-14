# `src/main.cpp` — DualCore sprint, Step 4 (code walkthrough)

This is ESP32 firmware for the NitroWorks RC-car ECU. The whole design goal is
**split the work across the chip's two CPU cores** so Bluetooth work never
stalls the display and vice versa. It uses the Arduino-ESP32 framework on top
of FreeRTOS.

## The big picture

`setup()` does hardware init and then creates **two pinned FreeRTOS tasks**, one
nailed to each core (`main.cpp:746-747`):

| Task     | Core | Priority | Job |
|----------|------|----------|-----|
| `btTask` | 0    | 3        | Bluepad32 BT stack, gamepad input, Pair/Reset buttons, pairing state machine, Mode LED |
| `uiTask` | 1    | 1        | Draw the OLED at ~6 Hz, emit `RING`/`BUZZ` commands on UART1 |

`loop()` just sleeps forever (`main.cpp:750`) — all real work is in the two tasks.

The two cores talk through **one shared struct**, `g_state`, and that's the
subject of Step 4 (below).

---

## 1. Configuration (lines 54–99)

- **Pins** — Mode LED on GPIO2, Pair button GPIO15, Reset button GPIO13, I²C
  OLED on 21/22, UART1 to a future companion chip on 16/17.
- **Tunables** — timing constants: BT poll every 5 ms, UI frame every 166 ms,
  S3 "READY" screen holds 1.5 s, reset toast 1 s, button debounce 40 ms, etc.
- **NVS keys** — `nitro-ecu`/`bonded`: a flag in flash remembering whether this
  car has been paired to a controller.
- **`LinkState` enum** — the three states the whole UI keys off: `SEARCH`,
  `PAIR`, `CONNECTED`.

---

## 2. Cross-core shared state (lines 111–147) — the Step 4 core

```cpp
struct RobotState {
  LinkState linkState; bool hasBond; bool connected;
  int32_t throttle; int32_t steer;
  uint32_t connectedAtMs; uint32_t resetToastAtMs; uint32_t pairBlankAtMs;
};
```

Three objects:

- **`g_state`** — the shared instance. Touched **only** while holding `g_stateMux`.
- **`g_stateMux`** — a `portMUX_TYPE` spinlock (ESP32's primitive for short
  critical sections shared between cores).
- **`g_pending`** — a **core-0-only scratch copy**. `btTask` scribbles into this
  freely with no lock, because nothing else touches it.

Two accessor functions:

- **`publishState()`** — core 0 calls this once per loop: locks, does
  `g_state = g_pending` (one struct copy), unlocks.
- **`readState()`** — core 1 calls this once per frame: locks, copies `g_state`
  into a local `snap`, unlocks, returns it.

**Why:** In step 3 these were 8 loose `volatile` globals. Each one was
individually safe (single writer core, single reader core, 32-bit aligned =
atomic on ESP32), but `uiTask` read them one at a time — so it could grab
`linkState` from btTask iteration N and `connectedAtMs` from iteration N+1 and
render a frame that never actually existed. The snapshot fixes that: every
field `uiTask` uses in one frame is guaranteed to come from the same `btTask`
iteration.

---

## 3. Buttons (lines 157–189)

`Button` struct + `buttonPressed()` is a standard debounce: reads the pin, and
only reports a press once the reading has been stable (unchanged) for
`BTN_DEBOUNCE_MS`, on the HIGH→LOW edge (buttons are active-low with internal
pull-ups).

---

## 4. The pairing state machine (core 0, lines 191–344)

This is cloned from an earlier `BluetoothPairing` sprint. The rules:

- **`openPairingWindow()`** — sets `pairingMode = true`, flips
  `g_pending.linkState = LINK_PAIR` immediately, disconnects any active
  controller, and schedules (defers by 200 ms) the expensive
  `forgetBluetoothKeys()` + `enableNewBluetoothConnections(true)`. The deferral
  matters: those calls trigger a flash-sector erase that freezes core 1's
  rendering for a few hundred ms, so it waits until the "you pressed Pair"
  feedback frame has reached the screen.
- **`resetBondedController()`** — wipes the NVS `bonded` flag first (durable),
  stamps `resetToastAtMs` (drives the S5 toast on the OLED), then opens a
  pairing window.
- **`onConnectedController()`** — when a gamepad connects: rejects it if there's
  no bond and no open pairing window (a guard against BT-Classic letting
  unexpected controllers in); otherwise, if pairing, it bonds
  (`persistBonded(true)`, writes NVS) and closes the window. Lights the Mode
  LED solid.
- **`onDisconnectedController()`** — clears the slot, drops the Mode LED.
- **`computeLinkState()`** — the authoritative reducer: connected → `CONNECTED`;
  else pairing-or-unbonded → `PAIR`; else → `SEARCH`.
- **`publishSticks()`** — reduces the whole gamepad to two numbers: `throttle` =
  −left-stick-Y (up = forward), `steer` = left-stick-X.

### `btTask()` main loop (lines 368–417)

Every 5 ms:

1. `pollButtons()`
2. `BP32.update()` (service the BT stack) → `publishSticks()`
3. Run the deferred pairing setup if its timer elapsed
4. Recompute `connected` / `connectedAtMs` / `linkState` into `g_pending`
5. **`publishState()`** — flush the whole struct to `g_state` under the lock
6. Heartbeat log once/second, stack-watermark log every 10 beats

---

## 5. OLED rendering (core 1, lines 420–618)

The draw helpers (`drawBtGlyph`, `drawWaves`, `drawStatusStrip`,
`renderSearchLike`, `renderConnected`, `fillTri`, `drawS4`, `drawResetToast`)
are a **verbatim pixel port** from a separate `BluetoothUIMockup` project — the
comment at `main.cpp:426` says don't re-derive them, change the mockup first.
They render the ECU-SPEC-002 screen set:

- **S1 SCAN / S2 PAIR** — `renderSearchLike()`: battery strip + big word +
  Bluetooth glyph with animated "searching" arcs.
- **S3 READY** — `renderConnected()`: bold glyph, solid arcs, shown for 1.5 s
  after connect.
- **S4 stick-check HUD** — `drawS4()`: takes `thr`/`str` from the snapshot.
  `s4PickAxis()` decides which single axis "owns" the screen
  (ACC / REV / TURN L / TURN R / IDLE) with a 400 ms linger back to idle, and
  draws a proportional bar.
- **S5 RESET…** — `drawResetToast()`: growing-dots toast for 1 s after a Reset
  press.

`s4PickAxis()` and `drawS4()` take throttle/steer as **parameters** now
(Step 4 change) instead of reading globals, so they operate on the frame's
snapshot.

### `uiTask()` main loop (lines 656–715)

Every 166 ms:

1. **`RobotState snap = readState()`** — one locked copy
2. Pull `link`, `connected`, `connAt`, `resetAt`, `pairAt` out of `snap`
3. If `link` changed since last frame → emit `RING SEARCH_BLINK` /
   `RING PAIR_BLINK` / `RING CONNECTED` (+ `BUZZ CONNECT`) on UART1. Nothing's
   wired to receive these yet — they're mirrored to USB serial as `tx>` for a
   later bring-up.
4. Pick and draw the screen based on `link` + the timers
5. Stack-watermark log every 30 frames

---

## 6. `ISOLATION_TEST` (compile flag, default 0)

When set to 1, each task busy-spins for 3 s periodically so you can watch on the
serial monitor that the *other* core keeps running through it — the proof that
the core split actually isolates the two workloads. This feeds the sprint's real
deliverable, the "G4 verdict" on whether the split holds timing.
