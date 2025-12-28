// MarioController_ESP32.ino
// -------------------------------------------
// Board: Original ESP32 DevKit
// Role : Bluetooth gamepad → UART bridge
//        SN30 Pro (Bluepad32) → Serial2 → ESP32-S3
//
// Packet format (MUST match S3 side):
//   struct InputPacket { int8_t moveX; uint8_t buttons; uint8_t adminArmRequest; uint8_t flags; }
//   Sent as bytes: [0xAA][moveX][buttons][adminArmRequest][flags]
//
// Flags:
//   bit0 (0x01) = controller connected
//   bit1 (0x02) = has human input this frame (any axis/button non-neutral)
//
// Button bits:
//   bit0 = jump  (A)
//   bit1 = spin  (B)
//   bit2 = run   (X or Y)
//   bit3 = down  (D-pad down)
//   bit4 = reset (Select)
//   bit5 = pause (Start)
//   bit6 = up    (D-pad up)
// -------------------------------------------

#include <Arduino.h>
#include <Bluepad32.h>

// Uncomment to enable debug prints on this board
//#define DEBUG_TX 1

// UART pins (match your wiring):
//  ESP32 TX2 (GPIO17) → ESP32-S3 RX1 (GPIO17)
//  ESP32 RX2 (GPIO16) ← ESP32-S3 TX1 (GPIO18)
const int LINK_TX_PIN = 17;   // ESP32 → S3
const int LINK_RX_PIN = 16;   // ESP32 ← S3 (unused for now)

static const uint8_t FLAG_CONNECTED       = 0x01;
static const uint8_t FLAG_HAS_HUMAN_INPUT = 0x02;

// Packet layout (4 bytes total)
struct InputPacket {
  int8_t  moveX;          // -127..+127 (left/right)
  uint8_t buttons;        // bitfield (see above)
  uint8_t adminArmRequest; // 1 = SELECT+UP held 5s, triggers OTA arm on S3
  uint8_t flags;          // connection + human input flags
};

ControllerPtr gController = nullptr;

// ─────────────────────────────────────
// Bluepad32 callbacks
// ─────────────────────────────────────
void onConnectedController(ControllerPtr ctl) {
  if (!gController) {
    gController = ctl;
#ifdef DEBUG_TX
    Serial.println("[ESP32] Controller connected");
#endif
  } else {
#ifdef DEBUG_TX
    Serial.println("[ESP32] Extra controller ignored");
#endif
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (ctl == gController) {
#ifdef DEBUG_TX
    Serial.println("[ESP32] Controller disconnected");
#endif
    gController = nullptr;
  }
}

// ─────────────────────────────────────
// Setup
// ─────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== MarioController_ESP32 (Gamepad Bridge) ===");

  // UART link to ESP32-S3
  Serial2.begin(115200, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  Serial.println("[ESP32] Serial2 started on RX2=16, TX2=17 @115200");

  // Bluepad32 init
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableNewBluetoothConnections(true);
  Serial.println("[ESP32] Bluepad32 ready. Put SN30 Pro in pairing mode.");
}

// ─────────────────────────────────────
// Send one input packet
// ─────────────────────────────────────
void sendInputPacket() {
  static int8_t  lastMoveX   = 0;
  static uint8_t lastButtons = 0;

  // ─────────────────────────────────────────────────────────────────
  // SELECT+UP Hold Detection (5 seconds → arm OTA on S3)
  // ─────────────────────────────────────────────────────────────────
  static uint32_t selectUpStartMs = 0;
  static bool armRequestSent = false;
  static bool selectUpWasPressed = false;
  
  InputPacket pkt;
  pkt.moveX = 0;
  pkt.buttons = 0;
  pkt.adminArmRequest = 0;  // default: no arm request
  pkt.flags = 0;

  if (gController && gController->isConnected()) {
    pkt.flags |= FLAG_CONNECTED;
    uint8_t dpad = gController->dpad();

    bool dpadUp    = dpad & 0x01;
    bool dpadDown  = dpad & 0x02;
    bool dpadRight = dpad & 0x04;
    bool dpadLeft  = dpad & 0x08;

    bool jumpBtn   = gController->a();              // normal jump
    bool spinBtn   = gController->b();              // spin jump
    bool runBtn    = gController->x() || gController->y();
    bool resetBtn  = gController->miscBack();       // Select = reset world
    bool startBtn  = gController->miscStart();      // Start = pause

    // ═══════════════════════════════════════════════════════════════
    // SELECT+UP HOLD COMBO FOR OTA ARMING
    // ═══════════════════════════════════════════════════════════════
    bool selectAndUpPressed = (resetBtn && dpadUp);
    
    if (selectAndUpPressed) {
      if (selectUpStartMs == 0) {
        selectUpStartMs = millis();  // Start timing
#ifdef DEBUG_TX
        Serial.println("[ADMIN] SELECT+UP pressed, starting timer...");
#endif
      }
      
      // After 5 seconds, send arm request ONCE
      if (!armRequestSent && (millis() - selectUpStartMs >= 5000)) {
        pkt.adminArmRequest = 1;
        armRequestSent = true;
        Serial.println("[ADMIN] SELECT+UP held 5s → sending arm request");
      }
      
      selectUpWasPressed = true;
    } else {
      // Reset when buttons released (debounce)
      if (selectUpWasPressed) {
        selectUpStartMs = 0;
        armRequestSent = false;
        selectUpWasPressed = false;
#ifdef DEBUG_TX
        Serial.println("[ADMIN] SELECT+UP released, resetting");
#endif
      }
    }
    // ═══════════════════════════════════════════════════════════════
    // Movement input (D-pad first, then analog stick)
    float moveInput = 0.0f;
    if (dpadLeft)  moveInput = -1.0f;
    if (dpadRight) moveInput = +1.0f;

    if (moveInput == 0.0f) {
      int lx = gController->axisX();  // ~-512..+512
      const int deadzone = 64;
      if (abs(lx) < deadzone) lx = 0;
      if (lx != 0) {
        moveInput = (float)lx / 512.0f;  // -1..+1-ish
      }
    }

    // Clamp and encode to -127..+127
    if (moveInput >  1.0f) moveInput =  1.0f;
    if (moveInput < -1.0f) moveInput = -1.0f;
    pkt.moveX = (int8_t)roundf(moveInput * 127.0f);

    // Build button bitfield
    uint8_t buttons = 0;
    if (jumpBtn)   buttons |= 0x01;  // bit0
    if (spinBtn)   buttons |= 0x02;  // bit1
    if (runBtn)    buttons |= 0x04;  // bit2
    if (dpadDown)  buttons |= 0x08;  // bit3
    if (resetBtn)  buttons |= 0x10;  // bit4 - Select = reset world
    if (startBtn)  buttons |= 0x20;  // bit5 - Start = pause
    if (dpadUp)    buttons |= 0x40;  // bit6 - D-pad up (pause menu navigation)

    pkt.buttons = buttons;

    // Human input heuristic (post-deadzone): any movement or any button bit set
    if (pkt.moveX != 0 || pkt.buttons != 0 || pkt.adminArmRequest != 0) {
      pkt.flags |= FLAG_HAS_HUMAN_INPUT;
    }
  }

  // Serialize: [0xAA][moveX][buttons][adminArmRequest][flags]
  uint8_t buf[5];
  buf[0] = 0xAA;
  memcpy(&buf[1], &pkt, 4);
  Serial2.write(buf, sizeof(buf));

#ifdef DEBUG_TX
  // Only print when something actually changed (or arm request sent)
  if (pkt.moveX != lastMoveX || pkt.buttons != lastButtons || pkt.adminArmRequest != 0 || pkt.flags != 0) {
    Serial.print("[ESP32] TX moveX=");
    Serial.print(pkt.moveX);
    Serial.print(" buttons=0x");
    Serial.print(pkt.buttons, HEX);
    Serial.print(" armReq=");
    Serial.println(pkt.adminArmRequest);
    lastMoveX   = pkt.moveX;
    lastButtons = pkt.buttons;
  }
#endif
}

void loop() {
  BP32.update();
  sendInputPacket();
  delay(20);   // ~50 packets per second
}
