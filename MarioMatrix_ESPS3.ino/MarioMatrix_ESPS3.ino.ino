// ====================================================================
// MarioMatrix_ESPS3.ino
// ====================================================================
// ESP32-S3 DevKit (8MB PSRAM) - Mario-style game on HUB75 LED matrix
// Display: 3x 64x64 P3 HUB75E panels = 192x64 total resolution
// Controller: UART link to separate ESP32 running MarioController_ESP32
//
// HUB75 Wiring: See pin definitions below (GPIO2-GPIO16)
// UART Link: RX1=GPIO17, TX1=GPIO18 @ 115200 baud
// Packet: [0xAA][moveX][buttons][adminArmRequest]
//
// Features:
// - SMW-style slope sliding (press DOWN on slopes)
// - Pipe transitions with SMW iris fade
// - Web admin portal with OTA firmware updates
// - WiFiManager captive portal for easy setup
// - Real-time weather display on HUD
// - Pause menu with restart/resume options
//
// NOTE: Controller firmware changes required for adminArmRequest field.
//       See "Controller Integration Notes" section below for details.
// ====================================================================

// ---- Core Arduino/ESP32 Libraries ----
#include <Arduino.h>
#include <FS.h>
#include <esp_partition.h>
#include <SPIFFS.h>

#define DEMO_HAS_SPIFFS 1
#define DEMO_FS SPIFFS
#include <Preferences.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_err.h>
#include <time.h>

// ---- Networking Libraries (alphabetical) ----
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>

#include "WeatherCompat.h"

// ---- Web Server & OTA Libraries ----
#include <ElegantOTA.h>
#include <Update.h>
#include <WebServer.h>

// ---- Hardware Library ----
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

// ---- Project Headers (game assets & maps) ----
#include "ItemSprites.h"
#include "TileIDs.h"
#include "Map_Build.h"
#include "Map_Overworld_1.h"
#include "Map_Overworld_2.h"
#include "Map_Overworld_3.h"
#include "MarioHudSprites.h"
#include "MarioPauseMenu.h"
#include "MarioSprites.h"
#include "MobSprites.h"

// Types that appear in function signatures must be visible before Arduino auto-generates prototypes.
#include "DemoTypes.h"

// Forward-declare types that may appear in Arduino-generated prototypes.
// (Arduino inserts prototypes after the last #include in the sketch.)
struct PiranhaPlant;

#include "WorldSprites.h"

// Visual-only tiles are defined in TileIDs.h (single source of truth).

// Ground_Tile_Flag is 14x15.
// IMPORTANT: Do NOT animate the pole pixels.
// The pole is on the left side (columns ~1-2). The flag cloth begins to the right.
static const int FLAG_SRC_W = 14;
static const int FLAG_SRC_H = 15;

// EXCLUDE POLE columns (left side) AND EXCLUDE gold cap rows (top).
// Pole lives around x=0..2. Gold cap is in the top ~5 rows.
// We only animate the cloth region.
static const int FLAG_ONLY_SRC_X = 3;   // removes pole pixels entirely
static const int FLAG_ONLY_SRC_Y = 5;   // removes gold cap rows entirely

static const int FLAG_ONLY_W     = 11;  // 14 - 3
static const int FLAG_ONLY_H     = 10;  // 15 - 5

// ─────────────────────────────────────────────
// Forward declarations needed early
// (Some helpers are called before their definitions)
// ─────────────────────────────────────────────
static void drawWorldTileCustomSize(const uint8_t* sprite, int width, int height, int px, int py);
static void drawWorldTileCustomSizeSubrect(const uint8_t* sprite,
                                           int width,
                                           int height,
                                           int px,
                                           int py,
                                           int srcX,
                                           int srcY,
                                           int subW,
                                           int subH);

static inline void getFlagTileOffset(uint8_t tileId, int &ox, int &oy);

// ---- Debug Flags ----
#define DEBUG_CTRL  0  // Controller input debug (throttled to 1000ms)
#define DEBUG_WEB   0  // Web server heartbeat (throttled to 5000ms)
#define DEBUG_ADMIN 1  // Admin/OTA arming messages
// Weather parsing/debug output (safe to leave enabled because weather fetch is infrequent)
#define DEBUG_WEATHER 1

// ---- OTA Configuration ----
#define ENABLE_ARDUINO_OTA 0  // Arduino IDE OTA disabled (use Web OTA only)

// ---- Hardware Configuration ----
// HUB75 Panel Layout
#define PANEL_WIDTH 64
#define PANEL_HEIGHT 64
#define PANELS_NUMBER 3
#define MATRIX_WIDTH (PANEL_WIDTH * PANELS_NUMBER)  // 192
#define MATRIX_HEIGHT PANEL_HEIGHT                   // 64

// UART Link Pins (to controller ESP32)
constexpr uint8_t LINK_RX_PIN = 17;  // ESP32-S3 RX1 <- Controller TX
constexpr uint8_t LINK_TX_PIN = 18;  // ESP32-S3 TX1 -> Controller RX

// ---- Global Matrix Pointer ----
MatrixPanel_I2S_DMA* matrix = nullptr;

// ---- Color Palette ----
// Palette indices: 0=transparent, 1=skin, 2=red, 3=brown, 4=blue, 5=yellow, 6=white
struct RGB { uint8_t r, g, b; };
const RGB heroPalette[] PROGMEM = {
  {  0,   0,   0},   // 0 - transparent
  {255, 210, 170},   // 1 - skin
  {228,  32,  32},   // 2 - red
  {120,  60,  10},   // 3 - brown
  {  0, 110, 220},   // 4 - blue
  {255, 220,  70},   // 5 - yellow
  {255, 255, 255},   // 6 - white
  {128, 128, 128},   // 7 - gray
  {  0, 255,   0},   // 8 - debug green
  {255,   0, 255},   // 9 - magenta (placeholder)
  {122, 200, 255},   // 10 - sky light
  { 79, 160, 255},   // 11 - sky mid / haze
  { 60, 128, 224},   // 12 - distant hills
  {240, 208, 144},   // 13 - dirt light
  {208, 160,  96},   // 14 - dirt mid
  {160, 112,  64},   // 15 - dirt dark / spots
  { 64,  64, 112},   // 16 - underground stone dark
  {112, 112, 160},   // 17 - underground stone mid
  { 64, 200,  96},   // 18 - pipe green light
  { 32, 128,  64},   // 19 - pipe green dark
  { 16,  16,  16},   // 20 - near-black (Bullet Bill body)
  {192, 192, 192},   // 21 - metal gray (Bullet rims / shine)
  {232,  88, 112},   // 22 - mouth pink/red (Bullet / enemies)
  { 32, 176,  96}    // 23 - grass highlight
};

// ---- Input Structures ----
// UART packet received from controller ESP32
static const uint8_t CTRL_FLAG_CONNECTED       = 0x01;
static const uint8_t CTRL_FLAG_HAS_HUMAN_INPUT = 0x02;

struct InputPacket { 
  int8_t moveX;                // -1 (left), 0 (neutral), +1 (right)
  uint8_t buttons;             // Bitfield: bit0=jump, bit1=spin, bit2=run
  uint8_t adminArmRequest;     // 1 = SELECT+UP held 5s (arms OTA)
  uint8_t flags;               // bit0=connected, bit1=human input this frame
};

// Decoded input state used by game logic
struct InputState {
  int8_t rawMoveX;
  float moveAxis;
  uint8_t btnJump;
  uint8_t btnSpin;
  uint8_t btnRun;
  uint8_t btnDown;
  uint8_t btnUp;
  uint8_t btnReset;
  uint8_t btnStart;
};
InputState gInput = {0, 0.0f, 0,0,0,0,0,0,0};

// ---- Demo / Auto-play control ----
enum ControlMode : uint8_t { AUTO_PLAY = 0, PLAYER = 1 };
static ControlMode gControlMode = AUTO_PLAY;

static bool     gControllerConnected = false;
static bool     gHumanInputThisFrame = false;
static uint32_t gLastHumanInputMs = 0;
static uint32_t gLastControllerPacketMs = 0;

// Attract loop state
static uint8_t  gAttractLoadedMapId = 0xFF; // which map's recording is currently loaded

enum AttractIntroState : uint8_t {
  ATTRACT_INTRO_NONE = 0,
  ATTRACT_INTRO_FADE_OUT = 1,
  ATTRACT_INTRO_HOLD_BLACK = 2,
  ATTRACT_INTRO_FADE_IN = 3,
};

static AttractIntroState gAttractIntroState = ATTRACT_INTRO_NONE;
static uint32_t gAttractIntroStartMs = 0;
// Default HOLD_BLACK duration for the attract intro (ms). Can be overridden for loop restarts.
static uint32_t gAttractIntroHoldBlackMs = 200;
static const uint32_t ATTRACT_LOOP_RESTART_HOLD_BLACK_MS = 5000;

static inline bool attractIntroActive() {
  return gAttractIntroState != ATTRACT_INTRO_NONE;
}


static const uint32_t AUTO_PLAY_IDLE_TIMEOUT_MS = 15000; // 15s idle -> return to attract (per spec)
static const uint32_t CONTROLLER_PACKET_TIMEOUT_MS = 250; // safety: treat as disconnected

// Attract start delay: require sustained no-human-input before starting demo.
static const uint32_t ATTRACT_START_DELAY_MS = 15000;
static uint32_t gAttractPendingSinceMs = 0;
static bool gAttractSessionActive = false;

// ---- Map1 Input Recorder (manual) ----
// Records the effective gameplay input (what Mario consumes) once per gameplay tick.
// Toggle recording via a safe hold chord.
static const uint32_t DEMO_REC_HOLD_MS = 1000;
static const uint32_t DEMO_REC_COOLDOWN_MS = 500;
static const uint16_t DEMO_REC_MIN_FRAMES = 30;

// Controller combo for recorder (safe toggle; avoids START/pause):
// SELECT + LEFT held for DEMO_REC_HOLD_MS.
// Note: LEFT is detected via strong negative rawMoveX (dpad/axis), then suppressed.
static const bool DEMO_REC_USE_START = false;

// ---- Playback test (Map1) ----
static const uint32_t DEMO_PLAY_HOLD_MS = 1000;
static const uint32_t DEMO_PLAY_COOLDOWN_MS = 500;

// Optional determinism debug prints (disabled by default).
#ifndef DEBUG_DEMO_DETERMINISM
#define DEBUG_DEMO_DETERMINISM 0
#endif

// Erase all stored official demo recordings (NVS) via a safe hold chord.
// Chord: SELECT + UP + RIGHT held.
static const uint32_t DEMO_ERASE_HOLD_MS = 1500;
static const uint32_t DEMO_ERASE_COOLDOWN_MS = 1000;

static uint32_t gDemoEraseChordStartMs = 0;
static bool     gDemoEraseChordTriggeredThisHold = false;
static uint32_t gDemoEraseChordCooldownUntilMs = 0;

static uint32_t gDemoPlayChordStartMs = 0;
static bool     gDemoPlayChordTriggeredThisHold = false;
static uint32_t gDemoPlayChordCooldownUntilMs = 0;

// Playback cancel guard: starting playback uses a human-input chord; ignore cancel briefly.
static uint32_t gDemoPlaybackIgnoreCancelUntilMs = 0;

static bool     gDemoPlaybackActive = false;
static bool     gDemoPlaybackArmed = false;   // loaded & waiting for stable control
static bool     gDemoPlaybackPlaying = false; // actively consuming frames
static uint8_t  gDemoPlaybackArmedMapId = 0xFF;
static uint8_t  gDemoPlaybackStableFrames = 0;
static uint16_t gDemoPlaybackIdx = 0;

// Map2 playback trace (debug aid): prints first few frames of input + state.
static uint16_t gDemoTraceRemaining = 0;
static uint8_t  gDemoTraceMapId = 0xFF;
static bool     gDemoTraceHaveLastFrame = false;
static uint16_t gDemoTraceLastFrameIdx = 0;
static int8_t   gDemoTraceLastMoveX = 0;
static uint8_t  gDemoTraceLastButtons = 0;

#ifndef DEBUG_DEMO_MAP2_TRACE
#define DEBUG_DEMO_MAP2_TRACE 0
#endif

#if DEBUG_DEMO_MAP2_TRACE
struct DemoTraceSample {
  uint32_t t;
  uint16_t idx;
  int8_t   mx;
  uint8_t  btn;
  float    hx;
  float    hy;
  float    hvx;
  float    hvy;
  uint8_t  onG;
  uint8_t  size;
  uint16_t coins;
  int8_t   kIdx;
  uint8_t  kAct;
  float    kx;
  float    ky;
  float    kvx;
  float    kvy;
};

static const uint8_t DEMO_TRACE_RING_CAP = 96;
static DemoTraceSample gDemoTraceRing[DEMO_TRACE_RING_CAP];
static uint8_t gDemoTraceRingHead = 0;
static bool gDemoTraceRingFilled = false;
static bool gDemoTraceKoopaDumped = false;

// NOTE: Implementations are placed later in the file after the relevant globals
// (heroX/heroY/gKoopas/etc.) are declared.
static void demoTraceReset();
static void demoTracePush(uint32_t nowMs);
static void demoTraceDump(const char *tag);
#endif

// When using official recordings in attract mode, delay consumption until player control
// is stable post-transition to avoid start timing drift.
static const uint8_t DEMO_PLAYBACK_STABLE_CONTROL_FRAMES = 1;

static const uint16_t DEMO_REC_MAX_FRAMES = 6000; // ~100s @60fps, ~12KB
static DemoRecFrame gDemoRecBuf[DEMO_REC_MAX_FRAMES];
static uint16_t gDemoRecWriteIdx = 0;
static uint16_t gDemoRecLength = 0;
static bool gDemoIsRecording = false;
static bool gDemoHasRecording = false;

static uint32_t gDemoChordStartMs = 0;
static bool     gDemoChordTriggeredThisHold = false;
static uint32_t gDemoChordCooldownUntilMs = 0;
static bool     gDemoSkipRecordThisFrame = false;

// Demo/record/playback deterministic timebase.
// Some gameplay systems were using millis() (wall-clock), which breaks determinism between
// record vs playback. During demo contexts we advance a simulation clock in fixed steps.
static uint64_t gDemoSimNowUs = 0;

static inline bool demoTimingActive() {
  return gDemoIsRecording || gDemoPlaybackActive || (gControlMode == AUTO_PLAY);
}

static inline void demoSimReset() {
  gDemoSimNowUs = 0;
}

static inline void demoSimAdvanceSeconds(float dtSeconds) {
  // dtSeconds is forced to 1/60 during demo contexts.
  // Use microseconds to avoid cumulative rounding drift.
  if (dtSeconds <= 0.0f) return;
  const double us = (double)dtSeconds * 1000000.0;
  uint64_t addUs = (uint64_t)(us + 0.5);
  if (addUs == 0) addUs = 1;
  gDemoSimNowUs += addUs;
}

static inline uint32_t gameplayNowMs() {
  if (demoTimingActive()) {
    return (uint32_t)(gDemoSimNowUs / 1000ULL);
  }
  return millis();
}

static inline uint32_t demoMix32(uint32_t x) {
  // Small integer hash / mix (deterministic across platforms)
  x ^= x >> 16;
  x *= 0x7feb352dU;
  x ^= x >> 15;
  x *= 0x846ca68bU;
  x ^= x >> 16;
  return x;
}

// gCurrentMapId is defined later (after the BuiltinMapId enum), but some helpers need it earlier.
extern uint8_t gCurrentMapId;

static inline uint8_t coinBrickInitialRemaining(int tx, int ty) {
  // 1..5 coins, deterministic for demo contexts so recordings can replay across reboots.
  uint32_t h = 0xC01DCAFEu;
  h ^= (uint32_t)gCurrentMapId * 0x9e3779b9U;
  h ^= (uint32_t)(tx & 0xFFFF) | ((uint32_t)(ty & 0xFFFF) << 16);
  h = demoMix32(h);
  return (uint8_t)(1 + (h % 5));
}

// ==================================================================== 
// Controller Integration Notes
// ====================================================================
// The controller ESP32 (MarioController_ESP32.ino) must implement:
//
// 1. Update InputPacket struct to 4 bytes:
//    struct InputPacket { int8_t moveX; uint8_t buttons; uint8_t adminArmRequest; uint8_t flags; };
//
// 2. Detect SELECT+UP held for 5 seconds:
//    static uint32_t selectUpStartMs = 0;
//    bool selectAndUpPressed = (btnSelect == LOW && btnUp == LOW);
//    if (selectAndUpPressed) {
//      if (selectUpStartMs == 0) selectUpStartMs = millis();
//      if (millis() - selectUpStartMs >= 5000) pkt.adminArmRequest = 1;
//    } else {
//      selectUpStartMs = 0;
//      pkt.adminArmRequest = 0;
//    }
//
// 3. Send 4-byte packet: Serial1.write(0xAA); Serial1.write((uint8_t*)&pkt, 4);
// ====================================================================

// ---- WiFi & Networking Configuration ----
const char* NTP_SERVER = "pool.ntp.org";

// ---- Web Server Globals ----
WebServer server(80);

static const char* DEVICE_NAME = "mario-matrix";
static const char* AP_NAME = "MarioMatrix-Setup";  // WiFiManager AP SSID
static const char* AP_PASS = nullptr;              // nullptr = open AP
static bool gWebServerStarted = false;

// ---- WiFi State Machine ----
enum WiFiState {
  WIFI_INIT,          // Initial state, attempting STA connection
  WIFI_TRY_STA,       // Trying to connect to saved WiFi (15s timeout)
  WIFI_PORTAL,        // Captive portal running (game continues)
  WIFI_CONNECTED,     // Successfully connected to WiFi
  WIFI_OFFLINE        // No WiFi, game runs offline
};

static WiFiState gWiFiState = WIFI_INIT;
static uint32_t gWiFiStateStartMs = 0;
static WiFiManager* gWiFiManager = nullptr;
static WiFiManagerParameter* gWiFiPortalHeaderParam = nullptr;
static bool gWiFiConnected = false;
static bool gShowIPOnMatrix = false;
static uint32_t gIPDisplayUntil = 0;

// Captive-portal success page window (keep portal alive briefly without blocking the game loop)
static bool     gPortalSuccessActive = false;
static uint32_t gPortalSuccessUntilMs = 0;

// Alias for legacy code compatibility
#define gWifiConnected gWiFiConnected

String fwVersion() {
  // simple version marker; update manually when you release
  return String("MarioMatrix S3 v") + __DATE__ + " " + __TIME__;
}

// ─────────────────────────────────────────────
// Secure OTA Admin System
// ─────────────────────────────────────────────
static const char* OTA_SECRET = "mwzd1QCNT7KOOlnNRSTArbXO5FE&uFE2";  // e.g. 32+ chars hex
static const char* ADMIN_USER = "admin";
static const char* ADMIN_PASS = "m!aV00gI5!N%iMul";

// Public Portal Authentication
static const char* PUBLIC_USER = "Mario";
static const char* PUBLIC_PASS = "Matrix";

static const uint32_t ARM_MS = 10UL * 60UL * 1000UL;  // 10 minutes
static uint32_t gArmUntil = 0;

inline bool isOTAArmed() {
  return gArmUntil != 0 && (int32_t)(gArmUntil - millis()) > 0;
}
inline uint32_t otaSecondsRemaining() {
  if (!isOTAArmed()) return 0;
  return (uint32_t)((gArmUntil - millis()) / 1000UL);
}

void armOTA() {
  gArmUntil = millis() + ARM_MS;
  Serial.println("[ADMIN] OTA ARMED (10 min)");
}
void disarmOTA() {
  gArmUntil = 0;
  Serial.println("[ADMIN] OTA DISARMED");
}

String adminRoot()   { return String("/") + OTA_SECRET; }
String adminHome()   { return adminRoot() + "/"; }
String adminUpdate() { return adminRoot() + "/update"; }
String adminOff()    { return adminRoot() + "/off"; }

bool requireAuth() {
  if (server.authenticate(ADMIN_USER, ADMIN_PASS)) return true;
  server.requestAuthentication();
  return false;
}

bool requirePublicAuth() {
  if (server.authenticate(PUBLIC_USER, PUBLIC_PASS)) return true;
  server.requestAuthentication();
  return false;
}

// Controller hook: called when SELECT+UP held 5s on controller
void onControllerArmRequest() {
  armOTA();
}

// ─────────────────────────────────────────────
// Pretty HTML Helpers (Dark Theme)
// ─────────────────────────────────────────────
String htmlHeader(const char* title) {
  return htmlHeader(title, "");
}

String htmlHeader(const char* title, const char* activeTab) {
  String h;
  h += "<!doctype html><html><head>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<meta charset='utf-8'>";
  h += "<title>"; h += title; h += "</title>";
  h += "<style>";
  h += "body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu; background:#0b0f14;color:#e6edf3;}";
  h += ".wrap{max-width:860px;margin:0 auto;padding:20px;}";
  h += ".card{background:#101826;border:1px solid #1f2a3a;border-radius:16px;padding:16px;margin:12px 0;box-shadow:0 6px 24px rgba(0,0,0,.35);}";
  h += ".row{display:flex;gap:12px;flex-wrap:wrap;}";
  h += ".pill{display:inline-block;padding:6px 10px;border-radius:999px;background:#162235;border:1px solid #24344d;font-size:12px;opacity:.95;}";
  h += "h1{margin:6px 0 12px;font-size:32px;letter-spacing:.3px;}";
  h += "h2{margin:0 0 10px;font-size:18px;opacity:.95;}";
  h += "a.btn,button.btn{display:inline-block;text-decoration:none;color:#e6edf3;background:#1b2a44;border:1px solid #2b3e62;padding:10px 14px;border-radius:12px;font-weight:600;cursor:pointer;}";
  h += "a.btn:hover,button.btn:hover{background:#223457;}";
  h += "a.btn.danger{background:#3a1720;border-color:#6a2a3a;}";
  h += "a.btn.danger:hover{background:#4a1c29;}";
  h += "a.btn.success{background:#1a3a27;border-color:#2a5a3a;}";
  h += "a.btn.success:hover{background:#234a32;}";
  h += ".muted{opacity:.75;font-size:13px;}";
  h += ".kv{display:grid;grid-template-columns:140px 1fr;gap:6px 10px;font-size:14px;}";
  h += ".kv div{padding:4px 0;border-bottom:1px dashed rgba(255,255,255,.06);}";
  h += ".tabs{display:flex;gap:8px;margin:12px 0 16px;border-bottom:2px solid #1f2a3a;}";
  h += ".tabs a{text-decoration:none;color:#9cc2ff;padding:10px 16px;border-radius:8px 8px 0 0;font-weight:600;opacity:.75;}";
  h += ".tabs a:hover{opacity:1;background:rgba(255,255,255,.05);}";
  h += ".tabs a.active{opacity:1;background:#1b2a44;border-bottom:2px solid #9cc2ff;margin-bottom:-2px;}";
  h += "@media (max-width:520px){.kv{grid-template-columns:110px 1fr;}h1{font-size:26px;}}";
  h += "</style></head><body><div class='wrap'>";
  
  // Add tab navigation if activeTab is specified
  if (activeTab && strlen(activeTab) > 0) {
    h += "<h1>🍄 MarioMatrix</h1>";
    h += "<div class='tabs'>";
    h += "<a href='/' class='";
    if (strcmp(activeTab, "status") == 0) h += "active";
    h += "'>Status</a>";
    h += "<a href='/info' class='";
    if (strcmp(activeTab, "info") == 0) h += "active";
    h += "'>Info</a>";
    h += "</div>";
  }
  
  return h;
}
String htmlFooter() { return "</div></body></html>"; }

// Timezone: America/Denver (simplified for now, no DST)
const long GMT_OFFSET_SEC = -7 * 3600;   // UTC-7
const int  DST_OFFSET_SEC = 0;           // we'll refine later if needed

// ─────────────────────────────────────────────
// HUD clock state (updated via NTP)
// ─────────────────────────────────────────────
struct HudClock {
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int second;
  bool valid;
};

HudClock gHudClock = {0,0,0,0,0,0,false};

// Wi-Fi / time status (NOTE: gWiFiConnected defined in WiFi state machine section above)
// bool gWifiConnected = false;  // REMOVED: duplicate, using gWiFiConnected instead
bool gTimeSynced    = false;

// Resync intervals
unsigned long gLastTimeSyncMs   = 0;
const unsigned long TIME_RESYNC_INTERVAL_MS = 6UL * 60UL * 60UL * 1000UL;  // 6 hours

unsigned long gLastClockPollMs  = 0;
const unsigned long CLOCK_POLL_INTERVAL_MS  = 1000UL; // update HUD clock once per second

// ─────────────────────────────────────────────
// Weather API configuration (Google Weather API)
// ─────────────────────────────────────────────
const char* WEATHER_HOST = "weather.googleapis.com";

// Weather API key storage (NVS-based, configured via admin panel)
static Preferences prefs;
// Demo recording storage (NVS-based, chunked blobs)
static Preferences demoPrefs;

// Demo persistence is stored in SPIFFS as binary files to avoid NVS/Preferences
// per-entry size limits and NVS fragmentation/free-page issues on long recordings.
static bool gSpiffsMounted = false;
static bool gSpiffsInitAttempted = false;
static bool gSpiffsSaveNotMountedWarned = false;

static inline uint8_t demoSlotFromMapId(uint8_t mapId) {
  // NOTE: Map IDs are defined later in the sketch, but demo persistence helpers live
  // near the top. Using numeric IDs here avoids a “not declared in this scope” error
  // from referencing enum values before they’re defined.
  // Overworld map IDs are stable: 1,2,3.
  return (mapId == 2) ? 2 : (mapId == 3) ? 3 : 1;
}

static inline const char* demoFilePathForMap(uint8_t mapId) {
  switch (demoSlotFromMapId(mapId)) {
    case 1: return "/demo/map1.bin";
    case 2: return "/demo/map2.bin";
    case 3: return "/demo/map3.bin";
    default: return "/demo/map1.bin";
  }
}

static inline const char* demoTempPathForMap(uint8_t mapId) {
  switch (demoSlotFromMapId(mapId)) {
    case 1: return "/demo/map1.tmp";
    case 2: return "/demo/map2.tmp";
    case 3: return "/demo/map3.tmp";
    default: return "/demo/map1.tmp";
  }
}

static void initSPIFFS() {
  if (gSpiffsInitAttempted) return;
  gSpiffsInitAttempted = true;

  // --- Partition table diagnostics (boot-time only) ---
  // NOTE: Avoid iterator-based dumping here; some core builds have hit heap asserts.
  // Use only esp_partition_find_first() to print partitions we care about.
  {
    const esp_partition_t *p = NULL;

    p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
    if (p) {
      Serial.printf("[FS] DATA subtype=0x%02X label=%s size=%u\n",
                    (unsigned)p->subtype, p->label, (unsigned)p->size);
    }

    p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, NULL);
    if (p) {
      Serial.printf("[FS] DATA subtype=0x%02X label=%s size=%u\n",
                    (unsigned)p->subtype, p->label, (unsigned)p->size);
    }

    p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
    if (p) {
      Serial.printf("[FS] DATA subtype=0x%02X label=%s size=%u\n",
                    (unsigned)p->subtype, p->label, (unsigned)p->size);
    }

    p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
    if (p) {
      Serial.printf("[FS] DATA subtype=0x%02X label=%s size=%u\n",
                    (unsigned)p->subtype, p->label, (unsigned)p->size);
    }
  }

  const esp_partition_t *spiffsPart = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA,
      ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
      NULL);

  if (!spiffsPart) {
    Serial.println("[FS] NO SPIFFS partition found in partition table (DATA/SPIFFS).");
    Serial.println("[FS] Partition Scheme mismatch: verify Tools->Partition Scheme actually applied to this board build.");
    gSpiffsMounted = false;
    return;
  }

  Serial.printf("[FS] Found SPIFFS partition label=%s addr=0x%08X size=%u\n",
                spiffsPart->label,
                (unsigned)spiffsPart->address,
                (unsigned)spiffsPart->size);

  // --- Mount attempts ---
  Serial.printf("[FS] SPIFFS mount attempt label=%s ...\n", spiffsPart->label);
  gSpiffsMounted = SPIFFS.begin(true);

  if (!gSpiffsMounted) {
    Serial.printf("[FS] SPIFFS mount attempt label=%s ...\n", spiffsPart->label);
    // Retry using the explicit partition label we found in the table.
    gSpiffsMounted = SPIFFS.begin(true, "/spiffs", 10, spiffsPart->label);
  }

  if (!gSpiffsMounted) {
    Serial.printf("[FS] SPIFFS mount FAIL (label=%s)\n", spiffsPart->label);
    Serial.println("[FS] SPIFFS mount FAIL (even after format)");
    return;
  }

  // Ensure /demo exists.
  if (!DEMO_FS.exists("/demo")) {
    if (DEMO_FS.mkdir("/demo")) {
      Serial.println("[FS] Created /demo");
    }
  }

  const unsigned totalB = (unsigned)DEMO_FS.totalBytes();
  const unsigned usedB  = (unsigned)DEMO_FS.usedBytes();
  const unsigned freeB  = (totalB >= usedB) ? (totalB - usedB) : 0;
  Serial.printf("[FS] SPIFFS mount OK total=%u used=%u free=%u\n", totalB, usedB, freeB);
}
static String gWeatherApiKey = "";
static bool gWeatherReady = false;

// Location coordinates (NVS-backed, defaults to Salt Lake City, UT)
static float gLatitude  = 40.7608;   // default fallback
static float gLongitude = -111.8910; // default fallback

// Forward declarations for weather key and location management
void loadWeatherKey(const char* tag);
void saveWeatherKey(const String& key);
void clearWeatherKey();
bool hasWeatherKey();
void debugWeatherKey(const char* tag);
void loadLocation();
void saveLocation(float lat, float lon);
void startWeatherIfReady();

// ─────────────────────────────────────────────
// HUD weather state (updated via HTTP API)
// ─────────────────────────────────────────────
static bool gWeatherHasTemp = false;  // true only when valid temp fetched
static int  gWeatherTempF = 0;        // default 0 (displays as 000F)

// Precipitation HUD state
// Percent is the probability of precipitation (0..100).
// IMPORTANT: preserve the last known value if a fetch fails or precip is missing.
static bool gWeatherHasPrecip = false;
static int  gWeatherPrecipPercent = 0;
static bool gWeatherPrecipIsSnow = false;

unsigned long gLastWeatherFetchMs = 0;
const unsigned long WEATHER_FETCH_INTERVAL_MS = 10UL * 60UL * 1000UL; // 10 minutes

// If a fetch attempt fails early (e.g., TLS connect fails), don't spam retries every frame.
// Instead, allow a shorter retry interval than the normal fetch interval.
const unsigned long WEATHER_RETRY_MIN_MS = 60UL * 1000UL; // 60 seconds

// Weather fetch can block on DNS/TLS/reads. Run it asynchronously so gameplay never freezes.
static volatile bool gWeatherFetchInProgress = false;
static uint32_t gWeatherFetchStartedMs = 0;


static inline void configureWeatherTlsClient(WiFiClientSecure &client) {
  client.setInsecure();
  // Weather is best-effort, but Google TLS handshakes can take a few seconds
  // depending on Wi-Fi conditions. Too-tight timeouts look like "connect failed".
  client.setTimeout(8000);
#if defined(ESP32)
  client.setHandshakeTimeout(8000);
#endif
  // Note: Some ESP32 Arduino cores (and some ESP32-S3 variants) don't expose
  // WiFiClientSecure::setBufferSizes(), so we avoid calling it for compatibility.

  // If available, reduce TLS IO buffers to ease heap pressure.
  // Default buffers can be large; with ~43KB free heap at fetch time, Google TLS can fail.
  WeatherCompat::setBufferSizesIfSupported(client, 4096, 1024);
}

static inline void markWeatherFetchAttemptNow() {
  const unsigned long nowMs = millis();
  // Preserve existing scheduling logic (based on gLastWeatherFetchMs + WEATHER_FETCH_INTERVAL_MS)
  // but shift timestamp so next attempt happens after WEATHER_RETRY_MIN_MS.
  if (WEATHER_FETCH_INTERVAL_MS > WEATHER_RETRY_MIN_MS) {
    gLastWeatherFetchMs = nowMs - (WEATHER_FETCH_INTERVAL_MS - WEATHER_RETRY_MIN_MS);
  } else {
    gLastWeatherFetchMs = nowMs;
  }
}

// ─────────────────────────────────────────────
// Weather Key Management (NVS Storage)
// ─────────────────────────────────────────────
void loadWeatherKey(const char* tag) {
  Serial.printf("[WeatherKey] %s load...\n", tag);
  prefs.begin("mario", true);  // read-only
  gWeatherApiKey = prefs.getString("wxkey", "");
  prefs.end();
  
  // Log result without printing actual key
  Serial.printf("[WeatherKey] %s: %s (len=%u)\n", 
    tag, 
    gWeatherApiKey.length() ? "SET" : "MISSING",
    (unsigned)gWeatherApiKey.length());
  
  if (gWeatherApiKey.length() > 0) {
    gWeatherReady = true;
    Serial.printf("[Weather] Using saved key: YES (len=%u)\n", (unsigned)gWeatherApiKey.length());
  } else {
    Serial.println("[Weather] Using saved key: NO");
  }
}

void saveWeatherKey(const String& key) {
  prefs.begin("mario", false);  // read-write
  prefs.putString("wxkey", key);
  prefs.end();
  gWeatherApiKey = key;
  gWeatherReady = (key.length() > 0);
  Serial.printf("[WeatherKey] saved: len=%u\n", (unsigned)key.length());
}

void clearWeatherKey() {
  prefs.begin("mario", false);  // read-write
  prefs.remove("wxkey");
  prefs.end();
  gWeatherApiKey = "";
  gWeatherReady = false;
  Serial.println("[Weather] API key cleared from NVS");
}

bool hasWeatherKey() {
  return gWeatherApiKey.length() > 0;
}

void debugWeatherKey(const char* tag) {
  Serial.printf("[WeatherKey] %s: len=%u\n", tag, (unsigned)gWeatherApiKey.length());
  Serial.printf("[WeatherKey] %s: %s\n", tag, gWeatherApiKey.length() ? "SET" : "MISSING");
}

void loadLocation() {
  prefs.begin("mario", true);  // read-only
  gLatitude  = prefs.getFloat("lat",  gLatitude);
  gLongitude = prefs.getFloat("lon",  gLongitude);
  prefs.end();
  Serial.printf("[Weather] Location loaded: %.6f, %.6f\n", gLatitude, gLongitude);
  Serial.printf("[Weather] Using saved location: %.6f, %.6f\n", gLatitude, gLongitude);
}

void saveLocation(float lat, float lon) {
  prefs.begin("mario", false);  // read-write
  prefs.putFloat("lat", lat);
  prefs.putFloat("lon", lon);
  prefs.end();
  gLatitude = lat;
  gLongitude = lon;
  Serial.printf("[Weather] Location saved: %.6f, %.6f\n", gLatitude, gLongitude);
}

void startWeatherIfReady() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!hasWeatherKey()) {
    static bool once = false;
    if (!once) {
      once = true;
      Serial.println("[Weather] Skipping: no API key set");
    }
    return;
  }
  gWeatherReady = true;
  gLastWeatherFetchMs = 0;  // Force immediate fetch on next tick
  Serial.println("[Weather] Ready. Fetching now...");
}

// ─────────────────────────────────────────────
// Arduino IDE OTA removed - use Web OTA portal only
// ─────────────────────────────────────────────

// ─────────────────────────────────────────────
// Sprite frames (18x14) – hero
// (same as your current engine)
// ─────────────────────────────────────────────

const int SPRITE_W     = 14;
const int SPRITE_H     = 18;
const int SPRITE_SCALE = 1;

const int heroW = SPRITE_W * SPRITE_SCALE;
const int heroH = SPRITE_H * SPRITE_SCALE;

// size-aware hitbox heights (boots → hat)
enum HeroSize : uint8_t { SIZE_SMALL = 0, SIZE_BIG = 1 };
HeroSize heroSize = SIZE_SMALL;

const float HERO_HEIGHT_SMALL = 14.0f;

// Big Mario must fit in a 2-tile (16px) gap.
// 18px causes false ceiling collisions even when the sprite looks like it fits.
const float HERO_HEIGHT_BIG   = 16.0f;

float heroHitH = HERO_HEIGHT_SMALL;

// Sprite-to-hitbox alignment (render only)
const int FOOT_OFFSET_BIG   = 1;  // was 3; lift big Mario up 2px so feet sit on ground
const int FOOT_OFFSET_SMALL = 2;

// where the hat "top" is for ceiling bonk (approx)
const int HAT_TOP_OFFSET = 4;

// ─────────────────────────────────────────────
// Tile map / world
// ─────────────────────────────────────────────
const int TILE_SIZE    = 8;

// World size: 10 screens wide × 2 screens tall
// Each screen = 192×64 pixels = 24×8 tiles
const int LEVEL_WIDTH  = 240;  // 24 tiles × 10 screens
const int LEVEL_HEIGHT = 32;   // 8 tiles × 4 screens

const int WORLD_WIDTH  = LEVEL_WIDTH * TILE_SIZE;
const int WORLD_HEIGHT = LEVEL_HEIGHT * TILE_SIZE;

// Large tunnel sprite collider (pixels)
const int TUNNEL_W = 18;
const int TUNNEL_H = 17;

// Left entrance sprite footprint (TileID 60)
static const int PIPE_LEFT_W = 17;
static const int PIPE_LEFT_H = 18;
static const int PIPE_LEFT_SINK_DIST = 16;
static const int PIPE_LEFT_MOUTH_PAD_Y = 4;

static inline void getTunnelColliderDims(uint8_t tileId, float &outW, float &outH) {
  // Upright/down tunnels are 18×17. Left-entrance tunnels are 17×18.
  if (tileId == TILE_ID_GREEN_TUNNEL_LEFT || tileId == TILE_ID_GREEN_TUNNEL_LEFT_2_4) {
    outW = 17.0f;
    outH = 18.0f;
  } else {
    outW = (float)TUNNEL_W;
    outH = (float)TUNNEL_H;
  }
}

// Runtime tile lookup (level[][]). Definition appears later in the file.
static inline uint8_t getTileId(int tx, int ty);

inline bool isTunnelTileId(uint8_t id) {
  return (
    id == TILE_ID_GREEN_TUNNEL ||
    id == TILE_ID_GREEN_TUNNEL_DOWN ||
    id == TILE_ID_GREEN_TUNNEL_LEFT ||
    id == TILE_ID_GREEN_TUNNEL_1_2 ||
    id == TILE_ID_GREEN_TUNNEL_DOWN_2_2 ||
    id == TILE_ID_GREEN_TUNNEL_LEFT_2_4 ||
    id == TILE_ID_GREEN_TUNNEL_4_4
  );
}

// Large tunnel collider tiles include NO-TELEPORT variants.
// Teleport/entry behavior is driven by the tunnel pairing table (TileIDs from Map_Build.h).
inline bool isTunnelColliderTileId(uint8_t id) {
  return (
    id == TILE_ID_GREEN_TUNNEL ||
    id == TILE_ID_GREEN_TUNNEL_DOWN ||
    id == TILE_ID_GREEN_TUNNEL_NOTEL ||
    id == TILE_ID_GREEN_TUNNEL_LEFT ||
    id == TILE_ID_GREEN_TUNNEL_1_2 ||
    id == TILE_ID_GREEN_TUNNEL_DOWN_2_2 ||
    id == TILE_ID_GREEN_TUNNEL_LEFT_2_4 ||
    id == TILE_ID_GREEN_TUNNEL_4_4
  );
}

// Given a pixel point (px,py), try to find a tunnel anchor tile whose
// 18x17 rect covers that pixel. If found, write anchor tile coords to
// outTx/outTy and return true. This back-checks up to 2 tiles left.
static inline bool findTunnelAnchorAtPixel(int px, int py, int &outTx, int &outTy) {
  int tx = px >> 3; // px / TILE_SIZE
  int ty = py >> 3;
  if (tx < 0 || ty < 0 || tx >= LEVEL_WIDTH || ty >= LEVEL_HEIGHT) return false;

  // Check current tile
  uint8_t id0 = getTileId(tx, ty);
  if (isTunnelColliderTileId(id0)) { outTx = tx; outTy = ty; return true; }

  // Check up to two tiles left (overhang area)
  if (tx - 1 >= 0) {
    uint8_t id1 = getTileId(tx - 1, ty);
    if (isTunnelColliderTileId(id1)) { outTx = tx - 1; outTy = ty; return true; }
  }
  if (tx - 2 >= 0) {
    uint8_t id2 = getTileId(tx - 2, ty);
    if (isTunnelColliderTileId(id2)) { outTx = tx - 2; outTy = ty; return true; }
  }

  return false;
}

// Viewport and map constants for tile map rendering
const int VIEW_TILES_W = 24;  // 192 / 8 = 24 tiles wide
const int VIEW_TILES_H = 8;   // 64 / 8 = 8 tiles tall

const int MAP_TILES_W = 240;  // 24 tiles × 10 screens
const int MAP_TILES_H = 32;   // 8 tiles × 4 screens

// Current map selection (pointers to PROGMEM maps).
// Default is Map_Build to preserve existing behavior.
enum BuiltinMapId : uint8_t {
  MAP_ID_BUILD = 0,
  MAP_ID_OVERWORLD_1 = 1,
  MAP_ID_OVERWORLD_2 = 2,
  MAP_ID_OVERWORLD_3 = 3,
};

// Change this ONE variable to pick which built-in map loads.
// (Pause/menu UI switching can be added later.)
uint8_t gCurrentMapId = MAP_ID_OVERWORLD_1;
// (moved earlier)

static inline uint8_t nextMapAfterClear(uint8_t currentId) {
  switch (currentId) {
    case MAP_ID_OVERWORLD_1: return MAP_ID_OVERWORLD_2;
    case MAP_ID_OVERWORLD_2: return MAP_ID_OVERWORLD_3;
    case MAP_ID_OVERWORLD_3: return MAP_ID_OVERWORLD_1;
    default: return MAP_ID_OVERWORLD_1;
  }
}

static const uint8_t (*gCurrentMapTiles)[MAP_TILES_W] = Map_Build;
static const uint8_t (*gCurrentMapMobs)[MAP_TILES_W]  = Map_Mobs;

// NOTE: Arduino's sketch preprocessor can auto-insert prototypes above this block.
// Using a plain integer type here avoids "type not declared" issues for that prototype.
static void selectBuiltinMap(uint8_t id) {
  switch (id) {
    default:
    case MAP_ID_BUILD:
      gCurrentMapTiles = Map_Build;
      gCurrentMapMobs  = Map_Mobs;
      break;
    case MAP_ID_OVERWORLD_1:
      gCurrentMapTiles = Map_Overworld_1;
      gCurrentMapMobs  = Map_Overworld_1_Mobs;
      break;
    case MAP_ID_OVERWORLD_2:
      gCurrentMapTiles = Map_Overworld_2;
      gCurrentMapMobs  = Map_Overworld_2_Mobs;
      break;
    case MAP_ID_OVERWORLD_3:
      gCurrentMapTiles = Map_Overworld_3;
      gCurrentMapMobs  = Map_Overworld_3_Mobs;
      break;
  }
}

// Single helper to get the selected map (pointers + dimensions).
// IMPORTANT: signature uses only builtin types to avoid Arduino auto-prototype issues.
static inline void getCurrentMapSelection(const uint8_t (**outTiles)[MAP_TILES_W],
                                         const uint8_t (**outMobs)[MAP_TILES_W],
                                         uint16_t *outWidthTiles,
                                         uint16_t *outHeightTiles)
{
  selectBuiltinMap(gCurrentMapId);

  if (outTiles) *outTiles = gCurrentMapTiles;
  if (outMobs)  *outMobs  = gCurrentMapMobs;
  if (outWidthTiles)  *outWidthTiles  = (uint16_t)MAP_TILES_W;
  if (outHeightTiles) *outHeightTiles = (uint16_t)MAP_TILES_H;
}

enum TileType : uint8_t {
  TILE_EMPTY = 0,
  TILE_GROUND = 1,
  TILE_Q_MUSHROOM = 2,   // ? block that spawns mushroom
  TILE_Q_GREEN_MUSHROOM = 24, // ? block that spawns green 1UP mushroom
  TILE_SOLID_SHAKE = 3,  // non-breaking "bump" block
  TILE_BRICK_BREAK = 4,  // breakable brick
  TILE_USED = 5,         // used ? block (solid)
  TILE_COIN = 6,         // collectible coin

  TILE_GROUND_TOP_LEFT      = 7,
  TILE_GROUND_TOP_RIGHT     = 8,
  TILE_GROUND_WALL_LEFT     = 9,
  TILE_GROUND_WALL_RIGHT    = 10,
  TILE_GROUND_CENTER_DIRT   = 11,
  TILE_GROUND_GRASS_SIDE_LEFT  = 12,
  TILE_GROUND_GRASS_SIDE_RIGHT = 13,
  TILE_GROUND_SLOPE_UP_RIGHT   = 14,
  TILE_GROUND_SLOPE_UP_LEFT    = 15,
  TILE_GROUND_VERT_CLIFF_LEFT  = 16,
  TILE_GROUND_VERT_CLIFF_RIGHT = 17,
  TILE_SEMI_GROUND             = 18   // one-way semi-solid platform (SemiGround_Tile_Array)
};

// Phase 2: Map tile IDs from Map_Build to physics TileType
// All terrain tiles mapped to appropriate physics types for collision
static TileType mapTileIdToTileType(uint8_t tileId) {
  switch (tileId) {
    // Empty/Air
    case TILE_ID_EMPTY:
      return TILE_EMPTY;
    
    // Basic ground tile
    case TILE_ID_GROUND:
      return TILE_GROUND;
    // Stone ledge behaves as ground
    case TILE_ID_STONE_LEDGE:
      return TILE_GROUND;
    
    // Top corners: treat like semi-solid platform (matches TILE_ID_SEMI_GROUND behavior)
    case TILE_ID_GROUND_TOP_LEFT:
      return TILE_SEMI_GROUND;
    case TILE_ID_GROUND_TOP_RIGHT:
      return TILE_SEMI_GROUND;
    
    // Vertical walls
    case TILE_ID_GROUND_WALL_LEFT:
      return TILE_GROUND_WALL_LEFT;
    case TILE_ID_GROUND_WALL_RIGHT:
      return TILE_GROUND_WALL_RIGHT;
    
    // Underground dirt fill
    case TILE_ID_GROUND_CENTER_DIRT:
      return TILE_GROUND_CENTER_DIRT;
    
    // Slopes (treat as solid blocks for now; smooth slope physics in future phase)
    case TILE_ID_SLOPE_UP_RIGHT:
      return TILE_GROUND_SLOPE_UP_RIGHT;
    case TILE_ID_SLOPE_UP_LEFT:
      return TILE_GROUND_SLOPE_UP_LEFT;

    // Underground roof should act like normal ground
    case TILE_ID_UNDERGROUND_ROOF:
      return TILE_GROUND;
    // Underground roof fill left should also act as solid ground
    case TILE_ID_UNDERGROUND_ROOF_FILL_LEFT:
      return TILE_GROUND;

    // Underground right wall should act like normal ground as well
    case TILE_ID_UNDERGROUND_WALL_RIGHT:
      return TILE_GROUND;

    // Underground left wall should also act like normal ground
    case TILE_ID_UNDERGROUND_WALL_LEFT:
      return TILE_GROUND;

    // Underground roof fill right should also act like normal ground
    case TILE_ID_UNDERGROUND_ROOF_FILL_RIGHT:
      return TILE_GROUND;

    // Underground ground fill (left) should also act as solid ground
    case TILE_ID_UNDERGROUND_GROUND_FILL_LEFT:
      return TILE_GROUND;

    // Underground ground fill (right) should also act as solid ground
    case TILE_ID_UNDERGROUND_GROUND_FILL_RIGHT:
      return TILE_GROUND;

    // Underground floor/ground should also act as solid ground
    case TILE_ID_UNDERGROUND_GROUND:
      return TILE_GROUND;

    // Treat grass/edge ground tiles as true solid ground for gameplay
    case TILE_ID_GRASS_SIDE_LEFT:
    case TILE_ID_GRASS_SIDE_RIGHT:
    case TILE_ID_FILL_LEFT:
    case TILE_ID_FILL_RIGHT:
    case TILE_ID_GRASS_DIRT_FILL_LEFT:
    case TILE_ID_GRASS_DIRT_FILL_RIGHT:
    case TILE_ID_GRASS_STRAIGHT_SIDE_RIGHT:
    case TILE_ID_GRASS_STRAIGHT_SIDE_LEFT:
      return TILE_GROUND;  // map to solid ground

    // Vertical cliff IDs should be treated as slope tile types at runtime
    case TILE_ID_VERT_CLIFF_LEFT:
      return TILE_GROUND_VERT_CLIFF_LEFT;
    case TILE_ID_VERT_CLIFF_RIGHT:
      return TILE_GROUND_VERT_CLIFF_RIGHT;
    
    // Semi-ground (one-way platform)
    case TILE_ID_SEMI_GROUND:
      return TILE_SEMI_GROUND;  // Dedicated type for jump-through platform

    case TILE_ID_BRICK_CLOUD:
      return TILE_SEMI_GROUND;
    
    // Large decorative tiles
    case TILE_ID_GREEN_TUNNEL:
      return TILE_GROUND;  // Solid pipe/tunnel for now
    case TILE_ID_GREEN_TUNNEL_DOWN:
      return TILE_GROUND;  // Solid downward tunnel entrance
    case TILE_ID_GREEN_TUNNEL_NOTEL:
      return TILE_GROUND;  // Solid tunnel (no teleport)
    case TILE_ID_GREEN_TUNNEL_LEFT:
      return TILE_GROUND;  // Solid left-entrance tunnel
    case TILE_ID_GREEN_TUNNEL_1_2:
    case TILE_ID_GREEN_TUNNEL_DOWN_2_2:
    case TILE_ID_GREEN_TUNNEL_LEFT_2_4:
    case TILE_ID_GREEN_TUNNEL_4_4:
      return TILE_GROUND;  // Named tunnel variants: solid like tunnels
    case TILE_ID_BUSH:
      return TILE_EMPTY;   // Decoration only; Mario walks through

    case TILE_ID_LAVA:
      return TILE_EMPTY;   // Decoration-only for now (same as bush)

    case TILE_ID_BG_CLOUD_PUFF_A:
    case TILE_ID_BG_CLOUD_LONG_A:
      return TILE_EMPTY;   // Decoration only; no collision

    case TILE_ID_MARIO_CASTLE:
      return TILE_EMPTY;   // Decoration only; no collision

    // Checkpoint flags (decoration only for now)
    case TILE_ID_CHECK_POINT:
    case TILE_ID_CHECKED_POINT:
      return TILE_EMPTY;

    // Flagpole (split tiles)
    // - Pole + top are grabbable (handled by flagpole logic) but not solid.
    // - Base is solid so Mario can land on it at the end of the slide.
    case TILE_ID_FLAG_BASE:
      return TILE_GROUND;
    
    // Interactive blocks and items
    case TILE_ID_Q_BLOCK:
      return TILE_Q_MUSHROOM;  // Question block
    case TILE_ID_SOLID_BLOCK:
      return TILE_SOLID_SHAKE;  // Non-breaking solid block
    case TILE_ID_BRICK:
      return TILE_BRICK_BREAK;  // Breakable brick
    case TILE_ID_USED_BLOCK:
      return TILE_USED;  // Used/empty block
    case TILE_ID_BRICK_COIN:
      return TILE_SOLID_SHAKE; // New coin-brick behaves as solid bumpable block for now
    case TILE_ID_GREEN_MUSHROOM_BRICK:
      return TILE_Q_GREEN_MUSHROOM; // Green 1UP brick
    case TILE_ID_COIN:
      return TILE_COIN;  // Collectible coin

    // Mob decorative tile (visual-only) -> non-solid
    case TILE_ID_MOB_GOOMBA:
    case TILE_ID_MOB_KOOPA_TROOPA:
      return TILE_EMPTY;
    case TILE_ID_MOB_PIRANHA_PLANT_L:
    case TILE_ID_MOB_PIRANHA_PLANT_R:
      return TILE_EMPTY;
    // Unknown/unhandled tile IDs
    default:
      return TILE_EMPTY;
  }
}

// Inverse mapper: convert physics TileType back to a TileID for legacy loaders/wrappers
static inline uint8_t tileTypeToTileId(TileType t) {
  switch (t) {
    case TILE_EMPTY:            return TILE_ID_EMPTY;

    // Ground family
    case TILE_GROUND:           return TILE_ID_GROUND;
    case TILE_GROUND_TOP_LEFT:  return TILE_ID_GROUND_TOP_LEFT;
    case TILE_GROUND_TOP_RIGHT: return TILE_ID_GROUND_TOP_RIGHT;
    case TILE_GROUND_WALL_LEFT: return TILE_ID_GROUND_WALL_LEFT;
    case TILE_GROUND_WALL_RIGHT:return TILE_ID_GROUND_WALL_RIGHT;
    case TILE_GROUND_CENTER_DIRT:return TILE_ID_GROUND_CENTER_DIRT;
    case TILE_GROUND_SLOPE_UP_RIGHT:   return TILE_ID_SLOPE_UP_RIGHT;
    case TILE_GROUND_SLOPE_UP_LEFT:    return TILE_ID_SLOPE_UP_LEFT;
    case TILE_GROUND_VERT_CLIFF_LEFT:  return TILE_ID_VERT_CLIFF_LEFT;
    case TILE_GROUND_VERT_CLIFF_RIGHT: return TILE_ID_VERT_CLIFF_RIGHT;

    // Semisolid
    case TILE_SEMI_GROUND:      return TILE_ID_SEMI_GROUND;

    // Bricks / items
    case TILE_Q_MUSHROOM:       return TILE_ID_Q_BLOCK;
    case TILE_Q_GREEN_MUSHROOM: return TILE_ID_GREEN_MUSHROOM_BRICK;
    case TILE_SOLID_SHAKE:      return TILE_ID_SOLID_BLOCK;
    case TILE_BRICK_BREAK:      return TILE_ID_BRICK;
    case TILE_USED:             return TILE_ID_USED_BLOCK;

    // Collectibles
    case TILE_COIN:             return TILE_ID_COIN;

    default:                    return TILE_ID_EMPTY;
  }
}

// Phase 4: Slope physics now use true linear ramp math (no lookup tables)

// Phase 3B: Compute ground Y for hero or mushroom standing on a slope tile
// Uses a simple linear ramp across the tile instead of 8-step lookup tables.
// Compute the Y position of the slope surface (feet position) at a given world X
// for a single slope tile. Works for both hero and mushroom.
static float slopeGroundYForHero(TileType t, int tileX, int tileY, float centerXWorld) {
  // Treat vertical cliff types as their corresponding slope equivalents
  if (t == TILE_GROUND_VERT_CLIFF_LEFT)  t = TILE_GROUND_SLOPE_UP_LEFT;
  if (t == TILE_GROUND_VERT_CLIFF_RIGHT) t = TILE_GROUND_SLOPE_UP_RIGHT;
  const float tileLeft    = tileX * TILE_SIZE;
  const float tileTopY    = tileY * TILE_SIZE;
  const float tileBottomY = tileTopY + TILE_SIZE;

  // 0.0 = left edge of tile, 1.0 = right edge of tile
  float frac = (centerXWorld - tileLeft) / (float)TILE_SIZE;
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;

  float feetYOnSlope;

  if (t == TILE_GROUND_SLOPE_UP_RIGHT) {
    // Up-right: left = bottom, right = top.
    // As Mario moves right, he goes up.
    feetYOnSlope = tileBottomY - frac * TILE_SIZE;
  } else {
    // Up-left: left = top, right = bottom.
    // As Mario moves left, he goes up.
    feetYOnSlope = tileTopY + frac * TILE_SIZE;
  }

  return feetYOnSlope;
}

inline bool tileIsSlope(TileType t) {
  return (t == TILE_GROUND_SLOPE_UP_RIGHT ||
          t == TILE_GROUND_SLOPE_UP_LEFT  ||
          t == TILE_GROUND_VERT_CLIFF_LEFT ||
          t == TILE_GROUND_VERT_CLIFF_RIGHT);
}

bool tileIsSolid(TileType t) {
  // Slopes are *not* solid for horizontal collision.
  // We still treat them as "floor" using tileIsSlope() in computeHeroGroundY().
  if (tileIsSlope(t)) {
    return false;
  }

  // Background tiles that Mario can walk through (decorative only):
  if (t == TILE_GROUND_TOP_LEFT ||
      t == TILE_GROUND_TOP_RIGHT ||
      t == TILE_GROUND_WALL_LEFT ||
      t == TILE_GROUND_WALL_RIGHT ||
      t == TILE_GROUND_CENTER_DIRT) {
    return false;
  }

  // Semi-solid platform is not solid (one-way only)
  if (t == TILE_SEMI_GROUND) {
    return false;
  }

  // Solid tiles that block movement:
  return (t == TILE_GROUND ||
      t == TILE_Q_MUSHROOM ||
      t == TILE_Q_GREEN_MUSHROOM ||
          t == TILE_SOLID_SHAKE ||
          t == TILE_BRICK_BREAK ||
          t == TILE_USED ||
          t == TILE_GROUND_GRASS_SIDE_LEFT ||
          t == TILE_GROUND_GRASS_SIDE_RIGHT);
}

// Slopes should not fully block horizontal movement – the ground
// logic already uses slopeGroundYForHero for them.
bool tileBlocksSide(TileType t) {
  if (tileIsSlope(t)) {
    return false;  // let slope be handled by vertical/ground logic
  }

  // Background tiles never block (decorative only)
  if (t == TILE_GROUND_CENTER_DIRT ||
      t == TILE_GROUND_WALL_LEFT ||
      t == TILE_GROUND_WALL_RIGHT ||
      t == TILE_GROUND_TOP_LEFT ||
      t == TILE_GROUND_TOP_RIGHT) {
    return false;
  }

  // Semi-solid platforms should not block horizontal movement
  if (t == TILE_SEMI_GROUND) {
    return false;
  }

  return tileIsSolid(t);
}

// Semi-solid platforms you can stand on, jump up through, and crouch-drop through.
bool tileIsDropThroughPlatform(TileType t) {
  // Only the dedicated semi-ground tile is a one-way/drop-through platform.
  // Ground flat tiles must always be solid.
  return (t == TILE_SEMI_GROUND);
}

// TileID-based helpers: authoritative checks by TileID (runtime-level map[] stores TileIDs)
static inline bool isSolidTileId(uint8_t id) {
  switch (id) {
    case TILE_ID_EMPTY:
      return false;

    // Core ground / walls / cliffs / slopes are solid via TileType mapping,
    // but we keep some explicit IDs here too.
    case TILE_ID_GROUND:
    case TILE_ID_GRASS_SIDE_LEFT:
    case TILE_ID_GRASS_SIDE_RIGHT:
    case TILE_ID_GRASS_STRAIGHT_SIDE_RIGHT:
    case TILE_ID_GRASS_STRAIGHT_SIDE_LEFT:

    // Green tunnel tiles must be solid so shells can’t pass through
    case TILE_ID_GREEN_TUNNEL:
    case TILE_ID_GREEN_TUNNEL_DOWN:
    case TILE_ID_GREEN_TUNNEL_NOTEL:
    case TILE_ID_GREEN_TUNNEL_LEFT:

    // Named tunnel variants / larger tunnel exits (also solid)
    case TILE_ID_GREEN_TUNNEL_1_2:
    case TILE_ID_GREEN_TUNNEL_DOWN_2_2:
    case TILE_ID_GREEN_TUNNEL_LEFT_2_4:
    case TILE_ID_GREEN_TUNNEL_4_4:

    // Solid blocks/bricks should also be solid
    case TILE_ID_SOLID_BLOCK:
    case TILE_ID_BRICK:
    case TILE_ID_USED_BLOCK:
    case TILE_ID_Q_BLOCK:

    // Stone ledge is terrain
    case TILE_ID_STONE_LEDGE:
      return true;

    default:
      // Everything else: defer to TileType physics mapping
      return tileIsSolid(mapTileIdToTileType(id));
  }
}

static inline bool isHazardTileId(uint8_t id) {
  switch (id) {
    case TILE_ID_LAVA:
      return true;
    default:
      return false;
  }
}

static inline bool isSemiSolidTileId(uint8_t id) {
  return (id == TILE_ID_SEMI_GROUND); // only ID 21
}

// ─────────────────────────────────────────────
// Mob-specific collision rules (Goomba/Koopa)
//
// Goal:
// - Goomba/Koopa must NOT phase through terrain tiles.
// - The only *terrain* tiles they may pass through are:
//   - Dirt Platform Top Corner (R)  -> TILE_ID_GROUND_TOP_RIGHT
//   - Wall (L)                      -> TILE_ID_GROUND_WALL_LEFT
//   - Wall (R)                      -> TILE_ID_GROUND_WALL_RIGHT
// - They should turn around before stepping into Lava.
//
// NOTE: We keep Mario physics unchanged; this is mob-only logic.
// ─────────────────────────────────────────────

static inline bool mobCanWalkThroughTerrainTileId(uint8_t id) {
  (void)id;
  // Mobs should not phase through terrain tiles.
  return false;
}

static inline bool mobTileBlocksSideByTileId(uint8_t id) {
  // Grass edge/straight tiles must ALWAYS block mobs sideways.
  // These are terrain tiles (not decoration) and should behave like solid blocks
  // for Goomba + Koopa.
  if (id == TILE_ID_GRASS_STRAIGHT_SIDE_LEFT ||
      id == TILE_ID_GRASS_STRAIGHT_SIDE_RIGHT ||
      id == TILE_ID_GRASS_SIDE_LEFT ||
      id == TILE_ID_GRASS_SIDE_RIGHT) {
    return true;
  }

  // These tiles are rendered as terrain blocks but are intentionally non-solid
  // for Mario (background/edge TileType). Enemies must treat them as blocking.
  if (id == TILE_ID_GROUND_TOP_LEFT ||
      id == TILE_ID_GROUND_TOP_RIGHT ||
      id == TILE_ID_GROUND_WALL_LEFT ||
      id == TILE_ID_GROUND_WALL_RIGHT ||
      id == TILE_ID_GROUND_CENTER_DIRT) {
    return true;
  }

  // Air / decoration / overlay markers
  switch (id) {
    case TILE_ID_EMPTY:
    case TILE_ID_COIN:
    case TILE_ID_BUSH:
    case TILE_ID_BG_CLOUD_PUFF_A:
    case TILE_ID_BG_CLOUD_LONG_A:
    case TILE_ID_MARIO_CASTLE:
    case TILE_ID_CHECK_POINT:
    case TILE_ID_CHECKED_POINT:
    case TILE_ID_MOB_GOOMBA:
    case TILE_ID_MOB_KOOPA_TROOPA:
    case TILE_ID_MOB_PIRANHA_PLANT_L:
    case TILE_ID_MOB_PIRANHA_PLANT_R:
      return false;

    // One-way platforms: don't block sideways
    case TILE_ID_SEMI_GROUND:
    case TILE_ID_BRICK_CLOUD:
      return false;

    // Lava is handled as a lookahead hazard (turn-around) rather than a wall.
    case TILE_ID_LAVA:
      return false;
  }

  // Slopes: allow side entry so mobs can climb/walk them.
  if (id == TILE_ID_SLOPE_UP_LEFT || id == TILE_ID_SLOPE_UP_RIGHT) {
    return false;
  }

  // Vertical cliffs should behave like walls for mobs.
  if (id == TILE_ID_VERT_CLIFF_LEFT || id == TILE_ID_VERT_CLIFF_RIGHT) {
    return true;
  }

  // Everything else uses the standard solid logic.
  // This includes TILE_ID_GROUND_TOP_LEFT and TILE_ID_GROUND_CENTER_DIRT,
  // which are intentionally treated as blocking for mobs.
  return isSolidTileId(id);
}

static inline bool mobHazardAhead(float nextX, float topY, float w, float h, int dir) {
  // Probe a point slightly in front of the mob, and slightly *below* its feet
  // so lava pits get detected before stepping into them.
  float baseX = (dir < 0) ? (nextX + 1.0f) : (nextX + w - 2.0f);
  float probeX = baseX + (float)dir * 3.0f;
  if (probeX < 0.0f) probeX = 0.0f;
  if (probeX > (float)WORLD_WIDTH - 1.0f) probeX = (float)WORLD_WIDTH - 1.0f;

  float probeY = topY + h + 2.0f;
  if (probeY < 0.0f) probeY = 0.0f;
  if (probeY > (float)WORLD_HEIGHT - 1.0f) probeY = (float)WORLD_HEIGHT - 1.0f;

  uint8_t tid = (uint8_t)tileIdAtWorld(probeX, probeY);
  return isHazardTileId(tid);
}

static inline bool goombaSideBlocked(float nextX, float y, float w, float h, int dir, float insetX) {
  const float probeX = (dir < 0) ? (nextX + insetX) : (nextX + w - 1.0f - insetX);

  // Probe multiple points vertically so short mobs don't miss walls,
  // and so walls at different heights (e.g., platform edges) are detected.
  // NOTE: Do not early-return on slopes; slopes are simply treated as non-blocking
  // by mobTileBlocksSideByTileId(), while other sampled tiles may still be walls.
  const float sampleYs[3] = {
    y + 1.0f,
    y + (h * 0.5f),
    y + h - 2.0f
  };

  for (int i = 0; i < 3; ++i) {
    float sy = sampleYs[i];
    if (sy < 0.0f) sy = 0.0f;
    if (sy > (float)WORLD_HEIGHT - 1.0f) sy = (float)WORLD_HEIGHT - 1.0f;
    uint8_t tid = (uint8_t)tileIdAtWorld(probeX, sy);
    if (mobTileBlocksSideByTileId(tid)) return true;
  }

  return false;
}

// Returns true for one-way platform tiles (semi-solid) - DEPRECATED, use tileIsDropThroughPlatform
bool tileIsOneWayPlatform(TileType t) {
  // One-way platform semantics apply only to the semi-ground tile.
  return (t == TILE_SEMI_GROUND);
}

// Returns true for grass-top tiles (flat, top-left, top-right)
bool tileIsGrassTop(TileType t) {
  return (t == TILE_GROUND ||        // maps to Ground_Tile_Array
          t == TILE_GROUND_TOP_LEFT ||  // maps to Ground_Tile_Top_Left
          t == TILE_GROUND_TOP_RIGHT);  // maps to Ground_Tile_Top_Right
}

// Map mode flag: true = using Map_Build from Map_Build.h, false = using level[] array
bool gUsingTileMap = false;

// mutable runtime level stores TileIDs (authoritative runtime map)
uint8_t level[LEVEL_HEIGHT][LEVEL_WIDTH]; // runtime TileID grid (authoritative)

// render-only overlay mob layer (does not affect tiles, tunnels, or physics)
// 0 = none, otherwise TILE_ID_MOB_* values from Map_Build.h
uint8_t mobLayer[LEVEL_HEIGHT][LEVEL_WIDTH];

// per-tile bounce animation
int8_t   tileOffset[LEVEL_HEIGHT][LEVEL_WIDTH];
uint8_t  tileAnimTimer[LEVEL_HEIGHT][LEVEL_WIDTH];

// Camera globals may be defined later; declare extern so functions here compile
extern float cameraX;
extern float cameraY;

// Simple brick fragment particle system
struct BrickFrag {
  bool    active;
  float   x, y;    // world coordinates (pixels)
  float   vx, vy;  // velocity (pixels/frame)
  uint8_t life;    // frames remaining
  uint16_t color;  // RGB565 color
};

const int MAX_BRICK_FRAGS = 24;
BrickFrag gBrickFrags[MAX_BRICK_FRAGS];

// Spawn 4 small fragments from a broken brick at tile (tx,ty)
void spawnBrickFragmentsAt(int tx, int ty) {
  int worldX = tx * TILE_SIZE;
  int worldY = ty * TILE_SIZE;

  // fragment spawn offsets (corners)
  const float ox[4] = {1.0f, 5.0f, 1.0f, 5.0f};
  const float oy[4] = {1.0f, 1.0f, 5.0f, 5.0f};
  const float vx[4] = {-0.9f, 0.9f, -0.5f, 0.5f};
  const float vy[4] = {-2.6f, -2.2f, -1.6f, -1.2f};

  // color for fragments (brick main color)
  uint16_t fragColor = matrix->color565(150, 75, 30);

  for (int i = 0; i < 4; ++i) {
    // find free slot
    for (int j = 0; j < MAX_BRICK_FRAGS; ++j) {
      if (!gBrickFrags[j].active) {
        gBrickFrags[j].active = true;
        gBrickFrags[j].x = worldX + ox[i];
        gBrickFrags[j].y = worldY + oy[i];
        gBrickFrags[j].vx = vx[i] + ((rand() & 31) - 16) * 0.01f;
        gBrickFrags[j].vy = vy[i] + ((rand() & 31) - 16) * 0.01f;
        gBrickFrags[j].life = 40 + (rand() & 31);
        gBrickFrags[j].color = fragColor;
        break;
      }
    }
  }
}

// Update fragments (called from game update loop)
void updateBrickFragments() {
  const float GRAV = 0.14f;
  for (int i = 0; i < MAX_BRICK_FRAGS; ++i) {
    BrickFrag &f = gBrickFrags[i];
    if (!f.active) continue;
    f.vy += GRAV;
    f.x += f.vx;
    f.y += f.vy;
    if (f.life > 0) f.life--;
    if (f.life == 0 || f.y > WORLD_HEIGHT) f.active = false;
  }
}

// Draw fragments (called from render path)
void drawBrickFragments() {
  for (int i = 0; i < MAX_BRICK_FRAGS; ++i) {
    BrickFrag &f = gBrickFrags[i];
    if (!f.active) continue;
    int sx = (int)(f.x - cameraX);
    int sy = (int)(f.y - cameraY);
    // draw small 2x2 pixel chunk
    for (int yy = 0; yy < 2; ++yy) {
      int py = sy + yy;
      if (py < 0 || py >= MATRIX_HEIGHT) continue;
      for (int xx = 0; xx < 2; ++xx) {
        int px = sx + xx;
        if (px < 0 || px >= MATRIX_WIDTH) continue;
        matrix->drawPixel(px, py, f.color);
      }
    }
  }
}

// ─────────────────────────────────────────────
// Tunnel system (pipes) – auto-paired from Map_Build
// ─────────────────────────────────────────────

struct TunnelEndpoint {
  int     tileX;
  int     tileY;
  uint8_t tileId;  // Named TileID from Map_Build.h (e.g. TILE_ID_GREEN_TUNNEL, _DOWN_2_2, _LEFT_2_4)
};

struct TunnelPair {
  TunnelEndpoint entry;  // upright pipe (Mario crouches on top)
  TunnelEndpoint exit;   // matching downward pipe

  TunnelEndpoint leftEntry;
  bool hasLeftEntry;
};

const int MAX_TUNNEL_PAIRS = 8;
TunnelPair gTunnelPairs[MAX_TUNNEL_PAIRS];
int        gTunnelPairCount = 0;

// ─────────────────────────────────────────────
// Pipe transition state (SMW-style animation)
// ─────────────────────────────────────────────
enum PipePhase : uint8_t {
  PIPE_PHASE_IDLE = 0,
  PIPE_PHASE_SINK,       // Mario sinking into entry pipe
  PIPE_PHASE_FADE_OUT,   // Fade to black
  PIPE_PHASE_HOLD_BLACK, // Hold at full black (prevents flash)
  PIPE_PHASE_TELEPORT,   // Hero/camera reposition
  PIPE_PHASE_FADE_IN,    // Fade from black
  PIPE_PHASE_RISE        // Mario rising/falling out of exit pipe
};

struct PipeTransitionState {
  bool active;              // true while in any pipe transition
  PipePhase phase;          // current phase

  bool entryIsLeft;         // if true, SINK moves horizontally (side-entry)
  int8_t entrySideDir;      // -1: slide left into pipe, +1: slide right into pipe

  float entrySnapX;          // snapped hero X at the moment we start side-entry
  float entrySnapY;          // snapped hero Y at the moment we start side-entry

  uint8_t pairIndex;        // which pipe pair we're using
  float entryX, entryY;     // entry pipe world pos (top center)
  float exitX, exitY;       // exit pipe world pos (spawn point)

  // Explicit exit endpoint (tile-anchored), so different entry types can choose
  // which pipe tile to exit from without changing tunnel pairing rules.
  int exitTileX;
  int exitTileY;
  uint8_t exitTileId;

  float heroStartX, heroStartY;   // hero pos when transition began
  float heroOffset;               // used for sink / rise offset (pixels)
  uint32_t phaseStartMs;          // millis() when this phase started

  bool exitIsDownPipe;      // true if Ground_Tile_Green_Tunnel_Down, etc.
  
  // Normalized animation progress
  float t;                  // 0.0 -> 1.0 normalized progress for enter/exit
  float duration;           // seconds, same value used by both enter and exit
  
  // Blackout phase control (prevents 1-frame flash)
  uint8_t holdBlackFrames;  // frames to hold at pure black before teleport
  bool teleportDone;        // teleport completed while black
};

static PipeTransitionState gPipeTransition;
static bool gHeroVisibleDuringPipe = true;

// Fade value: 0 = normal, 255 = fully black
static uint8_t gPipeFadeAmount = 0;

// ─────────────────────────────────────────────
// Death sequence (SMW-style) – reuses the same fade visuals as pipes
// ─────────────────────────────────────────────
enum DeathPhase : uint8_t {
  DEATH_NONE = 0,
  DEATH_POP_UP,
  DEATH_FALLING,
  DEATH_FADE_OUT,
  DEATH_BLACK_GAMEOVER,
  DEATH_FADE_IN,
  DEATH_DONE
};

// SMW-like death tuning (slower + floatier)
static const float DEATH_POP_VY_START = -1.9f;   // slower initial pop
static const float DEATH_GRAVITY_POP  =  0.08f;  // very light decel on the way up
static const float DEATH_GRAVITY_FALL =  0.11f;  // slow fall acceleration
static const float DEATH_VY_MAX_FALL  =  2.0f;   // slow terminal velocity
static const int   DEATH_POP_TICKS    =  60;     // longer hang time before falling

struct DeathState {
  bool active;
  DeathPhase phase;
  uint16_t ticks;
  float vy;
  uint32_t phaseStartMs;

  // True once we've performed the full fresh-start reset while the screen is black.
  bool didResetWhileBlack;

  // Respawn target chosen at time of death
  int16_t respawnTx;
  int16_t respawnTy;
  bool doHardReset;

  // Snapshot of checkpoint state at the time of death.
  // Used to preserve checkpoints across the fresh-start world reset.
  bool checkpointWasActive;
  int16_t checkpointTx;
  int16_t checkpointTy;

  // GAME / OVER slide positions during black
  int16_t gameX;
  int16_t overX;
};

static DeathState gDeath = { false, DEATH_NONE, 0, 0.0f, 0, false, -1, -1, false, false, -1, -1, 0, 0 };

static inline bool isDeathActive() {
  return gDeath.active;
}

static inline bool isDeathFading() {
  return gDeath.active &&
         (gDeath.phase == DEATH_FADE_OUT ||
          gDeath.phase == DEATH_BLACK_GAMEOVER ||
          gDeath.phase == DEATH_FADE_IN);
}

static inline bool deathIsBlackPhase() {
  return gDeath.active && (gDeath.phase == DEATH_BLACK_GAMEOVER);
}

static inline bool deathIsFadingPhase() {
  return gDeath.active && (gDeath.phase == DEATH_FADE_OUT || gDeath.phase == DEATH_FADE_IN);
}

static inline bool deathBlocksWorldRender() {
  return gDeath.active &&
         !gDeath.didResetWhileBlack &&
         (gDeath.phase == DEATH_FADE_OUT || gDeath.phase == DEATH_BLACK_GAMEOVER);
}

// Forward declarations for globals/functions used by death code (defined later).
extern float heroX;
extern float heroY;
extern float heroVX;
extern float heroVY;
extern bool  onGround;
extern float cameraY;
extern float slopeSlideVX;
extern bool  slopeSlideActive;
extern unsigned long gHeroInvincUntilMs;

static void respawnHeroAtMarkerTile(int16_t markerTx, int16_t markerTy);
void doHardReset();
void resetWorld(bool clearTransitionsAndCutscenes);

static void restoreCheckpointAfterWorldReset(bool wasActive, int16_t tx, int16_t ty);

static void resetWorldFreshStart();

static inline bool isPipeTransitionActive() {
  return gPipeTransition.active;
}

// Mario should render behind the pipe sprite during the phases where he is
// partially inside the pipe.
static inline bool heroShouldRenderBehindPipe() {
  if (!gPipeTransition.active) return false;
  switch (gPipeTransition.phase) {
    case PIPE_PHASE_SINK:
    case PIPE_PHASE_RISE:
      return true;
    default:
      return false;
  }
}

// Prevent immediately re-entering a DOWN pipe on the first post-exit frame.
// This also protects demo playback from edge-detection mismatches when DOWN is held.
static bool gPipeEntryNeedsDownRelease = false;

// ─────────────────────────────────────────────
// Death sequence helpers
// ─────────────────────────────────────────────

static void startDeathSequence(int16_t respawnTx, int16_t respawnTy, bool doHardReset,
                               bool checkpointWasActive, int16_t checkpointTx, int16_t checkpointTy) {
  // Do not interrupt pipe transition; also prevent re-trigger.
  if (gPipeTransition.active) return;
  if (gDeath.active) return;

  gDeath.active = true;
  gDeath.phase = DEATH_POP_UP;
  gDeath.ticks = 0;
  gDeath.vy = DEATH_POP_VY_START;
  gDeath.phaseStartMs = gameplayNowMs();
  gDeath.didResetWhileBlack = false;

  gDeath.respawnTx = respawnTx;
  gDeath.respawnTy = respawnTy;
  gDeath.doHardReset = doHardReset;

  gDeath.checkpointWasActive = checkpointWasActive;
  gDeath.checkpointTx = checkpointTx;
  gDeath.checkpointTy = checkpointTy;

  // Initialize GAME/OVER slide positions (off-screen)
  gDeath.gameX = -50;
  gDeath.overX = (int16_t)MATRIX_WIDTH + 10;

  // Freeze hero motion from normal physics
  heroVX = 0.0f;
  heroVY = 0.0f;
  slopeSlideVX = 0.0f;
  slopeSlideActive = false;
  gHeroInvincUntilMs = 0;

  // Fade starts later; ensure fade is cleared.
  gPipeFadeAmount = 0;
}

static void updateDeathSequence(uint32_t nowMs, float dtSeconds) {
  if (!gDeath.active) return;

  // Convert dt to ~frames for tuning consistency (loop is ~60 FPS).
  float dtFrames = dtSeconds * 60.0f;
  if (dtFrames < 0.5f) dtFrames = 0.5f;
  if (dtFrames > 6.0f) dtFrames = 6.0f;

  switch (gDeath.phase) {
    case DEATH_POP_UP: {
      heroY += gDeath.vy * dtFrames;
      gDeath.vy += DEATH_GRAVITY_POP * dtFrames;
      gDeath.ticks++;
      if (gDeath.ticks >= DEATH_POP_TICKS) {
        gDeath.phase = DEATH_FALLING;
        gDeath.ticks = 0;
      }
      break;
    }

    case DEATH_FALLING: {
      gDeath.vy += DEATH_GRAVITY_FALL * dtFrames;
      if (gDeath.vy > DEATH_VY_MAX_FALL) gDeath.vy = DEATH_VY_MAX_FALL;
      heroY += gDeath.vy * dtFrames;
      heroVX = 0.0f;
      heroVY = 0.0f;
      onGround = false;

      // When Mario is well below the visible screen, start fade out.
      if (heroY > (cameraY + (float)MATRIX_HEIGHT + 40.0f)) {
        gDeath.phase = DEATH_FADE_OUT;
        gDeath.phaseStartMs = nowMs;
        gDeath.ticks = 0;
      }
      break;
    }

    case DEATH_FADE_OUT: {
      const uint32_t FADE_OUT_MS = 700; // must match pipe fade
      uint32_t elapsed = nowMs - gDeath.phaseStartMs;
      if (elapsed > FADE_OUT_MS) elapsed = FADE_OUT_MS;
      gPipeFadeAmount = (uint8_t)map(elapsed, 0, FADE_OUT_MS, 0, 255);

      if (elapsed >= FADE_OUT_MS) {
        gPipeFadeAmount = 255;
        gDeath.phase = DEATH_BLACK_GAMEOVER;
        gDeath.ticks = 0;

        // Reset slide-in positions each death.
        gDeath.gameX = -50;
        gDeath.overX = (int16_t)MATRIX_WIDTH + 10;
      }
      break;
    }

    case DEATH_BLACK_GAMEOVER: {
      gPipeFadeAmount = 255; // force full black

      // While fully black, reset the entire world to a fresh-start state exactly once.
      // This guarantees fade-in only ever reveals the spawn-state (no flash of death location).
      if (!gDeath.didResetWhileBlack) {
        gDeath.didResetWhileBlack = true;

        if (gDeath.doHardReset) {
          doHardReset();
        } else {
          resetWorldFreshStart();

          // Restore checkpoint state after the fresh-start world reset,
          // so checkpoint respawn works across Map_Build and Overworld maps.
          restoreCheckpointAfterWorldReset(gDeath.checkpointWasActive, gDeath.checkpointTx, gDeath.checkpointTy);

          // Now put Mario at the selected respawn marker tile.
          // (Hard reset handles its own spawn / map selection.)
          respawnHeroAtMarkerTile(gDeath.respawnTx, gDeath.respawnTy);
        }

        // Safety: force black again in case resetWorld cleared any fade state.
        gPipeFadeAmount = 255;
      }

      // Slide GAME from left and OVER from right to meet at center.
      const int16_t targetGameX = 76;
      const int16_t targetOverX = 76 + 20; // second half starts after first 20 cols
      const int16_t step = 4;

      if (gDeath.gameX < targetGameX) {
        gDeath.gameX = (int16_t)min<int>(targetGameX, gDeath.gameX + step);
      }
      if (gDeath.overX > targetOverX) {
        gDeath.overX = (int16_t)max<int>(targetOverX, gDeath.overX - step);
      }

      gDeath.ticks++;
      // Hold black + GAME OVER for ~2.5 seconds.
      if (gDeath.didResetWhileBlack && gDeath.ticks >= 150) {
        gDeath.phase = DEATH_FADE_IN;
        gDeath.phaseStartMs = nowMs;
        gDeath.ticks = 0;
      }
      break;
    }

    case DEATH_FADE_IN: {
      const uint32_t FADE_IN_MS = 700; // must match pipe fade
      uint32_t elapsed = nowMs - gDeath.phaseStartMs;
      if (elapsed > FADE_IN_MS) elapsed = FADE_IN_MS;
      gPipeFadeAmount = (uint8_t)map(elapsed, 0, FADE_IN_MS, 255, 0);

      if (elapsed >= FADE_IN_MS) {
        gPipeFadeAmount = 0;
        gDeath.phase = DEATH_DONE;
        gDeath.ticks = 0;
      }
      break;
    }

    case DEATH_DONE: {
      gDeath.active = false;
      gDeath.phase = DEATH_NONE;
      gDeath.ticks = 0;
      break;
    }

    case DEATH_NONE:
    default:
      gDeath.active = false;
      gDeath.phase = DEATH_NONE;
      break;
  }
}

// ─────────────────────────────────────────────
// Shared pipe animation curve helpers
// ─────────────────────────────────────────────

static float clamp01(float v) { 
  return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); 
}

// Pipe animation curve (linear for now, matching original behavior)
static float pipeCurve(float t) {
  t = clamp01(t);
  return t; // linear
}

// ─────────────────────────────────────────────
// Mushroom (rising, then walking) state
// ─────────────────────────────────────────────
// Item types for mushrooms
enum ItemType : uint8_t {
  ITEM_RED_MUSHROOM = 0,
  ITEM_GREEN_MUSHROOM = 1
};

struct Mushroom {
  bool  active;
  bool  rising;
  float x;
  float y;
  float vx;
  float vy;
  float targetY;
  uint8_t type; // ITEM_RED_MUSHROOM or ITEM_GREEN_MUSHROOM
};
Mushroom gMushroom = { false, true, 0, 0, 0, 0, 0, ITEM_RED_MUSHROOM };

const float MUSH_W        = 8.0f;
const float MUSH_H        = 7.0f;
const float MUSH_GRAVITY  = 0.25f;
const float MUSH_MAX_FALL = 2.5f;
const float MUSH_SPEED    = 0.6f;

// ─────────────────────────────────────────────
// Goomba (simple walking enemy - Phase 1)
// ─────────────────────────────────────────────
struct Goomba {
  bool active;
  float x;
  float y;
  float vx;
  float vy;
  uint8_t animFrame; // 0/1
  uint16_t animTimer;
  uint8_t state; // 0 = alive, 1 = squished, 2 = dead
  uint8_t stateTimer;
};

const int MAX_GOOMBAS = 16;
Goomba gGoombas[MAX_GOOMBAS];

const float GOOMBA_W        = 7.0f;
const float GOOMBA_H        = 8.0f;
const float GOOMBA_GRAVITY  = MUSH_GRAVITY;
const float GOOMBA_MAX_FALL = MUSH_MAX_FALL;
const float GOOMBA_SPEED    = 0.45f; // slightly slower than mushrooms
const float GOOMBA_MAX_STEP_UP = 4.0f; // pixels allowed for stepping up

// Goomba states
static const uint8_t GOOMBA_STATE_ALIVE = 0;
static const uint8_t GOOMBA_STATE_SQUISHED = 1;
static const uint8_t GOOMBA_STATE_DEAD = 2;

// ─────────────────────────────────────────────
// Koopa Troopa (Goomba-like walking enemy)
// Spawned from Map_Build via TILE_ID_MOB_KOOPA_TROOPA (57)
// ─────────────────────────────────────────────
struct Koopa {
  bool active;
  float x;
  float y;
  float vx;
  float vy;
  int8_t dir;           // -1 = left, +1 = right (movement direction)
  int8_t facing;        // -1 = left, +1 = right (render facing)
  uint8_t animFrame;     // 0/1
  uint32_t lastAnimMs;   // walk frame toggle

  // Stuck detection
  float lastX;
  uint8_t stuckTicks;
};

const int MAX_KOOPAS = 12;
Koopa gKoopas[MAX_KOOPAS];

const float KOOPA_W = 11.0f;
const float KOOPA_H = 19.0f;
const float KOOPA_GRAVITY = GOOMBA_GRAVITY;
const float KOOPA_MAX_FALL = GOOMBA_MAX_FALL;
const float KOOPA_SPEED = GOOMBA_SPEED;
const float KOOPA_MAX_STEP_UP = GOOMBA_MAX_STEP_UP;

// ─────────────────────────────────────────────
// Koopa Shell (SMW-ish carry / kick / sliding)
// ─────────────────────────────────────────────
enum ShellState : uint8_t {
  SHELL_STATIONARY = 0,
  SHELL_SLIDING    = 1,
  SHELL_HELD       = 2,
  SHELL_THROWN_UP  = 3
};

struct KoopaShell {
  bool active;
  float x;
  float y;
  float vx;
  float vy;
  int8_t dir;
  uint8_t state;

  // Grace window after kick/drop so Mario doesn't insta-hurt
  uint32_t ignoreHeroUntilMs;
};

static const int MAX_SHELLS = 8;
static KoopaShell gShells[MAX_SHELLS];

static const float SHELL_W          = 7.0f;
static const float SHELL_H          = 5.0f;
static const float SHELL_GRAVITY    = GOOMBA_GRAVITY;
static const float SHELL_MAX_FALL   = GOOMBA_MAX_FALL;
// Shell tuning
static const float SHELL_KICK_SPEED   = 1.60f;   // slightly faster
static const float SHELL_BOUNCE_DAMP  = 1.00f;
static const uint32_t SHELL_KICK_GRACE_MS = 650; // longer grace so you can’t die instantly after kick

// Up-throw tuning
static const float SHELL_THROW_UP_VY = -3.6f;   // upward launch speed
static const float SHELL_THROW_UP_VX = 0.0f;    // straight up
static const uint32_t SHELL_THROW_GRACE_MS = 250;

static void resetShells();
static int  spawnShellAt(float x, float y);
static void updateShells(bool runPressed, bool runJustReleased, bool upPressed);
static void drawShells();
static void checkShellEnemyCollisions();
static void checkShellHeroCollisions(bool runPressed);

static void throwHeldShellUp();

// Koopa-specific slope / cliff helpers (TileID-based)
// NOTE: Prompts may refer to these IDs using Ground_Tile_*_ID naming.
#ifndef Ground_Tile_Slope_Up_Left_ID
#define Ground_Tile_Slope_Up_Left_ID TILE_ID_SLOPE_UP_LEFT
#endif
#ifndef Ground_Tile_Slope_Up_Right_ID
#define Ground_Tile_Slope_Up_Right_ID TILE_ID_SLOPE_UP_RIGHT
#endif
#ifndef Ground_Tile_Vert_Cliff_Left_ID
#define Ground_Tile_Vert_Cliff_Left_ID TILE_ID_VERT_CLIFF_LEFT
#endif
#ifndef Ground_Tile_Vert_Cliff_Right_ID
#define Ground_Tile_Vert_Cliff_Right_ID TILE_ID_VERT_CLIFF_RIGHT
#endif

static inline bool isKoopaSlopeTile(uint16_t tileId) {
  return tileId == Ground_Tile_Slope_Up_Left_ID ||
         tileId == Ground_Tile_Slope_Up_Right_ID;
}

static inline bool isKoopaVerticalCliff(uint16_t tileId) {
  return tileId == Ground_Tile_Vert_Cliff_Left_ID ||
         tileId == Ground_Tile_Vert_Cliff_Right_ID;
}

static inline uint16_t tileIdAtWorld(float wx, float wy) {
  if (wx < 0.0f || wy < 0.0f) return (uint16_t)TILE_ID_EMPTY;
  int tx = (int)(wx / TILE_SIZE);
  int ty = (int)(wy / TILE_SIZE);
  if (tx < 0 || tx >= LEVEL_WIDTH || ty < 0 || ty >= LEVEL_HEIGHT) return (uint16_t)TILE_ID_EMPTY;
  return (uint16_t)getTileId(tx, ty);
}

static inline bool koopaSideBlocked(float nextX, float y, int dir) {
  const float probeX = (dir < 0) ? nextX : (nextX + KOOPA_W - 1.0f);

  // Probe multiple points vertically so Koopas don't miss thin/offset walls.
  // NOTE: Do not early-return on slopes; slopes are treated as non-blocking by
  // mobTileBlocksSideByTileId(), while other sampled tiles may still be walls.
  const float sampleYs[3] = {
    y + 1.0f,
    y + (KOOPA_H * 0.5f),
    y + KOOPA_H - 2.0f
  };

  for (int i = 0; i < 3; ++i) {
    float sy = sampleYs[i];
    if (sy < 0.0f) sy = 0.0f;
    if (sy > (float)WORLD_HEIGHT - 1.0f) sy = (float)WORLD_HEIGHT - 1.0f;
    uint8_t tid = (uint8_t)tileIdAtWorld(probeX, sy);
    if (mobTileBlocksSideByTileId(tid)) return true;
  }

  return false;
}

// Hero invincibility after taking damage (ms)
unsigned long gHeroInvincUntilMs = 0;
const unsigned long HERO_INVINC_MS = 2000UL; // 2 seconds

// Forward declarations (defined later in this file)
extern float heroX;
extern float heroY;

static void checkHeroLavaDamage() {
  // Respect invincibility window so you don't get spam-hit every frame
  if (gameplayNowMs() < gHeroInvincUntilMs) return;

  // Use hero hitbox (same one you use for enemy collision)
  float hx1 = heroX;
  float hx2 = heroX + heroW;
  float hy1 = heroY;
  float hy2 = heroY + heroHitH;

  // Sample a few points near Mario's feet + mid-body so lava contact is reliable
  float sampleX[3] = { hx1 + 2, (hx1 + hx2) * 0.5f, hx2 - 2 };
  float sampleY[3] = { hy2 - 1, hy2 - 4, (hy1 + hy2) * 0.5f };

  for (int ix = 0; ix < 3; ix++) {
    for (int iy = 0; iy < 3; iy++) {
      uint8_t tid = (uint8_t)tileIdAtWorld(sampleX[ix], sampleY[iy]);
      if (!isHazardTileId(tid)) continue;

      // HIT LAVA
      // Use the existing enemy damage pathway so animation + i-frames match.
      // If big -> shrink; if small -> die.
      damageMarioFromEnemy();
      return;
    }
  }
}

// ─────────────────────────────────────────────
// Piranha Plant (SMW-style hazard)
// Spawned from Map_Mobs overlay (mobLayer[][]) using TILE_ID_MOB_PIRANHA_PLANT_L (45).
// TILE_ID_MOB_PIRANHA_PLANT_R (46) is a helper tile and is ignored.
// ─────────────────────────────────────────────
// -------------------- PIRANHA PLANT (Pipe Enemy) --------------------
static const int MAX_PIRANHAS = 16;

// SMW-ish timing (tweak later)
static const uint16_t PIRANHA_HIDDEN_MS = 1100;
static const uint16_t PIRANHA_OUT_MS    = 1100;

// Animation speed in pixels per update (1 = smooth, 2 = faster)
static const uint8_t PIRANHA_PIXELS_PER_TICK = 1;

// If Mario is close to the pipe, you *may* want to pause rising (optional)
// For this project we want the piranha to run purely on its timer cycle.
static const bool PIRANHA_PAUSE_IF_MARIO_NEAR = false;
static const int  PIRANHA_MARIO_NEAR_PX = 22; // horizontal distance threshold

enum PiranhaState : uint8_t {
  PIRANHA_HIDDEN = 0,
  PIRANHA_RISING,
  PIRANHA_OUT,
  PIRANHA_SINKING
};

struct PiranhaPlant {
  bool active;

  // Tile anchor (where mob tile 45 was placed) -- this is the tile row ABOVE the pipe tile.
  int16_t tileX;
  int16_t tileY;

  // Pixel positioning
  int16_t pipeTopYpx;   // y pixel where pipe opening begins (mask line)
  int16_t xpx;          // sprite draw x (world px)

  // Reveal animation (0..PIRANHA_H)
  uint8_t visibleH;

  // State machine
  PiranhaState state;
  uint32_t stateStartMs;

  // Animation frame toggle
  uint8_t  animFrame;
  uint32_t lastAnimMs;
};

static PiranhaPlant gPiranhas[MAX_PIRANHAS];

static const uint8_t PIRANHA_W = 18;
static const uint8_t PIRANHA_H = 46;

// Collision/pipe-entry grace threshold (visible pixels)
static const uint8_t PIRANHA_BLOCK_THRESHOLD_PX = 6;

// ─────────────────────────────────────────────
// Coins / score
// ─────────────────────────────────────────────
uint16_t coinCount = 0;

// ─────────────────────────────────────────────
// SMW-style coin pop animation (emerge + vanish)
// ─────────────────────────────────────────────
struct CoinPop {
  bool active;
  int16_t startX, startY;   // world px (top-left of 6x6 coin) INSIDE the block
  int16_t endX, endY;       // world px target (1 tile above/below)
  uint8_t age;              // frames
};

static const uint8_t COINPOP_LIFE = 12;   // short & snappy like SMW
static const uint8_t MAX_COIN_POPS = 8;

CoinPop gCoinPops[MAX_COIN_POPS];

// Track remaining coins inside coin bricks (0 = uninitialized)
uint8_t coinBrickRemaining[LEVEL_HEIGHT][LEVEL_WIDTH];

static inline float easeOutQuad(float t) {
  return 1.0f - (1.0f - t) * (1.0f - t);
}

void spawnCoinPopSMW(int tx, int ty, int8_t dir) {
  int slot = -1;
  for (int i = 0; i < MAX_COIN_POPS; i++) {
    if (!gCoinPops[i].active) { slot = i; break; }
  }
  if (slot < 0) return;

  const int coinW = 6;
  const int coinH = 6;
  int blockX = tx * TILE_SIZE;
  int blockY = ty * TILE_SIZE;

  // START: INSIDE the block (centered)
  int startX = blockX + (TILE_SIZE - coinW) / 2;
  int startY = blockY + (TILE_SIZE - coinH) / 2;

  // END: exactly 1 tile (8px) above/below the block, still centered
  int endX = startX;
  int endY = startY + (dir * TILE_SIZE);

  gCoinPops[slot].active = true;
  gCoinPops[slot].startX = startX;
  gCoinPops[slot].startY = startY;
  gCoinPops[slot].endX = endX;
  gCoinPops[slot].endY = endY;
  gCoinPops[slot].age = 0;
}

void updateCoinPopsSMW() {
  for (int i = 0; i < MAX_COIN_POPS; i++) {
    if (!gCoinPops[i].active) continue;
    gCoinPops[i].age++;
    if (gCoinPops[i].age >= COINPOP_LIFE) gCoinPops[i].active = false;
  }
}

void drawCoinPopsSMW() {
  for (int i = 0; i < MAX_COIN_POPS; i++) {
    CoinPop &c = gCoinPops[i];
    if (!c.active) continue;
    float t = (float)c.age / (float)(COINPOP_LIFE - 1);
    float e = easeOutQuad(t);
    int worldX = (int)(c.startX + (c.endX - c.startX) * e);
    int worldY = (int)(c.startY + (c.endY - c.startY) * e);
    int sx = worldX - (int)cameraX;
    int sy = worldY - (int)cameraY;
    drawWorldTileCustomSize(&worldCoin[0][0], 6, 6, sx, sy);
  }
}

static inline void resetCoinPopsSMW() {
  for (int i = 0; i < MAX_COIN_POPS; i++) {
    gCoinPops[i].active = false;
    gCoinPops[i].age = 0;
  }
}

static inline void resetBrickFragments() {
  for (int i = 0; i < MAX_BRICK_FRAGS; i++) {
    gBrickFrags[i].active = false;
    gBrickFrags[i].life = 0;
  }
}

static void resetWorldFreshStart() {
  // Use the same initialization path as a brand-new map load.
  resetWorld(true);

  // Clear transient runtime visuals so nothing carries across deaths.
  resetCoinPopsSMW();
  resetBrickFragments();
}

// Lives (editor expects a 3-digit lives window). Create if missing.
int gMarioLives = 5; // start at 5

// Map countdown time in seconds (300 default)
int gMapTimeSeconds = 300;
unsigned long gLastMapTimeTickMs = 0;

// ─────────────────────────────────────────────
// Game mode
// ─────────────────────────────────────────────
enum GameMode : uint8_t {
  GAME_MODE_PLAYING = 0,
  GAME_MODE_PAUSED = 1,
  GAME_MODE_TITLE = 2,
  GAME_MODE_GAMEOVER = 3
};

GameMode gGameMode = GAME_MODE_PLAYING;

// Pause menu screen state
enum PauseScreen : uint8_t {
  PAUSE_SCREEN_MAIN = 0,       // RESET / MAPS / OPTIONS / STATS / RESUME
  PAUSE_SCREEN_MAPS = 1,       // MAP 1 / MAP 2 / MAP 3 / BUILD
  PAUSE_SCREEN_OPTIONS = 2,    // WI-FI / SCREEN / STATS / BACK
  PAUSE_SCREEN_WEB_PORTAL = 3, // WEB PORTAL (viewer screen)
  PAUSE_SCREEN_BRIGHTNESS = 4  // BRIGHTNESS (adjustment screen)
};

PauseScreen gPauseScreen = PAUSE_SCREEN_MAIN;

// Main pause menu selection (4 items: RESET, MAPS, OPTIONS, RESUME)
enum PauseSelection : uint8_t {
  PAUSE_RESET = 0,
  PAUSE_MAPS  = 1,
  PAUSE_OPTIONS = 2,
  PAUSE_RESUME = 3  // PAUSE_STATS removed - not in sprite
};

PauseSelection gPauseSelection = PAUSE_RESET;

// Maps screen selection (4 items: MAP 1, MAP 2, MAP 3, BUILD)
enum MapsSelection : uint8_t {
  MAPS_MAP_1 = 0,
  MAPS_MAP_2 = 1,
  MAPS_MAP_3 = 2,
  MAPS_BUILD = 3
};

MapsSelection gMapsSelection = MAPS_MAP_1;

// Options screen selection (4 items: WI-FI, SCREEN, STATS, BACK)
enum OptionsSelection : uint8_t {
  OPTIONS_WIFI = 0,
  OPTIONS_SCREEN = 1,
  OPTIONS_STATS = 2,
  OPTIONS_BACK = 3
};

OptionsSelection gOptionsSelection = OPTIONS_WIFI;

// Web Portal screen selection (1 item: BACK only)
enum WebPortalSelection : uint8_t {
  WEBPORTAL_BACK = 0
};

WebPortalSelection gWebPortalSelection = WEBPORTAL_BACK;

// Brightness screen selection (2 items: BAR and BACK)
enum BrightnessSelection : uint8_t {
  BRIGHTNESS_BAR = 0,   // Brightness adjustment bar
  BRIGHTNESS_BACK = 1   // BACK button
};

BrightnessSelection gBrightnessSelection = BRIGHTNESS_BAR;  // Start on bar

// Brightness control (0..255 for setBrightness8)
uint8_t gBrightnessLevel = 64;  // Current brightness (matches default in setup)

// ─────────────────────────────────────────────
// Game state
// ─────────────────────────────────────────────
float heroX;
float heroY;
static float heroPrevX = 0.0f;
static float heroPrevY = 0.0f;
float heroVX = 0.0f;
float heroVY = 0.0f;

// Snapshot of Mario's vertical state "approaching" collisions.
// This is captured each frame before ground snapping can zero-out heroVY.
// Used only for demo-lenient stomp logic.
float gHeroApproachY = 0.0f;
float gHeroApproachVY = 0.0f;
float gHeroApproachHitH = HERO_HEIGHT_SMALL;
bool  gHeroApproachOnGround = true;
bool  onGround = true;

// Slope slide state
bool  slopeSlideActive    = false;  // true while doing SMW-style slide
float slopeSlideVX        = 0.0f;   // horizontal speed during slide
bool  slopeSlideOnSlope   = false;  // tracks if currently on slope during slide
float slopeSlideOffSlopeX = 0.0f;   // x position when leaving slope

const float SLOPE_POST_SLIDE_DIST = 48.0f;  // 6 tiles minimum glide on flat ground

// Drop-through platform state
bool  platformDropActive = false;   // when true, ignore drop-through platforms as ground
float platformDropStartFeetY = 0.0f; // hero feet Y at the moment we start dropping

uint8_t walkPhase = 0;

bool prevNormalJumpPressed = false;
bool prevSpinJumpPressed   = false;
bool prevDownPressed       = false;
bool prevLeftPressed       = false;
bool prevRightPressed      = false;
bool prevResetPressed      = false;
bool prevStartPressed      = false;

// RUN edge detection (used for shell pickup/drop)
bool prevRunPressed        = false;

static inline void resetInputEdgeDetectionState() {
  prevNormalJumpPressed = false;
  prevSpinJumpPressed   = false;
  prevDownPressed       = false;
  prevLeftPressed       = false;
  prevRightPressed      = false;
  prevResetPressed      = false;
  prevStartPressed      = false;
  prevRunPressed        = false;
}

static inline void primeInputEdgeDetectionFromPlaybackFrame(uint16_t frameIdx) {
  if (frameIdx >= gDemoRecLength) return;
  const DemoRecFrame &f = gDemoRecBuf[frameIdx];
  InputState tmp = {0, 0.0f, 0,0,0,0,0,0,0};
  unpackButtonsToInput(f.buttons, tmp);

  prevNormalJumpPressed = (tmp.btnJump != 0);
  prevSpinJumpPressed   = (tmp.btnSpin != 0);
  prevRunPressed        = (tmp.btnRun  != 0);
  prevDownPressed       = (tmp.btnDown != 0);
  prevLeftPressed       = (f.moveX < 0);
  prevRightPressed      = (f.moveX > 0);
  prevResetPressed      = (tmp.btnReset != 0);
  prevStartPressed      = (tmp.btnStart != 0);
}

// Prevent “walking bonk → bounce backward” by clamping horizontal velocity briefly after a ceiling hit.
static uint8_t gCeilingBonkFreezeFrames = 0;   // ignores moveInput
static uint8_t gCeilingBonkHoldVXFrames = 0;   // forces heroVX = 0

// Tiny forward carry after a ceiling bonk (position nudge, collision-checked).
static uint8_t gCeilingBonkCarryFrames = 0;
static int8_t  gCeilingBonkCarryDir    = 0; // -1 left, +1 right

// Hero facing used for shell interactions (does not affect rendering)
int8_t gHeroFacingDir = 1; // -1 left, +1 right

// Shell carry state
bool gHeroHoldingShell = false;
int16_t gHeldShellIndex = -1;

bool spinActive  = false;
bool slamActive  = false;
// When true, we've just broken a tile via slam this frame—skip immediate landing snap
bool gJustBrokeTileThisFrame = false;

#if DEBUG_DEMO_MAP2_TRACE
static void demoTraceReset() {
  gDemoTraceRingHead = 0;
  gDemoTraceRingFilled = false;
  gDemoTraceKoopaDumped = false;
}

static void demoTracePush(uint32_t nowMs) {
  DemoTraceSample &s = gDemoTraceRing[gDemoTraceRingHead];
  s.t = nowMs;
  s.idx = gDemoTraceHaveLastFrame ? gDemoTraceLastFrameIdx : 0;
  s.mx  = gDemoTraceLastMoveX;
  s.btn = gDemoTraceLastButtons;
  s.hx = heroX;
  s.hy = heroY;
  s.hvx = heroVX;
  s.hvy = heroVY;
  s.onG = onGround ? 1 : 0;
  s.size = (uint8_t)heroSize;
  s.coins = (uint16_t)coinCount;

  // Track nearest active koopa (X distance).
  int nearestIdx = -1;
  float nearestDx = 1e9f;
  for (int i = 0; i < MAX_KOOPAS; ++i) {
    const Koopa &kk = gKoopas[i];
    if (!kk.active) continue;
    float dx = fabsf((kk.x + KOOPA_W * 0.5f) - (heroX + heroW * 0.5f));
    if (dx < nearestDx) {
      nearestDx = dx;
      nearestIdx = i;
    }
  }

  s.kIdx = (int8_t)nearestIdx;
  if (nearestIdx >= 0) {
    const Koopa &kk = gKoopas[nearestIdx];
    s.kAct = 1;
    s.kx = kk.x;
    s.ky = kk.y;
    s.kvx = kk.vx;
    s.kvy = kk.vy;
  } else {
    s.kAct = 0;
    s.kx = s.ky = s.kvx = s.kvy = 0.0f;
  }

  gDemoTraceRingHead++;
  if (gDemoTraceRingHead >= DEMO_TRACE_RING_CAP) {
    gDemoTraceRingHead = 0;
    gDemoTraceRingFilled = true;
  }
}

static void demoTraceDump(const char *tag) {
  Serial.print("[DTR] DUMP ");
  Serial.print(tag ? tag : "(null)");
  Serial.print(" map=");
  Serial.print(gCurrentMapId);
  Serial.print(" playIdx=");
  Serial.print(gDemoPlaybackIdx);
  Serial.print(" ringFilled=");
  Serial.println(gDemoTraceRingFilled ? 1 : 0);

  uint8_t count = gDemoTraceRingFilled ? DEMO_TRACE_RING_CAP : gDemoTraceRingHead;
  uint8_t start = gDemoTraceRingFilled ? gDemoTraceRingHead : 0;

  for (uint8_t n = 0; n < count; ++n) {
    uint8_t i = (uint8_t)(start + n);
    if (i >= DEMO_TRACE_RING_CAP) i -= DEMO_TRACE_RING_CAP;
    const DemoTraceSample &s = gDemoTraceRing[i];

    Serial.print("[DTR] t=");
    Serial.print(s.t);
    Serial.print(" idx=");
    Serial.print(s.idx);
    Serial.print(" mx=");
    Serial.print((int)s.mx);
    Serial.print(" btn=0x");
    if (s.btn < 16) Serial.print('0');
    Serial.print(s.btn, HEX);
    Serial.print(" hero=(");
    Serial.print(s.hx, 2);
    Serial.print(",");
    Serial.print(s.hy, 2);
    Serial.print(") v=(");
    Serial.print(s.hvx, 2);
    Serial.print(",");
    Serial.print(s.hvy, 2);
    Serial.print(") onG=");
    Serial.print((int)s.onG);
    Serial.print(" size=");
    Serial.print((int)s.size);
    Serial.print(" coins=");
    Serial.print((int)s.coins);
    if (s.kAct) {
      Serial.print(" k#");
      Serial.print((int)s.kIdx);
      Serial.print("=(");
      Serial.print(s.kx, 2);
      Serial.print(",");
      Serial.print(s.ky, 2);
      Serial.print(") v=(");
      Serial.print(s.kvx, 2);
      Serial.print(",");
      Serial.print(s.kvy, 2);
      Serial.print(")");
    }
    Serial.println();
  }
}
#endif

// landing squash
int  landingSquashFrames = 0;
const int LANDING_SQUASH_DURATION = 6;

// -------------------------
// Checkpoint system
// -------------------------
static bool checkpointActive = false;
static int16_t checkpointTileX = -1;
static int16_t checkpointTileY = -1;

// Original map spawn (captured on map load)
static int16_t spawnTileX = -1;
static int16_t spawnTileY = -1;

// Flip animation state
static bool checkpointFlipAnimating = false;
static int16_t checkpointFlipTileX = -1;
static int16_t checkpointFlipTileY = -1;
static uint32_t checkpointFlipStartMs = 0;
static const uint16_t CHECKPOINT_FLIP_MS = 450; // tweak

// ------------------------------------------------------------
// Checkpoint flag wiggle animation (visual only)
// ------------------------------------------------------------
struct CheckpointAnim {
  bool active = false;
  int tileX = -1;
  int tileY = -1;
  uint8_t startTileId = 0;   // what we show at start of anim
  uint8_t endTileId = 0;     // what we show near end of anim
  uint32_t startMs = 0;
  uint16_t durationMs = 520; // tweak for feel (450–650ms is nice)
};

static CheckpointAnim gCheckpointAnim;

// Small wiggle pattern (pixel offsets). Keeps it “snappy” on LED matrix.
static const int8_t kCheckpointWiggle[] = { 0, 1, 0, -1, 0, 2, 0, -2, 0, 1, 0, -1, 0 };
static const uint8_t kCheckpointWiggleCount = sizeof(kCheckpointWiggle) / sizeof(kCheckpointWiggle[0]);

// Touch edge detection to avoid restarting the animation while standing still
static bool checkpointTouchingLastFrame = false;
static int16_t checkpointTouchLastTx = -1;
static int16_t checkpointTouchLastTy = -1;

static void startCheckpointWiggle(int tx, int ty, uint8_t startId, uint8_t endId) {
  gCheckpointAnim.active = true;
  gCheckpointAnim.tileX = tx;
  gCheckpointAnim.tileY = ty;
  gCheckpointAnim.startTileId = startId;
  gCheckpointAnim.endTileId = endId;
  gCheckpointAnim.startMs = gameplayNowMs();
}

// ─────────────────────────────────────────────
// Grow animation (small ↔ big flashing)
// ─────────────────────────────────────────────
bool isGrowing        = false;
int  growStep         = 0;
int  growFrameCounter = 0;
const int GROW_STEPS       = 6;
const int GROW_STEP_FRAMES = 6;

// movement / physics
const float MAX_WALK_SPEED          = 1.0f;
const float MAX_RUN_SPEED           = 1.8f;
const float ACCEL_GROUND            = 0.15f;
const float ACCEL_AIR               = 0.08f;
const float DECEL_GROUND            = 0.20f;
const float DECEL_AIR               = 0.10f;

const float GRAVITY                 = 0.5f;

// Slight boost to compensate for collision/hitbox tuning (restores 4-block jump feel)
const float JUMP_SPEED              = -5.4f;   // was -5.0f
const float SPIN_JUMP_SPEED         = -3.9f;   // was -3.6f
const float FAST_FALL_MULTIPLIER    = 2.0f;
const float SPIN_GRAVITY_MULTIPLIER = 0.45f;
const float MAX_SPIN_FALL_SPEED     = 4.0f;

// camera
float cameraX = 0.0f;
float cameraY = 0.0f;

// Vertical camera mode: when Mario rises high enough, latch into upward follow
// until he lands again (SMW-ish). This prevents the camera from creeping upward
// on small jumps while still following big vertical moves.
static bool gCameraUpFollowLatched = false;

// Height of HUD in pixels (top area that should never be fully blacked out)
const int HUD_HEIGHT_PIXELS = 12;  // adjust if your HUD is taller/shorter

// ─────────────────────────────────────────────
// Poses
// ─────────────────────────────────────────────
enum HeroPose : uint8_t {
  POSE_STAND = 0,
  POSE_WALK1,
  POSE_WALK2,
  POSE_JUMP,
  POSE_SPIN,
  POSE_CROUCH,
  POSE_SLIDE,  // SMW-style slope slide
  POSE_FALL    // Added for completeness
};

HeroPose currentPose = POSE_STAND;

// ─────────────────────────────────────────────
// Forward declarations
// ─────────────────────────────────────────────
TileType getTile(int tx, int ty);
float    computeHeroGroundY(float heroXLocal, float heroTopY, float hitH);
float    computeMushroomGroundY(float mushX, float mushTopY);
float    computeGoombaGroundY(float goombaX, float goombaTopY);
float    computeKoopaGroundY(float koopaX, float koopaTopY);
void     spawnGoombasFromLevel();
void     spawnKoopasFromLevel();
void     updateKoopas();
void     drawGoomba();
void     drawHUDCoinMini(int px, int py);
void     syncTimeFromNTP();
void     startWebServer();
void     startCaptivePortal();
void     showPortalSuccessPage();
void     loseLifeAndReset();
void     spawnMushroomFromBelow(int tx, int ty, uint8_t type = 0);
void     spawnMushroomFromAbove(int tx, int ty, uint8_t type = 0);
void     handleBlockHitFromBelow(int tx, int ty);
void     handleBlockHitFromAbove(int tx, int ty);
void     doHardReset();
void     loadTestMap(bool clearTransitionsAndCutscenes = true);
void     resetWorld(bool clearTransitionsAndCutscenes = true);
void     resetPipeTransitionState();
void     startPipeTransition(uint8_t pairIndex,
                             bool entryIsLeft,
                             int8_t entrySideDir,
                             float entryLockX,
                             float entryLockY,
                             float exitSpawnX,
                             float exitSpawnY,
                             int exitTileX,
                             int exitTileY,
                             uint8_t exitTileId);
void     handleTimeUp();
void     tickMapTimer();
void     tryPickupMushroom();
void     setHeroSize(HeroSize newSize);
void     updateCamera();
void     readInputFromSerial();
float    computeGoombaGroundY(float goombaX, float goombaTopY);
void     spawnGoombasFromLevel();
void     drawGoombas();
void     drawKoopas();
void     checkKoopaHeroCollisions();

void     resetPiranhas();
void     spawnPiranhasFromMobLayer();
void     updatePiranhas();
void     drawPiranhas();
void     checkPiranhaHeroCollisions();
bool     isPiranhaBlockingPipeEntry(int pipeTileX, int pipeTileY);

static void updateCheckpointTouch();
static void updateCheckpointFlipAnimation();
static void drawCheckpointFlipOverlay();

static inline uint8_t getMobId(int tx, int ty);
static inline void    setMobId(int tx, int ty, uint8_t id);
void                 drawOverlayMobs();

static void initFlagPoleRuntimeFromLevel();
static void updateFlagPoleCloth();
static void drawFlagPoleClothOverlay();
static void drawFlagTopOverlay();
static bool findPoleSegmentsInMap(int tx, int topTy, int baseTy, int &outFirstPoleTy, int &outLastPoleTy);

// ─────────────────────────────────────────────
// Flagpole (SMW-ish) state
// ─────────────────────────────────────────────

struct FlagPoleRuntime {
  bool active = false;
  int16_t tileX = -1;
  int16_t topTy = -1;
  int16_t baseTy = -1;

  int16_t firstPoleTy = -1;
  int16_t lastPoleTy = -1;

  float flagWorldY = 0.0f;
  float minFlagWorldY = 0.0f;
  float maxFlagWorldY = 0.0f;

  int8_t flagDir = 1;
  uint32_t lastIdleStepMs = 0;

  bool grabbed = false;
  bool sliding = false;

  bool coinsGiven = false;
  uint8_t awardedCoins = 0;
};

static FlagPoleRuntime gFlagPole;

enum FlagpoleState : uint8_t {
  FLAGPOLE_STATE_IDLE = 0,
  FLAGPOLE_STATE_SLIDE,
  FLAGPOLE_STATE_LAND_HOLD,

  // New level-end sequence
  FLAGPOLE_STATE_JUMP_OFF,
  FLAGPOLE_STATE_RUN_TO_DOOR,
  FLAGPOLE_STATE_WAIT_AT_DOOR,
  FLAGPOLE_STATE_FADE_OUT,
  FLAGPOLE_STATE_FADE_IN,

  // Legacy states (kept for compatibility)
  FLAGPOLE_STATE_WALK_OFF,
  FLAGPOLE_STATE_CLEAR
};

struct FlagpoleCtx {
  FlagpoleState state = FLAGPOLE_STATE_IDLE;
  bool active = false;
  bool freezeEnemies = false;

  int16_t tileX = -1;
  int16_t topTy = -1;
  int16_t baseTy = -1;

  // Pole geometry in world pixels
  float poleX = 0.0f;     // pole left x in world px
  float poleW = 2.0f;

  // Cached slide endpoint
  float slideEndHeroY = 0.0f;

  // Timeline
  uint32_t stateStartMs = 0;

  // ---- Level-end “run to castle” helpers ----
  bool jumpImpulseDone = false;
  bool doorTargetValid = false;
  float doorTargetX = 0.0f;     // pixel X Mario should stop at (door)
  uint32_t waitAtDoorMs = 0;    // timestamp when he stopped at the door
  uint32_t fadeStartMs = 0;     // timestamp when fade begins

};

static FlagpoleCtx gFlagpole;

static inline void resetFlagpoleState() {
  gFlagpole = FlagpoleCtx();
}

// ─────────────────────────────────────────────
// Map1 Input Recorder + Playback (defined here to avoid Arduino compile-order issues)
// ─────────────────────────────────────────────
static inline bool isOverworldMapForDemo(uint8_t mapId) {
  return (mapId == MAP_ID_OVERWORLD_1) || (mapId == MAP_ID_OVERWORLD_2) || (mapId == MAP_ID_OVERWORLD_3);
}

static inline bool isActiveMapRecordable() {
  return isOverworldMapForDemo(gCurrentMapId);
}

static inline uint8_t nextDemoLoopMap(uint8_t currentId) {
  (void)currentId;
  // Unattended attract loop must stay on Map 1 (Map 2/3 are disabled in AUTO_PLAY).
  return MAP_ID_OVERWORLD_1;
}

// ---- Official demo recording storage (NVS / Preferences, chunked) ----
// We store frames as packed bytes: [moveX][buttons] per frame.
// Preferences/NVS has practical per-blob limits, so we chunk into small blobs.
static const uint16_t DEMO_STORE_CHUNK_BYTES = 1024;
static const uint16_t DEMO_STORE_VERSION = 2;
static const uint32_t DEMO_STORE_MAGIC = 0x4D4D4452u; // "MMDR" (MarioMatrix Demo Recording)

// SPIFFS demo file format
static const uint32_t DEMO_FILE_MAGIC = 0x4F4D4544u; // "DEMO" little-endian
static const uint16_t DEMO_FILE_VERSION = 1;

struct DemoFileHeader {
  uint32_t magic;
  uint16_t version;
  uint8_t  mapSlot;      // 1..3
  uint8_t  reserved0;
  uint16_t frameCount;
  uint16_t reserved1;
  uint32_t dataBytes;    // frameCount * sizeof(DemoRecFrame)
  uint32_t crc;          // demoComputeFramesCrc over frames
};

static_assert(sizeof(DemoFileHeader) <= 32, "DemoFileHeader should stay small");

static inline void demoEraseKeyIfPresent(const char *key) {
  // Preferences doesn't expose erase-key reliably across cores; use raw NVS.
  nvs_handle_t h;
  if (nvs_open("demo", NVS_READWRITE, &h) != ESP_OK) return;
  nvs_erase_key(h, key);
  nvs_commit(h);
  nvs_close(h);
}

static inline void demoEraseLegacyChunkKeysForMap(uint8_t mapId) {
  // Old format used numeric chunk suffixes: rX_0, rX_1, ...
  // Erase a generous range to reclaim space.
  char key[16];
  char suffix[12];
  for (uint8_t i = 0; i < 64; ++i) {
    snprintf(suffix, sizeof(suffix), "%u", (unsigned)i);
    demoBuildKey(key, sizeof(key), mapId, suffix);
    demoEraseKeyIfPresent(key);
  }
}

static inline void demoEraseV2FrameKeyForMap(uint8_t mapId) {
  char keyFrm[16];
  demoBuildKey(keyFrm, sizeof(keyFrm), mapId, "frm");
  demoEraseKeyIfPresent(keyFrm);
}

static inline void demoEraseV3ChunkKeysForMap(uint8_t mapId) {
  // v3 format uses rX_frm0, rX_frm1, ... plus metadata keys.
  // We try to read the stored chunk count, otherwise erase a safe max range.
  uint16_t chunks = 0;
  {
    demoPrefs.begin("demo", true);
    char keyChunks[16];
    demoBuildKey(keyChunks, sizeof(keyChunks), mapId, "chunks");
    chunks = demoPrefs.getUShort(keyChunks, 0);
    demoPrefs.end();
  }

  const uint16_t maxChunks = (chunks > 0 && chunks <= 128) ? chunks : 64;

  char key[16];
  char suffix[12];
  for (uint16_t i = 0; i < maxChunks; ++i) {
    snprintf(suffix, sizeof(suffix), "frm%u", (unsigned)i);
    demoBuildKey(key, sizeof(key), mapId, suffix);
    demoEraseKeyIfPresent(key);
  }

  // Metadata keys (shared across v2/v3)
  demoBuildKey(key, sizeof(key), mapId, "len");    demoEraseKeyIfPresent(key);
  demoBuildKey(key, sizeof(key), mapId, "crc");    demoEraseKeyIfPresent(key);
  demoBuildKey(key, sizeof(key), mapId, "chunks"); demoEraseKeyIfPresent(key);
  demoBuildKey(key, sizeof(key), mapId, "ver");    demoEraseKeyIfPresent(key);
  demoBuildKey(key, sizeof(key), mapId, "mag");    demoEraseKeyIfPresent(key);
  demoBuildKey(key, sizeof(key), mapId, "ok");     demoEraseKeyIfPresent(key);
}

static inline uint32_t demoComputeFramesCrc(const DemoRecFrame *frames, uint16_t len) {
  uint32_t h = 2166136261u;
  for (uint16_t i = 0; i < len; ++i) {
    h = demoFnv1aUpdate(h, (uint8_t)frames[i].moveX);
    h = demoFnv1aUpdate(h, (uint8_t)frames[i].buttons);
  }
  return h;
}

static inline uint32_t demoFnv1aUpdate(uint32_t h, uint8_t b) {
  h ^= (uint32_t)b;
  h *= 16777619u;
  return h;
}

static uint32_t demoComputeRecordingCrc(uint16_t len) {
  return demoComputeFramesCrc(gDemoRecBuf, len);
}

static void demoEraseOfficialRecordingForMap(uint8_t mapId) {
  if (!isOverworldMapForDemo(mapId)) return;

  if (gSpiffsMounted) {
    const char *finalPath = demoFilePathForMap(mapId);
    const char *tmpPath   = demoTempPathForMap(mapId);
    if (DEMO_FS.exists(finalPath)) {
      bool ok = DEMO_FS.remove(finalPath);
      Serial.printf("[DEMO] FS delete %s -> %s\n", finalPath, ok ? "OK" : "FAIL");
    }
    if (DEMO_FS.exists(tmpPath)) {
      bool ok = DEMO_FS.remove(tmpPath);
      Serial.printf("[DEMO] FS delete %s -> %s\n", tmpPath, ok ? "OK" : "FAIL");
    }
  }

  // Legacy cleanup (optional): remove old demo keys from NVS.
  demoEraseV2FrameKeyForMap(mapId);
  demoEraseLegacyChunkKeysForMap(mapId);
  demoEraseV3ChunkKeysForMap(mapId);
}

static void demoEraseAllOfficialRecordings() {
  demoEraseOfficialRecordingForMap(MAP_ID_OVERWORLD_1);
  demoEraseOfficialRecordingForMap(MAP_ID_OVERWORLD_2);
  demoEraseOfficialRecordingForMap(MAP_ID_OVERWORLD_3);

  // Clear current in-memory recording/playback state too.
  gDemoHasRecording = false;
  gDemoRecLength = 0;
  gDemoRecWriteIdx = 0;
  gDemoPlaybackActive = false;
  gDemoPlaybackArmed = false;
  gDemoPlaybackPlaying = false;
  gDemoPlaybackArmedMapId = 0xFF;
  gDemoPlaybackStableFrames = 0;
  gDemoPlaybackIdx = 0;
  gAttractLoadedMapId = 0xFF;

  Serial.println("[DEMO] Cleared ALL official recordings (FS + legacy NVS cleanup)");
}

static void demoBuildKey(char *outKey, size_t outKeyLen, uint8_t mapId, const char *suffix) {
  // Keys must be <= 15 chars in Preferences. Keep short.
  // Example: "r1_len", "r2_0", "r3_ver"
  uint8_t slot = (mapId == MAP_ID_OVERWORLD_2) ? 2 : (mapId == MAP_ID_OVERWORLD_3) ? 3 : 1;
  snprintf(outKey, outKeyLen, "r%u_%s", slot, suffix);
}

static bool demoLoadOfficialRecordingForMap_NVS(uint8_t mapId) {
  gDemoHasRecording = false;
  gDemoRecLength = 0;
  gDemoRecWriteIdx = 0;

  if (!isOverworldMapForDemo(mapId)) return false;

  demoPrefs.begin("demo", true);

  // Version gate: ignore recordings from older formats / random stale data.
  char keyVer[16];
  demoBuildKey(keyVer, sizeof(keyVer), mapId, "ver");
  uint16_t ver = demoPrefs.getUShort(keyVer, 0);
  // Back-compat per spec: if ver is missing (0) treat as v2 single-blob.
  if (ver == 0) ver = 2;

  if (ver != 1 && ver != 2 && ver != 3) {
    Serial.print("[DEMO] Load fail (ver) mapId=");
    Serial.print(mapId);
    Serial.print(" ver=");
    Serial.println(ver);
    demoPrefs.end();
    return false;
  }

  // Magic gate: prevents accidental interpretation of garbage as a recording.
  char keyMag[16];
  demoBuildKey(keyMag, sizeof(keyMag), mapId, "mag");
  uint32_t mag = demoPrefs.getUInt(keyMag, 0);
  if (mag != DEMO_STORE_MAGIC) {
    Serial.print("[DEMO] Load fail (mag) mapId=");
    Serial.print(mapId);
    Serial.print(" mag=0x");
    Serial.println(mag, HEX);
    demoPrefs.end();
    return false;
  }

  // Commit marker: only treat recordings as valid if the save completed.
  char keyOk[16];
  demoBuildKey(keyOk, sizeof(keyOk), mapId, "ok");
  uint8_t ok = demoPrefs.getUChar(keyOk, 0);
  if (ok != 1) {
    // Back-compat: v1 recordings created before the atomic commit marker existed.
    // We attempt to load and then rely on CRC to reject partial/corrupt data.
    if (ver == 1) {
      Serial.print("[DEMO] Legacy recording missing commit marker; attempting load mapId=");
      Serial.println(mapId);
    } else {
      Serial.print("[DEMO] Load fail (not committed) mapId=");
      Serial.println(mapId);
      demoPrefs.end();
      return false;
    }
  }

  // Stored checksum (validated after load)
  char keyCrc[16];
  demoBuildKey(keyCrc, sizeof(keyCrc), mapId, "crc");
  const uint32_t expectedCrc = demoPrefs.getUInt(keyCrc, 0);

  char keyLen[16];
  demoBuildKey(keyLen, sizeof(keyLen), mapId, "len");
  uint16_t len = demoPrefs.getUShort(keyLen, 0);

  if (len == 0 || len > DEMO_REC_MAX_FRAMES) {
    Serial.print("[DEMO] Load fail (len) mapId=");
    Serial.print(mapId);
    Serial.print(" len=");
    Serial.println(len);
    demoPrefs.end();
    return false;
  }

  bool loaded = false;
  uint32_t crc = 0;

  if (ver == 2) {
    // v2: single blob containing frames (no chunk keys)
    char keyFrm[16];
    demoBuildKey(keyFrm, sizeof(keyFrm), mapId, "frm");

    const size_t expectedBytes = (size_t)len * sizeof(DemoRecFrame);
    const size_t blobLen = demoPrefs.getBytesLength(keyFrm);
    if (blobLen != expectedBytes) {
      Serial.print("[DEMO] Load fail (frames len) mapId=");
      Serial.print(mapId);
      Serial.print(" got=");
      Serial.print((unsigned)blobLen);
      Serial.print(" expected=");
      Serial.println((unsigned)expectedBytes);
      demoPrefs.end();
      return false;
    }

    size_t got = demoPrefs.getBytes(keyFrm, (void*)gDemoRecBuf, expectedBytes);
    demoPrefs.end();
    if (got != expectedBytes) {
      Serial.print("[DEMO] Load fail (frames read) mapId=");
      Serial.print(mapId);
      Serial.print(" got=");
      Serial.print((unsigned)got);
      Serial.print(" expected=");
      Serial.println((unsigned)expectedBytes);
      return false;
    }

    crc = demoComputeFramesCrc(gDemoRecBuf, len);
    loaded = true;
  } else if (ver == 3) {
    // v3: chunked frame bytes across rX_frm0..N (supports long maps)
    static_assert(sizeof(DemoRecFrame) == 2, "DemoRecFrame must be 2 bytes (moveX, buttons)");

    char keyChunks[16];
    demoBuildKey(keyChunks, sizeof(keyChunks), mapId, "chunks");
    const uint16_t chunkCount = demoPrefs.getUShort(keyChunks, 0);
    if (chunkCount == 0 || chunkCount > 128) {
      Serial.print("[DEMO] Load fail (chunks) mapId=");
      Serial.print(mapId);
      Serial.print(" chunks=");
      Serial.println(chunkCount);
      demoPrefs.end();
      return false;
    }

    // Keep chunk size conservative under typical NVS per-entry blob limits.
    // (Exact maximum varies by core/partition; 1800 is a safe default.)
    static const size_t DEMO_STORE_V3_CHUNK_BYTES = 1800;
    const size_t expectedBytes = (size_t)len * sizeof(DemoRecFrame);

    size_t offset = 0;
    for (uint16_t ci = 0; ci < chunkCount && offset < expectedBytes; ++ci) {
      char keyChunk[16];
      char suffix[12];
      snprintf(suffix, sizeof(suffix), "frm%u", (unsigned)ci);
      demoBuildKey(keyChunk, sizeof(keyChunk), mapId, suffix);

      const size_t want = (expectedBytes - offset) > DEMO_STORE_V3_CHUNK_BYTES
                            ? DEMO_STORE_V3_CHUNK_BYTES
                            : (expectedBytes - offset);

      size_t got = demoPrefs.getBytes(keyChunk, (void*)(((uint8_t*)gDemoRecBuf) + offset), want);
      if (got != want) {
        Serial.print("[DEMO] Load fail (chunk read) mapId=");
        Serial.print(mapId);
        Serial.print(" chunk=");
        Serial.print((unsigned)ci);
        Serial.print(" key=");
        Serial.print(keyChunk);
        Serial.print(" got=");
        Serial.print((unsigned)got);
        Serial.print(" expected=");
        Serial.println((unsigned)want);
        demoPrefs.end();
        return false;
      }

      offset += got;
    }

    demoPrefs.end();

    if (offset != expectedBytes) {
      Serial.print("[DEMO] Load fail (reconstruct bytes) mapId=");
      Serial.print(mapId);
      Serial.print(" got=");
      Serial.print((unsigned)offset);
      Serial.print(" expected=");
      Serial.println((unsigned)expectedBytes);
      return false;
    }

    crc = demoComputeFramesCrc(gDemoRecBuf, len);
    loaded = true;
  } else {
    // v1 legacy: chunked packed bytes
    const uint32_t totalBytes = (uint32_t)len * 2u;
    uint32_t offset = 0;
    uint16_t frameIdx = 0;
    uint16_t chunkIdx = 0;

    crc = 2166136261u;
    static uint8_t tmp[DEMO_STORE_CHUNK_BYTES];

    while (offset < totalBytes) {
      char keyChunk[16];
      char suffix[12];
      snprintf(suffix, sizeof(suffix), "%u", (unsigned)chunkIdx);
      demoBuildKey(keyChunk, sizeof(keyChunk), mapId, suffix);

      size_t got = demoPrefs.getBytes(keyChunk, tmp, DEMO_STORE_CHUNK_BYTES);
      if (got == 0) {
        Serial.print("[DEMO] Load fail (missing chunk) mapId=");
        Serial.print(mapId);
        Serial.print(" chunk=");
        Serial.println(chunkIdx);
        demoPrefs.end();
        gDemoRecLength = 0;
        gDemoRecWriteIdx = 0;
        return false;
      }

      for (size_t i = 0; i + 1 < got && offset < totalBytes; i += 2) {
        DemoRecFrame f;
        f.moveX = (int8_t)tmp[i + 0];
        f.buttons = tmp[i + 1];
        if (frameIdx < DEMO_REC_MAX_FRAMES) {
          gDemoRecBuf[frameIdx] = f;
        }

        crc = demoFnv1aUpdate(crc, (uint8_t)f.moveX);
        crc = demoFnv1aUpdate(crc, (uint8_t)f.buttons);

        frameIdx++;
        offset += 2;
      }

      chunkIdx++;
    }

    demoPrefs.end();

    if (frameIdx != len) {
      gDemoRecLength = 0;
      gDemoRecWriteIdx = 0;
      return false;
    }

    loaded = true;
  }

  if (!loaded) {
    gDemoRecLength = 0;
    gDemoRecWriteIdx = 0;
    return false;
  }

  if (expectedCrc == 0 || crc != expectedCrc) {
    Serial.print("[DEMO] Recording CRC mismatch for mapId=");
    Serial.println(mapId);
    Serial.print("[DEMO] expected=0x");
    Serial.print(expectedCrc, HEX);
    Serial.print(" got=0x");
    Serial.println(crc, HEX);
    gDemoRecLength = 0;
    gDemoRecWriteIdx = 0;
    return false;
  }

  gDemoRecLength = len;
  gDemoRecWriteIdx = len;
  gDemoHasRecording = true;
  return true;
}

static bool demoLoadOfficialRecordingForMap_SPIFFS(uint8_t mapId, bool &outFileFound) {
  outFileFound = false;
  gDemoHasRecording = false;
  gDemoRecLength = 0;
  gDemoRecWriteIdx = 0;

  if (!isOverworldMapForDemo(mapId)) return false;

  if (!gSpiffsMounted) return false;

  const char *path = demoFilePathForMap(mapId);
  if (!DEMO_FS.exists(path)) {
    return false;
  }
  outFileFound = true;

  File f = DEMO_FS.open(path, FILE_READ);
  if (!f) {
    Serial.printf("[DEMO] FS load fail (open) mapId=%u path=%s\n", (unsigned)mapId, path);
    return false;
  }

  DemoFileHeader hdr;
  const size_t hdrGot = f.readBytes((char*)&hdr, sizeof(hdr));
  if (hdrGot != sizeof(hdr)) {
    Serial.printf("[DEMO] FS load fail (hdr read) mapId=%u got=%u expected=%u\n",
                  (unsigned)mapId, (unsigned)hdrGot, (unsigned)sizeof(hdr));
    f.close();
    return false;
  }

  if (hdr.magic != DEMO_FILE_MAGIC || hdr.version != DEMO_FILE_VERSION) {
    Serial.printf("[DEMO] FS load fail (hdr magic/ver) mapId=%u magic=0x%08X ver=%u\n",
                  (unsigned)mapId, (unsigned)hdr.magic, (unsigned)hdr.version);
    f.close();
    return false;
  }

  const uint8_t expectedSlot = demoSlotFromMapId(mapId);
  if (hdr.mapSlot != expectedSlot) {
    Serial.printf("[DEMO] FS load fail (hdr mapSlot) mapId=%u slot=%u expected=%u\n",
                  (unsigned)mapId, (unsigned)hdr.mapSlot, (unsigned)expectedSlot);
    f.close();
    return false;
  }

  if (hdr.frameCount == 0 || hdr.frameCount > DEMO_REC_MAX_FRAMES) {
    Serial.printf("[DEMO] FS load fail (hdr frameCount) mapId=%u frames=%u\n",
                  (unsigned)mapId, (unsigned)hdr.frameCount);
    f.close();
    return false;
  }

  const size_t expectedBytes = (size_t)hdr.frameCount * sizeof(DemoRecFrame);
  if (hdr.dataBytes != expectedBytes) {
    Serial.printf("[DEMO] FS load fail (hdr dataBytes) mapId=%u dataBytes=%u expected=%u\n",
                  (unsigned)mapId, (unsigned)hdr.dataBytes, (unsigned)expectedBytes);
    f.close();
    return false;
  }

  // Read frames
  const size_t got = f.readBytes((char*)gDemoRecBuf, expectedBytes);
  f.close();
  if (got != expectedBytes) {
    Serial.printf("[DEMO] FS load fail (frames read) mapId=%u got=%u expected=%u\n",
                  (unsigned)mapId, (unsigned)got, (unsigned)expectedBytes);
    return false;
  }

  const uint32_t crc = demoComputeFramesCrc(gDemoRecBuf, hdr.frameCount);
  if (hdr.crc != 0 && crc != hdr.crc) {
    Serial.printf("[DEMO] FS load fail (CRC) mapId=%u expected=0x%08X got=0x%08X\n",
                  (unsigned)mapId, (unsigned)hdr.crc, (unsigned)crc);
    return false;
  }

  gDemoRecLength = hdr.frameCount;
  gDemoRecWriteIdx = hdr.frameCount;
  gDemoHasRecording = true;
  Serial.printf("[DEMO] Loaded SPIFFS demo mapId=%u frames=%u bytes=%u\n",
                (unsigned)mapId, (unsigned)hdr.frameCount, (unsigned)expectedBytes);
  return true;
}

static bool demoLoadOfficialRecordingForMap(uint8_t mapId) {
  // Spec: Load from SPIFFS first; only fall back to legacy NVS if SPIFFS isn't mounted.
  if (!gSpiffsMounted) {
    return demoLoadOfficialRecordingForMap_NVS(mapId);
  }

  bool fileFound = false;
  return demoLoadOfficialRecordingForMap_SPIFFS(mapId, fileFound);
}

static void demoSaveOfficialRecordingForMap(uint8_t mapId) {
  if (!isOverworldMapForDemo(mapId)) return;
  if (gDemoRecLength < DEMO_REC_MIN_FRAMES) return;

  static_assert(sizeof(DemoRecFrame) == 2, "DemoRecFrame must be 2 bytes (moveX, buttons)");

  if (!gSpiffsMounted) {
    if (!gSpiffsSaveNotMountedWarned) {
      gSpiffsSaveNotMountedWarned = true;
      Serial.println("[REC] Save failed: SPIFFS not mounted.");
    }
    // Consistent behavior: if SPIFFS isn't mounted, skip file save.
    return;
  }

  const size_t framesBytes = (size_t)gDemoRecLength * sizeof(DemoRecFrame);
  const size_t bytesNeeded = sizeof(DemoFileHeader) + framesBytes;
  const unsigned totalB = (unsigned)DEMO_FS.totalBytes();
  const unsigned usedB  = (unsigned)DEMO_FS.usedBytes();
  const unsigned freeB  = (totalB >= usedB) ? (totalB - usedB) : 0;

  Serial.printf("[REC] FS space total=%u used=%u free=%u needed=%u\n",
                totalB, usedB, freeB, (unsigned)bytesNeeded);

  const char *finalPath = demoFilePathForMap(mapId);
  const char *tmpPath   = demoTempPathForMap(mapId);

  // Clean up any stale temp file first.
  if (DEMO_FS.exists(tmpPath)) {
    DEMO_FS.remove(tmpPath);
  }

  File f = DEMO_FS.open(tmpPath, FILE_WRITE);
  if (!f) {
    Serial.printf("[REC] FS save failed (open tmp) mapId=%u path=%s\n", (unsigned)mapId, tmpPath);
    Serial.println("[REC] Erase other map recordings or use a larger SPIFFS partition.");
    return;
  }

  DemoFileHeader hdr;
  hdr.magic = DEMO_FILE_MAGIC;
  hdr.version = DEMO_FILE_VERSION;
  hdr.mapSlot = demoSlotFromMapId(mapId);
  hdr.reserved0 = 0;
  hdr.frameCount = gDemoRecLength;
  hdr.reserved1 = 0;
  hdr.dataBytes = (uint32_t)framesBytes;
  hdr.crc = demoComputeRecordingCrc(gDemoRecLength);

  const size_t wroteHdr = f.write((const uint8_t*)&hdr, sizeof(hdr));
  if (wroteHdr != sizeof(hdr)) {
    Serial.printf("[REC] FS save failed (hdr write) mapId=%u wrote=%u expected=%u\n",
                  (unsigned)mapId, (unsigned)wroteHdr, (unsigned)sizeof(hdr));
    f.close();
    DEMO_FS.remove(tmpPath);
    Serial.println("[REC] Erase other map recordings or use a larger SPIFFS partition.");
    return;
  }

  const size_t wroteFrames = f.write((const uint8_t*)gDemoRecBuf, framesBytes);
  if (wroteFrames != framesBytes) {
    Serial.printf("[REC] FS save failed (frames write) mapId=%u wrote=%u expected=%u\n",
                  (unsigned)mapId, (unsigned)wroteFrames, (unsigned)framesBytes);
    f.close();
    DEMO_FS.remove(tmpPath);
    Serial.println("[REC] Erase other map recordings or use a larger SPIFFS partition.");
    return;
  }

  f.flush();
  f.close();

  // Atomic replace: remove old final, then rename tmp -> final.
  if (DEMO_FS.exists(finalPath)) {
    DEMO_FS.remove(finalPath);
  }
  if (!DEMO_FS.rename(tmpPath, finalPath)) {
    Serial.printf("[REC] FS save failed (rename) mapId=%u tmp=%s final=%s\n",
                  (unsigned)mapId, tmpPath, finalPath);
    Serial.println("[REC] Erase other map recordings or use a larger SPIFFS partition.");
    return;
  }

  Serial.printf("[REC] Save backend=SPIFFS path=%s bytes=%u OK\n",
                finalPath, (unsigned)bytesNeeded);
}

static inline uint8_t packButtonsFromInput(const InputState &in) {
  uint8_t b = 0;
  if (in.btnJump)  b |= 0x01;
  if (in.btnSpin)  b |= 0x02;
  if (in.btnRun)   b |= 0x04;
  if (in.btnDown)  b |= 0x08;
  if (in.btnReset) b |= 0x10;
  if (in.btnStart) b |= 0x20;
  if (in.btnUp)    b |= 0x40;
  return b;
}

static inline void unpackButtonsToInput(uint8_t b, InputState &out) {
  out.btnJump  = b & 0x01;
  out.btnSpin  = b & 0x02;
  out.btnRun   = b & 0x04;
  out.btnDown  = b & 0x08;
  out.btnReset = b & 0x10;
  out.btnStart = b & 0x20;
  out.btnUp    = b & 0x40;
}

static void demoDeterminismDebugPrint(uint32_t nowMs) {
#if DEBUG_DEMO_DETERMINISM
  if (!demoTimingActive()) return;

  static uint32_t tick = 0;
  tick++;
  if ((tick % 30) != 0) return;

  int goombas = 0;
  for (int i = 0; i < MAX_GOOMBAS; ++i) {
    if (gGoombas[i].active) goombas++;
  }

  int koopas = 0;
  for (int i = 0; i < MAX_KOOPAS; ++i) {
    if (gKoopas[i].active) koopas++;
  }

  int piranhas = 0;
  for (int i = 0; i < MAX_PIRANHAS; ++i) {
    if (gPiranhas[i].active) piranhas++;
  }

  Serial.print("[DEMO] t=");
  Serial.print(nowMs);
  Serial.print(" map=");
  Serial.print(gCurrentMapId);
  Serial.print(" mode=");
  Serial.print((int)gControlMode);
  Serial.print(" play=");
  Serial.print(gDemoPlaybackActive ? 1 : 0);
  Serial.print(" idx=");
  Serial.print(gDemoPlaybackIdx);
  Serial.print("/");
  Serial.print(gDemoRecLength);
  Serial.print(" rec=");
  Serial.print(gDemoIsRecording ? 1 : 0);
  Serial.print(" hero=(");
  Serial.print(heroX, 2);
  Serial.print(",");
  Serial.print(heroY, 2);
  Serial.print(") mobs g/k/p=");
  Serial.print(goombas);
  Serial.print("/");
  Serial.print(koopas);
  Serial.print("/");
  Serial.println(piranhas);
#else
  (void)nowMs;
#endif
}

static void demoRecorderStart() {
  // Don’t allow recording while playing back.
  gDemoPlaybackActive = false;
  gDemoPlaybackArmed = false;
  gDemoPlaybackPlaying = false;
  gDemoPlaybackArmedMapId = 0xFF;
  gDemoPlaybackStableFrames = 0;
  gDemoPlaybackIdx = 0;

  gDemoRecWriteIdx = 0;
  gDemoRecLength = 0;
  gDemoHasRecording = false;

  // Start deterministic demo time at 0 so time-based enemies (e.g. piranhas) align
  // between record and playback.
  demoSimReset();

  // Mark recording ON before resetWorld so any millis()-based gameplay init that we
  // route through gameplayNowMs() uses the demo timebase.
  gDemoIsRecording = true;

  // Critical: reset the world RIGHT as recording begins so mobs/timers start at a known
  // baseline aligned with the first recorded frame.
  if (isActiveMapRecordable() && gGameMode == GAME_MODE_PLAYING) {
    Serial.println("[REC] Resetting world for fresh recording...");
    resetWorld(true);
  }
  Serial.println("[REC] Recording ON");
}

static void demoRecorderStop(const char *reason) {
  (void)reason;
  gDemoIsRecording = false;
  gDemoHasRecording = (gDemoRecLength >= DEMO_REC_MIN_FRAMES);
  Serial.print("[REC] Recording OFF len=");
  Serial.print(gDemoRecLength);
  Serial.print(" hasRecording=");
  Serial.println(gDemoHasRecording ? 1 : 0);

  // Persist "official" recordings for Map 1–3.
  if (gDemoHasRecording && isActiveMapRecordable()) {
    demoSaveOfficialRecordingForMap(gCurrentMapId);
  }
}

static void demoRecorderToggle() {
  if (!gDemoIsRecording) demoRecorderStart();
  else demoRecorderStop("toggle");
}

static void demoPlaybackStart() {
  if (!gDemoHasRecording || gDemoRecLength == 0) {
    Serial.println("[PLAY] No recording to play");
    return;
  }
  if (!isActiveMapRecordable()) {
    Serial.println("[PLAY] Playback only enabled on Map1-3");
    return;
  }
  if (gGameMode != GAME_MODE_PLAYING) {
    Serial.println("[PLAY] Playback only allowed during gameplay");
    return;
  }
  if (gFlagpole.active || gPipeTransition.active || gDeath.active) {
    Serial.println("[PLAY] Playback blocked during cutscene/transition");
    return;
  }
  // Ensure we aren't recording while starting playback.
  if (gDemoIsRecording) {
    demoRecorderStop("playback start");
  }

  // Start deterministic demo time at 0 so time-based enemies align with the
  // recording from the very first frame.
  demoSimReset();

  // Enable playback BEFORE resetWorld so any gameplay init routed through
  // gameplayNowMs() uses the deterministic demo timebase.
  gDemoPlaybackIdx = 0;
  gDemoPlaybackActive = true;
  gDemoPlaybackArmed = false;
  gDemoPlaybackPlaying = true;
  gDemoPlaybackArmedMapId = 0xFF;
  gDemoPlaybackStableFrames = 0;

  // Critical: reset the world RIGHT as playback begins so mobs/timers start at a known
  // baseline aligned with the first recorded frame.
  Serial.println("[PLAY] Resetting world for fresh playback...");
  resetWorld(true);

  // Ensure auto-play won't fight playback.
  setControlMode(PLAYER);
  // Ignore "human input cancels playback" briefly so the toggle chord can be released.
  gDemoPlaybackIgnoreCancelUntilMs = millis() + 250;
  Serial.println("[PLAY] Playback ON");
}

static void demoPlaybackStop(const char *reason) {
  if (!gDemoPlaybackActive) return;
  gDemoPlaybackActive = false;
  gDemoPlaybackArmed = false;
  gDemoPlaybackPlaying = false;
  gDemoPlaybackArmedMapId = 0xFF;
  gDemoPlaybackStableFrames = 0;
  Serial.print("[PLAY] Playback OFF reason=");
  Serial.println(reason ? reason : "(null)");
}

static void demoPlaybackToggle() {
  if (gDemoPlaybackActive) demoPlaybackStop("toggle");
  else demoPlaybackStart();
}

static inline bool isPlayerControlActiveForDemoPlayback() {
  if (gGameMode != GAME_MODE_PLAYING) return false;
  if (attractIntroActive()) return false;
  if (gFlagpole.active || gPipeTransition.active || gDeath.active) return false;
  // If a fade overlay is active, consider control not stable yet.
  if (gPipeFadeAmount != 0) return false;
  return true;
}

static void tickDemoPlaybackArming(uint32_t nowMs) {
  (void)nowMs;
  if (!gDemoPlaybackActive) {
    gDemoPlaybackArmed = false;
    gDemoPlaybackPlaying = false;
    gDemoPlaybackArmedMapId = 0xFF;
    gDemoPlaybackStableFrames = 0;

    gDemoTraceRemaining = 0;
    gDemoTraceMapId = 0xFF;
    gDemoTraceHaveLastFrame = false;
  #if DEBUG_DEMO_MAP2_TRACE
    demoTraceReset();
  #endif
    return;
  }

  if (!gDemoPlaybackArmed || gDemoPlaybackPlaying) return;

  // If we change maps while armed, drop the arm and let the attract loader re-arm.
  if (gDemoPlaybackArmedMapId != 0xFF && gCurrentMapId != gDemoPlaybackArmedMapId) {
    gDemoPlaybackArmed = false;
    gDemoPlaybackStableFrames = 0;

    gDemoTraceRemaining = 0;
    gDemoTraceMapId = 0xFF;
    gDemoTraceHaveLastFrame = false;
  #if DEBUG_DEMO_MAP2_TRACE
    demoTraceReset();
  #endif
    return;
  }

  if (!isPlayerControlActiveForDemoPlayback()) {
    gDemoPlaybackStableFrames = 0;
    return;
  }

  if (gDemoPlaybackStableFrames < 255) gDemoPlaybackStableFrames++;
  if (gDemoPlaybackStableFrames >= DEMO_PLAYBACK_STABLE_CONTROL_FRAMES) {
    // Official demo playback can be armed while a fade/cutscene runs.
    // Even though we pause input consumption, the deterministic demo clock still
    // advances every tick in AUTO_PLAY. That means the map can start at t≈fade length,
    // which breaks recordings that assume t=0 at the first playable frame.
    // Fix: re-zero the demo timebase right when playback actually begins.
    if (gControlMode == AUTO_PLAY) {
      demoSimReset();
      gLastMapTimeTickMs = gameplayNowMs();
      Serial.print("[DEMO] demoSimReset (playback start) mapId=");
      Serial.println(gCurrentMapId);
    }

    // Avoid spurious edge-detected actions on the first frame.
    // Prime prev* to match frame0 so "justPressed" only occurs when the
    // recording actually transitions from 0->1 on a later frame.
    gHumanInputThisFrame = false;
    primeInputEdgeDetectionFromPlaybackFrame(gDemoPlaybackIdx);

    gDemoPlaybackPlaying = true;
    gDemoPlaybackArmed = false;
    gDemoPlaybackStableFrames = 0;

    // Enable short trace for Map2 to debug early-frame desync.
    if (gCurrentMapId == 2) {
      gDemoTraceRemaining = 120; // ~2 seconds @60Hz
      gDemoTraceMapId = gCurrentMapId;
      gDemoTraceHaveLastFrame = false;
#if DEBUG_DEMO_MAP2_TRACE
      demoTraceReset();
#endif
    } else {
      gDemoTraceRemaining = 0;
      gDemoTraceMapId = 0xFF;
      gDemoTraceHaveLastFrame = false;
#if DEBUG_DEMO_MAP2_TRACE
      demoTraceReset();
#endif
    }

    // Ignore brief human input right as playback begins (chords / controller noise).
    gDemoPlaybackIgnoreCancelUntilMs = millis() + 250;

    Serial.print("[DEMO] Playback START mapId=");
    Serial.print(gCurrentMapId);
    Serial.println(" after stable control");
  }
}

static void tickDemoPlaybackChordAndSuppress(uint32_t nowMs) {
  // Only active in gameplay and only on Map1 for this test.
  if (gGameMode != GAME_MODE_PLAYING || !isActiveMapRecordable()) {
    gDemoPlayChordStartMs = 0;
    gDemoPlayChordTriggeredThisHold = false;
    return;
  }

  if (gDemoPlayChordCooldownUntilMs != 0 && nowMs < gDemoPlayChordCooldownUntilMs) {
    // Still cooling down; prevent chord buttons from causing actions.
    gInput.btnReset = 0;
    gInput.btnDown = 0;
    gInput.btnStart = 0;
    return;
  }

  // Chord: SELECT + DOWN held.
  const bool chordHeld =
    (gInput.btnReset != 0) &&
    (gInput.btnDown  != 0);

  if (chordHeld) {
    // This frame should not be treated as "human input cancels playback".
    // (The chord itself is human input, but we need it to successfully start playback.)
    gDemoSkipRecordThisFrame = true;

    // Suppress so it doesn't reset the world or modify gameplay.
    gInput.btnReset = 0;
    gInput.btnDown = 0;
    gInput.btnStart = 0;

    if (gDemoPlayChordStartMs == 0) {
      gDemoPlayChordStartMs = nowMs;
    }

    if (!gDemoPlayChordTriggeredThisHold && (nowMs - gDemoPlayChordStartMs) >= DEMO_PLAY_HOLD_MS) {
      demoPlaybackToggle();
      gDemoPlayChordTriggeredThisHold = true;
      gDemoPlayChordCooldownUntilMs = nowMs + DEMO_PLAY_COOLDOWN_MS;
      // Prevent recording/playback from capturing the toggle frame as gameplay input.
      gDemoSkipRecordThisFrame = true;
    }
  } else {
    gDemoPlayChordStartMs = 0;
    gDemoPlayChordTriggeredThisHold = false;
  }
}

static void tickDemoEraseChordAndSuppress(uint32_t nowMs) {
  // Only active in gameplay.
  if (gGameMode != GAME_MODE_PLAYING) {
    gDemoEraseChordStartMs = 0;
    gDemoEraseChordTriggeredThisHold = false;
    return;
  }

  if (gDemoEraseChordCooldownUntilMs != 0 && nowMs < gDemoEraseChordCooldownUntilMs) {
    // Still cooling down; prevent chord buttons from causing actions.
    gInput.btnReset = 0;
    gInput.btnUp = 0;
    gInput.btnStart = 0;
    gInput.rawMoveX = 0;
    gInput.moveAxis = 0.0f;
    return;
  }

  // Chord: SELECT + UP + RIGHT held.
  // (Avoids SELECT+LEFT recorder and SELECT+DOWN playback.)
  const bool chordHeld =
    (gInput.btnReset != 0) &&
    (gInput.btnUp    != 0) &&
    (gInput.btnDown  == 0) &&
    (gInput.rawMoveX >= 96);

  if (chordHeld) {
    gDemoSkipRecordThisFrame = true;

    // Suppress so it doesn't reset/pause or alter gameplay.
    gInput.btnReset = 0;
    gInput.btnUp = 0;
    gInput.btnStart = 0;
    gInput.rawMoveX = 0;
    gInput.moveAxis = 0.0f;

    if (gDemoEraseChordStartMs == 0) {
      gDemoEraseChordStartMs = nowMs;
    }

    if (!gDemoEraseChordTriggeredThisHold && (nowMs - gDemoEraseChordStartMs) >= DEMO_ERASE_HOLD_MS) {
      demoEraseAllOfficialRecordings();
      gDemoEraseChordTriggeredThisHold = true;
      gDemoEraseChordCooldownUntilMs = nowMs + DEMO_ERASE_COOLDOWN_MS;
      gDemoSkipRecordThisFrame = true;
    }
  } else {
    gDemoEraseChordStartMs = 0;
    gDemoEraseChordTriggeredThisHold = false;
  }
}

static void applyDemoPlaybackInputOrStop() {
  if (!gDemoPlaybackActive) return;

  // If we're armed (loaded) but not yet playing, do not consume frames.
  if (!gDemoPlaybackPlaying) return;

  // Do not advance demo frames during cutscenes/transitions.
  // This keeps recordings aligned and avoids inputs interfering with scripted sequences.
  if (gFlagpole.active || gPipeTransition.active || gDeath.active) {
    // IMPORTANT: do not force inputs to 0 here.
    // Edge-detected actions (e.g., pipe entry on justPressedDown) depend on the previous-frame
    // state. Zeroing during a transition can make the first post-transition frame look like a
    // fresh press and cause immediate re-entry/teleport and demo desync.
    return;
  }

  const uint32_t nowMs = millis();

  // Any real human input cancels playback immediately.
  // Ignore the toggle-chord frame(s) so playback can actually start.
  if (gHumanInputThisFrame && !gDemoSkipRecordThisFrame && nowMs >= gDemoPlaybackIgnoreCancelUntilMs) {
    demoPlaybackStop("human input");
    setControlMode(PLAYER);
    return;
  }

  // If we are not in gameplay anymore, stop.
  if (gGameMode != GAME_MODE_PLAYING) {
    demoPlaybackStop("not playing");
    return;
  }

  // Stop if we leave the overworld demo maps.
  if (!isActiveMapRecordable()) {
    demoPlaybackStop("left demo maps");
    return;
  }

  if (gDemoPlaybackIdx >= gDemoRecLength) {
    demoPlaybackStop("end");
    return;
  }

  const uint16_t appliedIdx = gDemoPlaybackIdx;
  const DemoRecFrame &f = gDemoRecBuf[gDemoPlaybackIdx++];
  gInput.rawMoveX = f.moveX;
  float axis = (float)f.moveX / 127.0f;
  if (axis >  1.0f) axis =  1.0f;
  if (axis < -1.0f) axis = -1.0f;
  gInput.moveAxis = axis;
  unpackButtonsToInput(f.buttons, gInput);

  // Save last applied frame for debug tracing.
  if (gDemoTraceRemaining > 0 && gDemoTraceMapId == gCurrentMapId) {
    gDemoTraceHaveLastFrame = true;
    gDemoTraceLastFrameIdx = appliedIdx;
    gDemoTraceLastMoveX = f.moveX;
    gDemoTraceLastButtons = f.buttons;
  }
}

static void tickDemoRecorderChordAndSuppress(uint32_t nowMs) {
  gDemoSkipRecordThisFrame = false;

  // Only active in gameplay and only on Map1-3 for this build.
  if (gGameMode != GAME_MODE_PLAYING || !isActiveMapRecordable()) {
    gDemoChordStartMs = 0;
    gDemoChordTriggeredThisHold = false;
    return;
  }

  if (gDemoChordCooldownUntilMs != 0 && nowMs < gDemoChordCooldownUntilMs) {
    // Still cooling down; also prevent chord buttons from causing actions.
    gDemoSkipRecordThisFrame = true;
    gInput.btnStart = 0;
    gInput.btnReset = 0;
    gInput.rawMoveX = 0;
    gInput.moveAxis = 0.0f;
    return;
  }

  // Chord: SELECT + LEFT held.
  // Require DOWN not held to avoid conflicting with SELECT+DOWN playback.
  const bool chordHeld =
    (gInput.btnReset != 0) &&
    (gInput.btnDown  == 0) &&
    (gInput.rawMoveX <= -96);

  if (chordHeld) {
    // While the chord is held we suppress inputs; do NOT record these neutral frames.
    gDemoSkipRecordThisFrame = true;

    // Suppress so it doesn't pause/reset or alter gameplay.
    gInput.btnStart = 0;
    gInput.btnReset = 0;
    gInput.rawMoveX = 0;
    gInput.moveAxis = 0.0f;

    if (gDemoChordStartMs == 0) {
      gDemoChordStartMs = nowMs;
    }

    if (!gDemoChordTriggeredThisHold && (nowMs - gDemoChordStartMs) >= DEMO_REC_HOLD_MS) {
      demoRecorderToggle();
      gDemoChordTriggeredThisHold = true;
      gDemoChordCooldownUntilMs = nowMs + DEMO_REC_COOLDOWN_MS;
      gDemoSkipRecordThisFrame = true; // don't record the toggle chord as gameplay input
    }
  } else {
    gDemoChordStartMs = 0;
    gDemoChordTriggeredThisHold = false;
  }
}

static void tickDemoRecorderPerGameplayTick(uint32_t nowMs) {
  (void)nowMs;
  if (!gDemoIsRecording) return;

  // Never record while playing back.
  if (gDemoPlaybackActive) return;

  // Stop if we leave Map1-3 (e.g., map change) or if the flagpole sequence starts.
  if (!isActiveMapRecordable()) {
    demoRecorderStop("left demo maps");
    return;
  }

  if (gFlagpole.active) {
    demoRecorderStop("flagpole");
    return;
  }

  if (gPipeTransition.active) {
    // Tunnels/pipes are scripted transitions. Don't stop the recording; just pause
    // recording frames until gameplay resumes.
    return;
  }

  if (gDeath.active) {
    demoRecorderStop("death");
    return;
  }

  if (gDemoSkipRecordThisFrame) return;

  if (gDemoRecWriteIdx >= DEMO_REC_MAX_FRAMES) {
    demoRecorderStop("buffer full");
    return;
  }

  DemoRecFrame f;
  f.moveX = gInput.rawMoveX;
  f.buttons = packButtonsFromInput(gInput);
  gDemoRecBuf[gDemoRecWriteIdx++] = f;
  gDemoRecLength = gDemoRecWriteIdx;
}

static inline bool isFlagpoleTileId(uint8_t id) {
  return (id == TILE_ID_FLAG_TOP || id == TILE_ID_FLAG_POLE || id == TILE_ID_FLAG_BASE);
}

// Helper used by flagpole end-cutscene to locate the castle tile anchor.
static bool findTileTopLeft(uint8_t tileId, int16_t &outTx, int16_t &outTy);

static inline bool rectOverlap(float ax1, float ay1, float ax2, float ay2,
                               float bx1, float by1, float bx2, float by2) {
  return (ax1 < bx2 && ax2 > bx1 && ay1 < by2 && ay2 > by1);
}

static bool findFlagpoleColumn(int tx, int ty, int &outTopTy, int &outBaseTy) {
  if (tx < 0 || tx >= LEVEL_WIDTH || ty < 0 || ty >= LEVEL_HEIGHT) return false;

  // Scan up for top tile
  int topTy = -1;
  for (int y = ty; y >= 0; --y) {
    uint8_t id = getTileId(tx, y);
    if (id == TILE_ID_FLAG_TOP) { topTy = y; break; }
    if (id != TILE_ID_EMPTY && !isFlagpoleTileId(id)) break;
  }

  // Scan down for base tile
  int baseTy = -1;
  for (int y = ty; y < LEVEL_HEIGHT; ++y) {
    uint8_t id = getTileId(tx, y);
    if (id == TILE_ID_FLAG_BASE) { baseTy = y; break; }
    if (id != TILE_ID_EMPTY && !isFlagpoleTileId(id)) break;
  }

  // If not found, fall back to the hit tile row.
  if (topTy < 0)  topTy  = ty;
  if (baseTy < 0) baseTy = ty;

  outTopTy = topTy;
  outBaseTy = baseTy;
  return true;
}

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static uint8_t computeFlagPoleCoinsFromClothY(float clothY) {
  if (!gFlagPole.active) return 0;
  float denom = (gFlagPole.maxFlagWorldY - gFlagPole.minFlagWorldY);
  if (denom < 1.0f) denom = 1.0f;
  float t = (gFlagPole.maxFlagWorldY - clothY) / denom; // 1=top, 0=bottom
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  // Tiers (SMW-ish feel)
  if      (t > 0.85f) return 50;
  else if (t > 0.65f) return 40;
  else if (t > 0.45f) return 30;
  else if (t > 0.25f) return 20;
  else                return 10;
}

static bool tryStartFlagpoleGrab() {
  if (gFlagpole.active) return false;
  if (gPipeTransition.active) return false;
  if (onGround) return false;
  if (heroVY < 0.0f) return false; // only grab while falling / descending
  if (isGrowing) return false;

  float hx1 = heroX;
  float hy1 = heroY;
  float hx2 = heroX + heroW;
  float hy2 = heroY + heroHitH;

  int txStart = (int)(hx1 / TILE_SIZE) - 1;
  int txEnd   = (int)((hx2 - 1) / TILE_SIZE) + 1;
  int tyStart = (int)(hy1 / TILE_SIZE) - 1;
  int tyEnd   = (int)((hy2 - 1) / TILE_SIZE) + 1;

  if (txStart < 0) txStart = 0;
  if (tyStart < 0) tyStart = 0;
  if (txEnd >= LEVEL_WIDTH)  txEnd = LEVEL_WIDTH - 1;
  if (tyEnd >= LEVEL_HEIGHT) tyEnd = LEVEL_HEIGHT - 1;

  for (int ty = tyStart; ty <= tyEnd; ++ty) {
    for (int tx = txStart; tx <= txEnd; ++tx) {
      uint8_t id = getTileId(tx, ty);
      if (id != TILE_ID_FLAG_POLE && id != TILE_ID_FLAG_BASE) continue;

      // Pole collision is narrow (2px) and uses the same draw offset.
      float px1, py1, px2, py2;
      if (id == TILE_ID_FLAG_POLE) {
        px1 = (float)(tx * TILE_SIZE + 3);
        py1 = (float)(ty * TILE_SIZE);
        px2 = px1 + 2.0f;
        py2 = py1 + 8.0f;
      } else {
        // Base: full tile
        px1 = (float)(tx * TILE_SIZE);
        py1 = (float)(ty * TILE_SIZE);
        px2 = px1 + 8.0f;
        py2 = py1 + 8.0f;
      }

      if (!rectOverlap(hx1, hy1, hx2, hy2, px1, py1, px2, py2)) continue;

      int topTy = -1, baseTy = -1;
      if (!findFlagpoleColumn(tx, ty, topTy, baseTy)) continue;

      float poleTopY    = (float)(topTy * TILE_SIZE);
      (void)poleTopY;

      // Initialize state
      gFlagpole.active = true;
      gFlagpole.freezeEnemies = true;
      gFlagpole.tileX = (int16_t)tx;
      gFlagpole.topTy = (int16_t)topTy;
      gFlagpole.baseTy = (int16_t)baseTy;
      gFlagpole.poleX = (float)(tx * TILE_SIZE + 3);
      gFlagpole.poleW = 2.0f;
      gFlagpole.stateStartMs = gameplayNowMs();
      gFlagpole.state = FLAGPOLE_STATE_SLIDE;

      // Slide endpoints
      gFlagpole.slideEndHeroY = (float)(baseTy * TILE_SIZE) - heroHitH;

      // Init new level-end sequence variables
      gFlagpole.jumpImpulseDone = false;
      gFlagpole.waitAtDoorMs = 0;
      gFlagpole.fadeStartMs = 0;
      gFlagpole.doorTargetValid = false;
      gFlagpole.doorTargetX = gFlagpole.poleX + 32.0f; // fallback: ~4 tiles right in pixels (32px)

      // Try to locate the castle top-left tile and derive a “door stop” X.
      // Castle sprite is 40px wide. Door is roughly centered; tune offset if needed.
      int16_t castleTx = -1, castleTy = -1;
      if (findTileTopLeft(TILE_ID_MARIO_CASTLE, castleTx, castleTy)) {
        const float castlePxX = (float)(castleTx * TILE_SIZE);
        // 18 is approx door center in your 40px castle.
        // Add + (TILE_SIZE * 0.5f) to correct the remaining half-tile (4px) misalignment.
        gFlagpole.doorTargetX = (castlePxX + 18.0f - (float)TILE_SIZE) + (TILE_SIZE * 0.5f);
        gFlagpole.doorTargetValid = true;
      }

      // Cloth-only flag behavior: lock cloth to grab height, then slide down with Mario.
      if (gFlagPole.active && gFlagPole.tileX == tx) {
        gFlagPole.grabbed = true;
        gFlagPole.sliding = true;

        // Reward is based on CURRENT flag position (not Mario Y)
        if (!gFlagPole.coinsGiven) {
          uint8_t coins = computeFlagPoleCoinsFromClothY(gFlagPole.flagWorldY);
          coinCount += coins;
          gFlagPole.coinsGiven = true;
          gFlagPole.awardedCoins = coins;
        }
      }

      // Snap Mario to the pole (hug from the left)
      heroVX = 0.0f;
      heroVY = 0.0f;
      onGround = false;
      slopeSlideActive = false;
      spinActive = false;
      slamActive = false;
      currentPose = POSE_JUMP;

      float attachX = gFlagpole.poleX - (float)heroW + 4.0f;
      if (attachX < 0.0f) attachX = 0.0f;
      if (attachX > WORLD_WIDTH - heroW) attachX = WORLD_WIDTH - heroW;
      heroX = attachX;

      return true;
    }
  }

  return false;
}

static void updateFlagpoleSequence(int &bobOffset, int &crouchOffset) {
  if (!gFlagpole.active) return;

  uint32_t now = gameplayNowMs();

  // Freeze most actions
  slopeSlideActive = false;
  spinActive = false;
  slamActive = false;

  switch (gFlagpole.state) {
    case FLAGPOLE_STATE_SLIDE: {
      const float SLIDE_SPEED = 1.15f;

      // Keep Mario locked onto the pole X
      float attachX = gFlagpole.poleX - (float)heroW + 4.0f;
      if (attachX < 0.0f) attachX = 0.0f;
      if (attachX > WORLD_WIDTH - heroW) attachX = WORLD_WIDTH - heroW;
      heroX = attachX;
      heroVX = 0.0f;

      heroY += SLIDE_SPEED;
      if (heroY >= gFlagpole.slideEndHeroY) {
        heroY = gFlagpole.slideEndHeroY;
        heroVY = 0.0f;
        onGround = true;
        gFlagpole.state = FLAGPOLE_STATE_LAND_HOLD;
        gFlagpole.stateStartMs = now;
      } else {
        onGround = false;
      }

      // Keep cloth synced to Mario during slide
      updateFlagPoleCloth();

      currentPose = POSE_JUMP;
      bobOffset = 0;
      crouchOffset = 0;
      break;
    }

    case FLAGPOLE_STATE_LAND_HOLD: {
      // Brief hold at the bottom, then begin level-end sequence

      // Keep Mario locked onto the pole X
      float attachX = gFlagpole.poleX - (float)heroW + 4.0f;
      if (attachX < 0.0f) attachX = 0.0f;
      if (attachX > WORLD_WIDTH - heroW) attachX = WORLD_WIDTH - heroW;
      heroX = attachX;
      heroVX = 0.0f;

      onGround = true;
      heroVY = 0.0f;
      currentPose = POSE_STAND;
      bobOffset = 0;
      crouchOffset = 0;

      // Cloth stops sliding once Mario lands
      if (gFlagPole.active) gFlagPole.sliding = false;

      if (now - gFlagpole.stateStartMs >= 600) {
        gFlagpole.state = FLAGPOLE_STATE_JUMP_OFF;
        gFlagpole.stateStartMs = now;
        gFlagpole.jumpImpulseDone = false;
      }
      break;
    }

    case FLAGPOLE_STATE_JUMP_OFF: {
      // One-time “hop off” impulse
      if (!gFlagpole.jumpImpulseDone) {
        gFlagpole.jumpImpulseDone = true;

        // Start from the pole attach position
        float attachX = gFlagpole.poleX - (float)heroW + 4.0f;
        if (attachX < 0.0f) attachX = 0.0f;
        if (attachX > WORLD_WIDTH - heroW) attachX = WORLD_WIDTH - heroW;
        heroX = attachX;

        heroVX = 1.2f;   // forward push
        heroVY = -3.0f;  // jump impulse
        onGround = false;
        currentPose = POSE_JUMP;
      }

      // Simple scripted jump physics (since normal movement/physics is skipped during cutscene)
      heroX += heroVX;
      if (heroX < 0.0f) heroX = 0.0f;
      if (heroX > WORLD_WIDTH - heroW) heroX = WORLD_WIDTH - heroW;

      heroVY += GRAVITY;
      float newY = heroY + heroVY;

      float groundY = computeHeroGroundY(heroX, newY, heroHitH);
      if (newY >= groundY) {
        heroY = groundY;
        heroVY = 0.0f;
        onGround = true;

        gFlagpole.state = FLAGPOLE_STATE_RUN_TO_DOOR;
        gFlagpole.stateStartMs = now;
      } else {
        heroY = newY;
        onGround = false;
      }

      bobOffset = 0;
      crouchOffset = 0;
      break;
    }

    case FLAGPOLE_STATE_RUN_TO_DOOR: {
      // Run speed
      const float runSpeed = 1.0f;

      // Stop threshold
      const float stopEpsilon = 1.0f;

      float dx = gFlagpole.doorTargetX - heroX;

      if (dx <= stopEpsilon) {
        // Arrived at door
        heroVX = 0.0f;
        heroX = gFlagpole.doorTargetX;
        if (heroX < 0.0f) heroX = 0.0f;
        if (heroX > WORLD_WIDTH - heroW) heroX = WORLD_WIDTH - heroW;

        // Keep feet snapped to ground.
        float groundY = computeHeroGroundY(heroX, heroY, heroHitH);
        heroY = groundY;
        onGround = true;
        heroVY = 0.0f;

        currentPose = POSE_STAND;
        bobOffset = 0;
        crouchOffset = 0;

        gFlagpole.state = FLAGPOLE_STATE_WAIT_AT_DOOR;
        gFlagpole.waitAtDoorMs = now;
        gFlagpole.stateStartMs = now;
      } else {
        heroVX = runSpeed;
        heroX += heroVX;
        if (heroX > WORLD_WIDTH - heroW) heroX = WORLD_WIDTH - heroW;

        // Keep feet snapped to ground.
        float groundY = computeHeroGroundY(heroX, heroY, heroHitH);
        heroY = groundY;
        onGround = true;
        heroVY = 0.0f;

        currentPose = POSE_STAND;
        bobOffset = ((now / 110) & 1) ? 1 : 0;
        crouchOffset = 0;
      }
      break;
    }

    case FLAGPOLE_STATE_WAIT_AT_DOOR: {
      heroVX = 0.0f;
      heroVY = 0.0f;
      // Keep feet snapped to ground.
      heroY = computeHeroGroundY(heroX, heroY, heroHitH);
      onGround = true;
      currentPose = POSE_STAND;
      bobOffset = 0;
      crouchOffset = 0;

      if (now - gFlagpole.waitAtDoorMs >= 1000) {
        // Begin fade to black (reuse tunnel fade overlay)
        gFlagpole.state = FLAGPOLE_STATE_FADE_OUT;
        gFlagpole.fadeStartMs = now;
        gFlagpole.stateStartMs = now;

        // Ensure fade starts from visible
        gPipeFadeAmount = 0;
      }
      break;
    }

    case FLAGPOLE_STATE_FADE_OUT: {
      heroVX = 0.0f;
      heroVY = 0.0f;
      heroY = computeHeroGroundY(heroX, heroY, heroHitH);
      onGround = true;
      currentPose = POSE_STAND;
      bobOffset = 0;
      crouchOffset = 0;

      // Fade duration (ms). Tune to match your tunnel feel.
      const uint32_t FADE_MS = 450;
      const uint32_t HOLD_BLACK_MS = 200;

      uint32_t t = now - gFlagpole.fadeStartMs;

      if (t <= FADE_MS) {
        // Ramp 0..255
        float a = (float)t / (float)FADE_MS;
        uint32_t v = (uint32_t)(a * 255.0f);
        if (v > 255) v = 255;
        gPipeFadeAmount = (uint8_t)v;
      } else {
        gPipeFadeAmount = 255;

        if (t >= (FADE_MS + HOLD_BLACK_MS)) {
          // Keep screen black while we reset.
          // IMPORTANT: do NOT clear cutscene states yet — we still need to fade back IN.
          // Level clear: advance to the next overworld map (1->2->3).
          if (gControlMode == AUTO_PLAY) {
            // In AUTO_PLAY, do NOT hot-reset the world mid-flagpole sequence.
            // Hand off to the attract intro restart, which holds black briefly,
            // then performs a clean reset + fade-in with stable demo start conditions.
            restartAttractLoopAfterAutoClear(now);
            break;
          } else {
            gCurrentMapId = nextMapAfterClear(gCurrentMapId);
            // New map starts with a fresh timer.
            gMapTimeSeconds = 300;
            gLastMapTimeTickMs = gameplayNowMs();
            loadTestMap(false);
          }

          // Start fading BACK IN
          gFlagpole.state = FLAGPOLE_STATE_FADE_IN;
          gFlagpole.fadeStartMs = now;
          gFlagpole.stateStartMs = now;

          // ensure full black at start of fade in
          gPipeFadeAmount = 255;
        }
      }
      break;
    }

    case FLAGPOLE_STATE_FADE_IN: {
      heroVX = 0.0f;
      heroVY = 0.0f;
      currentPose = POSE_STAND;
      bobOffset = 0;
      crouchOffset = 0;

      const uint32_t FADE_MS = 450;
      uint32_t t = now - gFlagpole.fadeStartMs;

      if (t <= FADE_MS) {
        float a = (float)t / (float)FADE_MS;
        // 255 -> 0
        int32_t v = 255 - (int32_t)(a * 255.0f);
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        gPipeFadeAmount = (uint8_t)v;
      } else {
        gPipeFadeAmount = 0;

        // End cutscene
        resetPipeTransitionState();
        resetFlagpoleState();
        return;
      }
      break;
    }

    case FLAGPOLE_STATE_WALK_OFF: {
      // Simple auto-walk to the right for a short duration.
      const float WALK_SPEED = 0.75f;
      heroX += WALK_SPEED;
      if (heroX > WORLD_WIDTH - heroW) heroX = WORLD_WIDTH - heroW;

      // Keep feet snapped to ground.
      float groundY = computeHeroGroundY(heroX, heroY, heroHitH);
      heroY = groundY;
      onGround = true;
      heroVY = 0.0f;

      currentPose = POSE_STAND;
      bobOffset = ((now / 110) & 1) ? 1 : 0;
      crouchOffset = 0;

      if (now - gFlagpole.stateStartMs >= 1200) {
        // Legacy path: steer into the new door-run sequence.
        gFlagpole.state = FLAGPOLE_STATE_RUN_TO_DOOR;
        gFlagpole.stateStartMs = now;
      }
      break;
    }

    case FLAGPOLE_STATE_CLEAR: {
      // Reload the map like a "next" transition placeholder.
      // This preserves coins/lives/time (unlike resetWorld()).
      loadTestMap();
      resetPipeTransitionState();
      resetFlagpoleState();
      return;
    }

    default:
    case FLAGPOLE_STATE_IDLE:
      resetFlagpoleState();
      return;
  }
}

// ─────────────────────────────────────────────
// Reset pipe transition state
// ─────────────────────────────────────────────

void resetPipeTransitionState() {
  gPipeTransition.active = false;
  gPipeTransition.phase = PIPE_PHASE_IDLE;
  gPipeTransition.entryIsLeft = false;
  gPipeTransition.entrySideDir = -1;
  gPipeTransition.entrySnapX = 0.0f;
  gPipeTransition.entrySnapY = 0.0f;
  gPipeTransition.pairIndex = 0;
  gPipeTransition.entryX = gPipeTransition.entryY = 0.0f;
  gPipeTransition.exitX  = gPipeTransition.exitY  = 0.0f;
  gPipeTransition.exitTileX = 0;
  gPipeTransition.exitTileY = 0;
  gPipeTransition.exitTileId = TILE_ID_EMPTY;
  gPipeTransition.heroStartX = gPipeTransition.heroStartY = 0.0f;
  gPipeTransition.heroOffset = 0.0f;
  gPipeTransition.phaseStartMs = 0;
  gPipeTransition.exitIsDownPipe = false;
  gHeroVisibleDuringPipe = true;
  gPipeFadeAmount = 0;
}

// ─────────────────────────────────────────────
// Helpers: tile lookup & ground calculation
// (same as your current code, unchanged)
// ─────────────────────────────────────────────

// Return original Map_Build tile ID at (tx, ty) from PROGMEM.
uint8_t getTileIdFromMapBuild(int tx, int ty) {
  if (tx < 0 || tx >= MAP_TILES_W || ty < 0 || ty >= MAP_TILES_H) {
    return TILE_ID_EMPTY;
  }
  return pgm_read_byte(&gCurrentMapTiles[ty][tx]);
}

// ---- World Map Helpers (SINGLE source of truth = TileID in level[][]) ----
static inline uint8_t getTileId(int tx, int ty) {
  if (tx < 0 || ty < 0 || tx >= LEVEL_WIDTH || ty >= LEVEL_HEIGHT) return TILE_ID_EMPTY;
  return level[ty][tx];
}

static inline void setTileId(int tx, int ty, uint8_t id) {
  if (tx < 0 || ty < 0 || tx >= LEVEL_WIDTH || ty >= LEVEL_HEIGHT) return;
  level[ty][tx] = id;
}

static bool findTileTopLeft(uint8_t tileId, int16_t &outTx, int16_t &outTy) {
  for (int16_t ty = 0; ty < LEVEL_HEIGHT; ty++) {
    for (int16_t tx = 0; tx < LEVEL_WIDTH; tx++) {
      if (getTileId(tx, ty) == tileId) {
        outTx = tx;
        outTy = ty;
        return true;
      }
    }
  }
  return false;
}

static inline uint8_t getMobId(int tx, int ty) {
  if (tx < 0 || ty < 0 || tx >= LEVEL_WIDTH || ty >= LEVEL_HEIGHT) return 0;
  return mobLayer[ty][tx];
}

static inline void setMobId(int tx, int ty, uint8_t id) {
  if (tx < 0 || ty < 0 || tx >= LEVEL_WIDTH || ty >= LEVEL_HEIGHT) return;
  mobLayer[ty][tx] = id;
}

// Single source-of-truth for spawn marker search: scan the LOADED runtime mobLayer[][]
static bool findMarioSpawnMarkerInLoadedMobLayer(int &outTx, int &outTy) {
  for (int ty = 0; ty < LEVEL_HEIGHT; ++ty) {
    for (int tx = 0; tx < LEVEL_WIDTH; ++tx) {
      if (mobLayer[ty][tx] == TILE_ID_MOB_MARIO_SPAWN) {
        outTx = tx;
        outTy = ty;
        Serial.print("[SPAWN] Found marker at tx=");
        Serial.print(outTx);
        Serial.print(" ty=");
        Serial.println(outTy);
        return true;
      }
    }
  }
  return false;
}

// Back-compat alias (kept to minimize churn elsewhere)
static inline bool findMarioSpawnMarker(int &outTx, int &outTy) {
  return findMarioSpawnMarkerInLoadedMobLayer(outTx, outTy);
}

// Spawn marker = AIR tile where Mario stands.
// Feet land on TOP of the solid tile below (same rule as editor).
static void setHeroSpawnFromMarkerTile(int spawnTx, int spawnTy) {
  heroX = (float)(spawnTx * TILE_SIZE);
  heroY = (float)((spawnTy + 1) * TILE_SIZE) - heroHitH;

  heroVX = 0.0f;
  heroVY = 0.0f;
  onGround = true;

  // Keep camera aligned too (resetWorld() will clamp/center again)
  cameraX = heroX - 96.0f;
  if (cameraX < 0.0f) cameraX = 0.0f;
}

// Spawn marker helper:
// The Mario Spawn marker is painted into an AIR tile.
// Scan downward until we find the first standable floor tile; return the air tile directly above that floor.
static int findStandAirTileYBelowMarker(int tx, int markerTy) {
  if (tx < 0) tx = 0;
  if (tx >= LEVEL_WIDTH) tx = LEVEL_WIDTH - 1;
  if (markerTy < 0) markerTy = 0;
  if (markerTy >= LEVEL_HEIGHT) markerTy = LEVEL_HEIGHT - 1;

  for (int ty = markerTy + 1; ty < LEVEL_HEIGHT; ++ty) {
    TileType t = getTile(tx, ty);

    bool isFloorTile = tileIsSolid(t) || tileIsSlope(t) || tileIsGrassTop(t);

    // Treat large tunnel colliders as floor (green tunnel 18px overhang).
    if (!isFloorTile) {
      int anchorTx, anchorTy;
      int samplePx = tx * TILE_SIZE + (TILE_SIZE / 2);
      int samplePy = ty * TILE_SIZE + (TILE_SIZE / 2);
      if (findTunnelAnchorAtPixel(samplePx, samplePy, anchorTx, anchorTy)) {
        isFloorTile = true;
      }
    }

    if (isFloorTile) {
      int standTy = ty - 1;
      if (standTy < 0) standTy = 0;
      return standTy;
    }
  }

  // Fallback: if we never find ground, keep marker semantics as-is.
  return markerTy;
}

// ─────────────────────────────────────────────
// Spawn marker support (Map_Mobs overlay)
// ─────────────────────────────────────────────
static const uint8_t MOB_MARIO_SPAWN = TILE_ID_MOB_MARIO_SPAWN; // must match editor/export

static bool findMarioSpawnInMobLayer(int &outTx, int &outTy) {
  for (int ty = 0; ty < LEVEL_HEIGHT; ++ty) {
    for (int tx = 0; tx < LEVEL_WIDTH; ++tx) {
      if (mobLayer[ty][tx] == MOB_MARIO_SPAWN) {
        outTx = tx;
        outTy = ty;
        Serial.print("[SPAWN] Found marker at tx=");
        Serial.print(outTx);
        Serial.print(" ty=");
        Serial.println(outTy);
        return true;
      }
    }
  }
  return false;
}

// Convert spawn tile → pixel coordinates.
// Marker is placed in the "air tile above ground", so feet should land on the tile BELOW it.
static void applyMarioSpawnFromMobLayerOrFallback() {
  int sx = -1;
  int sy = -1;

  if (findMarioSpawnInMobLayer(sx, sy)) {
    // Optional: hide marker so it never renders
    // mobLayer[sy][sx] = 0;

    // Center Mario on the spawn tile horizontally (Mario is wider than 1 tile)
    heroX = (sx * TILE_SIZE) + (TILE_SIZE / 2.0f) - (heroW / 2.0f);

    // Place feet on the tile below the marker (supports slight misplacement via scan-down)
    int standTy = findStandAirTileYBelowMarker(sx, sy);
    heroY = ((standTy + 1) * TILE_SIZE) - heroHitH;

    // Capture original spawn (tile coords of the AIR tile Mario stands in)
    spawnTileX = (int16_t)sx;
    spawnTileY = (int16_t)standTy;
  } else {
    // Fallback (safe default) if marker isn't present
    heroX = 32.0f;
    heroY = (TILE_SIZE * 15) - heroHitH;

    // Fallback spawn: heroX=32px -> tileX=4; heroY aligns to stand tileY=14
    spawnTileX = 4;
    spawnTileY = 14;
  }

  // Clear velocity + state
  heroVX = 0.0f;
  heroVY = 0.0f;
  onGround = true;

  spinActive = false;
  slamActive = false;
  landingSquashFrames = 0;
  walkPhase = 0;

  Serial.print("[SPAWN] Using heroX=");
  Serial.print(heroX);
  Serial.print(" heroY=");
  Serial.println(heroY);
}

// Respawn helper used by checkpoint + original spawn.
// `markerTy` is treated like the air tile where Mario stands (same semantics as spawn marker).
static void respawnHeroAtMarkerTile(int16_t markerTx, int16_t markerTy) {
  if (markerTx < 0 || markerTy < 0) return;

  int standTy = findStandAirTileYBelowMarker((int)markerTx, (int)markerTy);

  // Center Mario on the tile (matches spawn marker behavior)
  heroX = (markerTx * TILE_SIZE) + (TILE_SIZE / 2.0f) - (heroW / 2.0f);
  heroY = ((standTy + 1) * TILE_SIZE) - heroHitH;

  heroVX = 0.0f;
  heroVY = 0.0f;
  onGround = true;

  spinActive = false;
  slamActive = false;
  landingSquashFrames = 0;
  slopeSlideActive = false;
  platformDropActive = false;
  gJustBrokeTileThisFrame = false;

  // Cancel any pipe state/fade.
  resetPipeTransitionState();
  gPipeFadeAmount = 0;
  gHeroVisibleDuringPipe = true;

  // Drop any held shell on respawn
  if (gHeroHoldingShell && gHeldShellIndex >= 0 && gHeldShellIndex < MAX_SHELLS) {
    if (gShells[gHeldShellIndex].active) {
      gShells[gHeldShellIndex].state = SHELL_STATIONARY;
      gShells[gHeldShellIndex].vx = 0.0f;
      gShells[gHeldShellIndex].vy = 0.0f;
    }
  }
  gHeroHoldingShell = false;
  gHeldShellIndex = -1;

  snapCameraToHeroSpawnAndClamp();
}

// Keep camera sane after spawning (prevents "nothing visible" feel)
static void snapCameraToHeroAndClamp() {
  float targetX = heroX + (heroW / 2.0f) - (MATRIX_WIDTH / 2.0f);
  float targetY = heroY + (heroHitH / 2.0f) - (MATRIX_HEIGHT / 2.0f);

  // World size in pixels
  float worldW = LEVEL_WIDTH  * TILE_SIZE;
  float worldH = LEVEL_HEIGHT * TILE_SIZE;

  // Clamp
  if (targetX < 0.0f) targetX = 0.0f;
  if (targetY < 0.0f) targetY = 0.0f;
  if (targetX > worldW - MATRIX_WIDTH)  targetX = worldW - MATRIX_WIDTH;
  if (targetY > worldH - MATRIX_HEIGHT) targetY = worldH - MATRIX_HEIGHT;

  cameraX = targetX;
  cameraY = targetY;

  // Reset latch so the next jump behaves consistently.
  gCameraUpFollowLatched = false;
}

// Pipe exit camera snap:
// - Ground exits: align "ground home base" (feet near bottom row).
// - Ceiling/down-facing exits: keep Mario near the top so the player can see the drop.
static void snapCameraToHeroPipeExitAndClamp(bool exitIsDownPipe) {
  float targetX = heroX + (heroW / 2.0f) - (MATRIX_WIDTH / 2.0f);

  float targetY = 0.0f;
  if (exitIsDownPipe) {
    // Spawn near top (but not clipped)
    // Small empirical tweak: Pipe Down (2x2) exits sit a few pixels low.
    // Increasing this margin raises the camera (smaller cameraY).
    const float PIPE_EXIT_DOWN_TWEAK_PX = 3.0f;
    const float topMarginPx = (float)TILE_SIZE + PIPE_EXIT_DOWN_TWEAK_PX;
    targetY = heroY - topMarginPx;
  } else {
    // Ground exits: feet baseline at bottom tile row
    // Small empirical tweak: tunnel exit spawn positions can be a few pixels off
    // due to pipe art/anchor differences. Positive values move the camera up.
    const float PIPE_EXIT_GROUND_TWEAK_PX = 3.0f;
    const float desiredFeetScreenY = (float)MATRIX_HEIGHT - (float)TILE_SIZE + PIPE_EXIT_GROUND_TWEAK_PX;
    float heroFeetWorldY = heroY + heroHitH;
    targetY = heroFeetWorldY - desiredFeetScreenY;
  }

  // World size in pixels
  float worldW = LEVEL_WIDTH  * TILE_SIZE;
  float worldH = LEVEL_HEIGHT * TILE_SIZE;

  // Clamp
  if (targetX < 0.0f) targetX = 0.0f;
  if (targetY < 0.0f) targetY = 0.0f;
  if (targetX > worldW - MATRIX_WIDTH)  targetX = worldW - MATRIX_WIDTH;
  if (targetY > worldH - MATRIX_HEIGHT) targetY = worldH - MATRIX_HEIGHT;

  cameraX = targetX;
  cameraY = targetY;

  // Reset latch after teleports for consistent post-exit behavior.
  gCameraUpFollowLatched = false;
}

// Spawn/respawn camera snap:
// Anchor Mario's feet near the bottom of the viewport so the ground line is stable.
// This avoids spawning into a "floating" view where the ground is above the bottom row.
static void snapCameraToHeroSpawnAndClamp() {
  // Horizontal: keep existing centering behavior
  float targetX = heroX + (heroW / 2.0f) - (MATRIX_WIDTH / 2.0f);

  // Vertical: place feet so the ground tile Mario stands on is the bottom row
  // of the viewport (8px). This makes spawn feel "grounded" and consistent.
  const float SPAWN_GROUND_ROWS_FROM_BOTTOM = 1.0f; // 1 row of ground visible at bottom
  const float desiredFeetScreenY = (float)MATRIX_HEIGHT - ((float)TILE_SIZE * SPAWN_GROUND_ROWS_FROM_BOTTOM);
  float heroFeetWorldY = heroY + heroHitH;
  float targetY = heroFeetWorldY - desiredFeetScreenY;

  // World size in pixels
  float worldW = LEVEL_WIDTH  * TILE_SIZE;
  float worldH = LEVEL_HEIGHT * TILE_SIZE;

  // Clamp
  if (targetX < 0.0f) targetX = 0.0f;
  if (targetY < 0.0f) targetY = 0.0f;
  if (targetX > worldW - MATRIX_WIDTH)  targetX = worldW - MATRIX_WIDTH;
  if (targetY > worldH - MATRIX_HEIGHT) targetY = worldH - MATRIX_HEIGHT;

  cameraX = targetX;
  cameraY = targetY;

  // Reset latch so spawn starts from a clean camera state.
  gCameraUpFollowLatched = false;
}

// Compatibility adapter: existing code expects TileType
inline TileType getTile(int tx, int ty) {
  return mapTileIdToTileType(getTileId(tx, ty));
}

// Optional wrapper for legacy setTile usage (keeps source compat)
static inline void setTile(int tx, int ty, TileType t) {
  setTileId(tx, ty, tileTypeToTileId(t));
}

// ─────────────────────────────────────────────
// Tunnel pair discovery and lookup
// ─────────────────────────────────────────────

void buildTunnelPairsFromMap() {
  gTunnelPairCount = 0;
  memset(gTunnelPairs, 0, sizeof(gTunnelPairs));

  auto countTile = [&](uint8_t wantId) -> int {
    int count = 0;
    for (int ty = 0; ty < LEVEL_HEIGHT; ++ty) {
      for (int tx = 0; tx < LEVEL_WIDTH; ++tx) {
        if (getTileId(tx, ty) == wantId) {
          count++;
        }
      }
    }
    return count;
  };

  // Do not use magic tile numbers for legacy compatibility.
  // Always use named TileID constants from Map_Build.h.
  auto findFirstTile = [&](uint8_t wantId, TunnelEndpoint &out) -> bool {
    for (int ty = 0; ty < LEVEL_HEIGHT; ++ty) {
      for (int tx = 0; tx < LEVEL_WIDTH; ++tx) {
        uint8_t id = getTileId(tx, ty);
        if (id == wantId) {
          out.tileX = tx;
          out.tileY = ty;
          out.tileId = id;
          return true;
        }
      }
    }
    return false;
  };

  auto printPair = [&](int idx, const TunnelPair &tp) {
    Serial.print("[PIPE] Pair ");
    Serial.print(idx);
    Serial.print(": entryId=");
    Serial.print(tp.entry.tileId);
    Serial.print(" (");
    Serial.print(tp.entry.tileX);
    Serial.print(",");
    Serial.print(tp.entry.tileY);
    Serial.print(")");

    Serial.print(" exitId=");
    Serial.print(tp.exit.tileId);
    Serial.print(" (");
    Serial.print(tp.exit.tileX);
    Serial.print(",");
    Serial.print(tp.exit.tileY);
    Serial.print(")");

    if (tp.hasLeftEntry) {
      Serial.print(" leftId=");
      Serial.print(tp.leftEntry.tileId);
      Serial.print(" (");
      Serial.print(tp.leftEntry.tileX);
      Serial.print(",");
      Serial.print(tp.leftEntry.tileY);
      Serial.print(")");
    }
    Serial.println();
  };

  // Pair 0 (legacy): 19 -> 22, optional left entrance 32
  TunnelEndpoint entry19{0,0,TILE_ID_EMPTY};
  TunnelEndpoint exit22{0,0,TILE_ID_EMPTY};
  TunnelEndpoint left32{0,0,TILE_ID_EMPTY};

  // Sanity: tunnel endpoints should be unique. If not, demos will desync.
  // (We still pick the first occurrence for now; this just exposes the problem.)
  {
    const int c19 = countTile(TILE_ID_GREEN_TUNNEL);
    const int c22 = countTile(TILE_ID_GREEN_TUNNEL_DOWN);
    const int c32 = countTile(TILE_ID_GREEN_TUNNEL_LEFT);
    const int c41 = countTile(TILE_ID_GREEN_TUNNEL_1_2);
    const int c59 = countTile(TILE_ID_GREEN_TUNNEL_DOWN_2_2);
    const int c60 = countTile(TILE_ID_GREEN_TUNNEL_LEFT_2_4);
    const int c61 = countTile(TILE_ID_GREEN_TUNNEL_4_4);

    if (c19 > 1 || c22 > 1 || c32 > 1 || c41 > 1 || c59 > 1 || c60 > 1 || c61 > 1) {
      Serial.print("[PIPE] WARN: duplicate tunnel tiles: ");
      Serial.print("19="); Serial.print(c19);
      Serial.print(" 22="); Serial.print(c22);
      Serial.print(" 32="); Serial.print(c32);
      Serial.print(" 41="); Serial.print(c41);
      Serial.print(" 59="); Serial.print(c59);
      Serial.print(" 60="); Serial.print(c60);
      Serial.print(" 61="); Serial.println(c61);
    }
  }
  bool has19 = findFirstTile(TILE_ID_GREEN_TUNNEL, entry19);
  bool has22 = findFirstTile(TILE_ID_GREEN_TUNNEL_DOWN, exit22);
  bool has32 = findFirstTile(TILE_ID_GREEN_TUNNEL_LEFT, left32);
  if (has19 && has22 && gTunnelPairCount < MAX_TUNNEL_PAIRS) {
    TunnelPair &tp = gTunnelPairs[gTunnelPairCount];
    tp.entry = entry19;
    tp.exit  = exit22;
    tp.hasLeftEntry = has32;
    if (has32) tp.leftEntry = left32;
    printPair(gTunnelPairCount, tp);
    gTunnelPairCount++;
  }

  // Pair 1: 41 -> 59
  TunnelEndpoint entry41{0,0,TILE_ID_EMPTY};
  TunnelEndpoint exit59{0,0,TILE_ID_EMPTY};
  bool has41 = findFirstTile(TILE_ID_GREEN_TUNNEL_1_2, entry41);
  bool has59 = findFirstTile(TILE_ID_GREEN_TUNNEL_DOWN_2_2, exit59);
  if (has41 && has59 && gTunnelPairCount < MAX_TUNNEL_PAIRS) {
    TunnelPair &tp = gTunnelPairs[gTunnelPairCount];
    tp.entry = entry41;
    tp.exit  = exit59;
    tp.hasLeftEntry = false;
    printPair(gTunnelPairCount, tp);
    gTunnelPairCount++;
  }

  // Pair 2: left entry 60 -> upright exit 61 (no down exit required)
  TunnelEndpoint left60{0,0,TILE_ID_EMPTY};
  TunnelEndpoint exitUpright61{0,0,TILE_ID_EMPTY};
  bool has60 = findFirstTile(TILE_ID_GREEN_TUNNEL_LEFT_2_4, left60);
  bool has61 = findFirstTile(TILE_ID_GREEN_TUNNEL_4_4, exitUpright61);
  if (has60 && has61 && gTunnelPairCount < MAX_TUNNEL_PAIRS) {
    TunnelPair &tp = gTunnelPairs[gTunnelPairCount];
    tp.entry = exitUpright61;                 // used as the upright EXIT target for left-entry
    tp.exit  = TunnelEndpoint{0,0,TILE_ID_EMPTY}; // intentionally unused for this pair
    tp.leftEntry = left60;
    tp.hasLeftEntry = true;
    printPair(gTunnelPairCount, tp);
    gTunnelPairCount++;
  }

  Serial.print("[PIPE] Built tunnel pairs. Count=");
  Serial.println(gTunnelPairCount);
}

// Find a tunnel pair whose ENTRY area is under Mario's feet (position-based).
// We treat each GREEN_TUNNEL as an 18×17 sprite anchored at its tile.
bool findTunnelPairAtFeet(float heroX, float heroY, float heroHitH, int &outPairIndex) {
  outPairIndex = -1;
  if (gTunnelPairCount <= 0) return false;

  float heroCenterX = heroX + heroW * 0.5f;
  float heroFeetY   = heroY + heroHitH;

  // Small vertical tolerance so slight jitter still counts as "on top"
  const float verticalTolerance = 4.0f;

  for (int i = 0; i < gTunnelPairCount; ++i) {
    const TunnelEndpoint &e = gTunnelPairs[i].entry;

    // Pipe sprite is 18 pixels wide × 17 pixels tall, anchored at tile (tileX, tileY).
    float pipeLeft   = e.tileX * TILE_SIZE;
    float pipeTop    = e.tileY * TILE_SIZE;
    float pipeRight  = pipeLeft + 18.0f;
    float pipeBottom = pipeTop + 17.0f; // currently unused, but kept for clarity

    bool withinX = (heroCenterX >= pipeLeft) && (heroCenterX <= pipeRight);
    bool nearTop = (heroFeetY >= pipeTop - verticalTolerance) &&
                   (heroFeetY <= pipeTop + verticalTolerance);

    if (withinX && nearTop) {
      outPairIndex = i;
      return true;
    }
  }

  return false;
}

// Find a tunnel pair whose LEFT-ENTRANCE mouth is aligned with Mario while he is
// approaching from the right and holding LEFT.
//
// Left-entrance sprite is 17×18 pixels, anchored at (tileX, tileY) * TILE_SIZE.
// We detect when Mario's LEFT side is near the pipe's RIGHT edge, and his vertical
// center is within the opening band.
// Find a tunnel pair whose named left-entrance mouth is aligned with Mario.
// dir:
//  -1 = Mario is on the RIGHT of the entrance, pressing LEFT (slides left into pipe)
//  +1 = Mario is on the LEFT of the entrance, pressing RIGHT (slides right into pipe)
bool findTunnelPairAtSideEntrance(float heroX, float heroY, float heroHitH, int8_t dir, int &outPairIndex) {
  outPairIndex = -1;
  if (gTunnelPairCount <= 0) return false;
  if (gPipeTransition.active) return false;
  if (dir == 0) return false;

  // Per requirement: ONLY the named left-entrance TileID 60 should trigger this.
  const float pipeWidth  = 17.0f;
  const float pipeHeight = 18.0f;

  const float heroLeftX   = heroX;
  const float heroRightX  = heroX + heroW;
  const float heroCenterY = heroY + heroHitH * 0.5f;

  // Vertical mouth band.
  const float mouthBandPad = 4.0f;

  for (int i = 0; i < gTunnelPairCount; ++i) {
    const TunnelPair &tp = gTunnelPairs[i];
    if (!tp.hasLeftEntry) continue;

    if (tp.leftEntry.tileId != TILE_ID_GREEN_TUNNEL_LEFT_2_4) continue; // TileID 60 only

    const TunnelEndpoint &e = tp.leftEntry;
    float pipeLeft  = e.tileX * TILE_SIZE;
    float pipeTop   = e.tileY * TILE_SIZE;
    float pipeRight = pipeLeft + pipeWidth;

    const bool withinMouthY = (heroCenterY >= (pipeTop + mouthBandPad)) &&
                              (heroCenterY <= (pipeTop + pipeHeight - mouthBandPad));

    bool touchingEdge = false;
    if (dir < 0) {
      // Touching the pipe's RIGHT edge.
      touchingEdge = (heroLeftX <= pipeRight) && (heroRightX >= (pipeRight - 2.0f));
    } else {
      // Touching the pipe's LEFT edge.
      touchingEdge = (heroRightX >= pipeLeft) && (heroLeftX <= (pipeLeft + 2.0f));
    }

    if (touchingEdge && withinMouthY) {
      outPairIndex = i;
      return true;
    }
  }

  return false;
}

// Helper: decide if HUD should be drawn during pipe transition
static inline bool shouldDrawHUDNow() {
  // If not in transition, draw normally
  if (!gPipeTransition.active) return true;

  // During fade out/in, HUD should be hidden while we're essentially black.
  // Tune threshold if needed: 220-255 range usually kills the flash.
  const uint8_t BLACK_THRESH = 220;

  if (gPipeTransition.phase == PIPE_PHASE_FADE_OUT) {
    // Hide HUD for most/all of fade out, especially near full black
    return (gPipeFadeAmount < BLACK_THRESH);
  }

  if (gPipeTransition.phase == PIPE_PHASE_FADE_IN) {
    // Also hide HUD until we're visibly back (prevents "flash on black")
    return (gPipeFadeAmount < BLACK_THRESH);
  }

  // During sink/rise we want HUD visible (per prior requirement)
  return true;
}

// We want HUD visible during death pop/fall and death fade-out/in so it fades naturally.
// HUD should be hidden ONLY while fully black showing GAME OVER.
static inline bool shouldDrawHudThisFrame() {
  if (deathIsBlackPhase()) return false;
  return shouldDrawHUDNow();
}

// ─────────────────────────────────────────────
// Start pipe transition
// ─────────────────────────────────────────────

void startPipeTransition(uint8_t pairIndex,
                         bool entryIsLeft,
                         int8_t entrySideDir,
                         float entryLockX,
                         float entryLockY,
                         float exitSpawnX,
                         float exitSpawnY,
                         int exitTileX,
                         int exitTileY,
                         uint8_t exitTileId)
{
  resetPipeTransitionState();

  // Require releasing DOWN before allowing another DOWN-entry.
  gPipeEntryNeedsDownRelease = true;

  gPipeTransition.active = true;
  gPipeTransition.phase = PIPE_PHASE_SINK;
  gPipeTransition.pairIndex = pairIndex;

  gPipeTransition.entryIsLeft = entryIsLeft;
  gPipeTransition.entrySideDir = (entrySideDir == 0) ? -1 : entrySideDir;
  gPipeTransition.entryX = entryLockX;
  gPipeTransition.entryY = entryLockY;

  // For side-entry we treat entryLockX/Y as the snap point.
  gPipeTransition.entrySnapX = entryLockX;
  gPipeTransition.entrySnapY = entryLockY;

  gPipeTransition.exitX = exitSpawnX;
  gPipeTransition.exitY = exitSpawnY;
  gPipeTransition.exitTileX = exitTileX;
  gPipeTransition.exitTileY = exitTileY;
  gPipeTransition.exitTileId = exitTileId;

  // Preserve legacy behavior: exit direction is inferred from the chosen exit tile.
  gPipeTransition.exitIsDownPipe =
    (exitTileId == TILE_ID_GREEN_TUNNEL_DOWN) ||
    (exitTileId == TILE_ID_GREEN_TUNNEL_DOWN_2_2);

  gPipeTransition.heroStartX = heroX;
  gPipeTransition.heroStartY = heroY;
  gPipeTransition.heroOffset = 0.0f;
  gPipeTransition.phaseStartMs = gameplayNowMs();
  gHeroVisibleDuringPipe = true;
  gPipeFadeAmount = 0;
  
  // Initialize normalized progress (t goes 0->1 for enter, 1->0 for exit)
  // Duration calculated from original sink depth and speed: 18px @ 14px/s = ~1.286s
  gPipeTransition.t = 0.0f;
  gPipeTransition.duration = 18.0f / 14.0f; // sink depth / sink speed

  Serial.printf("[PIPE] Starting pipe transition on pair %u\n", pairIndex);
  Serial.printf("[PIPE] Phase -> %d\n", (int)gPipeTransition.phase);
}

// ─────────────────────────────────────────────
// Pipe transition animation update
// ─────────────────────────────────────────────

void updatePipeTransition(uint32_t nowMs, float dtSeconds) {
  if (!gPipeTransition.active) {
    return;
  }

  switch (gPipeTransition.phase) {
    case PIPE_PHASE_SINK: {
      // Mario sinks into the entry pipe using normalized progress
      gPipeTransition.t = clamp01(gPipeTransition.t + dtSeconds / gPipeTransition.duration);
      float u = pipeCurve(gPipeTransition.t);
      
      // Convert normalized progress to heroOffset
      const float sinkDepth = gPipeTransition.entryIsLeft ? (float)PIPE_LEFT_SINK_DIST : 18.0f;
      gPipeTransition.heroOffset = u * sinkDepth;

      // Entry motion depends on the entry mode.
      if (gPipeTransition.entryIsLeft) {
        // Slide into the pipe horizontally while keeping Y locked.
        // entrySideDir: -1 slides left, +1 slides right.
        heroX = gPipeTransition.entrySnapX + (float)gPipeTransition.entrySideDir * gPipeTransition.heroOffset;
        heroY = gPipeTransition.entrySnapY;
      } else {
        // Default/legacy: sink downward while X is locked.
        heroX = gPipeTransition.entryX;
        heroY = gPipeTransition.heroStartY + gPipeTransition.heroOffset;
      }

      // Freeze movement & crouch pose
      heroVX = 0.0f;
      heroVY = 0.0f;
      onGround = true;
      currentPose = POSE_CROUCH;

      // Hide Mario when sink progress reaches hide threshold
      const float hideProgress = 10.0f / sinkDepth; // normalized (10px / 18px)
      if (gPipeTransition.t >= hideProgress && gHeroVisibleDuringPipe) {
        gHeroVisibleDuringPipe = false;
      }

      // Transition to fade when fully sunk (t >= 1.0)
      if (gPipeTransition.t >= 1.0f) {
        gPipeTransition.phase = PIPE_PHASE_FADE_OUT;
        gPipeTransition.phaseStartMs = nowMs;
        Serial.printf("[PIPE] Phase -> %d\n", (int)gPipeTransition.phase);
      }
      break;
    }

    case PIPE_PHASE_FADE_OUT: {
      const uint32_t FADE_OUT_MS = 700; // slower fade to black
      uint32_t elapsed = nowMs - gPipeTransition.phaseStartMs;
      if (elapsed > FADE_OUT_MS) elapsed = FADE_OUT_MS;
      gPipeFadeAmount = (uint8_t)map(elapsed, 0, FADE_OUT_MS, 0, 255);

      if (elapsed >= FADE_OUT_MS) {
        // Ensure fully black
        gPipeFadeAmount = 255;
        
        // Enter HOLD_BLACK phase to prevent flash
        gPipeTransition.phase = PIPE_PHASE_HOLD_BLACK;
        gPipeTransition.phaseStartMs = nowMs;
        gPipeTransition.holdBlackFrames = 2; // hold for 2 frames minimum
        gPipeTransition.teleportDone = false;
        Serial.printf("[PIPE] Phase -> %d (HOLD_BLACK)\n", (int)gPipeTransition.phase);
      }
      break;
    }

    case PIPE_PHASE_HOLD_BLACK: {
      // Hold at pure black for at least 1 frame to prevent flash
      // Perform teleport exactly once while black
      if (!gPipeTransition.teleportDone) {
        // Teleport hero & camera to the exit pipe
        heroX = gPipeTransition.exitX;
        heroY = gPipeTransition.exitY;
        heroVX = 0.0f;
        heroVY = 0.0f;

        // Snap camera to a good view for the exit type (ground vs ceiling pipe)
        snapCameraToHeroPipeExitAndClamp(gPipeTransition.exitIsDownPipe);

        if (!gPipeTransition.exitIsDownPipe) {
          Serial.printf("[PIPE] Teleporting to upright exit tileId=%u at (%d,%d)\n",
                        (unsigned)gPipeTransition.exitTileId,
                        (int)gPipeTransition.exitTileX,
                        (int)gPipeTransition.exitTileY);
        } else {
          Serial.printf("[PIPE] Teleporting to down exit tileId=%u at (%d,%d)\n",
                        (unsigned)gPipeTransition.exitTileId,
                        (int)gPipeTransition.exitTileX,
                        (int)gPipeTransition.exitTileY);
        }
        Serial.printf("[PIPE] Teleported to exit. heroX=%.2f heroY=%.2f\n", heroX, heroY);
        
        // For exit rise/fall animation, reset offset
        gPipeTransition.heroOffset = 0.0f;
        
        gPipeTransition.teleportDone = true;
      }
      
      // Count down frames while holding black
      if (gPipeTransition.holdBlackFrames > 0) {
        gPipeTransition.holdBlackFrames--;
      }
      
      // After holding black for required frames, start fade-in
      if (gPipeTransition.holdBlackFrames == 0 && gPipeTransition.teleportDone) {
        gPipeTransition.phase = PIPE_PHASE_FADE_IN;
        gPipeTransition.phaseStartMs = nowMs;
        Serial.printf("[PIPE] Phase -> %d (FADE_IN)\n", (int)gPipeTransition.phase);
      }
      break;
    }
    
    case PIPE_PHASE_TELEPORT: {
      // This phase is now obsolete (handled in HOLD_BLACK)
      // but kept for compatibility if somehow reached
      gPipeTransition.phase = PIPE_PHASE_FADE_IN;
      gPipeTransition.phaseStartMs = nowMs;
      Serial.printf("[PIPE] Phase -> %d (legacy teleport path)\n", (int)gPipeTransition.phase);
      break;
    }

    case PIPE_PHASE_FADE_IN: {
      const uint32_t FADE_IN_MS = 700; // slower fade back in
      uint32_t elapsed = nowMs - gPipeTransition.phaseStartMs;
      if (elapsed > FADE_IN_MS) elapsed = FADE_IN_MS;
      gPipeFadeAmount = (uint8_t)map(elapsed, 0, FADE_IN_MS, 255, 0);

      if (elapsed >= FADE_IN_MS) {
        gPipeTransition.phase = PIPE_PHASE_RISE;
        gPipeTransition.phaseStartMs = nowMs;
        Serial.printf("[PIPE] Phase -> %d\n", (int)gPipeTransition.phase);

        // Start just inside pipe; Mario still hidden
        gHeroVisibleDuringPipe = false;
        
        // Initialize exit animation to play in reverse (1.0 -> 0.0)
        gPipeTransition.t = 1.0f;
        gPipeTransition.heroOffset = 0.0f;
      }
      break;
    }

    case PIPE_PHASE_RISE: {
      // Reverse playback: identical timing to enter
      gPipeTransition.t = clamp01(gPipeTransition.t - dtSeconds / gPipeTransition.duration);
      float u = pipeCurve(gPipeTransition.t);
      
      // Convert normalized progress using SAME mapping as enter (reversed)
      const int HERO_H = (int)heroHitH;

      // Compute proper exit position: start at pipe rim, rise/fall out
      {
        // Pipe tile -> world pixels
        int pipeWorldX = (int)gPipeTransition.exitTileX * TILE_SIZE;
        int pipeWorldY = (int)gPipeTransition.exitTileY * TILE_SIZE;

        // Pipe opening dimensions
        const float pipeWidth  = 18.0f;
        const float pipeHeight = 17.0f;

        // Define "rimY" as the top edge of the pipe tile
        int rimY = pipeWorldY;

        // Center Mario horizontally on pipe
        float centerX = pipeWorldX + pipeWidth * 0.5f - heroW * 0.5f;
        heroX = centerX;

        if (!gPipeTransition.exitIsDownPipe) {
          // Upright exit: Mario rises UP out of the pipe (no fall frame).
          // Start fully inside: feet near the bottom of the pipe interior.
          const int startInsideY = (int)(rimY + pipeHeight - HERO_H + 1);

          // Travel upward by ~18px (matches typical sink depth and avoids 1-frame pop).
          const float exitTravel = (float)TILE_SIZE + 10.0f; // 18px
          gPipeTransition.heroOffset = (1.0f - u) * exitTravel;
          heroY = (float)startInsideY - gPipeTransition.heroOffset;

          // Mario becomes visible using SAME threshold as enter (mirrored)
          // In enter: visible until hideProgress (~0.56), then hidden
          // In exit: hidden until (1 - hideProgress) (~0.44), then visible
          const float sinkDepth = 18.0f;
          const float hideProgress = 10.0f / sinkDepth; // same as enter
          if (!gHeroVisibleDuringPipe) {
            if (gPipeTransition.t <= (1.0f - hideProgress)) {
              gHeroVisibleDuringPipe = true;
            }
          }
        } else {
          // Downward exit: Mario falls OUT downward
          const int PIPE_RIM_Y = rimY + (int)pipeHeight; // bottom of pipe opening

          const float exitDepth = (float)HERO_H + 4.0f; // legacy behavior
          gPipeTransition.heroOffset = (1.0f - u) * exitDepth;

          // Mario's FEET start at bottom rim and move downward
          int feetY = PIPE_RIM_Y + (int)gPipeTransition.heroOffset;

          // Convert feet position to sprite top-left Y
          heroY = feetY - HERO_H;

          // Mario becomes visible using SAME threshold as enter (mirrored)
          const float sinkDepth = 18.0f;
          const float hideProgress = 10.0f / sinkDepth;
          if (!gHeroVisibleDuringPipe) {
            if (gPipeTransition.t <= (1.0f - hideProgress)) {
              gHeroVisibleDuringPipe = true;
            }
          }
        }
        
        // End transition when t reaches 0 (fully reversed)
        if (gPipeTransition.t <= 0.0f) {
          resetPipeTransitionState();
          onGround = false;
          currentPose = POSE_STAND;
          Serial.println("[PIPE] Transition complete. Control returned to player.");
        }
      }
      break;
    }

    case PIPE_PHASE_IDLE:
    default:
      // Should not stay active in idle; just fail-safe
      resetPipeTransitionState();
      break;
  }
}

// Returns the slope tile directly under the hero's center (within a small vertical range),
// and sets outSlopeDir to -1 (downhill left), +1 (downhill right), or 0 if no slope found.
TileType findSlopeBelowCenter(float heroX, float heroY, float heroW, float heroHitH, int &outSlopeDir) {
  outSlopeDir = 0;

  float heroFeetY   = heroY + heroHitH;
  float heroCenterX = heroX + heroW * 0.5f;

  int tileXCenter = (int)(heroCenterX / TILE_SIZE);
  int tileYFeet   = (int)(heroFeetY  / TILE_SIZE);

  // Look straight down from the center for up to 2 tiles
  for (int dy = 0; dy <= 2; ++dy) {
    TileType t = getTile(tileXCenter, tileYFeet + dy);
    if (tileIsSlope(t)) {
      if (t == TILE_GROUND_SLOPE_UP_RIGHT ||
          t == TILE_GROUND_VERT_CLIFF_RIGHT) {
        // Low on the left, high on the right → downhill to the right
        outSlopeDir = +1;
      } else if (t == TILE_GROUND_SLOPE_UP_LEFT ||
                 t == TILE_GROUND_VERT_CLIFF_LEFT) {
        // Low on the right, high on the left → downhill to the left
        outSlopeDir = -1;
      }
      return t;
    }
  }

  return TILE_EMPTY;
}

// A tile at (tx, ty) is a one-way platform if:
//  - It's a grass-top tile
//  - AND there is no solid tile directly below it.
// This makes floating grass blocks semi-solid, but ground stacks fully solid.
bool tileIsOneWayPlatformAt(int tx, int ty) {
  TileType t = getTile(tx, ty);
  // Only the semi-ground tile behaves as a one-way/drop-through platform.
  if (t != TILE_SEMI_GROUND) return false;

  TileType below = getTile(tx, ty + 1);
  // If below is solid ground or slope, do not treat as a platform.
  if (tileIsSolid(below) || tileIsSlope(below)) {
    return false;
  }
  return true;
}

// Ground for Mario – world coordinates
// Samples a few X positions under Mario's feet and finds the nearest
// floor or slope surface close to his current feet Y.
//
// - Allows small "step up" so he can walk up slopes / 1–2 pixel ledges.
// - Ignores ceilings and floors far above his feet.
// - Returns WORLD bottom as fallback if nothing is found.
float computeHeroGroundY(float heroXLocal, float heroTopY, float hitH) {
  const float feetY = heroTopY + hitH;

  // Allow up to 4 px "step up" to climb slopes / tiny ledges
  const float MAX_STEP_UP     = 4.0f;
  // Only look up to 4 tiles (32 px) below the current feet
  const float MAX_SCAN_DOWN   = 32.0f;

  // X sample points across Mario's width
  float sampleX[3];
  sampleX[0] = heroXLocal + heroW * 0.25f;
  sampleX[1] = heroXLocal + heroW * 0.50f;
  sampleX[2] = heroXLocal + heroW * 0.75f;

  // We keep the "closest" floor in terms of feetY (smallest >= feetY - MAX_STEP_UP)
  bool   foundFloor        = false;
  float  bestSurfaceFeetY  = feetY + MAX_SCAN_DOWN;

  for (int i = 0; i < 3; ++i) {
    float sx = sampleX[i];
    if (sx < 0.0f || sx >= WORLD_WIDTH) continue;

    int tx = (int)(sx / TILE_SIZE);

    // Start scanning near the current feet row and go downward a bit
    int startTy = (int)(feetY / TILE_SIZE) - 1;  // one row above, to catch slopes
    if (startTy < 0) startTy = 0;
    int endTy = startTy + 4; // up to 4 tiles down
    if (endTy >= LEVEL_HEIGHT) endTy = LEVEL_HEIGHT - 1;

    for (int ty = startTy; ty <= endTy; ++ty) {
      TileType t = getTile(tx, ty);

      // While dropping through a platform, ignore all drop-through tiles as ground.
      if (platformDropActive && tileIsDropThroughPlatform(t)) {
        continue;
      }

      bool isFloorTile =
        tileIsSolid(t) ||
        tileIsSlope(t);

      // Special-case: large tunnel colliders that may span multiple tile columns.
      // If any tunnel anchor to the left (up to 2 tiles) covers our sample X, treat it as floor.
      // Back-check for tunnel anchors that cover this sample X (handles 18px overhang)
      int anchorTx, anchorTy;
      int samplePx = (int)sx;
      int samplePy = ty * TILE_SIZE + (TILE_SIZE/2);
      if (findTunnelAnchorAtPixel(samplePx, samplePy, anchorTx, anchorTy)) {
        isFloorTile = true;
      }

      // Drop-through platforms only count as floor when we are falling
      // and NOT in a drop-through state.
      if (!isFloorTile) {
        bool fallingDown = (heroVY >= 0.0f);
        if (fallingDown &&
            !platformDropActive &&
            tileIsDropThroughPlatform(t)) {
          isFloorTile = true;
        }
      }

      if (!isFloorTile) continue;

      float surfaceFeetY;
      if (tileIsSlope(t)) {
        surfaceFeetY = slopeGroundYForHero(t, tx, ty, sx);
      } else {
        // Flat block: use tile top as floor surface
        surfaceFeetY = ty * TILE_SIZE;
      }

      // Allow small upward ground snap after bonk (prevents fall-through)
      const float MAX_UPWARD_SNAP = 4.0f;
      
      // Ignore anything far above our feet (would be a ceiling / overhead block)
      if (surfaceFeetY < feetY - MAX_STEP_UP - MAX_UPWARD_SNAP) {
        continue;
      }
      // Ignore anything way below our feet – we only care about nearby ground
      if (surfaceFeetY > feetY + MAX_SCAN_DOWN) {
        continue;
      }

      // We want the nearest floor in this vertical window
      if (!foundFloor || surfaceFeetY < bestSurfaceFeetY) {
        foundFloor       = true;
        bestSurfaceFeetY = surfaceFeetY;
      }
    }
  }

  float heroGroundY;
  if (foundFloor) {
    heroGroundY = bestSurfaceFeetY - hitH;
  } else {
    // Fallback: treat bottom of the world as ground
    heroGroundY = WORLD_HEIGHT - hitH;
  }

  // Clamp to world bounds
  if (heroGroundY < 0.0f)               heroGroundY = 0.0f;
  if (heroGroundY > WORLD_HEIGHT - hitH) heroGroundY = WORLD_HEIGHT - hitH;

  return heroGroundY;
}

// Ground for mushroom – world coordinates
// Same feet-based search as Mario, but slightly smaller "step up".
float computeMushroomGroundY(float mushX, float mushTopY) {
  const float feetY = mushTopY + MUSH_H;

  const float MAX_STEP_UP   = 3.0f;
  const float MAX_SCAN_DOWN = 32.0f;

  float sampleX[3];
  sampleX[0] = mushX + MUSH_W * 0.25f;
  sampleX[1] = mushX + MUSH_W * 0.50f;
  sampleX[2] = mushX + MUSH_W * 0.75f;

  bool  foundFloor       = false;
  float bestSurfaceFeetY = feetY + MAX_SCAN_DOWN;

  for (int i = 0; i < 3; ++i) {
    float sx = sampleX[i];
    if (sx < 0.0f || sx >= WORLD_WIDTH) continue;

    int tx = (int)(sx / TILE_SIZE);

    int startTy = (int)(feetY / TILE_SIZE) - 1;
    if (startTy < 0) startTy = 0;
    int endTy = startTy + 4;
    if (endTy >= LEVEL_HEIGHT) endTy = LEVEL_HEIGHT - 1;

    for (int ty = startTy; ty <= endTy; ++ty) {
      TileType t = getTile(tx, ty);

      bool isFloorTile = tileIsSolid(t) || tileIsSlope(t) || tileIsGrassTop(t);

      if (!isFloorTile) continue;

      float surfaceFeetY;
      if (tileIsSlope(t)) {
        surfaceFeetY = slopeGroundYForHero(t, tx, ty, sx);
      } else {
        surfaceFeetY = ty * TILE_SIZE;
      }

      if (surfaceFeetY < feetY - MAX_STEP_UP)   continue;
      if (surfaceFeetY > feetY + MAX_SCAN_DOWN) continue;

      if (!foundFloor || surfaceFeetY < bestSurfaceFeetY) {
        foundFloor       = true;
        bestSurfaceFeetY = surfaceFeetY;
      }
    }
  }

  float mushGroundY;
  if (foundFloor) {
    mushGroundY = bestSurfaceFeetY - MUSH_H;
  } else {
    mushGroundY = WORLD_HEIGHT - MUSH_H;
  }

  if (mushGroundY < 0.0f)                    mushGroundY = 0.0f;
  if (mushGroundY > WORLD_HEIGHT - MUSH_H)   mushGroundY = WORLD_HEIGHT - MUSH_H;

  return mushGroundY;
}

  // Ground for goomba – world coordinates
  // Similar to mushroom ground scan, but uses Goomba dimensions and
  // reuses slopeGroundYForHero() for slope tiles so Goomba can walk slopes.
  float computeGoombaGroundY(float goombaX, float goombaTopY) {
    const float feetY = goombaTopY + GOOMBA_H;

    const float MAX_STEP_UP   = 3.0f;
    const float MAX_SCAN_DOWN = 32.0f;

    float sampleX[3];
    sampleX[0] = goombaX + GOOMBA_W * 0.25f;
    sampleX[1] = goombaX + GOOMBA_W * 0.50f;
    sampleX[2] = goombaX + GOOMBA_W * 0.75f;

    bool  foundFloor       = false;
    float bestSurfaceFeetY = feetY + MAX_SCAN_DOWN;

    for (int i = 0; i < 3; ++i) {
      float sx = sampleX[i];
      if (sx < 0.0f || sx >= WORLD_WIDTH) continue;

      int tx = (int)(sx / TILE_SIZE);

      int startTy = (int)(feetY / TILE_SIZE) - 1;
      if (startTy < 0) startTy = 0;
      int endTy = startTy + 4;
      if (endTy >= LEVEL_HEIGHT) endTy = LEVEL_HEIGHT - 1;

      for (int ty = startTy; ty <= endTy; ++ty) {
        TileType t = getTile(tx, ty);

        bool isFloorTile = tileIsSolid(t) || tileIsSlope(t) || tileIsGrassTop(t);

        if (!isFloorTile) continue;

        float surfaceFeetY;
        if (tileIsSlope(t)) {
          surfaceFeetY = slopeGroundYForHero(t, tx, ty, sx);
        } else {
          surfaceFeetY = ty * TILE_SIZE;
        }

        if (surfaceFeetY < feetY - MAX_STEP_UP)   continue;
        if (surfaceFeetY > feetY + MAX_SCAN_DOWN) continue;

        if (!foundFloor || surfaceFeetY < bestSurfaceFeetY) {
          foundFloor       = true;
          bestSurfaceFeetY = surfaceFeetY;
        }
      }
    }

    float goombaGroundY;
    if (foundFloor) {
      goombaGroundY = bestSurfaceFeetY - GOOMBA_H;
    } else {
      goombaGroundY = WORLD_HEIGHT - GOOMBA_H;
    }

    if (goombaGroundY < 0.0f)                    goombaGroundY = 0.0f;
    if (goombaGroundY > WORLD_HEIGHT - GOOMBA_H)  goombaGroundY = WORLD_HEIGHT - GOOMBA_H;

    return goombaGroundY;
  }

  // Ground for koopa – world coordinates
  // Mirrors computeGoombaGroundY() but uses Koopa dimensions.
  float computeKoopaGroundY(float koopaX, float koopaTopY) {
    const float feetY = koopaTopY + KOOPA_H;

    const float MAX_STEP_UP   = KOOPA_MAX_STEP_UP;
    const float MAX_SCAN_DOWN = 32.0f;

    float sampleX[3];
    sampleX[0] = koopaX + KOOPA_W * 0.25f;
    sampleX[1] = koopaX + KOOPA_W * 0.50f;
    sampleX[2] = koopaX + KOOPA_W * 0.75f;

    bool  foundFloor       = false;
    float bestSurfaceFeetY = feetY + MAX_SCAN_DOWN;

    for (int i = 0; i < 3; ++i) {
      float sx = sampleX[i];
      if (sx < 0.0f || sx >= WORLD_WIDTH) continue;

      int tx = (int)(sx / TILE_SIZE);

      int startTy = (int)(feetY / TILE_SIZE) - 1;
      if (startTy < 0) startTy = 0;
      int endTy = startTy + 4;
      if (endTy >= LEVEL_HEIGHT) endTy = LEVEL_HEIGHT - 1;

      for (int ty = startTy; ty <= endTy; ++ty) {
        TileType t = getTile(tx, ty);

        bool isFloorTile = tileIsSolid(t) || tileIsSlope(t) || tileIsGrassTop(t);
        if (!isFloorTile) continue;

        float surfaceFeetY;
        if (tileIsSlope(t)) {
          surfaceFeetY = slopeGroundYForHero(t, tx, ty, sx);
        } else {
          surfaceFeetY = ty * TILE_SIZE;
        }

        if (surfaceFeetY < feetY - MAX_STEP_UP)   continue;
        if (surfaceFeetY > feetY + MAX_SCAN_DOWN) continue;

        if (!foundFloor || surfaceFeetY < bestSurfaceFeetY) {
          foundFloor       = true;
          bestSurfaceFeetY = surfaceFeetY;
        }
      }
    }

    float koopaGroundY;
    if (foundFloor) {
      koopaGroundY = bestSurfaceFeetY - KOOPA_H;
    } else {
      koopaGroundY = WORLD_HEIGHT - KOOPA_H;
    }

    if (koopaGroundY < 0.0f)                   koopaGroundY = 0.0f;
    if (koopaGroundY > WORLD_HEIGHT - KOOPA_H) koopaGroundY = WORLD_HEIGHT - KOOPA_H;

    return koopaGroundY;
  }

  // Multi-probe ground helper (direction-aware signature for Koopa movement).
  // Current computeKoopaGroundY() already uses 3 probes; this wrapper exists
  // so Koopa movement code can call the prompt-specified API.
  static inline float computeKoopaGroundYMultiProbe(float x, float topY, int /*dir*/) {
    return computeKoopaGroundY(x, topY);
  }

// ─────────────────────────────────────────────
// Koopa Shell helpers
// ─────────────────────────────────────────────
static inline float slopeSurfaceY(uint16_t tileId, float wx, int tileTx, int tileTy) {
  // Treat vertical cliff types as their corresponding slope equivalents
  if (tileId == (uint16_t)Ground_Tile_Vert_Cliff_Left_ID)  tileId = (uint16_t)Ground_Tile_Slope_Up_Left_ID;
  if (tileId == (uint16_t)Ground_Tile_Vert_Cliff_Right_ID) tileId = (uint16_t)Ground_Tile_Slope_Up_Right_ID;

  const float tileTop = tileTy * TILE_SIZE;
  float localX  = wx - (tileTx * TILE_SIZE);
  if (localX < 0.0f) localX = 0.0f;
  if (localX > (float)TILE_SIZE) localX = (float)TILE_SIZE;

  // Y increases downward.
  // UP_RIGHT: low on left, high on right
  if (tileId == (uint16_t)Ground_Tile_Slope_Up_Right_ID) {
    float y = tileTop + (TILE_SIZE - localX);
    if (y < tileTop + 1.0f) y = tileTop + 1.0f;
    if (y > tileTop + (float)TILE_SIZE) y = tileTop + (float)TILE_SIZE;
    return y;
  }

  // UP_LEFT: high on left, low on right
  if (tileId == (uint16_t)Ground_Tile_Slope_Up_Left_ID) {
    float y = tileTop + (localX + 1.0f);
    if (y < tileTop + 1.0f) y = tileTop + 1.0f;
    if (y > tileTop + (float)TILE_SIZE) y = tileTop + (float)TILE_SIZE;
    return y;
  }

  // Fallback: flat top
  return tileTop;
}

static float computeShellGroundY(float shellX, float shellTopY) {
  const float feetY = shellTopY + SHELL_H;

  // Allow small step-up so shells don't snag at tile seams.
  const float MAX_STEP_UP   = 4.0f;
  const float MAX_SCAN_DOWN = 32.0f;

  float sampleX[3] = {
    shellX + SHELL_W * 0.25f,
    shellX + SHELL_W * 0.50f,
    shellX + SHELL_W * 0.75f
  };

  bool  found = false;
  float bestSurfaceFeetY = feetY + MAX_SCAN_DOWN;

  for (int i = 0; i < 3; ++i) {
    float sx = sampleX[i];
    if (sx < 0.0f || sx >= WORLD_WIDTH) continue;

    // Scan from slightly above feet to below feet to find first surface.
    for (float dy = -MAX_STEP_UP; dy <= MAX_SCAN_DOWN; dy += 1.0f) {
      float probeFeetY = feetY + dy;

      int tx = (int)(sx / TILE_SIZE);
      int ty = (int)(probeFeetY / TILE_SIZE);
      if (tx < 0 || tx >= LEVEL_WIDTH || ty < 0 || ty >= LEVEL_HEIGHT) continue;

      uint16_t tid = (uint16_t)getTileId(tx, ty);
      if (!isSolidTileId((uint8_t)tid) && !isKoopaSlopeTile(tid) && !isKoopaVerticalCliff(tid)) continue;

      float surfaceFeetY;

      if (isKoopaSlopeTile(tid) || isKoopaVerticalCliff(tid)) {
        // For slopes, compute the surface Y inside the tile at sx
        surfaceFeetY = slopeSurfaceY(tid, sx, tx, ty);
      } else {
        // Flat solid tile: surface is tile top
        surfaceFeetY = ty * TILE_SIZE;
      }

      if (!found || surfaceFeetY < bestSurfaceFeetY) {
        found = true;
        bestSurfaceFeetY = surfaceFeetY;
      }
      break; // stop scanning once we found a surface for this sampleX
    }
  }

  float groundY;
  if (found) groundY = bestSurfaceFeetY - SHELL_H;
  else       groundY = WORLD_HEIGHT - SHELL_H;

  if (groundY < 0.0f) groundY = 0.0f;
  if (groundY > WORLD_HEIGHT - SHELL_H) groundY = WORLD_HEIGHT - SHELL_H;
  return groundY;
}

static void resetShells() {
  for (int i = 0; i < MAX_SHELLS; ++i) {
    gShells[i].active = false;
    gShells[i].x = 0.0f;
    gShells[i].y = 0.0f;
    gShells[i].vx = 0.0f;
    gShells[i].vy = 0.0f;
    gShells[i].dir = 1;
    gShells[i].state = SHELL_STATIONARY;
    gShells[i].ignoreHeroUntilMs = 0;
  }
  gHeroHoldingShell = false;
  gHeldShellIndex = -1;
}

static int spawnShellAt(float x, float y) {
  for (int i = 0; i < MAX_SHELLS; ++i) {
    if (gShells[i].active) continue;
    gShells[i].active = true;
    gShells[i].x = x;
    gShells[i].y = y;
    gShells[i].vx = 0.0f;
    gShells[i].vy = 0.0f;
    gShells[i].dir = 1;
    gShells[i].state = SHELL_STATIONARY;
    gShells[i].ignoreHeroUntilMs = 0;
    // Snap to ground
    float gy = computeShellGroundY(gShells[i].x, gShells[i].y);
    if (gShells[i].y > gy) gShells[i].y = gy;
    return i;
  }
  return -1;
}

static void releaseHeldShellInternal(bool kick) {
  if (!gHeroHoldingShell) return;
  if (gHeldShellIndex < 0 || gHeldShellIndex >= MAX_SHELLS) {
    gHeroHoldingShell = false;
    gHeldShellIndex = -1;
    return;
  }
  KoopaShell &sh = gShells[gHeldShellIndex];
  if (!sh.active) {
    gHeroHoldingShell = false;
    gHeldShellIndex = -1;
    return;
  }

  uint32_t now = gameplayNowMs();
  sh.dir = gHeroFacingDir;

  if (kick) {
    sh.state = SHELL_SLIDING;
    sh.vx = SHELL_KICK_SPEED * (float)sh.dir;
    sh.vy = 0.0f;
    sh.ignoreHeroUntilMs = now + SHELL_KICK_GRACE_MS;
  } else {
    sh.state = SHELL_STATIONARY;
    sh.vx = 0.0f;
    sh.vy = 0.0f;
    sh.ignoreHeroUntilMs = now + 250;
  }

  // Place shell OUTSIDE hero hitbox so we don't instantly collide
  float dropX = heroX + (heroW * 0.5f - SHELL_W * 0.5f) + (float)gHeroFacingDir * (heroW * 0.5f + SHELL_W * 0.5f + 2.0f);
  float dropY = heroY + heroHitH - SHELL_H;

  sh.x = dropX;
  sh.y = dropY;

  // Snap to ground after placing
  float gy = computeShellGroundY(sh.x, sh.y);
  if (sh.y > gy) sh.y = gy;

  // Extra safety: ignore hero briefly even if we are still touching (prevents "kick death")
  uint32_t now2 = gameplayNowMs();
  if (sh.ignoreHeroUntilMs < now2 + 300) sh.ignoreHeroUntilMs = now2 + 300;

  gHeroHoldingShell = false;
  gHeldShellIndex = -1;
}

static void throwHeldShellUp() {
  if (!gHeroHoldingShell) return;
  if (gHeldShellIndex < 0 || gHeldShellIndex >= MAX_SHELLS) {
    gHeroHoldingShell = false;
    gHeldShellIndex = -1;
    return;
  }

  KoopaShell &sh = gShells[gHeldShellIndex];
  if (!sh.active) {
    gHeroHoldingShell = false;
    gHeldShellIndex = -1;
    return;
  }

  // Release from hold into thrown-up state
  sh.state = SHELL_THROWN_UP;

  // Start just above Mario’s head, centered
  sh.x = heroX + (heroW * 0.5f) - (SHELL_W * 0.5f);
  sh.y = heroY - (float)SHELL_H - 1.0f;

  sh.vx = SHELL_THROW_UP_VX;
  sh.vy = SHELL_THROW_UP_VY;

  // Avoid instant self-hit
  sh.ignoreHeroUntilMs = gameplayNowMs() + SHELL_THROW_GRACE_MS;

  gHeroHoldingShell = false;
  gHeldShellIndex = -1;
}

static void updateShells(bool runPressed, bool runJustReleased, bool upPressed) {
  (void)runPressed;
  if (gFlagpole.active && gFlagpole.freezeEnemies) return;

  if (gHeroHoldingShell && gHeldShellIndex >= 0 && gHeldShellIndex < MAX_SHELLS) {
    KoopaShell &sh = gShells[gHeldShellIndex];
    if (!sh.active) {
      gHeroHoldingShell = false;
      gHeldShellIndex = -1;
    } else if (runJustReleased) {
      // On Run release: Up + release => throw up; else keep existing drop/kick behavior
      if (upPressed) {
        throwHeldShellUp();
      } else {
        bool kick = (fabsf(gInput.moveAxis) > 0.1f) || (fabsf(heroVX) > 0.1f);
        releaseHeldShellInternal(kick);
      }
    } else {
      sh.state = SHELL_HELD;
      sh.vx = 0.0f;
      sh.vy = 0.0f;
      sh.x = heroX + heroW * 0.5f - SHELL_W * 0.5f + (float)gHeroFacingDir * 2.0f;
      sh.y = heroY + heroHitH - SHELL_H - ((heroSize == SIZE_BIG) ? 6.0f : 4.0f);
    }
  }

  for (int i = 0; i < MAX_SHELLS; ++i) {
    KoopaShell &sh = gShells[i];
    if (!sh.active) continue;
    if (sh.state == SHELL_HELD) continue;

    sh.vy += SHELL_GRAVITY;
    if (sh.vy > SHELL_MAX_FALL) sh.vy = SHELL_MAX_FALL;

    if (sh.state == SHELL_STATIONARY) {
      sh.vx = 0.0f;
    } else if (sh.state == SHELL_SLIDING) {
      // Tiny drag each tick for SMW-ish feel
      sh.vx *= 0.995f;
      if (fabsf(sh.vx) < 0.20f) {
        sh.vx = 0.0f;
        sh.state = SHELL_STATIONARY;
      }
    }

    float nextX = sh.x + sh.vx;
    float nextY = sh.y + sh.vy;

    if (nextX < 0.0f) {
      nextX = 0.0f;
      sh.vx = -sh.vx * SHELL_BOUNCE_DAMP;
      sh.dir = (sh.vx < 0.0f) ? -1 : 1;
    } else if (nextX > WORLD_WIDTH - SHELL_W) {
      nextX = WORLD_WIDTH - SHELL_W;
      sh.vx = -sh.vx * SHELL_BOUNCE_DAMP;
      sh.dir = (sh.vx < 0.0f) ? -1 : 1;
    }

    // Thrown-up ceiling collision + bonk tiles above
    if (sh.state == SHELL_THROWN_UP && sh.vy < 0.0f) {
      float topY = nextY; // shell top edge
      float xL = nextX + 1.0f;
      float xM = nextX + (SHELL_W * 0.5f);
      float xR = nextX + SHELL_W - 2.0f;

      uint8_t tL = (uint8_t)tileIdAtWorld(xL, topY);
      uint8_t tM = (uint8_t)tileIdAtWorld(xM, topY);
      uint8_t tR = (uint8_t)tileIdAtWorld(xR, topY);

      if (isSolidTileId(tL) || isSolidTileId(tM) || isSolidTileId(tR)) {
        int bonkTx = (int)(xM / TILE_SIZE);
        int bonkTy = (int)(topY / TILE_SIZE);
        handleBlockHitFromBelow(bonkTx, bonkTy);

        // Stop upward motion and begin falling next tick
        sh.vy = 0.0f;

        // Nudge the shell down so it doesn't stay embedded (place shell top at tile bottom)
        nextY = (float)((bonkTy + 1) * TILE_SIZE);
      }
    }

    float top = sh.y;
    float bottom = sh.y + SHELL_H - 1.0f;
    int tyStart = (int)(top / TILE_SIZE);
    int tyEnd   = (int)(bottom / TILE_SIZE);
    if (tyStart < 0) tyStart = 0;
    if (tyEnd >= LEVEL_HEIGHT) tyEnd = LEVEL_HEIGHT - 1;

    // Bounce off walls/solids using a side sweep (top/mid/bottom)
    if (sh.state == SHELL_SLIDING && fabsf(sh.vx) > 0.01f) {
      float edgeX = (sh.vx < 0.0f) ? nextX : (nextX + SHELL_W - 1.0f);

      float yTop = nextY + 1.0f;
      float yMid = nextY + (SHELL_H * 0.5f);
      float yBot = nextY + SHELL_H - 1.0f;

      uint16_t idTop = tileIdAtWorld(edgeX, yTop);
      uint16_t idMid = tileIdAtWorld(edgeX, yMid);
      uint16_t idBot = tileIdAtWorld(edgeX, yBot);

      // If the "wall" is actually a slope tile, do NOT bounce.
      // Slopes are handled by ground/surface logic instead.
      bool hitSolidWall =
        (isSolidTileId((uint8_t)idTop) && !isKoopaSlopeTile(idTop)) ||
        (isSolidTileId((uint8_t)idMid) && !isKoopaSlopeTile(idMid)) ||
        (isSolidTileId((uint8_t)idBot) && !isKoopaSlopeTile(idBot));

      if (hitSolidWall) {
        // Bounce
        sh.vx = -sh.vx * SHELL_BOUNCE_DAMP;
        sh.dir = (sh.vx < 0.0f) ? -1 : 1;

        // Nudge the shell out of the wall to prevent getting embedded
        sh.x = nextX;
        if (sh.vx < 0.0f) {
          sh.x = floorf(sh.x) - 1.0f;
        } else {
          sh.x = floorf(sh.x) + 1.0f;
        }
        if (sh.x < 0.0f) sh.x = 0.0f;
        if (sh.x > WORLD_WIDTH - SHELL_W) sh.x = WORLD_WIDTH - SHELL_W;

        // Recompute nextX after bounce
        nextX = sh.x + sh.vx;
        if (nextX < 0.0f) nextX = 0.0f;
        if (nextX > WORLD_WIDTH - SHELL_W) nextX = WORLD_WIDTH - SHELL_W;
      }
    }

    float groundY = computeShellGroundY(nextX, nextY);
    if (nextY >= groundY) {
      nextY = groundY;
      sh.vy = 0.0f;

      if (sh.state == SHELL_THROWN_UP) {
        sh.state = SHELL_STATIONARY;
        sh.vx = 0.0f;
      }
    }

    sh.x = nextX;
    sh.y = nextY;
  }
}

static void drawShells() {
  const int w = (int)SHELL_W;
  const int h = (int)SHELL_H;
  for (int i = 0; i < MAX_SHELLS; ++i) {
    const KoopaShell &sh = gShells[i];
    if (!sh.active) continue;
    if (sh.state == SHELL_HELD) continue;

    int x0 = (int)(sh.x - cameraX);
    int y0 = (int)(sh.y - cameraY);
    if (x0 >= MATRIX_WIDTH || x0 + w <= 0) continue;
    if (y0 >= MATRIX_HEIGHT || y0 + h <= 0) continue;

    drawWorldTileCustomSize((const uint8_t*)&Mob_Koopa_Troopa_Shell[0][0], w, h, x0, y0);
  }
}

static void checkShellEnemyCollisions() {
  if (gFlagpole.active && gFlagpole.freezeEnemies) return;

  for (int si = 0; si < MAX_SHELLS; ++si) {
    const KoopaShell &sh = gShells[si];
    if (!sh.active) continue;
    if (sh.state != SHELL_SLIDING) continue;
    if (fabsf(sh.vx) < 0.2f) continue;

    float sx1 = sh.x;
    float sx2 = sh.x + SHELL_W;
    float sy1 = sh.y;
    float sy2 = sh.y + SHELL_H;

    for (int gi = 0; gi < MAX_GOOMBAS; ++gi) {
      Goomba &gg = gGoombas[gi];
      if (!gg.active) continue;
      if (gg.state != GOOMBA_STATE_ALIVE) continue;
      if (rectOverlap(sx1, sy1, sx2, sy2, gg.x, gg.y, gg.x + GOOMBA_W, gg.y + GOOMBA_H)) {
        gg.active = false;
        gg.state = GOOMBA_STATE_DEAD;
      }
    }

    for (int ki = 0; ki < MAX_KOOPAS; ++ki) {
      Koopa &kk = gKoopas[ki];
      if (!kk.active) continue;
      if (rectOverlap(sx1, sy1, sx2, sy2, kk.x, kk.y, kk.x + KOOPA_W, kk.y + KOOPA_H)) {
        kk.active = false;
      }
    }

    // Hit Piranha Plants (only if visible enough to collide)
    for (int p = 0; p < MAX_PIRANHAS; ++p) {
      PiranhaPlant &pp = gPiranhas[p];
      if (!pp.active) continue;
      if (pp.visibleH <= PIRANHA_BLOCK_THRESHOLD_PX) continue; // same grace as hero

      float px1 = (float)pp.xpx;
      float py1 = (float)pp.pipeTopYpx - (float)pp.visibleH;
      float px2 = px1 + (float)PIRANHA_W;
      float py2 = py1 + (float)pp.visibleH;

      if (sx1 < px2 && sx2 > px1 && sy1 < py2 && sy2 > py1) {
        // Kill piranha
        pp.active = false;
        pp.visibleH = 0;
        pp.state = PIRANHA_HIDDEN;
        pp.stateStartMs = gameplayNowMs();
      }
    }
  }
}

static void checkShellHeroCollisions(bool runPressed) {
  if (gFlagpole.active && gFlagpole.freezeEnemies) return;

  float hx1 = heroX;
  float hx2 = heroX + heroW;
  float hy1 = heroY;
  float hy2 = heroY + heroHitH;

  for (int i = 0; i < MAX_SHELLS; ++i) {
    KoopaShell &sh = gShells[i];
    if (!sh.active) continue;
    if (sh.state == SHELL_HELD) continue;

    float sx1 = sh.x;
    float sx2 = sh.x + SHELL_W;
    float sy1 = sh.y;
    float sy2 = sh.y + SHELL_H;

    if (!rectOverlap(hx1, hy1, hx2, hy2, sx1, sy1, sx2, sy2)) continue;

    // ─────────────────────────────────────────────
    // NEW RULES (easy gameplay):
    // - If Run (Y) is held and Mario touches a shell: PICK IT UP (even if sliding)
    // - If Run (Y) is NOT held and Mario touches a shell: TAKE DAMAGE
    // ─────────────────────────────────────────────

    // 1) Hold Y to pick up (always)
    if (runPressed && !gHeroHoldingShell) {
      gHeroHoldingShell = true;
      gHeldShellIndex = (int16_t)i;

      sh.state = SHELL_HELD;
      sh.vx = 0.0f;
      sh.vy = 0.0f;

      // Short ignore window so we don't get hurt on the pickup frame by other checks
      sh.ignoreHeroUntilMs = gameplayNowMs() + 250;
      continue;
    }

    // 2) Not holding Y = shell hurts (stationary OR sliding)
    {
      // Respect ignore window (mainly for just-released shells)
      if ((int32_t)(gameplayNowMs() - sh.ignoreHeroUntilMs) < 0) {
        continue;
      }

      damageMarioFromEnemy();

      // Knockback direction (keep existing behavior)
      if (gameplayNowMs() < gHeroInvincUntilMs) {
        heroVX = (hx2 <= sx2 && heroX < sh.x) ? -1.5f : 1.5f;
      }
      continue;
    }
  }
}

// ─────────────────────────────────────────────
// Mushroom helpers (unchanged)
// ─────────────────────────────────────────────
void spawnMushroomFromBelow(int tx, int ty, uint8_t type) {
  if (gMushroom.active) return;

  float tileTop    = ty * TILE_SIZE;
  float centerX    = tx * TILE_SIZE + TILE_SIZE / 2.0f;

  gMushroom.active  = true;
  gMushroom.rising  = true;
  gMushroom.x       = centerX - (MUSH_W / 2.0f);
  gMushroom.y       = tileTop + 1.0f;
  gMushroom.targetY = tileTop - 6.0f;
  gMushroom.vy      = -0.2f;
  gMushroom.vx      = 0.0f;
  gMushroom.type    = type;
}

void spawnMushroomFromAbove(int tx, int ty, uint8_t type) {
  if (gMushroom.active) return;

  float tileBottom = (ty + 1) * TILE_SIZE;
  float centerX    = tx * TILE_SIZE + TILE_SIZE / 2.0f;

  gMushroom.active  = true;
  gMushroom.rising  = true;
  gMushroom.x       = centerX - (MUSH_W / 2.0f);
  gMushroom.y       = tileBottom - MUSH_H - 1.0f;
  gMushroom.targetY = tileBottom + 2.0f;
  gMushroom.vy      = +0.15f;
  gMushroom.vx      = 0.0f;
  gMushroom.type    = type;
}

// Scan the loaded runtime map and spawn Goomba entities for any painted
// goomba markers found (either in mobLayer[][] from Map_Mobs, or in level[][]
// for backward compatibility), up to MAX_GOOMBAS.
// Each spawned Goomba clears its marker tile so the entity is authoritative.
void spawnGoombasFromLevel() {
  // IMPORTANT: Do not use magic tile numbers for legacy compatibility.
  // Always use named TileID constants from Map_Build.h (e.g. TILE_ID_MOB_GOOMBA).
  // Reset pool
  for (int i = 0; i < MAX_GOOMBAS; ++i) {
    gGoombas[i].active = false;
    gGoombas[i].x = 0.0f;
    gGoombas[i].y = 0.0f;
    gGoombas[i].vx = 0.0f;
    gGoombas[i].vy = 0.0f;
    gGoombas[i].animFrame = 0;
    gGoombas[i].animTimer = 0;
    gGoombas[i].state = GOOMBA_STATE_DEAD;
    gGoombas[i].stateTimer = 0;
  }

  int spawned = 0;

  auto spawnAt = [&](int tx, int ty) {
    if (spawned >= MAX_GOOMBAS) return;

    // find free slot
    int slot = -1;
    for (int i = 0; i < MAX_GOOMBAS; ++i) {
      if (!gGoombas[i].active) { slot = i; break; }
    }
    if (slot < 0) return; // no free slot

    float worldX = tx * TILE_SIZE + (TILE_SIZE - GOOMBA_W) / 2.0f;
    float worldY = ty * TILE_SIZE - GOOMBA_H; // start on top of tile

    // Snap to nearest ground under that X
    float groundY = computeGoombaGroundY(worldX, worldY);
    worldY = groundY;

    gGoombas[slot].active = true;
    gGoombas[slot].x = worldX;
    gGoombas[slot].y = worldY;
    gGoombas[slot].vy = 0.0f;
    // Always start moving left (deterministic)
    gGoombas[slot].vx = -GOOMBA_SPEED;
    gGoombas[slot].animFrame = 0;
    gGoombas[slot].animTimer = 0;
    gGoombas[slot].state = GOOMBA_STATE_ALIVE;
    gGoombas[slot].stateTimer = 0;

    spawned++;
  };

  // 1) Preferred: spawn from Map_Mobs overlay (mobLayer[][])
  for (int ty = 0; ty < LEVEL_HEIGHT && spawned < MAX_GOOMBAS; ++ty) {
    for (int tx = 0; tx < LEVEL_WIDTH && spawned < MAX_GOOMBAS; ++tx) {
      uint8_t id = getMobId(tx, ty);
      if (id != TILE_ID_MOB_GOOMBA) continue;
      spawnAt(tx, ty);
      // Remove the marker from the overlay so the entity is authoritative.
      setMobId(tx, ty, 0);
    }
  }

  // 2) Back-compat: spawn from the main tile layer (level[][])
  for (int ty = 0; ty < LEVEL_HEIGHT && spawned < MAX_GOOMBAS; ++ty) {
    for (int tx = 0; tx < LEVEL_WIDTH && spawned < MAX_GOOMBAS; ++tx) {
      uint8_t id = getTileId(tx, ty);
      if (id != TILE_ID_MOB_GOOMBA) continue;
      spawnAt(tx, ty);
      setTileId(tx, ty, TILE_ID_EMPTY);
    }
  }
}

// Scan the runtime level[] and spawn Koopa entities for any painted
// Koopa tiles found, up to MAX_KOOPAS. Each spawned Koopa replaces
// its tile with TILE_ID_EMPTY so the entity is authoritative.
void spawnKoopasFromLevel() {
  // Reset pool
  for (int i = 0; i < MAX_KOOPAS; ++i) {
    gKoopas[i].active = false;
    gKoopas[i].x = 0.0f;
    gKoopas[i].y = 0.0f;
    gKoopas[i].vx = 0.0f;
    gKoopas[i].vy = 0.0f;
    gKoopas[i].dir = 1;
    gKoopas[i].facing = 1;
    gKoopas[i].animFrame = 0;
    gKoopas[i].lastAnimMs = 0;
    gKoopas[i].lastX = 0.0f;
    gKoopas[i].stuckTicks = 0;
  }

  int spawned = 0;

  auto spawnAt = [&](int tx, int ty) {
    if (spawned >= MAX_KOOPAS) return;

    int slot = -1;
    for (int i = 0; i < MAX_KOOPAS; ++i) {
      if (!gKoopas[i].active) { slot = i; break; }
    }
    if (slot < 0) return;

    float worldX = tx * TILE_SIZE + (TILE_SIZE - KOOPA_W) / 2.0f;
    float worldY = ty * TILE_SIZE - KOOPA_H;

    float groundY = computeKoopaGroundY(worldX, worldY);
    worldY = groundY;

    gKoopas[slot].active = true;
    gKoopas[slot].x = worldX;
    gKoopas[slot].y = worldY;
    gKoopas[slot].vy = 0.0f;
    // Always start moving left (deterministic)
    gKoopas[slot].vx = -KOOPA_SPEED;
    gKoopas[slot].dir = -1;
    gKoopas[slot].facing = -1;
    gKoopas[slot].animFrame = 0;
    gKoopas[slot].lastAnimMs = gameplayNowMs();
    gKoopas[slot].lastX = gKoopas[slot].x;
    gKoopas[slot].stuckTicks = 0;

    spawned++;
  };

  // 1) Preferred: spawn from Map_Mobs overlay (mobLayer[][])
  for (int ty = 0; ty < LEVEL_HEIGHT && spawned < MAX_KOOPAS; ++ty) {
    for (int tx = 0; tx < LEVEL_WIDTH && spawned < MAX_KOOPAS; ++tx) {
      uint8_t id = getMobId(tx, ty);
      if (id != TILE_ID_MOB_KOOPA_TROOPA) continue;
      spawnAt(tx, ty);
      setMobId(tx, ty, 0);
    }
  }

  // 2) Back-compat: spawn from the main tile layer (level[][])
  for (int ty = 0; ty < LEVEL_HEIGHT && spawned < MAX_KOOPAS; ++ty) {
    for (int tx = 0; tx < LEVEL_WIDTH && spawned < MAX_KOOPAS; ++tx) {
      uint8_t id = getTileId(tx, ty);
      if (id != TILE_ID_MOB_KOOPA_TROOPA) continue;
      spawnAt(tx, ty);
      setTileId(tx, ty, TILE_ID_EMPTY);
    }
  }
}

void updateKoopas() {
  if (gFlagpole.active && gFlagpole.freezeEnemies) return;

  const float HITBOX_INSET_X_K = 1.0f;
  const uint32_t nowMs = gameplayNowMs();

  for (int i = 0; i < MAX_KOOPAS; ++i) {
    Koopa &kk = gKoopas[i];
    if (!kk.active) continue;

    const float prevX = kk.x;

    // Gravity
    kk.vy += KOOPA_GRAVITY;
    if (kk.vy > KOOPA_MAX_FALL) kk.vy = KOOPA_MAX_FALL;

    float nextXk = kk.x + kk.vx;

    int dirNow = kk.dir;
    if (kk.vx < -0.001f) dirNow = -1;
    else if (kk.vx > 0.001f) dirNow = 1;

    // Turn around before stepping into lava
    if (mobHazardAhead(nextXk, kk.y, KOOPA_W, KOOPA_H, dirNow)) {
      kk.vx = -kk.vx;
      kk.dir = (kk.vx < 0.0f) ? -1 : 1;
      kk.facing = kk.dir;
      nextXk = kk.x;
      dirNow = kk.dir;
    }

    // Side walls must always reverse direction.
    // Do NOT allow the generic "step-up" tolerance to bypass a real side collision.
    if (koopaSideBlocked(nextXk, kk.y, dirNow)) {
      kk.vx = -kk.vx;
      kk.dir = (kk.vx < 0.0f) ? -1 : 1;
      kk.facing = kk.dir;
      nextXk = kk.x;
      dirNow = kk.dir;
    } else {
      // Allow stepping onto gentle rises (slopes / tiny ledges)
      float testGround = computeKoopaGroundYMultiProbe(nextXk, kk.y, dirNow);
      if (fabsf(testGround - kk.y) <= KOOPA_MAX_STEP_UP + 0.01f) {
        // OK — do NOT block
      }
    }

    // Clamp Koopa inside world bounds (no walking off the map)
    if (nextXk < 0.0f) {
      nextXk = 0.0f;
      kk.vx = fabsf(kk.vx);   // force right
      kk.dir = 1;
      kk.facing = 1;
      dirNow = 1;
    } else if (nextXk > (float)WORLD_WIDTH - KOOPA_W) {
      nextXk = (float)WORLD_WIDTH - KOOPA_W;
      kk.vx = -fabsf(kk.vx);  // force left
      kk.dir = -1;
      kk.facing = -1;
      dirNow = -1;
    }

    // Apply X
    kk.x = nextXk;

    // Snap Koopa cleanly to slope surface after moving
    float groundAfterX = computeKoopaGroundYMultiProbe(kk.x, kk.y, dirNow);
    if (kk.y > groundAfterX) {
      kk.y = groundAfterX;
      kk.vy = 0.0f;
    }

    // Vertical / ground snap
    float nextYk = kk.y + kk.vy;
    float groundYk = computeKoopaGroundYMultiProbe(kk.x, nextYk, dirNow);
    if (nextYk >= groundYk) {
      kk.y = groundYk;
      kk.vy = 0.0f;
      if (kk.vx == 0.0f) {
        // Always restart moving left (deterministic)
        kk.vx = -KOOPA_SPEED;
        kk.dir = -1;
        kk.facing = -1;
      }
    } else {
      kk.y = nextYk;
    }

    // Lock Koopa facing to actual movement direction (prevents spinning)
    if (kk.vx < -0.001f) {
      kk.facing = -1;
      kk.dir = -1;
    } else if (kk.vx > 0.001f) {
      kk.facing = 1;
      kk.dir = 1;
    }

    // ─────────────────────────────────────────────
    // Stuck detection: if Koopa isn't making horizontal progress while grounded,
    // flip direction and try moving away.
    // ─────────────────────────────────────────────
    const float movedX = fabsf(kk.x - prevX);
    bool groundedNow = (fabsf(kk.y - computeKoopaGroundY(kk.x, kk.y)) < 0.5f);

    if (groundedNow && fabsf(kk.vx) > 0.05f && movedX < 0.01f) {
      if (kk.stuckTicks < 255) kk.stuckTicks++;
    } else {
      kk.stuckTicks = 0;
    }

    if (kk.stuckTicks >= 8) {
      kk.stuckTicks = 0;

      // Reverse direction
      kk.vx = -kk.vx;
      kk.dir = (kk.vx < 0.0f) ? -1 : 1;
      kk.facing = kk.dir;

      // Nudge 1px to help escape slope/edge snag
      kk.x += (kk.dir < 0) ? -1.0f : 1.0f;

      // Clamp safety (don’t escape map)
      if (kk.x < 0.0f) kk.x = 0.0f;
      if (kk.x > (float)WORLD_WIDTH - KOOPA_W) kk.x = (float)WORLD_WIDTH - KOOPA_W;
    }

    // 2-frame walk animation (every ~120ms), with bobbing handled at draw-time
    bool groundedAnim = (fabsf(kk.y - computeKoopaGroundYMultiProbe(kk.x, kk.y, kk.dir)) < 0.5f);
    if (groundedAnim && fabsf(kk.vx) > 0.05f) {
      if (nowMs - kk.lastAnimMs >= 120UL) {
        kk.animFrame ^= 1;
        kk.lastAnimMs = nowMs;
      }
    } else {
      kk.animFrame = 0;
      kk.lastAnimMs = nowMs;
    }
  }
}

// ─────────────────────────────────────────────
// Hero size change
// ─────────────────────────────────────────────
void setHeroSize(HeroSize newSize) {
  if (heroSize == newSize) return;

  float feetY = heroY + heroHitH;

  heroSize = newSize;
  if (heroSize == SIZE_SMALL) {
    heroHitH = HERO_HEIGHT_SMALL;
  } else {
    heroHitH = HERO_HEIGHT_BIG;
  }

  heroY = feetY - heroHitH;
}

// ─────────────────────────────────────────────
// Block hit handling
// ─────────────────────────────────────────────
void startBlockBounce(int tx, int ty) {
  if (tx < 0 || tx >= LEVEL_WIDTH || ty < 0 || ty >= LEVEL_HEIGHT) return;
  tileAnimTimer[ty][tx] = 6;
}

void handleBlockHitFromBelow(int tx, int ty) {
  // TileID-specific handling for coin-brick (SMW-style: 1 coin per hit)
  uint8_t id = getTileId(tx, ty);
  if (id == TILE_ID_BRICK_COIN) {
    if (coinBrickRemaining[ty][tx] == 0) {
      coinBrickRemaining[ty][tx] = demoTimingActive()
        ? coinBrickInitialRemaining(tx, ty)
        : (uint8_t)random(1, 6); // 1..5
    }

    if (coinBrickRemaining[ty][tx] > 0) {
      coinBrickRemaining[ty][tx]--;
      coinCount++; // one per hit
      spawnCoinPopSMW(tx, ty, -1); // pop up
      startBlockBounce(tx, ty);

      if (coinBrickRemaining[ty][tx] == 0) {
        setTileId(tx, ty, TILE_ID_USED_BLOCK);
      }
    }
    return;
  }

  TileType t = getTile(tx, ty);
  switch (t) {
    case TILE_Q_MUSHROOM:
      spawnMushroomFromBelow(tx, ty, ITEM_RED_MUSHROOM);
      setTileId(tx, ty, tileTypeToTileId(TILE_USED));
      startBlockBounce(tx, ty);
      break;
    case TILE_Q_GREEN_MUSHROOM:
      spawnMushroomFromBelow(tx, ty, ITEM_GREEN_MUSHROOM);
      setTileId(tx, ty, tileTypeToTileId(TILE_USED));
      startBlockBounce(tx, ty);
      break;

    case TILE_SOLID_SHAKE:
      startBlockBounce(tx, ty);
      break;

    case TILE_BRICK_BREAK:
      if (heroSize == SIZE_BIG) {
        setTileId(tx, ty, TILE_ID_EMPTY);
        // spawn visual fragments at the broken tile
        spawnBrickFragmentsAt(tx, ty);
        tileAnimTimer[ty][tx] = 0;
        tileOffset[ty][tx]    = 0;
      } else {
        startBlockBounce(tx, ty);
      }
      break;

    case TILE_USED:
      startBlockBounce(tx, ty);
      break;

    default:
      break;
  }
}

void handleBlockHitFromAbove(int tx, int ty) {
  // TileID-specific handling for coin-brick (SMW-style: 1 coin per hit)
  uint8_t id = getTileId(tx, ty);
  if (id == TILE_ID_BRICK_COIN) {
    if (coinBrickRemaining[ty][tx] == 0) {
      coinBrickRemaining[ty][tx] = demoTimingActive()
        ? coinBrickInitialRemaining(tx, ty)
        : (uint8_t)random(1, 6);
    }

    if (coinBrickRemaining[ty][tx] > 0) {
      coinBrickRemaining[ty][tx]--;
      coinCount++; // one per hit
      spawnCoinPopSMW(tx, ty, +1); // pop down
      startBlockBounce(tx, ty);

      if (coinBrickRemaining[ty][tx] == 0) {
        setTileId(tx, ty, TILE_ID_USED_BLOCK);
      }
    }
    return;
  }

  TileType t = getTile(tx, ty);
  switch (t) {
    case TILE_Q_MUSHROOM:
      spawnMushroomFromAbove(tx, ty, ITEM_RED_MUSHROOM);
      setTileId(tx, ty, tileTypeToTileId(TILE_USED));
      startBlockBounce(tx, ty);
      break;
    case TILE_Q_GREEN_MUSHROOM:
      spawnMushroomFromAbove(tx, ty, ITEM_GREEN_MUSHROOM);
      setTileId(tx, ty, tileTypeToTileId(TILE_USED));
      startBlockBounce(tx, ty);
      break;

    case TILE_SOLID_SHAKE:
      startBlockBounce(tx, ty);
      break;

    case TILE_BRICK_BREAK:
      if (heroSize == SIZE_BIG && slamActive) {
        setTileId(tx, ty, TILE_ID_EMPTY);
        // spawn visual fragments at the broken tile
        spawnBrickFragmentsAt(tx, ty);
        // mark that we just removed a tile via slam so physics can avoid snapping
        gJustBrokeTileThisFrame = true;
        tileAnimTimer[ty][tx] = 0;
        tileOffset[ty][tx]    = 0;
      } else {
        startBlockBounce(tx, ty);
      }
      break;

    case TILE_USED:
      startBlockBounce(tx, ty);
      break;

    default:
      break;
  }
}

// ─────────────────────────────────────────────
// Reset world / map / hero / mushroom
// ─────────────────────────────────────────────
void doHardReset() {
  // Game-over / hard reset always returns to Map 1.
  gCurrentMapId = MAP_ID_OVERWORLD_1;
  gMarioLives = 5;       // reset lives to 5
  resetWorld();          // this already resets map, coins, and gMapTimeSeconds = 300
}

void resetWorld(bool clearTransitionsAndCutscenes /* = true */) {
  Serial.println("[DEBUG] Resetting world (reload current map)...");

  // Reset coins when the world resets
  coinCount = 0;

  // Reset map timer for new life/map
  gMapTimeSeconds = 300;

  // Reset hero size
  heroSize = SIZE_SMALL;
  heroHitH = HERO_HEIGHT_SMALL;

  // Reset growth animation
  isGrowing        = false;
  growStep         = 0;
  growFrameCounter = 0;

  // Reset mushroom
  gMushroom.active  = false;
  gMushroom.rising  = true;
  gMushroom.x       = 0;
  gMushroom.y       = 0;
  gMushroom.vx      = 0;
  gMushroom.vy      = 0;
  gMushroom.targetY = 0;

  // Full reload Map_Build + Map_Mobs into runtime arrays + spawn + camera
  loadTestMap(clearTransitionsAndCutscenes);

  // IMPORTANT:
  // During the flagpole finish cutscene we fade-out then fade-in.
  // If we clear the flagpole/pipe states mid-fade, fade-in breaks.
  if (clearTransitionsAndCutscenes) {
    // Clear any active flagpole sequence
    resetFlagpoleState();

    // Clear any active pipe transition state
    resetPipeTransitionState();
    Serial.println("[PIPE] World reset, clearing pipe transition state.");
  }
}

// Load the test map from Map_Build.h (Map_Build array)
void loadTestMap(bool clearTransitionsAndCutscenes /* = true */) {
  Serial.println("[DEBUG] Loading test map from Map_Build...");

  // Select the currently configured built-in map.
  // NOTE: Log string kept as-is to avoid UI/log churn.
  const uint8_t (*mapTiles)[MAP_TILES_W] = nullptr;
  const uint8_t (*mapMobs)[MAP_TILES_W]  = nullptr;
  uint16_t mapW = 0;
  uint16_t mapH = 0;
  getCurrentMapSelection(&mapTiles, &mapMobs, &mapW, &mapH);

  // load Map_Build into runtime level[] (TileIDs). Do NOT enable tile map mode;
  // runtime must use `level[][]` as the single source of truth.

  // Clear the level array (remove all interactive tiles from original world)
  for (int ty = 0; ty < LEVEL_HEIGHT; ty++) {
    for (int tx = 0; tx < LEVEL_WIDTH; tx++) {
      level[ty][tx] = TILE_ID_EMPTY;
      mobLayer[ty][tx] = 0;
      tileOffset[ty][tx] = 0;
      tileAnimTimer[ty][tx] = 0;
      coinBrickRemaining[ty][tx] = 0;
    }
  }
  Serial.println("[DEBUG] Cleared level[] array for test map");

  // Phase 1: Populate level[][] from Map_Build for physics
  // Loop through the smaller of the two dimensions to avoid out-of-bounds
  int maxY = (LEVEL_HEIGHT < (int)mapH) ? LEVEL_HEIGHT : (int)mapH;
  int maxX = (LEVEL_WIDTH  < (int)mapW) ? LEVEL_WIDTH  : (int)mapW;
  
  for (int ty = 0; ty < maxY; ty++) {
    for (int tx = 0; tx < maxX; tx++) {
      // Read tile ID from Map_Build (PROGMEM)
      uint8_t tileId = pgm_read_byte(&mapTiles[ty][tx]);
      // Store TileID directly in runtime level (authoritative)
      level[ty][tx] = tileId;

      // Read overlay mob ID from Map_Mobs (PROGMEM)
      mobLayer[ty][tx] = pgm_read_byte(&mapMobs[ty][tx]);
    }
  }
  Serial.println("[DEBUG] Populated level[] from Map_Build into runtime level[] (TileIDs)");

  // Ensure runtime uses the populated `level[][]` as authoritative world state
  gUsingTileMap = false;
  Serial.print("[DEBUG] gUsingTileMap set to: ");
  Serial.println(gUsingTileMap);

  // On every full map load/reset: clear checkpoint state.
  checkpointActive = false;
  checkpointTileX = -1;
  checkpointTileY = -1;
  checkpointFlipAnimating = false;
  checkpointFlipTileX = -1;
  checkpointFlipTileY = -1;
  checkpointFlipStartMs = 0;

  gCheckpointAnim.active = false;
  gCheckpointAnim.tileX = -1;
  gCheckpointAnim.tileY = -1;
  gCheckpointAnim.startTileId = 0;
  gCheckpointAnim.endTileId = 0;
  gCheckpointAnim.startMs = 0;

  checkpointTouchingLastFrame = false;
  checkpointTouchLastTx = -1;
  checkpointTouchLastTy = -1;

  // Clear any active flagpole sequence (optional)
  if (clearTransitionsAndCutscenes) {
    resetFlagpoleState();
  }

  // Scan the loaded map once for the (single) flagpole and initialize cloth runtime
  initFlagPoleRuntimeFromLevel();

  // Spawn Mario from the Map_Mobs marker (MOB_MARIO_SPAWN = TILE_ID_MOB_MARIO_SPAWN)
  applyMarioSpawnFromMobLayerOrFallback();
  snapCameraToHeroSpawnAndClamp();

  // Build tunnel pairs from Map_Build (pipes)
  buildTunnelPairsFromMap();
  
  // Debug: dump built tunnel pairs (TileID + coords) after map load.
  Serial.println("[PIPE] Pair table dump:");
  for (int i = 0; i < gTunnelPairCount; ++i) {
    const TunnelPair &tp = gTunnelPairs[i];
    Serial.print("[PIPE] Pair ");
    Serial.print(i);
    Serial.print(" entryId=");
    Serial.print(tp.entry.tileId);
    Serial.print(" (");
    Serial.print(tp.entry.tileX);
    Serial.print(",");
    Serial.print(tp.entry.tileY);
    Serial.print(")");

    Serial.print(" exitId=");
    Serial.print(tp.exit.tileId);
    Serial.print(" (");
    Serial.print(tp.exit.tileX);
    Serial.print(",");
    Serial.print(tp.exit.tileY);
    Serial.print(")");

    Serial.print(" hasLeft=");
    Serial.print(tp.hasLeftEntry ? 1 : 0);
    if (tp.hasLeftEntry) {
      Serial.print(" leftId=");
      Serial.print(tp.leftEntry.tileId);
      Serial.print(" (");
      Serial.print(tp.leftEntry.tileX);
      Serial.print(",");
      Serial.print(tp.leftEntry.tileY);
      Serial.print(")");
    }
    Serial.println();
  }

  // Spawn any Goombas painted into the test map
  spawnGoombasFromLevel();
  // Spawn any Koopas painted into the test map
  spawnKoopasFromLevel();

  // Reset shell entities (spawned by stomping Koopas)
  resetShells();

  // Spawn Piranha Plants from the mob overlay layer
  spawnPiranhasFromMobLayer();

  // Note: We keep heroSize, coinCount, lives, and map timer as-is
  // so the test map acts as a "warp" rather than a full reset.
}

// ─────────────────────────────────────────────
// Flag pole cloth-only runtime (cap stays static)
// ─────────────────────────────────────────────
static void initFlagPoleRuntimeFromLevel() {
  gFlagPole = FlagPoleRuntime();

  for (int ty = 0; ty < LEVEL_HEIGHT; ++ty) {
    for (int tx = 0; tx < LEVEL_WIDTH; ++tx) {
      if (getTileId(tx, ty) != TILE_ID_FLAG_TOP) continue;

      // Find base below (stop if we hit a non-flag, non-empty tile)
      int baseTy = ty;
      for (int y = ty; y < LEVEL_HEIGHT; ++y) {
        uint8_t id = getTileId(tx, y);
        if (id == TILE_ID_FLAG_BASE) { baseTy = y; break; }
        if (id != TILE_ID_EMPTY && !isFlagpoleTileId(id)) break;
      }

      gFlagPole.active = true;
      gFlagPole.tileX = (int16_t)tx;
      gFlagPole.topTy = (int16_t)ty;
      gFlagPole.baseTy = (int16_t)baseTy;

      int firstPoleTy = -1;
      int lastPoleTy = -1;
      (void)findPoleSegmentsInMap(tx, ty, baseTy, firstPoleTy, lastPoleTy);
      gFlagPole.firstPoleTy = (int16_t)firstPoleTy;
      gFlagPole.lastPoleTy  = (int16_t)lastPoleTy;

      const int TS = 8;
      int ox = 0, oy = 0;
      getFlagTileOffset(TILE_ID_FLAG_TOP, ox, oy);

      // Match the static top-sprite anchor used by drawFlagTopOverlay.
      int worldY = ty * TS;
      int spriteY = worldY - (FLAG_SRC_H - TS) + oy;

      // Cloth sub-rect starts at FLAG_ONLY_SRC_Y within the 14x15 sprite.
      gFlagPole.minFlagWorldY = (float)(spriteY + FLAG_ONLY_SRC_Y);

      // Clamp cloth to pole length (based on last pole segment).
      int poleRefTy = (lastPoleTy >= 0) ? lastPoleTy : baseTy;
      int lastPoleBottomWorldY = (poleRefTy + 1) * TS;
      int maxByLastPole = lastPoleBottomWorldY - FLAG_ONLY_H;
      gFlagPole.maxFlagWorldY = (float)maxByLastPole;

      if (gFlagPole.maxFlagWorldY < gFlagPole.minFlagWorldY) {
        gFlagPole.maxFlagWorldY = gFlagPole.minFlagWorldY;
      }

      gFlagPole.flagWorldY = gFlagPole.minFlagWorldY;
      gFlagPole.flagDir = 1;
      gFlagPole.lastIdleStepMs = gameplayNowMs();
      return; // one pole is enough
    }
  }
}

static bool findPoleSegmentsInMap(int tx, int topTy, int baseTy, int &outFirstPoleTy, int &outLastPoleTy) {
  outFirstPoleTy = -1;
  outLastPoleTy  = -1;

  if (tx < 0 || tx >= LEVEL_WIDTH) return false;
  if (topTy < 0) return false;
  if (baseTy >= LEVEL_HEIGHT) return false;

  for (int y = topTy + 1; y < baseTy; y++) {
    uint8_t id = getTileId(tx, y);
    if (id == TILE_ID_FLAG_POLE) {
      if (outFirstPoleTy < 0) outFirstPoleTy = y;
      outLastPoleTy = y;
    }
  }

  // We expect at least 1 pole segment placed; if none, fail cleanly
  return (outFirstPoleTy >= 0 && outLastPoleTy >= 0);
}

static void updateFlagPoleCloth() {
  if (!gFlagPole.active) return;

  // Slide: flag falls down the pole until it hits the base-safe clamp.
  if (gFlagPole.sliding) {
    gFlagPole.flagWorldY += 1.5f;
    if (gFlagPole.flagWorldY >= gFlagPole.maxFlagWorldY) {
      gFlagPole.flagWorldY = gFlagPole.maxFlagWorldY;
      gFlagPole.sliding = false;
    }
    return;
  }

  // Idle bob: move 1px every N ms within clamp range.
  if (!gFlagPole.grabbed) {
    uint32_t now = gameplayNowMs();
    const uint32_t STEP_MS = 160;
    if (now - gFlagPole.lastIdleStepMs >= STEP_MS) {
      gFlagPole.lastIdleStepMs = now;
      gFlagPole.flagWorldY += (float)gFlagPole.flagDir * 1.0f;

      if (gFlagPole.flagWorldY <= gFlagPole.minFlagWorldY) {
        gFlagPole.flagWorldY = gFlagPole.minFlagWorldY;
        gFlagPole.flagDir = 1;
      } else if (gFlagPole.flagWorldY >= gFlagPole.maxFlagWorldY) {
        gFlagPole.flagWorldY = gFlagPole.maxFlagWorldY;
        gFlagPole.flagDir = -1;
      }
    }
  }
}

static void drawFlagPoleClothOverlay() {
  if (!gFlagPole.active) return;

  int ox = 0, oy = 0;
  getFlagTileOffset(TILE_ID_FLAG_TOP, ox, oy);

  const int TS = 8;

  int worldX = (int)gFlagPole.tileX * TS;
  int px = worldX - (int)cameraX + ox + FLAG_ONLY_SRC_X;
  int py = (int)gFlagPole.flagWorldY - (int)cameraY;

  // Draw ONLY the flag cloth sub-rect (no pole pixels)
  drawWorldTileCustomSizeSubrect((const uint8_t*)&Ground_Tile_Flag[0][0],
                                 FLAG_SRC_W,
                                 FLAG_SRC_H,
                                 px,
                                 py,
                                 FLAG_ONLY_SRC_X,
                                 FLAG_ONLY_SRC_Y,
                                 FLAG_ONLY_W,
                                 FLAG_ONLY_H);
}

// Draw the static gold cap/pole-head pixels on top of the world tile pass
// to avoid clipping/order issues. This MUST remain static.
static void drawFlagTopOverlay() {
  if (!gFlagPole.active) return;

  const int TS = 8;
  int ox = 0, oy = 0;
  getFlagTileOffset(TILE_ID_FLAG_TOP, ox, oy);

  int worldX = (int)gFlagPole.tileX * TS;
  int worldY = (int)gFlagPole.topTy * TS;

  // 14x15 top sprite is bottom-anchored to the 8x8 cell
  int spriteY = worldY - (FLAG_SRC_H - TS) + oy;

  int px = worldX - (int)cameraX + ox;
  int py = spriteY - (int)cameraY;

  // Draw the TOP sprite in pieces, skipping the cloth rectangle.
  // This removes the static duplicate flag; the animated overlay draws the cloth.

  // 1) Left side (cap + pole) up to FLAG_ONLY_SRC_X
  if (FLAG_ONLY_SRC_X > 0) {
    drawWorldTileCustomSizeSubrect(
      (const uint8_t*)&Ground_Tile_Flag[0][0],
      FLAG_SRC_W, FLAG_SRC_H,
      px, py,
      0, 0,
      FLAG_ONLY_SRC_X, FLAG_SRC_H
    );
  }

  // 2) Right side ABOVE the cloth (cap rows on the right)
  if (FLAG_ONLY_SRC_Y > 0) {
    drawWorldTileCustomSizeSubrect(
      (const uint8_t*)&Ground_Tile_Flag[0][0],
      FLAG_SRC_W, FLAG_SRC_H,
      px + FLAG_ONLY_SRC_X, py,
      FLAG_ONLY_SRC_X, 0,
      (FLAG_SRC_W - FLAG_ONLY_SRC_X), FLAG_ONLY_SRC_Y
    );
  }

  // 3) Right side BELOW the cloth (if any)
  int clothEndY = FLAG_ONLY_SRC_Y + FLAG_ONLY_H;
  if (clothEndY < FLAG_SRC_H) {
    drawWorldTileCustomSizeSubrect(
      (const uint8_t*)&Ground_Tile_Flag[0][0],
      FLAG_SRC_W, FLAG_SRC_H,
      px + FLAG_ONLY_SRC_X, py + clothEndY,
      FLAG_ONLY_SRC_X, clothEndY,
      (FLAG_SRC_W - FLAG_ONLY_SRC_X), (FLAG_SRC_H - clothEndY)
    );
  }
}

// ─────────────────────────────────────────────
// Drawing helpers (unchanged from your code)
// ─────────────────────────────────────────────
void drawSpriteFrame(const uint8_t frame[18][14], int16_t x0, int16_t y0) {
  if (!matrix) return;

  for (int y = 0; y < SPRITE_H; y++) {
    for (int x = 0; x < SPRITE_W; x++) {
      uint8_t idx = pgm_read_byte(&frame[y][x]);
      if (idx == 0) continue;

      RGB c = heroPalette[idx];
      uint16_t color = matrix->color565(c.r, c.g, c.b);

      int16_t sx0 = x0 + x * SPRITE_SCALE;
      int16_t sy0 = y0 + y * SPRITE_SCALE;

      for (int dy = 0; dy < SPRITE_SCALE; dy++) {
        int16_t sy = sy0 + dy;
        if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

        for (int dx = 0; dx < SPRITE_SCALE; dx++) {
          int16_t sx = sx0 + dx;
          if (sx < 0 || sx >= MATRIX_WIDTH) continue;
          matrix->drawPixel(sx, sy, color);
        }
      }
    }
  }
}

void drawHero(int16_t screenX, int16_t screenY, int bobOffset) {
  // Hide Mario while inside the pipe
  if (!gHeroVisibleDuringPipe) {
    return;
  }

  // Blink while invincible
  if (millis() < gHeroInvincUntilMs) {
    if (((millis() / 100) & 1) == 0) return; // skip draw to blink
  }

  const uint8_t (*frame)[14];

  if (heroSize == SIZE_BIG) {
    switch (currentPose) {
      case POSE_WALK1:  frame = heroWalk1;  break;
      case POSE_WALK2:  frame = heroWalk2;  break;
      case POSE_JUMP:   frame = heroJump;   break;
      case POSE_SPIN:   frame = heroSpin;   break;
      case POSE_CROUCH: frame = heroCrouch; break;
      case POSE_SLIDE:  frame = heroCrouch; break;  // Slide uses crouch sprite
      case POSE_STAND:
      default:          frame = heroStand;  break;
    }
  } else {
    // Small Mario: everything uses crouch, spin uses heroSpin
    switch (currentPose) {
      case POSE_SPIN:
        frame = heroSpin;
        break;
      case POSE_CROUCH:
      case POSE_JUMP:
      case POSE_WALK1:
      case POSE_WALK2:
      case POSE_STAND:
      default:
        frame = heroCrouch;
        break;
    }
  }

  // Holding-shell override (BIG + SMALL)
  if (gHeroHoldingShell) {
    if (heroSize == SIZE_BIG) {
      frame = heroStand_Holding_Shell_Frame;
    } else {
      frame = heroSmall_Holding_Shell_Frame;
    }
  }

  int footOffset = (heroSize == SIZE_BIG) ? FOOT_OFFSET_BIG : FOOT_OFFSET_SMALL;
  if (heroSize == SIZE_SMALL) {
    footOffset -= 2;
  }

  int16_t drawY = (int16_t)(screenY - bobOffset + footOffset);
  // Holding shell sprites sit 1px low; lift hero up 1px while holding
  if (gHeroHoldingShell) drawY -= 1;
  drawSpriteFrame(frame, screenX, drawY);
}

// Simple blue sky
void drawBackground() {
  uint16_t sky = matrix->color565(0, 32, 80);
  matrix->fillScreen(sky);
}

// ─────────────────────────────────────────────
// Coin rendering helper (round-ish 8x8 coin)
// ─────────────────────────────────────────────
void drawRoundCoin8x8(int px, int py) {
  if (!matrix) return;

  uint16_t gold   = matrix->color565(252, 216, 96);
  uint16_t shadow = matrix->color565(200, 160, 40);

  // Center of the 8x8 area
  const float cx = px + 3.5f;
  const float cy = py + 3.5f;
  const float rOuter2 = 3.5f * 3.5f;  // outer radius squared
  const float rInner2 = 2.3f * 2.3f;  // inner radius squared

  for (int y = 0; y < 8; y++) {
    int sy = py + y;
    if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

    for (int x = 0; x < 8; x++) {
      int sx = px + x;
      if (sx < 0 || sx >= MATRIX_WIDTH) continue;

      float dx = (sx + 0.5f) - cx;
      float dy = (sy + 0.5f) - cy;
      float d2 = dx*dx + dy*dy;

      if (d2 <= rOuter2) {
        bool edge = (d2 > rInner2);
        uint16_t col = edge ? shadow : gold;
        matrix->drawPixel(sx, sy, col);
      }
    }
  }
}

// ─────────────────────────────────────────────
// Helper: Draw a tile sprite from WorldSprites.h
// ─────────────────────────────────────────────
// Draws one tile sprite at world position (worldX, worldY) using heroPalette colors.
// For smaller sprites (like coins), centers them within the 8×8 tile space.
void drawWorldTileSprite(TileType t, int worldX, int worldY) {
  // Look up the sprite data
  WorldTileSprite sprite;
  memcpy_P(&sprite, &kWorldTileSprites[(int)t], sizeof(WorldTileSprite));
  
  if (sprite.data == nullptr || sprite.width == 0 || sprite.height == 0) {
    return; // No sprite for this tile type
  }

  // Convert world position to screen position
  int screenX = worldX - (int)cameraX;
  int screenY = worldY - (int)cameraY;

  // Center smaller sprites within 8×8 tile (e.g., 6×5 coin becomes +1,+1 offset)
  int offsetX = (8 - sprite.width) / 2;
  int offsetY = (8 - sprite.height) / 2;
  screenX += offsetX;
  screenY += offsetY;

  // Early clip check
  if (screenX + sprite.width <= 0 || screenX >= MATRIX_WIDTH) return;
  if (screenY + sprite.height <= 0 || screenY >= MATRIX_HEIGHT) return;

  // Draw sprite using heroPalette
  for (int y = 0; y < sprite.height; y++) {
    int sy = screenY + y;
    if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

    for (int x = 0; x < sprite.width; x++) {
      int sx = screenX + x;
      if (sx < 0 || sx >= MATRIX_WIDTH) continue;

      // Read palette index from PROGMEM
      // sprite.data points to [height][width] array
      uint8_t idx = pgm_read_byte(sprite.data + y * sprite.width + x);
      
      // Skip transparent pixels (index 0)
      if (idx == 0) continue;

      // Look up color in heroPalette
      RGB color;
      memcpy_P(&color, &heroPalette[idx], sizeof(RGB));
      matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
    }
  }
}

// Draw an 8x8 world tile from a PROGMEM sprite using heroPalette indices.
// v == 0 is treated as transparent.
static void drawWorldTile8x8(const uint8_t sprite[8][8], int px, int py) {
  if (!matrix) return;

  for (int y = 0; y < 8; ++y) {
    int sy = py + y;
    if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

    for (int x = 0; x < 8; ++x) {
      int sx = px + x;
      if (sx < 0 || sx >= MATRIX_WIDTH) continue;

      uint8_t v = pgm_read_byte(&sprite[y][x]);
      if (v == 0) continue;  // 0 = transparent

      RGB color;
      memcpy_P(&color, &heroPalette[v], sizeof(RGB));
      matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
    }
  }
}

// Draw a world tile with custom width (height always 8) using heroPalette indices.
static void drawWorldTileCustomWidth(const uint8_t* sprite, int width, int px, int py) {
  if (!matrix) return;

  for (int y = 0; y < 8; ++y) {
    int sy = py + y;
    if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

    for (int x = 0; x < width; ++x) {
      int sx = px + x;
      if (sx < 0 || sx >= MATRIX_WIDTH) continue;

      uint8_t v = pgm_read_byte(sprite + y * width + x);
      if (v == 0) continue;  // 0 = transparent

      RGB color;
      memcpy_P(&color, &heroPalette[v], sizeof(RGB));
      matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
    }
  }
}

// Draw a sprite with custom width and height
static void drawWorldTileCustomSize(const uint8_t* sprite, int width, int height, int px, int py) {
  if (!matrix) return;

  for (int y = 0; y < height; ++y) {
    int sy = py + y;
    if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

    for (int x = 0; x < width; ++x) {
      int sx = px + x;
      if (sx < 0 || sx >= MATRIX_WIDTH) continue;

      uint8_t v = pgm_read_byte(sprite + y * width + x);
      if (v == 0) continue;  // 0 = transparent

      RGB color;
      memcpy_P(&color, &heroPalette[v], sizeof(RGB));
      matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
    }
  }
}

// Draw a sprite with custom width and height, flipped horizontally.
static void drawWorldTileCustomSizeFlippedX(const uint8_t* sprite, int width, int height, int px, int py) {
  if (!matrix) return;

  for (int y = 0; y < height; ++y) {
    int sy = py + y;
    if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

    for (int x = 0; x < width; ++x) {
      int sx = px + x;
      if (sx < 0 || sx >= MATRIX_WIDTH) continue;

      int srcX = (width - 1) - x;
      uint8_t v = pgm_read_byte(sprite + y * width + srcX);
      if (v == 0) continue;  // 0 = transparent

      RGB color;
      memcpy_P(&color, &heroPalette[v], sizeof(RGB));
      matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
    }
  }
}

// Draw a sprite with custom width and height, but only draw the top `clipH` rows.
// This is used for the SMW-style piranha plant reveal so it appears to emerge from
// (and retract into) the pipe instead of floating in front of it.
static void drawWorldTileCustomSizeClippedHeight(const uint8_t* sprite, int width, int height, int px, int py, int clipH) {
  if (!matrix) return;
  if (clipH <= 0) return;
  if (clipH > height) clipH = height;

  for (int y = 0; y < clipH; ++y) {
    int sy = py + y;
    if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

    for (int x = 0; x < width; ++x) {
      int sx = px + x;
      if (sx < 0 || sx >= MATRIX_WIDTH) continue;

      uint8_t v = pgm_read_byte(sprite + y * width + x);
      if (v == 0) continue;

      RGB color;
      memcpy_P(&color, &heroPalette[v], sizeof(RGB));
      matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
    }
  }
}

// Draw a sub-rectangle from a PROGMEM sprite.
// Source and destination are in screen pixels.
static void drawWorldTileCustomSizeSubrect(const uint8_t* sprite,
                                           int width,
                                           int height,
                                           int px,
                                           int py,
                                           int srcX,
                                           int srcY,
                                           int subW,
                                           int subH) {
  if (!matrix) return;
  if (subW <= 0 || subH <= 0) return;

  if (srcX < 0) { px -= srcX; subW += srcX; srcX = 0; }
  if (srcY < 0) { py -= srcY; subH += srcY; srcY = 0; }
  if (srcX + subW > width)  subW = width - srcX;
  if (srcY + subH > height) subH = height - srcY;
  if (subW <= 0 || subH <= 0) return;

  for (int y = 0; y < subH; ++y) {
    int sy = py + y;
    if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

    int srcRow = (srcY + y) * width;
    for (int x = 0; x < subW; ++x) {
      int sx = px + x;
      if (sx < 0 || sx >= MATRIX_WIDTH) continue;

      uint8_t v = pgm_read_byte(sprite + srcRow + (srcX + x));
      if (v == 0) continue;

      RGB color;
      memcpy_P(&color, &heroPalette[v], sizeof(RGB));
      matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
    }
  }
}

// --- Render-time TileID -> sprite resolver (preserves exact visuals per TileID)
struct SpriteRef {
  const uint8_t* data;
  uint8_t width;
  uint8_t height;
};

// Ensure C-style linkage can refer to the type name
typedef struct SpriteRef SpriteRef;

static inline struct SpriteRef getSpriteForTileId(uint8_t tileId) {
  switch (tileId) {
    case TILE_ID_EMPTY: return { nullptr, 0, 0 };

    // 8×8 terrain
    case TILE_ID_GROUND: return { &Ground_Tile_Array[0][0], 8, 8 };
    case TILE_ID_GROUND_TOP_LEFT: return { &Ground_Tile_Top_Left[0][0], 8, 8 };
    case TILE_ID_GROUND_TOP_RIGHT: return { &Ground_Tile_Top_Right[0][0], 8, 8 };
    case TILE_ID_GROUND_WALL_LEFT: return { &Ground_Tile_Wall_Left[0][0], 8, 8 };
    case TILE_ID_GROUND_WALL_RIGHT: return { &Ground_Tile_Wall_Right[0][0], 8, 8 };
    case TILE_ID_GROUND_CENTER_DIRT: return { &Ground_Tile_Center_Dirt[0][0], 8, 8 };

    case TILE_ID_STONE_LEDGE: return { &Ground_Tile_Stone_Ledge[0][0], 8, 8 };
    case TILE_ID_UNDERGROUND_ROOF: return { &Under_Ground_Tile_Roof_Array[0][0], 8, 8 };
    case TILE_ID_UNDERGROUND_ROOF_FILL_LEFT: return { &Under_Ground_Tile_Roof_Fill_Left[0][0], 8, 8 };
    case TILE_ID_UNDERGROUND_ROOF_FILL_RIGHT: return { &Under_Ground_Tile_Roof_Fill_Right[0][0], 8, 8 };
    case TILE_ID_UNDERGROUND_WALL_LEFT: return { &Under_Ground_Tile_Wall_Left[0][0], 8, 8 };
    case TILE_ID_UNDERGROUND_WALL_RIGHT: return { &Under_Ground_Tile_Wall_Right[0][0], 8, 8 };
    case TILE_ID_UNDERGROUND_GROUND: return { &Under_Ground_Tile_Ground[0][0], 8, 8 };
    case TILE_ID_UNDERGROUND_GROUND_FILL_LEFT: return { &Under_Ground_Tile_Ground_Fill_Left[0][0], 8, 8 };
    case TILE_ID_UNDERGROUND_GROUND_FILL_RIGHT: return { &Under_Ground_Tile_Ground_Fill_Right[0][0], 8, 8 };

    case TILE_ID_GRASS_SIDE_LEFT: return { &Ground_Tile_Grass_Side_Left[0][0], 8, 8 };
    case TILE_ID_GRASS_SIDE_RIGHT: return { &Ground_Tile_Grass_Side_Right[0][0], 8, 8 };

    case TILE_ID_FILL_LEFT: return { &Ground_Tile_Fill_Left[0][0], 8, 8 };
    case TILE_ID_FILL_RIGHT: return { &Ground_Tile_Fill_Right[0][0], 8, 8 };

    case TILE_ID_GRASS_DIRT_FILL_LEFT: return { &Ground_Tile_Grass_Dirt_Fill_Left[0][0], 8, 8 };
    case TILE_ID_GRASS_DIRT_FILL_RIGHT: return { &Ground_Tile_Grass_Dirt_Fill_Right[0][0], 8, 8 };

    case TILE_ID_GRASS_STRAIGHT_SIDE_RIGHT: return { &Ground_Tile_Grass_Stright_Side_Right[0][0], 8, 8 };
    case TILE_ID_GRASS_STRAIGHT_SIDE_LEFT:  return { &Ground_Tile_Grass_Stright_Side_Left[0][0], 8, 8 };

    case TILE_ID_VERT_CLIFF_LEFT: return { &Ground_Tile_Vert_Cliff_Left[0][0], 8, 8 };
    case TILE_ID_VERT_CLIFF_RIGHT: return { &Ground_Tile_Vert_Cliff_Right[0][0], 8, 8 };

    case TILE_ID_SLOPE_UP_RIGHT: return { &Ground_Tile_Slope_Up_Right[0][0], 8, 8 };
    case TILE_ID_SLOPE_UP_LEFT:  return { &Ground_Tile_Slope_Up_Left[0][0], 8, 8 };

    case TILE_ID_SEMI_GROUND: return { &SemiGround_Tile_Array[0][0], 8, 8 };

    case TILE_ID_LAVA: {
      // Simple boiling animation: swap frames
      uint32_t t = millis();
      bool f = ((t / 360) % 2) != 0;   // tweak speed here if desired
      return { f ? &Ground_Tile_Lava_2[0][0] : &Ground_Tile_Lava_1[0][0], 8, 8 };
    }

    // Large tiles
    case TILE_ID_GREEN_TUNNEL: return { &Ground_Tile_Green_Tunnel[0][0], 18, 17 };
    case TILE_ID_GREEN_TUNNEL_DOWN: return { &Ground_Tile_Green_Tunnel_Down[0][0], 18, 17 };
    case TILE_ID_GREEN_TUNNEL_NOTEL: return { &Ground_Tile_Green_NoTel_Tunnel[0][0], 18, 17 };
    case TILE_ID_GREEN_TUNNEL_LEFT: return { &Ground_Tile_Green_Tunnel_Left[0][0], 17, 18 };
    case TILE_ID_GREEN_TUNNEL_1_2: return { &Ground_Tile_Green_Tunnel_1_2[0][0], 18, 17 };
    case TILE_ID_GREEN_TUNNEL_DOWN_2_2: return { &Ground_Tile_Green_Tunnel_Down_2_2[0][0], 18, 17 };
    case TILE_ID_GREEN_TUNNEL_LEFT_2_4: return { &Ground_Tile_Green_Tunnel_Left_2_4[0][0], 17, 18 };
    case TILE_ID_GREEN_TUNNEL_4_4: return { &Ground_Tile_Green_Tunnel_4_4[0][0], 18, 17 };
    case TILE_ID_BUSH: return { &Ground_Tile_Bush[0][0], 25, 8 };

    case TILE_ID_BG_CLOUD_PUFF_A: return { &BG_Cloud_Puff_A[0][0], 7, 5 };
    case TILE_ID_BG_CLOUD_LONG_A: return { &BG_Cloud_Long_A[0][0], 14, 5 };

    case TILE_ID_MARIO_CASTLE: return { &Mario_Castle[0][0], 40, 40 };

    // Checkpoint flags (15×23)
    case TILE_ID_CHECK_POINT: return { &Ground_Tile_Check_Point[0][0], 15, 23 };
    case TILE_ID_CHECKED_POINT: return { &Ground_Tile_Checked_Point[0][0], 15, 23 };

    case TILE_ID_FLAG_TOP:  return { &Ground_Tile_Flag[0][0], 14, 15 };
    case TILE_ID_FLAG_POLE: return { &Ground_Tile_Flag_Pole[0][0], 2, 8 };
    case TILE_ID_FLAG_BASE: return { &Ground_Tile_Flag_Pole_Base[0][0], 8, 8 };

    // Bricks/items
    case TILE_ID_Q_BLOCK: return { &Tile_Brick_Question_Array[0][0], 8, 8 };
    case TILE_ID_SOLID_BLOCK: return { &Tile_Brick_Solid_Array[0][0], 8, 8 };
    case TILE_ID_BRICK: return { &Tile_Brick_Breakable_Array[0][0], 8, 8 };
    case TILE_ID_USED_BLOCK: return { &Tile_Brick_Used_Brick_Array[0][0], 8, 8 };
    case TILE_ID_BRICK_COIN: return { &Tile_Brick_Coin_Array[0][0], 8, 8 };

    case TILE_ID_BRICK_CLOUD: return { &Tile_Brick_Cloud[0][0], 8, 8 };

    case TILE_ID_GREEN_MUSHROOM_BRICK: return { &Tile_Brick_GreenMushroom_Array[0][0], 8, 8 };

    // Coin (6×6) and future items
    case TILE_ID_COIN: return { &worldCoin[0][0], 6, 6 };
    case TILE_ID_GREEN_MUSHROOM: return { &Green_Mushroom_Sprite[0][0], 8, 7 };
    // Mob sprites (visual-only)
    case TILE_ID_MOB_GOOMBA: return { &Mob_Goomba[0][0], 7, 8 };
    case TILE_ID_MOB_KOOPA_TROOPA: return { (const uint8_t*)&Mob_Koopa_Troopa_Walk1[0][0], 11, 19 };
    case TILE_ID_MOB_PIRANHA_PLANT_L: return { (const uint8_t*)&Mob_Piranha_Plant[0][0], 17, 29 };
    case TILE_ID_MOB_PIRANHA_PLANT_R: return { nullptr, 0, 0 };
    default: return { nullptr, 0, 0 };
  }
}

static inline void getFlagTileOffset(uint8_t tileId, int &ox, int &oy) {
  ox = 0; oy = 0;

  // These offsets are the alignment we want:
  // - Pole centered in an 8x8 cell above base notch
  // - Flag top aligned to pole (the flag sprite has its own pole pixels)
  switch (tileId) {
    case TILE_ID_FLAG_POLE: // 2x8
      ox = 3; oy = 0;
      break;

    case TILE_ID_FLAG_TOP:  // 14x15
      ox = 2; oy = 1;
      break;

    default:
      break;
  }
}

static inline int getTileRenderYOffset(uint8_t tileId) {
  switch (tileId) {
    case TILE_ID_GREEN_TUNNEL_LEFT:
    case TILE_ID_GREEN_TUNNEL_LEFT_2_4:
      return -2; // raise sprite up 2 pixels (render-only)
    default:
      return 0;
  }
}
// Draw a single tile sprite by TileID from Map_Build.h at screen coordinates
void drawTileAt(uint8_t tileId, int px, int py) {
  switch (tileId) {
    case TILE_ID_EMPTY:
      // Nothing to draw (0)
      break;
      
    case TILE_ID_GROUND:
      // ID 1: Ground_Tile_Array
      drawWorldTile8x8(Ground_Tile_Array, px, py);
      break;
      
    case TILE_ID_GROUND_TOP_LEFT:
      // ID 2: Ground_Tile_Top_Left
      drawWorldTile8x8(Ground_Tile_Top_Left, px, py);
      break;
      
    case TILE_ID_GROUND_TOP_RIGHT:
      // ID 3: Ground_Tile_Top_Right
      drawWorldTile8x8(Ground_Tile_Top_Right, px, py);
      break;
      
    case TILE_ID_GROUND_WALL_LEFT:
      // ID 4: Ground_Tile_Wall_Left
      drawWorldTile8x8(Ground_Tile_Wall_Left, px, py);
      break;
      
    case TILE_ID_GROUND_WALL_RIGHT:
      // ID 5: Ground_Tile_Wall_Right
      drawWorldTile8x8(Ground_Tile_Wall_Right, px, py);
      break;
      
    case TILE_ID_GROUND_CENTER_DIRT:
      // ID 6: Ground_Tile_Center_Dirt
      drawWorldTile8x8(Ground_Tile_Center_Dirt, px, py);
      break;

    case TILE_ID_STONE_LEDGE:
      // ID 31: Ground_Tile_Stone_Ledge (solid ground)
      drawWorldTile8x8(Ground_Tile_Stone_Ledge, px, py);
      break;

    case TILE_ID_UNDERGROUND_ROOF:
      // ID 33: Under_Ground_Tile_Roof_Array (underground roof, solid)
      drawWorldTile8x8(Under_Ground_Tile_Roof_Array, px, py);
      break;

    case TILE_ID_UNDERGROUND_ROOF_FILL_LEFT:
      // ID 37: Under_Ground_Tile_Roof_Fill_Left (underground roof fill, solid)
      drawWorldTile8x8(Under_Ground_Tile_Roof_Fill_Left, px, py);
      break;

    case TILE_ID_UNDERGROUND_ROOF_FILL_RIGHT:
      // ID 38: Under_Ground_Tile_Roof_Fill_Right (underground roof fill, right)
      drawWorldTile8x8(Under_Ground_Tile_Roof_Fill_Right, px, py);
      break;

    case TILE_ID_UNDERGROUND_WALL_RIGHT:
      // ID 34: Under_Ground_Tile_Wall_Right (underground right wall, solid)
      drawWorldTile8x8(Under_Ground_Tile_Wall_Right, px, py);
      break;

    case TILE_ID_UNDERGROUND_WALL_LEFT:
      // ID 36: Under_Ground_Tile_Wall_Left (underground left wall, solid)
      drawWorldTile8x8(Under_Ground_Tile_Wall_Left, px, py);
      break;

    case TILE_ID_UNDERGROUND_GROUND:
      // ID 35: Under_Ground_Tile_Ground (underground floor, solid)
      drawWorldTile8x8(Under_Ground_Tile_Ground, px, py);
      break;
    case TILE_ID_UNDERGROUND_GROUND_FILL_LEFT:
      // ID 40: Under_Ground_Tile_Ground_Fill_Left (underground ground fill, left)
      drawWorldTile8x8(Under_Ground_Tile_Ground_Fill_Left, px, py);
      break;
    case TILE_ID_UNDERGROUND_GROUND_FILL_RIGHT:
      // ID 39: Under_Ground_Tile_Ground_Fill_Right (underground ground fill, right)
      drawWorldTile8x8(Under_Ground_Tile_Ground_Fill_Right, px, py);
      break;
      
    case TILE_ID_SLOPE_UP_RIGHT:
      // ID 7: Ground_Tile_Slope_Up_Right
      drawWorldTile8x8(Ground_Tile_Slope_Up_Right, px, py);
      break;
      
    case TILE_ID_SLOPE_UP_LEFT:
      // ID 8: Ground_Tile_Slope_Up_Left
      drawWorldTile8x8(Ground_Tile_Slope_Up_Left, px, py);
      break;
      
    case TILE_ID_GRASS_SIDE_LEFT:
      // ID 9: Ground_Tile_Grass_Side_Left
      drawWorldTile8x8(Ground_Tile_Grass_Side_Left, px, py);
      break;

    case TILE_ID_BRICK_COIN:
      // ID 29: Tile_Brick_Coin_Array (new coin-brick)
      drawWorldTile8x8(Tile_Brick_Coin_Array, px, py);
      break;

    case TILE_ID_BRICK_CLOUD:
      // ID 55: Tile_Brick_Cloud (semi-solid cloud brick)
      drawWorldTile8x8(Tile_Brick_Cloud, px, py);
      break;
      
    case TILE_ID_GREEN_MUSHROOM_BRICK:
      // ID 30: Tile_Brick_GreenMushroom_Array (render-only)
      drawWorldTile8x8(Tile_Brick_GreenMushroom_Array, px, py);
      break;
    // Mob: Goomba (official)
    case TILE_ID_MOB_GOOMBA:
      // ID 42: Mob_Goomba (7x8)
      drawWorldTileCustomSize(&Mob_Goomba[0][0], 7, 8, px, py);
      break;

    // Mob: Koopa Troopa (spawn tile preview)
    case TILE_ID_MOB_KOOPA_TROOPA: {
      const int w = 11;
      const int h = 19;
      const int drawX = px + (8 - w) / 2;
      const int drawY = py + 8 - h;
      drawWorldTileCustomSize((const uint8_t*)&Mob_Koopa_Troopa_Walk1[0][0], w, h, drawX, drawY);
      break;
    }
    case TILE_ID_MOB_PIRANHA_PLANT_L: {
      // Piranha is 17x29, but centered across TWO tiles (16px wide).
      const int w = 17;
      const int h = 29;

      // Anchor rules:
      // - placement tile is the LEFT tile of the 2-wide footprint
      // - bottom of sprite sits on the bottom of the tile row
      // - center horizontally across 2 tiles
      const int drawX = px;
      const int drawY = py + 8 - h;

      drawWorldTileCustomSize((const uint8_t*)&Mob_Piranha_Plant[0][0], w, h, drawX, drawY);
      break;
    }
    case TILE_ID_MOB_PIRANHA_PLANT_R:
      // Helper tile: intentionally draw nothing.
      break;
    case TILE_ID_GRASS_SIDE_RIGHT:
      // ID 10: Ground_Tile_Grass_Side_Right
      drawWorldTile8x8(Ground_Tile_Grass_Side_Right, px, py);
      break;
      
    case TILE_ID_FILL_LEFT:
      // ID 11: Ground_Tile_Fill_Left
      drawWorldTile8x8(Ground_Tile_Fill_Left, px, py);
      break;
      
    case TILE_ID_FILL_RIGHT:
      // ID 12: Ground_Tile_Fill_Right
      drawWorldTile8x8(Ground_Tile_Fill_Right, px, py);
      break;
      
    case TILE_ID_GRASS_DIRT_FILL_LEFT:
      // ID 13: Ground_Tile_Grass_Dirt_Fill_Left
      drawWorldTile8x8(Ground_Tile_Grass_Dirt_Fill_Left, px, py);
      break;
      
    case TILE_ID_GRASS_DIRT_FILL_RIGHT:
      // ID 14: Ground_Tile_Grass_Dirt_Fill_Right
      drawWorldTile8x8(Ground_Tile_Grass_Dirt_Fill_Right, px, py);
      break;
      
    case TILE_ID_GRASS_STRAIGHT_SIDE_RIGHT:
      // ID 15: Ground_Tile_Grass_Straight_Side_Right
      drawWorldTile8x8(Ground_Tile_Grass_Stright_Side_Right, px, py);
      break;
      
    case TILE_ID_GRASS_STRAIGHT_SIDE_LEFT:
      // ID 16: Ground_Tile_Grass_Straight_Side_Left
      drawWorldTile8x8(Ground_Tile_Grass_Stright_Side_Left, px, py);
      break;
      
    case TILE_ID_VERT_CLIFF_LEFT:
      // ID 17: Ground_Tile_Vert_Cliff_Left
      drawWorldTile8x8(Ground_Tile_Vert_Cliff_Left, px, py);
      break;
      
    case TILE_ID_VERT_CLIFF_RIGHT:
      // ID 18: Ground_Tile_Vert_Cliff_Right
      drawWorldTile8x8(Ground_Tile_Vert_Cliff_Right, px, py);
      break;
      
    case TILE_ID_SEMI_GROUND:
      // ID 21: SemiGround_Tile_Array
      drawWorldTile8x8(SemiGround_Tile_Array, px, py);
      break;

    case TILE_ID_LAVA: {
      uint32_t t = millis();
      bool f = ((t / 360) % 2) != 0;
      drawWorldTile8x8(f ? Ground_Tile_Lava_2 : Ground_Tile_Lava_1, px, py);
      break;
    }
      
    case TILE_ID_GREEN_TUNNEL:
      // ID 19: Ground_Tile_Green_Tunnel (18×17 pixels)
      drawWorldTileCustomSize(&Ground_Tile_Green_Tunnel[0][0], 18, 17, px, py);
      break;

    case TILE_ID_GREEN_TUNNEL_NOTEL:
      // ID 44: Ground_Tile_Green_NoTel_Tunnel (18×17 pixels) - solid, no teleport
      drawWorldTileCustomSize(&Ground_Tile_Green_NoTel_Tunnel[0][0], 18, 17, px, py);
      break;

    case TILE_ID_GREEN_TUNNEL_LEFT:
      // ID 32: Ground_Tile_Green_Tunnel_Left (17×18 pixels) - left entrance
      drawWorldTileCustomSize(&Ground_Tile_Green_Tunnel_Left[0][0], 17, 18, px, py + getTileRenderYOffset(tileId));
      break;

    case TILE_ID_GREEN_TUNNEL_1_2:
      // ID 41: Ground_Tile_Green_Tunnel_1_2 (18×17 pixels)
      drawWorldTileCustomSize(&Ground_Tile_Green_Tunnel_1_2[0][0], 18, 17, px, py);
      break;

    case TILE_ID_GREEN_TUNNEL_DOWN_2_2:
      // ID 59: Ground_Tile_Green_Tunnel_Down_2_2 (18×17 pixels)
      drawWorldTileCustomSize(&Ground_Tile_Green_Tunnel_Down_2_2[0][0], 18, 17, px, py);
      break;

    case TILE_ID_GREEN_TUNNEL_LEFT_2_4:
      // ID 60: Ground_Tile_Green_Tunnel_Left_2_4 (17×18 pixels) - left entrance variant
      drawWorldTileCustomSize(&Ground_Tile_Green_Tunnel_Left_2_4[0][0], 17, 18, px, py + getTileRenderYOffset(tileId));
      break;

    case TILE_ID_GREEN_TUNNEL_4_4:
      // ID 61: Ground_Tile_Green_Tunnel_4_4 (18×17 pixels)
      drawWorldTileCustomSize(&Ground_Tile_Green_Tunnel_4_4[0][0], 18, 17, px, py);
      break;
      
    case TILE_ID_GREEN_TUNNEL_DOWN:
      // ID 22: Ground_Tile_Green_Tunnel_Down (18×17 pixels)
      drawWorldTileCustomSize(&Ground_Tile_Green_Tunnel_Down[0][0], 18, 17, px, py);
      break;
      
    case TILE_ID_BUSH:
      // ID 20: Ground_Tile_Bush (25×8 pixels)
      drawWorldTileCustomSize(&Ground_Tile_Bush[0][0], 25, 8, px, py);
      break;

    case TILE_ID_BG_CLOUD_PUFF_A:
      drawWorldTileCustomSize(&BG_Cloud_Puff_A[0][0], 7, 5, px, py);
      break;

    case TILE_ID_BG_CLOUD_LONG_A:
      drawWorldTileCustomSize(&BG_Cloud_Long_A[0][0], 14, 5, px, py);
      break;

    case TILE_ID_MARIO_CASTLE:
      // Decoration only (no collision) — draw from top-left anchor.
      drawWorldTileCustomSize((const uint8_t*)&Mario_Castle[0][0], 40, 40, px, py);
      break;

    case TILE_ID_FLAG_TOP: {
      // Drawn in overlay pass (drawFlagTopOverlay) to prevent clipping by tiles above.
      return;
    }

    case TILE_ID_FLAG_POLE: {
      int ox, oy;
      getFlagTileOffset(tileId, ox, oy);
      drawWorldTileCustomSize((const uint8_t*)&Ground_Tile_Flag_Pole[0][0], 2, 8, px + ox, py + oy);
      break;
    }

    case TILE_ID_FLAG_BASE:
      drawWorldTile8x8(Ground_Tile_Flag_Pole_Base, px, py);
      break;

    case TILE_ID_CHECK_POINT:
      // ID 48: Ground_Tile_Check_Point (15×23) - bottom anchored on tile
      drawWorldTileCustomSize((const uint8_t*)&Ground_Tile_Check_Point[0][0], 15, 23, px, py + 8 - 23);
      break;

    case TILE_ID_CHECKED_POINT:
      // ID 49: Ground_Tile_Checked_Point (15×23) - bottom anchored on tile
      drawWorldTileCustomSize((const uint8_t*)&Ground_Tile_Checked_Point[0][0], 15, 23, px, py + 8 - 23);
      break;
      
    default:
      // Unknown tile ID: draw nothing
      break;
  }
}

static bool isCheckpointTile(uint8_t id) {
  return (id == TILE_ID_CHECK_POINT) || (id == TILE_ID_CHECKED_POINT);
}

static void drawCheckpointTileAnimated(int mapX, int mapY, int px, int py, uint8_t actualTileId) {
  uint8_t drawId = actualTileId;
  int xWiggle = 0;

  if (gCheckpointAnim.active && gCheckpointAnim.tileX == mapX && gCheckpointAnim.tileY == mapY) {
    uint32_t elapsed = gameplayNowMs() - gCheckpointAnim.startMs;

    // End animation
    if (elapsed >= gCheckpointAnim.durationMs) {
      gCheckpointAnim.active = false;
      drawId = gCheckpointAnim.endTileId; // final visual state
    } else {
      // Wiggle frame index
      const uint16_t frameMs = 40; // tweak: 30–50ms
      uint8_t idx = (elapsed / frameMs) % kCheckpointWiggleCount;
      xWiggle = kCheckpointWiggle[idx];

      // Visual flip timing:
      // - First ~60% show the start sprite
      // - Last ~40% show the end sprite
      if (elapsed < (uint32_t)(gCheckpointAnim.durationMs * 0.60f)) {
        drawId = gCheckpointAnim.startTileId;
      } else {
        drawId = gCheckpointAnim.endTileId;
      }
    }
  }

  drawTileAt(drawId, px + xWiggle, py);
}

// Re-draw the pipe ON TOP of Mario during pipe sink/rise.
// This masks Mario so it looks like he goes into the pipe.
static void drawActivePipeOverHero() {
  if (!matrix) return;
  if (!heroShouldRenderBehindPipe()) return;
  if (gPipeTransition.pairIndex >= gTunnelPairCount) return;

  int tx = -1;
  int ty = -1;
  int tileId = -1;

  const TunnelPair &tp = gTunnelPairs[gPipeTransition.pairIndex];

  if (gPipeTransition.phase == PIPE_PHASE_SINK) {
    // If entering from left, overlay the left entrance tile (e.g., tile 60).
    if (gPipeTransition.entryIsLeft && tp.hasLeftEntry) {
      tx = tp.leftEntry.tileX;
      ty = tp.leftEntry.tileY;
      tileId = tp.leftEntry.tileId;
    } else {
      // Upright entry: overlay the entry tile.
      tx = tp.entry.tileX;
      ty = tp.entry.tileY;
      tileId = tp.entry.tileId;
    }
  } else if (gPipeTransition.phase == PIPE_PHASE_RISE) {
    // During RISE, overlay the exit pipe tile (e.g., tile 61 upright).
    tx = gPipeTransition.exitTileX;
    ty = gPipeTransition.exitTileY;
    tileId = (int)gPipeTransition.exitTileId;
  }

  if (tx < 0 || ty < 0 || tileId < 0) return;

  int worldX = tx * TILE_SIZE;
  int worldY = ty * TILE_SIZE;
  int px = worldX - (int)cameraX;
  int py = worldY - (int)cameraY;

  drawTileAt((uint8_t)tileId, px, py);
}

// Back-compat wrapper (kept because other call sites may reference it).
void drawPipeForegroundDuringTransition() {
  drawActivePipeOverHero();
}

// Draw tiles that are visible in the camera view.
// NOTE: This is the single world render path used by drawScene().
void drawTiles() {
  uint16_t ground      = matrix->color565(0, 120, 40);

  uint16_t qBorder     = matrix->color565(252, 160, 68);
  uint16_t qFill       = matrix->color565(252, 216, 96);
  uint16_t qShadow     = matrix->color565(180, 112, 32);

  uint16_t brickMain   = matrix->color565(150, 75, 30);
  uint16_t brickDark   = matrix->color565(90, 45, 15);

  uint16_t solidFill   = matrix->color565(180, 140, 80);
  uint16_t usedFill    = matrix->color565(140, 100, 60);

  const int tileMargin = 2;
  const int extraLeftTiles = 5;
  const int LEFT_OVERDRAW_PX = extraLeftTiles * TILE_SIZE; // 40px

  // IMPORTANT: drawScene() calls drawTiles() for world rendering.
  // Extend the LEFT iteration window so large sprites (18x17 tunnels, 25x8 bushes, 40x40 castle)
  // keep being visited and drawn even after their anchor tile scrolls offscreen-left.
  // Keep RIGHT behavior unchanged (no extra columns beyond the screen window).
  int firstTileX = (int)(cameraX / TILE_SIZE) - (tileMargin + extraLeftTiles);
  int lastTileX  = (int)((cameraX + MATRIX_WIDTH - 1) / TILE_SIZE);
  if (firstTileX < 0) firstTileX = 0;
  if (lastTileX >= LEVEL_WIDTH) lastTileX = LEVEL_WIDTH - 1;

  auto shouldCullSprite = [&](int drawX, int drawY, int w, int h) -> bool {
    // LEFT: allow drawing until the sprite is fully 40px past the left edge.
    if (drawX + w <= -LEFT_OVERDRAW_PX) return true;
    // RIGHT: unchanged behavior.
    if (drawX >= MATRIX_WIDTH) return true;
    // TOP/BOTTOM: basic bounds.
    if (drawY + h <= 0) return true;
    if (drawY >= MATRIX_HEIGHT) return true;
    return false;
  };

  for (int ty = 0; ty < LEVEL_HEIGHT; ty++) {
    for (int tx = firstTileX; tx <= lastTileX; tx++) {
      // Render based on TileID (single source of truth for visuals)
      uint8_t id = getTileId(tx, ty);
      if (id == TILE_ID_EMPTY) continue;

      int worldX = tx * TILE_SIZE;
      int worldY = ty * TILE_SIZE + tileOffset[ty][tx];

      int px = worldX - (int)cameraX;
      int py = worldY - (int)cameraY;

      // NOTE: do NOT early-cull by 8x8 here; large sprites can extend beyond a tile.

      // Interactive tiles with special drawing
      if (id == TILE_ID_COIN) {
        const int drawX = px + 1;
        const int drawY = py + 1;
        if (shouldCullSprite(drawX, drawY, 6, 6)) continue;
        drawHUDCoinMini(drawX, drawY);
        continue;
      }

      // Render-only mob tile: Piranha (left anchor draws, right helper is invisible)
      if (id == TILE_ID_MOB_PIRANHA_PLANT_L) {
        const int w = 17;
        const int h = 29;
        const int drawX = px;
        const int drawY = py + 8 - h;
        if (shouldCullSprite(drawX, drawY, w, h)) continue;
        drawWorldTileCustomSize((const uint8_t*)&Mob_Piranha_Plant[0][0], w, h, drawX, drawY);
        continue;
      }
      if (id == TILE_ID_MOB_PIRANHA_PLANT_R) {
        continue;
      }

      // Checkpoint flags: draw with wiggle + visual flip when animating
      if (isCheckpointTile(id)) {
        // Bottom-anchored (15x23)
        const int w = 15;
        const int h = 23;
        const int drawX = px;
        const int drawY = py + 8 - h;
        if (shouldCullSprite(drawX, drawY, w, h)) continue;
        drawCheckpointTileAnimated(tx, ty, px, py, id);
        continue;
      }

      // Use sprite resolver for exact TileID visuals
      SpriteRef sr = getSpriteForTileId(id);
      if (sr.data == nullptr) continue;

      int ox, oy;
      getFlagTileOffset(id, ox, oy);

      // Flag-top: draw CAP only (cloth is a moving overlay)
      if (id == TILE_ID_FLAG_TOP) {
        // Drawn in overlay pass (drawFlagTopOverlay) to prevent clipping by tiles above.
        continue;
      }

      const int drawX = px + ox;
      const int drawY = py + oy + getTileRenderYOffset(id);
      const int w = (sr.width  > 0) ? (int)sr.width  : TILE_SIZE;
      const int h = (sr.height > 0) ? (int)sr.height : TILE_SIZE;
      if (shouldCullSprite(drawX, drawY, w, h)) continue;

      // If sprite is 8×8, use the optimized 8×8 path; otherwise use custom size
      if (sr.width == 8 && sr.height == 8) {
        drawWorldTile8x8((const uint8_t (*)[8])sr.data, drawX, drawY);
      } else {
        drawWorldTileCustomSize(sr.data, sr.width, sr.height, drawX, drawY);
      }
    }
  }

  // Draw moving cloth overlay (cap stays static)
  drawFlagTopOverlay();
  drawFlagPoleClothOverlay();
}

void drawMushroom() {
  if (!gMushroom.active) return;
  int x0 = (int)(gMushroom.x - cameraX);
  int y0 = (int)(gMushroom.y - cameraY);

  // Select sprite and dimensions based on mushroom type
  const uint8_t (*sprite7)[8] = nullptr;
  int w = 8, h = 7;
  if (gMushroom.type == ITEM_GREEN_MUSHROOM) {
    sprite7 = Green_Mushroom_Sprite;
  } else {
    sprite7 = Red_Mushroom_Sprite;
  }

  if (x0 >= MATRIX_WIDTH || x0 + w <= 0) return;
  if (y0 >= MATRIX_HEIGHT || y0 + h <= 0) return;

  for (int y = 0; y < h; y++) {
    int sy = y0 + y;
    if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

    for (int x = 0; x < w; x++) {
      int sx = x0 + x;
      if (sx < 0 || sx >= MATRIX_WIDTH) continue;

      uint8_t idx = pgm_read_byte(&sprite7[y][x]);
      if (idx == 0) continue;

      RGB color;
      memcpy_P(&color, &heroPalette[idx], sizeof(RGB));
      matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
    }
  }
}

// Tiny HUD coin for in-world coins — use the editor-supplied 6x6 design
void drawHUDCoinMini(int px, int py) {
  if (!matrix) return;

  for (int y = 0; y < 6; y++) {
    for (int x = 0; x < 6; x++) {
      uint8_t idx = pgm_read_byte(&worldCoin[y][x]);
      if (idx == 0) continue;

      RGB color;
      memcpy_P(&color, &heroPalette[idx], sizeof(RGB));

      matrix->drawPixelRGB888(px + x, py + y, color.r, color.g, color.b);
    }
  }
}

// HUD: compact digits + 'x' using a monospaced 3x5 font
// Each glyph is 5 rows of 3 bits (MSB on left). Bottom row is the baseline.
struct TinyGlyph {
  uint8_t rows[5];
};

const TinyGlyph tinyGlyphs[16] PROGMEM = {
  // 0
  {{ 0b111,
     0b101,
     0b101,
     0b101,
     0b111 }},
  // 1
  {{ 0b010,
     0b110,
     0b010,
     0b010,
     0b111 }},
  // 2
  {{ 0b111,
     0b001,
     0b111,
     0b100,
     0b111 }},
  // 3
  {{ 0b111,
     0b001,
     0b111,
     0b001,
     0b111 }},
  // 4
  {{ 0b101,
     0b101,
     0b111,
     0b001,
     0b001 }},
  // 5
  {{ 0b111,
     0b100,
     0b111,
     0b001,
     0b111 }},
  // 6
  {{ 0b111,
     0b100,
     0b111,
     0b101,
     0b111 }},
  // 7
  {{ 0b111,
     0b001,
     0b010,
     0b010,
     0b010 }},
  // 8
  {{ 0b111,
     0b101,
     0b111,
     0b101,
     0b111 }},
  // 9
  {{ 0b111,
     0b101,
     0b111,
     0b001,
     0b111 }},
      // 'x' (index 10) – new 3x5 X from matrix editor (rows: 000,101,010,101,000)
      {{ 0b000,
        0b101,
        0b010,
        0b101,
        0b000 }},
  // 'P' (index 11)
  {{ 0b111,
     0b101,
     0b111,
     0b100,
     0b100 }},
  // 'A' (index 12)
  {{ 0b111,
     0b101,
     0b111,
     0b101,
     0b101 }},
  // 'U' (index 13)
  {{ 0b101,
     0b101,
     0b101,
     0b101,
     0b111 }},
  // 'S' (index 14)
  {{ 0b111,
     0b100,
     0b111,
     0b001,
     0b111 }},
  // 'E' (index 15)
  {{ 0b111,
     0b100,
     0b111,
     0b100,
     0b111 }}
};

void drawTinyGlyph(int px, int py, uint8_t glyphIndex, uint16_t color) {
  if (!matrix) return;
  if (glyphIndex > 15) return;

  TinyGlyph g;
  memcpy_P(&g, &tinyGlyphs[glyphIndex], sizeof(TinyGlyph));

  for (int row = 0; row < 5; row++) {
    uint8_t bits = g.rows[row];
    for (int col = 0; col < 3; col++) {
      if (bits & (1 << (2 - col))) {
        int sx = px + col;
        int sy = py + row;
        if (sx < 0 || sx >= MATRIX_WIDTH || sy < 0 || sy >= MATRIX_HEIGHT) continue;
        matrix->drawPixel(sx, sy, color);
      }
    }
  }
}

void drawGoombas() {
  int w = (int)GOOMBA_W;
  int h = (int)GOOMBA_H;

  for (int i = 0; i < MAX_GOOMBAS; ++i) {
    if (!gGoombas[i].active) continue;
    int x0 = (int)(gGoombas[i].x - cameraX);
    int y0 = (int)(gGoombas[i].y - cameraY);

    if (x0 >= MATRIX_WIDTH || x0 + w <= 0) continue;
    if (y0 >= MATRIX_HEIGHT || y0 + h <= 0) continue;

    const uint8_t (*sprite7)[7] = (gGoombas[i].animFrame == 0) ? Mob_Goomba_Walk_1 : Mob_Goomba_Walk_2;

    if (gGoombas[i].state == GOOMBA_STATE_SQUISHED) {
      // draw squished: only draw lower rows to simulate flattening
      int yStart = 2;
      for (int y = yStart; y < h; y++) {
        int sy = y0 + (y - yStart) + 2; // nudge down a bit
        if (sy < 0 || sy >= MATRIX_HEIGHT) continue;
        for (int x = 0; x < w; x++) {
          int sx = x0 + x;
          if (sx < 0 || sx >= MATRIX_WIDTH) continue;
          uint8_t px = pgm_read_byte(&sprite7[y][x]);
          if (px == 0) continue;
          RGB color;
          memcpy_P(&color, &heroPalette[px], sizeof(RGB));
          matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
        }
      }
    } else {
      for (int y = 0; y < h; y++) {
        int sy = y0 + y;
        if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

        for (int x = 0; x < w; x++) {
          int sx = x0 + x;
          if (sx < 0 || sx >= MATRIX_WIDTH) continue;

          uint8_t px = pgm_read_byte(&sprite7[y][x]);
          if (px == 0) continue;
          RGB color;
          memcpy_P(&color, &heroPalette[px], sizeof(RGB));
          matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
        }
      }
    }
  }
}

void drawKoopas() {
  const int w = (int)KOOPA_W;
  const int h = (int)KOOPA_H;

  for (int i = 0; i < MAX_KOOPAS; ++i) {
    const Koopa &kk = gKoopas[i];
    if (!kk.active) continue;

    int x0 = (int)(kk.x - cameraX);
    int y0 = (int)(kk.y - cameraY);

    if (x0 >= MATRIX_WIDTH || x0 + w <= 0) continue;
    if (y0 >= MATRIX_HEIGHT || y0 + h <= 0) continue;

    // IMPORTANT:
    // Walk1 and Walk2 are directional (left vs right). Do NOT swap them for animation,
    // otherwise you get the 180° "twist" when combined with flip logic.
    const uint8_t *sprite = (kk.facing < 0)
      ? (const uint8_t*)&Mob_Koopa_Troopa_Walk1[0][0]   // left-facing art
      : (const uint8_t*)&Mob_Koopa_Troopa_Walk2[0][0];  // right-facing art

    // Bob down 1px on frame 2
    if (kk.animFrame == 1) y0 += 1;

    // No flip needed because sprites are already directional
    drawWorldTileCustomSize(sprite, w, h, x0, y0);
  }
}

void drawTinyGlyphScaled(int px, int py, uint8_t glyphIndex, uint16_t color, int scale) {
  if (!matrix) return;
  if (glyphIndex > 15) return;
  if (scale <= 0) return;

  TinyGlyph g;
  memcpy_P(&g, &tinyGlyphs[glyphIndex], sizeof(TinyGlyph));

  for (int row = 0; row < 5; ++row) {
    uint8_t bits = g.rows[row];
    for (int col = 0; col < 3; ++col) {
      if (bits & (1 << (2 - col))) {
        int baseX = px + col * scale;
        int baseY = py + row * scale;

        for (int dy = 0; dy < scale; ++dy) {
          int sy = baseY + dy;
          if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

          for (int dx = 0; dx < scale; ++dx) {
            int sx = baseX + dx;
            if (sx < 0 || sx >= MATRIX_WIDTH) continue;
            matrix->drawPixel(sx, sy, color);
          }
        }
      }
    }
  }
}

// Simple 3x5 'Y' and 'N' for YES/NO in the pause menu
void drawLetterY(int px, int py, uint16_t color) {
  if (!matrix) return;
  // Pattern (3x5):
  // 1 0 1
  // 1 0 1
  // 0 1 0
  // 0 1 0
  // 0 1 0
  const uint8_t rows[5] = {
    0b101,
    0b101,
    0b010,
    0b010,
    0b010
  };
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (rows[r] & (1 << (2 - c))) {
        int sx = px + c;
        int sy = py + r;
        if (sx < 0 || sx >= MATRIX_WIDTH || sy < 0 || sy >= MATRIX_HEIGHT) continue;
        matrix->drawPixel(sx, sy, color);
      }
    }
  }
}

void drawLetterN(int px, int py, uint16_t color) {
  if (!matrix) return;
  // Pattern (3x5):
  // 1 0 1
  // 1 1 1
  // 1 1 1
  // 1 1 1
  // 1 0 1
  const uint8_t rows[5] = {
    0b101,
    0b111,
    0b111,
    0b111,
    0b101
  };
  for (int r = 0; r < 5; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (rows[r] & (1 << (2 - c))) {
        int sx = px + c;
        int sy = py + r;
        if (sx < 0 || sx >= MATRIX_WIDTH || sy < 0 || sy >= MATRIX_HEIGHT) continue;
        matrix->drawPixel(sx, sy, color);
      }
    }
  }
}

// --- HUD drawing helpers (small font & blit helpers) ---
// Read a byte from a PROGMEM 2D array flattened pointer
static inline uint8_t prog_read(const uint8_t *arr, int idx) {
  return pgm_read_byte_near(arr + idx);
}

// Generic blit for a PROGMEM 2D array stored row-major at &arr[0][0]
void blitHudArray(const uint8_t *arr, int w, int h, int dstX, int dstY) {
  if (!matrix) return;
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      uint8_t v = prog_read(arr, r * w + c);
      if (v == 0) continue;  // palette index 0 = transparent

      int sx = dstX + c;
      int sy = dstY + r;

      // HARD CLIP to screen bounds so nothing wraps around.
      if (sx < 0 || sx >= MATRIX_WIDTH || sy < 0 || sy >= MATRIX_HEIGHT) {
        continue;
      }

      RGB color;
      memcpy_P(&color, &heroPalette[v], sizeof(RGB));
      matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
    }
  }
}

// Blit a sub-rectangle from a PROGMEM array.
// - fullW/fullH describe the source image dimensions
// - srcX/srcY/subW/subH describe which part to draw
void blitHudArraySubrect(const uint8_t *arr,
                         int fullW,
                         int fullH,
                         int dstX,
                         int dstY,
                         int srcX,
                         int srcY,
                         int subW,
                         int subH) {
  if (!matrix) return;
  if (subW <= 0 || subH <= 0) return;

  if (srcX < 0) { dstX -= srcX; subW += srcX; srcX = 0; }
  if (srcY < 0) { dstY -= srcY; subH += srcY; srcY = 0; }
  if (srcX + subW > fullW)  subW = fullW - srcX;
  if (srcY + subH > fullH)  subH = fullH - srcY;
  if (subW <= 0 || subH <= 0) return;

  for (int r = 0; r < subH; ++r) {
    int sr = srcY + r;
    int sy = dstY + r;
    if (sy < 0 || sy >= MATRIX_HEIGHT) continue;

    for (int c = 0; c < subW; ++c) {
      int sc = srcX + c;
      int sx = dstX + c;
      if (sx < 0 || sx >= MATRIX_WIDTH) continue;

      uint8_t v = prog_read(arr, sr * fullW + sc);
      if (v == 0) continue;

      RGB color;
      memcpy_P(&color, &heroPalette[v], sizeof(RGB));
      matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
    }
  }
}

// Blit but skip palette index 9 (magenta placeholders).
void blitHudArraySkip9(const uint8_t *arr, int w, int h, int dstX, int dstY) {
  if (!matrix) return;
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      uint8_t v = prog_read(arr, r * w + c);
      if (v == 0 || v == 9) continue;

      int sx = dstX + c;
      int sy = dstY + r;

      if (sx < 0 || sx >= MATRIX_WIDTH || sy < 0 || sy >= MATRIX_HEIGHT) continue;

      RGB color;
      memcpy_P(&color, &heroPalette[v], sizeof(RGB));
      matrix->drawPixelRGB888(sx, sy, color.r, color.g, color.b);
    }
  }
}

// Draw exactly 3 digits (zero-padded) using the tiny 3x5 font.
void drawNumber3Digits(int x, int y, int value, uint16_t color) {
  if (!matrix) return;
  if (value < 0) value = 0;
  if (value > 999) value = 999;

  int d0 = value / 100;         // hundreds
  int d1 = (value / 10) % 10;   // tens
  int d2 = value % 10;          // ones

  drawTinyGlyph(x + 0,  y, d0, color);
  drawTinyGlyph(x + 4,  y, d1, color);
  drawTinyGlyph(x + 8,  y, d2, color);
}

// Draw exactly 2 digits (zero-padded) using the tiny 3x5 font.
void drawNumber2Digits(int x, int y, int value, uint16_t color) {
  if (!matrix) return;
  if (value < 0) value = 0;
  if (value > 99) value = 99;

  int d0 = value / 10;      // tens
  int d1 = value % 10;      // ones

  drawTinyGlyph(x + 0, y, d0, color);
  drawTinyGlyph(x + 4, y, d1, color);
}

// Simple static HUD: draw exactly what the Matrix Editor exported.
// Uses MarioHudSprites.h arrays and heroPalette indices.
// No dynamic overlays or placeholder logic.
void drawStaticHudFromEditor() {
  if (!matrix) return;

  // Mario_Lives_Array removed - now drawn dynamically in drawHudLives()
  // Date removed - now drawn dynamically in drawHudDate()
  // Time is now drawn dynamically in drawHudClockTime()
  // Temp_Pos_Array removed - now drawn dynamically in drawHudTemp()
  // Rain_Array removed - now drawn dynamically in drawHudPrecip()
}

// Draw the MM/DD/YY date HUD using Date_Array and gHudClock.
void drawHudDate() {
  if (!matrix) return;

  // Date HUD block (same size & position as original Date_Array draw)
  constexpr int DATE_W = 31;
  constexpr int DATE_H = 5;
  constexpr int DATE_X = 33;
  constexpr int DATE_Y = 1;

  // Draw the static art, skipping palette index 9 (magenta placeholders)
  blitHudArraySkip9((const uint8_t*)&Date_Array[0][0], DATE_W, DATE_H, DATE_X, DATE_Y);

  // If the clock isn't valid yet, show 00/00/00
  int month = 0;
  int day   = 0;
  int year2 = 0;

  if (gHudClock.valid) {
    month = gHudClock.month;
    day   = gHudClock.day;
    year2 = gHudClock.year % 100;
  }

  // Clamp to 0–99 so the 2-digit renderer never overflows
  if (month < 0) month = 0;
  if (month > 99) month = 99;

  if (day < 0) day = 0;
  if (day > 99) day = 99;

  if (year2 < 0) year2 = 0;
  if (year2 > 99) year2 = 99;

  uint16_t white = matrix->color565(255, 255, 255);

  // Position of digits inside the 31x5 block:
  // Align these with the magenta 9 placeholders in Date_Array so the layout is:
  //   "MM/DD/YY"
  // Adjusted positions: month -1px, day +1px, year +3px for proper alignment
  const int monthX = DATE_X + 0;  // "M M" (shifted left by 1)
  const int dayX   = DATE_X + 12; // "D D" (shifted right by 1)
  const int yearX  = DATE_X + 24; // "Y Y" (shifted right by 3)
  const int digitY = DATE_Y + 0;  // center vertically in the 5-pixel tall row

  // Draw MM/DD/YY as 2-digit groups.
  drawNumber2Digits(monthX, digitY, month, white);
  drawNumber2Digits(dayX,   digitY, day,   white);
  drawNumber2Digits(yearX,  digitY, year2, white);
}

// Draw the HH:MM AM/PM HUD using Time_AM_Array / Time_PM_Array and gHudClock.
void drawHudClockTime() {
  if (!matrix) return;

  // Position and size of the time block on the matrix
  constexpr int TIME_W = 28;
  constexpr int TIME_H = 5;
  constexpr int TIME_X = 67;
  constexpr int TIME_Y = 1;

  // If clock is not valid yet, just show 00:00 AM as a fallback
  int hour24 = gHudClock.valid ? gHudClock.hour   : 0;
  int minute = gHudClock.valid ? gHudClock.minute : 0;

  // Convert 24-hour to 12-hour and decide AM/PM
  bool isPM = (hour24 >= 12);
  int hour12 = hour24 % 12;
  if (hour12 == 0) {
    hour12 = 12; // 0 -> 12 AM, 12 -> 12 PM
  }

  // Clamp minutes
  if (minute < 0)   minute = 0;
  if (minute > 59)  minute = 59;

  // Choose which background to use (AM or PM).
  const uint8_t* timeArr = isPM
    ? (const uint8_t*)&Time_PM_Array[0][0]
    : (const uint8_t*)&Time_AM_Array[0][0];

  // Draw the background, skipping magenta placeholders (palette index 9).
  blitHudArraySkip9(timeArr, TIME_W, TIME_H, TIME_X, TIME_Y);

  // Break HH:MM into individual digits
  int hh = hour12;
  int mm = minute;

  int d0 = (hh / 10) % 10;   // hour tens
  int d1 = hh % 10;          // hour ones
  int d2 = (mm / 10) % 10;   // minute tens
  int d3 = mm % 10;          // minute ones

  // Tracker indicator: time turns red when OTA is armed OR input recorder is recording.
  uint16_t timeColor = (isOTAArmed() || gDemoIsRecording)
    ? matrix->color565(228, 32, 32)
    : matrix->color565(255, 255, 255);

  // Layout inside the 28×5 block:
  // "H H : M M"
  //
  // Each tiny digit is 3px wide. We'll step by 4px so there is a 1px gap.
  // This keeps everything on the left side, leaving the AM/PM letters on the right
  // (already drawn by the Time_AM/Time_PM arrays).
  // Shifted left by 1 pixel to align better with the baked-in colon.
  const int baseY = TIME_Y;          // same top row as the time block
  const int d0X   = TIME_X + 0;      // hour tens (shifted left)
  const int d1X   = d0X   + 4;       // hour ones
  const int colonX = d1X  + 4;       // colon from the Time_AM/Time_PM array

  // Shift minute digits 1px LEFT so we get "1 1 : 5 8 P M"
  const int d2X   = colonX + 2;      // minute tens
  const int d3X   = d2X   + 4;       // minute ones

  // Draw hour digits
  drawTinyGlyph(d0X, baseY, d0, timeColor);
  drawTinyGlyph(d1X, baseY, d1, timeColor);

  // Colon is already drawn by Time_AM_Array / Time_PM_Array, so no need to draw it here.

  // Draw minute digits
  drawTinyGlyph(d2X, baseY, d2, timeColor);
  drawTinyGlyph(d3X, baseY, d3, timeColor);

  // The AM/PM letters to the right are already part of the Time_AM/Time_PM arrays
  // and were drawn by blitHudArraySkip9(), so we don't need to do anything else here.
}

// Draw the Temperature HUD with live temperature and sign (positive/negative).
void drawHudTemp() {
  if (!matrix) return;

  constexpr int TEMP_W = 15;
  constexpr int TEMP_H = 5;
  constexpr int TEMP_X = 98;
  constexpr int TEMP_Y = 1;

  // Always use numeric value (0 if no valid temp, or last valid temp)
  int temp = gWeatherHasTemp ? gWeatherTempF : 0;

  // Determine sign and absolute value
  bool isNegative = (temp < 0);
  int absTemp = isNegative ? -temp : temp;
  if (absTemp < 0) absTemp = 0;      // safety
  if (absTemp > 999) absTemp = 999;  // clamp

  // Choose positive or negative sign array
  const uint8_t* src = isNegative
    ? (const uint8_t*)&Temp_Neg_Array[0][0]
    : (const uint8_t*)&Temp_Pos_Array[0][0];

  // Blit sign art, skipping magenta (9) placeholders
  blitHudArraySkip9(src, TEMP_W, TEMP_H, TEMP_X, TEMP_Y);

  // Draw 3-digit temperature in white
  uint16_t white = matrix->color565(255, 255, 255);

  // Position digits inside the 15x5 block where magenta placeholders are
  // Shifted left by 1 pixel to create gap between last digit and "F"
  const int digitX = TEMP_X + 0;
  const int digitY = TEMP_Y + 0;

  drawNumber3Digits(digitX, digitY, absTemp, white);
}

// Draw the Coin HUD with live coinCount digits (no placeholder scanning).
void drawHudCoins() {
  if (!matrix) return;

  constexpr int COIN_W = 22;
  constexpr int COIN_H = 5;
  constexpr int COIN_X = 140;
  constexpr int COIN_Y = 1;

  // Blit static art but skip magenta placeholder pixels (palette 9).
  blitHudArraySkip9((const uint8_t*)&Coin_Array[0][0], COIN_W, COIN_H, COIN_X, COIN_Y);

  uint16_t white = matrix->color565(255, 255, 255);
  int coins = coinCount;
  if (coins < 0) coins = 0;
  if (coins > 999) coins = 999;

  // Magenta zeros in the editor start at column 11, row 0 inside the 22x5 block.
  const int digitX = COIN_X + 11;
  const int digitY = COIN_Y + 0;
  drawNumber3Digits(digitX, digitY, coins, white);
}

// Draw the Mario Lives HUD with live gMarioLives count.
void drawHudLives() {
  if (!matrix) return;

  // Mario lives HUD block from the editor
  constexpr int LIVES_W = 23;
  constexpr int LIVES_H = 11;
  constexpr int LIVES_X = 0;  // Must be 0 to match original art position
  constexpr int LIVES_Y = 0;

  // Draw Mario head + "x" + background, skipping magenta placeholders
  blitHudArraySkip9((const uint8_t*)&Mario_Lives_Array[0][0],
                    LIVES_W, LIVES_H, LIVES_X, LIVES_Y);

  // Clamp lives to 0–999 just in case
  int lives = gMarioLives;
  if (lives < 0)   lives = 0;
  if (lives > 999) lives = 999;

  uint16_t white = matrix->color565(255, 255, 255);

  // The three magenta digits in Mario_Lives_Array start at column 8,
  // then 12, then 16 inside the 23-wide block (8,12,16).
  // Our 3×5 digits use 4-pixel spacing (x, x+4, x+8),
  // so we just anchor the first digit at LIVES_X + 8.
  const int digitX = LIVES_X + 8;
  const int digitY = LIVES_Y + 6; // same baseline row as the magenta zeros

  drawNumber3Digits(digitX, digitY, lives, white);
}

// Draw the Rain/Snow HUD with precipitation percentage.
void drawHudPrecip() {
  if (!matrix) return;

  constexpr int ICON_W = 21;
  constexpr int ICON_H = 7;
  constexpr int ICON_X = 116;
  constexpr int ICON_Y = 0;

  // Choose which icon to use: rain vs snow.
  // Prefer explicit precipitation type when present; otherwise use temperature as a fallback.
  bool useSnowIcon = false;
  if (gWeatherHasPrecip) {
    useSnowIcon = gWeatherPrecipIsSnow;
  } else if (gWeatherHasTemp) {
    useSnowIcon = (gWeatherTempF <= 34);
  }

  const uint8_t* iconPtr = useSnowIcon
    ? (const uint8_t*)&Snow_Array[0][0]
    : (const uint8_t*)&Rain_Array[0][0];

  // Draw icon background, skipping magenta (9) so digits can overwrite
  blitHudArraySkip9(iconPtr, ICON_W, ICON_H, ICON_X, ICON_Y);

  // Display precipitation probability percent.
  // Preserve last known value if API fetch fails or precip is missing.
  int percent = gWeatherHasPrecip ? gWeatherPrecipPercent : 0;
  if (percent < 0)   percent = 0;
  if (percent > 100) percent = 100;

  uint16_t white = matrix->color565(255, 255, 255);

  // The magenta 9's for the digits start at column index 6 inside the 21-wide block
  // and repeat every 4 pixels: 6,10,14. drawNumber3Digits() already uses a 4-pixel spacing
  // (x, x+4, x+8) so we just offset from ICON_X by +6.
  const int digitX = ICON_X + 6;
  const int digitY = ICON_Y + 1;  // center vertically inside the 7-pixel high icon

  drawNumber3Digits(digitX, digitY, percent, white);
}

// Draw the Map TIME HUD with a simple yellow countdown.
void drawHudMapTime() {
  if (!matrix) return;

  constexpr int MAPTIME_W = 19; // matches Time_Array row stride
  constexpr int MAPTIME_H = 11;
  constexpr int MAPTIME_X = 173;
  constexpr int MAPTIME_Y = 0;

  // Blit static art, skipping transparent (0) and placeholder (9).
  blitHudArraySkip9((const uint8_t*)&Time_Array[0][0], MAPTIME_W, MAPTIME_H, MAPTIME_X, MAPTIME_Y);

  int t = gMapTimeSeconds;
  if (t < 0)   t = 0;
  if (t > 999) t = 999;

  char buf[4];
  snprintf(buf, sizeof(buf), "%03d", t);

  uint16_t yellow = matrix->color565(255, 220, 70);

  // Digits aligned to the magenta zeros in the editor export.
  const int digitX = MAPTIME_X + 4; // offset from left edge of Time_Array
  const int digitY = MAPTIME_Y + 6;

  for (int i = 0; i < 3; ++i) {
    int d = buf[i] - '0';
    drawTinyGlyph(digitX + i * 4, digitY, d, yellow);
  }
}

void drawHUD() {
  if (!matrix) return;
  drawStaticHudFromEditor(); // lives art only (no digits yet)
  drawHudLives();            // Mario lives count (x 005)
  drawHudDate();             // MM/DD/YY date
  drawHudClockTime();        // HH:MM AM/PM clock
  drawHudMapTime();          // level timer
  drawHudCoins();            // coin counter
  drawHudTemp();             // temperature
  drawHudPrecip();           // rain/snow %
}

// Draw the base scene (world + HUD) without flipping the buffer.
// The caller is responsible for calling matrix->flipDMABuffer() once per frame.
void drawScene(int bobOffset, int crouchOffset) {
  if (!matrix) return;
  // NOTE: Do not mutate cameraX/cameraY here.
  // Rendering already uses integer pixel coordinates via (int)cameraX/(int)cameraY
  // at draw sites; truncating the globals here quantizes camera motion and makes
  // vertical movement feel "steppy"/wrong.

  drawBackground();

  // Draw tiles from the runtime `level[][]` grid (single source of truth).
  drawTiles();

  // Render-only overlay mobs (independent from tiles/tunnels)
  drawOverlayMobs();

  // Gameplay piranha entities (hide/rise/collide)
  drawPiranhas();

  // Brick fragment particles (broken bricks)
  drawBrickFragments();

  drawMushroom();
  drawGoombas();
  drawKoopas();
  drawShells();

  int16_t heroScreenX = (int16_t)(heroX - cameraX);
  int16_t heroScreenY = (int16_t)(heroY - cameraY + crouchOffset);

  drawHero(heroScreenX, heroScreenY, bobOffset);

  // Draw coin pop particles on top of world/hero (SMW-style)
  drawCoinPopsSMW();

  // Foreground pipe redraw to mask Mario during sink/rise
  if (heroShouldRenderBehindPipe()) {
    drawActivePipeOverHero();
  }

  // HUD overlay (coin counter, etc.) - gated to prevent flash during black screen
  if (shouldDrawHudThisFrame()) {
    drawHUD();
  }

  // NOTE: No flipDMABuffer() here - caller flips once per frame
}

// Render-only overlay mobs from mobLayer[][], bottom-aligned to their anchor tile.
// This must not modify the tile map, physics, tunnel pairing, or teleport behavior.
void drawOverlayMobs() {
  if (!matrix) return;

  const int tileMargin = 2;

  int firstTileX = (int)(cameraX / TILE_SIZE) - tileMargin;
  int firstTileY = (int)(cameraY / TILE_SIZE) - tileMargin;
  int lastTileX  = (int)((cameraX + MATRIX_WIDTH) / TILE_SIZE) + tileMargin;
  int lastTileY  = (int)((cameraY + MATRIX_HEIGHT) / TILE_SIZE) + tileMargin;

  if (firstTileX < 0) firstTileX = 0;
  if (firstTileY < 0) firstTileY = 0;
  if (lastTileX >= LEVEL_WIDTH) lastTileX = LEVEL_WIDTH - 1;
  if (lastTileY >= LEVEL_HEIGHT) lastTileY = LEVEL_HEIGHT - 1;

  for (int ty = firstTileY; ty <= lastTileY; ++ty) {
    for (int tx = firstTileX; tx <= lastTileX; ++tx) {
      uint8_t mobId = getMobId(tx, ty);
      if (mobId == 0) continue;

      // Piranha plants are handled by the entity system (so they can hide/rise).
      if (mobId == TILE_ID_MOB_PIRANHA_PLANT_L ||
          mobId == TILE_ID_MOB_PIRANHA_PLANT_R ||
          mobId == TILE_ID_MOB_PIRANHA_PLANT) {
        continue;
      }

      switch (mobId) {
        default:
          // Unknown overlay mob ID: ignore
          break;
      }
    }
  }
}

// ─────────────────────────────────────────────
// Camera update - Smooth easing with dead zone
// ─────────────────────────────────────────────
void updateCamera() {
  // Camera constants (readable/adjustable; avoid magic numbers in logic)
  const float CAMERA_SCREEN_W_PX = (float)MATRIX_WIDTH;
  const float CAMERA_SCREEN_H_PX = (float)MATRIX_HEIGHT;
  const float CAMERA_TILE_SIZE_PX = (float)TILE_SIZE;
  const float CAMERA_MAP_H_TILES = (float)LEVEL_HEIGHT;

  // Horizontal camera: preserve existing behavior
  const float CAMERA_X_LERP = 0.12f;     // lower = smoother/laggier
  const float CAMERA_X_DEADZONE_HALF_PX = 20.0f;

  // Vertical camera: baseline is the 8x8 ground row Mario stands on.
  // This keeps the spawn ground visible and aligned to the screen tiles.
  const float CAMERA_GROUND_TOP_BASELINE_SCREEN_Y = (float)((VIEW_TILES_H - 1) * TILE_SIZE);

  // Follow-up trigger: engage upward follow before Mario reaches the top edge.
  // We use a direct pixel threshold (feet-on-screen) to avoid awkward coupling
  // between baseline and trigger.
  const float CAMERA_UP_FOLLOW_TRIGGER_SCREEN_Y = 28.0f; // ~24–32px from top on a 64px screen

  // Hysteresis prevents rapid latch/unlatch near the trigger.
  const float CAMERA_UP_FOLLOW_HYSTERESIS_PX = 8.0f;

  // Small deadband prevents micro-jitter from collision rounding.
  const float CAMERA_Y_DEADBAND_PX = 0.35f;
  const float CAMERA_Y_GROUNDED_DEADBAND_PX = 0.65f;

  // When we do move vertically, ease it in to avoid twitch.
  const float CAMERA_Y_LERP_UP   = 0.20f;
  const float CAMERA_Y_LERP_DOWN = 0.24f;

  // When latched (big jump), bias camera slightly upward so you can see more above.
  // (This pushes Mario slightly lower on-screen, but keep feet visible.)
  const float CAMERA_UP_LOOK_EXTRA_PX = 7.0f; // ~1 tile more view above on a 64px screen

  // Per-frame scroll caps (px/frame).
  const float CAMERA_Y_MAX_STEP_UP_PX = 1.20f;
  const float CAMERA_Y_MAX_STEP_UP_LATCHED_PX = 1.60f;
  const float CAMERA_Y_MAX_STEP_DOWN_PX = 2.40f;
  const float CAMERA_Y_MAX_STEP_DOWN_FALLING_PX = 4.00f;

  float heroCenterX = heroX + heroW * 0.5f;
  float screenCenterX = cameraX + CAMERA_SCREEN_W_PX * 0.5f;
  
  // Calculate target camera position with dead zone
  float targetCamX;
  
  if (heroCenterX < screenCenterX - CAMERA_X_DEADZONE_HALF_PX) {
    // Hero moved left outside dead zone - camera should follow
    targetCamX = heroCenterX - (CAMERA_SCREEN_W_PX * 0.5f - CAMERA_X_DEADZONE_HALF_PX);
  } else if (heroCenterX > screenCenterX + CAMERA_X_DEADZONE_HALF_PX) {
    // Hero moved right outside dead zone - camera should follow
    targetCamX = heroCenterX - (CAMERA_SCREEN_W_PX * 0.5f + CAMERA_X_DEADZONE_HALF_PX);
  } else {
    // Hero is inside dead zone - don't change target this frame
    targetCamX = cameraX;
  }

  // Clamp target to world bounds
  if (targetCamX < 0.0f) targetCamX = 0.0f;
  float maxCamX = (float)(WORLD_WIDTH - MATRIX_WIDTH);
  if (maxCamX < 0.0f) maxCamX = 0.0f;
  if (targetCamX > maxCamX) targetCamX = maxCamX;

  // Smoothly interpolate camera toward target
  cameraX += (targetCamX - cameraX) * CAMERA_X_LERP;

  // ─────────────────────────────────────────────
  // Vertical camera update (SMW-style band / deadzone)
  // ─────────────────────────────────────────────
  static bool sPrevOnGround = false;
  const bool justLanded = (onGround && !sPrevOnGround);

  float heroFeetWorldY = heroY + heroHitH;
  float heroFeetScreenY = heroFeetWorldY - cameraY;

  // Feet deadzone bounds (screen-space, px from top).
  // Camera only moves if feet cross these boundaries.
  const float TOP_FOOT_BOUND_PX = 24.0f;
  const float BOTTOM_FOOT_BOUND_PX = 56.0f; // near bottom on a 64px screen (tile row at y=56)

  const bool airborne = !onGround;
  const bool rising = (heroVY < 0.0f);

  float targetCamY = cameraY; // default: keep camera stable
  bool allowUp = false;

  // Upward follow gating:
  // Only allow camera to move upward if Mario is airborne, rising, and feet are above top bound.
  if (airborne && rising && heroFeetScreenY < TOP_FOOT_BOUND_PX) {
    allowUp = true;
    targetCamY = heroFeetWorldY - TOP_FOOT_BOUND_PX;
  }

  // Downward follow:
  // If feet go below the bottom bound (falling/landing), move camera down to keep feet in view.
  if (heroFeetScreenY > BOTTOM_FOOT_BOUND_PX) {
    targetCamY = heroFeetWorldY - BOTTOM_FOOT_BOUND_PX;
  }

  // Grounded behavior (to stop wobble):
  // Do NOT try to keep feet on an exact baseline and do NOT snap to tiles.
  // Only allow downward correction when feet go below the bottom bound.
  if (onGround) {
    allowUp = false;
    if (heroFeetScreenY <= BOTTOM_FOOT_BOUND_PX) {
      targetCamY = cameraY;
    }
  }

  // Keep legacy flag for other helpers (spawn resets, etc.).
  gCameraUpFollowLatched = allowUp;

  // Clamp to map bounds (critical to prevent OOB reads)
  float worldH = CAMERA_MAP_H_TILES * CAMERA_TILE_SIZE_PX;
  float maxCamY = worldH - CAMERA_SCREEN_H_PX;
  if (maxCamY < 0.0f) maxCamY = 0.0f;
  if (targetCamY < 0.0f) targetCamY = 0.0f;
  if (targetCamY > maxCamY) targetCamY = maxCamY;

  // Apply eased + capped per-frame camera motion
  float dyRaw = targetCamY - cameraY;

  // Adaptive rates/caps:
  // - Small corrections stay slow/smooth.
  // - Large gaps (tall jumps / fast falls) catch up quickly.
  float absDyRaw = (dyRaw < 0.0f) ? -dyRaw : dyRaw;

  // Base lerps (will be overridden by distance buckets)
  float lerpUp = 0.12f;
  float lerpDown = 0.18f;
  if (onGround) {
    // Ground should feel stable; only allow settle-down when needed.
    lerpUp = 0.0f;
    lerpDown = justLanded ? 0.22f : 0.16f;
  }

  // Distance buckets (px)
  // Note: dyRaw < 0 means camera must move UP (smaller cameraY).
  if (dyRaw < 0.0f) {
    if (!onGround && allowUp) {
      if (absDyRaw <= 8.0f)      lerpUp = 0.14f;
      else if (absDyRaw <= 16.0f) lerpUp = 0.24f;
      else                       lerpUp = 0.36f;
    }
  } else if (dyRaw > 0.0f) {
    if (absDyRaw <= 8.0f)       lerpDown = max(lerpDown, 0.20f);
    else if (absDyRaw <= 16.0f) lerpDown = max(lerpDown, 0.28f);
    else                        lerpDown = max(lerpDown, 0.34f);
  }

  float lerp = (dyRaw < 0.0f) ? lerpUp : lerpDown;
  float dy = dyRaw * lerp;

  // Deadband: absorb rounding jitter when grounded, but don't stall meaningful settle-down.
  const float deadband = onGround ? 1.25f : 0.45f;
  if (dy > -deadband && dy < deadband) {
    if (dyRaw > 0.0f) {
      dy = (dyRaw < 0.60f) ? dyRaw : 0.60f;
    } else {
      dy = 0.0f;
    }
  }

  // Per-frame caps (adaptive). Keep upward slower than downward.
  float maxStepUp = 1.10f;
  float maxStepDown = 2.40f;

  if (dyRaw < 0.0f) {
    if (!onGround && allowUp) {
      if (absDyRaw <= 8.0f)       maxStepUp = 1.2f;
      else if (absDyRaw <= 16.0f) maxStepUp = 2.3f;
      else                        maxStepUp = 3.6f;
    } else {
      // Not allowed to move up.
      maxStepUp = 0.0f;
    }
  } else if (dyRaw > 0.0f) {
    if (absDyRaw <= 8.0f)       maxStepDown = 1.6f;
    else if (absDyRaw <= 16.0f) maxStepDown = 2.8f;
    else                        maxStepDown = 4.2f;
  }

  if (onGround) {
    // Ground: never move up, only settle down.
    maxStepUp = 0.0f;
    maxStepDown = justLanded ? 2.60f : 1.40f;
  }
  float maxStep = (dyRaw < 0.0f) ? maxStepUp : maxStepDown;
  if (dyRaw > 0.0f && heroVY > 0.0f) {
    // Falling: allow a faster down catch-up.
    if (maxStep < 4.80f) maxStep = 4.80f;
  }
  if (dy >  maxStep) dy =  maxStep;
  if (dy < -maxStep) dy = -maxStep;
  cameraY += dy;

  // Final clamp (guarantee cameraY is always in-bounds)
  if (cameraY < 0.0f) cameraY = 0.0f;
  if (cameraY > maxCamY) cameraY = maxCamY;

  // No post-quantization here; rendering already uses integer pixel coords at draw sites.
  // Quantizing cameraY causes visible stepping during vertical motion.

  sPrevOnGround = onGround;
}

// ─────────────────────────────────────────────
// Input decoding from Serial1
// ─────────────────────────────────────────────
void readInputFromSerial() {
  // Reset per-frame signal; will be set if any parsed packet indicates human input.
  gHumanInputThisFrame = false;

  bool gotPacketThisFrame = false;

  while (Serial1.available() >= 5) {  // 5 bytes: header + 4-byte packet
    int header = Serial1.read();
    if (header != 0xAA) {
      // out-of-sync byte
      continue;
    }

    gotPacketThisFrame = true;

    uint8_t buf[4];
    Serial1.readBytes(buf, 4);

    InputPacket pkt = {0};  // Initialize to zero (adminArmRequest defaults to 0)
    memcpy(&pkt, buf, 4);

    gLastControllerPacketMs = millis();
    gControllerConnected = (pkt.flags & CTRL_FLAG_CONNECTED) != 0;

    // Compute "human input" on the S3 side for robustness.
    // Rationale: the controller-side flag can be overly sensitive (analog jitter / chord frames),
    // which would immediately cancel playback and/or constantly suppress auto-play.
    const int moveDeadzone = 8; // filters tiny noise
    const bool hasHuman = (abs((int)pkt.moveX) > moveDeadzone) || (pkt.buttons != 0) || (pkt.adminArmRequest != 0);
    if (hasHuman) gHumanInputThisFrame = true;

    gInput.rawMoveX = pkt.moveX;
    float axis = (float)pkt.moveX / 127.0f;
    if (axis >  1.0f) axis =  1.0f;
    if (axis < -1.0f) axis = -1.0f;
    gInput.moveAxis = axis;

    uint8_t b = pkt.buttons;
    gInput.btnJump  = b & 0x01;
    gInput.btnSpin  = b & 0x02;
    gInput.btnRun   = b & 0x04;
    gInput.btnDown  = b & 0x08;
    gInput.btnUp    = b & 0x40;  // ADDED: D-Pad UP navigation
    gInput.btnReset = b & 0x10;
    gInput.btnStart = b & 0x20;

#if DEBUG_CTRL
    // ═══════════════════════════════════════════════════════════════
    // COMPREHENSIVE CONTROLLER INPUT DEBUG (throttled to 1000ms)
    // ═══════════════════════════════════════════════════════════════
    static uint32_t lastDebugMs = 0;
    if (millis() - lastDebugMs > 1000) {
      lastDebugMs = millis();
      
      // Extract individual button states from bitfield
      bool up     = (b & 0x40) != 0;  // bit6
      bool down   = (b & 0x08) != 0;  // bit3
      bool left   = pkt.moveX < -64;  // approximate from analog
      bool right  = pkt.moveX > 64;   // approximate from analog
      bool select = (b & 0x10) != 0;  // bit4 - SELECT/BACK/MINUS
      bool start  = (b & 0x20) != 0;  // bit5 - START/PLUS
      bool btnA   = (b & 0x01) != 0;  // bit0 - Jump
      bool btnB   = (b & 0x02) != 0;  // bit1 - Spin
      bool btnX   = (b & 0x04) != 0;  // bit2 - Run (X or Y on controller)
      
      Serial.printf("[CTRL] up=%u dn=%u lt=%u rt=%u  sel=%u start=%u  A=%u B=%u X=%u  moveX=%d  armReq=%u\n",
        up, down, left, right,
        select, start,
        btnA, btnB, btnX,
        pkt.moveX,
        pkt.adminArmRequest
      );
    }
#endif

#if DEBUG_ADMIN
    // ═══════════════════════════════════════════════════════════════
    // TEMPORARY: S3-SIDE HOLD DETECTION (for proving button mapping)
    // ═══════════════════════════════════════════════════════════════
    // This will be moved to controller-side once mapping is confirmed
    static uint32_t holdStart = 0;
    static bool armSent = false;
    
    bool selectPressed = (b & 0x10) != 0;  // bit4 - SELECT button
    bool upPressed     = (b & 0x40) != 0;  // bit6 - D-Pad UP
    bool comboActive   = selectPressed && upPressed;
    
    if (comboActive) {
      if (holdStart == 0) {
        holdStart = millis();
        Serial.println("[ADMIN] SELECT+UP combo detected, starting 5s timer...");
      }
      
      if (!armSent && (millis() - holdStart >= 5000)) {
        armSent = true;
        armOTA();
        Serial.println("[ADMIN] OTA ARMED via SELECT+UP hold (10 minutes)");
      }
    } else {
      // Reset when either button released
      if (holdStart != 0) {
        Serial.println("[ADMIN] SELECT+UP combo released, resetting timer");
      }
      holdStart = 0;
      armSent = false;
    }
#endif

    // ═══════════════════════════════════════════════════════════════
    // Controller-side arm request (final design, currently being tested)
    // ═══════════════════════════════════════════════════════════════
    // Only arm on rising edge (0→1) to prevent spam-arming every frame
    static uint8_t prevArmRequest = 0;
    if (pkt.adminArmRequest == 1 && prevArmRequest == 0) {
      armOTA();
#if DEBUG_ADMIN
      Serial.println("[ADMIN] OTA ARMED via controller packet (10 minutes)");
#endif
    }
    prevArmRequest = pkt.adminArmRequest;

#ifdef DEBUG_INPUT
    static int8_t  lastMoveX   = 0;
    static uint8_t lastButtons = 0;
    if (pkt.moveX != lastMoveX || b != lastButtons) {
      Serial.print("[S3] RX moveX=");
      Serial.print(pkt.moveX);
      Serial.print(" buttons=0x");
      Serial.println(b, HEX);
      lastMoveX   = pkt.moveX;
      lastButtons = b;
    }
#endif
  }

  // If the controller stops sending packets, clear stale inputs so Mario doesn't keep walking.
  // This is especially important when switching into attract mode.
  const uint32_t nowMs = millis();
  if (!gotPacketThisFrame && gLastControllerPacketMs != 0 && (nowMs - gLastControllerPacketMs) > CONTROLLER_PACKET_TIMEOUT_MS) {
    gControllerConnected = false;
    gHumanInputThisFrame = false;
    gInput = {0, 0.0f, 0,0,0,0,0,0,0};
  }
}

// ─────────────────────────────────────────────
// Demo / Auto-play mode
// ─────────────────────────────────────────────
static uint8_t gAutoJumpHoldFrames = 0;
static uint8_t gAutoJumpCooldownFrames = 0;

static void resetAutoPlayState() {
  gAutoJumpHoldFrames = 0;
  gAutoJumpCooldownFrames = 0;
}

static void setControlMode(ControlMode mode) {
  if (gControlMode == mode) return;
  gControlMode = mode;
  resetAutoPlayState();
}

static void stopAttractPlayback() {
  gDemoPlaybackActive = false;
  gDemoPlaybackArmed = false;
  gDemoPlaybackPlaying = false;
  gDemoPlaybackArmedMapId = 0xFF;
  gDemoPlaybackStableFrames = 0;
  gDemoPlaybackIdx = 0;
  gAttractLoadedMapId = 0xFF;

  gAttractSessionActive = false;
  gAttractPendingSinceMs = 0;

  // Cancel any in-progress attract intro fade.
  gAttractIntroState = ATTRACT_INTRO_NONE;
  gAttractIntroStartMs = 0;
  // Ensure we don't leave a partial fade on-screen.
  if (gPipeFadeAmount > 0) gPipeFadeAmount = 0;
}

static void startAttractLoopFromBeginning(uint32_t nowMs) {
  // Always start the unattended loop at Map 1, with a fade-out/fade-in cue.
  stopAttractPlayback();
  gAttractIntroState = ATTRACT_INTRO_FADE_OUT;
  gAttractIntroStartMs = nowMs;
  gAttractIntroHoldBlackMs = 200;
  // Ensure the fade is visually noticeable immediately.
  gPipeFadeAmount = 12;

  gAttractSessionActive = true;
}

// After an AUTO_PLAY demo clears the map, restart the attract intro from a clean baseline.
// We are already black (flagpole fade-out), so we just hold black for a bit, then reset and fade in.
static void restartAttractLoopAfterAutoClear(uint32_t nowMs) {
  // Stop any demo playback state, but keep the screen black.
  gDemoPlaybackActive = false;
  gDemoPlaybackArmed = false;
  gDemoPlaybackPlaying = false;
  gDemoPlaybackArmedMapId = 0xFF;
  gDemoPlaybackStableFrames = 0;
  gDemoPlaybackIdx = 0;
  gAttractLoadedMapId = 0xFF;

  // End the flagpole cutscene immediately so it doesn't fight the intro.
  resetFlagpoleState();

  // Start intro at HOLD_BLACK (we're already at full black).
  gAttractIntroState = ATTRACT_INTRO_HOLD_BLACK;
  gAttractIntroStartMs = nowMs;
  gAttractIntroHoldBlackMs = ATTRACT_LOOP_RESTART_HOLD_BLACK_MS;
  gPipeFadeAmount = 255;

  // Consider this a continuing attract session.
  gAttractSessionActive = true;
}

static void tickAttractIntro(uint32_t nowMs) {
  if (gAttractIntroState == ATTRACT_INTRO_NONE) return;

  const uint32_t FADE_MS = 450;

  switch (gAttractIntroState) {
    case ATTRACT_INTRO_FADE_OUT: {
      const uint32_t t = nowMs - gAttractIntroStartMs;
      if (t <= FADE_MS) {
        // Monotonic fade-out: never decrease (prevents a 1-frame flash at start).
        uint8_t target = (uint8_t)map((int)t, 0, (int)FADE_MS, 0, 255);
        if (target < gPipeFadeAmount) target = gPipeFadeAmount;
        gPipeFadeAmount = target;
      } else {
        gPipeFadeAmount = 255;
        gAttractIntroState = ATTRACT_INTRO_HOLD_BLACK;
        gAttractIntroStartMs = nowMs;
      }
    } break;

    case ATTRACT_INTRO_HOLD_BLACK: {
      gPipeFadeAmount = 255;
      const uint32_t t = nowMs - gAttractIntroStartMs;
      if (t >= gAttractIntroHoldBlackMs) {
        // Reset to a fresh world right as we transition into fade-in.
        gCurrentMapId = MAP_ID_OVERWORLD_1;
        gGameMode = GAME_MODE_PLAYING;
        gPauseScreen = PAUSE_SCREEN_MAIN;
        gPauseSelection = PAUSE_RESET;

        // New attract session / map start: reset demo timebase so time-based systems
        // match the recording from the first frame.
        demoSimReset();
        gLastMapTimeTickMs = gameplayNowMs();
        Serial.println("[DEMO] demoSimReset (attract intro map start)");

        resetWorld(true);

        // resetWorld(true) clears pipe transition state, which can zero gPipeFadeAmount.
        // Ensure we stay fully black until FADE_IN begins to avoid a 1-frame flash.
        gPipeFadeAmount = 255;

        // Force re-load of the map's official recording after fade-in completes.
        gAttractLoadedMapId = 0xFF;
        gDemoPlaybackActive = false;
        gDemoPlaybackArmed = false;
        gDemoPlaybackPlaying = false;
        gDemoPlaybackArmedMapId = 0xFF;
        gDemoPlaybackStableFrames = 0;
        gDemoPlaybackIdx = 0;

        gInput = {0, 0.0f, 0,0,0,0,0,0,0};
        gHumanInputThisFrame = false;
        resetInputEdgeDetectionState();

        gAttractIntroState = ATTRACT_INTRO_FADE_IN;
        gAttractIntroStartMs = nowMs;
        // Restore default hold-black duration for the next normal attract entry.
        gAttractIntroHoldBlackMs = 200;
      }
    } break;

    case ATTRACT_INTRO_FADE_IN: {
      const uint32_t t = nowMs - gAttractIntroStartMs;
      if (t <= FADE_MS) {
        gPipeFadeAmount = (uint8_t)map((int)t, 0, (int)FADE_MS, 255, 0);
      } else {
        gPipeFadeAmount = 0;
        gAttractIntroState = ATTRACT_INTRO_NONE;
        gAttractIntroStartMs = 0;
      }
    } break;

    default:
      gAttractIntroState = ATTRACT_INTRO_NONE;
      gAttractIntroStartMs = 0;
      gPipeFadeAmount = 0;
      break;
  }
}

static void tickAttractDriver(uint32_t nowMs) {
  (void)nowMs;
  if (gControlMode != AUTO_PLAY) return;
  if (gGameMode != GAME_MODE_PLAYING) return;
  if (attractIntroActive()) return; // wait until fade-in completes

  // AUTO_PLAY must always run Map 1 only.
  if (gCurrentMapId != MAP_ID_OVERWORLD_1) {
    startAttractLoopFromBeginning(nowMs);
    return;
  }

  // Load the appropriate official recording for the current map (once per map).
  if (gAttractLoadedMapId != gCurrentMapId) {
    gAttractLoadedMapId = gCurrentMapId;
    const bool has = demoLoadOfficialRecordingForMap(gCurrentMapId);
    if (has) {
      gDemoPlaybackIdx = 0;
      gDemoPlaybackActive = true;
      gDemoPlaybackArmed = true;
      gDemoPlaybackPlaying = false;
      gDemoPlaybackArmedMapId = gCurrentMapId;
      gDemoPlaybackStableFrames = 0;

      // Map-start baseline: prevent stale edge-detection state from previous map.
      gInput = {0, 0.0f, 0,0,0,0,0,0,0};
      gHumanInputThisFrame = false;
      resetInputEdgeDetectionState();
      Serial.print("[DEMO] Using official recording for mapId=");
      Serial.println(gCurrentMapId);
    } else {
      gDemoPlaybackIdx = 0;
      gDemoPlaybackActive = false;
      gDemoPlaybackArmed = false;
      gDemoPlaybackPlaying = false;
      gDemoPlaybackArmedMapId = 0xFF;
      gDemoPlaybackStableFrames = 0;
      Serial.print("[DEMO] No official recording for mapId=");
      Serial.print(gCurrentMapId);
      Serial.println(" (fallback heuristic)");
    }
  }
}

static void updateControlMode(uint32_t nowMs) {
  bool connected = gControllerConnected;
  if (gLastControllerPacketMs != 0 && (nowMs - gLastControllerPacketMs) > CONTROLLER_PACKET_TIMEOUT_MS) {
    connected = false;
  }

  // Any human input immediately hands over control (same frame).
  if (gHumanInputThisFrame) {
    gLastHumanInputMs = nowMs;
    if (gControlMode != PLAYER) {
      stopAttractPlayback();
    }
    gAttractPendingSinceMs = 0;
    setControlMode(PLAYER);
    return;
  }

  // Determine whether conditions indicate we should be in attract mode.
  // Note: we *delay* entering attract mode to get the requested sequencing.
  bool wantsAttract = false;

  if (!connected) {
    wantsAttract = true;
  } else if (gLastHumanInputMs == 0) {
    // Connected but no human input since boot.
    wantsAttract = true;
  } else if ((nowMs - gLastHumanInputMs) > AUTO_PLAY_IDLE_TIMEOUT_MS) {
    wantsAttract = true;
  }

  // If attract is already running, don't restart it due to transient packet timeouts.
  if (wantsAttract && gControlMode == AUTO_PLAY && gAttractSessionActive) {
    return;
  }

  if (wantsAttract) {
    if (gAttractPendingSinceMs == 0) gAttractPendingSinceMs = nowMs;

    // Wait for sustained no-human-input before starting the fade+reset+demo.
    if ((nowMs - gAttractPendingSinceMs) < ATTRACT_START_DELAY_MS) {
      // Stay in PLAYER mode during the waiting window.
      if (gControlMode != PLAYER) {
        stopAttractPlayback();
        setControlMode(PLAYER);
      }
      return;
    }

    // Time elapsed: enter attract with the fade cue.
    gAttractPendingSinceMs = 0;
    if (gControlMode != AUTO_PLAY) {
      setControlMode(AUTO_PLAY);
    }
    if (!gAttractSessionActive) {
      startAttractLoopFromBeginning(nowMs);
    }
    return;
  }

  // Otherwise we should be in PLAYER mode.
  gAttractPendingSinceMs = 0;
  if (gControlMode != PLAYER) {
    stopAttractPlayback();
    setControlMode(PLAYER);
  }
  return;
}

static inline bool autoSolidAtPixel(int px, int py) {
  int tx = px >> 3;
  int ty = py >> 3;
  if (tx < 0 || ty < 0 || tx >= LEVEL_WIDTH || ty >= LEVEL_HEIGHT) return false;
  uint8_t id = getTileId(tx, ty);
  return isSolidTileId(id);
}

static void generateAutoInput(InputState &out) {
  out.rawMoveX = 127;
  out.moveAxis = 1.0f;
  out.btnJump  = 0;
  out.btnSpin  = 0;
  out.btnRun   = 1;
  out.btnDown  = 0;
  out.btnUp    = 0;
  out.btnReset = 0;
  out.btnStart = 0;

  // Simple forward probes (tile-based, deterministic)
  const int frontX = (int)(heroX + heroW + 1.0f);
  const int feetY  = (int)(heroY + heroHitH - 1.0f);      // last pixel of hitbox
  const int oneUpY = feetY - TILE_SIZE;                   // one tile above feet
  const int belowFeetY = (int)(heroY + heroHitH + 1.0f);  // inside the tile underfoot

  const bool blockedAheadAtFeet = autoSolidAtPixel(frontX, feetY);
  const bool blockedAheadOneUp  = autoSolidAtPixel(frontX, oneUpY);
  const bool obstacleAhead = blockedAheadAtFeet || blockedAheadOneUp;

  const int step1X = (int)(heroX + heroW + (1 * TILE_SIZE));
  const int step2X = (int)(heroX + heroW + (2 * TILE_SIZE));
  const bool groundStep1 = autoSolidAtPixel(step1X, belowFeetY);
  const bool groundStep2 = autoSolidAtPixel(step2X, belowFeetY);
  const bool gapAhead = (!groundStep1) || (!groundStep2);

  if (gAutoJumpCooldownFrames > 0) gAutoJumpCooldownFrames--;
  if (gAutoJumpHoldFrames > 0) {
    // Hold jump a bit while rising for a more reliable jump arc.
    if (heroVY < 0.0f) {
      out.btnJump = 1;
      gAutoJumpHoldFrames--;
    } else {
      gAutoJumpHoldFrames = 0;
    }
  }

  if (onGround && gAutoJumpCooldownFrames == 0) {
    if (obstacleAhead || gapAhead) {
      out.btnJump = 1;
      gAutoJumpHoldFrames = 6;
      gAutoJumpCooldownFrames = 10;
    }
  }
}

// ─────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────
// Wi‑Fi/time helpers (definitions placed here so setup()/loop() can call them)

// Non-blocking WiFi initialization - call once in setup()
void initWiFi() {
  Serial.println("[WiFi] Starting non-blocking WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin();  // Attempt to connect to saved credentials
  
  gWiFiState = WIFI_TRY_STA;
  gWiFiStateStartMs = millis();
  gWiFiConnected = false;
}

// WiFi state machine - call every loop() to process WiFi in background
void tickWiFi() {
  switch (gWiFiState) {
    
    case WIFI_INIT:
      // Should not reach here (initWiFi sets to TRY_STA)
      break;
    
    case WIFI_TRY_STA: {
      // Try to connect to saved WiFi for 15 seconds
      if (WiFi.status() == WL_CONNECTED) {
        // Connected successfully!
        gWiFiState = WIFI_CONNECTED;
        gWiFiConnected = true;
        WiFi.setSleep(false);
        Serial.printf("[WiFi] Connected! Open: http://%s/\n", WiFi.localIP().toString().c_str());
        
        // Show IP on matrix for 15 seconds
        gShowIPOnMatrix = true;
        gIPDisplayUntil = millis() + 15000;
        
        // Load weather settings from NVS and start weather/time services
        loadWeatherKey("wifi connected");
        loadLocation();
        syncTimeFromNTP();
        startWeatherIfReady();
        
        startWebServer();
        break;
      }
      
      // Timeout after 15 seconds - start captive portal
      if (millis() - gWiFiStateStartMs > 15000) {
        Serial.println("[WiFi] No saved WiFi found. Starting captive portal...");
        startCaptivePortal();
        gWiFiState = WIFI_PORTAL;
        gWiFiStateStartMs = millis();
      }
      break;
    }
    
    case WIFI_PORTAL: {
      // Process captive portal while game runs
      if (gWiFiManager) {
        gWiFiManager->process();  // Non-blocking portal processing
        
        // Check if WiFi connected via portal
        if (WiFi.status() == WL_CONNECTED) {
          // First frame of successful connection via portal
          if (!gWiFiConnected) {
            Serial.printf("[WiFi] Portal success! Open: http://%s/\n", WiFi.localIP().toString().c_str());

            gWiFiConnected = true;
            WiFi.setSleep(false);

            // Show IP on matrix
            gShowIPOnMatrix = true;
            gIPDisplayUntil = millis() + 15000;

            // Load weather settings from NVS and start services
            loadWeatherKey("wifi connected");
            loadLocation();
            syncTimeFromNTP();
            startWeatherIfReady();

            // Configure the portal success page and keep portal alive briefly
            showPortalSuccessPage();
            gPortalSuccessActive = true;
            gPortalSuccessUntilMs = millis() + 15000;

            startWebServer();
          }

          // After a short window, shut down the captive portal and move on.
          if (gPortalSuccessActive && (millis() > gPortalSuccessUntilMs)) {
            Serial.println("[WiFi] Closing captive portal after success window.");
            gPortalSuccessActive = false;

            if (gWiFiManager) {
              delete gWiFiManager;
              gWiFiManager = nullptr;
            }

            if (gWiFiPortalHeaderParam) {
              delete gWiFiPortalHeaderParam;
              gWiFiPortalHeaderParam = nullptr;
            }

            gWiFiState = WIFI_CONNECTED;
          }
        }
      }
      break;
    }
    
    case WIFI_CONNECTED:
      // WiFi is connected, nothing to do
      // (Web server handles requests via server.handleClient() in loop)
      break;
    
    case WIFI_OFFLINE:
      // Running without WiFi, nothing to do
      break;
  }
  
  // Clear IP display after timeout
  if (gShowIPOnMatrix && millis() > gIPDisplayUntil) {
    gShowIPOnMatrix = false;
  }
}

// Start captive portal (non-blocking setup)
void startCaptivePortal() {
  if (gWiFiManager) {
    delete gWiFiManager;
  }

  if (gWiFiPortalHeaderParam) {
    delete gWiFiPortalHeaderParam;
    gWiFiPortalHeaderParam = nullptr;
  }
  
  gWiFiManager = new WiFiManager();
  gWiFiManager->setDebugOutput(true);
  WiFi.mode(WIFI_AP_STA);  // Allow AP and STA simultaneously

  
  // ───────────────────────────────────────────────────────────────
  // BRANDING + DARK THEME (match main MarioMatrix portal styling)
  // ───────────────────────────────────────────────────────────────
  
  // Set portal title and restrict menu to ONLY wifi config and restart (STABLE)
  gWiFiManager->setTitle("MarioMatrix Setup");
  std::vector<const char*> menu = {"wifi","restart"};  // NO info, param, erase, update, sep, or ota
  gWiFiManager->setMenu(menu);
  gWiFiManager->setClass("invert"); // enables dark-ish base styles in WiFiManager
  
  // Inject custom CSS for dark theme + HIDE any Update/OTA/Erase/Info UI elements (CSS ONLY - no server access)
  gWiFiManager->setCustomHeadElement(
    "<style>"
    ":root{color-scheme:dark;}"
    "body{background:#0b0f14 !important; color:#e6edf3 !important; font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu !important;}"
    "h1,h2,h3{color:#e6edf3 !important; letter-spacing:.2px;}"
    ".wrap,.container,div{box-sizing:border-box;}"
    "form{max-width:860px !important; margin:16px auto !important; padding:0 14px !important;}"
    "fieldset{border:0 !important; padding:0 !important;}"
    "legend{display:none !important;}"
    "table{width:100% !important; border-collapse:separate !important; border-spacing:0 10px !important;}"
    "tr{background:#101826 !important; border:1px solid #1f2a3a !important; border-radius:16px !important; overflow:hidden !important;}"
    "td{padding:12px 12px !important; border:0 !important;}"
    "input,select{width:100% !important; padding:10px 12px !important; border-radius:12px !important; border:1px solid #24344d !important; background:#0f1725 !important; color:#e6edf3 !important; font-size:15px !important;}"
    "button,input[type=submit],.btn{border-radius:12px !important; padding:10px 14px !important; font-weight:700 !important; border:1px solid #2b3e62 !important; background:#1b2a44 !important; color:#e6edf3 !important;}"
    "button:hover,input[type=submit]:hover{background:#223457 !important;}"
    "a{color:#9cc2ff !important;}"
    ".msg{opacity:.85 !important;}"
    "#footer{opacity:.75 !important;}"
    /* SECURITY: Hide all Update/OTA/Erase/Info UI elements from captive portal (CSS ONLY - SAFE) */
    "a[href='/i'],a[href='/info'],a[href='/u'],a[href*='/erase'],a[href*='erase'],a[href*='update'],a[href*='param']{display:none!important;}"
    "button[name='update'],button[name='erase'],button[value*='Update'],button[value*='Erase'],input[value*='Update'],input[value*='Erase'],form[action='/u']{display:none!important;}"
    ".button.erase,.button.update,.button.info{display:none!important;}"
    /* HIDE WiFiManager status banners and "No AP set" messages */
    ".msg,.alert,.warning,.infomsg,.status,#msg,.infobox{display:none!important;}"
    "div.msg,span.msg,p.msg,div.infobox{display:none!important;}"
    "</style>"
  );
  
  // Add friendly instruction card at top of portal.
  // IMPORTANT: WiFiManager stores the pointer; this must outlive startCaptivePortal().
  gWiFiPortalHeaderParam = new WiFiManagerParameter(
    "<div style='max-width:860px;margin:18px auto 10px;padding:0 14px;'>"
    "  <div style='background:#101826;border:1px solid #1f2a3a;border-radius:16px;padding:16px;box-shadow:0 6px 24px rgba(0,0,0,.35);'>"
    "    <div style='font-size:28px;font-weight:800;margin-bottom:6px;'>🍄 MarioMatrix Setup</div>"
    "    <div style='opacity:.82;font-size:14px;line-height:1.4;'>"
    "      Choose your Wi-Fi network and enter the password.<br>"
    "      After setup, the Matrix will show its IP address (example: <b>http://192.168.1.246</b>) to open the control page."
    "    </div>"
    "  </div>"
    "</div>"
  );
  gWiFiManager->addParameter(gWiFiPortalHeaderParam);

  // ───────────────────────────────────────────────────────────────
  // NON-BLOCKING PORTAL CONFIGURATION
  // ───────────────────────────────────────────────────────────────

  gWiFiManager->setConfigPortalTimeout(0);  // No timeout - portal runs until connected
  gWiFiManager->setConnectTimeout(15);      // 15s to try each network

  // IMPORTANT: WiFiManager's startConfigPortal() is blocking by default.
  // Enable non-blocking mode so the game loop keeps running while the portal is open.
  gWiFiManager->setConfigPortalBlocking(false);
  
  // Start portal in non-blocking mode
  if (AP_PASS && strlen(AP_PASS) >= 8) {
    gWiFiManager->startConfigPortal(AP_NAME, AP_PASS);
  } else {
    gWiFiManager->startConfigPortal(AP_NAME);  // Open AP
  }
  
  Serial.println("[WiFi] Captive portal started (non-blocking). Connect to: " + String(AP_NAME));
}

// Show success page after portal connection (called once, then portal stops)
void showPortalSuccessPage() {
  if (!gWiFiManager || !gWiFiManager->server) return;
  
  String staIP = WiFi.localIP().toString();
  String ssid = WiFi.SSID();
  
  Serial.println("[WiFi] Serving success page for 15 seconds...");
  
  WebServer* successServer = gWiFiManager->server.get();
  if (successServer) {

    // Override root page with success message
    successServer->on("/", [staIP, ssid, successServer]() {
          String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
          // Best-effort auto-redirect (often fails in captive portal browsers)
          html += "<meta http-equiv='refresh' content='5;url=http://" + staIP + "/'>";
          html += "<title>Connected - MarioMatrix</title>";
          html += "<style>";
          html += ":root{color-scheme:dark;}";
          html += "body{background:#0b0f14;color:#e6edf3;font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu;margin:0;padding:20px;text-align:center;}";
          html += "h1{color:#e6edf3;margin-top:40px;font-size:32px;}";
          html += ".success{font-size:80px;margin:20px 0;animation:bounce 0.6s;}";
          html += "@keyframes bounce{0%,100%{transform:scale(1);} 50%{transform:scale(1.1);}}";
          html += ".card{background:#101826;border:1px solid #1f2a3a;border-radius:16px;padding:28px;margin:24px auto;max-width:600px;text-align:left;}";
          html += ".kv{display:grid;grid-template-columns:120px 1fr;gap:14px;align-items:center;margin:20px 0;}";
          html += ".kv>div:nth-child(odd){opacity:.7;font-size:15px;}";
          html += ".kv>div:nth-child(even){font-weight:600;font-family:monospace;color:#9cc2ff;font-size:16px;}";
          html += ".btn{display:inline-block;padding:18px 32px;background:#1b8f44;border:3px solid #2ea05a;border-radius:14px;color:#fff;text-decoration:none;font-weight:700;font-size:20px;margin-top:24px;box-shadow:0 4px 12px rgba(27,143,68,.4);}";
          html += ".btn:hover{background:#229955;transform:translateY(-2px);box-shadow:0 6px 16px rgba(27,143,68,.6);}";
          html += ".instructions{background:#1a2332;border:1px solid #2b3e62;border-radius:12px;padding:16px;margin-top:24px;font-size:14px;line-height:1.6;opacity:.9;}";
          html += ".url{font-family:monospace;font-size:18px;font-weight:700;color:#9cc2ff;margin:12px 0;padding:12px;background:#0f1725;border-radius:8px;border:1px solid #24344d;}";
          html += "</style>";
          html += "<script>";
          html += "setTimeout(()=>{";
          html += "  try{location.href='http://" + staIP + "/';}catch(e){console.log('Auto-redirect failed (expected in captive portal)');}";
          html += "}, 5000);";
          html += "</script>";
          html += "</head><body>";
          html += "<div class='success'>✅</div>";
          html += "<h1>WiFi Connected!</h1>";
          html += "<div class='card'>";
          html += "<div class='kv'>";
          html += "<div>Network</div><div>" + ssid + "</div>";
          html += "<div>IP Address</div><div>" + staIP + "</div>";
          html += "</div>";
          html += "<div class='url'>http://" + staIP + "/</div>";
          html += "<center><a href='http://" + staIP + "/' class='btn'>🍄 Open MarioMatrix Portal</a></center>";
          html += "<div class='instructions'>";
          html += "<strong>📱 Next Steps:</strong><br>";
          html += "1. Close this captive portal window<br>";
          html += "2. Disconnect from <b>MarioMatrix-Setup</b> network<br>";
          html += "3. Connect to your WiFi network (<b>" + ssid + "</b>)<br>";
          html += "4. Open a browser and go to: <b>http://" + staIP + "/</b>";
          html += "</div>";
          html += "</div>";
          html += "<div style='opacity:.5;margin-top:40px;font-size:13px;'>Portal will close in 15 seconds...</div>";
          html += "</body></html>";
          
      successServer->send(200, "text/html", html);
    });
    
    // NOTE: Do not block here. tickWiFi() keeps calling gWiFiManager->process()
    // while gPortalSuccessActive is true, and shuts down the portal after the timeout.
    Serial.println("[WiFi] Success page configured at 192.168.4.1 (non-blocking). ");
  }
}

// ─────────────────────────────────────────────
// Web Server + OTA Setup
// ─────────────────────────────────────────────
void startWebServer() {
  if (gWebServerStarted) return;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WEB] Not starting server (no Wi-Fi).");
    return;
  }

  // mDNS for web portal (mario-matrix.local)
  if (MDNS.begin(DEVICE_NAME)) {
    Serial.printf("[mDNS] Web portal: http://%s.local/\n", DEVICE_NAME);
  } else {
    Serial.println("[mDNS] Failed to start.");
  }

  // ═══════════════════════════════════════════
  // PUBLIC ROUTES (pretty, safe, no admin mention)
  // ═══════════════════════════════════════════

  server.on("/", HTTP_GET, []() {
    if (!requirePublicAuth()) return;
    
    String html = htmlHeader("MarioMatrix", "status");
    
    html += "<div class='card'>";
    html += "<h2>Status</h2>";
    html += "<div class='kv'>";
    html += "<div>Version</div><div><span class='pill'>" + fwVersion() + "</span></div>";
    html += "<div>IP Address</div><div>" + WiFi.localIP().toString() + "</div>";
    html += "<div>Wi-Fi SSID</div><div>" + String(WiFi.SSID()) + "</div>";
    
    unsigned long upSec = millis() / 1000;
    unsigned long upMin = upSec / 60;
    unsigned long upHr  = upMin / 60;
    String uptime = String(upHr) + "h " + String(upMin % 60) + "m " + String(upSec % 60) + "s";
    html += "<div>Uptime</div><div>" + uptime + "</div>";
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>🌤️ Weather Location</h2>";
    html += "<form method='POST' action='/public/weather'>";
    html += "<div style='margin-bottom:12px;'>";
    html += "<label for='lat' style='display:block;margin-bottom:6px;font-weight:600;'>Latitude:</label>";
    html += "<input type='number' id='lat' name='lat' step='0.000001' value='" + String(gLatitude, 6) + "' style='width:100%;padding:10px;border-radius:8px;border:1px solid #24344d;background:#0f1725;color:#e6edf3;' required>";
    html += "</div>";
    html += "<div style='margin-bottom:12px;'>";
    html += "<label for='lon' style='display:block;margin-bottom:6px;font-weight:600;'>Longitude:</label>";
    html += "<input type='number' id='lon' name='lon' step='0.000001' value='" + String(gLongitude, 6) + "' style='width:100%;padding:10px;border-radius:8px;border:1px solid #24344d;background:#0f1725;color:#e6edf3;' required>";
    html += "</div>";
    html += "<div style='opacity:.7;font-size:13px;margin-bottom:16px;'>Valid ranges: Latitude -90 to 90, Longitude -180 to 180</div>";
    html += "<button type='submit' class='btn' style='background:#1b8f44;border-color:#2ea05a;'>💾 Save Location</button>";
    html += "</form>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>Actions</h2>";
    html += "<div class='row'>";
    html += "<a href='/reboot' class='btn'>Reboot</a>";
    html += "<a href='/wipewifi' class='btn danger'>Wipe Wi-Fi</a>";
    html += "</div>";
    html += "<p class='muted'>Tip: After wiping Wi-Fi, reconnect to AP: " + String(AP_NAME) + "</p>";
    html += "</div>";
    
    html += htmlFooter();
    server.send(200, "text/html", html);
  });

  server.on("/reboot", HTTP_GET, []() {
    String html = htmlHeader("Rebooting");
    html += "<div class='card'><h2>Rebooting...</h2><p>Device will restart in a moment.</p></div>";
    html += htmlFooter();
    server.send(200, "text/html", html);
    delay(250);
    ESP.restart();
  });

  server.on("/info", HTTP_GET, []() {
    if (!requirePublicAuth()) return;
    
    String html = htmlHeader("System Info", "info");
    
    html += "<div class='card'>";
    html += "<h2>Device Information</h2>";
    html += "<div class='kv'>";
    html += "<div>Hostname</div><div>" + String(DEVICE_NAME) + "</div>";
    html += "<div>Firmware</div><div><span class='pill'>" + fwVersion() + "</span></div>";
    html += "<div>Chip Model</div><div>" + String(ESP.getChipModel()) + "</div>";
    html += "<div>CPU Frequency</div><div>" + String(getCpuFrequencyMhz()) + " MHz</div>";
    html += "<div>Flash Size</div><div>" + String(ESP.getFlashChipSize() / 1024 / 1024) + " MB</div>";
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>Network</h2>";
    html += "<div class='kv'>";
    html += "<div>IP Address</div><div>" + WiFi.localIP().toString() + "</div>";
    html += "<div>MAC Address</div><div>" + WiFi.macAddress() + "</div>";
    html += "<div>Wi-Fi SSID</div><div>" + String(WiFi.SSID()) + "</div>";
    html += "<div>RSSI</div><div>" + String(WiFi.RSSI()) + " dBm</div>";
    html += "<div>Gateway</div><div>" + WiFi.gatewayIP().toString() + "</div>";
    html += "<div>DNS</div><div>" + WiFi.dnsIP().toString() + "</div>";
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>System Status</h2>";
    html += "<div class='kv'>";
    
    unsigned long upSec = millis() / 1000;
    unsigned long upMin = upSec / 60;
    unsigned long upHr  = upMin / 60;
    unsigned long upDay = upHr / 24;
    String uptime;
    if (upDay > 0) {
      uptime = String(upDay) + "d " + String(upHr % 24) + "h " + String(upMin % 60) + "m";
    } else {
      uptime = String(upHr) + "h " + String(upMin % 60) + "m " + String(upSec % 60) + "s";
    }
    html += "<div>Uptime</div><div>" + uptime + "</div>";
    
    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t totalHeap = ESP.getHeapSize();
    uint32_t usedHeap = totalHeap - freeHeap;
    html += "<div>Free Heap</div><div>" + String(freeHeap / 1024) + " KB (" + String((freeHeap * 100) / totalHeap) + "% free)</div>";
    html += "<div>Used Heap</div><div>" + String(usedHeap / 1024) + " KB</div>";
    html += "<div>Min Free Heap</div><div>" + String(esp_get_minimum_free_heap_size() / 1024) + " KB</div>";
    
    // PSRAM info (ESP32-S3 specific)
    if (ESP.getPsramSize() > 0) {
      uint32_t freePsram = ESP.getFreePsram();
      uint32_t totalPsram = ESP.getPsramSize();
      html += "<div>PSRAM Total</div><div>" + String(totalPsram / 1024 / 1024) + " MB</div>";
      html += "<div>PSRAM Free</div><div>" + String(freePsram / 1024 / 1024) + " MB (" + String((freePsram * 100) / totalPsram) + "% free)</div>";
    }
    
    html += "</div>";
    html += "</div>";
    
    html += htmlFooter();
    server.send(200, "text/html", html);
  });

  server.on("/reboot", HTTP_GET, []() {
    if (!requirePublicAuth()) return;
    
    String html = htmlHeader("Rebooting");
    html += "<div class='card'><h2>Rebooting...</h2><p>Device will restart in a moment.</p></div>";
    html += htmlFooter();
    server.send(200, "text/html", html);
    delay(250);
    ESP.restart();
  });

  server.on("/wipewifi", HTTP_GET, []() {
    if (!requirePublicAuth()) return;
    
    String html = htmlHeader("Wiping Wi-Fi");
    html += "<div class='card'><h2>Wiping Wi-Fi credentials...</h2>";
    html += "<p>Device will reboot. Reconnect to AP: <b>" + String(AP_NAME) + "</b></p></div>";
    html += htmlFooter();
    server.send(200, "text/html", html);
    delay(250);
    WiFi.disconnect(true, true); // erase creds
    delay(250);
    ESP.restart();
  });
  
  // Public Weather Location Update (lat/long only, NO API key access)
  server.on("/public/weather", HTTP_POST, []() {
    if (!requirePublicAuth()) return;
    
    if (server.hasArg("lat") && server.hasArg("lon")) {
      float lat = server.arg("lat").toFloat();
      float lon = server.arg("lon").toFloat();
      
      // Validate ranges
      if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
        saveLocation(lat, lon);
        
        // Trigger immediate weather fetch if WiFi connected and key set
        if (gWiFiConnected && hasWeatherKey()) {
          gWeatherReady = true;
          gLastWeatherFetchMs = 0;  // Force immediate fetch
          Serial.println("[Weather] Public location updated: fetching now...");
        }
        
        server.sendHeader("Location", "/");
        server.send(302, "text/plain", "Location saved. Redirecting...");
      } else {
        server.send(400, "text/plain", "Invalid coordinates. Lat: -90 to 90, Lon: -180 to 180");
      }
    } else {
      server.send(400, "text/plain", "Missing lat/lon parameters");
    }
  });

  // ═══════════════════════════════════════════
  // HIDDEN ADMIN ROUTES (requires auth + arming)
  // ═══════════════════════════════════════════

  // Admin landing page: /<SECRET>/
  server.on(adminHome().c_str(), HTTP_GET, []() {
    if (!requireAuth()) return;
    
    String html = htmlHeader("Admin Panel");
    html += "<h1>🔐 Admin Panel</h1>";    
    
    // Add tab navigation
    html += "<div class='tabs'>";
    html += "<a href='" + adminHome() + "' class='active'>Status</a>";
    html += "<a href='" + adminRoot() + "/info'>Info</a>";
    html += "<a href='" + adminRoot() + "/weather'>Weather</a>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>OTA Status</h2>";
    if (isOTAArmed()) {
      uint32_t sec = otaSecondsRemaining();
      uint32_t min = sec / 60;
      html += "<p>✅ <b>OTA Armed</b> — " + String(min) + "m " + String(sec % 60) + "s remaining</p>";
      html += "<div class='row'>";
      html += "<a href='" + adminUpdate() + "' class='btn success'>Firmware Update</a>";
      html += "<a href='" + adminOff() + "' class='btn danger'>Disarm</a>";
      html += "</div>";
    } else {
      html += "<p>🔒 <b>OTA Not Armed</b></p>";
      html += "<p class='muted'>Hold SELECT + UP on controller for 5 seconds to arm OTA for 10 minutes.</p>";
      html += "<button type='button' onclick='location.reload()' class='btn'>Refresh</button>";
    }
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>Quick Actions</h2>";
    html += "<div class='row'>";
    html += "<a href='/' class='btn'>Public Portal</a>";
    html += "<a href='/reboot' class='btn'>Reboot</a>";
    html += "</div>";
    html += "</div>";
    
    html += htmlFooter();
    server.send(200, "text/html", html);
  });

  // Disarm OTA: /<SECRET>/off
  server.on(adminOff().c_str(), HTTP_GET, []() {
    if (!requireAuth()) return;
    disarmOTA();
    server.sendHeader("Location", adminHome());
    server.send(302, "text/plain", "Redirecting...");
  });

  // Admin Info page: /<SECRET>/info (authenticated, comprehensive system data)
  server.on((adminRoot() + "/info").c_str(), HTTP_GET, []() {
    if (!requireAuth()) return;
    
    String html = htmlHeader("Admin Info");
    html += "<h1>🔐 Admin Panel</h1>";
    
    // Add tab navigation
    html += "<div class='tabs'>";
    html += "<a href='" + adminHome() + "'>Status</a>";
    html += "<a href='" + adminRoot() + "/info' class='active'>Info</a>";
    html += "<a href='" + adminRoot() + "/weather'>Weather</a>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>Device Information</h2>";
    html += "<div class='kv'>";
    html += "<div>Hostname</div><div>" + String(DEVICE_NAME) + "</div>";
    html += "<div>Firmware</div><div><span class='pill'>" + fwVersion() + "</span></div>";
    html += "<div>Chip Model</div><div>" + String(ESP.getChipModel()) + "</div>";
    html += "<div>CPU Frequency</div><div>" + String(getCpuFrequencyMhz()) + " MHz</div>";
    html += "<div>Flash Size</div><div>" + String(ESP.getFlashChipSize() / 1024 / 1024) + " MB</div>";
    
    uint32_t sketchSize = ESP.getSketchSize();
    uint32_t freeSketch = ESP.getFreeSketchSpace();
    html += "<div>Sketch Size</div><div>" + String(sketchSize / 1024) + " KB</div>";
    html += "<div>Free Sketch Space</div><div>" + String(freeSketch / 1024) + " KB (" + String((freeSketch * 100) / (sketchSize + freeSketch)) + "% free)</div>";
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>Network</h2>";
    html += "<div class='kv'>";
    html += "<div>IP Address</div><div>" + WiFi.localIP().toString() + "</div>";
    html += "<div>MAC Address</div><div>" + WiFi.macAddress() + "</div>";
    html += "<div>Wi-Fi SSID</div><div>" + String(WiFi.SSID()) + "</div>";
    html += "<div>RSSI</div><div>" + String(WiFi.RSSI()) + " dBm</div>";
    html += "<div>Gateway</div><div>" + WiFi.gatewayIP().toString() + "</div>";
    html += "<div>DNS</div><div>" + WiFi.dnsIP().toString() + "</div>";
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>System Status</h2>";
    html += "<div class='kv'>";
    
    unsigned long upSec = millis() / 1000;
    unsigned long upMin = upSec / 60;
    unsigned long upHr  = upMin / 60;
    unsigned long upDay = upHr / 24;
    String uptime;
    if (upDay > 0) {
      uptime = String(upDay) + "d " + String(upHr % 24) + "h " + String(upMin % 60) + "m";
    } else {
      uptime = String(upHr) + "h " + String(upMin % 60) + "m " + String(upSec % 60) + "s";
    }
    html += "<div>Uptime</div><div>" + uptime + "</div>";
    
    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t totalHeap = ESP.getHeapSize();
    uint32_t usedHeap = totalHeap - freeHeap;
    html += "<div>Free Heap</div><div>" + String(freeHeap / 1024) + " KB (" + String((freeHeap * 100) / totalHeap) + "% free)</div>";
    html += "<div>Used Heap</div><div>" + String(usedHeap / 1024) + " KB</div>";
    html += "<div>Min Free Heap</div><div>" + String(esp_get_minimum_free_heap_size() / 1024) + " KB</div>";
    
    // PSRAM info (ESP32-S3 specific)
    if (ESP.getPsramSize() > 0) {
      uint32_t freePsram = ESP.getFreePsram();
      uint32_t totalPsram = ESP.getPsramSize();
      html += "<div>PSRAM Total</div><div>" + String(totalPsram / 1024 / 1024) + " MB</div>";
      html += "<div>PSRAM Free</div><div>" + String(freePsram / 1024 / 1024) + " MB (" + String((freePsram * 100) / totalPsram) + "% free)</div>";
    }
    
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>Quick Actions</h2>";
    html += "<div class='row'>";
    html += "<a href='" + adminHome() + "' class='btn'>Back to Status</a>";
    html += "<a href='/' class='btn'>Public Portal</a>";
    html += "</div>";
    html += "</div>";
    
    html += htmlFooter();
    server.send(200, "text/html", html);
  });

  // Admin Weather API Key Management: /<SECRET>/weather
  server.on((adminRoot() + "/weather").c_str(), HTTP_GET, []() {
    if (!requireAuth()) return;
    
    // Handle clear action
    if (server.hasArg("clear")) {
      clearWeatherKey();
      server.sendHeader("Location", adminRoot() + "/weather");
      server.send(302, "text/plain", "Redirecting...");
      return;
    }
    
    String html = htmlHeader("Weather Config");
    html += "<h1>🔐 Admin Panel</h1>";
    
    // Add tab navigation
    html += "<div class='tabs'>";
    html += "<a href='" + adminHome() + "'>Status</a>";
    html += "<a href='" + adminRoot() + "/info'>Info</a>";
    html += "<a href='" + adminRoot() + "/weather' class='active'>Weather</a>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>🌤️ Weather API Configuration</h2>";
    html += "<div class='kv'>";
    html += "<div>Status</div><div>";
    if (hasWeatherKey()) {
      html += "<span style='color:#2ea05a;'>✅ Key is set</span>";
    } else {
      html += "<span style='color:#d4693f;'>❌ No key set</span>";
    }
    html += "</div>";
    html += "<div>Location</div><div>" + String(gLatitude, 6) + ", " + String(gLongitude, 6) + "</div>";
    html += "<div>Provider</div><div>Google Weather API</div>";
    html += "</div>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>Set/Update API Key</h2>";
    html += "<form method='POST' action='" + adminRoot() + "/weather'>";
    html += "<div style='margin-bottom:12px;'>";
    html += "<label for='apikey' style='display:block;margin-bottom:6px;font-weight:600;'>Google Weather API Key:</label>";
    html += "<input type='text' id='apikey' name='apikey' placeholder='AIzaSy...' style='width:100%;padding:10px;border-radius:8px;border:1px solid #24344d;background:#0f1725;color:#e6edf3;font-family:monospace;' required>";
    html += "</div>";
    html += "<div style='opacity:.7;font-size:13px;margin-bottom:16px;'>Get your API key from <a href='https://console.cloud.google.com/apis/credentials' target='_blank' style='color:#9cc2ff;'>Google Cloud Console</a> with Weather API enabled.</div>";
    html += "<button type='submit' class='btn' style='background:#1b8f44;border-color:#2ea05a;'>💾 Save Key</button>";
    html += "</form>";
    html += "</div>";
    
    html += "<div class='card'>";
    html += "<h2>📍 Location Settings</h2>";
    html += "<form method='POST' action='" + adminRoot() + "/weather'>";
    html += "<div style='margin-bottom:12px;'>";
    html += "<label for='lat' style='display:block;margin-bottom:6px;font-weight:600;'>Latitude:</label>";
    html += "<input type='number' id='lat' name='lat' step='0.000001' value='" + String(gLatitude, 6) + "' style='width:100%;padding:10px;border-radius:8px;border:1px solid #24344d;background:#0f1725;color:#e6edf3;' required>";
    html += "</div>";
    html += "<div style='margin-bottom:12px;'>";
    html += "<label for='lon' style='display:block;margin-bottom:6px;font-weight:600;'>Longitude:</label>";
    html += "<input type='number' id='lon' name='lon' step='0.000001' value='" + String(gLongitude, 6) + "' style='width:100%;padding:10px;border-radius:8px;border:1px solid #24344d;background:#0f1725;color:#e6edf3;' required>";
    html += "</div>";
    html += "<div style='opacity:.7;font-size:13px;margin-bottom:16px;'>Valid ranges: Latitude -90 to 90, Longitude -180 to 180</div>";
    html += "<button type='submit' class='btn' style='background:#1b8f44;border-color:#2ea05a;'>💾 Save Location</button>";
    html += "</form>";
    html += "</div>";
    
    if (hasWeatherKey()) {
      html += "<div class='card'>";
      html += "<h2>Danger Zone</h2>";
      html += "<p style='opacity:.8;margin-bottom:12px;'>Remove the stored API key. Weather data will stop updating.</p>";
      html += "<a href='" + adminRoot() + "/weather?clear=1' class='btn' style='background:#a03030;border-color:#c04040;' onclick='return confirm(\"Clear weather API key?\");'>🗑️ Clear Key</a>";
      html += "</div>";
    }
    
    html += "<div class='card'>";
    html += "<h2>Quick Actions</h2>";
    html += "<div class='row'>";
    html += "<a href='" + adminHome() + "' class='btn'>Back to Status</a>";
    html += "<a href='/' class='btn'>Public Portal</a>";
    html += "</div>";
    html += "</div>";
    
    html += htmlFooter();
    server.send(200, "text/html", html);
  });
  
  // Handle weather API key and location POST
  server.on((adminRoot() + "/weather").c_str(), HTTP_POST, []() {
    if (!requireAuth()) return;
    
    // Handle API key save
    if (server.hasArg("apikey")) {
      String key = server.arg("apikey");
      key.trim();
      
      if (key.length() > 0) {
        saveWeatherKey(key);
        
        // Trigger immediate weather fetch if WiFi connected
        if (gWiFiConnected && hasWeatherKey()) {
          gWeatherReady = true;
          gLastWeatherFetchMs = 0;  // Force immediate fetch
          Serial.println("[Weather] Admin save: fetching now...");
        }
      }
    }
    
    // Handle location save
    if (server.hasArg("lat") && server.hasArg("lon")) {
      float lat = server.arg("lat").toFloat();
      float lon = server.arg("lon").toFloat();
      
      // Validate ranges
      if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
        saveLocation(lat, lon);
        
        // Trigger immediate weather fetch if WiFi connected and key set
        if (gWiFiConnected && hasWeatherKey()) {
          gWeatherReady = true;
          gLastWeatherFetchMs = 0;  // Force immediate fetch
          Serial.println("[Weather] Location updated: fetching now...");
        }
      } else {
        Serial.println("[Weather] Invalid lat/lon range rejected");
      }
    }
    
    server.sendHeader("Location", adminRoot() + "/weather");
    server.send(302, "text/plain", "Redirecting...");
  });

  // Manual OTA Upload: /<SECRET>/update
  server.on(adminUpdate().c_str(), HTTP_GET, []() {
    if (!requireAuth()) return;
    
    if (!isOTAArmed()) {
      server.send(403, "text/plain", "OTA not armed. Hold SELECT+UP on controller for 5s.");
      return;
    }
    
    String html = htmlHeader("OTA Firmware Update");
    html += "<h1>⚠️ Firmware Update</h1>";
    html += "<div class='card'>";
    html += "<h2>Upload New Firmware</h2>";
    html += "<p class='muted'>Select a .bin file compiled for ESP32-S3.</p>";
    html += "<form method='POST' action='" + adminUpdate() + "' enctype='multipart/form-data'>";
    html += "<input type='file' name='update' accept='.bin' required style='margin:10px 0;display:block;'>";
    html += "<button type='submit' class='btn success'>Upload & Flash</button>";
    html += "</form>";
    html += "</div>";
    html += "<div class='card'>";
    html += "<a href='" + adminHome() + "' class='btn'>← Back to Admin</a>";
    html += "</div>";
    html += htmlFooter();
    server.send(200, "text/html", html);
  });

  server.on(adminUpdate().c_str(), HTTP_POST, []() {
    if (!requireAuth()) return;
    
    if (!isOTAArmed()) {
      server.send(403, "text/plain", "OTA not armed.");
      return;
    }
    
    // Upload complete callback
    if (Update.hasError()) {
      String html = htmlHeader("Update Failed");
      html += "<div class='card'>";
      html += "<h2>❌ Update Failed</h2>";
      html += "<p>Error: " + String(Update.errorString()) + "</p>";
      html += "<a href='" + adminHome() + "' class='btn'>← Back</a>";
      html += "</div>";
      html += htmlFooter();
      server.send(500, "text/html", html);
    } else {
      String html = htmlHeader("Update Success");
      html += "<div class='card'>";
      html += "<h2>✅ Update Complete</h2>";
      html += "<p>Firmware uploaded successfully. Rebooting...</p>";
      html += "</div>";
      html += htmlFooter();
      server.send(200, "text/html", html);
      delay(500);
      ESP.restart();
    }
  }, []() {
    // Upload handler (called repeatedly during upload)
    HTTPUpload& upload = server.upload();
    
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("[OTA] Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("[OTA] Update Success: %u bytes\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  // 404 handler
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not found");
  });

  server.begin();
  gWebServerStarted = true;
  Serial.println("[WEB] Server started on port 80");
  Serial.println("[WEB] Public portal: http://" + WiFi.localIP().toString() + "/");
  Serial.println("[WEB] Admin panel: " + adminHome() + " (hidden)");
}

// Service web server frequently without lagging gameplay
inline void tickWebServer() {
  if (gWebServerStarted) {
    server.handleClient();
  } else {
    // If Wi-Fi comes up later, start web server then
    if (WiFi.status() == WL_CONNECTED) startWebServer();
  }
}

// NTP time sync
void syncTimeFromNTP() {
  if (!gWifiConnected) {
    Serial.println("[Time] Cannot sync: no Wi-Fi.");
    return;
  }

  Serial.println("[Time] Syncing from NTP...");
  configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVER);

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 5000)) {  // 5s timeout
    Serial.println("[Time] Failed to obtain time from NTP.");
    gTimeSynced = false;
    return;
  }

  gTimeSynced = true;
  gLastTimeSyncMs = millis();

  Serial.print("[Time] NTP sync OK: ");
  Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
}

// ─────────────────────────────────────────────
// Weather parsers and fetch helper (lightweight, string-based)
// ─────────────────────────────────────────────
static void logJsonSnippet(const String& json, int idx, int radius) {
  if (idx < 0) return;
  int start = idx - radius;
  if (start < 0) start = 0;
  int end = idx + radius;
  if (end > (int)json.length()) end = (int)json.length();
  Serial.printf("[Weather] JSON snippet @%d: ", idx);
  Serial.println(json.substring(start, end));
}

static bool parseNumericPercentAfterIndex(const String& json, int startIdx, int& outPercent) {
  if (startIdx < 0) return false;

  int percentIdx = json.indexOf("\"percent\"", startIdx);
  if (percentIdx < 0) return false;

  int colon = json.indexOf(':', percentIdx);
  if (colon < 0) return false;

  int start = colon + 1;
  while (start < (int)json.length() && (json[start] == ' ' || json[start] == '\t')) start++;

  int end = start;
  while (end < (int)json.length() && ((json[end] >= '0' && json[end] <= '9') || json[end] == '.')) end++;
  if (end <= start) return false;

  String s = json.substring(start, end);
  if (s.length() == 0) return false;

  float v = s.toFloat();

  // If this ever comes back as 0.0..1.0, treat it as probability and convert.
  int pct;
  if (v > 0.0f && v <= 1.0f) pct = (int)roundf(v * 100.0f);
  else                       pct = (int)roundf(v);

  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  outPercent = pct;
  return true;
}

static bool parseTemperatureF(const String& json, int& outTempF) {
  // Google Weather API (forecast/hours:lookup) returns JSON like:
  // {
  //   "forecastHours": [
  //     {
  //       "temperature": {
  //         "unit": "FAHRENHEIT",
  //         "degrees": 42.3
  //       },
  //       ...
  //     },
  //     ...
  //   ]
  // }
  //
  // We just grab the first "temperature.degrees" we see.

  // Prefer the intended path (forecastHours[0].temperature.degrees), but be tolerant of
  // schema variations by falling back to maxTemperature/minTemperature from days:lookup.
  int idx = json.indexOf("\"forecastHours\"");
  if (idx >= 0) idx = json.indexOf("\"temperature\"", idx);
  if (idx >= 0) idx = json.indexOf("\"degrees\"", idx);

  // Fallback: days:lookup max/min temperature.
  if (idx < 0) {
    idx = json.indexOf("\"maxTemperature\"");
    if (idx >= 0) idx = json.indexOf("\"degrees\"", idx);
  }
  if (idx < 0) {
    idx = json.indexOf("\"minTemperature\"");
    if (idx >= 0) idx = json.indexOf("\"degrees\"", idx);
  }
  if (idx < 0) return false;

  // Move to the ':' after "degrees"
  idx = json.indexOf(':', idx);
  if (idx < 0) return false;

  // Skip spaces
  int start = idx + 1;
  while (start < (int)json.length() &&
         (json[start] == ' ' || json[start] == '\t')) {
    start++;
  }

  // Read numeric value
  int end = start;
  while (end < (int)json.length() &&
         ((json[end] >= '0' && json[end] <= '9') || json[end] == '.' || json[end] == '-')) {
    end++;
  }

  if (end <= start) return false;

  String tempStr = json.substring(start, end);
  float tempVal = tempStr.toFloat();
  if (tempStr.length() == 0) return false;

  // Try to detect unit near the temperature block.
  int unitIdx = json.indexOf("\"unit\"", max(0, idx - 150));
  String unit = "?";
  if (unitIdx >= 0) {
    int q1 = json.indexOf('"', unitIdx + 6);
    if (q1 >= 0) {
      int q2 = json.indexOf('"', q1 + 1);
      if (q2 > q1) {
        unit = json.substring(q1 + 1, q2);
      }
    }
  }

  if (DEBUG_WEATHER) {
    Serial.printf("[Weather] Parsed temp: %.1f %s\n", tempVal, unit.c_str());
  }

  // We request unitsSystem=IMPERIAL, but be defensive in case a different unit appears.
  if (unit.indexOf("CELSIUS") >= 0 || unit.indexOf("celsius") >= 0) {
    tempVal = (tempVal * 9.0f / 5.0f) + 32.0f;
  }
  outTempF = (int)roundf(tempVal);
  return true;
}

// Parse Google Weather condition type
static int parseConditionCode(const String& json) {
  // Google Weather API returns a block like:
  // "weatherCondition": {
  //   "iconBaseUri": "...",
  //   "description": { "text": "Cloudy", "languageCode": "en" },
  //   "type": "CLOUDY"
  // }
  //
  // We'll read the "type" string and map it to a small code.

  int idx = json.indexOf("\"weatherCondition\"");
  if (idx < 0) return 0;

  idx = json.indexOf("\"type\"", idx);
  if (idx < 0) return 0;

  // Find the colon after "type"
  int colonIdx = json.indexOf(':', idx);
  if (colonIdx < 0) return 0;
  
  // Find the first quote after the colon (start of value)
  int start = json.indexOf('"', colonIdx);
  if (start < 0) return 0;
  start++;  // Move past the opening quote
  
  // Find the closing quote
  int end = json.indexOf('"', start);
  if (end < 0) return 0;

  String typeStr = json.substring(start, end);
  typeStr.toLowerCase();

  if (typeStr.indexOf("clear") >= 0)      return 1; // clear
  if (typeStr.indexOf("cloud") >= 0)      return 2; // cloudy
  if (typeStr.indexOf("rain") >= 0)       return 3; // rain
  if (typeStr.indexOf("snow") >= 0)       return 4; // snow
  if (typeStr.indexOf("storm") >= 0 || typeStr.indexOf("thunder") >= 0) return 5; // thunder / storm

  // Fallback: check description if needed
  int descIdx = json.indexOf("\"description\"", idx);
  if (descIdx >= 0) {
    int textIdx = json.indexOf("\"text\"", descIdx);
    if (textIdx >= 0) {
      int q1 = json.indexOf('"', textIdx + 6);
      if (q1 >= 0) {
        int q2 = json.indexOf('"', q1 + 1);
        if (q2 > q1) {
          String desc = json.substring(q1 + 1, q2);
          desc.toLowerCase();
          if (desc.indexOf("clear") >= 0)   return 1;
          if (desc.indexOf("cloud") >= 0)   return 2;
          if (desc.indexOf("rain") >= 0)    return 3;
          if (desc.indexOf("snow") >= 0)    return 4;
          if (desc.indexOf("storm") >= 0 || desc.indexOf("thunder") >= 0) return 5;
        }
      }
    }
  }

  return 0; // unknown/other
}

// Parse precipitation probability percent from JSON.
// Primary target: precipitation.probability.percent
// Fallbacks: precipitationProbability.percent, probabilityOfPrecipitation.percent
// Returns true if it found a percent. Decides rain vs snow using type (if present) and temperature.
static bool parsePrecipitation(const String& json, int ambientTempF,
                               int& outPercent, bool& outIsSnow) {
  outPercent = 0;
  outIsSnow = (ambientTempF <= 34);

  int percentVal = -1;
  int anchorIdx = -1;

  // Prefer forecastDays/daytimeForecast if present, since Google Console examples
  // and many responses expose precip probability there:
  // forecastDays[0].daytimeForecast.precipitation.probability.percent
  int dayAnchorIdx = -1;
  {
    int fd = json.indexOf("\"forecastDays\"");
    if (fd >= 0) {
      int dt = json.indexOf("\"daytimeForecast\"", fd);
      if (dt >= 0) {
        int p = json.indexOf("\"precipitation\"", dt);
        if (p >= 0) {
          int prob = json.indexOf("\"probability\"", p);
          if (prob >= 0 && parseNumericPercentAfterIndex(json, prob, percentVal)) {
            dayAnchorIdx = prob;
          } else if (parseNumericPercentAfterIndex(json, p, percentVal)) {
            dayAnchorIdx = p;
          }
        }
      }
    }
  }
  if (dayAnchorIdx >= 0) {
    anchorIdx = dayAnchorIdx;
  }

  // Attempt 1: precipitation -> probability -> percent
  if (percentVal < 0) {
    int idx = json.indexOf("\"precipitation\"");
    if (idx >= 0) {
      int probIdx = json.indexOf("\"probability\"", idx);
      if (probIdx >= 0 && parseNumericPercentAfterIndex(json, probIdx, percentVal)) {
        anchorIdx = probIdx;
      } else if (parseNumericPercentAfterIndex(json, idx, percentVal)) {
        anchorIdx = idx;
      }
    }
  }

  // Attempt 2: precipitationProbability
  if (percentVal < 0) {
    int pp = json.indexOf("\"precipitationProbability\"");
    if (pp >= 0 && parseNumericPercentAfterIndex(json, pp, percentVal)) {
      anchorIdx = pp;
    }
  }

  // Attempt 3: probabilityOfPrecipitation
  if (percentVal < 0) {
    int pop = json.indexOf("\"probabilityOfPrecipitation\"");
    if (pop >= 0 && parseNumericPercentAfterIndex(json, pop, percentVal)) {
      anchorIdx = pop;
    }
  }

  if (percentVal < 0) return false;
  outPercent = percentVal;

  // If there is a "type" near the precipitation block, prefer it for rain vs snow.
  int typeIdx = -1;
  if (anchorIdx >= 0) {
    // Constrain the search window so we don't accidentally grab unrelated "type" fields
    // (e.g., weatherCondition.type).
    const int searchEnd = min((int)json.length(), anchorIdx + 200);
    typeIdx = json.indexOf("\"type\"", anchorIdx);
    if (typeIdx >= searchEnd) typeIdx = -1;
  }
  if (typeIdx >= 0) {
    int q1 = json.indexOf('"', typeIdx + 6);
    if (q1 >= 0) {
      int q2 = json.indexOf('"', q1 + 1);
      if (q2 > q1) {
        String typeStr = json.substring(q1 + 1, q2);
        typeStr.toLowerCase();

        bool hasRain = (typeStr.indexOf("rain") >= 0);
        bool hasSnow = (typeStr.indexOf("snow") >= 0);

        if (hasSnow && !hasRain) outIsSnow = true;
        else if (hasRain && !hasSnow) outIsSnow = false;
        else if (hasRain && hasSnow) outIsSnow = (ambientTempF <= 34);
      }
    }
  }

  if (DEBUG_WEATHER) {
    Serial.printf("[Weather] Precip parse path: %s (anchor=%d pct=%d)\n",
                  (dayAnchorIdx >= 0) ? "forecastDays/daytimeForecast" : "generic",
                  anchorIdx,
                  outPercent);
  }

  return true;
}

// Fetch current weather via HTTPS
// Uses existing Wi-Fi connection (gWifiConnected)
// Updates gHudWeather and logs a single line on success
void fetchWeatherFromAPI() {
  if (!gWifiConnected) {
    // Don't change gWeatherHasTemp or gWeatherTempF - keep last value
    return;
  }
  
  if (!hasWeatherKey()) {
    // No API key configured - skip silently (already logged once at boot)
    // Don't change gWeatherHasTemp or gWeatherTempF - keep last value
    return;
  }

  Serial.println("[Weather] Fetching...");
  Serial.printf("[Weather] Using saved key: YES (len=%u)\n", (unsigned)gWeatherApiKey.length());
  Serial.printf("[Weather] Using saved location: %.6f, %.6f\n", gLatitude, gLongitude);

  if (DEBUG_WEATHER) {
    Serial.printf("[Weather] Free heap: %u bytes\n", (unsigned)ESP.getFreeHeap());
#if defined(ESP32)
    Serial.printf("[Weather] Free PSRAM: %u bytes\n", (unsigned)ESP.getFreePsram());
#endif
  }

  // Single request: days:lookup provides precip + max/min temp.
  String url = "https://";
  url += WEATHER_HOST;
  url += "/v1/forecast/days:lookup";
  url += "?key=" + gWeatherApiKey;
  url += "&location.latitude=" + String(gLatitude, 6);
  url += "&location.longitude=" + String(gLongitude, 6);
  url += "&unitsSystem=IMPERIAL";
  url += "&days=1";

  // Log URL without key
  if (DEBUG_WEATHER) {
    String logUrl = url;
    int keyIdx = logUrl.indexOf("?key=");
    if (keyIdx >= 0) {
      int keyEnd = logUrl.indexOf('&', keyIdx);
      if (keyEnd > keyIdx) {
        logUrl = logUrl.substring(0, keyIdx + 5) + "***" + logUrl.substring(keyEnd);
      }
    }
    Serial.print("[Weather] URL: ");
    Serial.println(logUrl);
  }

  WiFiClientSecure client;
  configureWeatherTlsClient(client);
  WeatherCompat::setSNIIfSupported(client, WEATHER_HOST);

  HTTPClient https;
  https.setTimeout(8000);
  if (!https.begin(client, url)) {
    Serial.println("[Weather] HTTPS begin() failed.");
    markWeatherFetchAttemptNow();
    return;
  }
  https.addHeader("Accept", "application/json");
  https.addHeader("Accept-Encoding", "identity");

  if (DEBUG_WEATHER) {
    Serial.printf("[Weather] Heap before GET: %u bytes\n", (unsigned)ESP.getFreeHeap());
#if defined(ESP32)
    Serial.printf("[Weather] PSRAM before GET: %u bytes\n", (unsigned)ESP.getFreePsram());
#endif
  }

  int httpCode = https.GET();
  if (DEBUG_WEATHER) {
    Serial.printf("[Weather] HTTP code: %d\n", httpCode);
    Serial.printf("[Weather] HTTP code str: %s\n", https.errorToString(httpCode).c_str());
    Serial.printf("[Weather] Heap after GET: %u bytes\n", (unsigned)ESP.getFreeHeap());
#if defined(ESP32)
    Serial.printf("[Weather] PSRAM after GET: %u bytes\n", (unsigned)ESP.getFreePsram());
#endif
  }

  if (httpCode != 200) {
    Serial.printf("[Weather] HTTP GET failed: %d\n", httpCode);
    if (DEBUG_WEATHER) {
      Serial.printf("[Weather] HTTP fail str: %s\n", https.errorToString(httpCode).c_str());
    }
    https.end();
    markWeatherFetchAttemptNow();
    return;
  }

  String body = https.getString();
  https.end();

  if (DEBUG_WEATHER) {
    Serial.println("[Weather] Raw JSON (trimmed):");
    Serial.println(body.substring(0, min((size_t)500, body.length())));
    Serial.println("=== END RESPONSE ===");
  }

  int tempF;
  if (!parseTemperatureF(body, tempF)) {
    Serial.println("[Weather] Failed to parse temperature.");
    markWeatherFetchAttemptNow();
    return;
  }

  int condCode = parseConditionCode(body);

  int precipPercent = 0;
  bool precipIsSnow = false;
  bool hasPrecip = parsePrecipitation(body, tempF, precipPercent, precipIsSnow);

  if (DEBUG_WEATHER) {
    Serial.printf("[Weather] Parsed precip: has=%d pct=%d isSnow=%d (temp=%dF cond=%d)\n",
                  hasPrecip ? 1 : 0, precipPercent, precipIsSnow ? 1 : 0, tempF, condCode);
  }
  
  // Store temperature in global HUD state
  gWeatherTempF = tempF;
  gWeatherHasTemp = true;

  // Store precipitation only when explicitly present. Otherwise preserve last value.
  if (hasPrecip) {
    gWeatherPrecipPercent = precipPercent;
    gWeatherPrecipIsSnow  = precipIsSnow;
    gWeatherHasPrecip     = true;
  }

  // Update timestamp after successful fetch
  gLastWeatherFetchMs = millis();
  
  Serial.printf("[Weather] Parsed: temp=%dF cond=%d\n", tempF, condCode);
  if (gWeatherHasPrecip) {
    Serial.printf("[Weather] HUD precip: %d%% (%s)\n",
                  gWeatherPrecipPercent,
                  gWeatherPrecipIsSnow ? "snow" : "rain");
  } else {
    Serial.println("[Weather] HUD precip: (missing)");
  }
  Serial.printf("[Weather] Temp stored: %dF\n", tempF);
}

static void weatherFetchTask(void *param) {
  (void)param;
  fetchWeatherFromAPI();
  gWeatherFetchInProgress = false;
  vTaskDelete(nullptr);
}

static void startWeatherFetchAsync() {
  if (gWeatherFetchInProgress) return;
  gWeatherFetchInProgress = true;
  gWeatherFetchStartedMs = millis();

  Serial.println("[Weather] Fetch triggered");

  BaseType_t ok = xTaskCreatePinnedToCore(
    weatherFetchTask,
    "weather",
    8192,
    nullptr,
    1,
    nullptr,
    0);

  if (ok != pdPASS) {
    // Fall back to synchronous fetch with bounded IO.
    Serial.println("[Weather] WARN: task start failed; running sync");
    fetchWeatherFromAPI();
    gWeatherFetchInProgress = false;
  }
}

// Periodic clock poll -> fills gHudClock
static void updateHudClockFromRTC() {
  if (!gTimeSynced) {
    gHudClock.valid = false;
    return;
  }

  unsigned long nowMs = millis();
  if (nowMs - gLastClockPollMs < CLOCK_POLL_INTERVAL_MS) {
    return; // only update once per second
  }
  gLastClockPollMs = nowMs;

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    gHudClock.valid = false;
    return;
  }

  gHudClock.year   = timeinfo.tm_year + 1900;
  gHudClock.month  = timeinfo.tm_mon + 1;
  gHudClock.day    = timeinfo.tm_mday;
  gHudClock.hour   = timeinfo.tm_hour;
  gHudClock.minute = timeinfo.tm_min;
  gHudClock.second = timeinfo.tm_sec;
  gHudClock.valid  = true;
}

// Network/time tick - call from loop()
void tickNetworkAndTime() {
  // Service web server (non-blocking)
  tickWebServer();
  
  unsigned long nowMs = millis();

  // Safety: if a weather task ever wedges, allow future attempts.
  if (gWeatherFetchInProgress && (uint32_t)(nowMs - gWeatherFetchStartedMs) > 15000) {
    Serial.println("[Weather] WARN: fetch stuck >15s, releasing lock");
    gWeatherFetchInProgress = false;
    markWeatherFetchAttemptNow();
  }

  // NTP resync (every 6h)
  if (gWifiConnected && (!gTimeSynced ||
      (nowMs - gLastTimeSyncMs > TIME_RESYNC_INTERVAL_MS))) {
    syncTimeFromNTP();
  }

  // Update our HUD clock struct from RTC (no Serial spam)
  updateHudClockFromRTC();

  // Weather fetch every WEATHER_FETCH_INTERVAL_MS (only if ready + WiFi + key)
  if (gWeatherReady && WiFi.status() == WL_CONNECTED && hasWeatherKey()) {
    // Check if it's time for a fetch (either first time or interval passed)
    if (gLastWeatherFetchMs == 0 || (nowMs - gLastWeatherFetchMs > WEATHER_FETCH_INTERVAL_MS)) {
      startWeatherFetchAsync();
      // Note: gLastWeatherFetchMs is set inside fetchWeatherFromAPI()/markWeatherFetchAttemptNow()
    }
  }
}
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== MarioMatrix_ESPS3 (UART Controller) ===");

#if defined(ESP32)
  // Helpful for diagnosing TLS/memory issues.
  Serial.printf("[PSRAM] Found=%d size=%u free=%u\n",
                psramFound() ? 1 : 0,
                (unsigned)ESP.getPsramSize(),
                (unsigned)ESP.getFreePsram());
#endif

  // UART1 from controller ESP32
  Serial1.begin(115200, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  Serial.println("[S3] Serial1 started on RX1=17, TX1=18 @115200");

  // ─────────────────────────────────────────
  // Matrix initialization FIRST (game must start immediately)
  // ─────────────────────────────────────────
  HUB75_I2S_CFG mxconfig(PANEL_WIDTH, PANEL_HEIGHT, PANELS_NUMBER);

  // Enable REAL DMA double buffering
  mxconfig.double_buff = true;

  // Pin mapping (matches your listed wiring)
  mxconfig.gpio.r1 = 4;
  mxconfig.gpio.g1 = 5;
  mxconfig.gpio.b1 = 6;

  mxconfig.gpio.r2 = 7;
  mxconfig.gpio.g2 = 15;
  mxconfig.gpio.b2 = 16;

  mxconfig.gpio.a  = 3;
  mxconfig.gpio.b  = 9;
  mxconfig.gpio.c  = 10;
  mxconfig.gpio.d  = 11;
  mxconfig.gpio.e  = 12;

  mxconfig.gpio.clk = 13;
  mxconfig.gpio.lat = 14;
  mxconfig.gpio.oe  = 2;

  mxconfig.driver = HUB75_I2S_CFG::FM6126A;
  mxconfig.setPixelColorDepthBits(5);

  // Fix 1-pixel horizontal shift on some HUB75 panels
  mxconfig.clkphase = false;

  matrix = new MatrixPanel_I2S_DMA(mxconfig);
  if (!matrix->begin()) {
    Serial.println("Matrix begin() FAILED");
    while (true) { delay(1000); }
  }

  // Slightly longer latch blanking to prevent ghosting
  matrix->setLatBlanking(2);

  // If your library ALSO supports this method, you can optionally enable it:
  // matrix->setDoubleBuffer(true);

  matrix->setBrightness8(64);
  matrix->clearScreen();

  Serial.print("Matrix ready. Size: ");
  Serial.print(matrix->width());
  Serial.print(" x ");
  Serial.println(matrix->height());

  // ─────────────────────────────────────────
  // SPIFFS mount (demo persistence lives in /demo/*.bin)
  // ─────────────────────────────────────────
  initSPIFFS();

  // ─────────────────────────────────────────
  // Load weather settings from NVS (boot time)
  // ─────────────────────────────────────────
  loadWeatherKey("boot");
  loadLocation();

  // ─────────────────────────────────────────
  // WiFi Event Handler (auto-start OTA on connect)
  // ─────────────────────────────────────────
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.print("[WiFi] GOT_IP: ");
        Serial.println(WiFi.localIP());
        gWiFiConnected = true;
        break;

      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.println("[WiFi] DISCONNECTED");
        gWiFiConnected = false;
        break;

      default:
        break;
    }
  });

  // ─────────────────────────────────────────
  // NON-BLOCKING Wi-Fi initialization (starts in background)
  // ─────────────────────────────────────────
  initWiFi();  // Non-blocking - will try for 15s, then start portal if needed

  // Use full Map_Build world (with tunnels)
  loadTestMap();
}

// ─────────────────────────────────────────────
// Mushroom pickup
// ─────────────────────────────────────────────
void tryPickupMushroom() {
  if (!gMushroom.active || gMushroom.rising) return;

  float hx1 = heroX;
  float hx2 = heroX + heroW;
  float hy1 = heroY;
  float hy2 = heroY + heroHitH;

  float mx1 = gMushroom.x;
  float mx2 = gMushroom.x + MUSH_W;
  float my1 = gMushroom.y;
  float my2 = gMushroom.y + MUSH_H;

  bool overlap =
    (hx1 < mx2 && hx2 > mx1 &&
     hy1 < my2 && hy2 > my1);

  if (overlap) {
    // TODO: When implementing enemy system, check if sliding kills enemies
    // if (isSliding && currentPose == POSE_SLIDE) {
    //   // Kill enemy instead of damaging player
    //   // gMushroom.active = false;
    //   // return;
    // }
    
    if (gMushroom.type == ITEM_GREEN_MUSHROOM) {
      gMarioLives++;
      if (gMarioLives > 999) gMarioLives = 999;
      gMushroom.active = false;
    } else {
      if (heroSize == SIZE_SMALL) {
        isGrowing        = true;
        growStep         = 0;
        growFrameCounter = 0;
#ifdef DEBUG_GAME
        Serial.println("Power-up: start grow animation");
#endif
      }
      gMushroom.active = false;
    }
  }
}

// Damage Mario from enemy contact (non-stomp).
void damageMarioFromEnemy() {
  unsigned long now = gameplayNowMs();
  if (now < gHeroInvincUntilMs) return; // currently invincible

  // During pipe transition or death sequence, ignore damage.
  if (gPipeTransition.active || gDeath.active) return;

  if (heroSize == SIZE_BIG) {
    // Shrink and give temporary invincibility
    setHeroSize(SIZE_SMALL);
    gHeroInvincUntilMs = now + HERO_INVINC_MS;
  } else {
    // Lose a life and reset
    loseLifeAndReset();
    // clear invincibility
    gHeroInvincUntilMs = 0;
  }
}

static inline bool piranhaRectOverlap(float ax1, float ay1, float ax2, float ay2,
                                      float bx1, float by1, float bx2, float by2) {
  return (ax1 < bx2 && ax2 > bx1 && ay1 < by2 && ay2 > by1);
}

static inline bool isHeroBlockingPipeArea(uint8_t pipeTileX, uint8_t pipeTileY) {
  // Match pipe transition geometry.
  const float pipeWidth  = 18.0f;
  const float pipeHeight = 17.0f;
  float px1 = pipeTileX * TILE_SIZE;
  float py1 = pipeTileY * TILE_SIZE;
  float px2 = px1 + pipeWidth;
  float py2 = py1 + pipeHeight;

  // Expand upward slightly so standing on the pipe counts as blocking.
  py1 -= 2.0f;

  float hx1 = heroX;
  float hx2 = heroX + heroW;
  float hy1 = heroY;
  float hy2 = heroY + heroHitH;

  return piranhaRectOverlap(hx1, hy1, hx2, hy2, px1, py1, px2, py2);
}

void resetPiranhas() {
  for (int i = 0; i < MAX_PIRANHAS; ++i) {
    gPiranhas[i].active = false;
    gPiranhas[i].tileX = 0;
    gPiranhas[i].tileY = 0;
    gPiranhas[i].pipeTopYpx = 0;
    gPiranhas[i].xpx = 0;
    gPiranhas[i].visibleH = 0;
    gPiranhas[i].state = PIRANHA_HIDDEN;
    gPiranhas[i].stateStartMs = 0;
    gPiranhas[i].animFrame = 0;
    gPiranhas[i].lastAnimMs = 0;
  }
}

static void spawnPiranhaAt(uint8_t tileX, uint8_t tileY) {
  int slot = -1;
  for (int i = 0; i < MAX_PIRANHAS; ++i) {
    if (!gPiranhas[i].active) { slot = i; break; }
  }
  if (slot < 0) return;

  PiranhaPlant &p = gPiranhas[slot];
  p.active = true;
  p.tileX = (int16_t)tileX;
  p.tileY = (int16_t)tileY;

  // The pipe tile is directly below the anchor tile.
  p.pipeTopYpx = (int16_t)((tileY + 1) * TILE_SIZE);

  // Align to the same anchor as the 18px-wide pipe sprite.
  p.xpx = (int16_t)(tileX * TILE_SIZE);

  p.visibleH = 0;
  p.state = PIRANHA_HIDDEN;
  p.stateStartMs = gameplayNowMs();
  p.animFrame = 0;
  p.lastAnimMs = gameplayNowMs();
}

void spawnPiranhasFromMobLayer() {
  resetPiranhas();

  for (int ty = 0; ty < LEVEL_HEIGHT; ++ty) {
    for (int tx = 0; tx < LEVEL_WIDTH; ++tx) {
      uint8_t id = getMobId(tx, ty);
      if (id == TILE_ID_MOB_PIRANHA_PLANT_L || id == TILE_ID_MOB_PIRANHA_PLANT) {
        spawnPiranhaAt((uint8_t)tx, (uint8_t)ty);
      }
    }
  }
}

static inline bool marioIsNearPipeX(int16_t pipeXpx) {
  int16_t marioCenterX = (int16_t)(heroX + heroW * 0.5f);
  int16_t pipeCenterX  = (int16_t)(pipeXpx + (PIRANHA_W / 2));
  int16_t dx = abs(marioCenterX - pipeCenterX);
  return (dx <= PIRANHA_MARIO_NEAR_PX);
}

void updatePiranhas() {
  if (gFlagpole.active && gFlagpole.freezeEnemies) {
    return;
  }
  uint32_t now = gameplayNowMs();

  for (int i = 0; i < MAX_PIRANHAS; ++i) {
    PiranhaPlant &p = gPiranhas[i];
    if (!p.active) continue;

    // Optional: SMW-like behavior: if Mario is very close, keep it from rising.
    if (PIRANHA_PAUSE_IF_MARIO_NEAR && (p.state == PIRANHA_HIDDEN || p.state == PIRANHA_RISING)) {
      if (marioIsNearPipeX(p.xpx)) {
        p.state = PIRANHA_HIDDEN;
        p.visibleH = 0;
        p.stateStartMs = now;
        continue;
      }
    }

    // Mouth animation toggle when at least partially visible.
    if (p.visibleH > 0) {
      if (now - p.lastAnimMs >= 160) {
        p.animFrame ^= 1;
        p.lastAnimMs = now;
      }
    } else {
      p.animFrame = 0;
      p.lastAnimMs = now;
    }

    switch (p.state) {
      case PIRANHA_HIDDEN: {
        // Make sure it is fully hidden.
        p.visibleH = 0;

        // Don't start rising if Mario overlaps the pipe opening.
        // The pipe tile is one row BELOW the anchor.
        if (now - p.stateStartMs >= PIRANHA_HIDDEN_MS) {
          if (isHeroBlockingPipeArea((uint8_t)p.tileX, (uint8_t)(p.tileY + 1))) {
            // Wait a little and try again.
            p.stateStartMs = now;
          } else {
            p.state = PIRANHA_RISING;
            p.stateStartMs = now;
          }
        }
        break;
      }

      case PIRANHA_RISING: {
        if (p.visibleH < PIRANHA_H) {
          uint16_t nextH = (uint16_t)p.visibleH + (uint16_t)PIRANHA_PIXELS_PER_TICK;
          p.visibleH = (uint8_t)((nextH > PIRANHA_H) ? PIRANHA_H : nextH);
        }
        if (p.visibleH >= PIRANHA_H) {
          p.visibleH = PIRANHA_H;
          p.state = PIRANHA_OUT;
          p.stateStartMs = now;
        }
        break;
      }

      case PIRANHA_OUT: {
        p.visibleH = PIRANHA_H;
        if (now - p.stateStartMs >= PIRANHA_OUT_MS) {
          p.state = PIRANHA_SINKING;
          p.stateStartMs = now;
        }
        break;
      }

      case PIRANHA_SINKING: {
        if (p.visibleH > 0) {
          int nextH = (int)p.visibleH - (int)PIRANHA_PIXELS_PER_TICK;
          if (nextH < 0) nextH = 0;
          p.visibleH = (uint8_t)nextH;
        }
        if (p.visibleH == 0) {
          p.state = PIRANHA_HIDDEN;
          p.stateStartMs = now;
        }
        break;
      }

      default:
        p.state = PIRANHA_HIDDEN;
        p.visibleH = 0;
        p.stateStartMs = now;
        break;
    }
  }
}

void drawPiranhas() {
  if (!matrix) return;

  const int tileMargin = 2;
  int firstTileX = (int)(cameraX / TILE_SIZE) - tileMargin;
  int firstTileY = (int)(cameraY / TILE_SIZE) - tileMargin;
  int lastTileX  = (int)((cameraX + MATRIX_WIDTH) / TILE_SIZE) + tileMargin;
  int lastTileY  = (int)((cameraY + MATRIX_HEIGHT) / TILE_SIZE) + tileMargin;
  if (firstTileX < 0) firstTileX = 0;
  if (firstTileY < 0) firstTileY = 0;
  if (lastTileX >= LEVEL_WIDTH) lastTileX = LEVEL_WIDTH - 1;
  if (lastTileY >= LEVEL_HEIGHT) lastTileY = LEVEL_HEIGHT - 1;

  for (int i = 0; i < MAX_PIRANHAS; ++i) {
    const PiranhaPlant &p = gPiranhas[i];
    if (!p.active) continue;
    if (p.visibleH == 0) continue;

    if (p.tileX < firstTileX || p.tileX > lastTileX || p.tileY < firstTileY || p.tileY > lastTileY) {
      continue;
    }

    const uint8_t (*frame)[18] = (p.animFrame == 0) ? Mob_Piranha_Plant_Frame1 : Mob_Piranha_Plant_Frame2;

    // Draw using SMW-style reveal: only draw the visible portion ABOVE the pipe opening.
    int worldX = (int)p.xpx;
    int worldY = (int)p.pipeTopYpx - (int)p.visibleH;

    int px = worldX - (int)cameraX;
    int py = worldY - (int)cameraY;

    drawWorldTileCustomSizeClippedHeight((const uint8_t*)&frame[0][0], PIRANHA_W, PIRANHA_H, px, py, (int)p.visibleH);
  }
}

void checkPiranhaHeroCollisions() {
  if (gFlagpole.active && gFlagpole.freezeEnemies) return;
  float hx1 = heroX;
  float hx2 = heroX + heroW;
  float hy1 = heroY;
  float hy2 = heroY + heroHitH;

  for (int i = 0; i < MAX_PIRANHAS; ++i) {
    const PiranhaPlant &p = gPiranhas[i];

    if (!p.active) continue;
    if (p.visibleH <= PIRANHA_BLOCK_THRESHOLD_PX) continue; // tiny tip grace

    float px1 = (float)p.xpx;
    float py1 = (float)p.pipeTopYpx - (float)p.visibleH;
    float px2 = px1 + (float)PIRANHA_W;
    float py2 = py1 + (float)p.visibleH;

    if (piranhaRectOverlap(hx1, hy1, hx2, hy2, px1, py1, px2, py2)) {
      damageMarioFromEnemy();
    }
  }
}

bool isPiranhaBlockingPipeEntry(int pipeTileX, int pipeTileY) {
  for (int i = 0; i < MAX_PIRANHAS; ++i) {
    const PiranhaPlant &p = gPiranhas[i];
    if (!p.active) continue;
    // The piranha anchor is one tile ABOVE the pipe.
    if ((int)p.tileX != pipeTileX || (int)(p.tileY + 1) != pipeTileY) continue;

    // Allow entry only when sufficiently hidden.
    if (p.visibleH > PIRANHA_BLOCK_THRESHOLD_PX) return true;
  }
  return false;
}

// Check collisions between Mario and goombas: stomp, slam, or side damage
void checkGoombaHeroCollisions() {
  if (gFlagpole.active && gFlagpole.freezeEnemies) return;
  // Hero bbox
  float hx1 = heroX;
  float hx2 = heroX + heroW;
  float hy1 = heroY;
  float hy2 = heroY + heroHitH;

  for (int i = 0; i < MAX_GOOMBAS; ++i) {
    Goomba &gg = gGoombas[i];
    if (!gg.active) continue;
    if (gg.state != GOOMBA_STATE_ALIVE) continue;

    float gx1 = gg.x;
    float gx2 = gg.x + GOOMBA_W;
    float gy1 = gg.y;
    float gy2 = gg.y + GOOMBA_H;

    bool overlap = (hx1 < gx2 && hx2 > gx1 && hy1 < gy2 && hy2 > gy1);
    if (!overlap) continue;

    // Stomp detection: Mario moving downward OR butt slam active, and feet overlap top ~3px of goomba
    bool movingDown = (heroVY > 0.0f) || slamActive;
    float stompThreshold = gy1 + 3.0f; // top ~3 pixels
    bool feetOverlapTop = (hy2 > gy1) && (hy2 <= stompThreshold + 2.0f);

    if (movingDown && feetOverlapTop && fabsf((hx1+hx2)/2.0f - (gx1+gx2)/2.0f) < GOOMBA_W) {
      // Stomp: squish goomba
      gg.state = GOOMBA_STATE_SQUISHED;
      gg.stateTimer = 12; // ~12 frames (~200ms)
      // bounce Mario
      heroVY = -3.0f;
      onGround = false;
      // award small bounce/particles could be added
      continue;
    }

    // Otherwise side/body contact: damage Mario
    damageMarioFromEnemy();
    // give brief invincibility if shrunk
    if (gameplayNowMs() < gHeroInvincUntilMs) {
      // knockback Mario slightly
      if (hx2 <= gx2 && heroX < gg.x) heroVX = -1.5f; else heroVX = 1.5f;
    }
  }
}

// Check collisions between Mario and koopas: stomp kills, otherwise side damage
void checkKoopaHeroCollisions() {
  if (gFlagpole.active && gFlagpole.freezeEnemies) return;

  float hx1 = heroX;
  float hx2 = heroX + heroW;
  float hy1 = heroY;
  float hy2 = heroY + heroHitH;

  for (int i = 0; i < MAX_KOOPAS; ++i) {
    Koopa &kk = gKoopas[i];
    if (!kk.active) continue;

    float kx1 = kk.x;
    float kx2 = kk.x + KOOPA_W;
    float ky1 = kk.y;
    float ky2 = kk.y + KOOPA_H;

    bool overlap = (hx1 < kx2 && hx2 > kx1 && hy1 < ky2 && hy2 > ky1);
    if (!overlap) continue;

    const bool demoLenient = demoTimingActive();

    // Prefer the "approach" snapshot in demo contexts (captured before ground snap).
    // In normal gameplay, use the current instantaneous state.
    const float stompY  = demoLenient ? gHeroApproachY : heroY;
    const float stompVY = demoLenient ? gHeroApproachVY : heroVY;
    const float stompHitH = demoLenient ? gHeroApproachHitH : heroHitH;
    const bool stompOnGround = demoLenient ? gHeroApproachOnGround : onGround;
    const float stompFeetY = stompY + stompHitH;

    // In demo contexts we want recordings to be robust to tiny sub-pixel drift.
    // Treat the apex/near-zero vertical velocity as "down" to preserve intended stomps.
    const float downEps = demoLenient ? 0.001f : 0.0f;
    bool movingDown = (stompVY > downEps) || slamActive || (demoLenient && fabsf(stompVY) <= downEps);

    // Stomp detection uses Mario's FEET (hy2) relative to the Koopa top.
    // For demos, allow additional penetration based on downward speed so that
    // high fall speed / tiny drift doesn't skip past the top-band in a single frame.
    const float stompBandPx = 6.0f;
    float stompEpsPx = 2.0f;
    if (demoLenient) {
      const float fallPx = (stompVY > 0.0f) ? stompVY : 0.0f;
      stompEpsPx += fallPx;
      // If we were airborne last frame but got snapped to ground before enemy checks,
      // keep stomp permissive to match recorded outcomes.
      if (!stompOnGround) stompEpsPx += 1.0f;
    }
    bool feetOverlapTop = (stompFeetY > ky1) && (stompFeetY <= (ky1 + stompBandPx + stompEpsPx));
    bool stompFromAbove = feetOverlapTop;

    float centerDx = fabsf(((hx1 + hx2) * 0.5f) - ((kx1 + kx2) * 0.5f));
    const float centerThresh = (float)KOOPA_W + (demoLenient ? 0.75f : 0.0f);
    bool centerOk = (centerDx < centerThresh);

#if DEBUG_DEMO_MAP2_TRACE
    if (!gDemoTraceKoopaDumped && gDemoPlaybackActive && gDemoPlaybackPlaying && gCurrentMapId == 2) {
      gDemoTraceKoopaDumped = true;
      demoTraceDump("KOOPA_OVERLAP");
      bool willStomp = movingDown && stompFromAbove && centerOk;
      Serial.print("[DTR] koopa#");
      Serial.print(i);
      Serial.print(" overlap=1 md=");
      Serial.print(movingDown ? 1 : 0);
      Serial.print(" hvy=");
      Serial.print(heroVY, 3);
      Serial.print(" feetTop=");
      Serial.print(feetOverlapTop ? 1 : 0);
      Serial.print(" fromAbove=");
      Serial.print(stompFromAbove ? 1 : 0);
      Serial.print(" vyA=");
      Serial.print(stompVY, 3);
      Serial.print(" ky=");
      Serial.print(ky1, 2);
      Serial.print(" cdx=");
      Serial.print(centerDx, 2);
      Serial.print(" hy=");
      Serial.print(heroY, 2);
      Serial.print(" hy2=");
      Serial.print(hy2, 2);
      Serial.print(" cOk=");
      Serial.print(centerOk ? 1 : 0);
      Serial.print(" stEps=");
      Serial.print(stompEpsPx, 2);
      Serial.print(" willStomp=");
      Serial.println(willStomp ? 1 : 0);
    }
#endif

    if (movingDown && stompFromAbove && centerOk) {
      // Stomp: convert koopa into a shell (always starts sliding LEFT)
      float shellX = kk.x + (KOOPA_W - SHELL_W) * 0.5f;
      Serial.print(" aHy=");
      Serial.print(stompY, 2);
      Serial.print(" aHy2=");
      Serial.print(stompFeetY, 2);
      float shellY = kk.y + (KOOPA_H - SHELL_H);
      int shIdx = spawnShellAt(shellX, shellY);
      if (shIdx >= 0) {
        KoopaShell &sh = gShells[shIdx];
        sh.dir = -1;                 // always left
        sh.state = SHELL_SLIDING;
        sh.vx = -SHELL_KICK_SPEED;
        sh.vy = 0.0f;
        sh.ignoreHeroUntilMs = gameplayNowMs() + 350; // short grace so stomp bounce doesn't insta-hurt
      }
      kk.active = false;

      // bounce Mario
      heroVY = -3.0f;
      onGround = false;
      continue;
    }

    damageMarioFromEnemy();
    if (gameplayNowMs() < gHeroInvincUntilMs) {
      if (hx2 <= kx2 && heroX < kk.x) heroVX = -1.5f; else heroVX = 1.5f;
    }
  }
}

// ─────────────────────────────────────────────
// Coin pickup
// ─────────────────────────────────────────────
void checkCoinPickup() {
  // Hero bounding box
  float hx1 = heroX;
  float hx2 = heroX + heroW;
  float hy1 = heroY;
  float hy2 = heroY + heroHitH;

  int txStart = (int)(hx1 / TILE_SIZE);
  int txEnd   = (int)((hx2 - 1) / TILE_SIZE);
  int tyStart = (int)(hy1 / TILE_SIZE);
  int tyEnd   = (int)((hy2 - 1) / TILE_SIZE);

  if (txStart < 0) txStart = 0;
  if (tyStart < 0) tyStart = 0;
  if (txEnd >= LEVEL_WIDTH)  txEnd = LEVEL_WIDTH - 1;
  if (tyEnd >= LEVEL_HEIGHT) tyEnd = LEVEL_HEIGHT - 1;

  for (int ty = tyStart; ty <= tyEnd; ty++) {
    for (int tx = txStart; tx <= txEnd; tx++) {
      uint8_t id = getTileId(tx, ty);
      if (id == TILE_ID_COIN) {
        setTileId(tx, ty, TILE_ID_EMPTY);
        coinCount++;

        Serial.print("Coin collected! Total = ");
        Serial.println(coinCount);
      }
    }
  }
}

// ─────────────────────────────────────────────
// Checkpoint system
// ─────────────────────────────────────────────

static void updateCheckpointTouch() {
  // Treat "touch" as overlap with Mario's bounding box.
  // This works whether the flag is placed in air above ground or in open space.
  float hx1 = heroX;
  float hx2 = heroX + heroW;
  float hy1 = heroY;
  float hy2 = heroY + heroHitH;

  int txStart = (int)(hx1 / TILE_SIZE);
  int txEnd   = (int)((hx2 - 1) / TILE_SIZE);
  int tyStart = (int)(hy1 / TILE_SIZE);
  int tyEnd   = (int)((hy2 - 1) / TILE_SIZE);

  if (txStart < 0) txStart = 0;
  if (tyStart < 0) tyStart = 0;
  if (txEnd >= LEVEL_WIDTH)  txEnd = LEVEL_WIDTH - 1;
  if (tyEnd >= LEVEL_HEIGHT) tyEnd = LEVEL_HEIGHT - 1;

  bool found = false;
  int foundTx = -1;
  int foundTy = -1;
  uint8_t foundId = TILE_ID_EMPTY;

  for (int ty = tyStart; ty <= tyEnd && !found; ++ty) {
    for (int tx = txStart; tx <= txEnd; ++tx) {
      uint8_t id = getTileId(tx, ty);
      if (id != TILE_ID_CHECK_POINT && id != TILE_ID_CHECKED_POINT) continue;
      found = true;
      foundTx = tx;
      foundTy = ty;
      foundId = id;
      break;
    }
  }

  bool newlyTouched = false;
  if (found) {
    newlyTouched = (!checkpointTouchingLastFrame) ||
                  (checkpointTouchLastTx != foundTx) ||
                  (checkpointTouchLastTy != foundTy);
  }

  checkpointTouchingLastFrame = found;
  if (found) {
    checkpointTouchLastTx = (int16_t)foundTx;
    checkpointTouchLastTy = (int16_t)foundTy;
  }

  if (!found) return;

  // Touching either flag type should make it the active respawn checkpoint.
  checkpointActive = true;
  checkpointTileX = (int16_t)foundTx;
  checkpointTileY = (int16_t)foundTy;

  uint32_t now = gameplayNowMs();

  // Visual-only wiggle animation on touch.
  if (newlyTouched) {
    if (foundId == TILE_ID_CHECK_POINT) {
      startCheckpointWiggle(foundTx, foundTy, TILE_ID_CHECK_POINT, TILE_ID_CHECKED_POINT);
    } else {
      startCheckpointWiggle(foundTx, foundTy, TILE_ID_CHECKED_POINT, TILE_ID_CHECKED_POINT);
    }
  }

  // Unchecked flag: start the existing flip->checked conversion.
  if (foundId == TILE_ID_CHECK_POINT) {
    // If we're already animating this exact tile, don't restart.
    if (checkpointFlipAnimating && checkpointFlipTileX == foundTx && checkpointFlipTileY == foundTy) {
      return;
    }

    checkpointFlipAnimating = true;
    checkpointFlipTileX = (int16_t)foundTx;
    checkpointFlipTileY = (int16_t)foundTy;
    checkpointFlipStartMs = now;

    Serial.print("[CHECKPOINT] Activated at tx=");
    Serial.print(foundTx);
    Serial.print(" ty=");
    Serial.println(foundTy);
  }
}

static void updateCheckpointFlipAnimation() {
  if (!checkpointFlipAnimating) return;

  uint32_t now = gameplayNowMs();
  if (now - checkpointFlipStartMs >= CHECKPOINT_FLIP_MS) {
    checkpointFlipAnimating = false;
    setTileId((int)checkpointFlipTileX, (int)checkpointFlipTileY, TILE_ID_CHECKED_POINT);

    Serial.print("[CHECKPOINT] Flip complete -> CHECKED at tx=");
    Serial.print(checkpointFlipTileX);
    Serial.print(" ty=");
    Serial.println(checkpointFlipTileY);
  }
}


static void drawCheckpointFlipOverlay() {
  if (!checkpointFlipAnimating) return;

  uint32_t now = gameplayNowMs();
  float t = (now - checkpointFlipStartMs) / (float)CHECKPOINT_FLIP_MS;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  const int fullW = 15;
  const int fullH = 23;
  int revealH = (int)(t * (float)fullH);
  if (revealH <= 0) return;
  if (revealH > fullH) revealH = fullH;

  int worldX = (int)checkpointFlipTileX * TILE_SIZE;
  int worldY = (int)checkpointFlipTileY * TILE_SIZE;

  int px = worldX - (int)cameraX;
  int py = worldY - (int)cameraY;

  // Bottom-anchored tile: sprite top starts at (py + 8 - fullH)
  int baseY = py + TILE_SIZE - fullH;
  int srcY  = fullH - revealH;
  int dstY  = baseY + srcY;

  drawWorldTileCustomSizeSubrect((const uint8_t*)&Ground_Tile_Checked_Point[0][0],
                                fullW,
                                fullH,
                                px,
                                dstY,
                                0,
                                srcY,
                                fullW,
                                revealH);
}


// ─────────────────────────────────────────────
// Map timer handlers (SMW-style countdown)
// ─────────────────────────────────────────────
void handleTimeUp() {
  Serial.println("TIME UP!");
  Serial.println("TIME UP! -> loseLifeAndReset");
  loseLifeAndReset();
  // Prevent immediate re-tick on the same millisecond
  gLastMapTimeTickMs = gameplayNowMs();
}

// Shared lose-life helper used by time-up and enemy damage
void loseLifeAndReset() {
  if (gPipeTransition.active) return;
  if (gDeath.active) return;

  gMarioLives--;
  if (gMarioLives < 0) gMarioLives = 0;
  gMapTimeSeconds = 300;

  // Game Over: fully reset the world (fresh map load restoring tiles/mobs/coins)
  // and restart from the original spawn.
  if (gMarioLives == 0) {
    // If we run out of lives on any map, the game resets back to Map 1.
    gCurrentMapId = MAP_ID_OVERWORLD_1;
    // Show death + GAME OVER, then hard reset.
    startDeathSequence(spawnTileX, spawnTileY, true,
                      false, -1, -1);
    return;
  }

  // Normal death: reset world to a fresh-start state, then respawn.
  // If a checkpoint has been activated, respawn from it.
  int16_t respawnTx = spawnTileX;
  int16_t respawnTy = spawnTileY;
  bool cpActive = checkpointActive && checkpointTileX >= 0 && checkpointTileY >= 0;
  if (cpActive) {
    respawnTx = checkpointTileX;
    respawnTy = checkpointTileY;
  }

  startDeathSequence(respawnTx, respawnTy, false,
                    cpActive, checkpointTileX, checkpointTileY);
}

// Restores checkpoint state after a full world reset (death fresh-start).
// Keeps the checkpoint tile visually in the CHECKED state and preserves respawn.
static void restoreCheckpointAfterWorldReset(bool wasActive, int16_t tx, int16_t ty) {
  if (!wasActive) return;
  if (tx < 0 || ty < 0) return;
  if (tx >= LEVEL_WIDTH || ty >= LEVEL_HEIGHT) return;

  checkpointActive = true;
  checkpointTileX = tx;
  checkpointTileY = ty;

  // Ensure the flag remains checked after the map reload.
  setTileId((int)tx, (int)ty, TILE_ID_CHECKED_POINT);
}

void tickMapTimer() {
  unsigned long now = gameplayNowMs();

  if (now - gLastMapTimeTickMs >= 1000UL) {
    gLastMapTimeTickMs = now;

    if (gMapTimeSeconds > 0) {
      gMapTimeSeconds--;
      if (gMapTimeSeconds <= 0) {
        gMapTimeSeconds = 0;
        handleTimeUp();
      }
    }
  }
}

// ─────────────────────────────────────────────
// Game update logic
// ─────────────────────────────────────────────

static bool heroWouldCollideAtX(float testX, float heroY, float heroW, float heroHitH) {
  const float HITBOX_INSET_X = 1.0f;

  float left  = testX + HITBOX_INSET_X;
  float right = testX + heroW - HITBOX_INSET_X;
  float top   = heroY;
  float bot   = heroY + heroHitH - 1.0f;

  int txL = (int)(left / TILE_SIZE);
  int txR = (int)((right - 1) / TILE_SIZE);
  int tyT = (int)(top / TILE_SIZE);
  int tyB = (int)(bot / TILE_SIZE);

  if (txL < 0 || txR >= LEVEL_WIDTH || tyT < 0 || tyB >= LEVEL_HEIGHT) return true;

  for (int ty = tyT; ty <= tyB; ty++) {
    for (int tx = txL; tx <= txR; tx++) {
      if (tileIsSolid(getTile(tx, ty))) return true;
    }
  }
  return false;
}

void updateGame(int &bobOffset, int &crouchOffset) {
  // Cloth runtime should update every frame (even during scripted sequences)
  updateFlagPoleCloth();

  // Track previous hero position once per frame (used for reliable top-landing tests)
  heroPrevX = heroX;
  heroPrevY = heroY;

  // Save the starting X for this frame so a ceiling bonk can cancel X movement cleanly.
  const float frameStartHeroX = heroX;

  // Decode inputs into local variables
  float moveInput           = gInput.moveAxis;
  bool  normalJumpPressed   = gInput.btnJump;
  bool  spinJumpPressed     = gInput.btnSpin;
  bool  runPressed          = gInput.btnRun;
  bool  downPressed         = gInput.btnDown;
  bool  upPressed           = gInput.btnUp;
  bool  leftPressed         = (gInput.rawMoveX < 0);
  bool  rightPressed        = (gInput.rawMoveX > 0);

  // If we just bonked our head, ignore horizontal input for a couple frames.
  // This prevents the “walking bonk → bounce backwards / clip” corner case.
  if (gCeilingBonkFreezeFrames > 0) {
    moveInput = 0.0f;
    gCeilingBonkFreezeFrames--;
  }

  // Hard-kill VX for a couple frames after a bonk (safe; prevents backward bounce)
  if (gCeilingBonkHoldVXFrames > 0) {
    heroVX = 0.0f;
    slopeSlideVX = 0.0f;
    slopeSlideActive = false;
    gCeilingBonkHoldVXFrames--;
  }

  bool runJustReleased = (!runPressed && prevRunPressed);

  bool justPressedDown = downPressed && !prevDownPressed;
  bool justPressedLeft = leftPressed && !prevLeftPressed;
  bool justPressedRight = rightPressed && !prevRightPressed;

  // Track facing direction for shell carry/kick (rendering is unchanged)
  if (moveInput < -0.1f)      gHeroFacingDir = -1;
  else if (moveInput > 0.1f)  gHeroFacingDir = 1;
  else if (heroVX < -0.1f)    gHeroFacingDir = -1;
  else if (heroVX > 0.1f)     gHeroFacingDir = 1;

  // Pipe transition is now handled in loopGameplay() before updateGame().
  // We just need to skip movement input during transitions.
  // (The updatePipeTransition function handles hero position directly.)

  // Flagpole scripted sequence (locks input/physics)
  if (gFlagpole.active) {
    updateFlagpoleSequence(bobOffset, crouchOffset);
    prevNormalJumpPressed = normalJumpPressed;
    prevSpinJumpPressed   = spinJumpPressed;
    prevRunPressed        = runPressed;
    prevDownPressed       = downPressed;
    prevRightPressed      = rightPressed;
    prevLeftPressed       = leftPressed;
    return;
  }

  // ─────────────────────────────────────────
  // Pipe entry – start scripted transition
  // ─────────────────────────────────────────
  if (gPipeEntryNeedsDownRelease) {
    if (!downPressed) {
      gPipeEntryNeedsDownRelease = false;
    } else {
      goto PipeEntryDone;
    }
  }

  if (onGround && justPressedDown && gTunnelPairCount > 0 && !gPipeTransition.active) {
    int tunnelIndex = -1;
    if (findTunnelPairAtFeet(heroX, heroY, heroHitH, tunnelIndex)) {
      if (tunnelIndex >= 0 && tunnelIndex < gTunnelPairCount) {
        TunnelPair &tp = gTunnelPairs[tunnelIndex];

        // Only allow DOWN-entry transitions for pairs that actually have a down-exit.
        // (Pair 2 is left-entry only and intentionally has no down exit.)
        bool exitIsDown =
          (tp.exit.tileId == TILE_ID_GREEN_TUNNEL_DOWN) ||
          (tp.exit.tileId == TILE_ID_GREEN_TUNNEL_DOWN_2_2);
        if (!exitIsDown) {
          goto PipeEntryDone;
        }

        // SMW-style gate: only allow pipe entry when the piranha is safely hidden.
        if (isPiranhaBlockingPipeEntry(tp.entry.tileX, tp.entry.tileY)) {
          // blocked
          goto PipeEntryDone;
        }

        Serial.printf("[PIPE] Enter (DOWN) pair %d entryId=%u exitId=%u\n",
                      tunnelIndex,
                      (unsigned)tp.entry.tileId,
                      (unsigned)tp.exit.tileId);
        
        // Calculate entry pipe center position
        const float pipeWidth  = 18.0f;
        const float pipeHeight = 17.0f;
        float entryPipeLeft = tp.entry.tileX * TILE_SIZE;
        float entryPipeTop  = tp.entry.tileY * TILE_SIZE;
        float entryCenterX  = entryPipeLeft + pipeWidth * 0.5f - heroW * 0.5f;
        float entryTopY     = entryPipeTop;
        
        // Calculate exit spawn position (hidden inside exit pipe)
        float exitPipeLeft   = tp.exit.tileX * TILE_SIZE;
        float exitPipeTop    = tp.exit.tileY * TILE_SIZE;
        float exitPipeBottom = exitPipeTop + pipeHeight;
        float exitCenterX    = exitPipeLeft + pipeWidth * 0.5f - heroW * 0.5f;
        
        // Start hidden inside the exit pipe
        float exitSpawnX = exitCenterX;
        float exitSpawnY = exitPipeBottom - heroHitH; // hidden inside
        
        startPipeTransition(tunnelIndex,
                            false,
                            0,
                            entryCenterX,
                            entryTopY,
                            exitSpawnX,
                            exitSpawnY,
                            tp.exit.tileX,
                            tp.exit.tileY,
                            tp.exit.tileId);
      }
    }
  }

  // ─────────────────────────────────────────
  // Pipe entry (LEFT entrance) – start scripted transition
  // ─────────────────────────────────────────
  // Trigger while holding LEFT/RIGHT so you can walk into the entrance.
  // For TileID 60, we allow both approaches:
  // - stand on RIGHT side, press LEFT
  // - stand on LEFT side, press RIGHT
  if (onGround && (leftPressed || rightPressed) && gTunnelPairCount > 0 && !gPipeTransition.active) {
    int tunnelIndex = -1;
    int8_t entryDir = leftPressed ? (int8_t)-1 : (int8_t)+1;
    if (findTunnelPairAtSideEntrance(heroX, heroY, heroHitH, entryDir, tunnelIndex)) {
      if (tunnelIndex >= 0 && tunnelIndex < gTunnelPairCount) {
        TunnelPair &tp = gTunnelPairs[tunnelIndex];
        if (!tp.hasLeftEntry) goto PipeEntryDone;

        // SMW-style gate: only allow pipe entry when the piranha is safely hidden.
        if (isPiranhaBlockingPipeEntry(tp.entry.tileX, tp.entry.tileY)) {
          goto PipeEntryDone;
        }

        Serial.printf("[PIPE] Side pipe entry found pair=%d tile=(%d,%d) dir=%d\n",
                tunnelIndex,
                (int)tp.leftEntry.tileX,
          (int)tp.leftEntry.tileY,
          (int)entryDir);
        Serial.printf("[PIPE] Enter (SIDE) pair %d leftId=%u exitUprightId=%u\n",
                tunnelIndex,
                (unsigned)tp.leftEntry.tileId,
                (unsigned)tp.entry.tileId);

        // Snap Mario cleanly to the mouth to avoid scraping/jitter.
        float entryPipeLeft = tp.leftEntry.tileX * TILE_SIZE;
        float entryPipeTop  = tp.leftEntry.tileY * TILE_SIZE;
        float entryPipeRight = entryPipeLeft + (float)PIPE_LEFT_W;
        float mouthX = (entryDir < 0) ? entryPipeRight : entryPipeLeft;

        float lockCenterY = entryPipeTop + ((float)PIPE_LEFT_MOUTH_PAD_Y + ((float)PIPE_LEFT_H - (float)PIPE_LEFT_MOUTH_PAD_Y)) * 0.5f;
        float snappedY = lockCenterY - heroHitH * 0.5f;

        // Place hero so his RIGHT edge touches the mouth plane.
        float snappedX = mouthX - (float)heroW;

        heroX = snappedX;
        heroY = snappedY;

        // Destination: exit upward out of the normal upright tunnel (tp.entry).
        const float exitPipeWidth  = 18.0f;
        const float exitPipeHeight = 17.0f;
        float exitPipeLeft   = tp.entry.tileX * TILE_SIZE;
        float exitPipeTop    = tp.entry.tileY * TILE_SIZE;
        float exitPipeBottom = exitPipeTop + exitPipeHeight;
        float exitCenterX    = exitPipeLeft + exitPipeWidth * 0.5f - heroW * 0.5f;

        float exitSpawnX = exitCenterX;
        float exitSpawnY = (exitPipeBottom - heroHitH) + 1.0f; // fully inside (prevents 1-frame "fall")

        Serial.println("[PIPE] Starting side pipe transition");
        startPipeTransition((uint8_t)tunnelIndex,
                            true,
                            entryDir,
                            snappedX,
                            snappedY,
                            exitSpawnX,
                            exitSpawnY,
                            tp.entry.tileX,
                            tp.entry.tileY,
                            tp.entry.tileId);
      }
    }
  }

PipeEntryDone:

  // Pipe transition drives hero position; prevent jitter and any one-frame impulses.
  // IMPORTANT: keep updatePipeTransition() running elsewhere; this just skips normal physics.
  if (isPipeTransitionActive()) {
    heroVX = 0.0f;
    heroVY = 0.0f;
    slopeSlideVX = 0.0f;
    slopeSlideActive = false;

    prevNormalJumpPressed = normalJumpPressed;
    prevSpinJumpPressed   = spinJumpPressed;
    prevRunPressed        = runPressed;
    prevDownPressed       = downPressed;
    prevRightPressed      = rightPressed;
    prevLeftPressed       = leftPressed;
    return;
  }

  // Death sequence drives hero position; skip normal physics/collisions.
  if (isDeathActive()) {
    heroVX = 0.0f;
    heroVY = 0.0f;
    slopeSlideVX = 0.0f;
    slopeSlideActive = false;

    prevNormalJumpPressed = normalJumpPressed;
    prevSpinJumpPressed   = spinJumpPressed;
    prevRunPressed        = runPressed;
    prevDownPressed       = downPressed;
    prevRightPressed      = rightPressed;
    prevLeftPressed       = leftPressed;
    return;
  }

  // Airborne grab onto flagpole (SMW-ish)
  if (tryStartFlagpoleGrab()) {
    updateFlagpoleSequence(bobOffset, crouchOffset);
    prevNormalJumpPressed = normalJumpPressed;
    prevSpinJumpPressed   = spinJumpPressed;
    prevRunPressed        = runPressed;
    prevDownPressed       = downPressed;
    prevRightPressed      = rightPressed;
    prevLeftPressed       = leftPressed;
    return;
  }

  // --- Slope detection for this frame (use both feet) ---
  float heroFeetY   = heroY + heroHitH;
  float heroLeftX   = heroX;
  float heroRightX  = heroX + heroW;

  int tileXLeft   = (int)(heroLeftX  / TILE_SIZE);
  int tileXRight  = (int)(heroRightX / TILE_SIZE);
  int tileYFeet   = (int)(heroFeetY  / TILE_SIZE);

  TileType tileBelowLeft  = getTile(tileXLeft,  tileYFeet);
  TileType tileBelowRight = getTile(tileXRight, tileYFeet);

  // Also check one tile below (for detecting top of slope)
  TileType tileBelowLeft2  = getTile(tileXLeft,  tileYFeet + 1);
  TileType tileBelowRight2 = getTile(tileXRight, tileYFeet + 1);

  bool onSlopeNow =
    tileIsSlope(tileBelowLeft) ||
    tileIsSlope(tileBelowRight);

  bool nearSlopeTop = 
    (!onSlopeNow && (tileIsSlope(tileBelowLeft2) || tileIsSlope(tileBelowRight2)));

  int slopeDir = 0; // -1 = downhill left, +1 = downhill right
  TileType slopeTile = TILE_EMPTY;
  if (tileIsSlope(tileBelowRight)) {
    slopeTile = tileBelowRight;
  } else if (tileIsSlope(tileBelowLeft)) {
    slopeTile = tileBelowLeft;
  } else if (tileIsSlope(tileBelowRight2)) {
    slopeTile = tileBelowRight2;
  } else if (tileIsSlope(tileBelowLeft2)) {
    slopeTile = tileBelowLeft2;
  }

  if (slopeTile == TILE_GROUND_SLOPE_UP_RIGHT) {
    // low on the left, high on the right -> downhill to the right
    slopeDir = +1;
  } else if (slopeTile == TILE_GROUND_SLOPE_UP_LEFT) {
    // low on the right, high on the left -> downhill to the left
    slopeDir = -1;
  }

  // Start SMW-style slope slide:
  if (onGround && !slopeSlideActive && downPressed) {
    int slideDir = 0;
    TileType slideSlope = findSlopeBelowCenter(heroX, heroY, heroW, heroHitH, slideDir);

    // Only start a slide if there is actually a slope under/just below the center
    if (slideSlope != TILE_EMPTY && slideDir != 0) {
      slopeSlideActive    = true;
      slopeSlideOnSlope   = true;        // still on slope initially
      slopeSlideOffSlopeX = heroX;       // will be updated when we actually leave the slope

      const float SLIDE_START_SPEED = 1.8f;  // keep or tweak

      slopeSlideVX = (slideDir > 0)
        ? +SLIDE_START_SPEED
        : -SLIDE_START_SPEED;

      heroVX = slopeSlideVX;
    }
  }

  // Horizontal movement
  float maxSpeed = runPressed ? MAX_RUN_SPEED : MAX_WALK_SPEED;
  float accel    = onGround ? ACCEL_GROUND  : ACCEL_AIR;
  float decel    = onGround ? DECEL_GROUND  : DECEL_AIR;

  // --- Horizontal movement (with collisions) ---
  if (!slopeSlideActive) {
    // Normal walking / running
    if (moveInput != 0.0f) {
      float target = moveInput * maxSpeed;
      if (heroVX < target) {
        heroVX += accel;
        if (heroVX > target) heroVX = target;
      } else if (heroVX > target) {
        heroVX -= accel;
        if (heroVX < target) heroVX = target;
      }
    } else {
      if (heroVX > 0.0f) {
        heroVX -= decel;
        if (heroVX < 0.0f) heroVX = 0.0f;
      } else if (heroVX < 0.0f) {
        heroVX += decel;
        if (heroVX > 0.0f) heroVX = 0.0f;
      }
    }
  } else {
    // Sliding behavior: ignore left/right input, update slide speed instead
    const float SLIDE_SLOPE_ACCEL   = 0.10f;  // accel while on slope
    const float SLIDE_MAX_SPEED     = 3.0f;   // speed clamp
    const float SLIDE_FLAT_FRICTION = 0.02f;  // smaller = longer slide
    const float SLIDE_STOP_SPEED    = 0.05f;  // lower = slide longer

    // Determine if we are still on a slope underneath the center
    int downhillDir = 0;
    TileType slopeUnderCenter = findSlopeBelowCenter(heroX, heroY, heroW, heroHitH, downhillDir);
    bool stillOnSlope = tileIsSlope(slopeUnderCenter) && downhillDir != 0;

    if (stillOnSlope) {
      slopeSlideOnSlope = true;
      // Accelerate downhill while on slope
      slopeSlideVX += (downhillDir > 0 ? +1.0f : -1.0f) * SLIDE_SLOPE_ACCEL;
      if (slopeSlideVX >  SLIDE_MAX_SPEED) slopeSlideVX =  SLIDE_MAX_SPEED;
      if (slopeSlideVX < -SLIDE_MAX_SPEED) slopeSlideVX = -SLIDE_MAX_SPEED;
    } else if (slopeSlideOnSlope && onGround) {
      // Just left the slope – mark where flat slide begins
      slopeSlideOnSlope = false;
      slopeSlideOffSlopeX = heroX;
    }

    if (!stillOnSlope && onGround) {
      // Now on flat ground: distance-based friction
      float flatDist = fabsf(heroX - slopeSlideOffSlopeX);

      if (flatDist < SLOPE_POST_SLIDE_DIST) {
        // Within guaranteed distance: minimal friction
        if (slopeSlideVX > 0.0f) {
          slopeSlideVX -= SLIDE_FLAT_FRICTION * 0.2f;
          if (slopeSlideVX < 0.0f) slopeSlideVX = 0.0f;
        } else if (slopeSlideVX < 0.0f) {
          slopeSlideVX += SLIDE_FLAT_FRICTION * 0.2f;
          if (slopeSlideVX > 0.0f) slopeSlideVX = 0.0f;
        }
      } else {
        // Beyond guaranteed distance: normal friction, allow stopping
        if (slopeSlideVX > 0.0f) {
          slopeSlideVX -= SLIDE_FLAT_FRICTION;
          if (slopeSlideVX < 0.0f) slopeSlideVX = 0.0f;
        } else if (slopeSlideVX < 0.0f) {
          slopeSlideVX += SLIDE_FLAT_FRICTION;
          if (slopeSlideVX > 0.0f) slopeSlideVX = 0.0f;
        }

        // Stop only when nearly zero
        if (fabsf(slopeSlideVX) <= SLIDE_STOP_SPEED) {
          slopeSlideVX     = 0.0f;
          slopeSlideActive = false;
        }
      }
    }

    heroVX = slopeSlideVX;

    if (!onGround) {
      // leaving the ground cancels the slide and becomes a fall
      slopeSlideActive = false;
    }

    heroVX = slopeSlideVX;
  }

  // Propose new X based on velocity
  float newX = heroX + heroVX;

  // Clamp to world bounds first
  if (newX < 0.0f) newX = 0.0f;
  if (newX > WORLD_WIDTH - heroW) newX = WORLD_WIDTH - heroW;

  // Simplified side-collision with step-up tolerance
  const float HITBOX_INSET_X = 1.0f;
  const float MAX_SIDE_STEP_UP_PX = 4.0f;

  float top = heroY;
  float bottom = heroY + heroHitH - 1.0f;

  int tyStart = (int)(top / TILE_SIZE);
  int tyEnd = (int)(bottom / TILE_SIZE);
  if (tyStart < 0) tyStart = 0;
  if (tyEnd >= LEVEL_HEIGHT) tyEnd = LEVEL_HEIGHT - 1;

  if (heroVX > 0.0f) {
    // Moving right: check tiles at the right edge
    float rightEdge = newX + heroW - 1.0f - HITBOX_INSET_X;
    int txRight = (int)(rightEdge / TILE_SIZE);

    if (txRight >= 0 && txRight < LEVEL_WIDTH) {
      bool hitWall = false;
      for (int ty = tyStart; ty <= tyEnd && !hitWall; ++ty) {
        // Check whether a tunnel anchor covers the pixel at the hero's right edge
        int anchorTx = -1, anchorTy = -1;
        int samplePx = (int)(rightEdge);
        int samplePy = ty * TILE_SIZE + (TILE_SIZE/2);
        if (findTunnelAnchorAtPixel(samplePx, samplePy, anchorTx, anchorTy)) {
          // anchor found and its rect intersects the sample point — treat as collision
          float tunnelLeft = anchorTx * TILE_SIZE;
          float tunnelTop  = anchorTy * TILE_SIZE;
          float tunnelW = 0.0f, tunnelH = 0.0f;
          getTunnelColliderDims(getTileId(anchorTx, anchorTy), tunnelW, tunnelH);
          float tunnelRight = tunnelLeft + tunnelW;
          float tunnelBottom = tunnelTop + tunnelH;

          float heroLeft = newX;
          float heroRight = newX + heroW - 1.0f;
          float heroTopF = top;
          float heroBottomF = bottom;

          if (!(heroRight < tunnelLeft || heroLeft > tunnelRight || heroBottomF < tunnelTop || heroTopF > tunnelBottom)) {
            hitWall = true;
            break;
          }
        }

        if (hitWall) break;

        // Fall back to regular per-tile side blocking
        TileType t = getTile(txRight, ty);
        if (!tileBlocksSide(t)) continue;

        float tileTop = ty * TILE_SIZE;

        // Step-up is ONLY allowed when grounded (prevents airborne corner ghosting that causes bonk-bounce)
        if (onGround && (tileTop - heroFeetY <= MAX_SIDE_STEP_UP_PX) && (tileTop - heroFeetY >= -TILE_SIZE)) {
          continue;
        }

        hitWall = true;
      }

      if (hitWall) {
        // If the colliding tile was a tunnel anchor, compute proper push-out
        // try to find an anchor that intersects hero; prefer the rightmost anchor
        bool pushed = false;
        int anchorTx = -1, anchorTy = -1;
        int samplePx = (int)(rightEdge);
        int samplePy = tyStart * TILE_SIZE + (TILE_SIZE/2);
        if (findTunnelAnchorAtPixel(samplePx, samplePy, anchorTx, anchorTy)) {
          float tunnelLeft = anchorTx * TILE_SIZE;
          float tunnelW = 0.0f, tunnelH = 0.0f;
          getTunnelColliderDims(getTileId(anchorTx, anchorTy), tunnelW, tunnelH);
          newX = tunnelLeft - heroW;
          heroVX = 0.0f;
          pushed = true;
        }
        if (!pushed) {
          newX = txRight * TILE_SIZE - heroW;
          heroVX = 0.0f;
        }
      }
    }
  } else if (heroVX < 0.0f) {
    // Moving left: check tiles at the left edge
    float leftEdge = newX + HITBOX_INSET_X;
    int txLeft = (int)(leftEdge / TILE_SIZE);

    if (txLeft >= 0 && txLeft < LEVEL_WIDTH) {
      bool hitWall = false;
      for (int ty = tyStart; ty <= tyEnd && !hitWall; ++ty) {
        // Check whether a tunnel anchor covers the pixel at the hero's left edge
        int anchorTx = -1, anchorTy = -1;
        int samplePx = (int)(leftEdge);
        int samplePy = ty * TILE_SIZE + (TILE_SIZE/2);
        if (findTunnelAnchorAtPixel(samplePx, samplePy, anchorTx, anchorTy)) {
          float tunnelLeft = anchorTx * TILE_SIZE;
          float tunnelTop  = anchorTy * TILE_SIZE;
          float tunnelW = 0.0f, tunnelH = 0.0f;
          getTunnelColliderDims(getTileId(anchorTx, anchorTy), tunnelW, tunnelH);
          float tunnelRight = tunnelLeft + tunnelW;
          float tunnelBottom = tunnelTop + tunnelH;

          float heroLeft = newX;
          float heroRight = newX + heroW - 1.0f;
          float heroTopF = top;
          float heroBottomF = bottom;

          if (!(heroRight < tunnelLeft || heroLeft > tunnelRight || heroBottomF < tunnelTop || heroTopF > tunnelBottom)) {
            hitWall = true;
            break;
          }
        }

        if (hitWall) break;

        TileType t = getTile(txLeft, ty);
        if (!tileBlocksSide(t)) continue;

        float tileTop = ty * TILE_SIZE;
        // Step-up is ONLY allowed when grounded (prevents airborne corner ghosting that causes bonk-bounce)
        if (onGround && (tileTop - heroFeetY <= MAX_SIDE_STEP_UP_PX) && (tileTop - heroFeetY >= -TILE_SIZE)) {
          continue;
        }

        hitWall = true;
      }

      if (hitWall) {
        // If tunnel anchor caused the hit, push to its right edge
        bool pushed = false;
        int anchorTx = -1, anchorTy = -1;
        int samplePx = (int)(leftEdge);
        int samplePy = tyStart * TILE_SIZE + (TILE_SIZE/2);
        if (findTunnelAnchorAtPixel(samplePx, samplePy, anchorTx, anchorTy)) {
          float tunnelLeft = anchorTx * TILE_SIZE;
          float tunnelW = 0.0f, tunnelH = 0.0f;
          getTunnelColliderDims(getTileId(anchorTx, anchorTy), tunnelW, tunnelH);
          newX = tunnelLeft + tunnelW;
          heroVX = 0.0f;
          pushed = true;
        }
        if (!pushed) {
          newX = (txLeft + 1) * TILE_SIZE;
          heroVX = 0.0f;
        }
      }
    }
  }

  // Commit the final X after collisions
  heroX = newX;

  bool movingHoriz = fabs(heroVX) > 0.05f;

  // Jump / spin logic
  if (!onGround) {
    // airborne, nothing special here before gravity
  } else {
    // Grounded
    if (slopeSlideActive && normalJumpPressed && !prevNormalJumpPressed) {
      // Jumping out of a slide
      slopeSlideActive = false;
    }

    // ---- Crouch-drop through platforms ----
    // Sample the tile directly under Mario's feet.
    float heroFeetY   = heroY + heroHitH;
    float heroCenterX = heroX + heroW * 0.5f;
    int   feetTileX   = (int)(heroCenterX / TILE_SIZE);
    int   feetTileY   = (int)(heroFeetY  / TILE_SIZE);

    TileType tileUnderFeet = getTile(feetTileX, feetTileY);
    bool onDropPlatform = tileIsDropThroughPlatform(tileUnderFeet);

    // If we're standing on a drop-through platform and press DOWN,
    // start a drop: platforms stop counting as ground until we're below them.
    if (onDropPlatform && justPressedDown) {
      platformDropActive      = true;
      platformDropStartFeetY  = heroFeetY;
      onGround                = false;

      // Ensure we move downward through the platform
      if (heroVY < 1.0f) heroVY = 1.0f;
    }

    // ---- Normal jump / other grounded logic ----
    // Only do a normal jump if we did NOT start a drop this frame
    if (!platformDropActive && normalJumpPressed && !prevNormalJumpPressed) {
      heroVY = JUMP_SPEED;
      onGround = false;
      spinActive = false;
      slamActive = false;
      landingSquashFrames = 0;
      if (slopeSlideActive) {
        slopeSlideActive = false;
      }
    } else if (spinJumpPressed && !prevSpinJumpPressed) {
      heroVY = SPIN_JUMP_SPEED;
      onGround = false;
      spinActive = true;
      slamActive = false;
      landingSquashFrames = 0;
      if (slopeSlideActive) {
        slopeSlideActive = false;
      }
    }
  }

  prevNormalJumpPressed = normalJumpPressed;
  prevSpinJumpPressed   = spinJumpPressed;
  prevRunPressed        = runPressed;
  prevDownPressed       = downPressed;
  prevRightPressed      = rightPressed;
  prevLeftPressed       = leftPressed;

  // Vertical physics
  if (!onGround) {
    float effectiveGravity = GRAVITY;

    if (downPressed && heroVY > 2.0f) {
      slamActive = true;
      spinActive = false;
    } else if (heroVY < 0.0f) {
      slamActive = false;
    }

    if (spinActive) {
      effectiveGravity *= SPIN_GRAVITY_MULTIPLIER;
    } else if (downPressed && heroVY > 0.0f && !slamActive) {
      effectiveGravity *= FAST_FALL_MULTIPLIER;
    }

    float newY = heroY + heroVY;

    if (heroVY < 0.0f) {
      const float preBonkVX = heroVX;
      float oldTop = heroY;
      float newTop = newY;

      float left  = heroX;
      float right = heroX + heroW - 1;

      int txStart = (int)(left  / TILE_SIZE);
      int txEnd   = (int)(right / TILE_SIZE);
      if (txStart < 0) txStart = 0;
      if (txEnd   >= LEVEL_WIDTH) txEnd = LEVEL_WIDTH - 1;

      int tyStart2 = (int)(newTop / TILE_SIZE);
      int tyEnd2   = (int)(oldTop / TILE_SIZE);
      if (tyStart2 < 0) tyStart2 = 0;
      if (tyEnd2   >= LEVEL_HEIGHT) tyEnd2 = LEVEL_HEIGHT - 1;

      bool ceilingHit = false;

      for (int ty = tyStart2; ty <= tyEnd2 && !ceilingHit; ty++) {
        for (int tx = txStart; tx <= txEnd; tx++) {
          // First, check for tunnel anchors that might create a ceiling at this column
          int anchorTx = -1, anchorTy = -1;
          int samplePx = (int)((left + right) * 0.5f);
          int samplePy = ty * TILE_SIZE + (TILE_SIZE/2);
          if (findTunnelAnchorAtPixel(samplePx, samplePy, anchorTx, anchorTy)) {
            float tunnelLeft = anchorTx * TILE_SIZE;
            float tunnelW = 0.0f, tunnelH = 0.0f;
            getTunnelColliderDims(getTileId(anchorTx, anchorTy), tunnelW, tunnelH);
            float tunnelRight = tunnelLeft + tunnelW;
            float tunnelBottom = anchorTy * TILE_SIZE + tunnelH;

            float heroLeft = left;
            float heroRight = right;

            if (oldTop >= tunnelBottom && newTop <= tunnelBottom && !(heroRight < tunnelLeft || heroLeft > tunnelRight)) {
              newTop = tunnelBottom - HAT_TOP_OFFSET;

              // Bonk: stop upward motion and prevent horizontal corner-clipping that can
              // "launch" Mario sideways / through blocks after a head hit.
              heroVY = 0.5f;
              ceilingHit = true;

              // Kill horizontal motion and cancel this frame’s X movement to prevent “walking bonk bounce”.
              heroX = frameStartHeroX;
              newX  = frameStartHeroX;
              heroVX = 0.0f;
              slopeSlideVX = 0.0f;
              slopeSlideActive = false;

              // Brief window: ignore input + hard kill VX (prevents backward bounce)
              gCeilingBonkFreezeFrames = 3;
              gCeilingBonkHoldVXFrames = 3;

              // NEW: small forward carry nudge for momentum feel (safe: position-based, not VX-based)
              if (fabsf(preBonkVX) > 0.10f) {
                gCeilingBonkCarryDir = (preBonkVX < 0.0f) ? -1 : 1;
                gCeilingBonkCarryFrames = 2;
              } else {
                gCeilingBonkCarryDir = 0;
                gCeilingBonkCarryFrames = 0;
              }

              handleBlockHitFromBelow(anchorTx, anchorTy);
              break;
            }
          }

          TileType t = getTile(tx, ty);
          if (!tileIsSolid(t)) continue;

          float tileBottom = (float)((ty + 1) * TILE_SIZE);

          if (oldTop >= tileBottom && newTop <= tileBottom) {
            newTop = tileBottom - HAT_TOP_OFFSET;

            // Bonk: stop upward motion and prevent horizontal corner-clipping that can
            // "launch" Mario sideways / through blocks after a head hit.
            heroVY = 0.5f;
            ceilingHit = true;

            // Kill horizontal motion and cancel this frame’s X movement to prevent “walking bonk bounce”.
            heroX = frameStartHeroX;
            newX  = frameStartHeroX;
            heroVX = 0.0f;
            slopeSlideVX = 0.0f;
            slopeSlideActive = false;

            // Brief window: ignore input + hard kill VX (prevents backward bounce)
            gCeilingBonkFreezeFrames = 3;
            gCeilingBonkHoldVXFrames = 3;

            // NEW: small forward carry nudge for momentum feel (safe: position-based, not VX-based)
            if (fabsf(preBonkVX) > 0.10f) {
              gCeilingBonkCarryDir = (preBonkVX < 0.0f) ? -1 : 1;
              gCeilingBonkCarryFrames = 2;
            } else {
              gCeilingBonkCarryDir = 0;
              gCeilingBonkCarryFrames = 0;
            }

            handleBlockHitFromBelow(tx, ty);
            break;
          }
        }
      }

      heroY = newTop;

      // Apply tiny forward carry after a bonk (keeps momentum feel without reintroducing backward bounce).
      if (gCeilingBonkCarryFrames > 0 && gCeilingBonkCarryDir != 0) {
        const float BONK_CARRY_PX = 0.90f;  // tune 0.60–1.20

        float testX = heroX + (float)gCeilingBonkCarryDir * BONK_CARRY_PX;

        // clamp to world
        if (testX < 0.0f) testX = 0.0f;
        if (testX > WORLD_WIDTH - heroW) testX = WORLD_WIDTH - heroW;

        // Only apply if it won't collide at the current (snapped) Y
        if (!heroWouldCollideAtX(testX, heroY, heroW, heroHitH)) {
          heroX = testX;
        }

        gCeilingBonkCarryFrames--;
        if (gCeilingBonkCarryFrames == 0) gCeilingBonkCarryDir = 0;
      }
    } else {
      heroY = newY;
    }

    // Turn off drop mode once we're below the platform
    if (platformDropActive) {
      float heroFeetYNow = heroY + heroHitH;

      // Once our feet are at least one tile below where we started,
      // allow platforms to catch us again.
      if (heroFeetYNow >= platformDropStartFeetY + TILE_SIZE) {
        platformDropActive = false;
      }
    }

    heroVY += effectiveGravity;

    if (spinActive && heroVY > MAX_SPIN_FALL_SPEED) {
      heroVY = MAX_SPIN_FALL_SPEED;
    }

    // Capture approach state for stomp classification (demo-lenient).
    gHeroApproachY = heroY;
    gHeroApproachVY = heroVY;
    gHeroApproachHitH = heroHitH;
    gHeroApproachOnGround = onGround;

    float heroGroundY = computeHeroGroundY(heroX, heroY, heroHitH);

    // Allow a small snap band so Mario doesn't "hover" or jitter when going down slopes.
    const float GROUND_SNAP_BAND = 2.0f;  // pixels
    if (heroY >= heroGroundY - GROUND_SNAP_BAND) {
      if (slamActive) {
        float sampleY = heroGroundY + heroHitH;
        float left  = heroX + heroW * 0.2f;
        float right = heroX + heroW * 0.8f - 1;
        if (left < 0) left = 0;
        if (right > WORLD_WIDTH - 1) right = WORLD_WIDTH - 1;

        int txStart = (int)(left  / TILE_SIZE);
        int txEnd   = (int)(right / TILE_SIZE);
        if (txStart < 0)            txStart = 0;
        if (txEnd   >= LEVEL_WIDTH) txEnd   = LEVEL_WIDTH - 1;

        int tyBlock = (int)(sampleY / TILE_SIZE);
        if (tyBlock >= 0 && tyBlock < LEVEL_HEIGHT) {
          for (int tx = txStart; tx <= txEnd; tx++) {
            if (tileIsSolid(getTile(tx, tyBlock))) {
              handleBlockHitFromAbove(tx, tyBlock);
              break;
            }
          }
        }
      }

      if (!gJustBrokeTileThisFrame) {
        // Normal landing: snap to ground and clear vertical motion
        heroY = heroGroundY;
        heroVY = 0.0f;
        onGround = true;
        spinActive = false;
        slamActive = false;
      } else {
        // If we just broke a tile by slam, skip the immediate snap this frame
        // to avoid resolving against a removed tile. Let gravity continue.
        gJustBrokeTileThisFrame = false;
      }

      if (!downPressed) {
        landingSquashFrames = LANDING_SQUASH_DURATION;
      } else {
        landingSquashFrames = 0;
      }
    }
  } else {
    float heroGroundY = computeHeroGroundY(heroX, heroY, heroHitH);

    // Positive diff = ground is below current feet, negative = ground above.
    float diff = heroGroundY - heroY;

    // Allow small steps / slope changes (both up and down) while staying grounded.
    // Only if the ground is significantly below us do we treat it as walking off a ledge.
    const float FALL_THRESHOLD = 4.0f;

    if (diff > FALL_THRESHOLD) {
      // Ground moved far below our current Y → start falling
      onGround = false;
      
      // If sliding and left ground, cancel slide
      if (slopeSlideActive) {
        slopeSlideActive = false;
        slopeSlideVX     = 0.0f;
      }
    } else {
      // Snap to ground (gentle up/down step)
      heroY   = heroGroundY;
      heroVY  = 0.0f;
      slamActive = false;
    }
  }

  // --- Final ground / slope follow pass ---
  if (onGround) {
    // Use our ground solver, which already knows about slope tiles.
    float groundY = computeHeroGroundY(heroX, heroY, heroHitH);

    // Snap Mario to the exact ground line so he never "floats" between tiles.
    // This removes the 8×8 stepping effect when going down a slope.
    heroY = groundY;
    heroVY = 0.0f;
  }

  // Pose / animation
  if (!onGround) {
    currentPose   = spinActive ? POSE_SPIN : POSE_JUMP;
    bobOffset     = 0;
    crouchOffset  = 0;
  } else if (slopeSlideActive) {
    currentPose   = POSE_CROUCH;  // slide uses crouch sprite
    crouchOffset  = 2;
    bobOffset     = 0;
  } else if (landingSquashFrames > 0) {
    currentPose   = POSE_CROUCH;
    crouchOffset  = 2;
    bobOffset     = 0;
    landingSquashFrames--;
  } else if (downPressed && heroSize == SIZE_BIG) {
    currentPose   = POSE_CROUCH;
    crouchOffset  = 2;
    if (movingHoriz) {
      walkPhase    = (walkPhase + 1) % 20;
      bool upFrame = (walkPhase < 10);
      bobOffset    = upFrame ? 1 : 0;
    } else {
      bobOffset = 0;
    }
  } else if (movingHoriz) {
    currentPose   = POSE_STAND;  // visually still using stand; bob only
    walkPhase     = (walkPhase + 1) % 20;
    bool upFrame  = (walkPhase < 10);
    bobOffset     = upFrame ? 1 : 0;
    crouchOffset  = 0;
  } else {
    currentPose   = POSE_STAND;
    bobOffset     = 0;
    crouchOffset  = 0;
  }

  // Update mushroom (unchanged from your code)
  if (gMushroom.active) {
    if (gMushroom.rising) {
      gMushroom.y += gMushroom.vy;
      if ((gMushroom.vy < 0.0f && gMushroom.y <= gMushroom.targetY) ||
          (gMushroom.vy > 0.0f && gMushroom.y >= gMushroom.targetY)) {
        gMushroom.y  = gMushroom.targetY;
        gMushroom.vy = 0.0f;
        gMushroom.rising = false;
      }
    } else {
      gMushroom.vy += MUSH_GRAVITY;
      if (gMushroom.vy > MUSH_MAX_FALL) gMushroom.vy = MUSH_MAX_FALL;

      float newX = gMushroom.x + gMushroom.vx;
      float newY = gMushroom.y + gMushroom.vy;

      float top    = newY;
      float bottom = newY + MUSH_H - 1;

      if (gMushroom.vx > 0.0f) {
        float rightEdge = newX + MUSH_W - 1;
        int txRight = (int)(rightEdge / TILE_SIZE);
        int tyStart = (int)(top    / TILE_SIZE);
        int tyEnd   = (int)(bottom / TILE_SIZE);
        if (tyStart < 0) tyStart = 0;
        if (tyEnd   >= LEVEL_HEIGHT) tyEnd = LEVEL_HEIGHT - 1;

        bool hitWall = false;
        for (int ty = tyStart; ty <= tyEnd && !hitWall; ty++) {
          if (tileIsSolid(getTile(txRight, ty))) {
            hitWall = true;
          }
        }
        if (hitWall) {
          newX = txRight * TILE_SIZE - MUSH_W;
          gMushroom.vx = -gMushroom.vx;
        }
      } else if (gMushroom.vx < 0.0f) {
        float leftEdge = newX;
        int txLeft = (int)(leftEdge / TILE_SIZE);
        int tyStart = (int)(top    / TILE_SIZE);
        int tyEnd   = (int)(bottom / TILE_SIZE);
        if (tyStart < 0) tyStart = 0;
        if (tyEnd   >= LEVEL_HEIGHT) tyEnd = LEVEL_HEIGHT - 1;

        bool hitWall = false;
        for (int ty = tyStart; ty <= tyEnd && !hitWall; ty++) {
          if (tileIsSolid(getTile(txLeft, ty))) {
            hitWall = true;
          }
        }
        if (hitWall) {
          newX = (txLeft + 1) * TILE_SIZE;
          gMushroom.vx = -gMushroom.vx;
        }
      }

      if (newX < 0) {
        newX = 0;
        gMushroom.vx = -gMushroom.vx;
      } else if (newX > WORLD_WIDTH - MUSH_W) {
        newX = WORLD_WIDTH - MUSH_W;
        gMushroom.vx = -gMushroom.vx;
      }

      float groundY = computeMushroomGroundY(newX, newY);
      if (newY >= groundY) {
        newY = groundY;
        gMushroom.vy = 0.0f;

        if (gMushroom.vx == 0.0f) {
          gMushroom.vx = MUSH_SPEED;
        }
      }

      gMushroom.x = newX;
      gMushroom.y = newY;
    }
  }

  // Goomba pool update
  for (int gi = 0; gi < MAX_GOOMBAS; ++gi) {
    Goomba &gg = gGoombas[gi];
    if (!gg.active) continue;

    if (gFlagpole.active && gFlagpole.freezeEnemies) {
      continue;
    }

    // Handle state: squished -> countdown, then remove
    if (gg.state == GOOMBA_STATE_SQUISHED) {
      if (gg.stateTimer > 0) gg.stateTimer--;
      if (gg.stateTimer == 0) {
        gg.active = false;
        gg.state = GOOMBA_STATE_DEAD;
      }
      continue; // no movement while squished
    }

    // Apply gravity
    gg.vy += GOOMBA_GRAVITY;
    if (gg.vy > GOOMBA_MAX_FALL) gg.vy = GOOMBA_MAX_FALL;

    float newXg = gg.x + gg.vx;
    float newYg = gg.y + gg.vy;

    // Clamp horizontal world bounds
    if (newXg < 0.0f) {
      newXg = 0.0f;
      gg.vx = -gg.vx;
    } else if (newXg > WORLD_WIDTH - GOOMBA_W) {
      newXg = WORLD_WIDTH - GOOMBA_W;
      gg.vx = -gg.vx;
    }

    // Turn around before stepping into lava
    int gDirNow = (gg.vx < 0.0f) ? -1 : 1;
    if (mobHazardAhead(newXg, gg.y, GOOMBA_W, GOOMBA_H, gDirNow)) {
      gg.vx = -gg.vx;
      newXg = gg.x;
      gDirNow = (gg.vx < 0.0f) ? -1 : 1;
    }

    // Side collision checks (feet-band probe)
    const float HITBOX_INSET_X_G = 1.0f;

    if (gg.vx > 0.0f) {
      float rightEdge = newXg + GOOMBA_W - 1.0f;
      int txRight = (int)(rightEdge / TILE_SIZE);
      bool hitWall = goombaSideBlocked(newXg, newYg, GOOMBA_W, GOOMBA_H, 1, HITBOX_INSET_X_G);
      if (hitWall) {
        // Wall contact must ALWAYS reverse (no step-up bypass).
        newXg = txRight * TILE_SIZE - GOOMBA_W;
        gg.vx = -gg.vx;
      }
    } else if (gg.vx < 0.0f) {
      float leftEdge = newXg;
      int txLeft = (int)(leftEdge / TILE_SIZE);
      bool hitWall = goombaSideBlocked(newXg, newYg, GOOMBA_W, GOOMBA_H, -1, HITBOX_INSET_X_G);
      if (hitWall) {
        // Wall contact must ALWAYS reverse (no step-up bypass).
        newXg = (txLeft + 1) * TILE_SIZE;
        gg.vx = -gg.vx;
      }
    }

    // Vertical / ground snap
    float groundYg = computeGoombaGroundY(newXg, newYg);
    if (newYg >= groundYg) {
      newYg = groundYg;
      gg.vy = 0.0f;
      if (gg.vx == 0.0f) {
        // Always restart moving left (deterministic)
        gg.vx = -GOOMBA_SPEED;
      }
    }

    gg.x = newXg;
    gg.y = newYg;

    // simple 2-frame walk animation: only when alive, grounded, and moving
    bool grounded = (fabs(newYg - groundYg) < 0.5f);
    if (gg.state == GOOMBA_STATE_ALIVE && grounded && fabsf(gg.vx) > 0.05f) {
      gg.animTimer++;
      if (gg.animTimer >= 8) {
        gg.animTimer = 0;
        gg.animFrame ^= 1;
      }
    } else {
      gg.animFrame = 0;
      gg.animTimer = 0;
    }
  }

  // Koopas (Goomba-like walkers)
  updateKoopas();

  // Koopa shells (carry / kick / sliding)
  updateShells(runPressed, runJustReleased, upPressed);

  // Piranha plants (SMW-style rise/hide)
  updatePiranhas();

  // Update tile bounce animations
  for (int ty = 0; ty < LEVEL_HEIGHT; ty++) {
    for (int tx = 0; tx < LEVEL_WIDTH; tx++) {
      uint8_t &timer = tileAnimTimer[ty][tx];
      if (timer > 0) {
        timer--;
        if (timer >= 4)      tileOffset[ty][tx] = -2;
        else if (timer >= 2) tileOffset[ty][tx] = -1;
        else                 tileOffset[ty][tx] = 0;
      } else {
        tileOffset[ty][tx] = 0;
      }
    }
  }

  // Update brick fragment particles
  updateBrickFragments();

  // Update coin pop particles (SMW)
  updateCoinPopsSMW();

  // ─────────────────────────────────────────────
  // Pickups / collectibles (mushroom, coins)
  // ─────────────────────────────────────────────
  tryPickupMushroom();   // grow power-up
  checkCoinPickup();     // coin collection
  // Shell interactions (shells can kill enemies; Mario can pick up/kick)
  checkShellEnemyCollisions();
  // Enemy collisions (stomp / damage)
  checkPiranhaHeroCollisions();
  checkShellHeroCollisions(runPressed);
  checkGoombaHeroCollisions();
  checkKoopaHeroCollisions();
  checkHeroLavaDamage();

  // Checkpoint: touch + flip animation
  updateCheckpointTouch();
  updateCheckpointFlipAnimation();

  // ─────────────────────────────────────────────
  // Grow animation (small ↔ big)
  // ─────────────────────────────────────────────

  if (isGrowing) {
    heroVX       = 0.0f;
    currentPose  = POSE_STAND;
    bobOffset    = 0;
    crouchOffset = 0;

    growFrameCounter++;
    if (growFrameCounter >= GROW_STEP_FRAMES) {
      growFrameCounter = 0;
      growStep++;

      if (heroSize == SIZE_SMALL) {
        setHeroSize(SIZE_BIG);
      } else {
        setHeroSize(SIZE_SMALL);
      }

      if (growStep >= GROW_STEPS) {
        setHeroSize(SIZE_BIG);
        isGrowing = false;
#ifdef DEBUG_GAME
        Serial.println("Power-up: grow animation complete");
#endif
      }
    }
  }

  // Map timer countdown (once per second)
  if (!gFlagpole.active) {
    tickMapTimer();
  }

  // Update camera after movement
  updateCamera();
}

// ─────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────
void loopGameplay() {
  int bobOffset    = 0;
  int crouchOffset = 0;
  currentPose      = POSE_STAND;

  // Update pipe transition with delta time
  uint32_t realNowMs = millis();

  static uint32_t lastMs = 0;
  if (lastMs == 0) lastMs = realNowMs;
  float dtSeconds = (realNowMs - lastMs) / 1000.0f;
  if (dtSeconds > 0.1f) dtSeconds = 0.1f; // clamp large deltas
  lastMs = realNowMs;

  // For recorded/auto-play demos we need deterministic timing. Use a fixed 60Hz step
  // so tunnel transitions and other time-based systems replay consistently.
  if (gDemoIsRecording || gDemoPlaybackActive || (gControlMode == AUTO_PLAY)) {
    dtSeconds = 1.0f / 60.0f;
  }

  // Advance deterministic demo clock (or keep it roughly synced when not in demo contexts).
  if (demoTimingActive()) {
    demoSimAdvanceSeconds(dtSeconds);
  } else {
    // Keep it non-stale in case we enter demo contexts without an explicit reset.
    gDemoSimNowUs = (uint64_t)realNowMs * 1000ULL;
  }

  uint32_t nowMs = gameplayNowMs();

  // Manual Map1 input recorder (records the effective input this tick)
  tickDemoRecorderPerGameplayTick(nowMs);

  updatePipeTransition(nowMs, dtSeconds);

  // Death sequence update (pop/fall/fade/game over/respawn)
  updateDeathSequence(nowMs, dtSeconds);

  updateGame(bobOffset, crouchOffset);

#if DEBUG_DEMO_MAP2_TRACE
  // Always keep a short rolling window during Map2 playback.
  if (gDemoPlaybackActive && gDemoPlaybackPlaying && gCurrentMapId == 2) {
    demoTracePush(nowMs);
  }
#endif

  demoDeterminismDebugPrint(nowMs);

  // ─────────────────────────────────────────────
  // BLACKOUT PHASE: Prevent 1-frame flash during teleport
  // ─────────────────────────────────────────────
  // If we're in HOLD_BLACK phase or fade is at full black, render NOTHING
  // except pure black screen. This prevents world/HUD from briefly appearing
  // during teleport.
  bool isBlackout = (gPipeTransition.phase == PIPE_PHASE_HOLD_BLACK) ||
                    (gPipeFadeAmount >= 254); // fully black threshold
  bool isDeathGameOverBlack = (gDeath.active && gDeath.phase == DEATH_BLACK_GAMEOVER);
  
  if (isBlackout) {
    // Pure black screen - no world. During death, draw GAME OVER text.
    matrix->fillScreen(0);

    if (isDeathGameOverBlack) {
      // Split the 40x5 art into two 20x5 halves so GAME and OVER can slide separately.
      const int y = 30;
      const uint8_t* src = (const uint8_t*)&Game_Over_Array[0][0];
      blitHudArraySubrect(src, 40, 5, (int)gDeath.gameX, y, 0, 0, 20, 5);
      blitHudArraySubrect(src, 40, 5, (int)gDeath.overX, y, 20, 0, 20, 5);
    }

    matrix->flipDMABuffer();
    tickNetworkAndTime();
    delay(16);
    return; // Skip all rendering
  }

  // Draw base scene (world + HUD)
  // Safety: during death fade-out/black (before reset), freeze the world render.
  // This avoids any possibility of drawing the old world with an updated camera.
  if (!deathBlocksWorldRender()) {
    drawScene(bobOffset, crouchOffset);
  }

  // Pipe fade overlay (SMW-ish iris shutter)
  // Fade HUD only during FADE_OUT / FADE_IN phases.
  if (gPipeFadeAmount > 0) {
    const int W = matrix->width();
    const int H = matrix->height();

    bool fadeHudNow =
      (gPipeTransition.active &&
       (gPipeTransition.phase == PIPE_PHASE_FADE_OUT ||
        gPipeTransition.phase == PIPE_PHASE_FADE_IN))
      || (gFlagpole.active && (gFlagpole.state == FLAGPOLE_STATE_FADE_OUT || gFlagpole.state == FLAGPOLE_STATE_FADE_IN))
      || attractIntroActive()
      || isDeathFading();

    const int worldTop    = fadeHudNow ? 0 : HUD_HEIGHT_PIXELS;
    const int worldBottom = H;
    const int worldHeight = worldBottom - worldTop;

    // coverage: 0.0 = no black, 1.0 = fully black
    float coverage = (float)gPipeFadeAmount / 255.0f;
    if (coverage < 0.0f) coverage = 0.0f;
    if (coverage > 1.0f) coverage = 1.0f;

    // Compute how tall the "open window" is.
    // At coverage=0 -> open window is full world.
    // At coverage=1 -> open window is 0.
    int openHeight = (int)((1.0f - coverage) * (float)worldHeight);
    if (openHeight < 0) openHeight = 0;
    if (openHeight > worldHeight) openHeight = worldHeight;

    // Center the open window vertically in the world area
    int openTop = worldTop + (worldHeight - openHeight) / 2;
    int openBottom = openTop + openHeight;

    uint16_t blackColor = matrix->color565(0, 0, 0);

    // Draw black above open window (worldTop -> openTop)
    for (int y = worldTop; y < openTop; ++y) {
      for (int x = 0; x < W; ++x) {
        matrix->drawPixel(x, y, blackColor);
      }
    }

    // Draw black below open window (openBottom -> worldBottom)
    for (int y = openBottom; y < worldBottom; ++y) {
      for (int x = 0; x < W; ++x) {
        matrix->drawPixel(x, y, blackColor);
      }
    }
  }

  // Flip buffer once per frame (stable rendering)
  matrix->flipDMABuffer();

  // Network / NTP ticking (updates gHudClock once per second)
  tickNetworkAndTime();

  delay(16); // ~60 FPS
}

void drawPauseMenuBackground() {
  // Background is now part of the complete sprite array, no separate drawing needed
}

// ─────────────────────────────────────────────
// WEB PORTAL IP Address Helper
// ─────────────────────────────────────────────
String getPortalIPString() {
  // If connected to WiFi (STA mode), return local IP
  if (WiFi.status() == WL_CONNECTED) {
    return WiFi.localIP().toString();
  }
  
  // If SoftAP is running, return AP IP
  IPAddress ap = WiFi.softAPIP();
  if (ap[0] != 0) {
    return ap.toString();
  }
  
  // No WiFi available
  return "NO WIFI";
}

// Draw a simple IP address string using tiny glyphs
void drawIPAddress(int x, int y, const String& ip, uint16_t color) {
  if (!matrix) return;
  
  int cursorX = x;
  for (size_t i = 0; i < ip.length(); i++) {
    char c = ip[i];
    
    if (c >= '0' && c <= '9') {
      // Draw digit
      uint8_t digit = c - '0';
      drawTinyGlyph(cursorX, y, digit, color);
      cursorX += 4;  // 3px glyph + 1px space
    } else if (c == '.') {
      // Draw dot (single pixel)
      if (cursorX >= 0 && cursorX < MATRIX_WIDTH && y + 4 >= 0 && y + 4 < MATRIX_HEIGHT) {
        matrix->drawPixel(cursorX + 1, y + 4, color);  // Bottom row of glyph space
      }
      cursorX += 3;  // 1px dot + 2px space
    } else if (c == ' ') {
      cursorX += 4;
    }
    // For other characters (like 'N', 'O', 'W', 'I', 'F'), just skip
    // or add custom glyph support if needed
  }
}

void drawPauseOverlay() {
  if (!matrix) return;

  // Position constants for centering menus on middle panel (x=64..127, center=96)
  constexpr int PAUSE_MENU_X_46 = 73;  // 96 - (46/2) = 73 (standard menus)
  constexpr int PAUSE_MENU_X_55 = 69;  // 96 - (55/2) = 69 (WEB_PORTAL menu)
  constexpr int PAUSE_Y = 8;           // Y position (always 8)

  // Determine position and size based on current screen
  int PAUSE_X;
  int PAUSE_W;
  const int PAUSE_H = PAUSE_MENU_HEIGHT;  // 48 (always)

  // Colors
  uint16_t redCap     = matrix->color565(200, 40, 40);   // palette 2
  uint16_t brownStalk = matrix->color565(180, 140, 90);  // palette 3
  uint16_t whiteSpots = matrix->color565(255, 255, 255); // palette 6

  uint16_t whiteText  = matrix->color565(255, 255, 255); // unselected text
  uint16_t blackText  = matrix->color565(0, 0, 0);       // selected text

  // Determine which array to use and selection index based on current screen
  const uint8_t* arrayPtr = nullptr;
  int currentSelection = -1;
  int itemCount = 0;
  bool enableHighlight = true;  // Disable highlighting for viewer screens
  
  if (gPauseScreen == PAUSE_SCREEN_MAIN) {
    arrayPtr = (const uint8_t*)&Paused_Reset_Maps_Stats_Resume_Array[0][0];
    currentSelection = (int)gPauseSelection;
    itemCount = 4;  // RESET, MAPS, OPTIONS, RESUME (STATS removed)
    PAUSE_W = 46;   // Standard width
    PAUSE_X = PAUSE_MENU_X_46;  // x=73
  } else if (gPauseScreen == PAUSE_SCREEN_MAPS) {
    arrayPtr = (const uint8_t*)&Paused_Map1_Map2_Map3_Build_Array[0][0];
    currentSelection = (int)gMapsSelection;
    itemCount = 4;  // MAP 1, MAP 2, MAP 3, BUILD
    PAUSE_W = 46;   // Standard width
    PAUSE_X = PAUSE_MENU_X_46;  // x=73
  } else if (gPauseScreen == PAUSE_SCREEN_OPTIONS) {
    arrayPtr = (const uint8_t*)&Paused_WIFI_SCREEN_STATS_BACK_Array[0][0];
    currentSelection = (int)gOptionsSelection;
    itemCount = 4;  // WI-FI, SCREEN, STATS, BACK
    PAUSE_W = 46;   // Standard width
    PAUSE_X = PAUSE_MENU_X_46;  // x=73
  } else if (gPauseScreen == PAUSE_SCREEN_WEB_PORTAL) {
    arrayPtr = (const uint8_t*)&Paused_WEB_PORTAL_Array[0][0];
    currentSelection = (int)gWebPortalSelection;  // BACK is always selected
    itemCount = 1;  // One selectable item: BACK
    enableHighlight = true;  // Enable highlighting for BACK
    PAUSE_W = 55;   // WEB_PORTAL is wider
    PAUSE_X = PAUSE_MENU_X_55;  // x=69 (centered for 55px width)
  } else { // PAUSE_SCREEN_BRIGHTNESS
    arrayPtr = (const uint8_t*)&Paused_BRIGHTNESS_Array[0][0];
    currentSelection = (int)gBrightnessSelection;  // BAR or BACK
    itemCount = 2;  // Two selectable items: BAR and BACK
    enableHighlight = true;  // Enable highlighting
    PAUSE_W = 55;   // BRIGHTNESS is wider (same as WEB_PORTAL)
    PAUSE_X = 68;   // x=68 per sprite comment (slightly different from WEB_PORTAL)
  }

  // Map LOCAL sprite rows → which option they belong to
  // (row numbers are in *sprite space*, 0..47)
  auto getOptionForRow = [itemCount](int y) -> int {
    if (gPauseScreen == PAUSE_SCREEN_MAIN && itemCount == 4) {
      // MAIN screen: RESET / MAPS / OPTIONS / RESUME (4 items, STATS removed)
      // Row ranges from Paused_Reset_Maps_Stats_Resume_Array:
      // Red section:
      // RESET   rows 15–19 → index 0
      // MAPS    rows 23–27 → index 1
      // Green section:
      // OPTIONS rows 31–35 → index 2
      // RESUME  rows 39–43 → index 3
      if (y >= 15 && y <= 19) return 0; // RESET
      if (y >= 23 && y <= 27) return 1; // MAPS
      if (y >= 31 && y <= 35) return 2; // OPTIONS
      if (y >= 39 && y <= 43) return 3; // RESUME
      return -1;
    } else if (gPauseScreen == PAUSE_SCREEN_MAPS && itemCount == 4) {
      // MAPS screen: MAP 1 / MAP 2 / MAP 3 / BUILD (4 items)
      // Matches the standard 46x48 menu row layout.
      if (y >= 15 && y <= 19) return 0; // MAP 1
      if (y >= 23 && y <= 27) return 1; // MAP 2
      if (y >= 31 && y <= 35) return 2; // MAP 3
      if (y >= 39 && y <= 43) return 3; // BUILD
      return -1;
    } else if (gPauseScreen == PAUSE_SCREEN_OPTIONS && itemCount == 4) {
      // OPTIONS screen: WI-FI / SCREEN / STATS / BACK (4 items)
      // Row ranges from Paused_WIFI_SCREEN_STATS_BACK_Array:
      // WI-FI   rows 15–19
      // SCREEN  rows 23–27  
      // STATS   rows 31–35 (estimated)
      // BACK    rows 39–43 (estimated)
      if (y >= 15 && y <= 19) return 0; // WI-FI
      if (y >= 23 && y <= 27) return 1; // SCREEN
      if (y >= 31 && y <= 35) return 2; // STATS
      if (y >= 39 && y <= 43) return 3; // BACK
    } else if (itemCount == 2 && gPauseScreen == PAUSE_SCREEN_BRIGHTNESS) {
      // BRIGHTNESS screen: BAR (rows 15–19) and BACK (rows 23–27)
      if (y >= 15 && y <= 19) return 0; // BAR
      if (y >= 23 && y <= 27) return 1; // BACK
    } else if (itemCount == 1) {
      // Single-item screens: WEB_PORTAL or BRIGHTNESS (BACK only)
      if (gPauseScreen == PAUSE_SCREEN_WEB_PORTAL) {
        // WEB_PORTAL: BACK on rows 30–34 (brown stem)
        if (y >= 30 && y <= 34) return 0; // BACK
      } else if (gPauseScreen == PAUSE_SCREEN_BRIGHTNESS) {
        // BRIGHTNESS: BACK on rows 23–27 (brown stem)
        if (y >= 23 && y <= 27) return 0; // BACK
      }
    }
    return -1;
  };

  for (int localY = 0; localY < PAUSE_H; ++localY) {
    for (int localX = 0; localX < PAUSE_W; ++localX) {
      // Read from the selected array (MAIN or OPTIONS screen)
      uint8_t v = pgm_read_byte(arrayPtr + (localY * PAUSE_W) + localX);
      if (v == 0) continue;  // transparent

      int screenX = PAUSE_X + localX;
      int screenY = PAUSE_Y + localY;

      // Hard clip → never wrap around top/bottom
      if (screenX < 0 || screenX >= MATRIX_WIDTH ||
          screenY < 0 || screenY >= MATRIX_HEIGHT) {
        continue;
      }

      uint16_t color = 0;

      // 9 = glyph pixels for text; recolor based on current selection
      if (v == 9) {
        if (enableHighlight) {
          int opt = getOptionForRow(localY);
          if (opt >= 0 && opt == currentSelection) {
            color = blackText;   // selected (filled black)
          } else {
            color = whiteText;   // unselected
          }
        } else {
          color = whiteText;  // WEB_PORTAL: all text white (no selection)
        }
      } else {
        // Mushroom & panel pixels:
        // 2 = red cap/background, 3 = stalk, 6 = white PAUSED / spots,
        // 1 = highlight/shading (treat as stalk color here).
        switch (v) {
          case 2: color = redCap;     break;
          case 3: color = brownStalk; break;
          case 6: color = whiteSpots; break;
          case 1: color = brownStalk; break; // or a skin/shadow color if you like
          default:
            // If any other index sneaks in, show it as white so it's visible.
            color = whiteSpots;
            break;
        }
      }

      matrix->drawPixel(screenX, screenY, color);
    }
  }

  // ─────────────────────────────────────────────
  // WEB PORTAL: Overlay IP Address
  // ─────────────────────────────────────────────
  if (gPauseScreen == PAUSE_SCREEN_WEB_PORTAL) {
    // WEB_PORTAL menu is 55px wide, centered at x=69, y=8
    constexpr int WEB_MENU_X = 69;  // PAUSE_MENU_X_55
    constexpr int WEB_MENU_Y = 8;
    constexpr int WEB_MENU_WIDTH = 55;
    
    // IP placeholder band position (where "000.000.000.000" is in sprite)
    constexpr int IP_ROW_OFFSET = 23;   // Y offset from menu top (moved up 3px)
    constexpr int IP_BOX_LEFT_PAD = 3;  // Left padding inside menu border
    constexpr int IP_BOX_WIDTH = 49;    // Width of IP text area (55 - 6px borders)
    constexpr int IP_BOX_HEIGHT = 5;    // Tiny font height

    // Get current IP address (no leading zeros: 192.168.1.24)
    String ipStr = getPortalIPString();

    // Calculate text width for centering
    // Tiny font: 4px per digit, 3px per dot
    int textWidth = 0;
    for (size_t i = 0; i < ipStr.length(); i++) {
      if (ipStr[i] == '.') textWidth += 3;
      else textWidth += 4;  // Digit or letter
    }
    
    // Position for IP box and text (both moved up 3px)
    int ipBoxX = WEB_MENU_X + IP_BOX_LEFT_PAD;
    int ipBoxY = WEB_MENU_Y + IP_ROW_OFFSET;
    int textX = ipBoxX + (IP_BOX_WIDTH - textWidth) / 2;
    int textY = ipBoxY;

    // Clear the placeholder area completely (erase magenta "000.000.000.000")
    // Oversized clear to ensure ALL placeholder pixels are removed
    // Use red mushroom cap background color (palette index 2)
    uint16_t bgColor = matrix->color565(200, 40, 40);
    int clearX = ipBoxX - 2;  // Expand 2px left
    int clearY = ipBoxY - 1;  // Expand 1px up
    int clearW = IP_BOX_WIDTH + 4;  // Expand 2px left + 2px right
    int clearH = IP_BOX_HEIGHT + 2;  // Expand 1px up + 1px down
    
    for (int cy = 0; cy < clearH; cy++) {
      for (int cx = 0; cx < clearW; cx++) {
        int px = clearX + cx;
        int py = clearY + cy;
        if (px >= 0 && px < MATRIX_WIDTH && py >= 0 && py < MATRIX_HEIGHT) {
          matrix->drawPixel(px, py, bgColor);
        }
      }
    }

    // Draw IP address text (white)
    uint16_t ipColor = matrix->color565(255, 255, 255);
    drawIPAddress(textX, textY, ipStr, ipColor);
  }

  // ─────────────────────────────────────────────
  // BRIGHTNESS: Overlay filled bar portion + cursor indicator
  // ─────────────────────────────────────────────
  if (gPauseScreen == PAUSE_SCREEN_BRIGHTNESS) {
    // BRIGHTNESS menu is 55px wide at x=68, y=8
    // Magenta bars (palette 9) are on rows 15-19 (5 rows tall)
    constexpr int BRIGHTNESS_MENU_X = 68;
    constexpr int BRIGHTNESS_MENU_Y = 8;
    constexpr int BAR_ROW_START = 15;  // First row with bars
    constexpr int BAR_ROW_COUNT = 5;   // Height of bars
    
    // Scan row 15 (first bar row) to find all magenta (palette 9) columns
    uint8_t barPositions[30];  // Max 30 bars
    uint8_t barCount = 0;
    
    for (int localX = 0; localX < 55 && barCount < 30; localX++) {
      uint8_t v = pgm_read_byte(&Paused_BRIGHTNESS_Array[BAR_ROW_START][localX]);
      if (v == 9) {  // Magenta bar
        barPositions[barCount++] = localX;
      }
    }
    
    if (barCount > 0) {
      // Map brightness level (0..255) to brightness step (0..barCount)
      // brightnessStep = how many bars should be FILLED (white)
      // Remaining bars should be BLACK (empty)
      int brightnessStep = (gBrightnessLevel * barCount) / 255;
      if (brightnessStep > barCount) brightnessStep = barCount;
      
      // Colors
      uint16_t whiteFill = matrix->color565(255, 255, 255);  // Filled bars (white)
      uint16_t blackEmpty = matrix->color565(0, 0, 0);       // Empty bars (palette 0)
      
      // Draw ALL bars: filled bars = white, empty bars = black
      for (int i = 0; i < barCount; i++) {
        int localX = barPositions[i];
        int screenX = BRIGHTNESS_MENU_X + localX;
        
        // Choose color: filled (white) or empty (black)
        uint16_t barColor = (i < brightnessStep) ? whiteFill : blackEmpty;
        
        // Fill all 5 rows of this bar column
        for (int row = 0; row < BAR_ROW_COUNT; row++) {
          int screenY = BRIGHTNESS_MENU_Y + BAR_ROW_START + row;
          if (screenX >= 0 && screenX < MATRIX_WIDTH && screenY >= 0 && screenY < MATRIX_HEIGHT) {
            matrix->drawPixel(screenX, screenY, barColor);
          }
        }
      }
      
      // Draw cursor indicator ONLY if BAR is selected
      if (gBrightnessSelection == BRIGHTNESS_BAR && brightnessStep > 0) {
        // Draw small white caret/triangle above the current bar position
        // Position: above the last filled bar (brightnessStep - 1)
        uint16_t whiteCursor = matrix->color565(255, 255, 255);
        int cursorBarIndex = brightnessStep - 1;
        if (cursorBarIndex >= 0 && cursorBarIndex < barCount) {
          int cursorLocalX = barPositions[cursorBarIndex];
          int cursorScreenX = BRIGHTNESS_MENU_X + cursorLocalX;
          int cursorY = BRIGHTNESS_MENU_Y + BAR_ROW_START - 2;  // 2 pixels above bar
          
          // Draw small downward-pointing triangle (caret): ^
          // Center pixel
          if (cursorScreenX >= 0 && cursorScreenX < MATRIX_WIDTH && cursorY >= 0 && cursorY < MATRIX_HEIGHT) {
            matrix->drawPixel(cursorScreenX, cursorY, whiteCursor);
          }
          // Bottom-left and bottom-right pixels
          if (cursorScreenX - 1 >= 0 && cursorScreenX - 1 < MATRIX_WIDTH && cursorY + 1 >= 0 && cursorY + 1 < MATRIX_HEIGHT) {
            matrix->drawPixel(cursorScreenX - 1, cursorY + 1, whiteCursor);
          }
          if (cursorScreenX + 1 >= 0 && cursorScreenX + 1 < MATRIX_WIDTH && cursorY + 1 >= 0 && cursorY + 1 < MATRIX_HEIGHT) {
            matrix->drawPixel(cursorScreenX + 1, cursorY + 1, whiteCursor);
          }
        }
      }
    }
  }
}

void exitPauseAndConsumeInput(bool doResetWorld) {
  if (doResetWorld) {
    resetWorld();
  }

  // Consume current button state so A/B/Start don't immediately trigger jumps/pause next frame
  prevNormalJumpPressed = gInput.btnJump;
  prevSpinJumpPressed   = gInput.btnSpin;
  prevResetPressed      = gInput.btnStart;  // FIX: Consume START button to prevent re-pause

  // Stop any residual movement from the stick while we're at it.
  gInput.rawMoveX = 0;
  gInput.moveAxis = 0.0f;

  // Return to gameplay and reset all pause state
  gGameMode       = GAME_MODE_PLAYING;
  gPauseScreen    = PAUSE_SCREEN_MAIN;
  gPauseSelection = PAUSE_RESET;  // Default selection to RESTART
  gOptionsSelection = OPTIONS_WIFI;
  
  Serial.println("[RESUME] exitPauseAndConsumeInput -> gameplay resumed");
  Serial.printf("[RESUME] State: gGameMode=%d gPauseScreen=%d\n", (int)gGameMode, (int)gPauseScreen);
}

void loopPauseMenu() {
  // Service web server in pause menu too
  server.handleClient();
  ElegantOTA.loop();
  delay(0);

  static bool prevPauseBtn = false;
  static bool prevJumpBtn  = false; // B (Jump) = confirm selection
  static bool prevSpinBtn  = false; // A (Spin) = back/resume
  static bool prevDown     = false;
  static bool prevUp       = false;

  // Read current relevant inputs
  bool pauseBtn = gInput.btnStart; // Start button
  bool jumpBtn  = gInput.btnJump;  // B = confirm
  bool spinBtn  = gInput.btnSpin;  // A = back
  bool downNow  = gInput.btnDown;  // D-pad down
  bool upNow    = gInput.btnUp;    // ADDED: D-Pad UP navigation

  // ----- 1) Pause/Start while paused → resume (behave like Resume option) -----
  if (pauseBtn && !prevPauseBtn) {
    // Just resume, do NOT reset world
    exitPauseAndConsumeInput(false);
    // Important: consume the button press to prevent re-triggering pause
    prevPauseBtn = pauseBtn;
    prevResetPressed = true;  // prevent immediate re-pause in GAME_MODE_PLAYING
    return;
  }
  prevPauseBtn = pauseBtn;

  // ----- 2) D-pad Down/Up: navigate current screen's menu -----
  if (gPauseScreen == PAUSE_SCREEN_MAIN) {
    // Main screen: 4 items (RESET, MAPS, OPTIONS, RESUME)
    if (downNow && !prevDown) {
      gPauseSelection = (PauseSelection)((gPauseSelection + 1) % 4);
      Serial.printf("[PAUSE] MAIN selection changed to: %d\n", (int)gPauseSelection);
    }
    if (upNow && !prevUp) {
      gPauseSelection = (PauseSelection)((gPauseSelection + 3) % 4);  // +3 = -1 with wraparound
      Serial.printf("[PAUSE] MAIN selection changed to: %d\n", (int)gPauseSelection);
    }
  } else if (gPauseScreen == PAUSE_SCREEN_MAPS) {
    // Maps screen: 4 items (MAP 1, MAP 2, MAP 3, BUILD)
    if (downNow && !prevDown) {
      gMapsSelection = (MapsSelection)((gMapsSelection + 1) % 4);
    }
    if (upNow && !prevUp) {
      gMapsSelection = (MapsSelection)((gMapsSelection + 3) % 4);
    }
  } else if (gPauseScreen == PAUSE_SCREEN_OPTIONS) {
    // Options screen: 4 items (WI-FI, SCREEN, STATS, BACK)
    if (downNow && !prevDown) {
      gOptionsSelection = (OptionsSelection)((gOptionsSelection + 1) % 4);
    }
    if (upNow && !prevUp) {
      gOptionsSelection = (OptionsSelection)((gOptionsSelection + 3) % 4);  // +3 = -1 with wraparound
    }
  } else if (gPauseScreen == PAUSE_SCREEN_WEB_PORTAL) {
    // WEB_PORTAL screen: only one item (BACK), UP/DOWN do nothing
    gWebPortalSelection = WEBPORTAL_BACK;  // Always on BACK
  } else if (gPauseScreen == PAUSE_SCREEN_BRIGHTNESS) {
    // BRIGHTNESS screen: 2 items (BAR and BACK)
    if (downNow && !prevDown) {
      gBrightnessSelection = (BrightnessSelection)((gBrightnessSelection + 1) % 2);
    }
    if (upNow && !prevUp) {
      gBrightnessSelection = (BrightnessSelection)((gBrightnessSelection + 1) % 2);  // +1 = toggle
    }
  }
  // WEB_PORTAL screen: no navigation (viewer screen)
  prevDown = downNow;
  prevUp = upNow;

  // ----- 2b) LEFT/RIGHT: adjust brightness (BRIGHTNESS screen only) -----
  static bool prevLeft = false;
  static bool prevRight = false;
  bool leftNow = (gInput.rawMoveX < 0);  // D-pad left
  bool rightNow = (gInput.rawMoveX > 0);  // D-pad right
  
  if (gPauseScreen == PAUSE_SCREEN_BRIGHTNESS && gBrightnessSelection == BRIGHTNESS_BAR) {
    constexpr uint8_t BRIGHTNESS_STEP = 17;  // 255/15 ≈ 17 (15 steps)
    
    if (leftNow && !prevLeft) {
      // Decrease brightness
      if (gBrightnessLevel >= BRIGHTNESS_STEP) {
        gBrightnessLevel -= BRIGHTNESS_STEP;
      } else {
        gBrightnessLevel = 0;
      }
      matrix->setBrightness8(gBrightnessLevel);
      Serial.print("[BRIGHTNESS] Decreased to: ");
      Serial.println(gBrightnessLevel);
    }
    
    if (rightNow && !prevRight) {
      // Increase brightness
      if (gBrightnessLevel <= 255 - BRIGHTNESS_STEP) {
        gBrightnessLevel += BRIGHTNESS_STEP;
      } else {
        gBrightnessLevel = 255;
      }
      matrix->setBrightness8(gBrightnessLevel);
      Serial.print("[BRIGHTNESS] Increased to: ");
      Serial.println(gBrightnessLevel);
    }
  }
  prevLeft = leftNow;
  prevRight = rightNow;

  // ----- 3) B (Jump) = confirm current selection -----
  if (jumpBtn && !prevJumpBtn) {
    Serial.printf("[PAUSE] Confirm pressed! Screen=%d Selection=%d\n", (int)gPauseScreen, (int)gPauseSelection);
    if (gPauseScreen == PAUSE_SCREEN_MAIN) {
      // Main screen actions (5 items)
      switch (gPauseSelection) {
        case PAUSE_RESET:
          // Reset current map/world and respawn using spawn marker
          resetWorld();
          exitPauseAndConsumeInput(false);
          break;

        case PAUSE_MAPS:
          // Enter MAPS submenu (do not unpause)
          gPauseScreen = PAUSE_SCREEN_MAPS;
          // Default cursor to current active map
          switch (gCurrentMapId) {
            case MAP_ID_OVERWORLD_1: gMapsSelection = MAPS_MAP_1; break;
            case MAP_ID_OVERWORLD_2: gMapsSelection = MAPS_MAP_2; break;
            case MAP_ID_OVERWORLD_3: gMapsSelection = MAPS_MAP_3; break;
            case MAP_ID_BUILD:
            default:                gMapsSelection = MAPS_BUILD; break;
          }
          Serial.println("[PAUSE] Switched to MAPS screen");
          break;

        case PAUSE_OPTIONS:
          // Switch to OPTIONS screen (do not unpause)
          gPauseScreen = PAUSE_SCREEN_OPTIONS;
          gOptionsSelection = OPTIONS_WIFI; // Reset to first option
          Serial.println("[PAUSE] Switched to OPTIONS screen");
          break;

        case PAUSE_RESUME:
          // Resume gameplay
          Serial.println("Resume selected -> unpausing");
          exitPauseAndConsumeInput(false);
          break;
      }
    } else if (gPauseScreen == PAUSE_SCREEN_MAPS) {
      // Maps screen actions (4 items): set map ID, reset/load, and respawn
      switch (gMapsSelection) {
        case MAPS_MAP_1: gCurrentMapId = MAP_ID_OVERWORLD_1; break;
        case MAPS_MAP_2: gCurrentMapId = MAP_ID_OVERWORLD_2; break;
        case MAPS_MAP_3: gCurrentMapId = MAP_ID_OVERWORLD_3; break;
        case MAPS_BUILD:
        default:         gCurrentMapId = MAP_ID_BUILD; break;
      }
      resetWorld();
      exitPauseAndConsumeInput(false);
    } else if (gPauseScreen == PAUSE_SCREEN_OPTIONS) {
      // Options screen actions (4 items)
      switch (gOptionsSelection) {
        case OPTIONS_WIFI:
          // Open WEB PORTAL screen
          gPauseScreen = PAUSE_SCREEN_WEB_PORTAL;
          Serial.println("[PAUSE] Switched to WEB PORTAL screen");
          break;

        case OPTIONS_SCREEN:
          // Open BRIGHTNESS screen
          gPauseScreen = PAUSE_SCREEN_BRIGHTNESS;
          gBrightnessSelection = BRIGHTNESS_BAR;  // Start on bar
          // Sync gBrightnessLevel with current matrix brightness (already set)
          Serial.println("[PAUSE] Switched to BRIGHTNESS screen");
          break;

        case OPTIONS_STATS:
          // Stats display - stub for now
          Serial.println("[PAUSE] STATS selected (not implemented)");
          // TODO: Implement stats display
          break;

        case OPTIONS_BACK:
          // Return to main menu (preserve last selection)
          gPauseScreen = PAUSE_SCREEN_MAIN;
          Serial.println("[PAUSE] Returned to MAIN screen");
          break;
      }
    } else if (gPauseScreen == PAUSE_SCREEN_WEB_PORTAL) {
      // Only one option: BACK - return to OPTIONS screen
      gPauseScreen = PAUSE_SCREEN_OPTIONS;
      gOptionsSelection = OPTIONS_WIFI;  // Return to WI-FI option
      Serial.println("[PAUSE] Returned to OPTIONS screen (BACK selected)");
    } else { // PAUSE_SCREEN_BRIGHTNESS
      // Only BACK returns to OPTIONS, BAR does nothing on A
      if (gBrightnessSelection == BRIGHTNESS_BACK) {
        gPauseScreen = PAUSE_SCREEN_OPTIONS;
        gOptionsSelection = OPTIONS_SCREEN;  // Return to SCREEN option
        Serial.println("[PAUSE] Returned to OPTIONS screen (BACK selected from BRIGHTNESS)");
      }
    }
  }
  prevJumpBtn = jumpBtn;

  // ----- 4) B (Spin) = back button -----
  if (spinBtn && !prevSpinBtn) {
    if (gPauseScreen == PAUSE_SCREEN_WEB_PORTAL) {
      // Return from WEB PORTAL to OPTIONS screen, land back on WI-FI
      gPauseScreen = PAUSE_SCREEN_OPTIONS;
      gOptionsSelection = OPTIONS_WIFI;  // Return to WI-FI option
      Serial.println("[PAUSE] Returned to OPTIONS screen (B pressed)");
    } else if (gPauseScreen == PAUSE_SCREEN_BRIGHTNESS) {
      // Return from BRIGHTNESS to OPTIONS screen, land back on SCREEN
      gPauseScreen = PAUSE_SCREEN_OPTIONS;
      gOptionsSelection = OPTIONS_SCREEN;  // Return to SCREEN option
      Serial.println("[PAUSE] Returned to OPTIONS screen (B pressed from BRIGHTNESS)");
    } else if (gPauseScreen == PAUSE_SCREEN_OPTIONS) {
      // Return from OPTIONS to MAIN screen, land back on OPTIONS item
      gPauseScreen = PAUSE_SCREEN_MAIN;
      gPauseSelection = PAUSE_OPTIONS;  // Keep cursor on OPTIONS
      Serial.println("[PAUSE] Returned to MAIN screen (on OPTIONS)");
    } else if (gPauseScreen == PAUSE_SCREEN_MAPS) {
      // Return from MAPS to MAIN screen, land back on MAPS item
      gPauseScreen = PAUSE_SCREEN_MAIN;
      gPauseSelection = PAUSE_MAPS;
      Serial.println("[PAUSE] Returned to MAIN screen (on MAPS)");
    } else {
      // On MAIN screen: back = resume/unpause
      exitPauseAndConsumeInput(false);
    }
  }
  prevSpinBtn = spinBtn;

  // ----- 5) Render a single composite frame -----
  // Draw base scene: world + HUD (frozen frame)
  drawScene(0, 0);

  // Draw pause menu overlay on top
  drawPauseOverlay();

  // Flip buffer once per frame (stable rendering, no flicker)
  matrix->flipDMABuffer();

  // Keep NTP/weather ticking while paused
  tickNetworkAndTime();

  delay(16); // ~60 FPS
}

void loop() {
  // Process WiFi state machine FIRST (non-blocking)
  tickWiFi();
  
  // Service web server (every frame, before any heavy work)
  server.handleClient();
  delay(0);  // Yield to WiFi stack

#if DEBUG_WEB
  // Debug heartbeat (throttled to 5000ms)
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 5000) {
    lastBeat = millis();
    Serial.println("[WEB] loop heartbeat");
  }
#endif

  // Always read controller first
  readInputFromSerial();

  // Recorder chord: must run before pause/reset checks.
  tickDemoRecorderChordAndSuppress(millis());

  // Playback chord: must run before reset/pause checks.
  tickDemoPlaybackChordAndSuppress(millis());

  // Erase stored official demo recordings: must run before reset/pause checks.
  tickDemoEraseChordAndSuppress(millis());

  // Decide who controls this frame (demo vs player)
  const uint32_t nowMs = millis();
  updateControlMode(nowMs);

  // Demo start cue: fade-out/fade-in when entering attract mode.
  tickAttractIntro(nowMs);

  // During the attract intro fade, keep gameplay input neutral.
  if (attractIntroActive()) {
    gInput = {0, 0.0f, 0,0,0,0,0,0,0};
  }

  // If we're in attract mode, ensure we load official recordings per map.
  tickAttractDriver(nowMs);

  // If an official recording is armed, wait until control is stable before consuming frames.
  tickDemoPlaybackArming(nowMs);

  // Playback overrides gameplay input (highest priority)
  applyDemoPlaybackInputOrStop();

  // Demo mode overrides gameplay input only (never drive pause menus)
  if (gGameMode == GAME_MODE_PLAYING && !gDemoPlaybackActive && gControlMode == AUTO_PLAY && !attractIntroActive()) {
    InputState autoInput;
    generateAutoInput(autoInput);
    gInput = autoInput;
  }

  switch (gGameMode) {
    case GAME_MODE_PLAYING: {
      // Start button (btnStart) = Pause toggle (enter pause menu)
      bool pauseBtn = gInput.btnStart;
      if (pauseBtn && !prevResetPressed) {
        gGameMode       = GAME_MODE_PAUSED;
        gPauseScreen    = PAUSE_SCREEN_MAIN; // Always start at main screen
        gPauseSelection = PAUSE_RESET;  // CHANGED: Default selection to RESTART
        gOptionsSelection = OPTIONS_WIFI; // Reset options selection
      }
      prevResetPressed = pauseBtn;

      loopGameplay();
    } break;

    case GAME_MODE_PAUSED:
      loopPauseMenu();
      break;

    default:
      gGameMode = GAME_MODE_PLAYING;
      break;
  }
}
