#include <Arduino.h>
#include "dw3000.h"
#include <Preferences.h>
#include <math.h>
#include <SPI.h>

// CRITICAL: DW3000 library uses external SPI settings - MUST be configured before spiBegin()
extern SPISettings _fastSPI;

#define APP_NAME "RTLS 4-NODES"

// Makerfabs ESP32 UWB DW3000 pin mapping from official examples.
const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 4;

static dwt_config_t config = {
    5,
    DWT_PLEN_128,
    DWT_PAC8,
    9,
    9,
    1,
    DWT_BR_6M8,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (129 + 8 - 8),
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

extern dwt_txconfig_t txconfig_options;

static constexpr uint16_t PAN_ID = 0xDECA;
static constexpr uint16_t TAG_ADDR = 0xBEEF;
static constexpr uint16_t ANCHOR_ADDR_BASE = 0xA100;

static constexpr int ANCHOR_COUNT = 3;
static constexpr uint8_t FUNC_POLL = 0xE0;
static constexpr uint8_t FUNC_RESP = 0xE1;
static constexpr uint16_t TX_ANT_DLY = 16385;
static constexpr uint16_t RX_ANT_DLY = 16385;
static constexpr uint32_t POLL_TX_TO_RESP_RX_DLY_UUS = 240;
static constexpr uint32_t RESP_RX_TIMEOUT_UUS = 600;
static constexpr uint32_t POLL_RX_TO_RESP_TX_DLY_UUS = 550;
static constexpr uint8_t RANGE_RETRIES = 3;
static constexpr uint32_t RANGE_HOLD_MS = 1500;

static constexpr uint8_t ALL_MSG_SN_IDX = 2;
static constexpr uint8_t MSG_DEST_IDX = 5;
static constexpr uint8_t MSG_SRC_IDX = 7;
static constexpr uint8_t MSG_FUNC_IDX = 9;
static constexpr uint8_t RESP_MSG_POLL_RX_TS_IDX = 10;
static constexpr uint8_t RESP_MSG_RESP_TX_TS_IDX = 14;

struct Point2D {
  float x;
  float y;
};

// Update these coordinates to match your real anchor layout (meters).
static Point2D anchorPos[ANCHOR_COUNT] = {
    {0.0f, 0.0f},
    {4.0f, 0.0f},
    {0.0f, 3.0f},
};

enum NodeRole : uint8_t {
  ROLE_ANCHOR = 0,
  ROLE_TAG = 1,
};

enum OutputMode : uint8_t {
  OUTPUT_TEXT = 0,
  OUTPUT_JSON = 1,
  OUTPUT_PLOT = 2,
};

struct RxMeta {
  bool ok;
  uint32_t statusReg;
  uint32_t frameLen;
  uint16_t srcAddr;
  uint16_t destAddr;
  uint8_t func;
};

Preferences prefs;
NodeRole nodeRole = ROLE_ANCHOR;
uint8_t anchorId = 1;
uint8_t frameSeq = 0;
OutputMode outputMode = OUTPUT_TEXT;
bool rxDebug = false;
float lastGoodRange[ANCHOR_COUNT] = {NAN, NAN, NAN};
uint32_t lastGoodRangeMs[ANCHOR_COUNT] = {0, 0, 0};
uint32_t lastDebugHeartbeatMs = 0;

String serialLine;

static uint16_t anchorAddress(uint8_t id) {
  return (uint16_t)(ANCHOR_ADDR_BASE + id);
}

static void writeU16LE(uint8_t *buf, uint8_t idx, uint16_t val) {
  buf[idx] = (uint8_t)(val & 0xFF);
  buf[idx + 1] = (uint8_t)(val >> 8);
}

static uint16_t readU16LE(const uint8_t *buf, uint8_t idx) {
  return (uint16_t)buf[idx] | ((uint16_t)buf[idx + 1] << 8);
}

static void loadConfig() {
  prefs.begin("rtls", true);
  nodeRole = (NodeRole)prefs.getUChar("role", (uint8_t)ROLE_ANCHOR);
  anchorId = prefs.getUChar("aid", 1);
  if (anchorId < 1 || anchorId > ANCHOR_COUNT) {
    anchorId = 1;
  }
  prefs.end();
}

static void saveConfig() {
  prefs.begin("rtls", false);
  prefs.putUChar("role", (uint8_t)nodeRole);
  prefs.putUChar("aid", anchorId);
  prefs.end();
}

static void printConfig() {
  Serial.println();
  Serial.println("--- NODE CONFIG ---");
  Serial.printf("Role      : %s\n", nodeRole == ROLE_TAG ? "TAG (car)" : "ANCHOR");
  Serial.printf("Anchor ID : %u\n", anchorId);
  Serial.printf("Address   : 0x%04X\n", nodeRole == ROLE_TAG ? TAG_ADDR : anchorAddress(anchorId));
  Serial.printf("Output    : %s\n", outputMode == OUTPUT_JSON ? "json" : (outputMode == OUTPUT_PLOT ? "plot" : "text"));
  Serial.printf("Rx Debug  : %s\n", rxDebug ? "on" : "off");
  Serial.println("Commands  : role tag | role anchor | id 1..3 | output text/json/plot | rxdebug on/off | show | help");
}

static void printHelp() {
  Serial.println("Commands:");
  Serial.println("  role tag");
  Serial.println("  role anchor");
  Serial.println("  id 1");
  Serial.println("  id 2");
  Serial.println("  id 3");
  Serial.println("  output text");
  Serial.println("  output json");
  Serial.println("  output plot");
  Serial.println("  rxdebug on");
  Serial.println("  rxdebug off");
  Serial.println("  show");
}

static void printRxFrameMeta(const char *prefix, const RxMeta &m) {
  Serial.printf("%s status=0x%08lX len=%lu src=0x%04X dst=0x%04X func=0x%02X ok=%d\n",
                prefix,
                (unsigned long)m.statusReg,
                (unsigned long)m.frameLen,
                m.srcAddr,
                m.destAddr,
                m.func,
                m.ok ? 1 : 0);
}

static void printMaybeFloat(float v) {
  if (isnan(v)) {
    Serial.print("null");
  } else {
    Serial.print(v, 3);
  }
}

static void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) {
    return;
  }

  if (cmd == "help") {
    printHelp();
    return;
  }

  if (cmd == "show") {
    printConfig();
    return;
  }

  if (cmd == "role tag") {
    nodeRole = ROLE_TAG;
    saveConfig();
    printConfig();
    return;
  }

  if (cmd == "role anchor") {
    nodeRole = ROLE_ANCHOR;
    saveConfig();
    printConfig();
    return;
  }

  if (cmd == "output text") {
    outputMode = OUTPUT_TEXT;
    printConfig();
    return;
  }

  if (cmd == "output json") {
    outputMode = OUTPUT_JSON;
    printConfig();
    return;
  }

  if (cmd == "output plot") {
    outputMode = OUTPUT_PLOT;
    printConfig();
    return;
  }

  if (cmd == "rxdebug on") {
    rxDebug = true;
    printConfig();
    return;
  }

  if (cmd == "rxdebug off") {
    rxDebug = false;
    printConfig();
    return;
  }

  if (cmd.startsWith("id ")) {
    int id = cmd.substring(3).toInt();
    if (id >= 1 && id <= ANCHOR_COUNT) {
      anchorId = (uint8_t)id;
      saveConfig();
      printConfig();
    } else {
      Serial.println("Invalid id (use 1..3)");
    }
    return;
  }

  Serial.println("Unknown command. Type: help");
}

static void processSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        handleCommand(serialLine);
        serialLine = "";
      }
    } else {
      serialLine += c;
      if (serialLine.length() > 80) {
        serialLine = "";
      }
    }
  }
}

static bool initDw3000() {
  // Initialize SPI at 16 MHz - REQUIRED for DW3000 reliable operation
  _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);
  Serial.println("[DW3000] Initializing SPI at 16 MHz...");
  
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  delay(2);
  
  Serial.println("[DW3000] SPI ready, checking IDLE_RC...");
  uint16_t checkCount = 0;
  while (!dwt_checkidlerc()) {
    Serial.println("[DW3000] IDLE_RC not ready, retrying...");
    delay(10);
    checkCount++;
    if (checkCount > 50) {
      Serial.println("[DW3000] IDLE_RC timeout - SPI or hardware issue!");
      return false;
    }
  }
  Serial.println("[DW3000] IDLE_RC check passed");

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Serial.println("[DW3000] INIT FAILED - check SPI connections!");
    return false;
  }
  Serial.println("[DW3000] Device initialized");

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
  if (dwt_configure(&config)) {
    Serial.println("[DW3000] CONFIG FAILED - invalid chip or calibration issue");
    return false;
  }
  Serial.println("[DW3000] Radio configured (Ch5, 6.8Mbps, PLEN=128)");

  dwt_configuretxrf(&txconfig_options);
  Serial.println("[DW3000] TX RF configured");
  
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  Serial.printf("[DW3000] Antenna delays set: RX=%u TX=%u\n", RX_ANT_DLY, TX_ANT_DLY);
  
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  Serial.println("[DW3000] LNA/PA enabled for RF");
  
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(600);
  Serial.println("[DW3000] Timing: RX after TX delay=240us, timeout=600us");
  
  Serial.println("[DW3000] *** INIT COMPLETE ***\n");
  return true;
}

static void buildPollMsg(uint8_t *msg, uint16_t dest, uint16_t src) {
  msg[0] = 0x41;
  msg[1] = 0x88;
  msg[2] = frameSeq;
  writeU16LE(msg, 3, PAN_ID);
  writeU16LE(msg, MSG_DEST_IDX, dest);
  writeU16LE(msg, MSG_SRC_IDX, src);
  msg[MSG_FUNC_IDX] = FUNC_POLL;
  msg[10] = 0;
  msg[11] = 0;
}

static void buildRespMsg(uint8_t *msg, uint16_t dest, uint16_t src) {
  msg[0] = 0x41;
  msg[1] = 0x88;
  msg[2] = frameSeq;
  writeU16LE(msg, 3, PAN_ID);
  writeU16LE(msg, MSG_DEST_IDX, dest);
  writeU16LE(msg, MSG_SRC_IDX, src);
  msg[MSG_FUNC_IDX] = FUNC_RESP;
}

static bool rangeToAnchor(uint8_t id, float *distanceOut, RxMeta *metaOut) {
  uint16_t destAddr = anchorAddress(id);
  uint8_t txPollMsg[12];
  uint8_t expectedResp[20];
  uint8_t rxBuffer[20];
  uint32_t statusReg;

  RxMeta localMeta = {false, 0, 0, 0, 0, 0};

  buildPollMsg(txPollMsg, destAddr, TAG_ADDR);
  buildRespMsg(expectedResp, TAG_ADDR, destAddr);

  if (rxDebug) {
    Serial.printf("TAG SEND POLL to A%u (0x%04X)\n", id, destAddr);
  }

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof(txPollMsg), txPollMsg, 0);
  dwt_writetxfctrl(sizeof(txPollMsg), 0, 1);
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  while (!((statusReg = dwt_read32bitreg(SYS_STATUS_ID)) &
           (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
  }

  localMeta.statusReg = statusReg;

  frameSeq++;

  if (!(statusReg & SYS_STATUS_RXFCG_BIT_MASK)) {
    if (rxDebug && (statusReg & SYS_STATUS_ALL_RX_TO)) {
      Serial.printf("TAG A%u TIMEOUT\n", id);
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    if (metaOut != nullptr) {
      *metaOut = localMeta;
    }
    return false;
  }

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  uint32_t frameLen = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  localMeta.frameLen = frameLen;
  if (frameLen > sizeof(rxBuffer)) {
    if (metaOut != nullptr) {
      *metaOut = localMeta;
    }
    return false;
  }

  dwt_readrxdata(rxBuffer, frameLen, 0);
  localMeta.destAddr = readU16LE(rxBuffer, MSG_DEST_IDX);
  localMeta.srcAddr = readU16LE(rxBuffer, MSG_SRC_IDX);
  localMeta.func = rxBuffer[MSG_FUNC_IDX];
  rxBuffer[ALL_MSG_SN_IDX] = 0;
  expectedResp[ALL_MSG_SN_IDX] = 0;

  if (memcmp(rxBuffer, expectedResp, 10) != 0) {
    if (metaOut != nullptr) {
      *metaOut = localMeta;
    }
    return false;
  }

  uint32_t pollTxTs = dwt_readtxtimestamplo32();
  uint32_t respRxTs = dwt_readrxtimestamplo32();
  uint32_t pollRxTs;
  uint32_t respTxTs;
  int32_t rtdInit;
  int32_t rtdResp;
  float clockOffsetRatio;

  clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
  resp_msg_get_ts(&rxBuffer[RESP_MSG_POLL_RX_TS_IDX], &pollRxTs);
  resp_msg_get_ts(&rxBuffer[RESP_MSG_RESP_TX_TS_IDX], &respTxTs);

  rtdInit = (int32_t)(respRxTs - pollTxTs);
  rtdResp = (int32_t)(respTxTs - pollRxTs);

  double tof = ((rtdInit - rtdResp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
  *distanceOut = (float)(tof * SPEED_OF_LIGHT);
  localMeta.ok = true;
  if (metaOut != nullptr) {
    *metaOut = localMeta;
  }
  return true;
}

static void runAnchorOnce() {
  uint16_t myAddr = anchorAddress(anchorId);
  uint8_t expectedPoll[12];
  uint8_t rxBuffer[20];
  uint32_t statusReg;

  buildPollMsg(expectedPoll, myAddr, TAG_ADDR);

  dwt_setrxtimeout(10000); // ~10 ms: lets loop breathe and print debug heartbeat.
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  while (!((statusReg = dwt_read32bitreg(SYS_STATUS_ID)) &
           (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
    processSerial();
  }

  // Restore default ranging timeout used by tag ranging exchanges.
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  if (statusReg & SYS_STATUS_ALL_RX_TO) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO);
    if (rxDebug) {
      uint32_t now = millis();
      if (now - lastDebugHeartbeatMs > 1000) {
        Serial.printf("ANCHOR %u waiting poll...\n", anchorId);
        lastDebugHeartbeatMs = now;
      }
    }
    return;
  }

  if (!(statusReg & SYS_STATUS_RXFCG_BIT_MASK)) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    return;
  }

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  uint32_t frameLen = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (frameLen > sizeof(rxBuffer)) {
    return;
  }

  dwt_readrxdata(rxBuffer, frameLen, 0);
  uint16_t rxSrcAddr = readU16LE(rxBuffer, MSG_SRC_IDX);
  uint16_t rxDstAddr = readU16LE(rxBuffer, MSG_DEST_IDX);
  if (rxDebug) {
    if (rxDstAddr == myAddr) {
      Serial.printf("ANCHOR %u RCV FOR ME src=0x%04X dst=0x%04X len=%lu\n", anchorId, rxSrcAddr, rxDstAddr, (unsigned long)frameLen);
    } else {
      Serial.printf("ANCHOR %u RCV IGNORED src=0x%04X dst=0x%04X\n", anchorId, rxSrcAddr, rxDstAddr);
    }
  }
  rxBuffer[ALL_MSG_SN_IDX] = 0;
  expectedPoll[ALL_MSG_SN_IDX] = 0;

  if (memcmp(rxBuffer, expectedPoll, 10) != 0) {
    return;
  }

  uint16_t sourceAddr = readU16LE(rxBuffer, MSG_SRC_IDX);
  uint64_t pollRxTs = get_rx_timestamp_u64();
  uint32_t respTxTime = (uint32_t)((pollRxTs + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
  dwt_setdelayedtrxtime(respTxTime);
  uint64_t respTxTs = (((uint64_t)(respTxTime & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

  uint8_t txRespMsg[20];
  buildRespMsg(txRespMsg, sourceAddr, myAddr);
  resp_msg_set_ts(&txRespMsg[RESP_MSG_POLL_RX_TS_IDX], pollRxTs);
  resp_msg_set_ts(&txRespMsg[RESP_MSG_RESP_TX_TS_IDX], respTxTs);

  dwt_writetxdata(sizeof(txRespMsg), txRespMsg, 0);
  dwt_writetxfctrl(sizeof(txRespMsg), 0, 1);
  int ret = dwt_starttx(DWT_START_TX_DELAYED);
  if (ret == DWT_SUCCESS) {
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    frameSeq++;
  }
}

static bool trilaterate2D(const Point2D &p1, const Point2D &p2, const Point2D &p3,
                          float r1, float r2, float r3, Point2D *out) {
  float A = 2.0f * (p2.x - p1.x);
  float B = 2.0f * (p2.y - p1.y);
  float C = (r1 * r1 - r2 * r2 - p1.x * p1.x + p2.x * p2.x - p1.y * p1.y + p2.y * p2.y);

  float D = 2.0f * (p3.x - p1.x);
  float E = 2.0f * (p3.y - p1.y);
  float F = (r1 * r1 - r3 * r3 - p1.x * p1.x + p3.x * p3.x - p1.y * p1.y + p3.y * p3.y);

  float det = A * E - B * D;
  if (fabsf(det) < 1e-6f) {
    return false;
  }

  out->x = (C * E - B * F) / det;
  out->y = (A * F - C * D) / det;
  return true;
}

void setup() {
  UART_init();
  delay(100);
  Serial.println("\n\n=== " APP_NAME " ===");
  Serial.println("[BOOT] Initializing...");
  
  loadConfig();
  Serial.println("[BOOT] Configuration loaded from NVS");

  Serial.println("[BOOT] Reading device ID before init...");
  uint32_t devId = dwt_readdevid();
  Serial.printf("[BOOT] DEVICE ID: 0x%08lx (should be 0xDECA0302)\n", (unsigned long)devId);
  
  if ((devId & 0xFFFF) != 0xDECA) {
    Serial.println("[ERROR] Device ID mismatch! SPI/hardware not responding correctly.");
    Serial.println("[ERROR] Check: RST=27, IRQ=34, SS=4 pins and SPI wiring.");
    while (1) delay(1000);
  }

  Serial.println("\n[BOOT] Calling initDw3000()...");
  if (!initDw3000()) {
    Serial.println("[ERROR] DW3000 initialization failed - check hardware/connections!");
    while (1) {
      delay(1000);
    }
  }

  printConfig();
  Serial.println("[BOOT] Ready to start ranging!\n");
}

void loop() {
  processSerial();

  if (nodeRole == ROLE_ANCHOR) {
    runAnchorOnce();
    return;
  }

  float d[ANCHOR_COUNT] = {NAN, NAN, NAN};
  for (uint8_t id = 1; id <= ANCHOR_COUNT; id++) {
    float dist = NAN;
    RxMeta m = {false, 0, 0, 0, 0, 0};
    bool ok = false;

    for (uint8_t attempt = 0; attempt < RANGE_RETRIES; attempt++) {
      if (rangeToAnchor(id, &dist, &m)) {
        ok = true;
        break;
      }
      delay(8);
    }

    if (ok) {
      d[id - 1] = dist;
      lastGoodRange[id - 1] = dist;
      lastGoodRangeMs[id - 1] = millis();
    } else {
      uint32_t now = millis();
      if (!isnan(lastGoodRange[id - 1]) && (now - lastGoodRangeMs[id - 1] <= RANGE_HOLD_MS)) {
        d[id - 1] = lastGoodRange[id - 1];
      }
    }

    if (rxDebug) {
      Serial.printf("TAG->A%u ", id);
      printRxFrameMeta("RX", m);
      if (!ok && !isnan(d[id - 1])) {
        Serial.printf("TAG->A%u using held range %.2f m\n", id, d[id - 1]);
    }
    delay(30);
  }

  bool posOk = false;
  Point2D pos = {NAN, NAN};
  if (!isnan(d[0]) && !isnan(d[1]) && !isnan(d[2])) {
    posOk = trilaterate2D(anchorPos[0], anchorPos[1], anchorPos[2], d[0], d[1], d[2], &pos);
  }

  if (outputMode == OUTPUT_JSON) {
    Serial.print("{\"type\":\"tag\",\"ranges\":{");
    Serial.print("\"a1\":");
    printMaybeFloat(d[0]);
    Serial.print(",\"a2\":");
    printMaybeFloat(d[1]);
    Serial.print(",\"a3\":");
    printMaybeFloat(d[2]);
    Serial.print("},\"pos\":{");
    Serial.print("\"x\":");
    if (posOk) {
      printMaybeFloat(pos.x);
    } else {
      Serial.print("null");
    }
    Serial.print(",\"y\":");
    if (posOk) {
      printMaybeFloat(pos.y);
    } else {
      Serial.print("null");
    }
    Serial.println("}}");
  } else if (outputMode == OUTPUT_PLOT) {
    Serial.print("PLOT,");
    if (posOk) {
      Serial.print(pos.x, 3);
    } else {
      Serial.print("nan");
    }
    Serial.print(",");
    if (posOk) {
      Serial.print(pos.y, 3);
    } else {
      Serial.print("nan");
    }
    Serial.print(",");
    if (isnan(d[0])) {
      Serial.print("nan");
    } else {
      Serial.print(d[0], 3);
    }
    Serial.print(",");
    if (isnan(d[1])) {
      Serial.print("nan");
    } else {
      Serial.print(d[1], 3);
    }
    Serial.print(",");
    if (isnan(d[2])) {
      Serial.println("nan");
    } else {
      Serial.println(d[2], 3);
    }
  } else {
    Serial.printf("RANGES m: A1=%.2f A2=%.2f A3=%.2f\n", d[0], d[1], d[2]);
    if (posOk) {
      Serial.printf("TAG POS m: X=%.2f Y=%.2f\n", pos.x, pos.y);
    } else if (isnan(d[0]) || isnan(d[1]) || isnan(d[2])) {
      Serial.println("WAITING: missing range from one or more anchors");
    } else {
      Serial.println("TRILATERATION FAILED (anchors collinear?)");
    }
  }

  delay(120);
}