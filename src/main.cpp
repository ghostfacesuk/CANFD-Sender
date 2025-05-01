#include <Arduino.h>
#include <FlexCAN_T4.h>

//-----------------------------------------------------------------------------
// Global constants & globals
//-----------------------------------------------------------------------------

// CAN bus settings
const int NUM_BAUD_RATES = 5;
const int BAUD_RATES[NUM_BAUD_RATES] = {
  1000000, 500000, 250000, 125000, 100000
};
const int NUM_DATA_RATES = 4;
const int DATA_RATES[NUM_DATA_RATES] = {
  1000000, 2000000, 4000000, 5000000
};

int currentBaudRateIndex = 0;
int currentDataRateIndex = 2;  // 4 Mbps default

const int FD_PAYLOAD_SIZE = 64;
const int STD_PAYLOAD_SIZE = 8;
int currentPayloadSize = FD_PAYLOAD_SIZE;
bool canFDMode = true;

const uint8_t PAYLOAD_DATA = 0xFF;
const int LED_Pin   = 13;
const int logButton= 29;

// Legacy overheads
const int CAN_FD_OVERHEAD  = 29;
const int CAN_STD_OVERHEAD = 44;
const int STUFF_BIT_RATIO  = 5;

// Improved FD/classic timing constants
static const int FD_ARB_BASE_BITS      = 50;
static const int FD_DATA_BASE_BITS     = 533;
static const int CLASSIC_CAN_BASE_BITS = 47;
static const float FD_ARB_STUFF_RATIO   = 1.0f/4;
static const float FD_DATA_STUFF_RATIO  = 1.0f/10;
static const float CLASSIC_STUFF_RATIO  = 1.0f/5;

// Bus-load variables
uint32_t lastBusLoadCalc = 0;
float    currentBusLoad  = 0.0f;

// Frame counting
uint32_t totalFramesSent   = 0;
uint32_t* framesSentByID   = NULL;
uint32_t sessionStartTime  = 0;
bool     countingFrames    = false;

// 3s bus-load monitor
bool     isMonitoringBusLoad           = false;
uint32_t busMonitorStartTime           = 0;
const uint32_t BUS_MONITOR_DURATION    = 3000;
uint32_t framesSentDuringMonitoring    = 0;
uint32_t framesReceivedDuringMonitoring= 0;
uint32_t startingFrameCount            = 0;

// Transmission control
bool sendCAN      = false;
bool useFFPayload = true;

// CAN interface and payload
FlexCAN_T4FD<CAN3, RX_SIZE_1024, TX_SIZE_1024> m_CANInterface;
uint8_t payloadData[FD_PAYLOAD_SIZE] = {0};
uint32_t frameIDs[100];
int frameCount = 1;

// 9.7 ms vs 10 ms toggle
bool use97ms = false;
unsigned long lastTxMicros = 0;

//-----------------------------------------------------------------------------
// Function prototypes
//-----------------------------------------------------------------------------
float estimateFrameTime(bool isFD, int payloadBytes);
float calculateBusLoad();
void clearTerminal();
void updateCANSettings();
void printCurrentRates();
void initFrameCounters();
void printFrameCountSummary();
void toggleCANMode();
void startBusLoadMonitoring();
void measureBusLoad();
void sendCANFrames();
void handleSerialInput(char input);

//-----------------------------------------------------------------------------
// Implementation
//-----------------------------------------------------------------------------

float estimateFrameTime(bool isFD, int payloadBytes) {
  int nominalBaud = BAUD_RATES[currentBaudRateIndex];
  int dataBaud    = DATA_RATES[currentDataRateIndex];

  if (!isFD) {
    int ov = CLASSIC_CAN_BASE_BITS + int(CLASSIC_CAN_BASE_BITS * CLASSIC_STUFF_RATIO);
    int db = payloadBytes * 8;
    db += int(db * CLASSIC_STUFF_RATIO);
    return float(ov + db) / float(nominalBaud);
  } else {
    int ab = FD_ARB_BASE_BITS + int(FD_ARB_BASE_BITS * FD_ARB_STUFF_RATIO);
    int db = payloadBytes * 8 + 21;
    db += int(db * FD_DATA_STUFF_RATIO);
    float tA = float(ab) / float(nominalBaud);
    float tD = float(db) / float(dataBaud);
    return tA + tD;
  }
}

float calculateBusLoad() {
  static uint32_t lastFrameCount = 0, lastCalcTime = 0;
  static float    lastBusLoad   = 0.0f;

  uint32_t now     = millis();
  uint32_t elapsed = now - lastCalcTime;
  if (elapsed < 200) return lastBusLoad;

  uint32_t sent = totalFramesSent - lastFrameCount;
  lastFrameCount = totalFramesSent;
  lastCalcTime   = now;

  if (sent == 0 || !sendCAN) {
    lastBusLoad = sendCAN ? lastBusLoad : 0.0f;
    return lastBusLoad;
  }

  float fps    = float(sent) / (elapsed / 1000.0f);
  float tFrame = estimateFrameTime(canFDMode, currentPayloadSize);
  float load   = fps * tFrame * 100.0f;
  lastBusLoad  = (load > 100.0f) ? 100.0f : load;
  return lastBusLoad;
}

void clearTerminal() {
  Serial.write("\033[2J");
  Serial.write("\033[H");
}

void updateCANSettings() {
  CANFD_timings_t cfg;
  cfg.clock      = CLK_24MHz;
  cfg.baudrate   = BAUD_RATES[currentBaudRateIndex];
  cfg.baudrateFD = DATA_RATES[currentDataRateIndex];
  cfg.propdelay  = 190;
  cfg.bus_length = 1;
  cfg.sample     = 85;

  m_CANInterface.begin();
  m_CANInterface.setBaudRate(cfg);
  m_CANInterface.setRegions(64);
  m_CANInterface.enableMBInterrupts();
}

void printCurrentRates() {
  Serial.print("CAN Mode: ");
  Serial.println(canFDMode ? "FD (64-byte payload)" : "Standard (8-byte payload)");

  Serial.print("Baud Rate: ");
  float br = BAUD_RATES[currentBaudRateIndex] / 1000000.0;
  if (br >= 1.0) {
    Serial.print(br, 3);
    Serial.println(" Mbps");
  } else {
    Serial.print(BAUD_RATES[currentBaudRateIndex] / 1000.0, 1);
    Serial.println(" Kbps");
  }

  if (canFDMode) {
    Serial.print("Data Rate: ");
    Serial.print(DATA_RATES[currentDataRateIndex] / 1000000.0, 1);
    Serial.println(" Mbps");
  }
}

void initFrameCounters() {
  if (framesSentByID) free(framesSentByID);
  framesSentByID = (uint32_t*)malloc(100 * sizeof(uint32_t));
  if (framesSentByID) {
    memset(framesSentByID, 0, 100 * sizeof(uint32_t));
    totalFramesSent = 0;
  }
}

void printFrameCountSummary() {
  uint32_t elapsed = (millis() - sessionStartTime) / 1000;
  Serial.println("\n=== Frame Transmission Summary ===");
  Serial.print("Session duration: ");
  Serial.print(elapsed);
  Serial.println(" seconds");
  Serial.print("Total frames sent: ");
  Serial.println(totalFramesSent);
  Serial.print("Avg fps: ");
  Serial.println(elapsed ? float(totalFramesSent)/elapsed : 0.0f);

  Serial.println("Frames per ID:");
  for (int i = 0; i < frameCount; i++) {
    Serial.print("0x");
    Serial.print(frameIDs[i], HEX);
    Serial.print(": ");
    Serial.println(framesSentByID[i]);
  }
  Serial.println("===================================");
}

void toggleCANMode() {
  canFDMode = !canFDMode;
  currentPayloadSize = canFDMode ? FD_PAYLOAD_SIZE : STD_PAYLOAD_SIZE;
  Serial.print("Switched to ");
  Serial.print(canFDMode ? "CAN FD mode" : "Standard CAN mode");
  Serial.println();
  updateCANSettings();
  printCurrentRates();
}

void startBusLoadMonitoring() {
  if (!isMonitoringBusLoad) {
    isMonitoringBusLoad            = true;
    busMonitorStartTime            = millis();
    framesSentDuringMonitoring     = 0;
    framesReceivedDuringMonitoring = 0;
    if (sendCAN) startingFrameCount = totalFramesSent;
    Serial.println("Starting bus load monitoring for 3 seconds...");
  }
}

void measureBusLoad() {
  if (!isMonitoringBusLoad) return;

  uint32_t now     = millis();
  uint32_t elapsed = now - busMonitorStartTime;

  if (sendCAN)
    framesSentDuringMonitoring = totalFramesSent - startingFrameCount;

  if (elapsed >= BUS_MONITOR_DURATION) {
    isMonitoringBusLoad = false;
    float durS   = elapsed / 1000.0f;
    float ourFps = framesSentDuringMonitoring / durS;
    float othFps = framesReceivedDuringMonitoring / durS;
    float totFps = ourFps + othFps;

    float tFrame  = estimateFrameTime(canFDMode, currentPayloadSize);
    float rawLoad = totFps * tFrame * 100.0f;
    if (rawLoad > 100.0f) rawLoad = 100.0f;

    Serial.println();
    Serial.println("=== Bus Load Results (3s) ===");
    Serial.print("Our fps:           ");    Serial.println(ourFps, 1);
    Serial.print("Other fps:         ");    Serial.println(othFps, 1);
    Serial.print("Total fps:         ");    Serial.println(totFps, 1);
    Serial.print("Est. bus load:     ");    Serial.print(rawLoad, 1); Serial.println("%");
    Serial.println("=============================");

    for (int i = 12; i < 15; i++) {
      m_CANInterface.setMB((FLEXCAN_MAILBOX)i, RX);
      m_CANInterface.setMBFilter((FLEXCAN_MAILBOX)i, ACCEPT_ALL);
    }
  }
}

void sendCANFrames() {
  if (!useFFPayload) {
    for (int i = 0; i < currentPayloadSize; i++) {
      payloadData[i]++;
      if (payloadData[i] > 0xFF) payloadData[i] = 0;
    }
  }
  for (int f = 0; f < frameCount; f++) {
    CANFD_message_t msg;
    msg.len = currentPayloadSize;
    msg.id  = frameIDs[f];
    msg.seq = true;
    msg.esi = false;
    msg.edl = canFDMode;
    msg.brs = canFDMode;
    for (int i = 0; i < currentPayloadSize; i++) {
      msg.buf[i] = useFFPayload ? PAYLOAD_DATA : payloadData[i];
    }
    bool sent = m_CANInterface.write(msg);
    if (sent && countingFrames && framesSentByID) {
      totalFramesSent++;
      framesSentByID[f]++;
    }
  }
}

void handleSerialInput(char input) {
  clearTerminal();
  switch (input) {
    case '9':
      use97ms = !use97ms;
      Serial.print("Transmission interval set to ");
      Serial.print(use97ms ? 9.7f : 10.0f, 1);
      Serial.println(" ms");
      break;

    case 'b': case 'B':
      if (!sendCAN) {
        currentBaudRateIndex = (currentBaudRateIndex + 1) % NUM_BAUD_RATES;
        updateCANSettings();
        printCurrentRates();
      } else {
        Serial.println("Cannot change baud rate while CAN is transmitting!");
      }
      break;

    case 'd': case 'D':
      if (!sendCAN) {
        if (canFDMode) {
          currentDataRateIndex = (currentDataRateIndex + 1) % NUM_DATA_RATES;
          updateCANSettings();
          printCurrentRates();
        } else {
          Serial.println("Data rate setting only available in CAN FD mode!");
        }
      } else {
        Serial.println("Cannot change data rate while CAN is transmitting!");
      }
      break;

    case 'm': case 'M':
      if (!sendCAN) {
        toggleCANMode();
      } else {
        Serial.println("Cannot change CAN mode while transmitting!");
      }
      break;

    case 'l': case 'L':
      startBusLoadMonitoring();
      break;

    case '1':
      useFFPayload = true;
      Serial.println("Switched to FF payload mode.");
      break;

    case '2':
      useFFPayload = false;
      Serial.println("Switched to incrementing payload mode.");
      break;

    case '+': case '=':
      if (frameCount < 100) {
        frameCount++;
        Serial.print("Frame count: ");
        Serial.println(frameCount);
        if (sendCAN) {
          currentBusLoad = calculateBusLoad();
          Serial.print("Bus load: ");
          Serial.print(currentBusLoad, 1);
          Serial.println("% (this device only)");
        }
      }
      break;

    case '-': case '_':
      if (frameCount > 1) {
        frameCount--;
        Serial.print("Frame count: ");
        Serial.println(frameCount);
        if (sendCAN) {
          currentBusLoad = calculateBusLoad();
          Serial.print("Bus load: ");
          Serial.print(currentBusLoad, 1);
          Serial.println("% (this device only)");
        }
      }
      break;

    case 's': case 'S':
      if (countingFrames) {
        Serial.println("\n=== Current Session Statistics ===");
        uint32_t elapsedTime = (millis() - sessionStartTime) / 1000;
        Serial.print("Time elapsed: ");
        Serial.print(elapsedTime);
        Serial.println(" seconds");
        Serial.print("Total frames sent so far: ");
        Serial.println(totalFramesSent);
        Serial.print("Current rate: ");
        Serial.print(elapsedTime > 0 ? (float)totalFramesSent / elapsedTime : 0);
        Serial.println(" frames/second");
        currentBusLoad = calculateBusLoad();
        Serial.print("Bus load: ");
        Serial.print(currentBusLoad, 1);
        Serial.println("% (this device only)");
        Serial.println("=================================");
      } else {
        Serial.println("No active transmission session.");
      }
      break;

    case 'h': case 'H':
      Serial.println("=== CAN Controller ===");
      Serial.println("Commands available:");
      Serial.print("m - Toggle CAN mode (");
      Serial.print(canFDMode ? "FD" : "Standard");
      Serial.println(")");
      Serial.print("b - Cycle baud rate: ");
      Serial.print(BAUD_RATES[currentBaudRateIndex] / 1000000.0, 3);
      Serial.println(" Mbps");
      if (canFDMode) {
        Serial.print("d - Cycle data rate: ");
        Serial.print(DATA_RATES[currentDataRateIndex] / 1000000.0, 1);
        Serial.println(" Mbps");
      }
      Serial.println("1 - FF payload mode");
      Serial.println("2 - Incrementing payload mode");
      Serial.println("+ - Add a frame");
      Serial.println("- - Remove a frame");
      Serial.println("S - Show current statistics (during transmission)");
      Serial.println("L - Measure bus load for 3 seconds");
      Serial.println("9 - Toggle TX interval 10ms <-> 9.7ms");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_Pin, OUTPUT);
  pinMode(logButton, INPUT_PULLUP);

  updateCANSettings();

  // Initialize frame IDs
  frameIDs[0] = 0x555;
  for (int i = 1; i < 100; i++) {
    frameIDs[i] = frameIDs[i - 1] + 1;
  }

  // Configure TX mailboxes 0–11
  for (int i = 0; i < 12; i++) {
    m_CANInterface.setMB((FLEXCAN_MAILBOX)i, TX);
  }
  // Configure RX mailboxes 12–14
  for (int i = 12; i < 15; i++) {
    m_CANInterface.setMB((FLEXCAN_MAILBOX)i, RX);
    m_CANInterface.setMBFilter((FLEXCAN_MAILBOX)i, ACCEPT_ALL);
  }
  m_CANInterface.enableMBInterrupts();

  initFrameCounters();
  printCurrentRates();

  lastTxMicros = micros();
}

void loop() {
  static int lastBtn = HIGH;
  int btn = digitalRead(logButton);

  // update device-only bus load every 250ms
  static uint32_t lastBL = 0;
  if (sendCAN && (millis() - lastBL >= 250)) {
    currentBusLoad = calculateBusLoad();
    lastBL = millis();
  }

  // RX if monitoring
  if (isMonitoringBusLoad) {
    CANFD_message_t rx;
    while (m_CANInterface.read(rx)) {
      bool ours = false;
      for (int i = 0; i < frameCount; i++) {
        if (rx.id == frameIDs[i]) { ours = true; break; }
      }
      if (!ours) framesReceivedDuringMonitoring++;
    }
    measureBusLoad();
  }

  m_CANInterface.events();

  // button toggle
  if (btn == LOW && lastBtn == HIGH) {
    delay(50);
    if (digitalRead(logButton) == LOW) {
      sendCAN = !sendCAN;
      digitalWrite(LED_Pin, sendCAN ? HIGH : LOW);
      if (sendCAN) {
        Serial.println("CAN transmission started.");
        initFrameCounters();
        countingFrames   = true;
        sessionStartTime = millis();
        lastTxMicros     = micros();
      } else {
        Serial.println("CAN transmission stopped.");
        countingFrames = false;
        printFrameCountSummary();
      }
    }
  }
  lastBtn = btn;

  // TX at 10ms or 9.7ms
  if (sendCAN) {
    unsigned long now = micros();
    unsigned long interval = use97ms ? 9700UL : 10000UL;
    if (now - lastTxMicros >= interval) {
      lastTxMicros += interval;
      sendCANFrames();
    }
  }

  // serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '9') {
      use97ms = !use97ms;
      Serial.print("TX interval now ");
      Serial.print(use97ms ? 9.7f : 10.0f, 1);
      Serial.println(" ms");
    } else {
      handleSerialInput(c);
    }
  }
}