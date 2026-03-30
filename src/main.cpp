#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SD.h>

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

// TX frequency options
const int NUM_TX_RATES = 4;
const unsigned long TX_INTERVALS_US[NUM_TX_RATES] = {
  1000000UL,  // 1 Hz
  100000UL,   // 10 Hz
  20000UL,    // 50 Hz
  10000UL     // 100 Hz
};
const char* TX_RATE_LABELS[NUM_TX_RATES] = {
  "1 Hz", "10 Hz", "50 Hz", "100 Hz"
};
int currentTxRateIndex = 3;  // default 100 Hz
unsigned long lastTxMicros = 0;

// ASC Replay constants
const int ASC_MAX_FILES = 10;
const int ASC_MAX_FILENAME = 64;
const int ASC_LINE_BUF_SIZE = 512;
const int ASC_MAX_FRAMES_PER_TICK = 50;

// ASC Replay state
bool ascMode = false;
bool ascPlaying = false;
bool ascFileOpen = false;
bool ascFileHasFD = false;
bool sdCardPresent = false;
File ascFile;
char ascFileList[ASC_MAX_FILES][ASC_MAX_FILENAME];
uint32_t ascFileSizes[ASC_MAX_FILES];
int ascFileCount = 0;
int ascSelectedFile = -1;

// ASC Replay timing
unsigned long ascStartMicros = 0;
double ascFirstTimestamp = -1.0;
uint32_t ascFramesSent = 0;
uint32_t ascLoopCount = 0;
uint32_t ascReplayStartTime = 0;

// ASC Pending frame
struct ASCFrame {
  double timestamp;
  uint32_t id;
  uint8_t data[64];
  uint8_t len;
  bool isFD;
};
bool ascFrameReady = false;
ASCFrame ascPendingFrame;
char ascLineBuf[ASC_LINE_BUF_SIZE];

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

// ASC Replay prototypes
bool scanFileForFD(const char* filename);
void listASCFiles();
void printASCMenu();
void printASCReplayStatus();
void enterASCMode();
void exitASCMode();
void selectASCFile(int index);
void startASCReplay();
void stopASCReplay();
bool readLineFromFile();
bool parseASCLine(ASCFrame& frame);
void sendASCFrame(const ASCFrame& frame);
bool readNextASCFrame();
void loopASCFile();
void processASCReplay();
void handleASCSerialInput(char input);

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

//-----------------------------------------------------------------------------
// ASC Replay Implementation
//-----------------------------------------------------------------------------

bool scanFileForFD(const char* filename) {
  File f = SD.open(filename);
  if (!f) return false;
  int state = 0;
  while (f.available()) {
    char c = f.read();
    switch (state) {
      case 0: state = (c == 'C') ? 1 : 0; break;
      case 1: state = (c == 'A') ? 2 : (c == 'C') ? 1 : 0; break;
      case 2: state = (c == 'N') ? 3 : (c == 'C') ? 1 : 0; break;
      case 3: state = (c == 'F') ? 4 : (c == 'C') ? 1 : 0; break;
      case 4:
        if (c == 'D') { f.close(); return true; }
        state = (c == 'C') ? 1 : 0;
        break;
    }
  }
  f.close();
  return false;
}

void listASCFiles() {
  ascFileCount = 0;
  File root = SD.open("/");
  if (!root) return;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    if (entry.isDirectory()) { entry.close(); continue; }
    const char* name = entry.name();
    int len = strlen(name);
    if (len >= 4 && strcasecmp(name + len - 4, ".asc") == 0) {
      if (ascFileCount < ASC_MAX_FILES) {
        strncpy(ascFileList[ascFileCount], name, ASC_MAX_FILENAME - 1);
        ascFileList[ascFileCount][ASC_MAX_FILENAME - 1] = '\0';
        ascFileSizes[ascFileCount] = entry.size();
        ascFileCount++;
      }
    }
    entry.close();
  }
  root.close();
}

void printASCMenu() {
  clearTerminal();
  Serial.println("=== ASC Replay Mode ===");
  Serial.print("CAN Mode: ");
  Serial.println(canFDMode ? "FD" : "Standard");
  Serial.println();

  if (ascFileCount == 0) {
    Serial.println("No .asc files found on SD card.");
    Serial.println("Copy .asc files to the root of the SD card.");
  } else {
    Serial.println("Files on SD card:");
    for (int i = 0; i < ascFileCount; i++) {
      Serial.print("  ");
      Serial.print(i + 1);
      Serial.print(" - ");
      Serial.print(ascFileList[i]);
      Serial.print("  (");
      if (ascFileSizes[i] >= 1048576) {
        Serial.print(ascFileSizes[i] / 1048576.0, 1);
        Serial.print(" MB");
      } else {
        Serial.print(ascFileSizes[i] / 1024.0, 1);
        Serial.print(" KB");
      }
      Serial.println(")");
    }
  }
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  1-9  Select file");
  Serial.println("  x    Exit ASC mode");
}

void printASCReplayStatus() {
  clearTerminal();
  Serial.println("=== ASC Replay ===");
  Serial.print("File: ");
  Serial.println(ascFileList[ascSelectedFile]);
  Serial.print("Status: ");
  Serial.println(ascPlaying ? "PLAYING" : "PAUSED");
  Serial.print("Frames sent: ");
  Serial.println(ascFramesSent);
  Serial.print("Loops completed: ");
  Serial.println(ascLoopCount);
  if (ascPlaying) {
    uint32_t elapsed = (millis() - ascReplayStartTime) / 1000;
    Serial.print("Elapsed: ");
    Serial.print(elapsed);
    Serial.println("s");
  }
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  p  Play / Pause");
  Serial.println("  s  Refresh status");
  Serial.println("  x  Stop and exit ASC mode");
}

void enterASCMode() {
  if (sendCAN) {
    Serial.println("Stop CAN transmission before entering ASC mode!");
    return;
  }

  // Try to (re)initialise SD card
  sdCardPresent = SD.begin(BUILTIN_SDCARD);
  if (!sdCardPresent) {
    Serial.println("No SD card detected! Insert an SD card and try again.");
    return;
  }

  ascMode = true;
  ascPlaying = false;
  ascSelectedFile = -1;
  listASCFiles();
  printASCMenu();
}

void exitASCMode() {
  if (ascFileOpen) {
    ascFile.close();
    ascFileOpen = false;
  }
  ascMode = false;
  ascPlaying = false;
  ascFrameReady = false;
  ascSelectedFile = -1;
  digitalWrite(LED_Pin, LOW);
  clearTerminal();
  Serial.println("Exited ASC mode.");
  printCurrentRates();
}

void selectASCFile(int index) {
  if (index < 0 || index >= ascFileCount) {
    Serial.println("Invalid file number!");
    return;
  }

  Serial.print("Scanning ");
  Serial.print(ascFileList[index]);
  Serial.println("...");

  bool hasFD = scanFileForFD(ascFileList[index]);
  if (hasFD && !canFDMode) {
    clearTerminal();
    Serial.println("This file contains CAN FD frames but you are");
    Serial.println("in Standard CAN mode.");
    Serial.println();
    Serial.println("Exit ASC mode (x) and switch to CAN FD mode (m)");
    Serial.println("before playing this file.");
    Serial.println();
    Serial.println("Press x to go back.");
    return;
  }

  ascSelectedFile = index;
  ascFileHasFD = hasFD;

  if (ascFileOpen) {
    ascFile.close();
    ascFileOpen = false;
  }
  ascFile = SD.open(ascFileList[index]);
  if (!ascFile) {
    Serial.println("Failed to open file!");
    ascSelectedFile = -1;
    return;
  }
  ascFileOpen = true;
  ascFramesSent = 0;
  ascLoopCount = 0;
  ascFirstTimestamp = -1.0;
  ascFrameReady = false;
  ascPlaying = false;

  printASCReplayStatus();
}

void startASCReplay() {
  if (!ascFileOpen) return;

  if (ascFirstTimestamp >= 0 && ascFrameReady) {
    // Resuming - adjust start time so pending frame plays immediately
    double targetUs = (ascPendingFrame.timestamp - ascFirstTimestamp) * 1000000.0;
    ascStartMicros = micros() - (unsigned long)targetUs;
  } else {
    ascStartMicros = micros();
  }

  ascPlaying = true;
  if (ascReplayStartTime == 0) ascReplayStartTime = millis();
  digitalWrite(LED_Pin, HIGH);
  printASCReplayStatus();
}

void stopASCReplay() {
  ascPlaying = false;
  digitalWrite(LED_Pin, LOW);
  printASCReplayStatus();
}

bool readLineFromFile() {
  if (!ascFile.available()) return false;
  int i = 0;
  while (ascFile.available() && i < ASC_LINE_BUF_SIZE - 1) {
    char c = ascFile.read();
    if (c == '\n') break;
    if (c != '\r') ascLineBuf[i++] = c;
  }
  ascLineBuf[i] = '\0';
  return true;
}

// Skip whitespace, return pointer to next non-whitespace char
static char* ascSkipWS(char* p) {
  while (*p == ' ' || *p == '\t') p++;
  return p;
}

// Read a whitespace-delimited token, return pointer past it
static char* ascReadToken(char* p, char* token, int maxLen) {
  p = ascSkipWS(p);
  int i = 0;
  while (*p && *p != ' ' && *p != '\t' && i < maxLen - 1) {
    token[i++] = *p++;
  }
  token[i] = '\0';
  return p;
}

bool parseASCLine(ASCFrame& frame) {
  char* p = ascLineBuf;
  char token[32];

  p = ascSkipWS(p);
  if (*p == '\0' || *p == ';' || *p == '/') return false;

  // Parse timestamp
  char* end;
  double ts = strtod(p, &end);
  if (end == p) return false;
  frame.timestamp = ts;
  p = end;

  // Next token determines frame type
  p = ascReadToken(p, token, sizeof(token));

  if (strcmp(token, "CANFD") == 0) {
    // CAN FD frame: CANFD channel dir id flags1 flags2 dlc dataLen data...
    frame.isFD = true;

    // Channel
    p = ascReadToken(p, token, sizeof(token));
    // Direction (Rx/Tx)
    p = ascReadToken(p, token, sizeof(token));
    if (strcmp(token, "Rx") != 0 && strcmp(token, "Tx") != 0) return false;
    // CAN ID (hex)
    p = ascReadToken(p, token, sizeof(token));
    frame.id = strtoul(token, NULL, 16);
    // Flag1
    p = ascReadToken(p, token, sizeof(token));
    // Flag2
    p = ascReadToken(p, token, sizeof(token));
    // DLC (hex)
    p = ascReadToken(p, token, sizeof(token));
    // Data length (decimal)
    p = ascReadToken(p, token, sizeof(token));
    frame.len = atoi(token);
    if (frame.len > 64) frame.len = 64;

    // Data bytes (hex)
    for (int i = 0; i < frame.len; i++) {
      p = ascReadToken(p, token, sizeof(token));
      if (token[0] == '\0') { frame.len = i; break; }
      frame.data[i] = (uint8_t)strtoul(token, NULL, 16);
    }
    return true;

  } else {
    // Possibly a standard CAN data frame: channel id dir d dlc data...
    // token is the channel number
    char* endp;
    long channel = strtol(token, &endp, 10);
    if (*endp != '\0' || channel < 1) return false;

    // CAN ID (hex)
    p = ascReadToken(p, token, sizeof(token));
    frame.id = strtoul(token, NULL, 16);

    // Direction
    p = ascReadToken(p, token, sizeof(token));
    if (strcmp(token, "Rx") != 0 && strcmp(token, "Tx") != 0) return false;

    // "d" for data frame
    p = ascReadToken(p, token, sizeof(token));
    if (strcmp(token, "d") != 0) return false;

    // DLC
    p = ascReadToken(p, token, sizeof(token));
    frame.len = atoi(token);
    if (frame.len > 8) frame.len = 8;
    frame.isFD = false;

    // Data bytes
    for (int i = 0; i < frame.len; i++) {
      p = ascReadToken(p, token, sizeof(token));
      if (token[0] == '\0') { frame.len = i; break; }
      frame.data[i] = (uint8_t)strtoul(token, NULL, 16);
    }
    return true;
  }
}

void sendASCFrame(const ASCFrame& frame) {
  CANFD_message_t msg = {};
  msg.id  = frame.id;
  msg.len = frame.len;
  msg.seq = true;
  msg.esi = false;
  msg.edl = frame.isFD;
  msg.brs = frame.isFD;
  for (int i = 0; i < frame.len; i++) {
    msg.buf[i] = frame.data[i];
  }
  m_CANInterface.write(msg);
}

bool readNextASCFrame() {
  while (readLineFromFile()) {
    ASCFrame frame;
    if (parseASCLine(frame)) {
      // Skip FD frames if in standard mode (safety check)
      if (frame.isFD && !canFDMode) continue;

      ascPendingFrame = frame;
      if (ascFirstTimestamp < 0) {
        ascFirstTimestamp = frame.timestamp;
      }
      return true;
    }
  }
  return false;
}

void loopASCFile() {
  ascFile.seek(0);
  ascFirstTimestamp = -1.0;
  ascStartMicros = micros();
  ascFrameReady = false;
  ascLoopCount++;
}

void processASCReplay() {
  if (!ascMode || !ascPlaying || !ascFileOpen) return;

  unsigned long now = micros();
  int sent = 0;

  while (sent < ASC_MAX_FRAMES_PER_TICK) {
    if (!ascFrameReady) {
      if (!readNextASCFrame()) {
        // End of file - loop back
        loopASCFile();
        if (!readNextASCFrame()) {
          Serial.println("\nASC file contains no playable frames!");
          stopASCReplay();
          return;
        }
      }
      ascFrameReady = true;
    }

    double targetUs = (ascPendingFrame.timestamp - ascFirstTimestamp) * 1000000.0;
    unsigned long elapsed = now - ascStartMicros;

    if ((double)elapsed < targetUs) break;

    sendASCFrame(ascPendingFrame);
    ascFramesSent++;
    ascFrameReady = false;
    sent++;
  }
}

void handleASCSerialInput(char input) {
  if (ascSelectedFile >= 0) {
    // In replay view
    switch (input) {
      case 'p': case 'P':
        if (ascPlaying) {
          stopASCReplay();
        } else {
          startASCReplay();
        }
        break;
      case 's': case 'S':
        printASCReplayStatus();
        break;
      case 'x': case 'X':
        exitASCMode();
        break;
    }
  } else {
    // In file selection view
    if (input >= '1' && input <= '9') {
      selectASCFile(input - '1');
    } else if (input == 'x' || input == 'X') {
      exitASCMode();
    }
  }
}

//-----------------------------------------------------------------------------
// Serial command handler
//-----------------------------------------------------------------------------

void handleSerialInput(char input) {
  clearTerminal();
  switch (input) {
    case 'f': case 'F':
      currentTxRateIndex = (currentTxRateIndex + 1) % NUM_TX_RATES;
      Serial.print("TX rate: ");
      Serial.println(TX_RATE_LABELS[currentTxRateIndex]);
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

    case 'p': case 'P':
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

    case 'a': case 'A':
      enterASCMode();
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
      Serial.println("P - Toggle CAN transmission start/stop");
      Serial.println("S - Show current statistics (during transmission)");
      Serial.println("L - Measure bus load for 3 seconds");
      Serial.print("f - Cycle TX rate: ");
      Serial.println(TX_RATE_LABELS[currentTxRateIndex]);
      Serial.println("A - Enter ASC replay mode (SD card)");
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

  // Configure TX mailboxes 0-11
  for (int i = 0; i < 12; i++) {
    m_CANInterface.setMB((FLEXCAN_MAILBOX)i, TX);
  }
  // Configure RX mailboxes 12-14
  for (int i = 12; i < 15; i++) {
    m_CANInterface.setMB((FLEXCAN_MAILBOX)i, RX);
    m_CANInterface.setMBFilter((FLEXCAN_MAILBOX)i, ACCEPT_ALL);
  }
  m_CANInterface.enableMBInterrupts();

  initFrameCounters();

  // Initialise SD card
  sdCardPresent = SD.begin(BUILTIN_SDCARD);
  if (sdCardPresent) {
    Serial.println("SD card detected.");
  } else {
    Serial.println("No SD card detected.");
  }

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
      if (ascMode && ascFileOpen) {
        // Toggle ASC playback
        if (ascPlaying) stopASCReplay();
        else startASCReplay();
      } else if (!ascMode) {
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
  }
  lastBtn = btn;

  // TX at selected frequency (normal mode only)
  if (sendCAN && !ascMode) {
    unsigned long now = micros();
    unsigned long interval = TX_INTERVALS_US[currentTxRateIndex];
    if (now - lastTxMicros >= interval) {
      lastTxMicros += interval;
      sendCANFrames();
    }
  }

  // ASC replay processing
  processASCReplay();

  // serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (ascMode) {
      handleASCSerialInput(c);
    } else {
      handleSerialInput(c);
    }
  }
}
