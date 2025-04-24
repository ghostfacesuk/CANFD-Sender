#include <Arduino.h>
#include <FlexCAN_T4.h>

// Define CAN bus settings
const int NUM_BAUD_RATES = 5;
const int BAUD_RATES[NUM_BAUD_RATES] = {1000000, 500000, 250000, 125000, 100000};  // 1 Mbps, 500 Kbps, 250 Kbps, 125 Kbps, 100 Kbps
const int NUM_DATA_RATES = 4;
const int DATA_RATES[NUM_DATA_RATES] = {1000000, 2000000, 4000000, 5000000};  // 1, 2, 4, 5 Mbps
int currentBaudRateIndex = 0;
int currentDataRateIndex = 2;  // Start with 4 Mbps as default

const int FD_PAYLOAD_SIZE = 64;  // Max payload for CAN FD
const int STD_PAYLOAD_SIZE = 8;  // Max payload for Standard CAN
int currentPayloadSize = FD_PAYLOAD_SIZE;  // Default to FD payload size
bool canFDMode = true;  // Default to CAN FD mode

const uint8_t PAYLOAD_DATA = 0xFF;
const int LED_Pin = 13;  // LED pin
const int logButton = 29;  // Button pin

// Variables for bus load calculation
const int CAN_FD_OVERHEAD = 29;  // CAN FD overhead bits (SOF, ID, Control, CRC, etc.)
const int CAN_STD_OVERHEAD = 44;  // Standard CAN overhead bits
const int STUFF_BIT_RATIO = 5;   // Estimate: add 1 stuff bit every 5 bits

// -----------------------------------------------------------------------------
// Improved FD and Classic CAN bit‐timing constants
static const int FD_ARB_BASE_BITS      = 50;   // SOF, ID, DLC, flags, ACK, EOF, IFS @ nominal speed
static const int FD_DATA_BASE_BITS     = 533;  // 64 bytes (512 bits) + 21‐bit CRC @ data speed
static const int CLASSIC_CAN_BASE_BITS = 47;   // ~47 bits typical for 11‐bit ID, 8‐byte data

// Rough stuff‐bit ratios
static const float FD_ARB_STUFF_RATIO   = 1.0f/4;   // ~1 stuff bit every 4 bits
static const float FD_DATA_STUFF_RATIO  = 1.0f/10;  // ~1 stuff bit every 10 bits
static const float CLASSIC_STUFF_RATIO  = 1.0f/5;   // typical for Classic CAN

// Estimate time (in seconds) to send one frame
float estimateFrameTime(bool isFD, int payloadBytes) {
  int nominalBaud = BAUD_RATES[currentBaudRateIndex];
  int dataBaud    = DATA_RATES[currentDataRateIndex];

  if (!isFD) {
    // Classic CAN
    int overheadBits = CLASSIC_CAN_BASE_BITS
                     + int(CLASSIC_CAN_BASE_BITS * CLASSIC_STUFF_RATIO);
    int dataBits     = payloadBytes * 8;
    dataBits        += int(dataBits * CLASSIC_STUFF_RATIO);
    return float(overheadBits + dataBits) / float(nominalBaud);
  } else {
    // CAN FD
    int arbBits  = FD_ARB_BASE_BITS
                 + int(FD_ARB_BASE_BITS * FD_ARB_STUFF_RATIO);
    int dataBits = payloadBytes * 8 + 21;  // assume 21‐bit CRC for >16 B
    dataBits    += int(dataBits * FD_DATA_STUFF_RATIO);

    float arbTime  = float(arbBits)  / float(nominalBaud);
    float dataTime = float(dataBits) / float(dataBaud);
    return arbTime + dataTime;
  }
}
// -----------------------------------------------------------------------------

uint32_t lastBusLoadCalc = 0;
float currentBusLoad = 0.0;

// Frame counting variables
uint32_t totalFramesSent = 0;  // Total frames sent in current session
uint32_t* framesSentByID = NULL;  // Array to track frames sent per ID
uint32_t sessionStartTime = 0;  // When the current session started
bool countingFrames = false;  // Whether we're currently counting frames

// Bus load monitoring variables
bool isMonitoringBusLoad = false;
uint32_t busMonitorStartTime = 0;
const uint32_t BUS_MONITOR_DURATION = 3000; // 3 seconds
uint32_t framesSentDuringMonitoring = 0;
uint32_t framesReceivedDuringMonitoring = 0;
uint32_t lastReceivedCount = 0;
uint32_t startingFrameCount = 0;  // Track starting frame count for differential measurement

int sendCount = 0; // sending serial message 
bool sendCAN = false; // Control flag for CAN sending
bool useFFPayload = true; // Control flag for payload mode

// Create CAN object
FlexCAN_T4FD<CAN3, RX_SIZE_1024, TX_SIZE_1024> m_CANInterface;

// Payload data array
uint8_t payloadData[FD_PAYLOAD_SIZE] = {0};

// CAN frame IDs array and current frame count
uint32_t frameIDs[100];  // Increased from 51 to 100 max frames
int frameCount = 1;  // Start with one frame

// Function prototypes
void handleSerialInput(char input);
void sendCANFrames();
void updateCANSettings();
void printCurrentRates();
float calculateBusLoad();
void clearTerminal();
void toggleCANMode();
void initFrameCounters();
void printFrameCountSummary();
void startBusLoadMonitoring();
void measureBusLoad();

void clearTerminal() {
  // ANSI escape codes to clear screen and move cursor to top
  Serial.write("\033[2J");    // Clear screen
  Serial.write("\033[H");     // Move cursor to home position (0,0)
}

// -----------------------------------------------------------------------------
float calculateBusLoad() {
  static uint32_t lastFrameCount = 0;
  static uint32_t lastCalcTime   = 0;
  static float    lastBusLoad    = 0.0f;

  uint32_t now     = millis();
  uint32_t elapsed = now - lastCalcTime;
  if (elapsed < 200) {
    // only recalc every 200ms
    return lastBusLoad;
  }

  uint32_t sent  = totalFramesSent - lastFrameCount;
  lastFrameCount = totalFramesSent;
  lastCalcTime   = now;

  if (sent == 0 || !sendCAN) {
    // no new frames or not sending => zero or previous
    lastBusLoad = sendCAN ? lastBusLoad : 0.0f;
    return lastBusLoad;
  }

  // frames per second
  float fps = float(sent) / (elapsed / 1000.0f);

  // time per frame
  float tFrame = estimateFrameTime(canFDMode, currentPayloadSize);

  // bus‐load %
  float load = fps * tFrame * 100.0f;
  lastBusLoad = (load > 100.0f) ? 100.0f : load;
  return lastBusLoad;
}
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  
  updateCANSettings();  // Initialize CAN with default settings

  pinMode(LED_Pin, OUTPUT);
  digitalWrite(LED_Pin, LOW);
  pinMode(logButton, INPUT_PULLUP);

  // Initialize frame IDs
  frameIDs[0] = 0x555;
  for (int i = 1; i < 100; i++) {  // Increased to 100
    frameIDs[i] = frameIDs[i - 1] + 1;
  }

  // Configure mailboxes - use the last few mailboxes for receiving
  for (int i = 0; i < 12; i++) {  // Use 12 instead of 14 for TX
    m_CANInterface.setMB((FLEXCAN_MAILBOX)i, TX);
  }
  
  // Set up at least 3 mailboxes for receiving
  for (int i = 12; i < 15; i++) {  // Configure mailboxes 12-14 for RX
    m_CANInterface.setMB((FLEXCAN_MAILBOX)i, RX);
    m_CANInterface.setMBFilter((FLEXCAN_MAILBOX)i, ACCEPT_ALL);
  }
  
  m_CANInterface.enableMBInterrupts();

  // Initialize frame counters
  initFrameCounters();

  printCurrentRates();
}

void initFrameCounters() {
  // Free existing counter if it exists
  if (framesSentByID != NULL) {
    free(framesSentByID);
  }
  
  // Allocate memory for per-ID frame counters
  framesSentByID = (uint32_t*)malloc(100 * sizeof(uint32_t));  // Increased to 100
  
  // Initialize all counters to zero
  if (framesSentByID != NULL) {
    totalFramesSent = 0;
    for (int i = 0; i < 100; i++) {  // Increased to 100
      framesSentByID[i] = 0;
    }
  }
}

void printFrameCountSummary() {
  uint32_t elapsedTime = (millis() - sessionStartTime) / 1000; // seconds
  
  Serial.println("\n=== Frame Transmission Summary ===");
  Serial.print("Session duration: ");
  
  // Format the time as hours:minutes:seconds
  uint32_t hours = elapsedTime / 3600;
  uint32_t minutes = (elapsedTime % 3600) / 60;
  uint32_t seconds = elapsedTime % 60;
  
  if (hours > 0) {
    Serial.print(hours);
    Serial.print(" hours, ");
  }
  
  if (minutes > 0 || hours > 0) {
    Serial.print(minutes);
    Serial.print(" minutes, ");
  }
  
  Serial.print(seconds);
  Serial.println(" seconds");
  
  Serial.print("Total frames sent: ");
  Serial.println(totalFramesSent);
  
  Serial.print("Average frames per second: ");
  Serial.println(elapsedTime > 0 ? (float)totalFramesSent / elapsedTime : 0);
  
  Serial.println("\nFrames sent per ID:");
  for (int i = 0; i < frameCount && i < 100; i++) {  // Added bounds check
    Serial.print("0x");
    Serial.print(frameIDs[i], HEX);
    Serial.print(": ");
    Serial.println(framesSentByID[i]);
  }
  Serial.println("===============================");
}

void toggleCANMode() {
  if (canFDMode) {
    // Switch to Standard CAN mode
    canFDMode = false;
    currentPayloadSize = STD_PAYLOAD_SIZE;
    Serial.println("Switched to Standard CAN mode (8-byte payload)");
  } else {
    // Switch to CAN FD mode
    canFDMode = true;
    currentPayloadSize = FD_PAYLOAD_SIZE;
    Serial.println("Switched to CAN FD mode (64-byte payload)");
  }
  
  updateCANSettings();
  printCurrentRates();
}

void updateCANSettings() {
  // We need to stop and restart the controller
  CANFD_timings_t config;
  config.clock = CLK_24MHz;
  config.baudrate = BAUD_RATES[currentBaudRateIndex];
  config.baudrateFD = DATA_RATES[currentDataRateIndex];
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 85;

  // Begin will initialize the controller
  m_CANInterface.begin();
  m_CANInterface.setBaudRate(config);
  m_CANInterface.setRegions(64);
  m_CANInterface.enableMBInterrupts();
  
  // Maintain dedicated RX mailboxes
  for (int i = 0; i < 12; i++) {
    m_CANInterface.setMB((FLEXCAN_MAILBOX)i, TX);
  }
  for (int i = 12; i < 15; i++) {
    m_CANInterface.setMB((FLEXCAN_MAILBOX)i, RX);
    m_CANInterface.setMBFilter((FLEXCAN_MAILBOX)i, ACCEPT_ALL);
  }
}

void printCurrentRates() {
  Serial.print("CAN Mode: ");
  Serial.println(canFDMode ? "FD (64-byte payload)" : "Standard (8-byte payload)");
  
  Serial.print("Baud Rate: ");
  float baudRate = BAUD_RATES[currentBaudRateIndex] / 1000000.0;
  if (baudRate >= 1.0) {
    Serial.print(baudRate, 3);
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

void startBusLoadMonitoring() {
  if (!isMonitoringBusLoad) {
    isMonitoringBusLoad = true;
    busMonitorStartTime = millis();
    framesSentDuringMonitoring = 0;
    framesReceivedDuringMonitoring = 0;
    lastReceivedCount = 0;
    
    // If we're transmitting, store the starting frame count to measure the difference
    if (sendCAN) {
      startingFrameCount = totalFramesSent;
    }
    
    // Make sure receive mailboxes are properly configured
    for (int i = 12; i < 15; i++) {
      m_CANInterface.setMB((FLEXCAN_MAILBOX)i, RX);
      m_CANInterface.setMBFilter((FLEXCAN_MAILBOX)i, ACCEPT_ALL);
    }
    
    Serial.println("Starting bus load monitoring for 3 seconds...");
  }
}

void measureBusLoad() {
  if (!isMonitoringBusLoad) return;

  uint32_t now     = millis();
  uint32_t elapsed = now - busMonitorStartTime;

  // Count our sent frames during the 3s window
  if (sendCAN) {
    framesSentDuringMonitoring = totalFramesSent - startingFrameCount;
  }

  // Once 3s is up, compute & print
  if (elapsed >= BUS_MONITOR_DURATION) {
    isMonitoringBusLoad = false;
    float durS    = elapsed / 1000.0f;
    float ourFps  = framesSentDuringMonitoring / durS;
    float otherFps= framesReceivedDuringMonitoring / durS;
    float totalFps= ourFps + otherFps;

    // Use the same per-frame timing as calculateBusLoad()
    float tFrame  = estimateFrameTime(canFDMode, currentPayloadSize);
    float rawLoad = totalFps * tFrame * 100.0f;
    if (rawLoad > 100.0f) rawLoad = 100.0f;

    // Print fully left-aligned
    Serial.println();
    Serial.println("=== Bus Load Results (3s) ===");
    Serial.print  ("Our fps:           "); Serial.println(ourFps, 1);
    Serial.print  ("Other fps:         "); Serial.println(otherFps, 1);
    Serial.print  ("Total fps:         "); Serial.println(totalFps, 1);
    Serial.print  ("Est. bus load:     "); Serial.print(rawLoad, 1); Serial.println("%");
    Serial.println("=============================");

    // Re-arm the RX mailboxes
    for (int i = 12; i < 15; i++) {
      m_CANInterface.setMB((FLEXCAN_MAILBOX)i, RX);
      m_CANInterface.setMBFilter((FLEXCAN_MAILBOX)i, ACCEPT_ALL);
    }
  }
}

uint32_t lastTx = 0;

void loop() {
  static int lastButtonState = HIGH;
  int buttonState = digitalRead(logButton);

  // Update bus load calculation periodically but don't print automatically
  static uint32_t lastBusUpdateTime = 0;
  if (sendCAN && (millis() - lastBusUpdateTime >= 250)) {
    currentBusLoad = calculateBusLoad();
    lastBusUpdateTime = millis();
  }
  
  // Update bus monitoring if active
  if (isMonitoringBusLoad) {
    CANFD_message_t rx_msg;
    while (m_CANInterface.read(rx_msg)) {
      bool isOurFrame = false;
      for (int i = 0; i < frameCount; i++) {
        if (rx_msg.id == frameIDs[i]) {
          isOurFrame = true;
          break;
        }
      }
      if (!isOurFrame) {
        framesReceivedDuringMonitoring++;
      }
    }
    measureBusLoad();
  }

  m_CANInterface.events();

  // Button press handling
  if (buttonState == LOW && lastButtonState == HIGH) {
    delay(50); // Debounce
    if (digitalRead(logButton) == LOW) {
      sendCAN = !sendCAN;
      lastTx = millis();
      
      if (sendCAN) {
        // Starting transmission
        Serial.println("CAN transmission started.");
        // Reset and start counting
        initFrameCounters();
        countingFrames = true;
        sessionStartTime = millis();
      } else {
        // Stopping transmission
        Serial.println("CAN transmission stopped.");
        countingFrames = false;
        // Print the summary of frames sent
        printFrameCountSummary();
      }
      
      digitalWrite(LED_Pin, sendCAN ? HIGH : LOW);
    }
  }
  lastButtonState = buttonState;

  // Serial command handling
  if (Serial.available() > 0) {
    char input = Serial.read();
    handleSerialInput(input);
  }

  // CAN message handling
  if (sendCAN) {
    uint32_t currentTime = millis();
    if ((currentTime - lastTx) >= 10) {
      // Always reset to the current time to avoid cumulative drift
      lastTx = currentTime;
      sendCANFrames();
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
    bool frameSent = false;
    
    if (canFDMode) {
      CANFD_message_t msg;
      msg.len = currentPayloadSize;
      msg.id = frameIDs[f];
      msg.brs = true;  
      msg.edl = true;  
      msg.esi = false; 
      msg.seq = true;

      digitalWrite(LED_Pin, HIGH);

      for (int i = 0; i < currentPayloadSize; i++) {
        msg.buf[i] = useFFPayload ? PAYLOAD_DATA : payloadData[i];
      }

      frameSent = m_CANInterface.write(msg);
      if (!frameSent) {
        Serial.print("Failed to send FD frame on ID 0x");
        Serial.println(frameIDs[f], HEX);
        digitalWrite(LED_Pin, LOW);
      }
    } else {
      CANFD_message_t msg;
      msg.len = currentPayloadSize;
      msg.id = frameIDs[f];
      msg.brs = false;
      msg.edl = false;
      msg.esi = false;
      msg.seq = true;

      digitalWrite(LED_Pin, HIGH);

      for (int i = 0; i < currentPayloadSize; i++) {
        msg.buf[i] = useFFPayload ? PAYLOAD_DATA : payloadData[i];
      }

      frameSent = m_CANInterface.write(msg);
      if (!frameSent) {
        Serial.print("Failed to send standard frame on ID 0x");
        Serial.println(frameIDs[f], HEX);
        digitalWrite(LED_Pin, LOW);
      }
    }
    
    if (frameSent && countingFrames && framesSentByID != NULL) {
      totalFramesSent++;
      framesSentByID[f]++;
    }
  }
}

void handleSerialInput(char input) {
  clearTerminal();
  switch(input) {
    case 'b':
    case 'B':
      if (!sendCAN) {
        currentBaudRateIndex = (currentBaudRateIndex + 1) % NUM_BAUD_RATES;
        updateCANSettings();
        printCurrentRates();
      } else {
        Serial.println("Cannot change baud rate while CAN is transmitting!");
      }
      break;
    case 'd':
    case 'D':
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
    case 'm':
    case 'M':
      if (!sendCAN) {
        toggleCANMode();
      } else {
        Serial.println("Cannot change CAN mode while transmitting!");
      }
      break;
    case 'l':
    case 'L':
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
    case '+':
    case '=':
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
    case '-':
    case '_':
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
    case 's':
    case 'S':
      if (countingFrames) {
        Serial.println("\n=== Current Session Statistics ===");
        uint32_t elapsedTime = (millis() - sessionStartTime) / 1000; // seconds
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
    case 'h':
    case 'H':
      Serial.println("=== CAN Controller ===");
      Serial.println("Commands available:");
      Serial.print("m - Toggle CAN mode (");
      Serial.print(canFDMode ? "FD" : "Standard");
      Serial.println(")");
      Serial.print("b - Cycle baud rate: ");
      
      float baudRate = BAUD_RATES[currentBaudRateIndex] / 1000000.0;
      if (baudRate >= 1.0) {
        Serial.print(baudRate, 3);
        Serial.println(" Mbps");
      } else {
        Serial.print(BAUD_RATES[currentBaudRateIndex] / 1000.0, 1);
        Serial.println(" Kbps");
      }
      
      if (canFDMode) {
        Serial.print("d - Cycle data rate ");
        Serial.print(DATA_RATES[currentDataRateIndex] / 1000000.0, 1);
        Serial.println(" Mbps");
      }
      if (sendCAN) {
        Serial.print("Current bus load: ");
        Serial.print(currentBusLoad, 1);
        Serial.println("% (Note: Displays only this device's contribution)");
      }
      Serial.println("\r\n=== Live modes ===");
      Serial.println("1 - FF payload mode");
      Serial.println("2 - Incrementing payload mode");
      Serial.println("+ - Add a frame");
      Serial.println("- - Remove a frame");
      Serial.println("S - Show current statistics (during transmission)");
      Serial.println("L - Measure bus load for 3 seconds");
      break;
  }
}
