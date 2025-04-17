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

float calculateBusLoad() {
  static uint32_t lastFrameCount = 0;
  static uint32_t lastCalcTime = 0;
  static float lastBusLoad = 0.0;
  
  uint32_t currentTime = millis();
  uint32_t elapsedTime = currentTime - lastCalcTime;
  
  // Only recalculate every 200ms to avoid fluctuations
  if (elapsedTime < 200) {
    return lastBusLoad;
  }
  
  // Calculate frames transmitted since last check
  uint32_t framesTransmitted = totalFramesSent - lastFrameCount;
  lastFrameCount = totalFramesSent;
  lastCalcTime = currentTime;
  
  // If no frames or not transmitting, assume the previous value or 0
  if (framesTransmitted == 0 || !sendCAN) {
    lastBusLoad = sendCAN ? lastBusLoad : 0.0;
    return lastBusLoad;
  }
  
  // Calculate frames per second based on actual transmission rate
  float framesPerSecond = (float)framesTransmitted / (elapsedTime / 1000.0);
  
  // Calculate bits per frame
  int dataBits = currentPayloadSize * 8;  // Data bits
  int dataStuffBits = dataBits / STUFF_BIT_RATIO;  // Estimated stuff bits in data
  int overhead = canFDMode ? CAN_FD_OVERHEAD : CAN_STD_OVERHEAD;
  int overheadStuffBits = overhead / STUFF_BIT_RATIO;
  
  // Calculate time for arbitration phase and data phase
  float arbitrationTime = (float)(overhead + overheadStuffBits) / BAUD_RATES[currentBaudRateIndex];
  float dataTime;
  
  if (canFDMode) {
    dataTime = (float)(dataBits + dataStuffBits) / DATA_RATES[currentDataRateIndex];
  } else {
    dataTime = (float)(dataBits + dataStuffBits) / BAUD_RATES[currentBaudRateIndex];
  }
  
  float totalFrameTime = arbitrationTime + dataTime;
  
  // Calculate bus load percentage based on actual measured rate
  lastBusLoad = (totalFrameTime * framesPerSecond) * 100.0;
  
  // Since we can't measure other nodes' traffic without the statistics API,
  // we'll add a warning in the help menu about this limitation
  
  return min(lastBusLoad, 100.0);  // Cap at 100%
}

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

  // Configure the first 14 mailboxes for sending
  for (int i = 0; i < 14; i++) {
    m_CANInterface.setMB((FLEXCAN_MAILBOX)i, TX);
  }
  
  // Configure at least one mailbox for receiving
  m_CANInterface.setMB((FLEXCAN_MAILBOX)14, RX);
  m_CANInterface.setMBFilter((FLEXCAN_MAILBOX)14, ACCEPT_ALL);
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
  // Since both reset() and end() are private/unavailable, let's just create a new configuration
  
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
    
    // Ensure we have proper mailbox configuration for receiving
    if (!sendCAN) {
      // Use at least one mailbox for receiving all messages
      m_CANInterface.setMB((FLEXCAN_MAILBOX)0, RX);
      m_CANInterface.setMBFilter((FLEXCAN_MAILBOX)0, ACCEPT_ALL);
    }
    
    Serial.println("Starting bus load monitoring for 3 seconds...");
  }
}

void measureBusLoad() {
  if (isMonitoringBusLoad) {
    // Monitor time elapsed
    uint32_t currentTime = millis();
    uint32_t elapsedTime = currentTime - busMonitorStartTime;
    
    // Count frames we've sent during monitoring (if transmitting)
    if (sendCAN) {
      // This needs to be the difference between start and end, not just the total
      static uint32_t startFrameCount = 0;
      if (elapsedTime <= 50) { // Record initial count near the start
        startFrameCount = totalFramesSent;
      }
      framesSentDuringMonitoring = totalFramesSent - startFrameCount;
    }
    
    // If monitoring period is over, show results
    if (elapsedTime >= BUS_MONITOR_DURATION) {
      isMonitoringBusLoad = false;
      
      // Calculate frames per second and bus load
      float monitorDuration = elapsedTime / 1000.0; // seconds
      
      // Count our transmitted frames
      float ourFramesPerSecond = 0;
      if (sendCAN) {
        ourFramesPerSecond = framesSentDuringMonitoring / monitorDuration;
      }
      
      // Count received frames from other nodes
      float otherFramesPerSecond = framesReceivedDuringMonitoring / monitorDuration;
      
      // Combined frames per second
      float totalFramesPerSecond = ourFramesPerSecond + otherFramesPerSecond;
      
      // Calculate average frame size based on current mode
      // For simplicity, assume other nodes are using same format as we are
      int frameBits = canFDMode ? 
        ((FD_PAYLOAD_SIZE * 8) + CAN_FD_OVERHEAD + (FD_PAYLOAD_SIZE * 8 / STUFF_BIT_RATIO)) :
        ((STD_PAYLOAD_SIZE * 8) + CAN_STD_OVERHEAD + (STD_PAYLOAD_SIZE * 8 / STUFF_BIT_RATIO));
      
      // Calculate bits per second
      float bitsPerSecond = totalFramesPerSecond * frameBits;
      
      // Calculate bus load as percentage of current baud rate
      float busSpeed = (float)BAUD_RATES[currentBaudRateIndex];
      float totalBusLoad = (bitsPerSecond / busSpeed) * 100.0;
      
      // Debug information
      Serial.println("\nDebug Info:");
      Serial.print("Frames sent during monitoring: "); Serial.println(framesSentDuringMonitoring);
      Serial.print("Frames received during monitoring: "); Serial.println(framesReceivedDuringMonitoring);
      Serial.print("Monitor duration (s): "); Serial.println(monitorDuration);
      Serial.print("Our frames/s: "); Serial.println(ourFramesPerSecond);
      Serial.print("Other frames/s: "); Serial.println(otherFramesPerSecond);
      Serial.print("Frame bits: "); Serial.println(frameBits);
      Serial.print("Bits/s: "); Serial.println(bitsPerSecond);
      Serial.print("Bus speed: "); Serial.println(busSpeed);
      Serial.print("Raw bus load calculation: "); Serial.println(totalBusLoad);
      
      // Cap at 100% for display purposes
      if (totalBusLoad > 100.0) totalBusLoad = 100.0;
      
      // Show results
      Serial.println("\n=== Bus Load Measurement Results ===");
      Serial.print("Monitoring period: ");
      Serial.print(monitorDuration, 1);
      Serial.println(" seconds");
      
      if (sendCAN) {
        Serial.print("Frames sent by us: ");
        Serial.println(framesSentDuringMonitoring);
        Serial.print("Our frames per second: ");
        Serial.println(ourFramesPerSecond, 1);
      }
      
      Serial.print("Frames from other nodes: ");
      Serial.println(framesReceivedDuringMonitoring);
      Serial.print("Other nodes frames per second: ");
      Serial.println(otherFramesPerSecond, 1);
      
      Serial.print("Total frames per second: ");
      Serial.println(totalFramesPerSecond, 1);
      Serial.print("Estimated total bus load: ");
      Serial.print(totalBusLoad, 1);
      Serial.println("%");
      Serial.println("=====================================");
      
      // Restore mailboxes for transmitting if we modified them
      if (!sendCAN) {
        // Reconfigure the mailboxes for transmitting
        for (int i = 0; i < 14; i++) {
          m_CANInterface.setMB((FLEXCAN_MAILBOX)i, TX);
        }
        // Keep mailbox 14 for receiving
        m_CANInterface.setMB((FLEXCAN_MAILBOX)14, RX);
        m_CANInterface.setMBFilter((FLEXCAN_MAILBOX)14, ACCEPT_ALL);
      }
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
  
// In the loop function, modify the bus monitoring code to check for messages from other nodes
  // Update bus monitoring if active
  if (isMonitoringBusLoad) {
    // Check for CAN messages while monitoring
    CANFD_message_t rx_msg;
    if (m_CANInterface.read(rx_msg)) {
      framesReceivedDuringMonitoring++;
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
    if ((millis()-lastTx) >= 10) {
      if ((millis()-lastTx) > 100)
        lastTx = millis();
      else
        lastTx += 10;
      sendCANFrames();
    }
  }
}

void sendCANFrames() {
  // If not using FF payload, increment the payload data just once per cycle
  // This ensures all frames get the same data values
  if (!useFFPayload) {
    for (int i = 0; i < currentPayloadSize; i++) {
      payloadData[i]++;
      if (payloadData[i] > 0xFF) payloadData[i] = 0;
    }
  }

  for (int f = 0; f < frameCount; f++) {
    int mailboxIndex = f % 14; // Cycling through 0 to 13
    bool frameSent = false;
    
    if (canFDMode) {
      // Use CAN FD message format
      CANFD_message_t msg;
      msg.len = currentPayloadSize;
      msg.id = frameIDs[f];
      msg.brs = true;  // Bit Rate Switch
      msg.edl = true;  // Extended Data Length
      msg.esi = false; // Error State Indicator - set to false
      msg.seq = true;

      digitalWrite(LED_Pin, HIGH);

      for (int i = 0; i < currentPayloadSize; i++) {
        msg.buf[i] = useFFPayload ? PAYLOAD_DATA : payloadData[i];
      }

      frameSent = m_CANInterface.write(msg);
      if (!frameSent) {
        Serial.print("Failed to send FD frame on MB ");
        Serial.println(mailboxIndex);
        digitalWrite(LED_Pin, LOW);
      }
    } else {
      // Use Standard CAN message format - send using the FD API but with standard flags
      CANFD_message_t msg;
      msg.len = currentPayloadSize;
      msg.id = frameIDs[f];
      msg.brs = false;  // No Bit Rate Switch for standard CAN
      msg.edl = false;  // No Extended Data Length for standard CAN
      msg.esi = false;  // Error State Indicator - set to false
      msg.seq = true;

      digitalWrite(LED_Pin, HIGH);

      for (int i = 0; i < currentPayloadSize; i++) {
        msg.buf[i] = useFFPayload ? PAYLOAD_DATA : payloadData[i];
      }

      frameSent = m_CANInterface.write(msg);
      if (!frameSent) {
        Serial.print("Failed to send standard frame on MB ");
        Serial.println(mailboxIndex);
        digitalWrite(LED_Pin, LOW);
      }
    }
    
    // Increment the frame counters if the frame was sent successfully
    if (frameSent && countingFrames && framesSentByID != NULL) {
      totalFramesSent++;
      framesSentByID[f]++;
    }
  }
}

void handleSerialInput(char input) {
  clearTerminal();  // Clear terminal before printing new information
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
      if (frameCount < 100) {  // Increased to 100
        frameCount++;
        Serial.print("Frame count: ");
        Serial.println(frameCount);
        if (sendCAN) {
          // Update and display bus load
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
          // Update and display bus load
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
        
        // Update and display bus load
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
      
      // Display the current baud rate
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
      Serial.println("1 - FF payload mode");
      Serial.println("2 - Incrementing payload mode");
      Serial.println("+ - Add a frame");
      Serial.println("- - Remove a frame");
      Serial.println("s - Show current statistics (during transmission)");
      Serial.println("l - Measure bus load for 3 seconds");
      break;
  }
}