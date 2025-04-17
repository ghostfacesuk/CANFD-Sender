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

int sendCount = 0; // sending serial message 
bool sendCAN = false; // Control flag for CAN sending
bool useFFPayload = true; // Control flag for payload mode

// Create CAN object
FlexCAN_T4FD<CAN3, RX_SIZE_1024, TX_SIZE_1024> m_CANInterface;

// Payload data array
uint8_t payloadData[FD_PAYLOAD_SIZE] = {0};

// CAN frame IDs array and current frame count
uint32_t frameIDs[51];
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

void clearTerminal() {
  // ANSI escape codes to clear screen and move cursor to top
  Serial.write("\033[2J");    // Clear screen
  Serial.write("\033[H");     // Move cursor to home position (0,0)
}

float calculateBusLoad() {
  if (!sendCAN) return 0.0;
  
  // Calculate bits per frame
  int dataBits = currentPayloadSize * 8;  // Data bits
  int dataStuffBits = dataBits / STUFF_BIT_RATIO;  // Estimated stuff bits in data
  int overhead = canFDMode ? CAN_FD_OVERHEAD : CAN_STD_OVERHEAD;
  
  // Calculate time for arbitration phase (including stuff bits) and data phase
  float arbitrationTime = (float)(overhead + (overhead / STUFF_BIT_RATIO)) / BAUD_RATES[currentBaudRateIndex];
  float dataTime;
  
  if (canFDMode) {
    dataTime = (float)(dataBits + dataStuffBits) / DATA_RATES[currentDataRateIndex];
  } else {
    dataTime = (float)(dataBits + dataStuffBits) / BAUD_RATES[currentBaudRateIndex];
  }
  
  float totalFrameTime = arbitrationTime + dataTime;
  
  // Calculate frames per second (we're sending every 10ms)
  float framesPerSecond = frameCount * (1000.0 / 10.0);  // 10ms interval
  
  // Calculate bus load percentage
  float busLoad = (totalFrameTime * framesPerSecond) * 100.0;
  
  return min(busLoad, 100.0);  // Cap at 100%
}

void setup() {
  Serial.begin(115200);
  
  updateCANSettings();  // Initialize CAN with default settings

  pinMode(LED_Pin, OUTPUT);
  digitalWrite(LED_Pin, LOW);
  pinMode(logButton, INPUT_PULLUP);

  // Initialize frame IDs
  frameIDs[0] = 0x555;
  for (int i = 1; i < 51; i++) {
    frameIDs[i] = frameIDs[i - 1] + 1;
  }

  // Configure the first 14 mailboxes for sending
  for (int i = 0; i < 14; i++) {
    m_CANInterface.setMB((FLEXCAN_MAILBOX)i, TX);
  }

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
  framesSentByID = (uint32_t*)malloc(51 * sizeof(uint32_t));
  
  // Initialize all counters to zero
  if (framesSentByID != NULL) {
    totalFramesSent = 0;
    for (int i = 0; i < 51; i++) {
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
  for (int i = 0; i < frameCount; i++) {
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

uint32_t lastTx = 0;

void loop() {
  static int lastButtonState = HIGH;
  int buttonState = digitalRead(logButton);

  // Update bus load calculation every 500ms
  if (sendCAN && (millis() - lastBusLoadCalc >= 500)) {
    currentBusLoad = calculateBusLoad();
    lastBusLoadCalc = millis();
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
      if (frameCount < 51) {
        frameCount++;
        Serial.print("Frame count: ");
        Serial.println(frameCount);
        if (sendCAN) {
          currentBusLoad = calculateBusLoad();
          Serial.print("Bus load: ");
          Serial.print(currentBusLoad, 1);
          Serial.println("%");
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
          Serial.println("%");
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
        Serial.println("%");
      }
      Serial.println("1 - FF payload mode");
      Serial.println("2 - Incrementing payload mode");
      Serial.println("+ - Add a frame");
      Serial.println("- - Remove a frame");
      Serial.println("s - Show current statistics (during transmission)");
      break;
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