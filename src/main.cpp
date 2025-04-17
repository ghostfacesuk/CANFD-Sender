#include <Arduino.h>
#include <FlexCAN_T4.h>

// Define CAN bus settings
const int NUM_BAUD_RATES = 2;
const int BAUD_RATES[NUM_BAUD_RATES] = {1000000, 500000};  // 1 Mbps, 500 Kbps
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

  printCurrentRates();
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
  Serial.print(BAUD_RATES[currentBaudRateIndex] / 1000000.0, 3);
  Serial.println(" Mbps");
  
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
      Serial.println(sendCAN ? "CAN transmission started." : "CAN transmission stopped.");
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
    case 'h':
    case 'H':
      Serial.println("=== CAN Controller ===");
      Serial.println("Commands available:");
      Serial.print("m - Toggle CAN mode (");
      Serial.print(canFDMode ? "FD" : "Standard");
      Serial.println(")");
      Serial.print("b - Cycle baud rate ");
      Serial.print(BAUD_RATES[currentBaudRateIndex] / 1000000.0, 3);
      Serial.println(" Mbps");
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
      break;
  }
}

void sendCANFrames() {
  for (int f = 0; f < frameCount; f++) {
    int mailboxIndex = f % 14; // Cycling through 0 to 13
    
    if (canFDMode) {
      // Use CAN FD message format
      CANFD_message_t msg;
      msg.len = currentPayloadSize;
      msg.id = frameIDs[f];
      msg.brs = true;  // Bit Rate Switch
      msg.edl = true;  // Extended Data Length
      msg.seq = true;

      digitalWrite(LED_Pin, HIGH);

      for (int i = 0; i < currentPayloadSize; i++) {
        msg.buf[i] = useFFPayload ? PAYLOAD_DATA : payloadData[i];
      }

      if (!m_CANInterface.write(msg)) {
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

      if (!m_CANInterface.write(msg)) {
        Serial.print("Failed to send standard frame on MB ");
        Serial.println(mailboxIndex);
        digitalWrite(LED_Pin, LOW);
      }
    }

    if (!useFFPayload) {
      for (int i = 0; i < currentPayloadSize; i++) {
        payloadData[i]++;
        if (payloadData[i] > 0xFF) payloadData[i] = 0;
      }
    }
  }
}