#include <Arduino.h>
#include <FlexCAN_T4.h>

// Define CAN bus settings
const int NUM_BAUD_RATES = 2;
const int BAUD_RATES[NUM_BAUD_RATES] = {1000000, 500000};  // 1 Mbps, 500 Kbps
const int NUM_DATA_RATES = 4;
const int DATA_RATES[NUM_DATA_RATES] = {1000000, 2000000, 4000000, 5000000};  // 1, 2, 4, 5 Mbps
int currentBaudRateIndex = 0;
int currentDataRateIndex = 2;  // Start with 4 Mbps as default

const int PAYLOAD_SIZE = 64;
const uint8_t PAYLOAD_DATA = 0xFF;
const int LED_Pin = 13;  // LED pin
const int logButton = 29;  // Button pin

// Variables for bus load calculation
const int CAN_FD_OVERHEAD = 29;  // CAN FD overhead bits (SOF, ID, Control, CRC, etc.)
const int STUFF_BIT_RATIO = 5;   // Estimate: add 1 stuff bit every 5 bits
uint32_t lastBusLoadCalc = 0;
float currentBusLoad = 0.0;

int sendCount = 0; // sending serial message 
bool sendCAN = false; // Control flag for CAN sending
bool useFFPayload = true; // Control flag for payload mode

// Create CAN object
FlexCAN_T4FD<CAN3, RX_SIZE_1024, TX_SIZE_1024> m_CANInterface;

// Payload data array
uint8_t payloadData[PAYLOAD_SIZE] = {0};

// CAN frame IDs array and current frame count
uint32_t frameIDs[51];
int frameCount = 1;  // Start with one frame

// Function prototypes
void handleSerialInput(char input);
void sendCANFrames();
void updateCANSettings();
void printCurrentRates();
float calculateBusLoad();

float calculateBusLoad() {
  if (!sendCAN) return 0.0;
  
  // Calculate bits per frame
  int dataBits = PAYLOAD_SIZE * 8;  // Data bits
  int dataStuffBits = dataBits / STUFF_BIT_RATIO;  // Estimated stuff bits in data
  
  // Calculate time for arbitration phase (including stuff bits) and data phase
  float arbitrationTime = (float)(CAN_FD_OVERHEAD + (CAN_FD_OVERHEAD / STUFF_BIT_RATIO)) / BAUD_RATES[currentBaudRateIndex];
  float dataTime = (float)(dataBits + dataStuffBits) / DATA_RATES[currentDataRateIndex];
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

void updateCANSettings() {
  CANFD_timings_t config;
  config.clock = CLK_24MHz;
  config.baudrate = BAUD_RATES[currentBaudRateIndex];
  config.baudrateFD = DATA_RATES[currentDataRateIndex];
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 85;

  m_CANInterface.begin();
  m_CANInterface.setBaudRate(config);
  m_CANInterface.setRegions(64);
  m_CANInterface.enableMBInterrupts();
}

void printCurrentRates() {
  Serial.print("Current Baud Rate: ");
  Serial.print(BAUD_RATES[currentBaudRateIndex] / 1000000.0, 3);
  Serial.println(" Mbps");
  Serial.print("Current Data Rate: ");
  Serial.print(DATA_RATES[currentDataRateIndex] / 1000000.0, 1);
  Serial.println(" Mbps");
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
        currentDataRateIndex = (currentDataRateIndex + 1) % NUM_DATA_RATES;
        updateCANSettings();
        printCurrentRates();
      } else {
        Serial.println("Cannot change data rate while CAN is transmitting!");
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
      Serial.println("Commands available:");
      Serial.print("b - Cycle baud rate ");
      Serial.print(BAUD_RATES[currentBaudRateIndex] / 1000000.0, 3);
      Serial.println(" Mbps (when not transmitting)");
      Serial.print("d - Cycle data rate ");
      Serial.print(DATA_RATES[currentDataRateIndex] / 1000000.0, 1);
      Serial.println(" Mbps (when not transmitting)");
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
    CANFD_message_t msg;
    msg.len = PAYLOAD_SIZE;
    msg.id = frameIDs[f];
    msg.brs = true;
    msg.edl = true;
    msg.seq = true;

    digitalWrite(LED_Pin, HIGH);

    for (int i = 0; i < PAYLOAD_SIZE; i++) {
      msg.buf[i] = useFFPayload ? PAYLOAD_DATA : payloadData[i];
    }

    if (!m_CANInterface.write(msg)) {
      Serial.print("Failed to send on MB ");
      Serial.println(mailboxIndex);
      digitalWrite(LED_Pin, LOW);
    }

    if (!useFFPayload) {
      for (int i = 0; i < PAYLOAD_SIZE; i++) {
        payloadData[i]++;
        if (payloadData[i] > 0xFF) payloadData[i] = 0;
      }
    }
  }
}