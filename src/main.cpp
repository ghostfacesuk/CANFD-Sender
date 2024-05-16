#include <Arduino.h>
#include <FlexCAN_T4.h>

// Define CAN bus settings
const int CAN3_BAUD_RATE = 1000000;  // 1 Mbps
const int CAN3_DATA_RATE = 4000000;  // 4 Mbps
const int PAYLOAD_SIZE = 64;
const uint8_t PAYLOAD_DATA = 0xFF;
const int LED_Pin = 13;  // LED pin
const int logButton = 29;  // Button pin

int sendCount = 0; // sending serial message 
bool sendCAN = false; // Control flag for CAN sending
bool useFFPayload = true; // Control flag for payload mode

// Create CAN object
FlexCAN_T4FD<CAN3, RX_SIZE_1024, TX_SIZE_16> m_CANInterface;

// Payload data array
uint8_t payloadData[PAYLOAD_SIZE] = {0};

// CAN frame IDs array and current frame count
uint32_t frameIDs[36];
int frameCount = 1;  // Start with one frame

// Function prototypes
void handleSerialInput(char input);
void sendCANFrames();

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait until serial console is opened

  CANFD_timings_t config;
  config.clock = CLK_24MHz;
  config.baudrate = CAN3_BAUD_RATE;
  config.baudrateFD = CAN3_DATA_RATE;
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 85;

  m_CANInterface.begin();
  m_CANInterface.setBaudRate(config);
  m_CANInterface.setRegions(64);
  m_CANInterface.enableMBInterrupts();

  pinMode(LED_Pin, OUTPUT);
  digitalWrite(LED_Pin, LOW);
  pinMode(logButton, INPUT_PULLUP);

  // Initialize frame IDs
  frameIDs[0] = 0x555;
  for (int i = 1; i < 36; i++) {
    frameIDs[i] = frameIDs[i - 1] + 1;
  }

  // Configure the first 14 mailboxes for sending
  for (int i = 0; i < 14; i++) {
    m_CANInterface.setMB((FLEXCAN_MAILBOX)i, TX);
  }
}

void loop() {
  static int lastButtonState = HIGH;
  int buttonState = digitalRead(logButton);

  // Button press handling
  if (buttonState == LOW && lastButtonState == HIGH) {
    delay(50); // Debounce
    if (digitalRead(logButton) == LOW) {
      sendCAN = !sendCAN;
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
    sendCANFrames();
  }
}


void handleSerialInput(char input) {
  switch(input) {
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
      if (frameCount < 36) {
        frameCount++;
        Serial.print("Frame count: ");
        Serial.println(frameCount);
      }
      break;
    case '-':
    case '_':
      if (frameCount > 1) {
        frameCount--;
        Serial.print("Frame count: ");
        Serial.println(frameCount);
      }
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

    for (int i = 0; i < PAYLOAD_SIZE; i++) {
      msg.buf[i] = useFFPayload ? PAYLOAD_DATA : payloadData[i];
    }

    if (!m_CANInterface.write((FLEXCAN_MAILBOX)mailboxIndex, msg)) {
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

  delay(10);
  digitalWrite(LED_Pin, LOW); // Turn off LED after sending
}
