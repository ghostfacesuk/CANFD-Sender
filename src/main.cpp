#include <Arduino.h>
#include <FlexCAN_T4.h>

// Define CAN bus settings
const int CAN3_BAUD_RATE = 1000000;  // 1 Mbps
const int CAN3_DATA_RATE = 4000000;  // 4 Mbps
const uint32_t FRAME_ID = 0x555;
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

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  // while (!Serial); //when enabled does not start CAN Tx until serial monitor is opened

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

  pinMode(LED_Pin, OUTPUT);  // Set the LED pin as an output
  digitalWrite(LED_Pin, LOW);  // Ensure LED is off initially
  pinMode(logButton, INPUT_PULLUP);  // Set the log button as an input with internal pull-up resistor
}

void loop() {
  static int lastButtonState = HIGH;  // the previous reading from the input pin
  int buttonState = digitalRead(logButton);  // current state of the button

  // Check if button state changed from high to low (button press)
  if (buttonState == LOW && lastButtonState == HIGH) {
    // Debounce delay
    delay(50);
    // Check button state again to confirm it is still pressed
    if (digitalRead(logButton) == LOW) {
      sendCAN = !sendCAN;  // Toggle the sending state
      if (sendCAN) {
        Serial.println("CAN transmission started.");
      } else {
        Serial.println("CAN transmission stopped.");
        digitalWrite(LED_Pin, LOW);  // Ensure LED is off when transmission stops
      }
    }
  }
  lastButtonState = buttonState;  // Save the current state as the last state, for next loop iteration

  // Check for serial input to switch payload modes
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == '1') {
      useFFPayload = true;
      Serial.println("Switched to FF payload mode.");
    } else if (input == '2') {
      useFFPayload = false;
      Serial.println("Switched to incrementing payload mode.");
    }
  }

  if (sendCAN) {
    // Create CAN FD frame
    CANFD_message_t msg;
    msg.len = PAYLOAD_SIZE;    // 64 bytes payload
    msg.id = FRAME_ID;         // Frame ID
    msg.brs = true;            // Enable baud rate switching
    msg.edl = true;            // Indicate extended data length (FD)

    // Fill payload based on the current mode
    if (useFFPayload) {
      for (int i = 0; i < PAYLOAD_SIZE; i++) {
        msg.buf[i] = PAYLOAD_DATA;
      }
    } else {
      for (int i = 0; i < PAYLOAD_SIZE; i++) {
        msg.buf[i] = payloadData[i];
      }
    }

    // Send CAN FD frame
    if (m_CANInterface.write(msg)) {
      digitalWrite(LED_Pin, HIGH);  // Turn on the LED
      sendCount++;
      if (sendCount == 100) {
        Serial.println("100 CAN FD frames sent.");
        sendCount = 0;
      }
    } else {
      digitalWrite(LED_Pin, LOW);  // Turn off the LED if sending failed
    }

    // Increment payload data if in incrementing mode
    if (!useFFPayload) {
      for (int i = 0; i < PAYLOAD_SIZE; i++) {
        payloadData[i]++;
        if (payloadData[i] > 0xFF) {
          payloadData[i] = 0;
        }
      }
    }

    // Wait for 10ms
    delay(10);
    digitalWrite(LED_Pin, LOW);  // Turn off the LED after delay, showing the LED on only when sending
  }
}
