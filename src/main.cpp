#include <Arduino.h>
#include <FlexCAN_T4.h>

// Define CAN bus settings
const int CAN1_BAUD_RATE = 1000000;  // 1 Mbps
const int CAN1_DATA_RATE = 4000000;  // 4 Mbps
const uint32_t FRAME_ID = 0x555;
const int PAYLOAD_SIZE = 64;
const uint8_t PAYLOAD_DATA = 0xFF;

// Create CAN object
FlexCAN_T4FD<CAN3, RX_SIZE_1024, TX_SIZE_16> m_CANInterface;

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  while (!Serial);

  CANFD_timings_t config;
  config.clock = CLK_24MHz;
  config.baudrate = 1000000;
  config.baudrateFD = 4000000;
  config.propdelay = 190;
  config.bus_length = 1;
  config.sample = 85;
 
  m_CANInterface.begin();
  m_CANInterface.setBaudRate(config);
  m_CANInterface.setRegions(64);
  m_CANInterface.enableMBInterrupts();

}

void loop() {
  // Create CAN FD frame
  CANFD_message_t msg;
  msg.len = PAYLOAD_SIZE;    // 64 bytes payload
  msg.id = FRAME_ID;         // Frame ID
  msg.brs = true;            // Enable baud rate switching
  msg.edl = true;            // Indicate extended data length (FD)

  // Fill payload with 0xFF
  for (int i = 0; i < PAYLOAD_SIZE; i++) {
    msg.buf[i] = PAYLOAD_DATA;
  }

  // Send CAN FD frame
  if (m_CANInterface.write(msg)) {
    Serial.println("CAN FD frame sent.");
  } else {
    Serial.println("Failed to send CAN FD frame.");
  }

  // Wait for 10ms
  delay(10);
}
