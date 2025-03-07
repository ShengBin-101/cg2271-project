#include <Bluepad32.h>

// Maximum number of controllers allowed.
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Use hardware Serial2 for UART communication.
// Adjust TX_PIN and RX_PIN to your wiring.
#define UART_TX_PIN 17
#define UART_RX_PIN 16
HardwareSerial UARTSerial(2);

// ----- Bluepad32 Callbacks -----
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but no empty slot available");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }
  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but was not found in myControllers");
  }
}

// ----- Process Controller Data and Send UART -----
// This function reads the required values, maps them to 0-3 levels, packs them into a byte, and sends via UART.
void processController(ControllerPtr ctl) {
  // Ensure we have a gamepad with updated data.
  if (!ctl->isConnected() || !ctl->hasData() || !ctl->isGamepad()) return;

  // Read values:
  // axisRX: range (-511 to 512) – used for steering left/right.
  int16_t axisRX = ctl->axisRX();
  // throttle(): (0 to 1023) – used as forward control.
  int throttleVal = ctl->throttle();
  // brake(): (0 to 1023) – used as backward control.
  int brakeVal = ctl->brake();

  // --- Map axisRX to left/right levels ---
  // If axisRX is negative, map its absolute value to left level.
  // If positive, map to right level.
  uint8_t leftLevel = 0, rightLevel = 0;
  if (axisRX < 0) {
    // Map abs(axisRX) from 0-511 to 0-3
    leftLevel = map(abs(axisRX), 0, 508, 0, 3);
    if (leftLevel > 3) leftLevel = 3;
  } else if (axisRX > 0) {
    rightLevel = map(axisRX, 0, 512, 0, 3);
    if (rightLevel > 3) rightLevel = 3;
  }
  
  // --- Map throttle and brake to forward/backward levels ---
  // Map throttle (forward) from 0-1023 to 0-3.
  uint8_t forwardLevel = map(throttleVal, 0, 1020, 0, 3);
  if (forwardLevel > 3) forwardLevel = 3;
  // Map brake (backward) from 0-1023 to 0-3.
  uint8_t backwardLevel = map(brakeVal, 0, 1020, 0, 3);
  if (backwardLevel > 3) backwardLevel = 3;

  // --- Pack the 4 levels into one byte ---
  // Bit layout (from LSB to MSB):
  // Bits 1-0: left, Bits 3-2: right, Bits 5-4: forward, Bits 7-6: backward
  uint8_t packet = ((backwardLevel & 0x03) << 6) |
                   ((forwardLevel  & 0x03) << 4) |
                   ((rightLevel    & 0x03) << 2) |
                   ((leftLevel     & 0x03) << 0);

  // Send the packet over UART.
  UARTSerial.write(packet);

  // Optional: print debug info.
  Serial.print("axisRX: "); Serial.print(axisRX);
  Serial.print(" | Left: "); Serial.print(leftLevel);
  Serial.print(" Right: "); Serial.print(rightLevel);
  Serial.print(" | Forward: "); Serial.print(forwardLevel);
  Serial.print(" Backward: "); Serial.println(backwardLevel);
}

// Process all connected controllers.
void processControllers() {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] && myControllers[i]->isConnected() && myControllers[i]->hasData()) {
      if (myControllers[i]->isGamepad()) {
        processController(myControllers[i]);
      }
    }
  }
}

// ----- Arduino Setup and Loop -----
void setup() {
  Serial.begin(115200);
  // Initialize UART Serial2 for communication with Kinetis Cortex‑M.
  UARTSerial.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Set up Bluepad32 callbacks.
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void loop() {
  // Update Bluepad32 – fetch data from connected controllers.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  delay(50); // Adjust delay as needed.
}
