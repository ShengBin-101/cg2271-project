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
  // axisRX: range (-508 to 512) – used for steering left/right.
  int16_t axisRX = ctl->axisRX();
  // axisY: range (-511 to 512) – used for steering left/right.
  int16_t axisLY = ctl->axisY();
  // buttonX: bool - used to activate buzzer.
  bool buttonX = ctl->a();

  // --- Map axisRX to steer levels ---
  // Most left steer will be level 0
  // Most right steer will be level 14
  // No steer will be level 7
  uint8_t steerLevel = 7;
  if (axisRX < -31)  
    steerLevel = map(abs(axisRX), 32, 508, 6, 0);
  else if (axisRX > 31)  
    steerLevel = map(axisRX, 32, 512, 8, 14);
  
  // --- Map axisLY to throttle levels ---
  // Most backward throttle will be level 0
  // Most forward throttle will be level 14
  // No throttle will be level 7
  uint8_t throttleLevel = 7;
  if (axisLY > 31)  
    throttleLevel = map(axisLY, 32, 512, 6, 0);
  else if (axisLY < -31)  
    throttleLevel = map(abs(axisLY), 32, 508, 8, 14);

  // If button X is pressed, used throttle level at 15 to
  // indicate buzzer needs to turn on
  if(buttonX){
    throttleLevel = 15;
  }

  // --- Pack throttleLevel into bits 7-4 and steerLevel into bits 3-0 ---
  uint8_t packet = ((throttleLevel & 0x0F) << 4) |  // Bits 7-4
                   ((steerLevel & 0x0F) << 0);      // Bits 3-0

  // Send the packet over UART.
  UARTSerial.write(packet);

  // Optional: print debug info.
  Serial.print("Steering: "); Serial.print(axisRX);
  Serial.print(" | "); Serial.print(steerLevel);
  Serial.print(" level |");
  Serial.print(" Throttle: "); Serial.print(axisLY);
  Serial.print(" | "); Serial.print(throttleLevel);
  Serial.println(" level");
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