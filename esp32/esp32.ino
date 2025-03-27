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
// #include <Bluepad32.h>

// // Maximum number of controllers allowed.
// ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// // Use hardware Serial2 for UART communication.
// // Adjust TX_PIN and RX_PIN to your wiring.
// #define UART_TX_PIN 17
// #define UART_RX_PIN 16
// HardwareSerial UARTSerial(2);

// // ----- Macros for easier adjustment -----
// #define DEAD_ZONE_THRESHOLD 31
// #define AXISRX_MIN 32
// #define AXISRX_MAX 508
// #define AXISLY_MIN 32
// #define AXISLY_MAX 512
// #define STEER_LEVEL_MIN 0
// #define STEER_LEVEL_MAX 14
// #define THROTTLE_LEVEL_MIN 0
// #define THROTTLE_LEVEL_MAX 14
// #define NEUTRAL_LEVEL 7

// // Global flag for enabling/disabling debug printing
// bool DEBUG_PRINT = true;

// // Custom non-linear mapping function
// uint8_t nonLinearMap(int16_t input, int16_t inputMin, int16_t inputMax, uint8_t outputMin, uint8_t outputMax) {
//   // Normalize input to a range of 0.0 to 1.0
//   float normalized = (float)(input - inputMin) / (inputMax - inputMin);
//   if (normalized < 0.0) normalized = 0.0;
//   if (normalized > 1.0) normalized = 1.0;

//   // Apply a non-linear transformation (e.g., quadratic)
//   float transformed = normalized * normalized; // Quadratic scaling

//   // Scale back to the output range
//   return outputMin + (uint8_t)(transformed * (outputMax - outputMin));
// }

// // ----- Bluepad32 Callbacks -----
// void onConnectedController(ControllerPtr ctl) {
//   bool foundEmptySlot = false;
//   for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
//     if (myControllers[i] == nullptr) {
//       if (DEBUG_PRINT) {
//         Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
//       }
//       myControllers[i] = ctl;
//       foundEmptySlot = true;
//       break;
//     }
//   }
//   if (!foundEmptySlot && DEBUG_PRINT) {
//     Serial.println("CALLBACK: Controller connected, but no empty slot available");
//   }
// }

// void onDisconnectedController(ControllerPtr ctl) {
//   bool foundController = false;
//   for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
//     if (myControllers[i] == ctl) {
//       if (DEBUG_PRINT) {
//         Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
//       }
//       myControllers[i] = nullptr;
//       foundController = true;
//       break;
//     }
//   }
//   if (!foundController && DEBUG_PRINT) {
//     Serial.println("CALLBACK: Controller disconnected, but was not found in myControllers");
//   }
// }

// // ----- Process Controller Data and Send UART -----
// void processController(ControllerPtr ctl) {
//   // Ensure we have a gamepad with updated data.
//   if (!ctl->isConnected() || !ctl->hasData() || !ctl->isGamepad()) return;

//   // Read values:
//   int16_t axisRX = ctl->axisRX();
//   int16_t axisLY = ctl->axisY();

//   // --- Non-linear mapping for axisRX (Steering) ---
//   uint8_t steerLevel = NEUTRAL_LEVEL;
//   if (axisRX < -DEAD_ZONE_THRESHOLD)
//     steerLevel = nonLinearMap(abs(axisRX), AXISRX_MIN, AXISRX_MAX, NEUTRAL_LEVEL - 1, STEER_LEVEL_MIN);
//   else if (axisRX > DEAD_ZONE_THRESHOLD)
//     steerLevel = nonLinearMap(axisRX, AXISRX_MIN, AXISRX_MAX, NEUTRAL_LEVEL + 1, STEER_LEVEL_MAX);

//   // --- Non-linear mapping for axisLY (Throttle) ---
//   uint8_t throttleLevel = NEUTRAL_LEVEL;
//   if (axisLY > DEAD_ZONE_THRESHOLD)
//     throttleLevel = nonLinearMap(axisLY, AXISLY_MIN, AXISLY_MAX, NEUTRAL_LEVEL - 1, THROTTLE_LEVEL_MIN);
//   else if (axisLY < -DEAD_ZONE_THRESHOLD)
//     throttleLevel = nonLinearMap(abs(axisLY), AXISLY_MIN, AXISLY_MAX, NEUTRAL_LEVEL + 1, THROTTLE_LEVEL_MAX);

//   // --- Pack throttleLevel into bits 7-4 and steerLevel into bits 3-0 ---
//   uint8_t packet = ((throttleLevel & 0x0F) << 4) | ((steerLevel & 0x0F) << 0);

//   // Send the packet over UART.
//   UARTSerial.write(packet);

//   // Optional: print debug info.
//   if (DEBUG_PRINT) {
//     Serial.print("Steering: "); Serial.print(axisRX);
//     Serial.print(" | "); Serial.print(steerLevel);
//     Serial.print(" level |");
//     Serial.print(" Throttle: "); Serial.print(axisLY);
//     Serial.print(" | "); Serial.print(throttleLevel);
//     Serial.println(" level");
//   }
// }

// // Process all connected controllers.
// void processControllers() {
//   for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
//     if (myControllers[i] && myControllers[i]->isConnected() && myControllers[i]->hasData()) {
//       if (myControllers[i]->isGamepad()) {
//         processController(myControllers[i]);
//       }
//     }
//   }
// }

// // ----- Arduino Setup and Loop -----
// void setup() {
//   Serial.begin(115200);
//   // Initialize UART Serial2 for communication with Kinetis Cortex‑M.
//   UARTSerial.begin(9600, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

//   if (DEBUG_PRINT) {
//     Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
//     const uint8_t* addr = BP32.localBdAddress();
//     Serial.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
//   }

//   // Set up Bluepad32 callbacks.
//   BP32.setup(&onConnectedController, &onDisconnectedController);
//   BP32.forgetBluetoothKeys();
//   BP32.enableVirtualDevice(false);
// }

// void loop() {
//   // Update Bluepad32 – fetch data from connected controllers.
//   bool dataUpdated = BP32.update();
//   if (dataUpdated) {
//     processControllers();
//   }
//   delay(50); // Adjust delay as needed.
// }