/**
 * Trinamic TMC2209 has UART interface that replaces need for driving motor with STEP/DIR pins
 *
 * This example uses DIAG pin to reverse motor direction if stall is detected.
 */

// INCLUDES
// See https://github.com/janelia-arduino/TMC2209
#include <TMC2209.h>

// CONSTANTS
const uint8_t runCurrentPercent = 100;
// Detection threshold for stall. 0..255. The StallGuard value SG_RESULT is compared to the double of this threshold.
// If SG_RESULT ≤ SGTHRS*2 stall has occurred, so higher values of threshold = more sensitive
const uint8_t stallGuardThreshold = 40;
constexpr uint8_t diagPin = 13;
// Rx connected directly to PDN pin
constexpr uint8_t rxPin = 26;
// Tx connected to PDN via 1k resistor. See https://gist.github.com/marcelrv/540565978f5b29cf5f2df9ad86af6632
constexpr uint8_t txPin = 27;
// LED pin will be used as an indicator for when StallGuard has been triggered
constexpr uint8_t ledPin = 2;
// Driver enable pin (could probably be tied to GND?)
constexpr uint8_t enPin = 14;


// GLOBALS
// Instantiate serial interface to the TMC2209
TMC2209 stepper_driver;
volatile bool isStalled = false;
unsigned long lastStallTime = 0;
int8_t dir = 1;

// ISR called when DIAG pin goes high, indicating a stall
void IRAM_ATTR diagnose(){
  if(millis() - lastStallTime > 1000){
    isStalled = true;
    lastStallTime = millis();
  }
}

void setup() {
  // For debugging
  Serial.begin(115200);
  Serial.println(__FILE__ __DATE__);

  // Set output pins
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);
  pinMode(ledPin, OUTPUT);
  // HIGH level on stall detection or driver error. See Table 2.2 https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf
  pinMode(diagPin, INPUT);
  attachInterrupt(diagPin, diagnose, RISING);

  // Setup secondary UART interface to TMC2209
  stepper_driver.setup(Serial2, 115200, TMC2209::SERIAL_ADDRESS_0, rxPin, txPin);
  if(!stepper_driver.isSetupAndCommunicating()) {
    Serial.println("Stepper driver not setup and communicating!");
    return;
  }
  // Setup TMC2209 driver
  stepper_driver.clearReset();
  stepper_driver.clearDriveError();
  stepper_driver.setRunCurrent(runCurrentPercent);
  // See 5.3 StallGuard Control
  stepper_driver.setCoolStepDurationThreshold(0xFFFF);
  stepper_driver.setStallGuardThreshold(stallGuardThreshold);
  stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);
  // Changing this will have an effect on the stall guard threshold
  //stepper_driver.setMicrostepsPerStep(4);
  stepper_driver.enable();

  stepper_driver.moveAtVelocity(20000 * dir);
}

void loop() {
  // Output to the serial monitor if we stall
  if(isStalled){
    Serial.println("STALLED!");
    isStalled = false;
    dir = -dir;
    stepper_driver.moveAtVelocity(20000 * dir);
  }
  // LED indicates stall status
  digitalWrite(ledPin, millis() - lastStallTime < 500 ? HIGH : LOW);  
}