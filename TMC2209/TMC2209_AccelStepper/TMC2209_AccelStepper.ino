/**
 * Trinamic TMC2209 has DIAG pin which is triggered when StallGuard activated.
 * This example sends STEP/DIR commands generated via AccelStepper library
 * and reverses direction when stall detected by interrupt triggered from DIAG pin
 */

// INCLUDES
// See https://github.com/janelia-arduino/TMC2209
#include <TMC2209.h>
#include <AccelStepper.h>

// CONSTANTS
// Current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 100;

// Detection threshold for stall. 0..255. 
const uint8_t STALL_GUARD_THRESHOLD = 30;
constexpr uint8_t stepPin = 25;
constexpr uint8_t dirPin = 33;
constexpr uint8_t diagPin = 13;
// Rx connected directly to PDN pin
constexpr uint8_t rxPin = 26;
// Tx conneted to PDN via 1k resistor. See https://gist.github.com/marcelrv/540565978f5b29cf5f2df9ad86af6632
constexpr uint8_t txPin = 27;
// LED pin will be used as an indicator for when StallGuard has been triggered
constexpr uint8_t ledPin = 2;
// Driver enable pin (could probably be tied to GND?)
constexpr uint8_t enPin = 14;

// GLOBALS
// Instantiate serial interface to the TMC2209
TMC2209 stepper_driver;
// Define a stepper and the pins it will use
AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, stepPin, dirPin);
volatile bool isStalled = false;
unsigned long lastStallTime = 0;
int8_t dir = 1;

// ISR called when DIAG pin goes high, indicating a stall
void IRAM_ATTR diagnose(){
  if(millis() - lastStallTime > 3000){
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
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  // HIGH level on stall detection or driver error. See Table 2.2 https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf
  pinMode(diagPin, INPUT);
  attachInterrupt(diagPin, diagnose, RISING);

  // Setup secondary UART interface to TMC2209
  // Note that even though we're not retrieving the stallguard value from this, it is still required to
  // configure the TMC2209
  stepper_driver.setup(Serial2, 115200, TMC2209::SERIAL_ADDRESS_0, rxPin, txPin);

  // Setup TMC2209 driver
  stepper_driver.clearReset();
  stepper_driver.clearDriveError();
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  // Required for DIAG output of StallGuard to work
  // See s11.2 https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.09.pdf
  // See 5.3 StallGuard Control
  stepper_driver.setCoolStepDurationThreshold(0xFFFF);
  stepper_driver.setStallGuardThreshold(STALL_GUARD_THRESHOLD);
  stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);
  // Changing this will have an effect on the stall guard threshold
  stepper_driver.setMicrostepsPerStep(2);
  stepper_driver.enable();
  if(!stepper_driver.isSetupAndCommunicating()) {
    Serial.println("Stepper driver not setup and communicating!");
    return;
  }
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);
}

void loop() {
  // Output to the serial monitor if we stall
  if(isStalled){
    Serial.println("STALLED!");
    isStalled = false;
    dir = -dir;
  }
  // LED indicates stall status
  digitalWrite(ledPin, millis() - lastStallTime < 500 ? HIGH : LOW);

  stepper.move(100*dir);
  stepper.run();
}