/**
 * Trinamic TMC2209 has UART interface that replaces need for driving motor with STEP/DIR pins
 *
 * This example uses UART interface to control the motor, and also schedules a task to 
 * periodically poll the TMC2209 to see whether a stall has been detected.
 */

// INCLUDES
// See https://github.com/janelia-arduino/TMC2209
#include <TMC2209.h>
#include <TaskScheduler.h>

// CONSTANTS
const uint8_t runCurrentPercent = 100;
// Detection threshold for stall. 0..255. The StallGuard value SG_RESULT is compared to the double of this threshold.
// If SG_RESULT ≤ SGTHRS*2 stall has occurred, so higher values of threshold = more sensitive
const uint8_t stallGuardThreshold = 100;
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

void stallguardPollCallback();
Task stallguardPollTask(500, TASK_FOREVER, &stallguardPollCallback);
Scheduler ts;

void stallguardPollCallback() {
  Serial.print("Stallguard ");
  // ADDRESS_SG_RESULT
  // A higher value signals a lower motor load and more torque headroom.
  uint16_t stall_guard_result = stepper_driver.getStallGuardResult();
  Serial.println(stall_guard_result);

  if((millis() - lastStallTime > 3000) && (stall_guard_result < 100)) { 
    dir = -dir;
    Serial.println("STALLED");
    lastStallTime = millis();
    digitalWrite(ledPin, HIGH);
    stepper_driver.moveAtVelocity(20000 * dir);
  }
  else {
    digitalWrite(ledPin, LOW);
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

  Serial.print("Starting scheduler...");
  ts.init();
  ts.addTask(stallguardPollTask);
  Serial.print("added pollTask...");
  stallguardPollTask.enable();
  Serial.println("pollTask enabled");

  stepper_driver.moveAtVelocity(20000 * dir);
}

void loop() {
  // Service task scheduler
  ts.execute();
}