#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not set."
#endif

#include <Arduino.h>
#include "ZigbeeCore.h"
#include "StepperUart.h"
#include "ZigbeeCoveringHelper.h"

#define ZIGBEE_COVERING_ENDPOINT 10
#define BUTTON_PIN 9 // ESP32-C6/H2 Boot button

#define MOTOR_DIR_PIN 18
#define MOTOR_STEP_PIN 20
#define MOTOR_ENABLE_PIN 23

StepperUart stepperMotor(MOTOR_DIR_PIN, MOTOR_STEP_PIN, MOTOR_ENABLE_PIN);

void blink(uint8_t count)
{
  for (uint8_t i = 0; i < count; i++)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(BUTTON_PIN, INPUT_PULLUP); // Init button for factory reset
  pinMode(LED_BUILTIN, OUTPUT);      // Init LED pin
  digitalWrite(LED_BUILTIN, LOW);    // Turn on LED

  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  digitalWrite(MOTOR_ENABLE_PIN, HIGH); // Disable motor during setup

  // Wait a while for serial monitor to connect
  delay(2000);

  createAndSetupZigbeeEndpoint(10);

#ifndef ZIGBEE_DISABLED
  Serial.println("Calling Zigbee.begin()");
  blink(7);

  if (!Zigbee.begin())
  {
    Serial.println("Zigbee failed to start!");
    return;
  }

  Serial.println("Connecting to network");
  while (!Zigbee.connected())
  {
    Serial.print(".");
    delay(100);
  }
  Serial.println("Connected!");
#endif

  stepperMotor.init(); 
  readAndUpdateZigbeeCoverState(stepperMotor);
  stepperMotor.setPositionUpdateCallback(updatePosition);

  // Blink LED to indicate that the device is ready
  blink(3);
}

static unsigned long buttonPressTime = 0;
void loop()
{
  // Check if the button is pressed for manual open/close
  if (digitalRead(BUTTON_PIN) == LOW)
  {
    if (buttonPressTime == 0)
    {
      buttonPressTime = millis();
    }
    else if (millis() - buttonPressTime > 200)
    {
      buttonPressTime = 0;

      if (stepperMotor.getCurrentPosition() > 0)
      {
        Serial.println("Closing cover.");
        closeCover();
      }
      else
      {
        Serial.println("Opening cover.");
        openCover();
      }
      vTaskDelay(portTICK_PERIOD_MS * 400);
    }
  }
  else
  {
    buttonPressTime = 0;
  }

  switch (Serial.read())
  {
  case 'o':
    openCover();
    break;
  case 'c':
    closeCover();
    break;
  case 's':
    stopCover();
    break;
  case 'h':
    HomingRoutine();
    break;
  case '1':
    stepperMotor.setSpeed(1000, 8);
    blink(1);
    break;
  case '2':
    stepperMotor.setSpeed(2400, 8);
    blink(2);
    break;
  case '3':
    stepperMotor.setSpeed(5000, 8);
    blink(3);
    break;
  case '4':
    stepperMotor.setSpeed(7500, 8);
    blink(3);
    break;
  case '5':
    stepperMotor.setSpeed(10000, 8);
    blink(3);
    break;

  case '+':
    {
      uint8_t currentSGTHRS = stepperMotor.getSGTHRS();
      if (currentSGTHRS < 144)
      {
        stepperMotor.setSGTHRS(currentSGTHRS + 10);
        Serial.printf("Increased SGTHRS to: %d\n", currentSGTHRS + 10);
      }
      else
      {
        Serial.println("SGTHRS is already at maximum (144).");
      }
    }
    break;
  
  case '-':
    {
      uint8_t currentSGTHRS = stepperMotor.getSGTHRS();
      if (currentSGTHRS > 0)
      {
        stepperMotor.setSGTHRS(currentSGTHRS - 10);
        Serial.printf("Decreased SGTHRS to: %d\n", currentSGTHRS - 10);
      }
      else
      {
        Serial.println("SGTHRS is already at minimum (0).");
      }
    }
  }
  vTaskDelay(portTICK_PERIOD_MS * 50); // Yield to other tasks
}
