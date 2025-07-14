#pragma once

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <HardwareSerial.h>

#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver

class StepperUart
{
public:
    StepperUart(int dirPin, int stepPin, int enablePin);
    void init();
    void setSpeed(float speed, int microsteps);
    void setSGTHRS(uint8_t threshold);
    uint8_t getSGTHRS()
    {
        return driver.SGTHRS();
    }
    void moveTo(int position);
    int getTargetPosition()
    {
        return targetPosition;
    }
    void stop();
    int getCurrentPosition();
    void setCurrentPosition(int position);
    void setPositionUpdateCallback(void (*callback)(int))
    {
        positionUpdateCallback = callback;
    }
    bool isRunning();

    void (*positionUpdateCallback)(int) = nullptr;

    void enableMotor()
    {
        digitalWrite(enablePin, LOW); // Enable the motor
        motorEnabled = true;
    }
    void disableMotor()
    {
        digitalWrite(enablePin, HIGH); // Disable the motor
        motorEnabled = false;
    }
    TMC2209Stepper getDriver()
    {
        return driver;
    }
    void forceStop();

private:
    FastAccelStepperEngine engine = FastAccelStepperEngine();
    FastAccelStepper *_stepper;

    HardwareSerial HWSerial = HardwareSerial(0);
    TMC2209Stepper driver = TMC2209Stepper(&HWSerial, R_SENSE, DRIVER_ADDRESS);

    int targetPosition;
    float speed;
    int enablePin;
    int dirPin;
    int stepPin;
    bool motorEnabled;
};
