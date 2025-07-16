#include <StepperUart.h>

int lastUpdate = -1;

StepperUart::StepperUart(int dirPin, int stepPin, int enablePin)
    : engine(), enablePin(enablePin), stepPin(stepPin), dirPin(dirPin), targetPosition(0), speed(5000), motorEnabled(false), positionUpdateCallback(nullptr)
{
    engine.init();
}

void vUpdatePositionTask(TimerHandle_t xTimer)
{
    StepperUart *stepper = static_cast<StepperUart *>(pvTimerGetTimerID(xTimer));
    int32_t currentPosition = stepper->getCurrentPosition();

    if (currentPosition == lastUpdate)
    {
        return;
    }

    lastUpdate = currentPosition;

    if (stepper->positionUpdateCallback)
    {
        stepper->positionUpdateCallback(currentPosition);
    }
}

void vStallDetectTask(TimerHandle_t xTimer)
{
    StepperUart *stepper = static_cast<StepperUart *>(pvTimerGetTimerID(xTimer));

    TMC2209Stepper driver = stepper->getDriver();
    if (driver.diag())
    {
        // Serial.println("Stall detected!");
        // Serial.printf("TSTEP: %d, SG: %d\n", driver.TSTEP(), driver.SG_RESULT());
        stepper->forceStop();
        driver.SG_RESULT(); // Read StallGuard value to clear the flag
    }
    // else if (stepper->isRunning())
    //     Serial.printf("TSTEP: %d, SG: %d, SGTHRS: %d\n", driver.TSTEP(), driver.SG_RESULT(), driver.SGTHRS());

}

void StepperUart::init()
{
    pinMode(enablePin, OUTPUT);
    disableMotor(); // Disable the motor initially

    HWSerial.begin(115200);

    _stepper = engine.stepperConnectToPin(stepPin);
    _stepper->setDirectionPin(dirPin);
    _stepper->setEnablePin(enablePin, true);
    _stepper->setAutoEnable(true);
    _stepper->setDelayToDisable(1000);
    _stepper->setSpeedInHz(speed);
    _stepper->setAcceleration(1e6);

    driver.begin();           // SPI: Init CS pins and possible SW SPI pins
    driver.rms_current(1500); // Set motor RMS current

    setSpeed(speed, 8); // Set speed and microsteps

    driver.pwm_autoscale(true); // Needed for stealthChop
    driver.pwm_autograd(true);
    driver.en_spreadCycle(false); // false = StealthChop / true = SpreadCycle
    // driver.semin(0); // disable coolstep

    TimerHandle_t updateTimer = xTimerCreate(
        "UpdateTask",        // Timer name
        pdMS_TO_TICKS(1000),  // Timer interval
        pdTRUE,              // Auto-reload
        this,                // pass the stepper instance to the task
        vUpdatePositionTask  // Callback function
    );
    xTimerStart(updateTimer, 0); // Start the timer

    TimerHandle_t stallTask = xTimerCreate(
        "StepperTask",     // Timer name
        pdMS_TO_TICKS(5), // Timer interval
        pdTRUE,            // Auto-reload
        this,              // pass the stepper instance to the task
        vStallDetectTask   // Callback function
    );
    xTimerStart(stallTask, 0); // Start the timer
}

void StepperUart::setSpeed(float speed, int microsteps)
{
    driver.microsteps(microsteps);

    // Calculate TSTEP based on speed and microsteps, 12MHz clock
    // TSTEP = (12MHz / (speed * microsteps))
    int tstep = (12000000 / (speed / microsteps * 256)) + 1;
    driver.TCOOLTHRS(tstep * 1.3); // Set TCOOLTHRS to slighty above nominal speed to disable stallguard and coolstep when decelerating
    printf("TSTEP: %d, TCOOLTHRS: %d\n", tstep, driver.TCOOLTHRS());

    this->speed = speed;
    _stepper->setSpeedInHz(speed);
}
void StepperUart::setSGTHRS(uint8_t threshold)
{
    driver.SGTHRS(threshold);
    Serial.printf("SGTHRS set to: %d\n", threshold);
}
void StepperUart::moveTo(int32_t position)
{
    targetPosition = position;
    _stepper->moveTo(position);
}
void StepperUart::stop()
{
    _stepper->stopMove();
}
void StepperUart::forceStop()
{
    _stepper->forceStop();
}
int32_t StepperUart::getCurrentPosition()
{
    return _stepper->getCurrentPosition();
}
void StepperUart::setCurrentPosition(int32_t position)
{
    _stepper->setCurrentPosition(position);
    lastUpdate = position;
    if (this->positionUpdateCallback)
    {
        this->positionUpdateCallback(position);
    }
}
bool StepperUart::isRunning()
{
    return _stepper->isRunning();
}