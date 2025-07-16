#include "ZigbeeCoveringHelper.h"
#include <ZigbeeCore.h>
#include <ep/ZigbeeWindowCovering.h>
#include <ep/ZigbeeAnalog.h>
#include <Preferences.h>

static ZigbeeWindowCovering *zbCovering = nullptr;
static ZigbeeAnalog *zbAnalogStallSensitivity = nullptr;
static ZigbeeAnalog *zbAnalogBottomLimit = nullptr;
static ZigbeeAnalog *zbAnalogTopLimit = nullptr;
static ZigbeeAnalog *zbAnalogSpeed = nullptr;
static StepperUart *stepperMotor = nullptr;

static Preferences prefs;

static uint16_t BOTTOM_LIMIT = -1; // Bottom limit in cm
static uint16_t TOP_LIMIT = -1;    // Top limit in cm

// 2.0cm diameter spool, 200 steps per round x 8 microsteps x 4.666666666666:1 gear ratio (~ 1188)
const uint32_t STEPS_PER_CM = (200 * 8 * 4.667) / (2.0 * PI);

// Because the tension in the string is high when lifting the cover, we overshoot the target a bit
// and then move back down to the target position.
const uint32_t liftBackOff = 0.3 * STEPS_PER_CM; // how much we initially overshoot the target when moving up

static boolean flag_init = false;

void updatePosition(int32_t currentPosition)
{
    float currentLift = 1.0 * currentPosition / STEPS_PER_CM - TOP_LIMIT;
    float currentLiftPercentage = (currentLift * 100.0) / (1.0 * BOTTOM_LIMIT - TOP_LIMIT);

    prefs.begin("ZBCover");
    int32_t savedPosition = prefs.getInt("currentPosition", 0);
    prefs.end();

    if (savedPosition == currentPosition)
        return;
    
    prefs.putInt("currentPosition", static_cast<int32_t>(currentPosition));
    Serial.printf("Saved lift position: %d (%.2f%%).\n", currentPosition, currentLiftPercentage);

    if (!Zigbee.started() || zbCovering == nullptr)
        return;

    if (currentLiftPercentage > 100)
        zbCovering->setLiftPercentage(100);
    else if (currentLiftPercentage < 0)
        zbCovering->setLiftPercentage(0);
    else
        zbCovering->setLiftPercentage(currentLiftPercentage);
}

void waitForMotorToStop()
{
    if (stepperMotor != nullptr)
    {
        while (stepperMotor->isRunning())
        {
            vTaskDelay(10);
        }
    }
}

void homingRoutine()
{
    Serial.println("Homing routine started.");
    if (stepperMotor != nullptr)
    {
        stepperMotor->moveTo(stepperMotor->getCurrentPosition() + STEPS_PER_CM * -100);
        waitForMotorToStop();
        Serial.println("position after homing: " + String(stepperMotor->getCurrentPosition()));

        stepperMotor->moveTo(stepperMotor->getCurrentPosition() + STEPS_PER_CM * 2); // Move back a few cm
        waitForMotorToStop();

        stepperMotor->setCurrentPosition(0); // Set the current position to 0 after homing
        openCover();                         // Open the cover to the top limit

        Serial.println("Homing routine completed.");
    }
}

bool targetReached(int target)
{
    if (stepperMotor != nullptr)
    {
        while (stepperMotor->isRunning())
        {
            if (stepperMotor->getTargetPosition() != target)
            {
                return false;
            }
            vTaskDelay(50);
        }
    }
    if (stepperMotor->getCurrentPosition() == target)
    {
        return true;
    }
    return false;
}

void openCover()
{
    // Because we are moving up, create a new task where we move the motor, wait for it to stop and then back off a bit
    // to prevent tension in the string
    TaskFunction_t openCoverTask = [](void *)
    {
        if (stepperMotor != nullptr)
        {
            stepperMotor->moveTo(TOP_LIMIT * STEPS_PER_CM - liftBackOff); // Move to the top limit minus the lift back off
            if (targetReached(TOP_LIMIT * STEPS_PER_CM - liftBackOff))
            {
                stepperMotor->moveTo(TOP_LIMIT * STEPS_PER_CM + liftBackOff);
            }
        }
        updatePosition(stepperMotor->getCurrentPosition());
        vTaskDelete(NULL);
    };
    xTaskCreate(openCoverTask, "OpenCoverTask", 2048, nullptr, 1, nullptr);
}

void closeCover()
{
    if (&stepperMotor != nullptr)
        stepperMotor->moveTo(BOTTOM_LIMIT * STEPS_PER_CM);
}

static uint8_t stopCounter = 0;
static uint32_t lastStopTime = 0;
static bool flag_homing = false;

void stopCover()
{
    if (flag_homing)
    {
        stepperMotor->forceStop();
        return;
    }

    if (&stepperMotor != nullptr && stepperMotor->isRunning())
    {
        // If we are moving up, we release the tension by moving back down a bit
        if (stepperMotor->getTargetPosition() < stepperMotor->getCurrentPosition())
        {
            stepperMotor->moveTo(stepperMotor->getCurrentPosition() + liftBackOff);
            waitForMotorToStop();
        }
        stepperMotor->stop();
        updatePosition(stepperMotor->getCurrentPosition());
    }

    // If stop is called three times in quick succession, we start homing procedure
    if (millis() - lastStopTime < 500)
    {
        if (++stopCounter >= 2)
        {
            TaskFunction_t homingTask = [](void *)
            {
                flag_homing = true;
                homingRoutine();
                flag_homing = false;
                stopCounter = 0;
                vTaskDelete(NULL);
            };
            xTaskCreate(homingTask, "HomingTask", 2048, nullptr, 1, nullptr);
        }
    }
    else
    {
        stopCounter = 0; // Reset the counter if enough time has passed
    }
    lastStopTime = millis(); // Update the last stop time
}

void goToLiftPercentage(uint8_t liftPercentage)
{
    float newLift = (TOP_LIMIT * 1.0 + (liftPercentage * 1.0 * (BOTTOM_LIMIT - TOP_LIMIT) / 100.0));
    int32_t newPosition = static_cast<int32_t>(newLift * STEPS_PER_CM);
    Serial.printf("New requested lift from Zigbee: %.2f cm / %d (%d \%)\n", newLift, newPosition, liftPercentage);

    if (&stepperMotor == nullptr)
        return;

    if (newPosition < stepperMotor->getCurrentPosition())
    {
        stepperMotor->moveTo(newPosition - liftBackOff); // Move to the target minus the lift back off
        
        // Because we are moving up, create a new task where we wait for it to stop and then back off a bit
        // to prevent tension in the string
        TaskFunction_t backOffTask = [](void *pvParameters)
        {
            int32_t newPosition = stepperMotor->getTargetPosition();
            Serial.printf("Moving to position: %d\n", newPosition);
            if (stepperMotor != nullptr)
            {
                if (targetReached(newPosition))
                {
                    stepperMotor->moveTo(newPosition + liftBackOff);
                    if (targetReached(newPosition + liftBackOff))
                    {
                        updatePosition(stepperMotor->getCurrentPosition());
                    }
                }
            }
            vTaskDelete(NULL);
        };
        xTaskCreate(backOffTask, "backOffTask", 2048, nullptr, 1, nullptr);
    }
    else {
        // If we are moving down, we just move to the target position as we are already releasing the tension
        stepperMotor->moveTo(newPosition);
    }
}

void onBottomLimitChange(float analog)
{
    Serial.printf("Bottom limit set: %.2f cm\n", analog);
    prefs.begin("ZBCover");
    prefs.putUInt("bottomLimit", static_cast<uint16_t>(analog));
    prefs.end();
    BOTTOM_LIMIT = static_cast<uint16_t>(analog);

    if (flag_init && stepperMotor != nullptr)
    {
        Serial.printf("Moving to bottom lift: %d cm\n", BOTTOM_LIMIT);
        stepperMotor->moveTo(BOTTOM_LIMIT * STEPS_PER_CM);
    }
}

void onTopLimitChange(float analog)
{
    Serial.printf("Top limit set: %.2f cm\n", analog);
    prefs.begin("ZBCover");
    prefs.putUInt("topLimit", static_cast<uint16_t>(analog));
    prefs.end();
    TOP_LIMIT = static_cast<uint16_t>(analog);

    if (flag_init && stepperMotor != nullptr)
    {
        Serial.printf("Moving to top lift: %d cm\n", TOP_LIMIT);
        stepperMotor->moveTo(TOP_LIMIT * STEPS_PER_CM);
    }
}

void onSpeedChange(float analog)
{
    Serial.printf("Speed changed: %.2f\n", analog);
    prefs.begin("ZBCover");
    prefs.putFloat("speed", analog);
    prefs.end();

    if (flag_init && stepperMotor != nullptr)
    {
        stepperMotor->setSpeed(analog, 8);
    }
}

void onAnalogStallSensitivityChange(float analog)
{
    Serial.printf("Stall sensitivity changed: %.2f\n", analog);
    prefs.begin("ZBCover");
    prefs.putUInt("SGTHRS", static_cast<uint8_t>(analog));
    prefs.end();

    if (flag_init && stepperMotor != nullptr)
        stepperMotor->setSGTHRS(static_cast<uint8_t>(analog));
}

void createAndSetupZigbeeEndpoints()
{
    zbCovering = new ZigbeeWindowCovering(10);

    zbCovering->setManufacturerAndModel("sando@home", "WindowCoveringV3");
    zbCovering->setCoveringType(ZigbeeWindowCoveringType::ROLLERSHADE);
    zbCovering->setConfigStatus(true, true, false, false, false, false, false);
    zbCovering->setMode(false, false, false, false);
    zbCovering->setLimits(0, 100, 0, 0);

    zbCovering->onOpen(openCover);
    zbCovering->onClose(closeCover);
    zbCovering->onGoToLiftPercentage(goToLiftPercentage);
    zbCovering->onStop(stopCover);

    zbAnalogStallSensitivity = new ZigbeeAnalog(12);
    zbAnalogStallSensitivity->setManufacturerAndModel("sando@home", "WindowCoveringV3");
    zbAnalogStallSensitivity->addAnalogOutput();
    zbAnalogStallSensitivity->setAnalogOutputApplication(ESP_ZB_ZCL_AO_APP_TYPE_COUNT_UNITLESS);
    zbAnalogStallSensitivity->setAnalogOutputDescription("Stall sensitivity");
    zbAnalogStallSensitivity->setAnalogOutputResolution(1.0f);
    zbAnalogStallSensitivity->setAnalogOutputMinMax(0.0f, 144.0f); // Set min and max values for stall sensitivity
    zbAnalogStallSensitivity->onAnalogOutputChange(onAnalogStallSensitivityChange);

    zbAnalogBottomLimit = new ZigbeeAnalog(13);
    zbAnalogBottomLimit->setManufacturerAndModel("sando@home", "WindowCoveringV3");
    zbAnalogBottomLimit->addAnalogOutput();
    zbAnalogBottomLimit->setAnalogOutputApplication(ESP_ZB_ZCL_AO_APP_TYPE_COUNT_UNITLESS);
    zbAnalogBottomLimit->setAnalogOutputDescription("Max lift height in cm");
    zbAnalogBottomLimit->setAnalogOutputResolution(1.0f);
    zbAnalogBottomLimit->setAnalogOutputMinMax(0.0f, 400.0f); // Set min and max values for lift height
    zbAnalogBottomLimit->onAnalogOutputChange(onBottomLimitChange);

    zbAnalogTopLimit = new ZigbeeAnalog(14);
    zbAnalogTopLimit->setManufacturerAndModel("sando@home", "WindowCoveringV3");
    zbAnalogTopLimit->addAnalogOutput();
    zbAnalogTopLimit->setAnalogOutputApplication(ESP_ZB_ZCL_AO_APP_TYPE_COUNT_UNITLESS);
    zbAnalogTopLimit->setAnalogOutputDescription("Max lift height in cm");
    zbAnalogTopLimit->setAnalogOutputResolution(1.0f);
    zbAnalogTopLimit->setAnalogOutputMinMax(0.0f, 400.0f); // Set min and max values for lift height
    zbAnalogTopLimit->onAnalogOutputChange(onTopLimitChange);

    zbAnalogSpeed = new ZigbeeAnalog(15);
    zbAnalogSpeed->setManufacturerAndModel("sando@home", "WindowCoveringV3");
    zbAnalogSpeed->addAnalogOutput();
    zbAnalogSpeed->setAnalogOutputApplication(ESP_ZB_ZCL_AO_APP_TYPE_COUNT_UNITLESS);
    zbAnalogSpeed->setAnalogOutputDescription("Stepper speed");
    zbAnalogSpeed->setAnalogOutputResolution(1.0f);
    zbAnalogSpeed->setAnalogOutputMinMax(0.0f, 15000.0f); // Set min and max values for speed
    zbAnalogSpeed->onAnalogOutputChange(onSpeedChange);

    Zigbee.addEndpoint(zbCovering);
    Zigbee.addEndpoint(zbAnalogStallSensitivity);
    Zigbee.addEndpoint(zbAnalogBottomLimit);
    Zigbee.addEndpoint(zbAnalogTopLimit);
    Zigbee.addEndpoint(zbAnalogSpeed);
}

void readAndUpdateZigbeeCoverState(StepperUart &motor)
{
    prefs.begin("ZBCover");
    int32_t savedPosition = prefs.getInt("currentPosition", 0);
    uint8_t SGTHRS = prefs.getUInt("SGTHRS", 130);    // Default to 130 if not set
    BOTTOM_LIMIT = prefs.getUInt("bottomLimit", 100); // Default to 100cm if not set
    TOP_LIMIT = prefs.getUInt("topLimit", 10);        // Default to 10cm if not set
    float speed = prefs.getFloat("speed", 7500.0f);   // Default to 7500 steps/s if not set
    prefs.end();

    Serial.printf("Read and applied configs from prefs:\n");
    Serial.printf("saved position: %d\n", savedPosition);
    Serial.printf("stall sensitivity: %d\n", SGTHRS);
    Serial.printf("bottom limit: %d cm\n", BOTTOM_LIMIT);
    Serial.printf("top limit: %d cm\n", TOP_LIMIT);
    Serial.printf("speed: %.0f\n", speed);

    uint8_t savedLiftPercentage = (savedPosition / STEPS_PER_CM - TOP_LIMIT) * 100 / (BOTTOM_LIMIT - TOP_LIMIT);
    Serial.printf("Calculated lift percentage: %d\n", savedLiftPercentage);
    Serial.printf("Calculated lift in cm: %d\n", savedPosition / STEPS_PER_CM);

    stepperMotor = &motor;
    if (stepperMotor != nullptr)
    {
        stepperMotor->setCurrentPosition(savedPosition);
        stepperMotor->setSGTHRS(SGTHRS);
        stepperMotor->setSpeed(speed, 8);
    }
    if (Zigbee.started() && zbCovering != nullptr)
    {
        zbCovering->setLiftPercentage(savedLiftPercentage);
    }
    if (Zigbee.started() && zbAnalogStallSensitivity != nullptr)
    {
        zbAnalogStallSensitivity->setAnalogOutput(static_cast<float>(SGTHRS));
    }
    if (Zigbee.started() && zbAnalogBottomLimit != nullptr)
    {
        zbAnalogBottomLimit->setAnalogOutput(static_cast<float>(BOTTOM_LIMIT));
    }
    if (Zigbee.started() && zbAnalogTopLimit != nullptr)
    {
        zbAnalogTopLimit->setAnalogOutput(static_cast<float>(TOP_LIMIT));
    }
    if (Zigbee.started() && zbAnalogSpeed != nullptr)
    {
        zbAnalogSpeed->setAnalogOutput(speed);
    }

    flag_init = true;
}