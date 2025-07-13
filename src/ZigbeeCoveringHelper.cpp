#include "ZigbeeCoveringHelper.h"
#include <ZigbeeCore.h>
#include <ep/ZigbeeWindowCovering.h>
#include <ep/ZigbeeAnalog.h>
#include <Preferences.h>

static ZigbeeWindowCovering *zbCovering = nullptr;
static ZigbeeAnalog *zbAnalogMaxHeight = nullptr;
static ZigbeeAnalog *zbAnalogStallSensitivity = nullptr;
static StepperUart *stepperMotor = nullptr;

static Preferences prefs;

static uint16_t MAX_LIFT = -1; // Maximum lift in cm

// 2.0cm diameter spool, 200 steps per round x 8 microsteps x 4.666666666666:1 gear ratio
static uint16_t STEPS_PER_CM = (200 * 8 * 4.667) / (2.0 * PI);

static boolean flag_init = false;

void updatePosition(int currentPosition)
{
    uint16_t currentLift = currentPosition / STEPS_PER_CM;
    uint8_t currentLiftPercentage = (currentLift * 100) / MAX_LIFT;

    prefs.begin("ZBCover");
    {
        uint16_t savedPosition = prefs.getUInt("currentPosition", 0);
        if (savedPosition != currentPosition)
        {
            prefs.putUInt("currentPosition", currentPosition);
            Serial.printf("Saved lift position: %d (%d%%).\n", currentPosition, currentLiftPercentage);
        }
    }
    prefs.end();

    if (!Zigbee.started() || zbCovering == nullptr)
        return;

    const uint8_t safeLiftPercentage = currentLiftPercentage > 100 ? 100 : currentLiftPercentage < 0 ? 0 : currentLiftPercentage;
    zbCovering->setLiftPercentage(safeLiftPercentage);
}

void HomingRoutine()
{
    Serial.println("Homing routine started.");
    if (stepperMotor != nullptr)
    {
        stepperMotor->moveTo(stepperMotor->getCurrentPosition() + STEPS_PER_CM * -10); // Move 10 cm up
        while (stepperMotor->isRunning())
        {
            vTaskDelay(100);
        }
        Serial.println("position after homing: " + String(stepperMotor->getCurrentPosition()));

        stepperMotor->moveTo(stepperMotor->getCurrentPosition() + STEPS_PER_CM * 2); // Move back a few cm
        while (stepperMotor->isRunning())
        {
            vTaskDelay(100);
        }

        stepperMotor->setCurrentPosition(0); // Set the current position to 0 after homing
        updatePosition(0); // Update the position in Zigbee and prefs

        Serial.println("Homing routine completed.");
    }
}

void openCover()
{
    if (&stepperMotor != nullptr)
        stepperMotor->moveTo(0);
}

void closeCover()
{
    if (&stepperMotor != nullptr)
        stepperMotor->moveTo(MAX_LIFT * STEPS_PER_CM);
}

static uint8_t stopCounter = 0;
static uint32_t lastStopTime = 0;

void stopCover()
{
    if (&stepperMotor != nullptr)
        stepperMotor->stop();
    
    // If stop is called three times in quick succession, we start homing procedure
    if (millis() - lastStopTime < 500)
    {
        stopCounter++;
        if (stopCounter >= 3)
        {
            HomingRoutine();
            stopCounter = 0; // Reset the counter after homing
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
    uint16_t newLift = (liftPercentage * MAX_LIFT) / 100;
    Serial.printf("New requested lift from Zigbee: %d (%d)\n", newLift, liftPercentage);

    if (&stepperMotor != nullptr)
        stepperMotor->moveTo(newLift * STEPS_PER_CM);
}

void onAnalogMaxHeightChange(float analog)
{
    Serial.printf("Max height set: %.2f cm\n", analog);
    prefs.begin("ZBCover");
    prefs.putUInt("maxLift", static_cast<uint16_t>(analog));
    prefs.end();
    MAX_LIFT = static_cast<uint16_t>(analog);

    if (flag_init && stepperMotor != nullptr)
    {
        Serial.printf("Moving to max lift: %d cm\n", MAX_LIFT);
        stepperMotor->moveTo(MAX_LIFT * STEPS_PER_CM);
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

void createAndSetupZigbeeEndpoint(uint8_t endpoint)
{
    zbCovering = new ZigbeeWindowCovering(endpoint);

    zbCovering->setManufacturerAndModel("sando@home", "WindowCoveringV3");
    zbCovering->setCoveringType(ZigbeeWindowCoveringType::ROLLERSHADE);
    zbCovering->setConfigStatus(true, true, false, false, false, false, false);
    zbCovering->setMode(false, false, false, false);
    zbCovering->setLimits(0, 100, 0, 0);

    zbCovering->onOpen(openCover);
    zbCovering->onClose(closeCover);
    zbCovering->onGoToLiftPercentage(goToLiftPercentage);
    zbCovering->onStop(stopCover);

    zbAnalogMaxHeight = new ZigbeeAnalog(endpoint + 1);
    zbAnalogMaxHeight->setManufacturerAndModel("sando@home", "WindowCoveringV3");
    zbAnalogMaxHeight->addAnalogOutput();
    zbAnalogMaxHeight->setAnalogOutputApplication(ESP_ZB_ZCL_AO_APP_TYPE_COUNT_UNITLESS);
    zbAnalogMaxHeight->setAnalogOutputDescription("Max lift height in cm");
    zbAnalogMaxHeight->setAnalogOutputResolution(1.0f);
    zbAnalogMaxHeight->setAnalogOutputMinMax(0.0f, 400.0f); // Set min and max values for lift height
    zbAnalogMaxHeight->onAnalogOutputChange(onAnalogMaxHeightChange);

    zbAnalogStallSensitivity = new ZigbeeAnalog(endpoint + 2);
    zbAnalogStallSensitivity->setManufacturerAndModel("sando@home", "WindowCoveringV3");
    zbAnalogStallSensitivity->addAnalogOutput();
    zbAnalogStallSensitivity->setAnalogOutputApplication(ESP_ZB_ZCL_AO_APP_TYPE_COUNT_UNITLESS);
    zbAnalogStallSensitivity->setAnalogOutputDescription("Stall sensitivity");
    zbAnalogStallSensitivity->setAnalogOutputResolution(1.0f);
    zbAnalogStallSensitivity->setAnalogOutputMinMax(0.0f, 144.0f); // Set min and max values for stall sensitivity
    zbAnalogStallSensitivity->onAnalogOutputChange(onAnalogStallSensitivityChange);

    Zigbee.addEndpoint(zbCovering);
    Zigbee.addEndpoint(zbAnalogMaxHeight);
    Zigbee.addEndpoint(zbAnalogStallSensitivity);
}

void readAndUpdateZigbeeCoverState(StepperUart &motor)
{
    prefs.begin("ZBCover");
    uint16_t savedPosition = prefs.getUInt("currentPosition", 0);
    MAX_LIFT = prefs.getUInt("maxLift", 100);      // Default to 100 if not set
    uint8_t SGTHRS = prefs.getUInt("SGTHRS", 130); // Default to 130 if not set
    prefs.end();

    Serial.printf("Read and applied configs from prefs:\n");
    Serial.printf("max lift: %d\n", MAX_LIFT);
    Serial.printf("saved position: %d\n", savedPosition);
    Serial.printf("stall sensitivity: %d\n", SGTHRS);

    uint8_t savedLiftPercentage = savedPosition / STEPS_PER_CM * 100 / MAX_LIFT;
    Serial.printf("Calculated lift percentage: %d\n", savedLiftPercentage);
    Serial.printf("Calculated lift in cm: %d\n", savedPosition / STEPS_PER_CM);

    stepperMotor = &motor;
    if (stepperMotor != nullptr)
    {
        stepperMotor->setCurrentPosition(savedPosition);
        stepperMotor->setSGTHRS(SGTHRS);
    }
    if (Zigbee.started() && zbCovering != nullptr)
    {
        zbCovering->setLiftPercentage(savedLiftPercentage);
    }
    if (Zigbee.started() && zbAnalogMaxHeight != nullptr)
    {
        zbAnalogMaxHeight->setAnalogOutput(static_cast<float>(MAX_LIFT));
    }
    if (Zigbee.started() && zbAnalogStallSensitivity != nullptr)
    {
        zbAnalogStallSensitivity->setAnalogOutput(static_cast<float>(SGTHRS));
    }

    flag_init = true; // initial setup is done
}