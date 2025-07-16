#pragma once

#include <Arduino.h>
#include "StepperUart.h"

void updatePosition(int32_t currentPosition);

void openCover();
void closeCover();
void stopCover();
void goToLiftPercentage(uint8_t liftPercentage);
void homingRoutine();

void createAndSetupZigbeeEndpoints();
void readAndUpdateZigbeeCoverState(StepperUart &motor);
