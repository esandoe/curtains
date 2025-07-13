#pragma once

#include <Arduino.h>
#include "StepperUart.h"

void updatePosition(int currentPosition);

void openCover();
void closeCover();
void stopCover();
void goToLiftPercentage(uint8_t liftPercentage);
void HomingRoutine();

void createAndSetupZigbeeEndpoint(uint8_t endpoint = 10);
void readAndUpdateZigbeeCoverState(StepperUart &motor);
