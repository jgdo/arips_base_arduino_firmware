#pragma once

void steppersInit();
void steppersEnable();
void steppersDisable();
bool steppersAreEnabled();
void steppersSetSpeed(float left_mps, float right_mps);
void steppersSetSpeedSteps(float leftSteps_ps, float rightSteps_ps);
void stepperCheckTimeout();
