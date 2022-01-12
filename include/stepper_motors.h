#pragma once

// #include <utility>

void steppersInit();
void steppersEnable();
void steppersDisable();
bool steppersAreEnabled();
void steppersSetSpeed(float left_mps, float right_mps);
void steppersSetSpeedSteps(float leftSteps_ps, float rightSteps_ps);
void stepperCheckTimeout();

void steppersGetDist(double& left, double& right);
