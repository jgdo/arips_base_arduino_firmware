#pragma once

#include <cstdint>

void batteryInit();
void batterySet(bool on);
void batterySetWithTimeoutMs(uint32_t ms);
bool batteryGetState();
void batteryCheckTimeout();
float batteryReadVoltage();
