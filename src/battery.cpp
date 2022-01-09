#include <battery.h>

#include <Arduino.h>

static constexpr auto PIN_BATTERY_SW = 8;
static constexpr auto PIN_BATTERY_VOLTAGE = A0;

static uint32_t batteryTimeout_ms = 0;
static uint32_t lastBatteryTime_ms = 0;

void batteryInit()
{
    digitalWrite(PIN_BATTERY_SW, 0);
    pinMode(PIN_BATTERY_SW, OUTPUT);
}

void batterySet(bool on)
{
    digitalWrite(PIN_BATTERY_SW, on);
    lastBatteryTime_ms = millis();
}

void batterySetWithTimeoutMs(uint32_t ms) {
    batteryTimeout_ms = ms;
    batterySet(ms > 0);
}

bool batteryGetState()
{
    return digitalRead(PIN_BATTERY_SW);
}

void batteryCheckTimeout()
{
    if(batteryGetState()) {
        const auto millisElapsed = millis() - lastBatteryTime_ms;

        if (millisElapsed >= batteryTimeout_ms)
        {
            batterySet(0);
        }
    }
}

float batteryReadVoltage()
{
    static constexpr auto BATTERY_VOLTAGE_ADC_FACTOR = 3.3F * 4.0F / 4096.0F;
    return analogRead(PIN_BATTERY_VOLTAGE) * BATTERY_VOLTAGE_ADC_FACTOR;
}
