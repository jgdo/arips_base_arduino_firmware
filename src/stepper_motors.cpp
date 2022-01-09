#include <stepper_motors.h>

#include <Arduino.h>

static constexpr auto DUE_CLOCK_FREQ = 84000000;

static constexpr auto PIN_M0_ENABLE = 7;
static constexpr auto PIN_M0_DIR = 6;
static constexpr auto PIN_M1_ENABLE = 4;
static constexpr auto PIN_M1_DIR = 2;

static uint32_t lastSteppersTime_ms = 0;
static constexpr uint32_t STEPPERS_TIMEOUT_MS = 500;

void steppersInit()
{
    pinMode(PIN_M0_ENABLE, OUTPUT);
    pinMode(PIN_M0_DIR, OUTPUT);
    pinMode(PIN_M1_ENABLE, OUTPUT);
    pinMode(PIN_M1_DIR, OUTPUT);

    steppersDisable();

    // see https://forum.arduino.cc/t/arduino-due-set-pwm-frequency-change/516496/3

    PMC->PMC_PCER1 |= PMC_PCER1_PID33; // TC7 power ON - Timer Counter 2 channel 1 IS TC7 - See page 38
    PIOC->PIO_PDR |= PIO_PDR_P25;      // The pin is no more driven by GPIO
    PIOC->PIO_ABSR |= PIO_PC25B_TIOA6; // Periperal type B  - See page 859

    TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 // MCK/2, clk on rising edge
                                | TC_CMR_WAVE              // Waveform mode
                                | TC_CMR_WAVSEL_UP_RC      // UP mode with automatic trigger on RC Compare
                                | TC_CMR_ACPA_CLEAR        // Clear TIOA7 on RA compare match  -- See page 883
                                | TC_CMR_ACPC_SET;         // Set TIOA7 on RC compare match

    TC2->TC_CHANNEL[0].TC_IER = TC_IER_CPCS; // Interrupt on RC compare match
    NVIC_EnableIRQ(TC6_IRQn);

    PMC->PMC_PCER1 |= PMC_PCER1_PID34; // TC7 power ON - Timer Counter 2 channel 1 IS TC7 - See page 38
    PIOC->PIO_PDR |= PIO_PDR_P28;      // The pin is no more driven by GPIO
    PIOC->PIO_ABSR |= PIO_PC28B_TIOA7; // Periperal type B  - See page 859

    TC2->TC_CHANNEL[1].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 // MCK/2, clk on rising edge
                                | TC_CMR_WAVE              // Waveform mode
                                | TC_CMR_WAVSEL_UP_RC      // UP mode with automatic trigger on RC Compare
                                | TC_CMR_ACPA_CLEAR        // Clear TIOA7 on RA compare match  -- See page 883
                                | TC_CMR_ACPC_SET;         // Set TIOA7 on RC compare match

    TC2->TC_CHANNEL[1].TC_IER = TC_IER_CPCS; // Interrupt on RC compare match
    NVIC_EnableIRQ(TC7_IRQn);
}

void steppersEnable()
{
    digitalWrite(PIN_M0_ENABLE, 0);
    digitalWrite(PIN_M1_ENABLE, 0);
}

void steppersDisable()
{
    TC2->TC_CHANNEL[0].TC_CCR = 0;
    TC2->TC_CHANNEL[1].TC_CCR = 0;

    digitalWrite(PIN_M0_ENABLE, 1);
    digitalWrite(PIN_M1_ENABLE, 1);
}

bool steppersAreEnabled() {
    return !digitalRead(PIN_M0_ENABLE) || !digitalRead(PIN_M1_ENABLE);
}

void steppersSetSpeed(float left_mps, float right_mps)
{
    auto calcSteps_ps = [](float vel_mps)
    {
        static constexpr auto GEAR_RATIO = 37.0F / 13.0F;
        static constexpr auto STEPS_PER_ROUND = 800;
        static constexpr auto WHEEL_DIAM = 0.12F;

        return vel_mps / (WHEEL_DIAM * static_cast<float>(PI)) * GEAR_RATIO * STEPS_PER_ROUND;
    };

    steppersSetSpeedSteps(calcSteps_ps(left_mps), calcSteps_ps(right_mps));
}

void steppersSetSpeedSteps(float leftSteps_ps, float rightSteps_ps)
{
    static constexpr auto MIN_STEPS_PS = 1.0F;
    static constexpr auto RIGHT_MOTOR_SIGN = 1.0F;
    static constexpr auto LEFT_MOTOR_SIGN = -1.0F;

    // reverse if necessary
    leftSteps_ps *= LEFT_MOTOR_SIGN;
    rightSteps_ps *= RIGHT_MOTOR_SIGN;

    // disable timers
    TC2->TC_CHANNEL[0].TC_CCR = 0;
    TC2->TC_CHANNEL[1].TC_CCR = 0;

    // turn off motors if speed close to zero
    if(abs(leftSteps_ps) < MIN_STEPS_PS || abs(rightSteps_ps) < MIN_STEPS_PS) {
        steppersDisable();
        return;
    }

    lastSteppersTime_ms = millis();

    auto calcRC = [](float steps_ps) -> int
    {
        return DUE_CLOCK_FREQ / 2 / steps_ps;
    };

    const int leftRC = calcRC(leftSteps_ps);
    const int rightRC = calcRC(rightSteps_ps);

    TC2->TC_CHANNEL[0].TC_RC = abs(leftRC);     //<*********************  Frequency = (Mck/2)/TC_RC
    TC2->TC_CHANNEL[0].TC_RA = abs(leftRC / 2); //<********************   Duty cycle = (TC_RA/TC_RC) * 100  %
    TC2->TC_CHANNEL[0].TC_CV = 0;

    TC2->TC_CHANNEL[1].TC_RC = abs(rightRC);     //<*********************  Frequency = (Mck/2)/TC_RC
    TC2->TC_CHANNEL[1].TC_RA = abs(rightRC / 2); //<********************   Duty cycle = (TC_RA/TC_RC) * 100  %
    TC2->TC_CHANNEL[1].TC_CV = 0;

    digitalWrite(PIN_M0_DIR, leftRC > 0);
    digitalWrite(PIN_M1_DIR, rightRC > 0);
    steppersEnable();

    // enable timers
    TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;
    TC2->TC_CHANNEL[1].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN;
}

void stepperCheckTimeout()
{
    if(steppersAreEnabled()) {
        const auto millisElapsed = millis() - lastSteppersTime_ms;

        if (millisElapsed >= STEPPERS_TIMEOUT_MS)
        {
            steppersDisable();
        }
    }
}

void TC6_Handler()
{
    TC2->TC_CHANNEL[0].TC_SR;
}

void TC7_Handler()
{
    TC2->TC_CHANNEL[1].TC_SR;
}
