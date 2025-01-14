#include <Arduino.h>

#define MOTOR_STEPS       200 // Steps per revolution
#define MOTOR_MICRO_STEPS 2   // Microsteps
#define MOTOR_GEAR_RATIO  20  // Gear ratio

#define MOTOR_RPM         8.5 // RPM after gear
#define MOTOR_DIRECTION   LOW // HIGH = forward, LOW = backward

#define MOTOR_ACCEL       1000 // steps per seconds-squared
#define MOTOR_DECEL       1000 // steps per seconds-squared

#define MOTOR_PUL_PIN     9
#define MOTOR_DIR_PIN     8

#define MIN_MOTOR_SPEED   10 // Steps per second
#define MIN_YIELD_MICROS  50 // Don't call yield if we have a wait shorter than this
#define PULSE_DUR_MICROS  20 // Pulse HIGH/LOW duration

// Test time in seconds
#define TEST_TIME_SEC     10

static inline void delayMicros(unsigned long delay_us, unsigned long start_us = 0)
{
    if (delay_us)
    {
        if (!start_us)
        {
            start_us = micros();
        }
        if (delay_us > MIN_YIELD_MICROS)
        {
            yield();
        }
        // See https://www.gammon.com.au/millis
        while (micros() - start_us < delay_us)
            ;
    }
}

void setup()
{
    pinMode(MOTOR_PUL_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);

    digitalWrite(MOTOR_PUL_PIN, LOW);
    digitalWrite(MOTOR_DIR_PIN, MOTOR_DIRECTION);
}

void loop()
{
    double steps_per_min = (double)MOTOR_STEPS * MOTOR_MICRO_STEPS * MOTOR_GEAR_RATIO * MOTOR_RPM; // 68000 steps per minute
    double steps_per_sec = steps_per_min / 60.;                                                    // 1133.33 steps per second
    double motor_speed   = MIN_MOTOR_SPEED;                                                        // steps per second

    // Accelerate
    while (motor_speed < steps_per_sec)
    {
        // Pulse
        digitalWrite(MOTOR_PUL_PIN, HIGH);
        delayMicros(PULSE_DUR_MICROS);
        digitalWrite(MOTOR_PUL_PIN, LOW);
        delayMicros(PULSE_DUR_MICROS);

        // Wait for end of step
        unsigned long step_end_time = (unsigned long)(1000000. / motor_speed + 0.5);
        if (step_end_time > PULSE_DUR_MICROS * 2)
            delayMicros(step_end_time - PULSE_DUR_MICROS * 2);

        // Calculate new motor speed
        motor_speed += (double)MOTOR_ACCEL / motor_speed;
        if (motor_speed > steps_per_sec)
            motor_speed = steps_per_sec;
    }

    // Cruise
    unsigned long step_cnt     = 0;
    unsigned long cruise_start = micros();
    while (micros() - cruise_start < 1000000L * TEST_TIME_SEC)
    {
        // Pulse
        digitalWrite(MOTOR_PUL_PIN, HIGH);
        delayMicros(PULSE_DUR_MICROS);
        digitalWrite(MOTOR_PUL_PIN, LOW);
        delayMicros(PULSE_DUR_MICROS);

        // Wait for end of step
        step_cnt++;
        unsigned long step_end_time = (unsigned long)((double)step_cnt / steps_per_sec * 1000000. + 0.5);
        unsigned long current_time  = micros() - cruise_start;
        if (current_time < step_end_time)
            delayMicros(step_end_time - current_time);
    }

    // Decelerate
    while (motor_speed > MIN_MOTOR_SPEED)
    {
        // Pulse
        digitalWrite(MOTOR_PUL_PIN, HIGH);
        delayMicros(PULSE_DUR_MICROS);
        digitalWrite(MOTOR_PUL_PIN, LOW);
        delayMicros(PULSE_DUR_MICROS);

        // Calculate new motor speed
        motor_speed -= (double)MOTOR_DECEL / motor_speed;
        if (motor_speed < MIN_MOTOR_SPEED)
            motor_speed = MIN_MOTOR_SPEED;

        // Wait for end of step
        unsigned long step_end_time = (unsigned long)(1000000. / motor_speed + 0.5);
        if (step_end_time > PULSE_DUR_MICROS * 2)
            delayMicros(step_end_time - PULSE_DUR_MICROS * 2);
    }

    // Wait 5 seconds
    delay(5000);
}
