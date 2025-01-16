#include <TimerOne.h>

#define MOTOR_STEPS       200 // Full steps per revolution
#define MOTOR_MICRO_STEPS 2   // Microsteps
#define MOTOR_GEAR_RATIO  20  // Gear ratio

#define MOTOR_RPM         8.5 // RPM after gear
#define MOTOR_DIRECTION   LOW // HIGH = forward, LOW = backward
#define MOTOR_PWM_DUTY    512 // 50% duty cycle

#define MOTOR_ACCEL       1500      // steps per seconds-squared
#define MOTOR_DECEL       1500      // steps per seconds-squared
#define MOTOR_MIN_SPEED   1000000UL // Minimum step time in micro seconds

#define MOTOR_DIR_PIN     8
#define MOTOR_PUL_PIN     9

// Test time in seconds
#define TEST_TIME_SEC     60

volatile unsigned long step_count = 0; // use volatile for shared variables

void stepCounter()
{
    step_count++;
}

void setup()
{
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    digitalWrite(MOTOR_DIR_PIN, MOTOR_DIRECTION);

    Timer1.initialize(MOTOR_MIN_SPEED);
    Timer1.pwm(MOTOR_PUL_PIN, MOTOR_PWM_DUTY); // 50% duty
    Timer1.attachInterrupt(stepCounter);
    Timer1.stop();

    Serial.begin(9600);
}

void loop()
{
    // Delay 5 seconds
    Serial.println("Loop start.");
    delay(5000);

    // Calculate step period
    double steps_per_min = (double)MOTOR_STEPS * MOTOR_MICRO_STEPS * MOTOR_GEAR_RATIO * MOTOR_RPM; // 68000 steps per minute
    unsigned long step_period = (unsigned long)(60000000. / steps_per_min + 0.5);

    // Start timer
    Serial.println("Starting timer..");
    static unsigned long time_diff;
    static unsigned long step_count_copy;
    time_diff       = micros();
    step_count_copy = 0;
    noInterrupts();
    step_count = 0;
    interrupts();
    Timer1.setPeriod(step_period);
    Timer1.setPwmDuty(MOTOR_PUL_PIN, MOTOR_PWM_DUTY);

    // Wait till counter reached 1 minute
    while (true)
    {
        noInterrupts();
        step_count_copy = step_count;
        interrupts();
        if (step_count_copy >= (unsigned long)(steps_per_min / 60. * TEST_TIME_SEC))
            break;
        delay(1);
    }

    // Stop timer
    Timer1.stop();
    time_diff = micros() - time_diff;
    Serial.print("Stopped timer, step count: ");
    Serial.println(step_count_copy);
    Serial.print("Time difference: ");
    Serial.println(time_diff);
}
