#include <TimerOne.h>

#define MOTOR_STEPS       200    // Full steps per revolution
#define MOTOR_MICRO_STEPS 2      // Microsteps (2 * 200 = 400)
#define MOTOR_GEAR_RATIO  20     // Gear ratio
#define MOTOR_RPM         8.5    // RPM after gear (20 * 8.5 = 170)
#define MOTOR_DIRECTION   LOW    // HIGH = forward, LOW = backward
#define MOTOR_PWM_DUTY    512    // 50% duty cycle, max 1024
#define MOTOR_MAX_PERIOD  150000 // Min motor RPM = 1 (before gear)
#define MOTOR_MIN_PERIOD  150    // Max motor RPM = 1000 (before gear)
#define MOTOR_ACCEL       1.0    // Acceleration, deceleration

#define MOTOR_DIR_PIN     8
#define MOTOR_PUL_PIN     9

// Test time in seconds
#define TEST_TIME_SEC     60

// Motor step count
volatile unsigned long motor_step_count = 0; // use volatile for shared variables

static void stepCounterISR()
{
    motor_step_count++;
}

static void setMotorStepCount(unsigned long step_count)
{
    noInterrupts();
    motor_step_count = step_count;
    interrupts();
}

static unsigned long getMotorStepCount()
{
    static unsigned long step_count_copy;
    noInterrupts();
    step_count_copy = motor_step_count;
    interrupts();
    return step_count_copy;
}

// Motor
const double MOTOR_NOM_PERIOD =
    60000000. / ((double)MOTOR_STEPS * MOTOR_MICRO_STEPS * MOTOR_GEAR_RATIO * MOTOR_RPM); // 882.35 microseconds per step
double motor_step_period = MOTOR_MAX_PERIOD;

static void setupMotor()
{
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    digitalWrite(MOTOR_DIR_PIN, MOTOR_DIRECTION);

    motor_step_period = MOTOR_MAX_PERIOD;

    Timer1.initialize((unsigned long)(motor_step_period + 0.5));
    Timer1.pwm(MOTOR_PUL_PIN, MOTOR_PWM_DUTY);
    Timer1.attachInterrupt(stepCounterISR);
    Timer1.stop();
}

static void setMotorSpeed(double target_period)
{
    if (target_period < MOTOR_MIN_PERIOD)
        target_period = MOTOR_MIN_PERIOD;
    if (target_period > MOTOR_MAX_PERIOD)
        target_period = MOTOR_MAX_PERIOD;

    double start_speed = MOTOR_NOM_PERIOD / motor_step_period;
    double end_speed   = MOTOR_NOM_PERIOD / target_period;
    double speed_diff  = end_speed - start_speed;

    long iterations = (long)(fabs(speed_diff) * 1000. / MOTOR_ACCEL + 0.5);
    if (iterations < 0)
        iterations = 0;

    unsigned long start_time, end_time;
    start_time = micros();

    for (long i = 0; i < iterations; ++i)
    {
        motor_step_period = MOTOR_NOM_PERIOD / (start_speed + speed_diff * (double)i / iterations);

        Timer1.setPeriod((unsigned long)(motor_step_period + 0.5));
        Timer1.setPwmDuty(MOTOR_PUL_PIN, MOTOR_PWM_DUTY);

        end_time = (unsigned long)(i + 1) * 1000; // Every iteration takes 1000 microseconds
        while (micros() - start_time < end_time)
            delayMicroseconds(1);
    }

    motor_step_period = target_period;

    Timer1.setPeriod((unsigned long)(motor_step_period + 0.5));
    Timer1.setPwmDuty(MOTOR_PUL_PIN, MOTOR_PWM_DUTY);
}

static void stopMotor()
{
    setMotorSpeed(MOTOR_MAX_PERIOD);
    Timer1.stop();
}

void setup()
{
    setupMotor();

    Serial.begin(9600);
}

void loop()
{
    // Delay 2 seconds
    Serial.println("Loop start.");
    delay(2000);

    // Accelerate
    unsigned long start_time = micros();
    stopMotor();
    setMotorStepCount(0);
    setMotorSpeed(MOTOR_NOM_PERIOD);
    unsigned long ramp_up_steps = getMotorStepCount();
    unsigned long ramp_up_time  = micros() - start_time;

    // Cruise
    double total_time    = TEST_TIME_SEC * 1000000.;
    double cruise_time   = total_time - ramp_up_time * 2.;
    double total_steps   = total_time / MOTOR_NOM_PERIOD;
    double cruise_steps  = total_steps - ramp_up_steps * 2.;
    double cruise_period = cruise_time / cruise_steps;
    setMotorSpeed(cruise_period);
    Serial.print("Ramp up steps: ");
    Serial.println(ramp_up_steps);
    Serial.print("Ramp up time: ");
    Serial.println(ramp_up_time);
    Serial.print("Cruise speed: ");
    Serial.println(MOTOR_NOM_PERIOD / cruise_period);
    while (micros() - start_time < (total_time - ramp_up_time))
        delay(1);

    // Decelerate
    stopMotor();
    unsigned long final_steps = getMotorStepCount();
    double        final_time  = micros() - start_time;
    Serial.print("Final steps: ");
    Serial.println(final_steps);
    Serial.print("Final time: ");
    Serial.println(final_time);
}
