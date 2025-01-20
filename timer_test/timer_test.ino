#include <TimerOne.h>

#define MOTOR_STEPS       200 // Full steps per revolution
#define MOTOR_MICRO_STEPS 2   // Microsteps
#define MOTOR_GEAR_RATIO  20  // Gear ratio

#define MOTOR_RPM         8.5 // RPM after gear
#define MOTOR_DIRECTION   LOW // HIGH = forward, LOW = backward
#define MOTOR_PWM_DUTY    512 // 50% duty cycle

#define MOTOR_DIR_PIN     8
#define MOTOR_PUL_PIN     9

// Test time in seconds
#define TEST_TIME_SEC     60


// Motor methods
volatile unsigned long step_count = 0; // use volatile for shared variables

// ~882 micro seconds step period
const double STEP_PERIOD = 60000000. / ((double)MOTOR_STEPS * MOTOR_MICRO_STEPS * MOTOR_GEAR_RATIO * MOTOR_RPM);
const double MIN_SPEED   = STEP_PERIOD / 1000000.; // max period is 1 second
const double MAX_SPEED   = STEP_PERIOD / 40.;      // min period is 40 micro seconds

static void stepCounterISR()
{
    step_count++;
}

static void setupMotor()
{
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    digitalWrite(MOTOR_DIR_PIN, MOTOR_DIRECTION);

    Timer1.initialize(1000000); // 1 second period
    Timer1.pwm(MOTOR_PUL_PIN, MOTOR_PWM_DUTY); // 50% duty
    Timer1.attachInterrupt(stepCounterISR);
    Timer1.stop();
}

static void setMotorSpeed(double speed)
{
    if (speed > 0.) {
        if (speed < MIN_SPEED)
            speed = MIN_SPEED;
        if (speed > MAX_SPEED)
            speed = MAX_SPEED;

        Timer1.setPeriod((unsigned long)(STEP_PERIOD / speed + 0.5));
        Timer1.setPwmDuty(MOTOR_PUL_PIN, MOTOR_PWM_DUTY);
    }
    else {
        Timer1.stop();
    }
}

static void setMotorStepCount(unsigned long count)
{
    noInterrupts();
    step_count = count;
    interrupts();
}

static unsigned long getMotorStepCount()
{
    static unsigned long count_copy;
    noInterrupts();
    count_copy = step_count;
    interrupts();
    return count_copy;
}

static void delayMicros(unsigned long delay_us, unsigned long start_us = 0)
{
    if (delay_us)
    {
        if (!start_us)
        {
            start_us = micros();
        }
        // See https://www.gammon.com.au/millis
        while (micros() - start_us < delay_us)
            ;
    }
}

void setup()
{
    setupMotor();

    Serial.begin(9600);
}

void loop()
{
    // Delay 5 seconds
    Serial.println("Loop start.");
    delay(2000);

    // Ramp up motor
    int iterations = 1000;
    double ramp_time = 1000000.;
    Serial.print("Starting with iterations: ");
    Serial.println(iterations);
    Serial.print("and ramp time: ");
    Serial.println(ramp_time / 1000000.);
    unsigned long start_time = micros();
    setMotorStepCount(0);
    for (int i = 0; i < iterations; ++i) {
        setMotorSpeed((double)i / iterations);
        delayMicros((unsigned long)(ramp_time * (i + 1) / iterations) - (micros() - start_time));
    }

    // Cruise
    double total_time = TEST_TIME_SEC * 1000000.;
    double cruise_time = total_time - 2. * ramp_time;
    double total_steps = total_time / STEP_PERIOD;
    unsigned long ramp_up_steps = getMotorStepCount();
    double cruise_steps = total_steps - ramp_up_steps * 2.;
    double cruise_speed = STEP_PERIOD / (cruise_time / cruise_steps);
    setMotorSpeed(cruise_speed);
    Serial.print("Ramp up steps: ");
    Serial.println(ramp_up_steps);
    Serial.print("Cruise speed: ");
    Serial.println(cruise_speed);
    unsigned long after_time = (unsigned long)(ramp_time + cruise_time);
    delayMicros((unsigned long)(ramp_time + cruise_time) - (micros() - start_time));

    // Ramp down motor
    for (int i = 0; i < iterations; ++i) {
        setMotorSpeed((double)(iterations - i - 1) / iterations);
        delayMicros((unsigned long)(ramp_time * (i + 1) / iterations) - ((micros() - start_time) - after_time));
    }
    setMotorSpeed(0);
    unsigned long final_steps = getMotorStepCount();
    double final_time = (micros() - start_time);
    Serial.print("Final steps: ");
    Serial.println(final_steps);
    Serial.print("Final time: ");
    Serial.println(final_time);
}
