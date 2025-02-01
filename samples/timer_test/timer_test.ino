// Motor includes
#include <TimerOne.h>

// Motor defines
#define MOTOR_STEPS       200 // Full steps per revolution
#define MOTOR_MICRO_STEPS 2   // Microsteps (2 * 200 = 400)

#define MOTOR_NOM_RPM     170 // Nominal RPM before gear (20 * 8.5 = 170)
#define MOTOR_MIN_RPM     100 // Minimal working RPM
#define MOTOR_MAX_RPM     500 // Maximum working RPM

#define MOTOR_MAX_PERIOD  150000 // Min timer period in us, RPM = 1 (before gear)
#define MOTOR_MIN_PERIOD  150    // Max timer period in us, RPM = 1000 (before gear)

#define MOTOR_DIRECTION   LOW // LOW = forward, HIGH = backward
#define MOTOR_PWM_DUTY    512 // 50% duty cycle, max 1024

#define MOTOR_ACCEL       1.0f // Acceleration
#define MOTOR_DECEL       0.7f // Deceleration

#define MOTOR_ACC_PERIOD  1 // Acceleration update period in ms

#define MOTOR_DIR_PIN     8
#define MOTOR_PUL_PIN     9

// Motor step counter
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

// Motor speed
static float motor_start_speed   = 0.0f;
static float motor_current_speed = 0.0f;
static float motor_target_speed  = 0.0f;

static void setupMotor()
{
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    digitalWrite(MOTOR_DIR_PIN, MOTOR_DIRECTION);

    motor_start_speed   = 0.0f;
    motor_current_speed = 0.0f;
    motor_target_speed  = 0.0f;

    Timer1.initialize(MOTOR_MAX_PERIOD);
    Timer1.pwm(MOTOR_PUL_PIN, MOTOR_PWM_DUTY);
    Timer1.attachInterrupt(stepCounterISR);
    Timer1.stop();
}

float calculateNewMotorSpeed(float start_speed, long distance, unsigned long end_time)
{
    static const float MIN_MOTOR_TARGET_SPEED = (float)MOTOR_MIN_RPM / MOTOR_NOM_RPM; // 100 RPM
    static const float MAX_MOTOR_TARGET_SPEED = (float)MOTOR_MAX_RPM / MOTOR_NOM_RPM; // 500 RPM

    // Get target speed
    float target_speed = (float)distance / end_time;

    // Return if target speed is not within boundaries
    if (target_speed < MIN_MOTOR_TARGET_SPEED)
        return 0.0f;
    if (target_speed > MAX_MOTOR_TARGET_SPEED)
        return MAX_MOTOR_TARGET_SPEED;

    // Calculate end speed using the abc-formula
    // a = start_speed
    // b = end_speed
    // y = target_speed
    // x = end_time
    // acc = MOTOR_ACCEL or MOTOR_DECEL
    float end_speed = target_speed;

    if (target_speed >= start_speed)
    {
        float d   = (float)MOTOR_ACCEL * end_time;
        end_speed = start_speed + d - sqrtf(d * (d + 2.0f * (start_speed - target_speed)));
    }
    else
    {
        float d   = (float)-MOTOR_DECEL * end_time;
        end_speed = start_speed + d + sqrtf(d * (d + 2.0f * (start_speed - target_speed)));
    }

    // Clip end speed
    if (end_speed < MIN_MOTOR_TARGET_SPEED)
        end_speed = 0.0f;
    if (end_speed > MAX_MOTOR_TARGET_SPEED)
        end_speed = MAX_MOTOR_TARGET_SPEED;

    // Return calculated end speed
    return end_speed;
}

// Motor acceleration
static unsigned long motor_accel_iter_pos      = 0;
static unsigned long motor_accel_iter_cnt      = 0;
static unsigned long motor_accel_start_time_us = micros();

static void beginMotorAcceleration(long distance, unsigned long end_time)
{
    motor_start_speed  = motor_current_speed;
    motor_target_speed = calculateNewMotorSpeed(motor_start_speed, distance, end_time);

    float speed_diff = motor_target_speed - motor_start_speed;
    float accel      = speed_diff >= 0.0f ? MOTOR_ACCEL : MOTOR_DECEL;

    motor_accel_iter_pos = 0;

    long count = (long)(fabsf(speed_diff) * (1000.f / MOTOR_ACC_PERIOD) / accel + 0.5f);
    if (count < 0)
        count = 0;
    motor_accel_iter_cnt = count;

    motor_accel_start_time_us = micros();
}

static bool accelerateMotor()
{
    static const float MOTOR_NOM_PERIOD =
        60000000.f / ((float)MOTOR_STEPS * MOTOR_MICRO_STEPS * MOTOR_NOM_RPM); // 882.35 microseconds per step

    static const float MIN_MOTOR_ACCEL_SPEED = MOTOR_NOM_PERIOD / MOTOR_MAX_PERIOD; // 1 RPM
    static const float MAX_MOTOR_ACCEL_SPEED = MOTOR_NOM_PERIOD / MOTOR_MIN_PERIOD; // 1000 RPM

    motor_accel_iter_pos++;

    float speed_diff = motor_target_speed - motor_start_speed;
    float new_speed  = motor_start_speed + speed_diff * ((float)motor_accel_iter_pos / motor_accel_iter_cnt);

    bool stop_timer = new_speed < MIN_MOTOR_ACCEL_SPEED;

    if (new_speed < MIN_MOTOR_ACCEL_SPEED)
        new_speed = MIN_MOTOR_ACCEL_SPEED;
    if (new_speed > MAX_MOTOR_ACCEL_SPEED)
        new_speed = MAX_MOTOR_ACCEL_SPEED;

    unsigned long end_time_us = motor_accel_iter_pos * MOTOR_ACC_PERIOD * 1000;
    while (micros() - motor_accel_start_time_us < end_time_us)
        delayMicroseconds(1);

    motor_current_speed = new_speed;

    Timer1.setPeriod((unsigned long)(MOTOR_NOM_PERIOD / motor_current_speed + 0.5f));
    Timer1.setPwmDuty(MOTOR_PUL_PIN, MOTOR_PWM_DUTY);

    if (stop_timer)
        Timer1.stop();

    return motor_accel_iter_pos >= motor_accel_iter_cnt;
}

//======================================================
// Test code

#define TEST_TIME_SEC 10

void setup()
{
    setupMotor();

    Serial.begin(9600);
}

void loop()
{
    // Delay 2 seconds
    Serial.println("Loop start.");
    Serial.println();
    delay(2000);
    unsigned long start_time, stop_time, test_step_cnt;

    // Accelerate
    Serial.println("Accelerating to double speed..");
    setMotorStepCount(0);
    start_time = micros();
    beginMotorAcceleration(TEST_TIME_SEC * 2, TEST_TIME_SEC);
    while (!accelerateMotor())
        ;
    stop_time = micros();
    Serial.print("Acc time in ms: ");
    Serial.println((stop_time - start_time) / 1000.f);
    while (micros() - start_time < TEST_TIME_SEC * 1000000)
        ;
    test_step_cnt = getMotorStepCount();
    Serial.print("Step count after acc: ");
    Serial.println(test_step_cnt);
    Serial.println();

    // Decelerate
    Serial.println("Decelerating to nominal speed..");
    setMotorStepCount(0);
    start_time = micros();
    beginMotorAcceleration(TEST_TIME_SEC, TEST_TIME_SEC);
    while (!accelerateMotor())
        ;
    stop_time = micros();
    Serial.print("Dec time in ms: ");
    Serial.println((stop_time - start_time) / 1000.f);
    while (micros() - start_time < TEST_TIME_SEC * 1000000)
        ;
    test_step_cnt = getMotorStepCount();
    Serial.print("Step count after dec: ");
    Serial.println(test_step_cnt);
    Serial.println();

    // Stop motor
    Serial.println("Stopping motor..");
    setMotorStepCount(0);
    start_time = micros();
    beginMotorAcceleration(0, TEST_TIME_SEC);
    while (!accelerateMotor())
        ;
    stop_time     = micros();
    test_step_cnt = getMotorStepCount();
    Serial.print("Stop time in ms: ");
    Serial.println((stop_time - start_time) / 1000.f);
    Serial.print("Step count after stop: ");
    Serial.println(test_step_cnt);
    Serial.println();
}
