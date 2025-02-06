/*
 * Libraries:
 * - RTClib      (2.1.4)
 * - NeoGPS      (4.2.9)
 * - NeoSWSerial (3.0.5)
 * - TimerOne    (1.1.1)
 *
 * Library mods:
 * - Comment line 31 in RTClib/src/RTClib.h (SECONDS_PER_DAY)
 * - Copy and overwrite both config files to NeoGPS/src/
 *
 * NeoGPS notes:
 * - Only RMC is needed for Date+Time
 * - NMEAGPS_EXPLICIT_MERGING is used
 * - NMEAGPS_RECOGNIZE_ALL needs to be off
 */

//==================================//
//========== User Defines ==========//
//==================================//

// Enable serial debug
#define SERIAL_DEBUG

// State loop settings
#define SECONDS_PER_STATE_LOOP   30 // Duration of state loop, 30 seconds = half a minute

// Motor settings
#define MOTOR_STEPS              200 // Full steps per revolution
#define MOTOR_MICRO_STEPS        2   // Microsteps (2 * 200 = 400)

#define MOTOR_NOM_RPM            170 // Nominal RPM before gear (20 * 8.5 = 170)
#define MOTOR_MIN_RPM            100 // Minimal working RPM
#define MOTOR_MAX_RPM            500 // Maximum working RPM

#define MOTOR_MAX_PERIOD         150000 // Max timer period in us, RPM = 1 (before gear)
#define MOTOR_MIN_PERIOD         150    // Min timer period in us, RPM = 1000 (before gear)

#define MOTOR_DIRECTION          LOW // LOW = forward, HIGH = backward
#define MOTOR_PWM_DUTY           512 // 50% duty cycle, max 1024

#define MOTOR_ACCEL              1.0f // Acceleration, smaller slower, higher is faster
#define MOTOR_DECEL              1.0f // Deceleration, smaller slower, higher is faster

#define MOTOR_ACC_PERIOD         1 // Acceleration update period in ms

#define MOTOR_DIR_PIN            8 // Motor direction pin (digital pin 8)
#define MOTOR_PUL_PIN            9 // Motor pulse pin (digital pin 9)

// Metal sensor settings
#define METAL_SENSOR_HOUR_PIN    12 // Metal sensor hour pin (digital pin 12)
#define METAL_SENSOR_MINUTE_PIN  11 // Metal sensor minute pin (digital pin 11)

#define METAL_SENSOR_HOUR_POS    3     // Metal sensor hour position, sitting at 03:00
#define METAL_SENSOR_MINUTE_POS  (-15) // Metal sensor minute position, in seconds

#define METAL_SENSOR_CHECK_COUNT 4  // How many checks to perform for each metal sensor
#define METAL_SENSOR_CHECK_DELAY 80 // Amount of time between each metal sensor check in ms

// GPS serial port pins
#define GPS_PORT_RX_PIN          2 // GPS serial port RX pin (digital pin 2, attached to TX of GPS module)
#define GPS_PORT_TX_PIN          3 // GPS serial port TX pin (digital pin 3, attached to RX of GPS module)

// Local time zone offset
#define TIME_ZONE                (+1) // UTC+1 (Amsterdam)

// EU daylight saving time configuration
static const struct
{
    static const uint8_t springMonth = 3;  // March
    static const uint8_t springDate  = 31; // Latest last Sunday
    static const uint8_t springHour  = 2;  // 2AM

    static const uint8_t fallMonth = 10; // October
    static const uint8_t fallDate  = 31; // Latest last Sunday
    static const uint8_t fallHour  = 2;  // 2AM
} EU_DST;

// Time constants
typedef uint32_t       clock12_t;
static const clock12_t DEF_SECONDS_PER_MINUTE = 60;
static const clock12_t DEF_SECONDS_PER_HOUR   = 60 * DEF_SECONDS_PER_MINUTE;
static const clock12_t DEF_SECONDS_PER_CLOCK  = 12 * DEF_SECONDS_PER_HOUR;


//==============================//
//========== Includes ==========//
//==============================//

// GPS
#include <NMEAGPS.h>
#include <NeoSWSerial.h>

// RTC
#include <Wire.h>
#include <RTClib.h>

// Motor
#include <TimerOne.h>

// Clock
#include <EEPROM.h>

//====================================//
//========== NeoGPS Config ===========//
//====================================//

#if !defined(GPS_FIX_TIME) | !defined(GPS_FIX_DATE)
    #error You must define GPS_FIX_TIME and GPS_FIX_DATE in GPSfix_cfg.h!
#endif

#if !defined(NMEAGPS_PARSE_RMC) & !defined(NMEAGPS_PARSE_ZDA)
    #error You must define NMEAGPS_PARSE_RMC or NMEAGPS_PARSE_ZDA in NMEAGPS_cfg.h!
#endif

#if defined(NMEAGPS_INTERRUPT_PROCESSING)
    #error You must *undefine* NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif


//======================================//
//========== GPS/RTC Objects ===========//
//======================================//

// GPS parser and fix
static NMEAGPS gpsParser; // This parses received characters
static gps_fix gpsFix;    // This contains all the parsed pieces

// GPS serial port
static NeoSWSerial gpsPort(GPS_PORT_RX_PIN, GPS_PORT_TX_PIN);

// RTC object
static RTC_DS1307 rtc;

//======================================//
//========== GPS/RTC Methods ===========//
//======================================//

static void readGPSAndUpdateRTC()
{
    // Read from GPS port if characters are available
    if (gpsParser.available(gpsPort))
    {
        gpsFix = gpsParser.read();

        if (gpsFix.valid.time && gpsFix.valid.date)
        {
#ifdef SERIAL_DEBUG
            // Print GPS time
            Serial.print("[readGPSTime] Current GPS Time: ");
            Serial << gpsFix.dateTime;
            Serial.println();
#endif

            // Adjust RTC to GPS time
            rtc.adjust(DateTime((NeoGPS::clock_t)gpsFix.dateTime + SECONDS_FROM_1970_TO_2000));
        }
    }
    else
    {
        // Delay a little bit
        delay(10);
    }
}

// Adjusts date/time to local time zone with DST
static void adjustTimeToLocalDST(NeoGPS::clock_t& seconds)
{
    // Convert seconds to date/time struct
    static NeoGPS::time_t dt;
    dt.init();
    dt += seconds;

    // Calculate DST changeover times once per reset and year
    static NeoGPS::time_t  changeover(0);
    static NeoGPS::clock_t springForward = 0;
    static NeoGPS::clock_t fallBack      = 0;

    if ((springForward == 0) || (changeover.year != dt.year))
    {
        // Set current year and zero minutes and seconds
        changeover.year    = dt.year;
        changeover.minutes = 0;
        changeover.seconds = 0;

        // Calculate the spring changeover time (seconds)
        changeover.month = EU_DST.springMonth; // March
        changeover.date  = EU_DST.springDate;  // 31
        changeover.hours = EU_DST.springHour;  // 2AM
        changeover.set_day();

        // Step back to a Sunday, if day != SUNDAY
        changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
        springForward = (NeoGPS::clock_t)changeover;

        // Calculate the fall changeover time (seconds)
        changeover.month = EU_DST.fallMonth; // October
        changeover.date  = EU_DST.fallDate;  // 31
        changeover.hours = EU_DST.fallHour;  // 2AM
        changeover.set_day();

        // Step back to a Sunday, if day != SUNDAY
        changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
        fallBack = (NeoGPS::clock_t)changeover;
    }

    // First offset from UTC to the local time zone
    seconds += (NeoGPS::clock_t)TIME_ZONE * NeoGPS::SECONDS_PER_HOUR;

    // Then add an hour if DST is in effect
    if ((seconds >= springForward) && (seconds < fallBack))
        seconds += (NeoGPS::clock_t)NeoGPS::SECONDS_PER_HOUR;
}

static clock12_t getAdjustedRTCTime()
{
    // Get RTC time in seconds
    NeoGPS::clock_t rtcTime = rtc.now().secondstime(); // Seconds since 2000

    // Adjust RTC time to local time with DST
    adjustTimeToLocalDST(rtcTime);

    // Convert seconds to date/time struct
    static NeoGPS::time_t dt;
    dt.init();
    dt += rtcTime;

#ifdef SERIAL_DEBUG
    // Print adjusted RTC time
    Serial.print("[getAdjustedRTCTime] Adjusted RTC Time: ");
    Serial << dt;
    Serial.println();
#endif

    // Convert date/time to 12 hour clock time in seconds
    return ((clock12_t)dt.hours * DEF_SECONDS_PER_HOUR + (clock12_t)dt.minutes * DEF_SECONDS_PER_MINUTE + (clock12_t)dt.seconds)
         % DEF_SECONDS_PER_CLOCK;
}

//====================================//
//========== Motor Methods ===========//
//====================================//

// Motor nominal period in microseconds
static const double MOTOR_NOM_PERIOD =
    (60. * 1000. * 1000.) / ((double)MOTOR_STEPS * MOTOR_MICRO_STEPS * MOTOR_NOM_RPM); // 882.35 microseconds per step

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

float calculateNewMotorSpeed(float start_speed, float distance, unsigned int end_time)
{
    static const float MIN_MOTOR_TARGET_SPEED = (float)MOTOR_MIN_RPM / MOTOR_NOM_RPM; // 100 RPM
    static const float MAX_MOTOR_TARGET_SPEED = (float)MOTOR_MAX_RPM / MOTOR_NOM_RPM; // 500 RPM

    // End time can not be zero
    if (end_time < 1)
        end_time = 1;

    // Get target speed
    float target_speed = distance / (float)end_time;

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
        end_speed = start_speed + d - sqrtf(fabsf(d * (d + 2.0f * (start_speed - target_speed))));
    }
    else
    {
        float d   = (float)-MOTOR_DECEL * end_time;
        end_speed = start_speed + d + sqrtf(fabsf(d * (d + 2.0f * (start_speed - target_speed))));
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

static void beginMotorAcceleration(float distance, unsigned int end_time)
{
    motor_start_speed  = motor_current_speed;
    motor_target_speed = calculateNewMotorSpeed(motor_start_speed, distance, end_time);

    float speed_diff = motor_target_speed - motor_start_speed;
    float accel      = speed_diff >= 0.0f ? MOTOR_ACCEL : MOTOR_DECEL;

    motor_accel_iter_pos = 0;

    long count = (long)(fabsf(speed_diff) * (1000.f / MOTOR_ACC_PERIOD) / accel + 0.5f);
    if (count < 1)
        count = 1;
    motor_accel_iter_cnt = count;

    motor_accel_start_time_us = micros();
}

static bool accelerateMotor()
{
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

    Timer1.setPeriod((unsigned long)(MOTOR_NOM_PERIOD / motor_current_speed + 0.5));
    Timer1.setPwmDuty(MOTOR_PUL_PIN, MOTOR_PWM_DUTY);

    if (stop_timer)
        Timer1.stop();

    return motor_accel_iter_pos >= motor_accel_iter_cnt;
}

//===========================================//
//========== Metal Sensor Methods ===========//
//===========================================//

void setupMetalSensors()
{
    pinMode(METAL_SENSOR_HOUR_PIN, INPUT);
    pinMode(METAL_SENSOR_MINUTE_PIN, INPUT);
}

// Blocking
bool isClockAtZeroHours()
{
    int count = 0;

    for (int i = 1; i < METAL_SENSOR_CHECK_COUNT; i++)
    {
        count += !digitalRead(METAL_SENSOR_HOUR_PIN);
        delay(METAL_SENSOR_CHECK_DELAY);
    }

    count += !digitalRead(METAL_SENSOR_HOUR_PIN);

    return count >= (METAL_SENSOR_CHECK_COUNT / 2 + 1);
}

// Non-blocking
bool isClockAtZeroMinutes()
{
    static unsigned long start = millis();
    static int           count = 0;
    static bool          state = false;

    if (millis() - start >= METAL_SENSOR_CHECK_DELAY)
    {
        start = millis();

        if (!digitalRead(METAL_SENSOR_MINUTE_PIN) != state)
            count++;
        else if (count > 0)
            count--;

        if (count >= METAL_SENSOR_CHECK_COUNT)
        {
            count = 0;
            state = !state;
            return state;
        }
    }

    return false;
}

//====================================//
//============= EEPROM ===============//
//====================================//

static uint16_t EEPROM_pos = 0;

static void readClockTimeFromEEPROM(clock12_t& clock_time, clock12_t resolution)
{
    int t1;

    uint16_t size   = sizeof(t1);
    uint16_t length = EEPROM.length() / size;

    for (uint16_t i = 0; i < length; i++)
    {
        EEPROM.get(i * size, t1);

        if (t1 >= 0)
        {
            clock_time = ((clock12_t)t1 * resolution) % DEF_SECONDS_PER_CLOCK;
            EEPROM_pos = i;
            return;
        }
    }

    clock_time = 0;
    EEPROM_pos = 0;
}

static void writeClockTimeToEEPROM(const clock12_t& clock_time, clock12_t resolution)
{
    if (resolution < 1)
        resolution = 1;

    int t1 = (clock_time % DEF_SECONDS_PER_CLOCK) / resolution;

    static int prev_t1 = -1;
    if (t1 == prev_t1)
        return;
    prev_t1 = t1;

    uint16_t size   = sizeof(t1);
    uint16_t length = EEPROM.length() / size;

    EEPROM.put(EEPROM_pos * size, -1);
    EEPROM_pos = (EEPROM_pos + 1) % length;
    EEPROM.put(EEPROM_pos * size, t1);
}

//====================================//
//========== Clock Methods ===========//
//====================================//

/*
State loop flow:

Using three states and going through them every half a minute:
1. GPS read (2 seconds), RTC update and calculate new speed
2. Accelerate motor to new speed (around 1 second)
3. Wait for half minute to end, then update clock time
4. Lastly write clock time and reset GPS

At all times:
- Read zero minutes metal sensor
*/

enum LoopState
{
    STATE_READ_GPS   = 0,
    STATE_ACCELERATE = 1,
    STATE_WAIT_END   = 2,
    STATE_WRITE_TIME = 3
};

static clock12_t clockTime_sec = 0; // Mechanical clock time in seconds
static clock12_t clockTime_ms  = 0; // Mechanical clock time in milliseconds
static clock12_t prev_motor_ms = 0; // Milliseconds since last zero minutes trigger

void setupClock()
{
    // Read clock time from EEPROM
    readClockTimeFromEEPROM(clockTime_sec, SECONDS_PER_STATE_LOOP);
    clockTime_ms = clockTime_sec * 1000;

    // Reset motor step count and time
    setMotorStepCount(0);
    prev_motor_ms = 0;
}

void updateClock()
{
    // Always check zero minutes sensor first
    static bool                zero_minutes_reached      = false;
    static unsigned long       zero_minutes_trigger_time = DEF_SECONDS_PER_CLOCK * 1000; // Arbitrarely large initialization
    static const unsigned long ZERO_MINUTES_TRIGGER_WAIT = DEF_SECONDS_PER_MINUTE * 10 * 1000; // 10 minutes in milliseconds

    if (millis() - zero_minutes_trigger_time >= ZERO_MINUTES_TRIGGER_WAIT)
    {
        if (isClockAtZeroMinutes())
        {
            zero_minutes_reached      = true;
            zero_minutes_trigger_time = millis();

            // Reset motor step count and time
            setMotorStepCount(0);
            prev_motor_ms = 0;
        }
    }

    // Switch state
    static LoopState     loop_state         = STATE_READ_GPS;
    static unsigned long loop_start_time_ms = millis();
    switch (loop_state)
    {
        case STATE_READ_GPS:
        {
            // Allow GPS to be read for 2 seconds
            if (millis() - loop_start_time_ms < 2000)
            {
                // Read GPS and update RTC
                readGPSAndUpdateRTC();
            }
            else
            {
                // Stop GPS serial reading
                gpsPort.ignore();

                // Get distance from clock time to RTC time + one state loop from now
                clock12_t rtcTime_ms = (getAdjustedRTCTime() + SECONDS_PER_STATE_LOOP + DEF_SECONDS_PER_CLOCK) * 1000;
                float     distance   = ((rtcTime_ms - clockTime_ms) % (DEF_SECONDS_PER_CLOCK * 1000)) / 1000.f;

                // Distance becomes negative if more than 6 hours
                if (distance > DEF_SECONDS_PER_CLOCK / 2)
                    distance -= DEF_SECONDS_PER_CLOCK;

                // Begin motor acceleration towards new speed
                beginMotorAcceleration(distance, SECONDS_PER_STATE_LOOP);

#ifdef SERIAL_DEBUG
                // Print distance
                Serial.print("[updateClock] New target speed: ");
                Serial.print(motor_target_speed);
                Serial.print(" , time distance: ");
                Serial.println(distance);
#endif

                // Go to next state
                loop_state = STATE_ACCELERATE;
            }
        }
        break;

        case STATE_ACCELERATE:
        {
            // Accelerate or decelerate the motor and go to next state when ready
            if (accelerateMotor())
            {
                // Go to next state
                loop_state = STATE_WAIT_END;
            }
        }
        break;

        case STATE_WAIT_END:
        {
            // Save current time
            unsigned long current_time = millis();

            // Update clock time if zero minutes reached
            if (zero_minutes_reached)
            {
                zero_minutes_reached = false;

                // Get current hour
                // If zero hours metal sensor is triggered,
                // set clock time to 3:00, as the hour sensor is located at 3:00
                // Otherwise, round the current clock time to a whole hour
                clock12_t hours = isClockAtZeroHours() ? METAL_SENSOR_HOUR_POS
                                                       : (clockTime_sec + DEF_SECONDS_PER_HOUR / 2) / DEF_SECONDS_PER_HOUR;

                // Get zero minutes metal sensor offset in seconds
                clock12_t offset_sec = DEF_SECONDS_PER_CLOCK + METAL_SENSOR_MINUTE_POS;

                // Set new clock time
                clockTime_sec = (hours * DEF_SECONDS_PER_HOUR + offset_sec) % DEF_SECONDS_PER_CLOCK;
                clockTime_ms  = clockTime_sec * 1000;

                // End of loop reached
                loop_start_time_ms = current_time;

                // Go to next state
                loop_state = STATE_WRITE_TIME;
            }
            // Otherwise, update clock time if end of loop reached
            else if (current_time - loop_start_time_ms >= SECONDS_PER_STATE_LOOP * 1000)
            {
                // Get current motor time since last zero minutes trigger
                clock12_t curr_motor_ms = (clock12_t)((double)getMotorStepCount() * MOTOR_NOM_PERIOD / 1000. + 0.5);

                // Set new clock time
                clockTime_ms  = (clockTime_ms + (curr_motor_ms - prev_motor_ms)) % (DEF_SECONDS_PER_CLOCK * 1000);
                clockTime_sec = (clockTime_ms + 500) / 1000;

                // Update previous with current motor time
                prev_motor_ms = curr_motor_ms;

                // End of loop reached
                loop_start_time_ms = current_time;

                // Go to next state
                loop_state = STATE_WRITE_TIME;
            }
            else
            {
                // Delay a little bit
                delay(10);
            }
        }
        break;

        case STATE_WRITE_TIME:
        {
            // Write updated clock time to EEPROM
            writeClockTimeToEEPROM(clockTime_sec, SECONDS_PER_STATE_LOOP);

#ifdef SERIAL_DEBUG
            // Get hours, minutes, seconds
            uint8_t hours   = clockTime_sec / DEF_SECONDS_PER_HOUR;
            uint8_t minutes = (clockTime_sec % DEF_SECONDS_PER_HOUR) / DEF_SECONDS_PER_MINUTE;
            uint8_t seconds = clockTime_sec % DEF_SECONDS_PER_MINUTE;

            // Print mechanical clock time
            Serial.println();
            Serial.print("[updateClock] Mechanical clock time: ");

            if (hours < 10)
                Serial.print("0");
            Serial.print(hours);
            Serial.print(":");

            if (minutes < 10)
                Serial.print("0");
            Serial.print(minutes);
            Serial.print(":");

            if (seconds < 10)
                Serial.print("0");
            Serial.println(seconds);
#endif

            // Reset GPS parser before reading GPS again
            gpsParser.reset();
            gpsPort.flush();
            gpsPort.listen();

            // Go to next state
            loop_state = STATE_READ_GPS;
        }
        break;
    }
}

//======================================//
//========== Arduino Methods ===========//
//======================================//

void setup()
{
#ifdef SERIAL_DEBUG
    // Begin debug serial communication
    Serial.begin(9600);
    while (!Serial)
        void;
    Serial.flush();
#endif

    // Begin GPS serial communication
    gpsPort.begin(9600);
    gpsPort.ignore();

    // Begin RTC I2C communication
    rtc.begin();

    // Setup metal sensor
    setupMetalSensors();

    // Setup motor
    setupMotor();

    // Setup clock
    setupClock();
}

void loop()
{
    // Update clock
    updateClock();
}
