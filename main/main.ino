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

// Motor and clock settings
#define MOTOR_DIRECTION         HIGH // HIGH or LOW (OFFICAL)

#define NOMINAL_CLOCK_SPEED_MIN 98  // 0 - 255 (OFFICAL)
#define NOMINAL_CLOCK_SPEED_MAX 118 // 0 - 255 (OFFICAL)

int32_t       nominalClockSpeed = 108; // 0 - 255 (OFFICAL)
const int32_t doubleClockSpeed  = 202; // 0 - 255 (OFFICAL)

// Metal sensor settings
#define METAL_SENSOR_HOUR_PIN   12 // Digital Pin 12 (OFFICAL)
#define METAL_SENSOR_MINUTE_PIN 11 // Digital Pin 11 (OFFICAL)
#define METAL_SENSOR_HOUR_POS   3  // Hours (0 - 11) (OFFICAL)

// GPS serial port pins
#define GPS_PORT_RX_PIN         2 // Digital Pin 2 (attached to TX of GPS module)
#define GPS_PORT_TX_PIN         3 // Digital Pin 3 (attached to RX of GPS module)

// Local time zone offset
#define TIME_ZONE               (+1) // UTC+1 (Amsterdam)

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

#include <Wire.h>
#include <RTClib.h>
#include <NMEAGPS.h>
#include <NeoSWSerial.h>


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

// Clock time
static clock12_t clockTime = 0; // seconds

//======================================//
//========== GPS/RTC Methods ===========//
//======================================//

static NeoGPS::clock_t getGPSTime()
{
    // Reset GPS parser
    gpsParser.reset();

    // Read GPS for a maximum of two seconds
    unsigned long start_time = millis();

    while (millis() - start_time < 2000)
    {
        if (gpsParser.available(gpsPort))
        {
            gpsFix = gpsParser.read();

            if (gpsFix.valid.time && gpsFix.valid.date)
            {
#ifdef SERIAL_DEBUG
                // Print GPS time
                Serial.print("[getGPSTime] Current GPS Time: ");
                Serial << gpsFix.dateTime;
                Serial.println();
#endif

                // Return GPS time
                return (NeoGPS::clock_t)gpsFix.dateTime;
            }
        }
    }

#ifdef SERIAL_DEBUG
    // GPS read failed
    Serial.println("[getGPSTime] Failed to read GPS time..");
#endif

    // Return 0 when GPS read failed
    return 0;
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
    // First try to get GPS time
    NeoGPS::clock_t gpsTime = getGPSTime(); // Seconds since 2000

    // Adjust RTC time to GPS time if GPS reading succeeded
    if (gpsTime != 0)
        rtc.adjust(DateTime(gpsTime + SECONDS_FROM_1970_TO_2000));

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
    return (clock12_t)(dt.hours % 12) * DEF_SECONDS_PER_HOUR + (clock12_t)dt.minutes * DEF_SECONDS_PER_MINUTE
         + (clock12_t)dt.seconds;
}

//====================================//
//========== Motor Methods ===========//
//====================================//

void setMotorSpeed(byte speed)
{
    // Do nothing
}

//===========================================//
//========== Metal Sensor Methods ===========//
//===========================================//

bool isClockAtZeroHours()
{
    if (digitalRead(METAL_SENSOR_HOUR_PIN))
        return false;

    delay(250);

    if (digitalRead(METAL_SENSOR_HOUR_PIN))
        return false;

    delay(250);

    return !digitalRead(METAL_SENSOR_HOUR_PIN);
}

bool isClockAtZeroMinutes()
{
    if (digitalRead(METAL_SENSOR_MINUTE_PIN))
        return false;

    delay(250);

    if (digitalRead(METAL_SENSOR_MINUTE_PIN))
        return false;

    delay(250);

    return !digitalRead(METAL_SENSOR_MINUTE_PIN);
}

//====================================//
//============= EEPROM ===============//
//====================================//

#include <EEPROM.h>

static uint16_t EEPROM_pos = 0;

void readClockTimeFromEEPROM(clock12_t& clock_time)
{
    int t1;

    uint16_t size   = sizeof(t1);
    uint16_t length = EEPROM.length() / size;

    for (uint16_t i = 0; i < length; i++)
    {
        EEPROM.get(i * size, t1);

        if (t1 >= 0)
        {
            clock_time = ((clock12_t)t1 * DEF_SECONDS_PER_MINUTE) % DEF_SECONDS_PER_CLOCK;
            EEPROM_pos = i;
            return;
        }
    }

    clock_time = 0;
    EEPROM_pos = 0;
}

void writeClockTimeToEEPROM(const clock12_t& clock_time)
{
    int t1 = (clockTime % DEF_SECONDS_PER_CLOCK) / DEF_SECONDS_PER_MINUTE;

    uint16_t size   = sizeof(t1);
    uint16_t length = EEPROM.length() / size;

    EEPROM.put(EEPROM_pos * size, -1);
    EEPROM_pos = (EEPROM_pos + 1) % length;
    EEPROM.put(EEPROM_pos * size, t1);
}

//====================================//
//========== Clock Methods ===========//
//====================================//

int32_t getRTCTime()
{
    return getAdjustedRTCTime();
}

void setClockToZeroPosition()
{
    // Set clock time to zero
    clockTime = METAL_SENSOR_HOUR_POS * DEF_SECONDS_PER_HOUR; // The hour sensor is located at 03:00

    // Set motor to double speed
    setMotorSpeed(doubleClockSpeed);

    // Wait until we hit zero minutes and zero hours
    while (!(isClockAtZeroMinutes() && isClockAtZeroHours()))
        delay(100); // 100 ms

#ifdef SERIAL_DEBUG
    Serial.println("[setClockToZeroPosition] Clock reached initial position of 03:00");
#endif
}

int32_t getTimeDifferenceBetween(int32_t upperTime, int32_t lowerTime)
{
    int32_t timeDifference = upperTime - lowerTime;

    if (abs(timeDifference) > DEF_SECONDS_PER_CLOCK / 2)
    {
        if (timeDifference > 0)
            timeDifference -= DEF_SECONDS_PER_CLOCK;
        else
            timeDifference += DEF_SECONDS_PER_CLOCK;
    }

    return timeDifference;
}

void adjustClockSpeed()
{
    // Check if it is time to adjust the clock speed
    if (isClockAtZeroMinutes())
    {
        // Update clock time
        if (isClockAtZeroHours())
            clockTime = METAL_SENSOR_HOUR_POS * DEF_SECONDS_PER_HOUR; // The hour sensor is located at 03:00
        else
            clockTime = (clockTime + DEF_SECONDS_PER_HOUR) % DEF_SECONDS_PER_CLOCK;

#ifdef SERIAL_DEBUG
        Serial.print("[adjustClockSpeed] Current clock time in hours: ");
        Serial.println((float)clockTime / DEF_SECONDS_PER_HOUR);
#endif

        // Get time difference between clock time and RTC time
        int32_t timeDifference = getTimeDifferenceBetween(clockTime, getRTCTime());

#ifdef SERIAL_DEBUG
        Serial.print("[adjustClockSpeed] Difference between clock time and RTC time in hours: ");
        Serial.println((float)timeDifference / DEF_SECONDS_PER_HOUR);
#endif

        // Set motor speed depending on time difference
        if (timeDifference > DEF_SECONDS_PER_HOUR / 2)
        {
            // Case: the clock is half an hour or more ahead of RTC time

            // Stop the motor
            setMotorSpeed(0);

#ifdef SERIAL_DEBUG
            Serial.println("[adjustClockSpeed] Motor stopped, now waiting for RTC time to catch up...");
#endif

            // Wait till RTC time caught up with clock time
            while (getTimeDifferenceBetween(clockTime, getRTCTime()) > 0)
                delay(100); // 100 ms

            // Set motor speed to nominal speed
            setMotorSpeed(nominalClockSpeed);

#ifdef SERIAL_DEBUG
            Serial.println("[adjustClockSpeed] RTC time caught up with clock time! Running at nominal speed.");
#endif
        }
        else if (timeDifference < -DEF_SECONDS_PER_HOUR / 2)
        {
            // Case: the clock is half an hour or more behind RTC time

            // Set motor speed to double speed
            setMotorSpeed(doubleClockSpeed);

#ifdef SERIAL_DEBUG
            Serial.println("[adjustClockSpeed] Motor set to double speed.");
#endif
        }
        else
        {
            // Case: the clock is not more than half an hour off RTC time

            // Adjust nominal clock speed, example:
            // At 3:00 we were on time perfectly.
            // We are now at 4:00 and we are 3 minutes ahead.
            // That means, that our nominal speed that we used to calculate the clock speed at 3:00, is wrong.
            // So, before we calculate the clock speed to get from 4:00 to five, we want a better nominal speed.
            if (timeDifference > 30 && nominalClockSpeed > NOMINAL_CLOCK_SPEED_MIN)
                nominalClockSpeed--;
            if (timeDifference < 30 && nominalClockSpeed < NOMINAL_CLOCK_SPEED_MAX)
                nominalClockSpeed++;

#ifdef SERIAL_DEBUG
            Serial.print("[adjustClockSpeed] Nominal clock speed adjusted to: ");
            Serial.println(nominalClockSpeed);
#endif

            // Calculate the clock speed
            byte clockSpeed = (nominalClockSpeed * 2 - doubleClockSpeed)
                            + ((doubleClockSpeed - nominalClockSpeed) * DEF_SECONDS_PER_HOUR * 10 + 5)
                                  / ((timeDifference + DEF_SECONDS_PER_HOUR) * 10);

            // Set motor speed to clock speed
            setMotorSpeed(clockSpeed);

#ifdef SERIAL_DEBUG
            Serial.print("[adjustClockSpeed] Clock speed calculated and set to: ");
            Serial.println(clockSpeed);
#endif
        }

        // Delay until we passed zero minutes
        int32_t waitTime = (getRTCTime() + 300) % DEF_SECONDS_PER_CLOCK; // 300 seconds = 5 minutes
        while (getTimeDifferenceBetween(waitTime, getRTCTime()) > 0)
            delay(100); // 100 ms

#ifdef SERIAL_DEBUG
        Serial.println("[adjustClockSpeed] End of 5 minute delay.");
#endif
    }

    // Delay a little bit for stability
    delay(100); // 100 ms
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
    // gpsPort.begin(9600);

    // Begin RTC I2C communication
    // rtc.begin();

    // Setup metal sensor pins
    // pinMode(METAL_SENSOR_HOUR_PIN, INPUT);
    // pinMode(METAL_SENSOR_MINUTE_PIN, INPUT);

    // Set clock to zero position
    // setClockToZeroPosition();

    readClockTimeFromEEPROM(clockTime);
}

void loop()
{
    // Just keep adjusting the clock speed
    // adjustClockSpeed();

    unsigned long start_time = millis();

    clockTime = (clockTime + DEF_SECONDS_PER_MINUTE) % DEF_SECONDS_PER_CLOCK;

    // writeClockTimeToEEPROM(clockTime);

    Serial.print("Current clock time: ");
    Serial.print(clockTime);
    Serial.print(" and EEPROM pos: ");
    Serial.println(EEPROM_pos);

    while (millis() - start_time < 1000)
        delay(1);
}
