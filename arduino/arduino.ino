//==================================//
//========== User Defines ==========//
//==================================//

// Enable serial debug
//#define SERIAL_DEBUG

// Motor settings
#define MOTOR_DIRECTION    HIGH // HIGH or LOW (OFFICAL)
#define MOTOR_START_SPEED  109  // 0 - 255 (OFFICAL)
#define MOTOR_DOUBLE_SPEED 200  // 0 - 255 (OFFICAL)

// Metal sensor settings
#define METAL_SENSOR_INVERT_INPUT true // true or false  (OFFICAL)
#define METAL_SENSOR_HOUR_PIN     12   // Digital Pin 12 (OFFICAL)
#define METAL_SENSOR_MINUTE_PIN   11   // Digital Pin 11 (OFFICAL)
#define METAL_SENSOR_HOUR_POS     3    // Hours (0 - 11) (OFFICAL)

// GPS serial port pins
#define GPS_PORT_RX_PIN 2 // Digital Pin 2 (attached to TX of GPS module)
#define GPS_PORT_TX_PIN 3 // Digital Pin 3 (attached to RX of GPS module)

// Local time zone offset
#define TIME_ZONE (+1) // UTC+1 (Amsterdam)

// EU daylight saving time configuration
static const struct
{
    static const uint8_t springMonth =  3; // March
    static const uint8_t springDate  = 31; // Latest last Sunday
    static const uint8_t springHour  =  2; // 2AM

    static const uint8_t fallMonth   = 10; // October
    static const uint8_t fallDate    = 31; // Latest last Sunday
    static const uint8_t fallHour    =  2; // 2AM
} EU_DST;

// Time constants
static const int32_t DEF_SECONDS_PER_MINUTE = 60;
static const int32_t DEF_SECONDS_PER_HOUR   = 3600;
static const int32_t DEF_SECONDS_PER_CLOCK  = 43200;


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

#if !defined(NMEAGPS_INTERRUPT_PROCESSING)
#error You must define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
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

// Clock time and clock speed
static int32_t clockTime = 0; // seconds
static byte clockSpeed = MOTOR_START_SPEED; // 0 - 255


//======================================//
//========== GPS/RTC Methods ===========//
//======================================//

// Adjusts date/time to local time zone with DST
static void adjustTime(NeoGPS::time_t &dt)
{
    // Convert date/time structure to seconds
    NeoGPS::clock_t seconds = dt;

    // Calculate DST changeover times once per reset and year
    static NeoGPS::time_t changeover;
    static NeoGPS::clock_t springForward, fallBack;

    if ((springForward == 0) || (changeover.year != dt.year))
    {
        // Set current year and zero minutes and seconds
        changeover.year = dt.year;
        changeover.minutes = 0;
        changeover.seconds = 0;

        // Calculate the spring changeover time (seconds)
        changeover.month = EU_DST.springMonth;
        changeover.date  = EU_DST.springDate;
        changeover.hours = EU_DST.springHour;
        changeover.set_day();

        // Step back to a Sunday, if day != SUNDAY
        changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
        springForward = (NeoGPS::clock_t)changeover;

        // Calculate the fall changeover time (seconds)
        changeover.month = EU_DST.fallMonth;
        changeover.date  = EU_DST.fallDate;
        changeover.hours = EU_DST.fallHour - 1; // To account for the apparent DST+1
        changeover.set_day();

        // Step back to a Sunday, if day != SUNDAY
        changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
        fallBack = (NeoGPS::clock_t)changeover;
    }

    // First offset from UTC to the local time zone
    seconds += TIME_ZONE * NeoGPS::SECONDS_PER_HOUR;

    // Then add an hour if DST is in effect
    if ((springForward <= seconds) && (seconds < fallBack))
        seconds += NeoGPS::SECONDS_PER_HOUR;

    // Convert seconds back to a date/time structure
    dt = seconds;
}

// Reads GPS time from the GPS serial port
// and adjusts the RTC time to the GPS time
static void adjustRTCTimeToGPSTime()
{
    unsigned long start = millis();

    // Read GPS for a maximum of one second
    while (millis() - start < 1000)
    {
        if (gpsParser.available())
        {
            gpsFix = gpsParser.read();

            if (gpsFix.valid.time && gpsFix.valid.date)
            {
                // Adjust GPS date/time to local time zone with DST
                adjustTime(gpsFix.dateTime);

                // Adjust RTC date/time to GPS date/time
                rtc.adjust(DateTime((NeoGPS::clock_t)gpsFix.dateTime + SECONDS_FROM_1970_TO_2000));

                // Print GPS and RTC date/time
                #ifdef SERIAL_DEBUG
                    Serial.print("Read GPS Time: ");
                    Serial << gpsFix.dateTime;
                    Serial.println();

                    DateTime now = rtc.now();
                    Serial.print("Adjusted RTC Time: ");
                    Serial.print(now.year(), DEC);
                    Serial.print('-');
                    Serial.print(now.month(), DEC);
                    Serial.print('-');
                    Serial.print(now.day(), DEC);
                    Serial.print(' ');
                    Serial.print(now.hour(), DEC);
                    Serial.print(':');
                    Serial.print(now.minute(), DEC);
                    Serial.print(':');
                    Serial.print(now.second(), DEC);
                    Serial.println();
                #endif

                // Break out of read loop
                break;
            }
        }
    }
}

// Handler to enable GPS interrupt processing
static void gpsParserInterruptHandler(uint8_t c)
{
    gpsParser.handle(c);
}


//====================================//
//========== Motor Methods ===========//
//====================================//

void setupMotorShield()
{
    // Setup motor 1
    pinMode(4, OUTPUT); // 1A
    pinMode(5, OUTPUT); // 1B
    pinMode(9, OUTPUT); // P_1

    digitalWrite(4, MOTOR_DIRECTION);
    digitalWrite(5, !MOTOR_DIRECTION);
    digitalWrite(9, LOW);

    // Setup motor 2
    pinMode(7, OUTPUT);  // 2A
    pinMode(8, OUTPUT);  // 2B
    pinMode(10, OUTPUT); // P_2

    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    digitalWrite(10, LOW);

    // Enable motor shield
    pinMode(6, OUTPUT); // EN
    digitalWrite(6, HIGH);
}

void setMotorSpeed(byte speed)
{
    analogWrite(9, speed); // Motor 1
}


//===========================================//
//========== Metal Sensor Methods ===========//
//===========================================//

bool isClockAtZeroHours()
{
    return digitalRead(METAL_SENSOR_HOUR_PIN) ? !METAL_SENSOR_INVERT_INPUT : METAL_SENSOR_INVERT_INPUT;
}

bool isClockAtZeroMinutes()
{
    return digitalRead(METAL_SENSOR_MINUTE_PIN) ? !METAL_SENSOR_INVERT_INPUT : METAL_SENSOR_INVERT_INPUT;
}


//====================================//
//========== Clock Methods ===========//
//====================================//

int32_t getTargetClockTime()
{
    // Adjust RTC time to GPS time
    adjustRTCTimeToGPSTime();

    // Get current RTC date/time
    DateTime now = rtc.now();

    // Convert date/time to clock time in seconds
    return (now.hour() % 12) * DEF_SECONDS_PER_HOUR +
        now.minute() * DEF_SECONDS_PER_MINUTE +
        now.second();
}

void setClockToTargetClockTime()
{
    // Set clock time to zero
    int32_t clockTime = METAL_SENSOR_HOUR_POS * DEF_SECONDS_PER_HOUR; // The hour sensor is located at 03:00

    // Set motor to double speed
    setMotorSpeed(MOTOR_DOUBLE_SPEED);

    // Wait until we hit zero minutes and zero hours
    while (!(isClockAtZeroMinutes() && isClockAtZeroHours())) void;

    #ifdef SERIAL_DEBUG
        Serial.println("[setClockToTargetClockTime] Clock reached initial position of 03:00");
    #endif

    while (true)
    {
        int32_t targetClockTimeFull = getTargetClockTime();
        int32_t targetClockTimeMinutes = targetClockTimeFull % DEF_SECONDS_PER_HOUR;
        int32_t targetClockTimeHours = targetClockTimeFull - targetClockTimeMinutes;
    
        if (clockTime == targetClockTimeHours)
        {
            #ifdef SERIAL_DEBUG
                Serial.print("[setClockToTargetClockTime] All hours added, remaining minutes to add: ");
                Serial.println(targetClockTimeMinutes * 2 / DEF_SECONDS_PER_MINUTE);
            #endif
            
            int32_t clockTimeWhenFullySet = (targetClockTimeFull + targetClockTimeMinutes) % DEF_SECONDS_PER_CLOCK;

            while (true)
            {
                // Get clock time difference between current and target clock time
                int32_t clockTimeDifference = clockTimeWhenFullySet - getTargetClockTime();
        
                // Adjust clock time difference if it is bigger than six hours
                if (abs(clockTimeDifference) > DEF_SECONDS_PER_HOUR * 6)
                {
                    if (clockTimeDifference > 0)
                        clockTimeDifference -= DEF_SECONDS_PER_CLOCK;
                    else
                        clockTimeDifference += DEF_SECONDS_PER_CLOCK;
                }
                    
                if (clockTimeDifference <= 0)
                    break;
            }

            // Set motor to start speed
            setMotorSpeed(MOTOR_START_SPEED);

            #ifdef SERIAL_DEBUG
                Serial.println("[setClockToTargetClockTime] Clock reached target clock time! Now running at normal speed");
            #endif

            // Delay to make sure we passed zero minutes
            if (targetClockTimeMinutes % 1800 < 300)
                delay(600 * 1000); // 10 minutes
    
            return;
        }

        // Delay to make sure we passed zero minutes
        delay(300 * 1000); // 5 minutes

        // Wait until we hit zero minutes
        while (!isClockAtZeroMinutes()) void;

        // Add one hour to clock time
        clockTime = (clockTime + DEF_SECONDS_PER_HOUR) % DEF_SECONDS_PER_CLOCK;

        #ifdef SERIAL_DEBUG
            Serial.print("[setClockToTargetClockTime] Added one hour to clock time: ");
            Serial.println(clockTime / DEF_SECONDS_PER_HOUR);
        #endif
    }
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

        // Get clock time difference between current and target clock time
        int32_t clockTimeDifference = clockTime - getTargetClockTime();

        #ifdef SERIAL_DEBUG
            Serial.print("[adjustClockSpeed] Clock time difference in hours (1): ");
            Serial.println((float)clockTimeDifference / DEF_SECONDS_PER_HOUR);
        #endif

        // Adjust clock time difference if it is bigger than six hours
        if (abs(clockTimeDifference) > DEF_SECONDS_PER_HOUR * 6)
        {
            if (clockTimeDifference > 0)
                clockTimeDifference -= DEF_SECONDS_PER_CLOCK;
            else
                clockTimeDifference += DEF_SECONDS_PER_CLOCK;
        }

        #ifdef SERIAL_DEBUG
            Serial.print("[adjustClockSpeed] Clock time difference in hours (2): ");
            Serial.println((float)clockTimeDifference / DEF_SECONDS_PER_HOUR);
        #endif

        // Adjust clock speed if clock time difference is bigger than a minute
        if (abs(clockTimeDifference) > 30) // Half a minute
        {
            if (clockTimeDifference > 0)
                clockSpeed--;
            else
                clockSpeed++;
        }

        #ifdef SERIAL_DEBUG
            Serial.print("[adjustClockSpeed] Adjusted clock speed: ");
            Serial.println(clockSpeed);
            Serial.println();
        #endif

        // Set the motor speed to the clock speed
        setMotorSpeed(clockSpeed);

        // Delay until we passed zero minutes
        delay(300 * 1000); // 5 minutes
    }
}


//======================================//
//========== Arduino Methods ===========//
//======================================//

void setup()
{
    // Begin debug serial communication
    #ifdef SERIAL_DEBUG
        Serial.begin(9600);
        while (!Serial) void;
        Serial.flush();
    #endif

    // Begin GPS serial communication
    gpsPort.attachInterrupt(gpsParserInterruptHandler);
    gpsPort.begin(9600);

    // Begin RTC I2C communication
    rtc.begin();

    // Setup motor shield
    setupMotorShield();
    setMotorSpeed(MOTOR_START_SPEED);

    // Setup metal sensor pins
    pinMode(METAL_SENSOR_HOUR_PIN, INPUT);
    pinMode(METAL_SENSOR_MINUTE_PIN, INPUT);

    // Set clock to target clock time
    setClockToTargetClockTime();
}

void loop()
{
    // Just keep adjusting the clock speed
    adjustClockSpeed();
}
