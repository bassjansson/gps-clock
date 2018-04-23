//=============================//
//========== Defines ==========//
//=============================//

// Enable serial debug
#define SERIAL_DEBUG

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


//======================================//
//========== GPS/RTC Methods ===========//
//======================================//

// Adjusts date/time to local time zone with DST
void adjustTime(NeoGPS::time_t &dt)
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
static void adjustRTCTimeByGPSTime()
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


//======================================//
//========== Arduino Methods ===========//
//======================================//

void setup()
{
    // Begin debug serial communication
    #ifdef SERIAL_DEBUG
        Serial.begin(9600);
        while (!Serial);
        Serial.flush();
    #endif

    // Begin GPS serial communication
    gpsPort.attachInterrupt(gpsParserInterruptHandler);
    gpsPort.begin(9600);

    // Begin RTC I2C communication
    rtc.begin();
}

void loop()
{
    adjustRTCTimeByGPSTime();

    while (millis() % 5000 > 0);

    #ifdef SERIAL_DEBUG
        Serial.println("End of loop!");
    #endif
}
