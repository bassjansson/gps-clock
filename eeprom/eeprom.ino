/*
 * Set these two defines to the current mechanical time of the clock.
 *
 * For example, 04:20 would be:
 *
 * #define CLOCK_TIME_HOURS   4  // 0 - 11 hours
 * #define CLOCK_TIME_MINUTES 20 // 0 - 59 minutes
 */
#define CLOCK_TIME_HOURS       0 // 0 - 11 hours
#define CLOCK_TIME_MINUTES     0 // 0 - 59 minutes

//============================================================

// State loop settings
#define SECONDS_PER_STATE_LOOP 30 // Duration of state loop, 30 seconds = half a minute

// Time constants
typedef uint32_t       clock12_t;
static const clock12_t DEF_SECONDS_PER_MINUTE = 60;
static const clock12_t DEF_SECONDS_PER_HOUR   = 60 * DEF_SECONDS_PER_MINUTE;
static const clock12_t DEF_SECONDS_PER_CLOCK  = 12 * DEF_SECONDS_PER_HOUR;

// Mechanical clock time in seconds
static clock12_t clockTime = 0;

// EEPROM
#include <EEPROM.h>

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

// Arduino
void setup()
{
    // First read clock time from EEPROM
    readClockTimeFromEEPROM(clockTime, SECONDS_PER_STATE_LOOP);

    // Set new clock time
    clockTime = (CLOCK_TIME_HOURS * DEF_SECONDS_PER_HOUR + CLOCK_TIME_MINUTES * DEF_SECONDS_PER_MINUTE) % DEF_SECONDS_PER_CLOCK;

    // Write new clock time to EEPROM
    writeClockTimeToEEPROM(clockTime, SECONDS_PER_STATE_LOOP);
}

void loop()
{
    // Do nothing
    delay(1000);
}
