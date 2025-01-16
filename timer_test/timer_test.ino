#include <TimerOne.h>

void setup(void)
{
    Timer1.initialize(13700);
    Timer1.attachInterrupt(blinkLED); // blinkLED to run every 0.15 seconds
    Serial.begin(9600);
}

volatile unsigned long period_time = 0; // use volatile for shared variables
volatile bool period_time_changed = 0; // use volatile for shared variables

void blinkLED(void)
{
    static unsigned long curr_micros = micros();
    static unsigned long prev_micros = micros();

    curr_micros = micros();

    period_time = curr_micros - prev_micros;
    period_time_changed = 1;

    prev_micros = curr_micros;
}

// The main program will print the blink count
// to the Arduino Serial Monitor
void loop(void)
{
    static unsigned long period_time_copy = 0; // holds a copy
    static unsigned long prev_period_copy = 0;

    // to read a variable which the interrupt code writes, we
    // must temporarily disable interrupts, to be sure it will
    // not change while we are reading.  To minimize the time
    // with interrupts off, just quickly make a copy, and then
    // use the copy while allowing the interrupt to keep working.
    noInterrupts();
    if (period_time_changed) {
        period_time_changed = 0;
        period_time_copy = period_time;
        prev_period_copy = 0;
    }
    interrupts();

    if (period_time_copy != prev_period_copy) {
        Serial.println(period_time_copy / 10000.);
        prev_period_copy = period_time_copy;
    }

    // Every five seconds change frequency
    static unsigned long prev_change_time = millis();
    static bool on_off = 0;
    if (millis() - prev_change_time > 5000) {
        prev_change_time = millis();
        Timer1.setPeriod(13700 + 3670 * on_off);
        on_off = !on_off;
    }

    delay(1);
}
