#include <Arduino.h>

IntervalTimer fastTimer, slowTimer;
volatile bool ledState = false; // Track state of LED
volatile long counter = 0;

void fastISR()
{
    ledState = !ledState;
    // wrap around the counter
    counter == 0x7FFFFFFF ? counter = 0 : counter++;
}

void slowISR()
{
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Toggle LED

    noInterrupts();
    long counterCopy = counter;
    interrupts();
    Serial.println(counterCopy);
}

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW); // Ensure LED starts in a known state

    // Configure the timer
    fastTimer.begin(fastISR, 1000); // Interval in microseconds (1 ms)
    fastTimer.priority(255);        // Set the highest priority

    slowTimer.begin(slowISR, 1000000); // Interval in microseconds (1 s)
    slowTimer.priority(0);             // Set the lowest priority
}

void loop()
{
    // Nothing to do here
}
