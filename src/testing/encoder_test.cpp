#include <Arduino.h>
#include <conf_hardware.h>
#include <roboost/motor_control/motor_drivers/motor_driver.hpp>

#define MAX_ENCODERS 16
#define US_DEBOUNCE 10
#define ENCODER_RESOLUTION 360
#define VELOCITY_TIMEOUT 100000

#ifdef ESP32
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
#define _ENTER_CRITICAL() portENTER_CRITICAL_SAFE(&spinlock)
#define _EXIT_CRITICAL() portEXIT_CRITICAL_SAFE(&spinlock)
#else
#define _ENTER_CRITICAL() noInterrupts()
#define _EXIT_CRITICAL() interrupts()
#endif

class InterruptEncoder
{
public:
    enum precision_level_t
    {
        LOW_PRECISION = 1,    // 1 * PPR
        MEDIUM_PRECISION = 2, // 2 * PPR
        HIGH_PRECISION = 2    // 2 * PPR, as both edges are used
    };

    struct config_t
    {
        precision_level_t precision;
        int pin_a;
        int pin_b;
        int encoder_resolution;
    };

private:
    bool attached = false;
    config_t config;

public:
    InterruptEncoder(const config_t& config);
    virtual ~InterruptEncoder();
    void attach();
    volatile int64_t count = 0;
    volatile int64_t last_time = 0;
    volatile int64_t micros_between_ticks = 0;
    volatile int direction = 0; // 1 for clockwise, -1 for counterclockwise

    int64_t read();
    int get_direction();
    int64_t get_micros_between_ticks();
    int64_t get_position_ticks(); // Position in ticks

    // Helper methods to convert ticks to radians and velocity to radians per second
    float ticks_to_radians(int64_t ticks, bool wrap = true);
    float ticks_per_microsecond_to_radians_per_second(int64_t ticks_per_microsecond);

    void update();
    void update_single();
    void update_dual();
};

// ISR for LOW and MEDIUM precision levels
void IRAM_ATTR encoder_isr_single(void* arg)
{
    InterruptEncoder* object = (InterruptEncoder*)arg;
    object->update_single();
}

// ISR for HIGH precision level
void IRAM_ATTR encoder_isr_dual(void* arg)
{
    InterruptEncoder* object = (InterruptEncoder*)arg;
    object->update_dual();
}

InterruptEncoder::InterruptEncoder(const config_t& config) : config(config) {}

InterruptEncoder::~InterruptEncoder()
{
    if (attached)
    {
        detachInterrupt(digitalPinToInterrupt(config.pin_a));
        if (config.precision == HIGH_PRECISION)
        {
            detachInterrupt(digitalPinToInterrupt(config.pin_b));
        }
    }
}

// This method is used for LOW and MEDIUM precision levels
void InterruptEncoder::update_single()
{
    unsigned long current_time = micros();
    if (current_time - last_time < US_DEBOUNCE)
    {
        return;
    }

    _ENTER_CRITICAL(); // Enter critical section
    bool state_a = digitalRead(config.pin_a);
    bool state_b = digitalRead(config.pin_b);

    if (state_a)
    {
        if (state_a == state_b)
        {
            count++;
            direction = 1; // Clockwise
        }
        else
        {
            count--;
            direction = -1; // Counterclockwise
        }

        micros_between_ticks = current_time - last_time;
        last_time = current_time;
    }

    _EXIT_CRITICAL(); // Exit critical section
}

void InterruptEncoder::update_dual()
{
    // Based on https://makeatronics.blogspot.com/2013/02/efficiently-reading-quadrature-with.html
    unsigned long current_time = micros();
    if (current_time - last_time < US_DEBOUNCE)
    {
        return;
    }

    static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
    static uint8_t enc_val = 0;

    _ENTER_CRITICAL(); // Enter critical section

    // Shift the previous state to the left by 2 bits
    enc_val = enc_val << 2;

    // Read the current state of the pins and combine with previous state
    enc_val = enc_val | ((digitalRead(config.pin_a) << 1) | digitalRead(config.pin_b));

    // Mask to keep only the last 4 bits
    enc_val = enc_val & 0b1111;

    // Update the count based on the look-up table
    count += lookup_table[enc_val];
    direction = (lookup_table[enc_val] > 0) ? 1 : (lookup_table[enc_val] < 0) ? -1 : direction;

    // Update timing only when the count changes
    if (lookup_table[enc_val] != 0)
    {
        micros_between_ticks = current_time - last_time;
        last_time = current_time;
    }

    _EXIT_CRITICAL(); // Exit critical section
}

int64_t InterruptEncoder::read()
{
    _ENTER_CRITICAL();
    int64_t current_count = count;
    _EXIT_CRITICAL();
    return current_count;
}

int InterruptEncoder::get_direction()
{
    _ENTER_CRITICAL();
    int current_direction = direction;
    _EXIT_CRITICAL();
    return current_direction;
}

int64_t InterruptEncoder::get_micros_between_ticks()
{
    _ENTER_CRITICAL();
    int64_t current_micros_between_ticks = micros_between_ticks;
    _EXIT_CRITICAL();
    return current_micros_between_ticks;
}

int64_t InterruptEncoder::get_position_ticks()
{
    _ENTER_CRITICAL();
    int64_t current_position_ticks = count;
    _EXIT_CRITICAL();
    return current_position_ticks;
}

float InterruptEncoder::ticks_to_radians(int64_t ticks, bool wrap)
{
    float resolution_multiplier = (config.precision == LOW_PRECISION) ? 1 : (config.precision == MEDIUM_PRECISION) ? 2 : 4;
    float radians = (float(ticks) / (config.encoder_resolution * resolution_multiplier)) * 2.0 * PI;
    return wrap ? fmod(radians, 2.0 * PI) : radians;
}

float InterruptEncoder::ticks_per_microsecond_to_radians_per_second(int64_t ticks_per_microsecond)
{
    float resolution_multiplier = (config.precision == LOW_PRECISION) ? 1 : (config.precision == MEDIUM_PRECISION) ? 2 : 4;
    return (float(ticks_per_microsecond) / (config.encoder_resolution * resolution_multiplier)) * 2.0 * PI * 1'000'000;
}

void InterruptEncoder::attach()
{
    if (attached)
        return;
    pinMode(config.pin_a, INPUT_PULLUP);
    pinMode(config.pin_b, INPUT_PULLUP);
    Serial.print(">attached:");
    if (config.precision == LOW_PRECISION)
    {
        attachInterruptArg(digitalPinToInterrupt(config.pin_a), encoder_isr_single, this, RISING);
        Serial.print(0);
    }
    else if (config.precision == MEDIUM_PRECISION)
    {
        attachInterruptArg(digitalPinToInterrupt(config.pin_a), encoder_isr_single, this, CHANGE);
        Serial.print(1);
    }
    else // HIGH
    {
        attachInterruptArg(digitalPinToInterrupt(config.pin_a), encoder_isr_dual, this, CHANGE);
        attachInterruptArg(digitalPinToInterrupt(config.pin_b), encoder_isr_dual, this, CHANGE);
        Serial.print(2);
    }
    attached = true;
}

InterruptEncoder::config_t config = {InterruptEncoder::MEDIUM_PRECISION, M3_ENC_A, M3_ENC_B, ENCODER_RESOLUTION};
InterruptEncoder my_encoder(config);
roboost::motor_control::L298NMotorDriver motor_driver = {M3_IN1, M3_IN2, M3_ENA, M3_PWM_CNL};

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    my_encoder.attach();
    motor_driver.set_motor_control(-(1 << (roboost::motor_control::PWM_RESOLUTION - 1)));
}

void loop()
{
    int64_t count = my_encoder.read();
    int direction = my_encoder.get_direction();
    int64_t position_ticks = my_encoder.get_position_ticks();
    int64_t micros_between_ticks = my_encoder.get_micros_between_ticks();

    static long last_time = 0;
    float velocity = 0;
    long current_time = micros();
    if (current_time - my_encoder.last_time < VELOCITY_TIMEOUT)
    {
        if (micros_between_ticks > 0)
        {
            velocity = 1'000'000.f * config.precision / float(micros_between_ticks); // Ticks per second
            velocity *= direction;                                                   // Direction
        }
    }

    Serial.print(">dt:");
    Serial.println(micros_between_ticks);
    Serial.print(">count:");
    Serial.println(count);
    Serial.print(">velocity:");
    Serial.println(velocity, 6);
    Serial.print(">direction:");
    Serial.println(direction);
    Serial.print(">position (ticks):");
    Serial.println(position_ticks);

    // Optionally print radians and radians per second using helper methods
    Serial.print(">position (radians):");
    Serial.println(my_encoder.ticks_to_radians(position_ticks), 6);
    Serial.print(">velocity (radians/s):");
    Serial.println(my_encoder.ticks_to_radians(velocity, false), 6);

    Serial.print(">encoder_timeout:");
    Serial.println(current_time - my_encoder.last_time);

    Serial.print(">loop_time:");
    Serial.println(current_time - last_time);

    last_time = current_time;

    delay(50);
}
