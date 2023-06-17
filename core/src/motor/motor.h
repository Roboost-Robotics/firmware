/// @brief 

// TODO: make virtual, implementations should be specific motor applications (like HBridge or VESC)
class Motor
{
public:
    Motor(const int PWM_PIN, const int CW_PIN, const int CCW_PIN)
        : PWM_PIN(PWM_PIN), CW_PIN(CW_PIN), CCW_PIN(CCW_PIN) {}

private:
    const int PWM_PIN;
    const int CW_PIN;
    const int CCW_PIN;

    double rotationSpeed; // rad/s
    double dutyCycle; // 0 - 100

}