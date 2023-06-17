class PID
{
public:
    // Constructor
    PID(double kp, double ki, double kd)
        : Kp(kp), Ki(ki), Kd(kd), prevError(0), integral(0) {}

    // Compute the control signal using the PID algorithm
    double computeControlSignal(double error, double dt);

private:
    double Kp, Ki, Kd;
    double prevError;
    double integral;
}; 