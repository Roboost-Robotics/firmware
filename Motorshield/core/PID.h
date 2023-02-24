class PIDController
{
public:
    // Constructor
    PIDController(double kp, double ki, double kd)
        : Kp(kp), Ki(ki), Kd(kd), prevError(0), integral(0) {}

    // Compute the control signal using the PID algorithm
    double computeControlSignal(double error, double dt)
    {
        // Proportional term
        double p = Kp * error;

        // Integral term
        integral += error * dt;
        double i = Ki * integral;

        // Derivative term
        double d = Kd * (error - prevError) / dt;

        // Save the current error for the next iteration
        prevError = error;

        // Return the sum of the three terms
        return p + i + d;
    }

private:
    double Kp, Ki, Kd;
    double prevError;
    double integral;
}; 