// Kinematics.h

#include <BasicLinearAlgebra.h>

#ifndef KINEMATICS_H
#define KINEMATICS_H

class Kinematics {
public:
    virtual BLA::Matrix<3> calculate_robot_velocity(const BLA::Matrix<4>& wheel_velocity) = 0;
    virtual BLA::Matrix<4> calculate_wheel_velocity(const BLA::Matrix<3>& robot_velocity) = 0;
};

class MecanumKinematics4W : public Kinematics {
public:
    MecanumKinematics4W(double wheel_radius, double wheel_base, double track_width);

    BLA::Matrix<3> calculate_robot_velocity(const BLA::Matrix<4>& wheel_velocity) override;
    BLA::Matrix<4> calculate_wheel_velocity(const BLA::Matrix<3>& robot_velocity) override;

private:
    double wheel_radius_; // radius of wheels
    double wheel_base_; // distance between wheel contact point in x direction
    double track_width_; // distance between wheel contact point in y direction
};

// TODO: add SwerveKinematics3W implementation

#endif // KINEMATICS_H
