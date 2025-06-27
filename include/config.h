#ifndef _PROS_CONFIG_H_
#define _PROS_CONFIG_H_

#include "main.h"
#include "chassis.h"
#include "proxy.h"
#include "config.h"
#include "odom.h"
#include "chassis.h"
#include "pid.h"

// Controllers
    pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motors
    // Mecanum Drivetrain
        
        pros::Motor topLeft6(-6, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor topLeft2(-7, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor bottomLeft6(-8,pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor bottomLeft2(-9, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor topRight6(1, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor topRight2(2, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor bottomRight6(3, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor bottomRight2(4, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        
    // Intake
        pros::Motor inputLeft(11, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor inputRight(5, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

    // Sensors
        pros::Rotation parallelLeftOdom(99);
        pros::Rotation parallelRightOdom(99);
        pros::Rotation perpOdom(99);

        pros::IMU inertial1(99);
        pros::IMU inertial2(99);

// Program Module Initialization

    HoloChassis chassis = HoloChassis({&topLeft6, &topLeft2}, {&topRight6, &topRight2}, {&bottomLeft6, &bottomLeft2}, {&bottomRight6, &bottomRight2});

    OdomPod leftOdom(&parallelLeftOdom, 2);
    OdomPod rightOdom(&parallelRightOdom, 2);
    TrackingSensor fbOdom(
        []() -> double {
            return ((leftOdom.measure() + rightOdom.measure()) / 2);
        },
        [](double val) {
            parallelLeftOdom.set_position(val);
            parallelRightOdom.set_position(val);
            return;
        },
        []() {
            parallelLeftOdom.reset_position();
            parallelRightOdom.reset_position();
            return;
        }
    );
    TrackingSensor angVelTracker(
        []() -> double {
            return ((leftOdom.measureVelocity() - rightOdom.measureVelocity()) / 5);
        },
        [](double val) {
            return;
        },
        []() {
            return;
        }
    );

    OdomPod perpendicularOdom = OdomPod(&perpOdom, 2);
    TrackingSensor lrOdom(
        []() -> double {
            return perpendicularOdom.measure();
        },
        [](double val) {
            perpOdom.set_position(val);
            return;
        },
        []() {
            perpOdom.reset_position();
            return;
        }
    );

    KalmanFilter Kalman1(&inertial1, angVelTracker);
    KalmanFilter Kalman2(&inertial2, angVelTracker);
    TrackingSensor headingTracker(
        []() -> double {
            return getAggregatedHeading(Kalman1, Kalman2);
        },
        [](double val) {
            return;
        },
        []() {
            return;
        }
    );

    double distFromLastReset = 0;
    double lastHeading = 0;
    TrackingSensor PIDHeadingTracker(
        []() -> double {
            double changeInHeading = getAggregatedHeading(Kalman1, Kalman2) - lastHeading;
            if (changeInHeading > 315) {
                changeInHeading -= 360;
            } else if (changeInHeading < -315) {
                changeInHeading += 360;
            }
            distFromLastReset += changeInHeading;
            return distFromLastReset;
        },
        [](double val) {
            return;
        },
        []() {
            distFromLastReset = 0;
            return;
        }
    );

    PowerUnit xPower(
        [](double power) {
            chassis.setX(power);
        },
        []() {
            chassis.setX(0);
        }
    );

    PowerUnit yPower(
        [](double power) {
            chassis.setY(power);
        },
        []() {
            chassis.setY(0);
        }
    );

    PowerUnit thetaPower(
        [](double power) {
            chassis.setTheta(power);
        },
        []() {
            chassis.setTheta(0);
        }
    );

    Pose startPose = {0, 0, 0};

    Odometry odom(fbOdom, headingTracker, {0, 0, 0}, lrOdom);
    ;

    ConstantContainer xConstants = {0, 0, 0};
    ConstantContainer yConstants = {0, 0, 0};
    ConstantContainer thetaConstants = {0, 0, 0};

    double xTol = 0;
    double yTol = 0;
    double thetaTol = 0;

    PIDController xPID(fbOdom, xConstants, chassis.m_xOutputCorrect, xTol);
    PIDController yPID(lrOdom, yConstants, chassis.m_yOutputCorrect, yTol);
    PIDController thetaPID(headingTracker, thetaConstants, chassis.m_thetaOutputCorrect, thetaTol);
    








    


#endif