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
        pros::Motor input(18, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor storage(17, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor output(19, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

    // Sensors
        pros::Rotation parallelLeftOdom(12);
        pros::Rotation parallelRightOdom(13);
        pros::Rotation perpOdom(11);

        pros::IMU inertial1(14);
        pros::IMU inertial2(15);

// Program Module Initialization

    HoloChassis chassis = HoloChassis({&topLeft6, &topLeft2}, {&topRight6, &topRight2}, {&bottomLeft6, &bottomLeft2}, {&bottomRight6, &bottomRight2});

    OdomPod leftOdom(&parallelLeftOdom, 2);
    OdomPod rightOdom(&parallelRightOdom, 2);
    TrackingSensor fbOdom(
        []() -> double {
            std::cout << "got value from odom\n";
            return ((leftOdom.measure() + rightOdom.measure()) / 2);
        },
        [](double val) {
            std::cout << "reached odom reset\n";
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

    /* Odometry odom(fbOdom, headingTracker, startPose, lrOdom);
    odom.updateLoop();

    double distSinceLastReset = 0;
    TrackingSensor xOdom(
        [](){
            return odom.m_robotPose.
        }
    ); */

    ConstantContainer xConstants = {4, 0.1, 2.7};
    ConstantContainer yConstants = {4, 0.1, 2.7};
    ConstantContainer thetaConstantsSub90 = {3, 0.2, 26};
    ConstantContainer thetaConstantsAbove90 = {2.3, 0.24, 32};

    double xTol = 1;
    double yTol = 1;
    double thetaTolSub90 = 2.5;
    double thetaTolAbove90 = 3.5;

    PIDController xPID(fbOdom, xConstants, chassis.m_xOutputCorrect, xTol);
    PIDController yPID(lrOdom, yConstants, chassis.m_yOutputCorrect, yTol);
    PIDController thetaPIDSub90(headingTracker, thetaConstantsSub90, chassis.m_thetaOutputCorrect, thetaTolSub90);
    PIDController thetaPIDAbove90(headingTracker, thetaConstantsAbove90, chassis.m_thetaOutputCorrect, thetaTolAbove90);
    








    


#endif