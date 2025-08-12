#ifndef _PROS_CONFIG_H_
#define _PROS_CONFIG_H_

#include "main.h"
#include "chassis.h"
#include "interface.h"
#include "odom.h"
#include "pid.h"
#include "profiling.h"
#include "kalman.h"
#include "math.h"
#include "mcl.h"

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

        pros::Distance front(12);
        pros::Distance left(11);
        pros::Distance right(13);

        pros::IMU inertial1(14);
        pros::IMU inertial2(15);

    // Three-Wire Devices
        pros::adi::DigitalOut unloader(8);

// Program Module Initialization

    HoloChassis chassis = HoloChassis({&topLeft6, &topLeft2}, {&topRight6, &topRight2}, {&bottomLeft6, &bottomLeft2}, {&bottomRight6, &bottomRight2});

    OdomPod leftOdom(&parallelLeftOdom, 2);
    OdomPod rightOdom(&parallelRightOdom, 2);
    TrackingSensor fbTrack(
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
        }
    );

    OdomPod perpendicularOdom = OdomPod(&perpOdom, 2);
    TrackingSensor lrTrack(
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

    KalmanFilter Kalman1 = KalmanFilter(&inertial1, angVelTracker);
    KalmanFilter Kalman2 = KalmanFilter(&inertial2, angVelTracker);

    TrackingSensor headingTracker(
        []() -> double {
            return Kalman1.getFilteredHeading();
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
            lastHeading = getAggregatedHeading(Kalman1, Kalman2);
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

    Pose startPose = {0, 0, 0};

    Odometry odom(fbTrack, headingTracker, startPose, lrTrack);

    ConstantContainer xConstants = {4, 0.1, 2.7};
    ConstantContainer yConstants = {4, 0.1, 2.7};
    ConstantContainer thetaConstantsSub90 = {3, 0.2, 26};
    ConstantContainer thetaConstantsAbove90 = {2.3, 0.24, 32};

    double xTol = 1;
    double yTol = 1;
    double thetaTolSub90 = 2.5;
    double thetaTolAbove90 = 3.5;

    PIDController xPID(fbTrack, xConstants, chassis.m_xOutputCorrect, xTol);
    PIDController yPID(lrTrack, yConstants, chassis.m_yOutputCorrect, yTol);
    PIDController thetaPIDSub90(PIDHeadingTracker, thetaConstantsSub90, chassis.m_thetaOutputCorrect, thetaTolSub90);
    PIDController thetaPIDAbove90(PIDHeadingTracker, thetaConstantsAbove90, chassis.m_thetaOutputCorrect, thetaTolAbove90);

    PIDSet robotPIDs(&xPID, &yPID, &thetaPIDSub90, &thetaPIDAbove90);
    PoseTracker currentPose(&odom);

    TrackingSensor frontDistance(
        []() -> double {
            double val = front.get();
            return val == 9999 ? -1 : (val / 10) / 2.54;
        }
    );

    TrackingSensor leftDistance(
        []() -> double {
            double val = left.get();
            return val == 9999 ? -1 : (val / 10) / 2.54;
        }
    );

    TrackingSensor rightDistance(
        []() -> double {
            double val = right.get();
            return val == 9999 ? -1 : (val / 10) / 2.54;
        }
    );

    TrackingSensor linAngleTracker(
        []() -> double {
            double offsetFromHeading = chassis.m_lAng.get() < 180 ? chassis.m_lAng.get() : chassis.m_lAng.get() - 360;
            return headingTracker.get() + offsetFromHeading;
        }
    );

    VelocityController follower(&chassis.m_xOutput, &chassis.m_yOutput, &chassis.m_thetaOutput, &currentPose, robotPIDs);

    ParticleFilter mcl(frontDistance, leftDistance, rightDistance, headingTracker, 
                       chassis.m_lVel, linAngleTracker, chassis.m_aVel, 
                       {{-4, 7.5}, {-7.5, 0}, {7.5, 0}}, {0, -90, 90});

#endif