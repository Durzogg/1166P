#ifndef _PROS_CONFIG_H_
#define _PROS_CONFIG_H_

#include "main.h"
#include "chassis.h"
#include "tracking-inl.h"
#include "config.h"
#include "power-inl.h"
#include "odom.h"
#include "chassis.h"

// Controllers
    pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motors
    // Mecanum Drivetrain
        
        pros::Motor topLeft6(1, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor topLeft2(99, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor bottomLeft6(6,pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor bottomLeft2(99, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor topRight6(-5, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor topRight2(99, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

        pros::Motor bottomRight6(-4, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor bottomRight2(99, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

        // common groupings for easy access
        pros::MotorGroup tlbr6({1, -4}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::MotorGroup trbl6({-5, 6}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);

        pros::MotorGroup tlbr2({99, 99}, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        pros::MotorGroup trbl2({99, 99}, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);

        pros::MotorGroup all6({1, -4, -5, 6}, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::MotorGroup all2({99, 99, 99, 99}, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        
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

    HoloChassis chassis = HoloChassis({topLeft6, topLeft2}, {bottomLeft6, bottomLeft2}, {topRight6, topRight2}, {bottomRight6, bottomRight2});

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
            return ((leftOdom.measureVelocity() - rightOdom.measureVelocity()));
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

    KalmanFilter Kalman1(&inertial1,);









// Global Values
    extern double g_gearRatio6;
    extern double g_gearRatio2;
    extern double g_maxRPM;
    extern double g_diameter;
    extern double g_distBetweenWheels;

    // Global Values
    double g_gearRatio6 = 0.8;
    double g_gearRatio2 = 3;
    double g_maxRPM = 480;
    double g_diameter = 4;
    


#endif