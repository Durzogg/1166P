#ifndef _PROS_INIT_H_
#define _PROS_INIT_H_

#include "main.h"

// Controllers
    extern pros::Controller master;

// Motors
    // Drivetrain 
        extern pros::Motor topLeft6;
        extern pros::Motor bottomLeft6;
        extern pros::Motor topRight6;
        extern pros::Motor bottomRight6;

        extern pros::Motor topLeft2;
        extern pros::Motor bottomLeft2;
        extern pros::Motor topRight2;
        extern pros::Motor bottomRight2;

        // Drivetrain Groups
        extern pros::MotorGroup trbl6;
        extern pros::MotorGroup tlbr6;

        extern pros::MotorGroup trbl2;
        extern pros::MotorGroup tlbr2;

        extern pros::MotorGroup all6;
        extern pros::MotorGroup all2;

    // Intake
        extern pros::Motor inputLeft;
        extern pros::Motor inputRight;

// Global Values
    extern double g_gearRatio6;
    extern double g_gearRatio2;
    extern double g_maxRPM;
    extern double g_diameter;
    extern double g_distBetweenWheels;


#endif