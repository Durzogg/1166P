#include "init.h"
#include "power-inl.h"
#include "chassis.h"

// Controllers
    pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motors
    // X-Drivetrain
        /*
        pros::Motor topLeft(13, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor bottomLeft(11,pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor topRight(-8, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor bottomRight(-17, pros::v5::MotorGears::blue, pros::v5::MotorEncoderUnits::degrees);
        */

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

        HoloChassis chassis = HoloChassis({topLeft6, topLeft2}, {bottomLeft6, bottomLeft2}, {topRight6, topRight2}, {bottomRight6, bottomRight2});
        
    // Intake
        pros::Motor inputLeft(11, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
        pros::Motor inputRight(5, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);


// Global Values
        double g_gearRatio6 = 0.8;
        double g_gearRatio2 = 3;
        double g_maxRPM = 480;
        double g_diameter = 4;
    
        