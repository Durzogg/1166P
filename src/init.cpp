#include "init.h"

//Controllers
    pros::Controller master(pros::E_CONTROLLER_MASTER);



    //Motors
        // X-Drivetrain
            /*
            pros::Motor topLeft(13, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor bottomLeft(11,pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor topRight(-8, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor bottomRight(-17, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
            */

        // Mecanum Drivetrain
            
            pros::Motor topLeft(1, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor bottomLeft(6,pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor topRight(-5, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor bottomRight(-4, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
            
            