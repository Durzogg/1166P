#include "init.h"

//Controllers
    pros::Controller master(pros::E_CONTROLLER_MASTER);



    //Motors
        //Drivetrain
            pros::Motor topLeft(-8, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor bottomLeft(-9,pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor topRight(-10, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);
            pros::Motor bottomRight(-10, pros::v5::MotorGears::green, pros::v5::MotorEncoderUnits::degrees);