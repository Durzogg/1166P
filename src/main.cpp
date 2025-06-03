#include "init.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	master.rumble("-.-");
	

	// Front, Middle, Rear
	topLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	bottomLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	topRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	bottomRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	// Driving variables
	int drvfb;
	int drvlr;
	int drvtrdz = 10;

	while (true) {
	/*
	// Differential Drivetrain Control 
		drvfb = master.get_analog(ANALOG_LEFT_Y);
		drvlr = master.get_analog(ANALOG_RIGHT_X);

		if ((abs(drvfb) > drvtrdz) || (abs(drvlr) > drvtrdz)) {
      		// ^^ Checks to see if either joystick has moved out of the deadzMasterone
			rightDrivetrain.move((drvfb-(drvlr)));
      		leftDrivetrain.move((drvfb+(drvlr)));	
    	} else {
			rightDrivetrain.brake();
      		leftDrivetrain.brake();
    	} 
	*/
	// X-Drive Control
		std::cout << "x:" << master.get_analog(ANALOG_LEFT_X) << "\n";
		std::cout << "y:" << master.get_analog(ANALOG_LEFT_Y) << "\n\n";

	pros::delay(20);
	}
}