#include "z-config.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	inertial1.set_heading(270);
	pros::delay(3000);
    Kalman1.startFilter();
    Kalman2.startFilter();
	parallelLeftOdom.set_position(0);
	parallelRightOdom.set_position(0);
	perpOdom.set_position(0);
	odom.updateLoop();
	chassis.addPID(&xPID, &yPID);
	mcl.start();
	master.print(0, 0, "Initialized!");
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
	chassis.continuousPower();
	chassis.brakeMode(pros::v5::MotorBrake::hold);
	
	CubicHermiteSpline testSpline = CubicHermiteSpline({0, 0}, {0, 60}, {30, 30}, {60, 30});
	MotionProfile* testProfile = new MotionProfile(&testSpline, RPMtoIPS(480), {0, 0}, {0, 0.9});
	/*for (int i = 0; i < testProfile->holoProfile.size(); i++) {
		std::cout << "x = " << testProfile->holoProfile[i].x << ", y = " << testProfile->holoProfile[i].y;
		std::cout << ", heading = " << testProfile->holoProfile[i].heading;
		std::cout << ", lvel.x = " << testProfile->holoProfile[i].linVel.x << ", lvel.y = " << testProfile->holoProfile[i].linVel.y << ", lvel.mag = " << testProfile->holoProfile[i].linVel.magnitude << ", avel = " << testProfile->holoProfile[i].angVel << "\n";
	}*/
	// follower.startProfile(testProfile, false);
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
	autonomous();
	master.rumble("-.-");

	int deadzone = 15;

	while (true) {
	// Mecanum Drive Control
		chassis.driverControl(master, deadzone);
		chassis.move();


	// Intake Control
	// Long Goal
	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		input.move(128);
		storage.move(-128);
		output.move(-128);
	} 
	// High Center Goal
	else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
		input.move(96);
		storage.move(-96);
		output.move(96);
	} 
	// Storage
	else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		input.move(128);
		storage.move(128);
		output.move(128);
	}
	// Low Center Goal
	else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
		input.move(-80);
		storage.move(-80);
		output.brake();
	}
	else {
		input.brake();
		storage.brake();
		output.brake();
	}
	// color sort
	if (((master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) || (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) 
		|| (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) || (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)))
		&&
		(((colorEnabled == 1) && (color.get_hue() > 180)) // remove blue
		|| ((colorEnabled == 2) && (color.get_hue() < 35)))) // remove red
	{
		input.move(128);
		storage.brake();
		output.move(-128);
	}

	// color sort toggle
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		colorEnabled++;
		if (colorEnabled >= 2) {colorEnabled = 0;}
	}

	// Unloader Mech
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
		unloader.set_value(!unloader.get_value());
	}

	// Aligner Mech
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
		aligner.set_value(!aligner.get_value());
	}

	// Descorer Mech
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
		aligner.set_value(!descorer.get_value());
	}

	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
		std::cout << "f: " << frontDistance.get() << ", l: " << leftDistance.get() << ", r: " << rightDistance.get() << "\n";
	}

	

	pros::delay(20);
	}
}