#include "z-config.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::delay(3000);
	inertial1.set_heading(90);
	inertial2.set_heading(90);
    Kalman1.startFilter();
    Kalman2.startFilter();
	//parallelLeftOdom.set_position(0);
	//parallelRightOdom.set_position(0);
	// perpOdom.set_position(0);
	// odom.updateLoop();
	// chassis.addPID(&fbPID, &thetaPIDAbove90);
	// mcl.start();
	color.set_led_pwm(100);
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

//hif
	// autonomous setup

	input.tare_position();
	storage.tare_position();
	output.tare_position();

	int autonnumber = 2;


	switch (autonnumber) {
		case 1:
		case -1:
			// forward, turn to block, take block
			bPID.movement(11.5);
			thetaPID(makeRelative(-10))->movement(makeRelative(-10));
			input.move(128);
			storage.move(128);
			output.move(128);
			fPID.movement(-10);
			pros::delay(500);
			// turn to goal, move to goal, back up to align
			thetaPID(makeRelative(110))->movement(makeRelative(110));
			chassis.setFB(40);
			pros::delay(1500);
			chassis.setFB(-40);
			pros::delay(250);
			chassis.setFB(0);
			pros::delay(200);
			// score in goal
			aligner.set_value(true);
			input.move(128);
			storage.move(-128);
			output.move(-128);
			pros::delay(6000);
			// ending back-up
			chassis.setFB(-40);
			pros::delay(250);
			chassis.setFB(0);
			break;
		case 2:
		case -2:
			// forward, turn to block, take block
			bPID.movement(11.5);
			thetaPID(makeRelative(165))->movement(makeRelative(165));
			input.move(128);
			storage.move(128);
			output.move(128);
			fPID.movement(-12);
			pros::delay(500);
			// turn to goal, move to goal
			thetaPID(makeRelative(-40))->movement(makeRelative(-40));
			chassis.setFB(40);
			pros::delay(1500);
			chassis.setFB(0);
			pros::delay(200);
			// score in goal
			input.move(-60);
			storage.move(-60);
			output.brake();
			pros::delay(6000);
			// ending back-up
			chassis.setFB(-40);
			pros::delay(250);
			chassis.setFB(0);
			break;
		case 3:
		case -3:
			// test
			break;
	}
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

	chassis.brake();

	int deadzone = 15;

	int colorDelay = 0;
	int colorEnabled = 0;

	master.print(0, 0, "color = %d", colorEnabled);

	while (true) {
	// Differential Drive Control
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
		(((colorEnabled == 1) && (color.get_hue() > 60)) // remove blue
		|| ((colorEnabled == 2) && (color.get_hue() < 20)))) // remove red
	{
		colorDelay = 10;
	}
	if (colorDelay > 0) {
		colorDelay--;
		input.move(128);
		if (colorDelay > 5) {storage.move(-128);}
		output.move(-128);
	}


	// color sort toggle
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
		colorEnabled++;
		if (colorEnabled > 2) {colorEnabled = 0;}

		switch (colorEnabled) {
			case 0:
				master.print(0, 0, "remove none");
			case 1:
				master.print(0, 0, "remove blue");
			case 2:
				master.print(0, 0, "remove rhed");

		}
	}

	// Unloader Mech
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
		unloader.set_value(!unloader.get_value());
	}

	// Aligner Mech
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		aligner.set_value(!aligner.get_value());
	}

	// Descorer Mech
	if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
		descorer.set_value(!descorer.get_value());
	}

	pros::delay(20);
	}
}