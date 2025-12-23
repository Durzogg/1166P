#include "z-config-r2.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::delay(3000);
	inertial1.set_heading(startPose.heading);
	inertial2.set_heading(startPose.heading);
    Kalman1.startFilter();
    Kalman2.startFilter();
	parallelLeftOdom.set_position(0);
	parallelRightOdom.set_position(0);
	// perpOdom.set_position(0);
	odom.updateLoop();
	chassis.addPID(&fbPID, &thetaPIDAbove90);
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

	stage1.tare_position();
	stage2.tare_position();
	stage3.tare_position();


	switch (autonnumber) {
		case 1:
		case -1:
			// push Alliance Partner and intake their pre-load
			stage1.move(-128);
			stage2.move(-128);
			chassis.move_relative(5, 600, false);
			
			// move to Loader and intake from Loader
			std::cout << currentPose.get().heading << "\n";
			// chassis.moveToPoint({46, -47.35}, false, true);
			
			thetaPID(makeRelative(90))->movement(makeRelative(90));
			/*
			loader.set_value(true);
			pros::delay(50);
			chassis.move_relative(10, 600, false);
			*/
			/*
			// move to Long Goal and score all prior blocks in goal
			chassis.move_relative(-30, 600, false);
			stage1.move(-128);
			stage2.move(-128);
			stage3.move(-128);
			pros::delay(3000);
			chassis.moveToPoint({});
			stage3.brake();

			// grab mid-Blocks across field, then score into low goal
			chassis.moveToPoint({});
			chassis.moveToPoint({});
			chassis.moveToPoint({10, 10});
			stage1.move(128);
			stage2.move(128);
			stage3.move(128);
			pros::delay(3000);


			// back up to other Loader and face it, then grab from it
			chassis.moveToPoint({});
			thetaPIDSub90.movement(90);
			loader.set_value(true);
			stage1.move(-128);
			stage2.move(-128);
			pros::delay(50);
			chassis.move_relative(10, 600, false);

			// score in Long Goal 2
			chassis.move_relative(-30, 600, false);
			stage1.move(-128);
			stage2.move(-128);
			stage3.move(-128);
			pros::delay(3000);
			chassis.moveToPoint({});
			stage3.brake();
			*/
			break;
		case 2:
			chassis.move_relative(27, 375, false);
			loader.set_value(true);
			chassis.m_thetaOutput.move(-10);
			manualTurn(290, 15);
			chassis.m_thetaOutput.stop();
			chassis.m_fbOutput.move(65);
			stage1.move(-128);
			stage2.move(-128);
			pros::delay(2000);
			chassis.m_fbOutput.stop();
			pros::delay(4000);
			chassis.brake();
			chassis.m_fbOutput.move(-100);
			pros::delay(1000);
			stage1.move(-128);
			stage2.move(-128);
			stage3.move(-128);
			pros::delay(10000);
			break;
		case -2:
			chassis.move_relative(30, 375, false);
			loader.set_value(true);
			chassis.m_thetaOutput.move(9.3);
			manualTurn(255, 15);
			chassis.m_thetaOutput.stop();
			chassis.m_fbOutput.move(40);
			stage1.move(-128);
			stage2.move(-128);
			pros::delay(1000);
			chassis.m_fbOutput.stop();
			pros::delay(2000);
			chassis.brake();
			chassis.m_fbOutput.move(-80);
			pros::delay(1000);
			stage1.move(-128);
			stage2.move(-128);
			stage3.move(-128);
			pros::delay(2500);
			chassis.m_fbOutput.stop();
			chassis.move_relative(19.5, 400, false);
			stage3.brake();
			chassis.m_thetaOutput.move(10);
			manualTurn(40, 15);
			chassis.m_thetaOutput.stop();
			loader.set_value(false);
			chassis.move_relative(70, 200, false);
			pros::delay(5000);
			break;
		case 3:
		case -3:
			stage1.move(-128);
			stage2.move(-128);
			stage3.move(-128);
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
	master.rumble("-.-");

	chassis.continuousPower();
	chassis.brake();
	chassis.brakeMode(pros::v5::MotorBrake::coast);

	int deadzone = 15;

	int colorDelay = 0;
	int colorEnabled = 0;

	master.print(0, 0, "color = %d", colorEnabled);

	while (true) {

		std::cout << "{" << currentPose.get().x << ", " << currentPose.get().y << "}\n";
	// Differential Drive Control
		chassis.driverControl(master, deadzone);


			std::cout << currentPose.get().heading << "\n";
	// Intake Control
	// Long Goal
	if (master.get_digital(INTAKE_ALL_IN)) {
		stage1.move(-128);
		stage2.move(-128);
		stage3.move(-128);
		// ramp.set_value(true);
	} 
	// High Center Goal
	else if (master.get_digital(INTAKE_DROP)) {
		stage1.move(-128);
		stage2.move(-128);
		stage3.move(-128);
		ramp.set_value(false);
	}
	// stage2
	else if (master.get_digital(INTAKE_HOLD)) {
		stage1.move(-128);
		stage2.move(-128);
		stage3.brake();
	}
	// Low Center Goal
	else if (master.get_digital(INTAKE_ALL_OUT)) {
		stage1.move(128);
		stage2.move(128);
		stage3.move(128);
	}
	else {
		stage1.brake();
		stage2.brake();
		stage3.brake();
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
		stage1.move(128);
		if (colorDelay > 5) {stage2.move(-128);}
		stage3.move(-128);
	}


	// color sort toggle
	if (master.get_digital_new_press(COLOR_TOGGLE)) {
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

	// Ramp Mech
	if (master.get_digital_new_press(RAMP_TOGGLE)) {
		ramp.set_value(!ramp.get_value());
	}

	// Loader Mech
	if (master.get_digital_new_press(LOADER_TOGGLE)) {
		loader.set_value(!loader.get_value());
	}

	// Finger Mech
	if (master.get_digital_new_press(FINGER_TOGGLE)) {
		sexjoke.set_value(!sexjoke.get_value());
	}

	pros::delay(20);
	}
}