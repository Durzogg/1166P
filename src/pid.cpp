#include "init.h"

void PIDMover(
		Point goalPosition, // goal coordinate position
		bool reverse, // defaults to false- explicitly set to true to reverse the robot

		std::vector<std::function<void(void)>> customs, // a lambda function that will execute during the PID (optional)
		std::vector<double> executeAts // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
		)
{


	drivetrain.set_brake_mode(MOTOR_BRAKE_HOLD);

// PID Calculation Variables
	// General Variables
	double power = 0;

	double tolerance = 1;
	std::vector<bool> customsCompleted(customs.size(), false);
	bool actionCompleted = false;
	int cyclesAtGoal = 0;
	int cyclesFlipping = 0;

	// Constants (need to be tuned individually for every robot)
	ConstantContainer moverConstants;
		moverConstants.kP = 4; // 4
		moverConstants.kI = 0.1; // 0.1
		moverConstants.kD = 2.7; // 2.7
	





// sets the set point to the difference between the current point and the goal point
	Point originalPosition = {universalCurrentLocation.x, universalCurrentLocation.y};
	double setPoint = calculateDistance(originalPosition, goalPosition);
	double remainingDistance = setPoint;
	bool greaterThanNegativeLine = false;

// finds the part of the coordinate plane in which the robot has passed its destination
	Inequality negativeSide = calculatePerpendicularInequality(originalPosition, goalPosition);

// Odometry Measurement Setup
	bool isPositive = setPoint > 0; // Checks if the movement is positive or negative

// Odometry Pre-Measurement
	
	// used to measure the rotational sensor values of all the motors (this comes in degrees)
	double currentDistanceMovedByWheel = readOdomPod(Rotational);

	// this initializes variables that are used to measure values from previous cycles
	PIDReturn cycle;
	cycle.prevError = setPoint - currentDistanceMovedByWheel;
	cycle.power = 0;
	cycle.prevIntegral = 0;



	while (actionCompleted != true) {

	// gets the power for the current cycle
	cycle = PIDCalc(currentDistanceMovedByWheel, setPoint, isPositive, moverConstants, cycle);
/*
	// checks to see if the robot has been flipping between directions and stops in the exit condition if it is
	if ((power > 0 && cycle.power < 0) || (power < 0 && cycle.power > 0)) {
		cyclesFlipping++;
	}
*/
	power = cycle.power;

	// finds if the robot has passed the perpendicular line's inequality or not
	greaterThanNegativeLine = universalCurrentLocation.y >= (negativeSide.slope * universalCurrentLocation.x) + negativeSide.yIntercept;

	// handles the line if it is vertical
	if (std::isnan(negativeSide.slope)) {
		greaterThanNegativeLine = universalCurrentLocation.x > negativeSide.yIntercept;
	} else if (negativeSide.slope == 0) {
		greaterThanNegativeLine = universalCurrentLocation.y > negativeSide.yIntercept;
	}

	// reverses the direction if the robot has passed the inequality
	if ((greaterThanNegativeLine && negativeSide.equality < 0) ||
		(!greaterThanNegativeLine && negativeSide.equality > 0)) {
			power *= -1;
	}
	// reverses the direction if the robot has been commanded to move in reverse
	if (reverse) {
		power *= -1;
	}

	// moves the wheels at the desired power, ending the cycle
	drivetrain.move(power);



	// Custom lambda function that will execute if given and the robot has reached the point given by executeAt
		
	for (int i = 0; i < customs.size(); i++) {
		// ensures that the code will only run if the function has been provided and if executeAt has been reached
		if (customs[i] != 0 && (currentDistanceMovedByWheel >= executeAts[i]) && !customsCompleted[i]) {
			// runs the function
			customs[i]();
			// prevents the function from running again
			customsCompleted[i] = true;
		}
	}

// PID Looping Odometry Measurement

		// fifteen millisecond delay between cycles
		pros::delay(15);

		std::cout << "ucl = " << universalCurrentLocation.x << ", " << universalCurrentLocation.y << "\n";


		if (std::isnan(findIntersection(findLineWithHeading({universalCurrentLocation.x, universalCurrentLocation.y}, universalCurrentLocation.heading), {negativeSide.slope, negativeSide.yIntercept}).x) ||
			std::isnan(findIntersection(findLineWithHeading({universalCurrentLocation.x, universalCurrentLocation.y}, universalCurrentLocation.heading), {negativeSide.slope, negativeSide.yIntercept}).y)) {
			continue;
		}
		// fixes the goal point to be in front of where we are facing
		goalPosition = findIntersection(findLineWithHeading({universalCurrentLocation.x, universalCurrentLocation.y}, universalCurrentLocation.heading), {negativeSide.slope, negativeSide.yIntercept});
		setPoint = calculateDistance(originalPosition, goalPosition);

		// calculates the distance moved as the difference between the distance left to move
		// and the total distance to move
		remainingDistance = calculateDistance({universalCurrentLocation.x, universalCurrentLocation.y}, goalPosition);
		currentDistanceMovedByWheel = setPoint - remainingDistance;

		std::cout << "remaining = " << remainingDistance << "\n";

		pros::lcd::print(0, "remaining = %f", remainingDistance);

		// checks to see if the robot has completed the movement by checking if it is within a range of the perpendicular line of its goal point
		if (remainingDistance <= 0 + tolerance)
			{
				if (true) {
					actionCompleted = true;
					drivetrain.brake();
				} else {
					cyclesAtGoal++;
				}
			} else {
				cyclesAtGoal = 0;
			}

		

		
		// checks to see if the robot has been flipping back and forth in direction at the exit location and stops it if id does
		/*   if (cyclesFlipping >= 5) {
				actionCompleted = true;
				AllWheels.brake();
		   }
		*/
		
	}

}

void PIDTurner(
		int setPoint, // the goal inertial heading in degrees
		int direction, // 1 for left and 2 for right

		std::vector<std::function<void(void)>> customs, // a lambda function that will execute during the PID (optional)
		std::vector<int> executeAts // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
		)
{

	drivetrain.set_brake_mode(MOTOR_BRAKE_HOLD);

	// pauses the position updating for the turn
	coordinateUpdater_task_ptr->notify();

// PID CALCULATION VARIABLES
// General Variables
	double power = 0;
	double negativePower;

	double tolerance = 2.5;
	int cyclesAtGoal = 0;
	std::vector<bool> customsCompleted(executeAts.size(), false);
	bool actionCompleted = false;

// Constants -- tuning depends on whether the robot is moving or turning
	ConstantContainer turnerConstants;


// Checks if the movement is positive or negative
	bool isPositive = setPoint > universalCurrentLocation.heading;

// PID LOOPING VARIABLES
	double inertialReadingInit = universalCurrentLocation.heading;
	double distanceToMove = 0;

	if (direction == 1) {
		// standard left turn is negative, so the calculation makes it positive if it is a normal turn
		// ex: current = 90, goal = 45 -> -45 degree turn -> positive 45 degree turn by calculation
		// 90 - 45 = 45 degree turn left
		distanceToMove = inertialReadingInit - setPoint;
	}
	else if (direction == 2) {
		// standard right turn is positive, so the calculation keeps it positive if it is a normal turn
		// ex: current = 45, goal = 90 -> 45 degree turn -> positive 45 degree turn by calculation
		// 90 - 45 = 45 degree turn right
		distanceToMove = setPoint - inertialReadingInit;
	}

	// if the error is positive, then the calculation is fine and is left
	if (distanceToMove >= 0) {
		// do nothing
	}
	// otherwise, the turn takes the "long way" around the circle, and the calculation has provided the
	// value of the negative short way - adding 360 to this value gives the long way around the circle,
	// which is what is needed
	// ex: current = 90, goal = 45, direction = right -> calculated -45 degree turn -> + 360 -> 315 (length of long way)
	// 45 - 90 = -45 (short way, negative) + 360 = 315 (long way, positive)
	else {
		distanceToMove += 360;
	}
	// the calculation has now yielded a positive value that is the error needed of our turn in the proper
	// direction, making it similar to how a forward/backward movement is coded

	// finally, the code sets a new value that will be set to the distance moved to zero to finalize this similarity
	// distanceToMove is analogous to setPoint on PIDMover, and changeInReading is analogous to currentDistanceMovedByWheel
	double changeInReading = 0;
	double prevChangeInReading = 0;

	// it also sets a value that is used as an intermediate value in the switch calculation every loop
	double changeInDistance = 0;


/*
// Arc Calculation
	double diameter = 10.5; // manually measured diameter of the circle that has a center at the center of rotation of the robot and extends out to the middle of the wheels
	double radius = diameter / 2; // basic circle math - the radius is half of the diameter
	double centralAngleOfArc = distanceToMove; // the central angle is the previously-calculated angle for the robot to move
	double arcLength = centralAngleOfArc * radius; // arc length formula (angle in radians * radius)
	distanceToMove = arcLength; // sets distanceToMove to the length of the arc instead of the angle, as this will make for better calculations of speed
*/


	// constant definitions
	// >= 90 degree turns
	if (distanceToMove >= 90) {
		turnerConstants.kP = 3;
		turnerConstants.kI = 0.2; // 0.2
		turnerConstants.kD = 26; // 13.1
	// < 90 degree turns
	} else {
		turnerConstants.kP = 2.3; // 2.3
		turnerConstants.kI = 0.24; // 0.05
		turnerConstants.kD = 32; // 50
		tolerance = 3.5;
	}

	// this initializes variables that are used to measure values from previous cycles
	PIDReturn cycle;
	cycle.prevError = distanceToMove - changeInReading;
	cycle.power = 0;
	cycle.prevIntegral = 0;

	 

	while (!actionCompleted) {

	// calculates the change in heading at the very start of each cycle 
	// to ensure that the robot has not passed its relative zero by mistake
	changeInDistance = direction == 1
		? inertialReadingInit - universalCurrentLocation.heading
		: universalCurrentLocation.heading - inertialReadingInit;
	changeInReading = (changeInDistance < 0)
		    ? changeInDistance + 360
			: changeInDistance;
	// if the robot's change in distance between the last two cycles is too great,
	// then throw out the new value and use the one from the last successful cycle
	if (changeInReading - prevChangeInReading > 90) {
		changeInReading = prevChangeInReading;
	}
	// if the check passes, then the current change in distance is used
	// is tracked for the next cycle
	else {
		prevChangeInReading = changeInReading;
	}

	// gets the power for the current cycle
	cycle = PIDCalc(changeInReading, distanceToMove, isPositive, turnerConstants, cycle);
	power = cycle.power;


	// custom lambda functions
	for (int i = 0; i < customs.size(); i++) {
		// ensures that the code will only run if the function has been provided and if executeAt has been reached
		if (customs[i] != 0 && ((changeInReading >= executeAts[i] && isPositive) || (changeInReading <= executeAts[i] && !isPositive)) && !customsCompleted[i]) {
			// runs the function
			customs[i]();
			// prevents the function from running again
			customsCompleted[i] = true;
		}
	}



	// PID LOOPING CODE

		negativePower = power * -1;

		// the power will never be negative and invert the turns because distanceToMove is always positive
			if (direction == 1) {
				leftDrivetrain.move(negativePower);
				rightDrivetrain.move(power);
			}
			else if (direction == 2) {
				leftDrivetrain.move(power);
				rightDrivetrain.move(negativePower);
			}

		pros::delay(15);

		// the change in reading is set to the absolute value of the change in reading due to everything being positive
		changeInDistance = direction == 1
			? inertialReadingInit - universalCurrentLocation.heading
			: universalCurrentLocation.heading - inertialReadingInit;
		changeInReading = (changeInDistance < 0)
		    ? changeInDistance + 360
			: changeInDistance;



/*
		// the change in reading is converted to an arc to make it align with the original arc
		double changeInCentralAngleOfArc = changeInReading;
		double arcDistanceMoved = radius * changeInCentralAngleOfArc;
		changeInReading = arcDistanceMoved;
*/

		if (((changeInReading <= (distanceToMove + tolerance)) && (changeInReading >= (distanceToMove - tolerance)))) {
				if (cyclesAtGoal >= 30) {
					actionCompleted = true;
					drivetrain.brake();
					coordinateUpdater_task_ptr->notify_clear();
				} else {
					cyclesAtGoal++;
				}
		} else {
			cyclesAtGoal = 0;
		}
	}
}

void PIDArc(
	int chordLength, // the distance between the robot's starting position and its destination position
	int maxDist, // the maximum distance of the straight line from your current position and the setPoint to the arc (should be measured at half-point)
	int direction, // 1 for left, 2 for right

	std::vector<std::function<void(void)>> customs, // a lambda function that will execute during the PID (optional)
	std::vector<int> executeAts // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
	)
{
// Checks if the movement is positive or negative
	bool isPositive = chordLength > 0;


	drivetrain.set_brake_mode(MOTOR_BRAKE_HOLD);


// PID CALCULATION VARIABLES
// General Variables
	int error;
	int power;
	int tolerance = 0.75;
	std::vector<bool> customsCompleted;
	bool actionCompleted = false;

// Proportional Variables
	int proportionalOut;

// Integral Variables
	int integral = 0;
	int integralLimiter = 512; // customizable
	int integralOut;

// Derivative Variables
    int derivative;
    int derivativeOut;
	int prevError;
	

// Constants -- tuning depends on whether the robot is moving or turning
	ConstantContainer arcConstants;
	arcConstants.kP = 3.2; // customizable
	arcConstants.kI = 0.3; // customizable
	arcConstants.kD = 0.1; // customizable


// PID LOOPING VARIABLES
	chordLength = chordLength * 2.54; // converts from inches to cm
	maxDist = maxDist * 2.54;
	
	for (int i = 0; i < executeAts.size(); i++) {
		executeAts[i] *= 2.54;
	}


// Odometry
	RotationalTurn.reset();
	double currentDistanceMovedByWheel = readOdomPod(RotationalTurn);

	error = (int) (chordLength - currentDistanceMovedByWheel);
	prevError = error;

// Arc Measurement
	if (!isPositive) {chordLength = -chordLength;}
	double halfSetPoint = chordLength / 2; // divides the chord/setPoint into two halves, one on each side of the bisector maxDist
	double diameterMinusMaxDist = (halfSetPoint * halfSetPoint) / maxDist; // uses the Intersecting Chords Theorem to find the diameter of the circle (excluding maxDist)
	double diameter = diameterMinusMaxDist + maxDist; // adds maxDist to the previous variable to find the diameter of the circle
	double radius = diameter / 2; // basic circle math - the radius is half of the diameter
	double centralAngleOfArc = std::acos(((radius * radius) + (radius * radius) - (chordLength * chordLength)) / (2 * radius * radius)); // Law of Cosines to find central angle - a and b are the radius, and c is the chord (returns radians)
	double arcLength = centralAngleOfArc * radius; // arc length formula (angle in radians * radius)
	double setPoint = arcLength; // sets the setPoint to the length of the arc instead of the length of the chord, as it will actually be moving that far


// Arc Odometry
	double oRadius = (((halfSetPoint * halfSetPoint) / maxDist) + maxDist) / 2;
	double distBetweenWheels = 13 * 2.54;
	double mult = ((oRadius - distBetweenWheels) / oRadius);

// Arc Turning - turns the robot so that it starts the movement perpendicular to the center of the circle so it can move along the arc accurately
	double angleOfArcFromStartPosRAD = std::acos(((radius * radius) + (chordLength * chordLength) - (radius * radius)) / (2 * radius * chordLength)); // Law of Cosines to find starting angle of arc - a and c are the radius, and b is the chord (returns radians)
	int angleOfArcFromStartPosDEG = (int) (angleOfArcFromStartPosRAD * (180 / 3.14)); // converts the radians to degrees
	int angleToTurn = 90 - angleOfArcFromStartPosDEG; // as we are already facing toward the destination, this sets our value to turn as the angle between the chord and a line perpendicular to the radius of the circle
	int directionForTurn = direction == 1
		? 2
		: 1;
	int newHeading = directionForTurn == 1
		? Inertial1.get_heading() - angleToTurn // negative left turn
		: Inertial1.get_heading() + angleToTurn; // positive right turn

	// if (newHeading < 0) {newHeading = newHeading + 360;} // makes the new heading the actual new heading if it is a negative

	PIDTurner(newHeading, directionForTurn);

	if (!isPositive) {setPoint = -setPoint;}


	// this initializes variables that are used to measure values from previous cycles
	PIDReturn cycle;
	cycle.prevError = setPoint - currentDistanceMovedByWheel;
	cycle.power = 0;
	cycle.prevIntegral = 0;


	while (actionCompleted != true) {

	// gets the power for the current cycle
	cycle = PIDCalc(currentDistanceMovedByWheel, setPoint, isPositive, arcConstants, cycle);




	// Custom lambda function that will execute if given and the robot has reached the point given by executeAt
		
	for (int i = 0; i < customs.size(); i++) {
		// ensures that the code will only run if the function has been provided and if executeAt has been reached
		if (customs[i] != 0 && ((currentDistanceMovedByWheel >= executeAts[i] && isPositive) || (currentDistanceMovedByWheel <= executeAts[i] && !isPositive)) && !customsCompleted[i]) {
			// runs the function
			customs[i]();
			// prevents the function from running again
			customsCompleted[i] = true;
		}
	}

	// caps motor power at 128 if it goes beyond it to ensure that the multiplier makes one side spin slower
		if (power > 128) {
			power = 128;
		}
		if (power < -128) {
			power = -128;
		}

		// causes the wheels to move in the proper direction, with the outer wheels being normal and the inner wheels being multiplied by mult
			if (direction == 1) {
				rightDrivetrain.move(power);
				leftDrivetrain.move(power * mult);
			} else if (direction == 2) {
				rightDrivetrain.move(power * mult);
				leftDrivetrain.move(power);
			}


		pros::delay(15);

		currentDistanceMovedByWheel = readOdomPod(RotationalTurn);

		if (((currentDistanceMovedByWheel <= setPoint + tolerance) && (currentDistanceMovedByWheel >= setPoint - tolerance))) {
				actionCompleted = true;
				drivetrain.brake();
		}
	}
}



void PIDArm(
		int setPoint, // how far you want to move in inches

		std::vector<std::function<void(void)>> customs, // a lambda function that will execute during the PID (optional)
		std::vector<int> executeAts // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
		)
{


	arm.set_brake_mode(MOTOR_BRAKE_HOLD);

// PID Calculation Variables
	// General Variables
	int error;
	int power;
	int tolerance = 2;
	std::vector<bool> customsCompleted;
	bool actionCompleted = false;

	// Constants (need to be tuned individually for every robot)
	ConstantContainer moverConstants;
	moverConstants.kP = 1.28; // customizable
	moverConstants.kI = 0.4; // customizable
	moverConstants.kD = 0.1; // customizable

	
	




// Odometry Measurement Setup
	bool isPositive = setPoint > 0; // Checks if the movement is positive or negative
	setPoint = setPoint * 2.54; // converts from inches to cm, as the function call uses inches for ease of measurement

	for (int i = 0; i < executeAts.size(); i++) {
		executeAts[i] *= 2.54;
	}

	double wheelCircumference = 3.14 * 3.25; // 3.25 is the wheel diameter in inches
	double wheelRevolution = wheelCircumference * 2.54; // wheel circumference in cm
						// this is equivalent to how far the robot moves in one 360-degree rotation of its wheels
	long double singleDegree = wheelRevolution / 360; // the distance that the robot moves in one degree of rotation of its wheels



// Odometry Pre-Measurement
	// resets the rotation of all motors before the movement so the movement can be calculated from zero to the destination
	arm.tare_position();

	// used to measure the rotational sensor values of all the motors (this comes in degrees)
	double armMeasurement = arm.get_position();

	double currentMotorReading = (armMeasurement / 2); // measures the average rotation of all motors to determine the movement of the entire robot
	double currentWheelReading = currentMotorReading; // measures the current reading (in degrees) of the wheel by multiplying it by the gear ratio

	// measures the current distance moved by the robot by multiplying the number of degrees that it has moved 
	// by the number of centimeters moved in a single degree of movement
	double currentDistanceMovedByWheel = currentWheelReading * singleDegree; 

	// this initializes variables that are used to measure values from previous cycles
	PIDReturn cycle;
	cycle.prevError = setPoint - currentDistanceMovedByWheel;
	cycle.power = 0;
	cycle.prevIntegral = 0;

	 

	while (actionCompleted != true) {

	// gets the power for the current cycle
	cycle = PIDCalc(currentDistanceMovedByWheel, setPoint, isPositive, moverConstants, cycle);

	// moves the wheels at the desired power, ending the cycle
	arm.move(cycle.power);



	// Custom lambda function that will execute if given and the robot has reached the point given by executeAt
		
	for (int i = 0; i < customs.size(); i++) {
		// ensures that the code will only run if the function has been provided and if executeAt has been reached
		if (customs[i] != 0 && ((currentDistanceMovedByWheel >= executeAts[i] && isPositive) || (currentDistanceMovedByWheel <= executeAts[i] && !isPositive)) && !customsCompleted[i]) {
			// runs the function
			customs[i]();
			// prevents the function from running again
			customsCompleted[i] = true;
		}
	}


// PID Looping Odometry Measurement

		// fifteen millisecond delay between cycles
		pros::delay(15);

		// finds the degrees of measurement of the motors
		armMeasurement = arm.get_position();

		// reassigns the "distance moved" variables for the next cycle after the delay
		currentMotorReading = (armMeasurement / 2); // degrees
		currentWheelReading = currentMotorReading; // degrees = degrees * gear ratio multiplier
		currentDistanceMovedByWheel = currentWheelReading * singleDegree; // centimeters

		// checks to see if the robot has completed the movement by checking several conditions, and ends the movement if needed
		if (((currentDistanceMovedByWheel <= setPoint + tolerance) && (currentDistanceMovedByWheel >= setPoint - tolerance))) {
				actionCompleted = true;
				arm.brake();
		}
	}
}



PIDReturn PIDCalc(
	double distanceMoved, // current distance moved (in odometry units)
	double setPoint, // goal distance to move (in odometry units)
	bool isPositive, // direction of movement
	ConstantContainer constants, // all constants
	PIDReturn lastCycle // data from previous cycle
	)
{
	PIDReturn thisCycle;
	// P: Proportional -- slows down as we reach our target for more accuracy

		// error = goal reading - current reading
		double error = setPoint - distanceMoved;
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		double proportionalOut = error * constants.kP;




	// I: Integral -- starts slow and speeds up as time goes on to prevent undershooting

		// starts the integral at the error, then compounds it with the new current error every loop
		double integral = lastCycle.prevIntegral + error;
		// prevents the integral variable from causing the robot to overshoot
		if ((((error <= 0)) && (lastCycle.prevError > 0)) || 
			(((error >= 0)) && (lastCycle.prevError < 0))) {
			integral = 0;
		}
		// prevents the integral from winding up too much, causing the number to be beyond the control of
        // even kI
		// if we want to make this better, see Solution #3 for 3.3.2 in the packet
		/* if (((isPositive) && (error >= 100)) || ((!isPositive) && (error <= -100))) {
			integral = 0;
			}
		*/
		if ((integral > 100) || (integral < -100)) {
			integral = integral > 100
				? 100
				: -100;
			}
		// kI (integral constant) brings integral down to a reasonable/useful output number
		double integralOut = integral * constants.kI;

		// adds integral to return structure for compounding
		thisCycle.prevIntegral = integral;



	// D: Derivative -- slows the robot more and more as it goes faster

        // starts the derivative by making it the rate of change from the previous cycle to this one
        double derivative = error - lastCycle.prevError;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        double derivativeOut = derivative * constants.kD;

		// sets the previous error to the current error for use in the next derivative
		thisCycle.prevError = error;



	// Adds the results of each of the calculations together to get the desired power
		double power = proportionalOut + integralOut + derivativeOut;

		thisCycle.power = power;

	// returns a PIDReturn structure
		return thisCycle;
}