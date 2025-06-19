#include "pid.h"

PIDController::PIDController(TrackingSensor pidSensor, ConstantContainer constants) {

}

void PIDController::movement(
    Point goalPosition, // goal coordinate position
    bool reverse, // defaults to false- explicitly set to true to reverse the robot

    std::vector<std::function<void(void)>> customs, // a lambda function that will execute during the PID (optional)
    std::vector<double> executeAts // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
    )
{


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

PIDReturn PIDController::calculateOutput(
	double distanceMoved, // current distance moved (in odometry units)
	double setPoint, // goal distance to move (in odometry units)
	bool isPositive, // direction of movement
	PIDReturn lastCycle // data from previous cycle
	)
{
	PIDReturn thisCycle;
	// P: Proportional -- slows down as we reach our target for more accuracy

		// error = goal reading - current reading
		double error = setPoint - distanceMoved;
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		double proportionalOut = error * m_constants.kP;




	// I: Integral -- starts slow and speeds up as time goes on to prevent undershooting

		// starts the integral at the error, then compounds it with the new current error every loop
		double integral = p_integral + error;
		// prevents the integral variable from causing the robot to overshoot
		if ((((error <= 0)) && (p_error > 0)) || 
			(((error >= 0)) && (p_error < 0))) {
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
		double integralOut = integral * m_constants.kI;

		// adds integral to return structure for compounding
		thisCycle.prevIntegral = integral;



	// D: Derivative -- slows the robot more and more as it goes faster

        // starts the derivative by making it the rate of change from the previous cycle to this one
        double derivative = error - p_error;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        double derivativeOut = derivative * m_constants.kD;

		// sets the previous error to the current error for use in the next derivative
		thisCycle.prevError = error;



	// Adds the results of each of the calculations together to get the desired power
		double power = proportionalOut + integralOut + derivativeOut;

		thisCycle.power = power;

	// returns a PIDReturn structure
		return thisCycle;
}