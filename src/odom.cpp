#include "config.h"
#include "odom.h"

OdomPod::OdomPod(pros::Rotation* odom, double wheelDiameter) {
    m_odom = odom;
    m_diameter = wheelDiameter;
}

double calculateSingleDegree(double wheelDiameter) {
    // sets up the odometry to convert angle readings to cm
    double wheelCircumference = M_PI * wheelDiameter; // 2 is the pre-measured wheel diameter in inches
	long double singleDegree = wheelCircumference / 360; // the distance that the robot moves in one degree of rotation of its wheels

    return singleDegree;
}


double OdomPod::measure(void) {
    // gets the centimeter distance moved in a single degree of rotation
    double singleDegree = calculateSingleDegree(m_diameter); // 2 is the pre-measured wheel diameter in inches (may need to be changed for precision)

    // gets the reading from the rotational sensor
    int rawReading = m_odom->get_position() * -1; // gives centidegrees (a 0-36,000 scale)
    double readingDeg = rawReading / 100; // reduces the centidegrees to degrees (a 0-360 scale)
    
    // converts the reading to centimeters
    double readingCM = readingDeg * singleDegree;

    return readingCM;
}


double OdomPod::measureVelocity(void) {
    // gets the centimeter distance moved in a single degree of rotation
    double singleDegree = calculateSingleDegree(2); // 2 is the pre-measured wheel diameter in inches


    // gets the reading from the rotational sensor
    int rawReading = m_odom->get_velocity(); // gives centidegrees (a 0-36,000 scale) per second
    double readingDeg = rawReading / 100; // reduces the centidegrees to degrees (a 0-360 scale) per second
    
    // converts the reading to centimeters
    double readingCM = readingDeg * singleDegree;

    return rawReading;
}

double OdomPod::measureHeading(void) {
    // the pre-measured distance between the robot's center of rotation
    double distanceBetweenCenterAndOdom = 10.0;

    // gets the distance that the robot moved (in cm)
    double odomReading = (double) this->measure();

    // the angle that the robot has moved
    // measured by finding the central angle of the arc with the distance from the odom pod
    // (using a derived version of the arc formula)
    double robotHeadingRadians = (double) odomReading / distanceBetweenCenterAndOdom;
    //     theta/angle  =    arc      /           radius
    double robotHeadingDegrees = (robotHeadingRadians * 180.0) / M_PI;

    return robotHeadingDegrees;
}





Odometry::Odometry(TrackingSensor movementSensor, // this sensor should track how far the robot has moved in some way
                        TrackingSensor headingSensor, // this sensor should track how far the robot has turned in some way
                        Pose startPosition // the pose that the robot starts at at the start of autonomous
                        ) 
{


    // sets the current location to the offset
    m_robotPose = startPosition;

    // resets the rotational sensor to zero
    movementSensor.reset();

    // sets the headings to the heading offset
    headingSensor.set(startPosition.heading);
}


Point Odometry::update(double heading, double moved) {
    double originalHeading = heading;
    // switches the heading based on the direction of the turn
    heading = moved >= 0
        ? heading // does nothing if the distance moved is positive
        : heading < 180 // flips if the distance moved is negative
            ? heading + 180 // flips the heading to itself + 180 if it is less than 180, putting it on the greater side of the circle
            : heading - 180; // flips the heading to itself - 180 if it is greater than 180, putting it on the lesser side of the circle

    // calculates the angle of only the triangle by subtracting from it based on its quadrant
    double triangleAngle = 0;
    if (heading < 90) {
       triangleAngle = heading;
    } else if (heading < 180) {
        triangleAngle = heading - 90;
    } else if (heading < 270) {
        triangleAngle = heading - 180;
    } else if (heading < 360) {
        triangleAngle = heading - 270;
    }
    // treats the distance moved as the hypotenuse of and the heading as the base angle of a triangle
    // and uses them to calculate the value of both legs (the changes in x and y)
    double xChange = 0;
    double yChange = 0;
    // if the heading is in quadrants 1 or 3, then the x-value is the opposite leg (sine) and the y-value is the adjacent leg (cosine)
    if ((heading < 90) || (heading >= 180 && heading < 270)) {
        xChange = std::sin(((triangleAngle * M_PI) / 180)) * moved;
        yChange = std::cos(((triangleAngle * M_PI) / 180)) * moved;
    }
    // otherwise, if the heading is in quadrants 2 or 4, then the x-value is the adjacent leg (cosine) and the y-value is the opposite leg (sine)
    else {
        xChange = std::cos(((triangleAngle * M_PI) / 180)) * moved;
        yChange = std::sin(((triangleAngle * M_PI) / 180)) * moved;
    }

    // reverses the movement of x and y if they moved in a positive direction and moved down or if they moved in a negative direction and moved up
    // checks for this by checking if the robot moved forward and had a heading that is in the positive y-axis
    if (originalHeading > 90 && originalHeading < 270) { // if heading is between 90 and 270 on the bottom side (moving down in the y-axis), flip the y-movement
        yChange = -yChange;
    }
    if (originalHeading < 360 && originalHeading > 180) { // if heading is between 180 and 360 on the left side (moving down in the x-axis), flip the x-movement
        xChange = -xChange;
    }

    // sets the final x and y positions to the changes in x and y added to the previous coordinates
    double xLoc = m_robotPose.x + xChange;
    double yLoc = m_robotPose.y + yChange;

    return {xLoc, yLoc};
}

// continually updates the value of the universal current location for use by every function
void Odometry::updateLoop() {
    if (!m_taskRunning) {
        m_taskRunning = true;
        loopTask = new pros::Task([this](){this->updateLoop();});
        return;
    }

    // declaration of previous location
    Pose previousLocation = m_robotPose;
    // calculates the change in odometry for the location update
    double changeInOdom = 0;
    double previousOdom = 0;
    double cumulativeOdom = 0;
    
    while (true) {
        if (!pros::Task::notify_take(true, 5)) { // while task is not paused by notification, run cycle 
                                                // (the 5 waits for 5 milliseconds before the loop starts and serves as the delay)
            // calculates the current distance moved
            cumulativeOdom = m_movementSensor.get();
            // calculates the change in odometry reading based on the previous measurement
            changeInOdom = cumulativeOdom - previousOdom;
            // updates the location
            double newHeading = m_headingSensor.get();
            Point newLocation = this->update(newHeading, changeInOdom);
            //universalCurrentLocation = {newLocation.x, newLocation.y, Inertial1.get_heading()};
            m_robotPose = {newLocation.x, newLocation.y, newHeading};
            //std::cout << "x = " << universalCurrentLocation.x << ", y = " << universalCurrentLocation.y << ", h = " << universalCurrentLocation.heading << "\n";
            // previous location for use in next cycle
            previousLocation = m_robotPose;
            // cumulative odometry value for use in next cycle as previous value
            previousOdom = cumulativeOdom;
        } else { // ensures that the code does not break while it is paused by a notification
            previousOdom = cumulativeOdom;
            previousLocation = m_robotPose;
        }
    }
}
