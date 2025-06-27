#include "main.h"

VelocityController::VelocityController(PowerUnit xOutput, PowerUnit yOutput, PowerUnit thetaOutput) {
    this->xOutput = xOutput;
    this->yOutput = yOutput;
    this->thetaOutput = thetaOutput;
}

// calculates the linear travel of a single degree of movement for a wheel of a given diameter
double VelocityController::calculateSingleDegree(double wheelDiameter) {
    // sets up the odometry to convert angle readings to cm
    double wheelCircumference = M_PI * wheelDiameter; // 2 is the pre-measured wheel diameter in inches
	long double singleDegree = wheelCircumference / 360; // the distance that the robot moves in one degree of rotation of its wheels

    return singleDegree;
}

// (private)
void VelocityController::followProfile(MotionProfile* currentlyFollowing, bool RAMSETE, bool reverse)
{
    // step-related variables
    double currentStep = 0;
    double step = 1 / (double) currentlyFollowing->profile.size();

    // point variables
    HoloMPPoint currentPoint = {0, 0, 0, 0, 0, 0};
    HoloMPPoint nextPoint = {0, 0, 0, 0, 0, 0};
    // speed variables
    Vector linVel;
    double angVel = 0;
    std::vector<double> velocitiesRPM = {0, 0};
    // clock variables
    double delay = 5;
    // action variables
    std::vector<bool> actionCompleteds = {false, false, false, false, false, false};

    // control loop
    while (true) {

        // calculates the current point as the nearest point to the current step
        currentPoint = currentlyFollowing->findNearestHoloPoint(currentStep);

        // executes custom actions if the profile has reached or passed their t-point and have not yet been activated
        for (int i = 0; i < actions.size(); i++) {
            if (((currentPoint.t >= actionTs[i])) && !actionCompleteds[i]) {
                actions[i]();
                actionCompleteds[i] = true;
            }
        }
        
        double linVel = IPStoRPM(currentPoint.linVel.magnitude);

        // sends the output voltage to the motors
        if (reverse && !RAMSETE) {
            xOutput.move(-linVel * std::cos(currentPoint.linVel.angle));
            yOutput.move(-linVel * std::cos(currentPoint.linVel.angle));
            thetaOutput.move(angVel);
        } else {
            xOutput.move(linVel * std::cos(currentPoint.linVel.angle));
            yOutput.move(linVel * std::cos(currentPoint.linVel.angle));
            thetaOutput.move(angVel);
        }

        // 5 ms delay (- the time taken to calculate)
        pros::delay(5);

        // if the current step is the final point (t = 1 - step), 
        // then the drivetrain is stopped and the function ends
        if (currentPoint.t == currentlyFollowing->profile[currentlyFollowing->profile.size() - 1].t) {
            if (currentlyFollowing->zones[currentlyFollowing->zones.size() - 1].zoneLine.slope + currentlyFollowing->zones[currentlyFollowing->zones.size() - 1].zoneLine.yIntercept == 0) {
                xOutput.stop();
                yOutput.stop();
                thetaOutput.stop();
            }
            return;
        }

        // goes to the next step of the function if it did not end
        currentStep += step;
    }
}

// starts the filter loop if it is not already active (public)
void VelocityController::startProfile(MotionProfile* profile, bool reverse, bool RAMSETE) {

    // auto controlLoopFunction = [this, path, RAMSETE] () {return this->followProfile(*this->queuedProfile, path, RAMSETE);};

    // if (controlLoop_task_ptr == NULL) {
        // pros::Task* controlLoop_task_ptr = new pros::Task(controlLoopFunction);
        this->followProfile(profile, RAMSETE, reverse);
    // }
}

void VelocityController::addAction(std::function<void(void)> action, double time) {
    double actionT = time;
    this->actions.push_back(action);
    this->actionTs.push_back(actionT);
}

void VelocityController::clearActions(void) {
    this->actions.clear();
    this->actionTs.clear();
    this->actionCompleteds = {false, false, false, false, false, false};
}
