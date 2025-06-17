#include "init.h"

VelocityController::VelocityController(std::vector<std::function<void(void)>> actions, std::vector<double> actionTs) {
    //this->actions = {[](){master.rumble(".");pros::lcd::print(0, "0.25");}, [](){master.rumble(".");pros::lcd::print(1, "0,5");intake.move(128);}, [](){master.rumble(".");pros::lcd::print(2, "0.9");}};
    this->actions = actions;
    this->actionTs = actionTs;
}

// uses the kinematic equations of a differential chassis and unit conversions to convert a linear and angular velocity to something that can be used
// by each side of the drivetrain
std::vector<double> VelocityController::calculateOutputOfSides(Vector linearVelocityIPS, double angularVelocityRADPS, double profileMaxIPS) {
    
    // non-angular movement
    double TL_BR_IPS = linearVelocityIPS.y - linearVelocityIPS.x;
    double TR_BL_IPS = linearVelocityIPS.y + linearVelocityIPS.x;

    // rotation
    double rotationDifference = (angularVelocityRADPS * g_distBetweenWheels) / 2;

    double tlIPS = TL_BR_IPS - rotationDifference;
    double blIPS = TR_BL_IPS - rotationDifference;

    double trIPS = TR_BL_IPS + rotationDifference;
    double brIPS = TL_BR_IPS + rotationDifference;

    // ips to rpm
    double tlRPM = (tlIPS * 60 / (M_PI * g_diameter)) / g_gearRatio6; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))
    double trRPM = (trIPS * 60 / (M_PI * g_diameter)) / g_gearRatio6; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))
    double blRPM = (blIPS * 60 / (M_PI * g_diameter)) / g_gearRatio6; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))
    double brRPM = (brIPS * 60 / (M_PI * g_diameter)) / g_gearRatio6; // rpm = m/s * (60 s / min) * (1 rotation / (single degree travel * 360))
    
    double profileMax = IPStoRPM(profileMaxIPS);

    double offender = 0;
    if ((std::abs(tlRPM) > profileMax) && (std::abs(tlRPM) > offender)) {
        tlRPM = offender;
    } else if ((std::abs(trRPM) > profileMax) && (std::abs(trRPM) > offender)) {
        trRPM = offender;
    } else if ((std::abs(blRPM) > profileMax) && (std::abs(blRPM) > offender)) {
        blRPM = offender;
    } else if ((std::abs(brRPM) > profileMax) && (std::abs(brRPM) > offender)) {
        brRPM = offender;
    }

    if (offender != 0) {
        double scaling = profileMax / std::abs(offender);
        tlRPM *= scaling;
        trRPM *= scaling;
        blRPM *= scaling;
        brRPM *= scaling;
    }

    return {tlRPM, trRPM, blRPM, brRPM};
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
    MPPoint currentPoint = {0, 0, 0, 0, 0, 0};
    MPPoint nextPoint = {0, 0, 0, 0, 0, 0};
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
        currentPoint = currentlyFollowing->findNearestPoint(currentStep);
        //std::cout << currentPoint.t << "\n";

        // std::cout << "x = " << currentPoint.x << ", y = " << currentPoint.y << ", h = " << currentPoint.heading << ", lv = " << currentPoint.linVel << ", av = " << currentPoint.angVel << ", t = " << currentPoint.t << "\n";

        // sets linear and angular velocities to that of the current point - these are changed by RAMSETE if it is on
        double linVel = currentPoint.linVel;
        double angVel = currentPoint.angVel;

        // std::cout << "lv = " << linVel << ", rv = " << angVel << "\n";

        // calculation of output of each side with error corrections from RAMSETE
        if (RAMSETE) {
            Pose location = {universalCurrentLocation.x * 0.0254, universalCurrentLocation.y * 0.0254, universalCurrentLocation.heading};
            nextPoint = currentlyFollowing->findNearestPoint(currentStep + step);
            nextPoint = {nextPoint.x * 0.0254, nextPoint.y * 0.0254, nextPoint.heading};

            if (reverse) {
                linVel *= -1;

                if (location.heading > 180) {
                    location.heading -= 180;
                } else {
                    location.heading += 180;
                }
            }

            double fixedOdomAngle = fixAngle(location.heading) * (M_PI / 180);
            double fixedNextAngle = fixAngle(nextPoint.heading) * (M_PI / 180);



        //textToWrite.push_back("odom = " + std::to_string(location.heading) + ", fodom = " + std::to_string(fixedOdomAngle) + "\n");
        //textToWrite.push_back("next = " + std::to_string(nextPoint.heading) + ", fnext = " + std::to_string(fixedNextAngle) + "\n\n");
        // rotation of the x, y, and heading errors to fit the local frame
            Pose error;
            error.x = (std::cos(fixedOdomAngle) * (nextPoint.x - location.x)) + (std::sin(fixedOdomAngle) * (nextPoint.y - location.y));
            if (reverse) {error.x *= -1;}
            error.y = (std::cos(fixedOdomAngle) * (nextPoint.y - location.y)) - (std::sin(fixedOdomAngle) * (nextPoint.x - location.x));
            error.heading = fixedNextAngle - fixedOdomAngle;
            // std::cout << error.heading << "\n";

            linVel *= 0.0254;

            // bounds the error from 0-180 to prevent the correction from being an un-optimal turn direction (where going the other way would be faster)
            if (error.heading < -M_PI) {
                error.heading = error.heading + (2 * M_PI);
            } else if (error.heading > M_PI) {
                error.heading = (2 * M_PI) - error.heading;
            }

            //std::cout << "prp = " << fixedNextAngle << ", ap = " << fixedOdomAngle << ", c = " << error.heading << "\n";
            //std::cout << "ucl = " << location.heading << ", actual = " << getAggregatedHeading(Kalman1, Kalman2) << ", used = " << fixedOdomAngle << "\n";

        // tuning constants (current values are from the widely accepted defaults from FTCLib)
            double b = 2.0; // this is a proportional gain for each of the different error elements of the controller (put into the gain value calculations)
            double zeta = 0.7; // this is a dampener for the direct movements (k1 and k3/x-value and heading-value)

        // gain values that serve as scaling multipliers for the outputs based on the profile's velocity and pre-set constants
            // k1/k3: the proportional gain value for both the local frame of x and heading
            double k = 2 * zeta * std::sqrt(std::pow(angVel, 2) + (b * std::pow(linVel, 2)));
            // k2: the gain value for the y-value, which the robot cannot move directly on and is thus handled differently 
            double k2 = b * linVel;

        // control inputs that determine the controller's influence on the velocity based on the error
            // simply the normal gain value multiplied by the x-error because that can be directly moved upon (input for linear velocity)
            double u1 = k * error.x;
            // the special gain value is used for the y-value, and it is also scaled with the part in purple parenthesis to let it switch directions smoothly if it is past
            // its goal point; the second part is the same simple direct angular movement for the error in heading that is used in the same way with the x-value for linear
            // movement
            double u2 = (k2 * (std::sin(error.heading) / error.heading) * error.y) + (k * error.heading);

        // actual calculations of modified linear and angular velocities as additions/subtractions to the profile's original values
            // the original linear velocity is also transformed to follow the robot's error in heading before having the control input subtracted from it
            linVel = ((linVel * 0.0254) * std::cos(error.heading)) + u1;
            // the angular velocity does not need to be transformed in the same way that the linear velocity needs to because it is already angular in reference to the robot
            angVel = currentPoint.angVel + u2;

            linVel *= 39.37008;

            linVel *= currentPoint.linVel / currentlyFollowing->maxSpeed;
            delay = 10;
        }

        // standard calculation of output of each side based on specifications of the motion profile
        velocitiesRPM = this->calculateOutputOfSides(linVel, angVel, currentlyFollowing->maxSpeed);

        // executes custom actions if the profile has reached or passed their t-point and have not yet been activated
        for (int i = 0; i < actions.size(); i++) {
            if (((currentPoint.t >= actionTs[i])) && !actionCompleteds[i]) {
                actions[i]();
                actionCompleteds[i] = true;
            }
        }
        
        // converts the velocity in rpm to velocity in millivolts
        int maxVoltage = 12000; // innate max voltage of motors (in mV)
        double rpmToV = maxVoltage / g_maxRPM; // multiplier to convert rpm to voltage (in units of millivoltage / rpm so multiplying it by rpm cancels to millivoltage)

        // sends the output voltage to the motors
        if (reverse && !RAMSETE) {
            topLeft6.move_voltage(-velocitiesRPM[0] * rpmToV);
            topRight6.move_voltage(-velocitiesRPM[1] * rpmToV);
            bottomLeft6.move_voltage(-velocitiesRPM[2] * rpmToV);
            bottomRight6.move_voltage(-velocitiesRPM[3] * rpmToV);

            topLeft2.move_voltage((-velocitiesRPM[0] / g_gearRatio2) * rpmToV);
            topRight2.move_voltage((-velocitiesRPM[1] / g_gearRatio2) * rpmToV);
            bottomLeft2.move_voltage((-velocitiesRPM[0] / g_gearRatio2) * rpmToV);
            bottomRight2.move_voltage((-velocitiesRPM[1] / g_gearRatio2) * rpmToV);
        } else {
            topLeft6.move_voltage(velocitiesRPM[0] * rpmToV);
            topRight6.move_voltage(velocitiesRPM[1] * rpmToV);
            bottomLeft6.move_voltage(velocitiesRPM[2] * rpmToV);
            bottomRight6.move_voltage(velocitiesRPM[3] * rpmToV);

            topLeft2.move_voltage((velocitiesRPM[0] / g_gearRatio2) * rpmToV);
            topRight2.move_voltage((velocitiesRPM[1] / g_gearRatio2) * rpmToV);
            bottomLeft2.move_voltage((velocitiesRPM[0] / g_gearRatio2) * rpmToV);
            bottomRight2.move_voltage((velocitiesRPM[1] / g_gearRatio2) * rpmToV);
        }

        // 5 ms delay (- the time taken to calculate)
        pros::delay(delay);

        // if the current step is the final point (t = 1 - step), 
        // then the drivetrain is stopped and the function ends
        if (currentPoint.t == currentlyFollowing->profile[currentlyFollowing->profile.size() - 1].t) {
            if (currentlyFollowing->zones[currentlyFollowing->zones.size() - 1].zoneLine.slope + currentlyFollowing->zones[currentlyFollowing->zones.size() - 1].zoneLine.yIntercept == 0) {
                all6.brake();
                all2.brake();
            }
            //std::cout << (pros::millis() - startTime) / (double) 1000 << "\n";
            //std::cout << currentlyFollowing->totalTime << "\n";
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