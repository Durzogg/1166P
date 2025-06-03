/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * \copyright Copyright (c) 2017-2023, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convenient for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"

/**
 * You should add more #includes here
 */
//#include "okapi/api.hpp"
#include <list>
#include <cmath>
#include <functional>
#include <vector>
#include <deque>
#include <cstdio>
#include <string>

#include "profiling.h"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

#define waitUntil(condition) while (!(condition)) { pros::delay(50); }

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

// structures
struct Inequality {
    double slope;
    double yIntercept;
    int equality;
};
struct PIDReturn {
    double prevError;
    double prevIntegral; 
    int power;
};
struct ConstantContainer {
    double kP; 
    double kI;
    double kD;
};

// pid.cpp
void PIDMoverBasic(void);
void PIDMover(Point goalPosition, bool reverse = false,                 std::vector<std::function<void(void)>> custom = {}, std::vector<double> executeAts = {});
void PIDTurner(int setPoint, int direction,                 std::vector<std::function<void(void)>> custom = {}, std::vector<int> executeAt = {});
void PIDArc(int chordLength, int maxDist, int direction,                std::vector<std::function<void(void)>> custom = {}, std::vector<int> executeAt = {});

PIDReturn PIDCalc(double distanceMoved, double setPoint, bool isPositive, ConstantContainer constants, PIDReturn lastCycle);

//hif
// kalman.cpp
class KalmanFilter {
    private:
        // instance variables
        pros::IMU* inertial; // defined in constructor
        pros::Rotation* turnRotational; // defined in constructor
        pros::Task* filterLoop_ptr; // is initialized when the Kalman filter turns on

        double filteredHeading; // updates as Kalman filter runs
        double filterUncertainty; // updates as Kalman filter runs

        int delay; // time between cycles

        std::deque<double> measurementVariances; // list of all measurement variances from the estimate
        std::deque<double> predictionVariances; // list of all prediction variances from the estimate


        // internal methods
        void KalmanFilterLoop(void); // actual filter


    public:
        // public methods
        KalmanFilter(pros::IMU* inertial, pros::Rotation* turnRotational); // constructor

        // return methods for the filter, updated constantly as the filter runs
        double getFilteredHeading(void);
        double getFilterUncertainty(void);

        // start and stop methods for the filter
        void startFilter(void);
        void endFilter(void);
};

// fileio.cpp
class File {
    private:
        std::string filename;

    public:
        File(std::string filename);
        std::string readFile(void);
        void writeFile(std::string text);
        void appendFile(std::string text);
        void writeLoop(void);
};

// autons.cpp
    // AWP autons
    void AWP(int color);
    void autoTest(void);

    // specific autons
    void GoalRush(int color);
    void RingSide(int color);
    void autoSkills(void);

// draw.cpp
void drawAutonSelector(void);
void drawBasicSelector(void);

// odom.cpp
void initializeRobotOnCoordinate(pros::Rotation *rotational, pros::Imu *imu1, pros::Imu *imu2, Point offset, int startHeading);
Point updateLocation(double heading, double dist);
void updateCoordinateLoop(void);

// math.cpp
double calculateDistance(Point point1, Point point2);
double calculateStandardDeviation(std::deque<double> listOfDifferences);
Line calculatePerpendicularNonInequality(Point point1, Point point2);
Inequality calculatePerpendicularInequality(Point point1, Point point2);
int findEquality(Inequality line, Point includedPoint);
Point findIntersection(Line line1, Line line2);
Line findLineWithHeading(Point point1, int heading);
double findHeadingOfLine(Point point1, Point point2);
Line findLineWithPoints(Point point1, Point point2);
QuadraticPolyData derivativeOfCubicPoly(CubicPolyData cubicPoly);
Line derivativeOfQuadratic(QuadraticPolyData quadPoly);
double fixAngle(double originalAngle);
double RPMtoIPS(double rpm);
double IPStoRPM(double ips);


// tracking.cpp
double calculateSingleDegree(double wheelDiameter);
double readOdomPod(pros::Rotation odom);
double readOdomVelocity(pros::Rotation odom);
double readOdomAngle(pros::Rotation turnOdom);
double getAggregatedHeading(KalmanFilter inertial1, KalmanFilter inertial2);
void bindTurnTrackingWheelHeading();

// sidetasks.cpp
void ArmMacros(void);
void eject(void);
void autoEject(void);
void CutoffPID(Point goalPoint, bool reverse, double maxAllowableTime);
void CutoffTurnPID(Point goalPoint, bool reverse, double maxAllowableTime, int direction);
void CutoffTurnHeadingPID(int goalHeading, bool reverse, double maxAllowableTime, int direction);
void coords(void);

// profiles.cpp
std::vector<MotionProfile*> AWPSetup(int color);
std::vector<MotionProfile*> GoalRushSetup(int color);
std::vector<MotionProfile*> RingSetup(int color);
std::vector<MotionProfile*> SkillsSetup(void);

#endif  // _PROS_MAIN_H_
