#ifndef _PIDH_
#define _PIDH_

#include "profiling.h"
#include "tracking-inl.h"
#include "main.h"

class PIDController {
    public:
        PIDController(TrackingSensor pidSensor, ConstantContainer constants, bool isMotion, Point goalPoint);

    private:
        PIDReturn calculateOutput(
            double distanceMoved, // current distance moved (in odometry units)
            double setPoint, // goal distance to move (in odometry units)
            bool isPositive, // direction of movement
            PIDReturn lastCycle // data from previous cycle
        );
        void movement(
            Point goalPosition, // goal coordinate position
            bool reverse, // defaults to false- explicitly set to true to reverse the robot
        
            std::vector<std::function<void(void)>> customs, // a lambda function that will execute during the PID (optional)
            std::vector<double> executeAts // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
        );

        TrackingSensor m_sensor;
        ConstantContainer m_constants;
        bool m_isMotion;

        double p_error;
        double p_integral;
};

#endif