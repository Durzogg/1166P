#ifndef _PIDH_
#define _PIDH_

#include "profiling.h"
#include "tracking-inl.h"
#include "power-inl.h"
#include "main.h"

class PIDController {
    public:
        PIDController(
            TrackingSensor pidSensor, // sensor that the PID will receive feedback from
            ConstantContainer constants, // PID tuning constants for the controller (must be tuned for every robot)
            PowerUnit output, // motor setup that the PID will provide power to
            double tolerance // the acceptable deviation from the set point (inches)
        );
        PIDController() = default;
        ~PIDController();

        void movement(
            double setPoint, // goal coordinate position
            bool reverse = false, // defaults to false- explicitly set to true to reverse the robot
            bool enableGoalModification = false, // defaults to false- explicitly set to true to make controller run as task and enable continuous modification to set point
        
            std::vector<std::function<void(void)>> customs, // a lambda function that will execute during the PID (optional)
            std::vector<double> executeAts // the distance point (in inches) that you want to trigger the custom lambda function at (optional)
        );

        void modMovement(
            double setPoint
        );
    
    
    private:
        double calculateOutput(
            double distanceMoved, // current distance moved (in odometry units)
            double setPoint // goal distance to move (in odometry units)
        );
        TrackingSensor m_sensor;
        ConstantContainer m_constants;
        PowerUnit m_output;
        double m_tolerance;
        pros::Task* m_movementTask;
        pros::Mutex movementLock;

        double m_setPointMod;
        bool m_modFresh;

        double p_error = 0;
        double p_integral = 0;

        double p_time = 0;
};

#endif