#ifndef _ODOM_H_
#define _ODOM_H_

#include "main.h"
#include "proxy.h"

class OdomPod {
    public:
        OdomPod(pros::Rotation* odom, double wheelDiameter);

        double measure(void);
        double measureVelocity(void);
        double measureHeading(void);

    private:
        pros::Rotation* m_odom;
        double m_diameter;
};

class Odometry {
    public:
        Odometry(
            TrackingSensor movementSensor, // this sensor should track how far the robot has moved in some way
            TrackingSensor headingSensor, // this sensor should track how far the robot has turned in some way
            Pose startPosition // the pose that the robot starts at at the start of autonomous
        );
        Odometry(
            TrackingSensor movementSensor, // this sensor should track how far the robot has moved in some way
            TrackingSensor headingSensor, // this sensor should track how far the robot has turned in some way
            Pose startPosition, // the pose that the robot starts at at the start of autonomous
            TrackingSensor sideSensor // side movement sensor (optional)
        );
        Point update(double heading, double moved);
        void updateLoop();
    
    private:
        void holoUpdateLoop();

        Pose m_robotPose;
        bool m_taskRunning;
        bool m_isHolo;
        bool m_isSide;
        pros::Task* loopTask;

        TrackingSensor m_movementSensor;
        TrackingSensor m_headingSensor;
        TrackingSensor m_sideSensor;
};

#endif