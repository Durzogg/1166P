#ifndef _MCL_H_
#define _MCL_H_

#include <random>
#include <memory>

#include "main.h"
#include "math.h"
#include "interface.h"

struct Particle {
    double x;
    double y;
    double heading;
    double weight = 0;
};

class ParticleFilter {
    public:
        ParticleFilter(TrackingSensor front, TrackingSensor left, TrackingSensor right, TrackingSensor headingTracker, Pose startPose = {0, 0, -1}, int numParticles = 500);
        void test();


    private:
        void distributeParticles(int numParticles);
        void distributeParticles(int numParticles, Pose startPose);

        void motionUpdate(double linVel, double angVel, double time, double linAngle);

        void sensorUpdate(void);

        void normalizeWeights(void);

        Pose estimatePosition(void);

        void resample(void);

        TrackingSensor m_front;
        TrackingSensor m_left;
        TrackingSensor m_right;
        TrackingSensor m_heading;

        int m_numParticles;

        std::vector<Particle>* m_particles;
        std::minstd_rand m_randengine;


};

#endif