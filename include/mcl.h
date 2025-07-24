#ifndef _MCL_H_
#define _MCL_H_

#include <random>
#include <memory>

#include "main.h"
#include "math.h"

class ParticleFilter {
    public:
        ParticleFilter(pros::Distance* front, pros::Distance* left, pros::Distance* right, Pose startPose = {0, 0, -1}, int numParticles = 500);
        void test();


    private:
        void distributeParticles(int numParticles);
        void distributeParticles(int numParticles, Pose startPose);

        pros::Distance* m_front;
        pros::Distance* m_left;
        pros::Distance* m_right;

        int m_numParticles;

        std::vector<Pose>* m_particles;
        std::minstd_rand m_randengine;


};

#endif