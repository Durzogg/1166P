#include "mcl.h"

ParticleFilter::ParticleFilter(pros::Distance* front, pros::Distance* left, pros::Distance* right, Pose startPose, int numParticles) {
    m_front = front;
    m_left = left;
    m_right = right;

    m_numParticles = numParticles;
    m_particles = new std::vector<Pose>();

    std::random_device seeder = std::random_device();
    m_randengine = std::minstd_rand(seeder.operator()());

    if (startPose.heading == -1) {
        this->distributeParticles(numParticles);
    } else {
        this->distributeParticles(numParticles, startPose);
    }
}

void ParticleFilter::distributeParticles(int numParticles) {

    std::uniform_real_distribution<double> xyDistributor(-72.0, 72.0);
    std::uniform_real_distribution<double> thetaDistributor(0.0, 360.0);
    std::cout << "hallo\n";
    for (int i = 0; i < numParticles; i++) {
        m_particles->push_back({xyDistributor(m_randengine), xyDistributor(m_randengine), thetaDistributor(m_randengine)});
    }
}

void ParticleFilter::distributeParticles(int numParticles, Pose startPose) {

    double deviation = 10;
    std::cout << "hello\n";
    std::normal_distribution<double> xDistributor(startPose.x, deviation);
    std::normal_distribution<double> yDistributor(startPose.y, deviation);
    std::normal_distribution<double> thetaDistributor(startPose.heading, deviation);

    for (int i = 0; i < numParticles; i++) {
        double particleX = 1000;
        while ((particleX > 72) || (particleX < -72)) {
            particleX = xDistributor(m_randengine);
        }
        double particleY = 1000;
        while ((particleY > 72) || (particleY < -72)) {
            particleY = yDistributor(m_randengine);
        }
        double particleHeading = thetaDistributor(m_randengine);
        if (particleHeading < 0) {
            particleHeading += 360;
        } else if (particleHeading > 360) {
            particleHeading -= 360;
        }
        m_particles->push_back({particleX, particleY, particleHeading});
    }

}

void ParticleFilter::test(void) {
    for (int i = 0; i < m_numParticles; i++) {
        std::cout << "(" << m_particles->operator[](i).x << ", " << m_particles->operator[](i).y << ")\n";
    }
}