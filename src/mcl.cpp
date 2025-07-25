#include "mcl.h"

ParticleFilter::ParticleFilter(TrackingSensor front, TrackingSensor left, TrackingSensor right, TrackingSensor headingTracker, Pose startPose, int numParticles) {
    m_front = front;
    m_left = left;
    m_right = right;
    m_heading = headingTracker;

    m_numParticles = numParticles;
    m_particles = new std::vector<Particle>();

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

void ParticleFilter::motionUpdate(double linVel, double angVel, double time, double linAngle) {

    std::normal_distribution<double> vNoiseDistributor(0, 5);
    std::normal_distribution<double> wNoiseDistributor(0, 3);
    std::normal_distribution<double> linAngleNoiseDistributor(0, 5);

    for (int i = 0; i < m_particles->size(); i++) {
        double linVelNoise = vNoiseDistributor(m_randengine);
        double angVelNoise = wNoiseDistributor(m_randengine);
        double linAngleNoise = linAngleNoiseDistributor(m_randengine);

        double linDistX = ((linVel + linVelNoise) * time) * std::cos((M_PI / 180) * (linAngle + linAngleNoise));
        double linDistY = ((linVel + linVelNoise) * time) * std::sin((M_PI / 180) * (linAngle + linAngleNoise));
        double angDist = (angVel + angVelNoise) * time;

        m_particles->operator[](i).x += linDistX;
        m_particles->operator[](i).y += linDistY;
        m_particles->operator[](i).heading += angDist;
    }
}

void ParticleFilter::sensorUpdate() {
    double heading = m_heading.get();
    double variance = 5;
    double stepRadius = 0.1;

    std::vector<double> sensorHeadingOffset = {0, -90, 90};
    std::vector<double> sensorDistanceOffset = {3, 3, 3};

    double actualF = m_front.get();
    double actualL = m_left.get();
    double actualR = m_right.get();
    std::vector<double> actuals = {actualF, actualL, actualR};

    for (int i = 0; i < m_particles->size(); i++) {
        
        double totalWeight = 0;

        for (int j = 0; j < 3; j++) {
            double sensorHeadingRadians = (M_PI / 180) * fixAngle(m_particles->operator[](i).heading + sensorHeadingOffset[j]);
            Point offset = {sensorDistanceOffset[j] * std::cos(sensorHeadingRadians), sensorDistanceOffset[j] * std::sin(sensorHeadingRadians)};
            Point start = {m_particles->operator[](i).x + offset.x, m_particles->operator[](i).y + offset.y};
            Point end = start;
            Point step = {stepRadius * std::cos(sensorHeadingRadians), stepRadius * std::sin(sensorHeadingRadians)};
            while (((end.x <= 72) && (end.x >= -72)) && ((end.y <= 72) && (end.y >= -72))) {
                end.x += step.x;
                end.y += step.y;
            }
            double distance = calculateDistance(start, end);

            totalWeight += std::pow(M_E, (-1 * std::pow(distance - actuals[j], 2)) / (2 * std::pow(variance, 2)));
        }
            totalWeight /= 3;

            m_particles->operator[](i).weight = totalWeight;
    }
}

void ParticleFilter::test(void) {
    //std::vector<Particle> initialParticles;
    /*std::cout << "[";
    for (int i = 0; i < m_numParticles; i++) {
        // std::cout << "(" << m_particles->operator[](i).x << ", " << m_particles->operator[](i).y << ")\n";
        std::cout << "(" << m_particles->operator[](i).x << ", " << m_particles->operator[](i).y << "), ";
        //initialParticles.push_back(m_particles->operator[](i));
    }
    std::cout << "\b\b]\n\n\n\n\nWITH WEIGHTS:\n\n\n\n\n\n";*/
    this->sensorUpdate();
    /*this->motionUpdate(10, 90, 0.25, fixAngle(270));
    for (int i = 0; i < m_numParticles; i++) {
        std::cout << "(1 - t)(" << initialParticles[i].x << ", " << initialParticles[i].y << ") + ";
        std::cout << "t(" << m_particles->operator[](i).x << ", " << m_particles->operator[](i).y << ")\n";
    }*/
   for (int i = 0; i < m_numParticles; i++) {
        std::cout << "Particle #" << i + 1 << " - (" << m_particles->operator[](i).x << ", " << m_particles->operator[](i).y << ", " << m_particles->operator[](i).heading << ", " << m_particles->operator[](i).weight << ")\n";
        //std::cout << m_particles->operator[](i).weight << ", ";
   }
}