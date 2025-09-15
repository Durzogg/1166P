#include "chassis.h"

HoloChassis::HoloChassis(std::vector<pros::Motor*> FL, std::vector<pros::Motor*> FR, std::vector<pros::Motor*> BL, std::vector<pros::Motor*> BR) {
    m_FL = FL;
    m_FR = FR;
    m_BL = BL;
    m_BR = BR;

    m_xOutput = PowerUnit(
        [this](double power) {m_xPower = power;},
        [this]() {m_xPower = 0;}
    );
    m_yOutput = PowerUnit(
        [this](double power) {m_yPower = power;},
        [this]() {m_yPower = 0;}
    );
    m_thetaOutput = PowerUnit(
        [this](double power) {m_thetaPower = power;},
        [this]() {m_thetaPower = 0;}
    );



    m_xOutputCorrect = PowerUnit(
        [this](double power) {m_xCorrect = power;},
        [this]() {m_xCorrect = 0;}
    );
    m_yOutputCorrect = PowerUnit(
        [this](double power) {m_yCorrect = power;},
        [this]() {m_yCorrect = 0;}
    );
    m_thetaOutputCorrect = PowerUnit(
        [this](double power) {m_thetaCorrect = power;},
        [this]() {m_thetaCorrect = 0;}
    );

    m_lVel = TrackingSensor(
        [this]() -> double {
            double xrpmSpeed = (480 / 128) * (m_xPower + m_xCorrect);
            double xipsSpeed = RPMtoIPS(xrpmSpeed);

            double yrpmSpeed = (480 / 128) * (m_yPower + m_yCorrect);
            double yipsSpeed = RPMtoIPS(yrpmSpeed);

            //std::cout << "lv: " << std::sqrt(std::pow(xipsSpeed, 2) + std::pow(yipsSpeed, 2)) << "\n";

            return std::sqrt(std::pow(xipsSpeed, 2) + std::pow(yipsSpeed, 2));
        },
        [this](double val) {
            return;
        },
        [this]() {
            return;
        }
    );
    m_lAng = TrackingSensor(
        [this]() -> double {
            double xrpmSpeed = (480 / 128) * (m_xPower + m_xCorrect);
            double xipsSpeed = RPMtoIPS(xrpmSpeed);

            double yrpmSpeed = (480 / 128) * (m_yPower + m_yCorrect);
            double yipsSpeed = RPMtoIPS(yrpmSpeed);

            double angle = (180 / M_PI) * std::atan2(xipsSpeed, yipsSpeed);
            if (angle < 0) {angle += 360;}

            return angle;
        },
        [this](double val) {
            return;
        },
        [this]() {
            return;
        }
    );
    m_aVel = TrackingSensor(
        [this]() -> double {
            double g_distBetweenWheels = 7.5;
            double maxAngVel = (RPMtoIPS(480) / (g_distBetweenWheels / 2));
            double angRadSpeed = (maxAngVel / 128) * (m_thetaPower + m_thetaCorrect);
            return angRadSpeed;
        },
        [this](double val) {
            return;
        },
        [this]() {
            return;
        }
    );

    m_xPower = 0;
    m_yPower = 0;
    m_thetaPower = 0;

    m_xCorrect = 0;
    m_yCorrect = 0;
    m_thetaCorrect = 0;

    m_xPID = NULL;
    m_yPID = NULL;
    m_hasPID = false;

    chassisTask = NULL;
}

HoloChassis::HoloChassis(std::vector<pros::Motor*> FL, std::vector<pros::Motor*> FR, std::vector<pros::Motor*> BL, std::vector<pros::Motor*> BR,
                         PIDController* xPID, PIDController* yPID) 
            : HoloChassis(FL, FR, BL, BR)
{
    m_xPID = xPID;
    m_yPID = yPID;
    m_hasPID = true;
}

void HoloChassis::addPID(PIDController* xPID, PIDController* yPID) {
    m_xPID = xPID;
    m_yPID = yPID;
    m_hasPID = true;
}

HoloChassis::~HoloChassis() {
    if (chassisTask == NULL) {
        chassisTask->remove();
    }
}

void HoloChassis::move() {
    m_xCorrect = 0;
    m_yCorrect = 0;
    m_thetaCorrect = 0;
    for (int i = 0; i < m_FL.size(); i++) {
        m_FL[i]->move(((m_xPower + m_xCorrect) + (m_yPower + m_yCorrect)) + (m_thetaPower + m_thetaCorrect));
    }
    for (int i = 0; i < m_FR.size(); i++) {
        m_FR[i]->move(((m_xPower + m_xCorrect) - (m_yPower + m_yCorrect)) - (m_thetaPower + m_thetaCorrect));
    }
    for (int i = 0; i < m_BL.size(); i++) {
        m_BL[i]->move(((m_xPower + m_xCorrect) - (m_yPower + m_yCorrect)) + (m_thetaPower + m_thetaCorrect));
    }
    for (int i = 0; i < m_BR.size(); i++) {
        m_BR[i]->move(((m_xPower + m_xCorrect) + (m_yPower + m_yCorrect)) - (m_thetaPower + m_thetaCorrect));
    }
}

void HoloChassis::brake() {
    for (int i = 0; i < m_FL.size(); i++) {
        m_FL[i]->brake();
    }
    for (int i = 0; i < m_FR.size(); i++) {
        m_FR[i]->brake();
    }
    for (int i = 0; i < m_BL.size(); i++) {
        m_BL[i]->brake();
    }
    for (int i = 0; i < m_BR.size(); i++) {
        m_BR[i]->brake();
    }
}

void HoloChassis::driverControl(pros::Controller controller, double dz) {
    m_xPower = controller.get_analog(ANALOG_RIGHT_Y);
    m_yPower = controller.get_analog(ANALOG_RIGHT_X);
    m_thetaPower = controller.get_analog(ANALOG_LEFT_X);

    if ((m_xPower < dz) && (m_xPower > -dz)) {
        m_xPower = 0;
    }
    if ((m_yPower < dz) && (m_yPower > -dz)) {
        m_yPower = 0;
    }
}

void HoloChassis::setX(double power) {
    m_xPower = power;
}

void HoloChassis::setY(double power) {
    m_yPower = power;
}

void HoloChassis::setTheta(double power) {
    m_thetaPower = power;
}

void HoloChassis::brakeMode(pros::MotorBrake type) {
    for (int i = 0; i < m_FL.size(); i++) {
        m_FL[i]->set_brake_mode(type);
    }
    for (int i = 0; i < m_FR.size(); i++) {
        m_FR[i]->set_brake_mode(type);
    }
    for (int i = 0; i < m_BL.size(); i++) {
        m_BL[i]->set_brake_mode(type);
    }
    for (int i = 0; i < m_BR.size(); i++) {
        m_BR[i]->set_brake_mode(type);
    }
}

void HoloChassis::continuousPower(void) {
    bool hasNotStarted = chassisLock.try_lock();
    if (hasNotStarted) {
        chassisTask = new pros::Task([this](){
                while (true) {
                this->move();
                pros::delay(5);
            }
        });
    }
}

void HoloChassis::moveToPoint(Point localPoint, bool nonblocking) {
    if (!m_hasPID) {
        return;
    }
    bool wasCont = !chassisLock.try_lock();
    if (!wasCont) {
        chassisLock.unlock();
        this->continuousPower();
    }
    m_xPID->movement(localPoint.x, true);
    m_yPID->movement(localPoint.y, nonblocking);
    if (!wasCont) {
        // remove chassis task
    }
    return;
}






DiffChassis::DiffChassis(std::vector<pros::Motor*> left, std::vector<pros::Motor*> right) {
    m_left = left;
    m_right = right;

    m_fbOutput = PowerUnit(
        [this](double power) {m_fbPower = power;},
        [this]() {m_fbPower = 0;}
    );
    m_thetaOutput = PowerUnit(
        [this](double power) {m_thetaPower = power;},
        [this]() {m_thetaPower = 0;}
    );



    m_fbOutputCorrect = PowerUnit(
        [this](double power) {m_fbCorrect = power;},
        [this]() {m_fbCorrect = 0;}
    );
    m_thetaOutputCorrect = PowerUnit(
        [this](double power) {m_thetaCorrect = power;},
        [this]() {m_thetaCorrect = 0;}
    );

    m_lVel = TrackingSensor(
        [this]() -> double {
            double xrpmSpeed = (450 / 128) * (m_fbPower + m_fbCorrect);
            double xipsSpeed = RPMtoIPS(xrpmSpeed);

            //std::cout << "lv: " << std::sqrt(std::pow(xipsSpeed, 2) + std::pow(yipsSpeed, 2)) << "\n";

            return xipsSpeed;
        },
        [this](double val) {
            return;
        },
        [this]() {
            return;
        }
    );
    m_lAng = TrackingSensor(
        [this]() -> double {
            return 0;
        },
        [this](double val) {
            return;
        },
        [this]() {
            return;
        }
    );
    m_aVel = TrackingSensor(
        [this]() -> double {
            double g_distBetweenWheels = 13;
            double maxAngVel = ((2 * RPMtoIPS(450)) / g_distBetweenWheels);
            double angRadSpeed = (maxAngVel / 128) * (m_thetaPower + m_thetaCorrect);
            return angRadSpeed;
        },
        [this](double val) {
            return;
        },
        [this]() {
            return;
        }
    );

    m_fbPower = 0;
    m_thetaPower = 0;

    m_fbCorrect = 0;
    m_thetaCorrect = 0;

    m_fbPID = NULL;
    m_thetaPID = NULL;
    m_hasPID = false;

    chassisTask = NULL;
}

DiffChassis::DiffChassis(std::vector<pros::Motor*> left, std::vector<pros::Motor*> right,
                         PIDController* fbPID, PIDController* thetaPID) 
            : DiffChassis(left, right)
{
    m_fbPID = fbPID;
    m_thetaPID = thetaPID;
    m_hasPID = true;
}

void DiffChassis::addPID(PIDController* fbPID, PIDController* thetaPID) {
    m_fbPID = fbPID;
    m_thetaPID = thetaPID;
    m_hasPID = true;
}

DiffChassis::~DiffChassis() {
    if (chassisTask == NULL) {
        chassisTask->remove();
    }
}

void DiffChassis::move() {
    m_fbCorrect = 0;
    m_thetaCorrect = 0;
    double g_distBetweenWheels = 13;
    for (int i = 0; i < m_left.size(); i++) {
        m_left[i]->move((m_fbPower + m_fbCorrect) - (((m_thetaPower + m_thetaCorrect) * g_distBetweenWheels) / 2));
    }
    for (int i = 0; i < m_right.size(); i++) {
        m_right[i]->move((m_fbPower + m_fbCorrect) + (((m_thetaPower + m_thetaCorrect) * g_distBetweenWheels) / 2));
    }
}

void DiffChassis::brake() {
    for (int i = 0; i < m_left.size(); i++) {
        m_left[i]->brake();
    }
    for (int i = 0; i < m_right.size(); i++) {
        m_right[i]->brake();
    }
}

void DiffChassis::driverControl(pros::Controller controller, double dz) {
    m_fbPower = controller.get_analog(ANALOG_RIGHT_Y);
    m_thetaPower = controller.get_analog(ANALOG_LEFT_X);

    if ((m_fbPower < dz) && (m_fbPower > -dz)) {
        m_fbPower = 0;
    }
    if ((m_thetaPower < dz) && (m_thetaPower > -dz)) {
        m_thetaPower = 0;
    }
}

void DiffChassis::setFB(double power) {
    m_fbPower = power;
}

void DiffChassis::setTheta(double power) {
    m_thetaPower = power;
}

void DiffChassis::brakeMode(pros::MotorBrake type) {
    for (int i = 0; i < m_left.size(); i++) {
        m_left[i]->set_brake_mode(type);
    }
    for (int i = 0; i < m_right.size(); i++) {
        m_right[i]->set_brake_mode(type);
    }
}

void DiffChassis::continuousPower(void) {
    bool hasNotStarted = chassisLock.try_lock();
    if (hasNotStarted) {
        chassisTask = new pros::Task([this](){
                while (true) {
                this->move();
                pros::delay(5);
            }
        });
    }
}

void DiffChassis::moveToPoint(Point localPoint, bool nonblocking) {
    if (!m_hasPID) {
        return;
    }
    bool wasCont = !chassisLock.try_lock();
    if (!wasCont) {
        chassisLock.unlock();
        this->continuousPower();
    }
    m_thetaPID->movement(findHeadingOfLine({0, 0}, localPoint), false);
    m_fbPID->movement(calculateDistance({0, 0}, localPoint), nonblocking);
    if (!wasCont) {
        // remove chassis task
    }
    return;
}