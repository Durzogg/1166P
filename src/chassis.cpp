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
        [this](double power) {m_yCorrect = power; std::cout << "pow = " << power << "\n";},
        [this]() {m_yCorrect = 0;}
    );
    m_thetaOutputCorrect = PowerUnit(
        [this](double power) {m_thetaCorrect = power;},
        [this]() {m_thetaCorrect = 0;}
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