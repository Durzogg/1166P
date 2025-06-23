#include "chassis.h"

HoloChassis::HoloChassis(std::vector<pros::Motor> FL, std::vector<pros::Motor> FR, std::vector<pros::Motor> BL, std::vector<pros::Motor> BR) {
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
}

void HoloChassis::move() {
    for (int i = 0; i < m_FL.size(); i++) {
        m_FL[i].move((m_yPower + m_xPower) + m_thetaPower);
    }
    for (int i = 0; i < m_FR.size(); i++) {
        m_FR[i].move((m_yPower - m_xPower) - m_thetaPower);
    }
    for (int i = 0; i < m_BL.size(); i++) {
        m_BL[i].move((m_yPower - m_xPower) + m_thetaPower);
    }
    for (int i = 0; i < m_BR.size(); i++) {
        m_BR[i].move((m_yPower + m_xPower) - m_thetaPower);
    }
}

void HoloChassis::driverControl(pros::Controller controller, double dz) {
    m_xPower = controller.get_analog(ANALOG_RIGHT_X);
    m_yPower = controller.get_analog(ANALOG_RIGHT_Y);
    m_thetaPower = controller.get_analog(ANALOG_LEFT_X);

    if ((m_xPower < dz) && (m_xPower > -dz)) {
        m_xPower = 0;
    }
    if ((m_yPower < dz) && (m_yPower > -dz)) {
        m_yPower = 0;
    }
}

void HoloChassis::brakeMode(pros::MotorBrake type) {
    for (int i = 0; i < m_FL.size(); i++) {
        m_FL[i].set_brake_mode(type);
    }
    for (int i = 0; i < m_FR.size(); i++) {
        m_FR[i].set_brake_mode(type);
    }
    for (int i = 0; i < m_BL.size(); i++) {
        m_BL[i].set_brake_mode(type);
    }
    for (int i = 0; i < m_BR.size(); i++) {
        m_BR[i].set_brake_mode(type);
    }
}