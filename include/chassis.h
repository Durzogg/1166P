#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include <vector>
#include "main.h"
#include "proxy.h"
#include "pid.h"

class HoloChassis {
    public:
        HoloChassis(std::vector<pros::Motor*> FL, std::vector<pros::Motor*> FR, std::vector<pros::Motor*> BL, std::vector<pros::Motor*> BR);
        HoloChassis(std::vector<pros::Motor*> FL, std::vector<pros::Motor*> FR, std::vector<pros::Motor*> BL, std::vector<pros::Motor*> BR,
                    PIDController* xPID, PIDController* yPID);
        void addPID(PIDController* xPID, PIDController* yPID);
        ~HoloChassis();

        void move(void);
        void brake(void);
        void brakeMode(pros::MotorBrake type);
        void driverControl(pros::Controller controller, double dz);
        void continuousPower(void);
        void moveToPoint(Point localPoint, bool nonblocking = false);

        void setX(double power);
        void setY(double power);
        void setTheta(double power);


        PowerUnit m_xOutput;
        PowerUnit m_xOutputCorrect;

        PowerUnit m_yOutput;
        PowerUnit m_yOutputCorrect;

        PowerUnit m_thetaOutput;
        PowerUnit m_thetaOutputCorrect;

    private:
        std::vector<pros::Motor*> m_FL;
        std::vector<pros::Motor*> m_FR;
        std::vector<pros::Motor*> m_BL;
        std::vector<pros::Motor*> m_BR;

        double m_xPower;
        double m_xCorrect;

        double m_yPower;
        double m_yCorrect;

        double m_thetaPower;
        double m_thetaCorrect;

        PIDController* m_xPID;
        PIDController* m_yPID;
        bool m_hasPID;

        pros::Mutex chassisLock;
        pros::Task* chassisTask;
};

#endif