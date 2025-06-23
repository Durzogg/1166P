#include <vector>
#include "init.h"
#include "power-inl.h"

class HoloChassis {
    public:
        HoloChassis(std::vector<pros::Motor> FL, std::vector<pros::Motor> FR, std::vector<pros::Motor> BL, std::vector<pros::Motor> BR);

        void move(void);
        void brakeMode(pros::MotorBrake type);
        void driverControl(pros::Controller controller, double dz);

    private:
        std::vector<pros::Motor> m_FL;
        std::vector<pros::Motor> m_FR;
        std::vector<pros::Motor> m_BL;
        std::vector<pros::Motor> m_BR;

        PowerUnit m_xOutput;
        double m_xPower;

        PowerUnit m_yOutput;
        double m_yPower;

        PowerUnit m_thetaOutput;
        double m_thetaPower;
};