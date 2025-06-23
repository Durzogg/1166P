#include <functional>

// declaration
class PowerUnit {
    public:
        PowerUnit(std::function<void(double val)> move, std::function<void(void)> stop);
        PowerUnit() = default;

        void move(double val);
        void stop(void);

    private:
        std::function<void(double)> m_move;
        std::function<void(void)> m_stop;

};

// implementation
PowerUnit::PowerUnit(std::function<void(double val)> move, std::function<void(void)> stop) {
    m_move = move;
    m_stop = stop;
}

void PowerUnit::move(double val) {
    return m_move(val);
}

void PowerUnit::stop(void) {
    return m_stop();
}