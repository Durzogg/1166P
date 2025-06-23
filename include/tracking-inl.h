#include <functional>

// declaration
class TrackingSensor {
    public:
        TrackingSensor(std::function<double(void)> getValue, std::function<void(double val)> setValue, std::function<void(void)> reset);
        TrackingSensor() = default;
        double get(void);
        void set(double val);
        void reset(void);

    private:
        std::function<double(void)> m_getValue;
        std::function<void(double)> m_setValue;
        std::function<void(void)> m_reset;

};

// implementation
TrackingSensor::TrackingSensor(std::function<double(void)> getValue, std::function<void(double val)> setValue, std::function<void(void)> reset) {
    m_getValue = getValue;
    m_setValue = setValue;
    m_reset = reset;
}

double TrackingSensor::get() {
    return m_getValue();
}

void TrackingSensor::set(double val) {
    return m_setValue(val);
}

void TrackingSensor::reset(void) {
    return m_reset();
}