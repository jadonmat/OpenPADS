#include "core/Parameter.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace opads {

Parameter::Parameter(
    std::string name,
    std::string units,
    double defaultValue,
    double minValue,
    double maxValue,
    double step
)
    : m_name(std::move(name))
    , m_units(std::move(units))
    , m_value(std::clamp(defaultValue, minValue, maxValue))
    , m_min(minValue)
    , m_max(maxValue)
    , m_step(step) {
}

const std::string& Parameter::getName() const noexcept {
    return m_name;
}

const std::string& Parameter::getUnits() const noexcept {
    return m_units;
}

double Parameter::getValue() const noexcept {
    return m_value;
}

float Parameter::getFloat() const noexcept {
    return static_cast<float>(m_value);
}

double Parameter::getMin() const noexcept {
    return m_min;
}

double Parameter::getMax() const noexcept {
    return m_max;
}

double Parameter::getStep() const noexcept {
    return m_step;
}

bool Parameter::setValue(double value) noexcept {
    const double clamped = std::clamp(value, m_min, m_max);
    if (std::abs(clamped - m_value) <= 1e-12) {
        return false;
    }
    m_value = clamped;
    return true;
}

bool Parameter::setFloat(float value) noexcept {
    return setValue(static_cast<double>(value));
}

} // namespace opads
