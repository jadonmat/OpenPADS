#pragma once

#include <string>

namespace opads {

/// Bounded engineering parameter with units, range clamping, and change detection.
class Parameter {
public:
    Parameter() = default;
    Parameter(
        std::string name,
        std::string units,
        double defaultValue,
        double minValue,
        double maxValue,
        double step = 0.1
    );

    const std::string& getName() const noexcept;
    const std::string& getUnits() const noexcept;

    double getValue() const noexcept;
    float getFloat() const noexcept;
    double getMin() const noexcept;
    double getMax() const noexcept;
    double getStep() const noexcept;

    /// Clamps to [min, max] and returns true when the value changed.
    bool setValue(double value) noexcept;
    bool setFloat(float value) noexcept;

private:
    std::string m_name;
    std::string m_units;
    double m_value{0.0};
    double m_min{0.0};
    double m_max{1.0};
    double m_step{0.1};
};

} // namespace opads
