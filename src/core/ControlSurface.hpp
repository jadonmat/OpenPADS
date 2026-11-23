#pragma once

#include <cstddef>
#include <string>

#include "core/Parameter.hpp"

namespace opads {

/// Trailing-edge control surface attached to a Wing bay (aileron, flap, elevator, rudder).
class ControlSurface {
public:
    ControlSurface();
    ControlSurface(
        std::string name,
        std::size_t bayIndex,
        double spanStartFrac,
        double spanEndFrac,
        double chordFrac,
        double deflectionDeg,
        bool symmetric
    );

    const std::string& getName() const noexcept;
    void setName(std::string name);

    std::size_t getBayIndex() const noexcept;
    void setBayIndex(std::size_t index) noexcept;

    Parameter& spanStartFrac() noexcept;
    const Parameter& spanStartFrac() const noexcept;

    Parameter& spanEndFrac() noexcept;
    const Parameter& spanEndFrac() const noexcept;

    Parameter& chordFrac() noexcept;
    const Parameter& chordFrac() const noexcept;

    Parameter& deflectionDeg() noexcept;
    const Parameter& deflectionDeg() const noexcept;

    bool isSymmetric() const noexcept;
    void setSymmetric(bool symmetric) noexcept;

private:
    std::string m_name{"Aileron"};
    std::size_t m_bayIndex{0};
    Parameter m_spanStartFrac;
    Parameter m_spanEndFrac;
    Parameter m_chordFrac;
    Parameter m_deflectionDeg;
    bool m_symmetric{false};
};

} // namespace opads
