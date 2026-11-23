#include "core/ControlSurface.hpp"

#include <utility>

namespace opads {

ControlSurface::ControlSurface()
    : m_spanStartFrac("Start Span", "b", 0.55, 0.0, 0.95, 0.02)
    , m_spanEndFrac("End Span", "b", 0.92, 0.05, 1.0, 0.02)
    , m_chordFrac("Chord Frac", "c", 0.25, 0.05, 0.50, 0.01)
    , m_deflectionDeg("Deflection", "deg", 0.0, -45.0, 45.0, 0.5) {
}

ControlSurface::ControlSurface(
    std::string name,
    std::size_t bayIndex,
    double spanStartFrac,
    double spanEndFrac,
    double chordFrac,
    double deflectionDeg,
    bool symmetric
)
    : ControlSurface() {
    m_name = std::move(name);
    m_bayIndex = bayIndex;
    m_spanStartFrac.setValue(spanStartFrac);
    m_spanEndFrac.setValue(spanEndFrac);
    m_chordFrac.setValue(chordFrac);
    m_deflectionDeg.setValue(deflectionDeg);
    m_symmetric = symmetric;
}

const std::string& ControlSurface::getName() const noexcept { return m_name; }
void ControlSurface::setName(std::string name) { m_name = std::move(name); }

std::size_t ControlSurface::getBayIndex() const noexcept { return m_bayIndex; }
void ControlSurface::setBayIndex(std::size_t index) noexcept { m_bayIndex = index; }

Parameter& ControlSurface::spanStartFrac() noexcept { return m_spanStartFrac; }
const Parameter& ControlSurface::spanStartFrac() const noexcept { return m_spanStartFrac; }

Parameter& ControlSurface::spanEndFrac() noexcept { return m_spanEndFrac; }
const Parameter& ControlSurface::spanEndFrac() const noexcept { return m_spanEndFrac; }

Parameter& ControlSurface::chordFrac() noexcept { return m_chordFrac; }
const Parameter& ControlSurface::chordFrac() const noexcept { return m_chordFrac; }

Parameter& ControlSurface::deflectionDeg() noexcept { return m_deflectionDeg; }
const Parameter& ControlSurface::deflectionDeg() const noexcept { return m_deflectionDeg; }

bool ControlSurface::isSymmetric() const noexcept { return m_symmetric; }
void ControlSurface::setSymmetric(bool symmetric) noexcept { m_symmetric = symmetric; }

} // namespace opads
