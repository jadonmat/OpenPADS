#include "core/WingSection.hpp"

#include <algorithm>
#include <cmath>

#include <glm/trigonometric.hpp>

namespace opads {

WingSection::WingSection()
    : m_span("Span", "m", 6.0, 0.05, 100.0, 0.1)
    , m_rootChord("Root Chord", "m", 2.2, 0.05, 50.0, 0.05)
    , m_tipChord("Tip Chord", "m", 1.1, 0.02, 50.0, 0.05)
    , m_sweepDeg("Sweep", "deg", 12.0, -75.0, 75.0, 0.5)
    , m_sweepLocation("Sweep Loc", "c", 0.25, 0.0, 1.0, 0.05)
    , m_dihedralDeg("Dihedral", "deg", 4.0, -89.0, 90.0, 0.5)
    , m_rootTwistDeg("Root Twist", "deg", 1.0, -45.0, 45.0, 0.25)
    , m_tipTwistDeg("Tip Twist", "deg", -2.0, -45.0, 45.0, 0.25)
    , m_twistLocation("Twist Loc", "c", 0.25, 0.0, 1.0, 0.05)
    , m_rootAirfoil(Airfoil::createNaca4("2412"))
    , m_tipAirfoil(Airfoil::createNaca4("2412")) {
}

Parameter& WingSection::span() noexcept { return m_span; }
const Parameter& WingSection::span() const noexcept { return m_span; }

Parameter& WingSection::rootChord() noexcept { return m_rootChord; }
const Parameter& WingSection::rootChord() const noexcept { return m_rootChord; }

Parameter& WingSection::tipChord() noexcept { return m_tipChord; }
const Parameter& WingSection::tipChord() const noexcept { return m_tipChord; }

Parameter& WingSection::sweepDeg() noexcept { return m_sweepDeg; }
const Parameter& WingSection::sweepDeg() const noexcept { return m_sweepDeg; }

Parameter& WingSection::sweepLocation() noexcept { return m_sweepLocation; }
const Parameter& WingSection::sweepLocation() const noexcept { return m_sweepLocation; }

Parameter& WingSection::dihedralDeg() noexcept { return m_dihedralDeg; }
const Parameter& WingSection::dihedralDeg() const noexcept { return m_dihedralDeg; }

Parameter& WingSection::rootTwistDeg() noexcept { return m_rootTwistDeg; }
const Parameter& WingSection::rootTwistDeg() const noexcept { return m_rootTwistDeg; }

Parameter& WingSection::tipTwistDeg() noexcept { return m_tipTwistDeg; }
const Parameter& WingSection::tipTwistDeg() const noexcept { return m_tipTwistDeg; }

Parameter& WingSection::twistLocation() noexcept { return m_twistLocation; }
const Parameter& WingSection::twistLocation() const noexcept { return m_twistLocation; }

Airfoil& WingSection::getRootAirfoil() noexcept { return m_rootAirfoil; }
const Airfoil& WingSection::getRootAirfoil() const noexcept { return m_rootAirfoil; }

Airfoil& WingSection::getTipAirfoil() noexcept { return m_tipAirfoil; }
const Airfoil& WingSection::getTipAirfoil() const noexcept { return m_tipAirfoil; }

int WingSection::getSpanSubdivisions() const noexcept {
    return m_spanSubdivisions;
}

void WingSection::setSpanSubdivisions(int count) {
    m_spanSubdivisions = std::clamp(count, 1, 64);
}

double WingSection::getArea() const noexcept {
    return 0.5 * m_span.getValue() * (m_rootChord.getValue() + m_tipChord.getValue());
}

double WingSection::getProjectedYSpan() const noexcept {
    return m_span.getValue() * std::cos(glm::radians(m_dihedralDeg.getValue()));
}

double WingSection::getTaperRatio() const noexcept {
    const double cr = m_rootChord.getValue();
    return (cr > 1e-9) ? (m_tipChord.getValue() / cr) : 1.0;
}

double WingSection::getMeanAerodynamicChord() const noexcept {
    const double cr = m_rootChord.getValue();
    const double lambda = getTaperRatio();
    return (2.0 / 3.0) * cr * (1.0 + lambda + lambda * lambda) / (1.0 + lambda);
}

glm::vec3 WingSection::computeTipLeadingEdge(const glm::vec3& rootLE) const noexcept {
    const float s = m_span.getFloat();
    const float cr = m_rootChord.getFloat();
    const float ct = m_tipChord.getFloat();
    const float sweepRad = glm::radians(m_sweepDeg.getFloat());
    const float dihedralRad = glm::radians(m_dihedralDeg.getFloat());
    const float f = m_sweepLocation.getFloat();

    const float dx = f * (cr - ct) + s * std::tan(sweepRad);
    const float dy = s * std::cos(dihedralRad);
    const float dz = s * std::sin(dihedralRad);
    return rootLE + glm::vec3(dx, dy, dz);
}

} // namespace opads
