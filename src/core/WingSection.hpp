#pragma once

#include <glm/vec3.hpp>

#include "core/Airfoil.hpp"
#include "core/Parameter.hpp"

namespace opads {

/// One spanwise trapezoidal bay between a root and tip airfoil cross-section.
class WingSection {
public:
    WingSection();

    Parameter& span() noexcept;
    const Parameter& span() const noexcept;

    Parameter& rootChord() noexcept;
    const Parameter& rootChord() const noexcept;

    Parameter& tipChord() noexcept;
    const Parameter& tipChord() const noexcept;

    Parameter& sweepDeg() noexcept;
    const Parameter& sweepDeg() const noexcept;

    Parameter& sweepLocation() noexcept;
    const Parameter& sweepLocation() const noexcept;

    Parameter& dihedralDeg() noexcept;
    const Parameter& dihedralDeg() const noexcept;

    Parameter& rootTwistDeg() noexcept;
    const Parameter& rootTwistDeg() const noexcept;

    Parameter& tipTwistDeg() noexcept;
    const Parameter& tipTwistDeg() const noexcept;

    Parameter& twistLocation() noexcept;
    const Parameter& twistLocation() const noexcept;

    Airfoil& getRootAirfoil() noexcept;
    const Airfoil& getRootAirfoil() const noexcept;

    Airfoil& getTipAirfoil() noexcept;
    const Airfoil& getTipAirfoil() const noexcept;

    int getSpanSubdivisions() const noexcept;
    void setSpanSubdivisions(int count);

    /// Panel planform area 0.5 * span * (cr + ct).
    double getArea() const noexcept;
    /// Projected Y span (span * cos(dihedral)).
    double getProjectedYSpan() const noexcept;
    /// Taper ratio ct / cr.
    double getTaperRatio() const noexcept;
    /// Mean aerodynamic chord of this trapezoidal bay.
    double getMeanAerodynamicChord() const noexcept;
    /// Computes tip leading-edge position from root leading-edge position.
    glm::vec3 computeTipLeadingEdge(const glm::vec3& rootLE) const noexcept;

private:
    Parameter m_span;
    Parameter m_rootChord;
    Parameter m_tipChord;
    Parameter m_sweepDeg;
    Parameter m_sweepLocation;
    Parameter m_dihedralDeg;
    Parameter m_rootTwistDeg;
    Parameter m_tipTwistDeg;
    Parameter m_twistLocation;

    Airfoil m_rootAirfoil;
    Airfoil m_tipAirfoil;
    int m_spanSubdivisions{8};
};

} // namespace opads
