#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/Airfoil.hpp"
#include "core/Component.hpp"
#include "core/Parameter.hpp"

namespace opads {

/// Parametric multi-blade propeller/rotor with spinner hub and radial twist/chord lofting.
class Propeller : public Component {
public:
    Propeller(std::uint64_t id, std::string name = "Propeller");

    std::string getTypeName() const override;
    std::vector<std::vector<glm::vec3>> getHighlightLoops() const override;
    std::vector<glm::vec3> getSnapSamplePointsLocal() const override;

    Parameter& diameter() noexcept;
    const Parameter& diameter() const noexcept;

    Parameter& hubDiameter() noexcept;
    const Parameter& hubDiameter() const noexcept;

    Parameter& spinnerLength() noexcept;
    const Parameter& spinnerLength() const noexcept;

    Parameter& rootChord() noexcept;
    const Parameter& rootChord() const noexcept;

    Parameter& tipChord() noexcept;
    const Parameter& tipChord() const noexcept;

    Parameter& collectivePitchDeg() noexcept;
    const Parameter& collectivePitchDeg() const noexcept;

    Parameter& washoutDeg() noexcept;
    const Parameter& washoutDeg() const noexcept;

    Parameter& sweepDeg() noexcept;
    const Parameter& sweepDeg() const noexcept;

    int getBladeCount() const noexcept;
    void setBladeCount(int count);

    bool isClockwise() const noexcept;
    void setClockwise(bool cw);

    bool getShowDiskRing() const noexcept;
    void setShowDiskRing(bool show) noexcept;

    Airfoil& getBladeAirfoil() noexcept;
    const Airfoil& getBladeAirfoil() const noexcept;

    double getDiskArea() const noexcept;
    double getSolidity() const noexcept;

protected:
    void rebuildMesh() override;

private:
    Parameter m_diameter;
    Parameter m_hubDiameter;
    Parameter m_spinnerLength;
    Parameter m_rootChord;
    Parameter m_tipChord;
    Parameter m_collectivePitchDeg;
    Parameter m_washoutDeg;
    Parameter m_sweepDeg;

    int m_bladeCount{3};
    bool m_clockwise{true};
    bool m_showDiskRing{true};
    Airfoil m_bladeAirfoil;
};

} // namespace opads
