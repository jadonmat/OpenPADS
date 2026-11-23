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
    /// Optional radial blade station for exact non-linear chord/twist/sweep/rake distributions.
    struct BladeStation {
        double radiusFrac{0.2};
        double chord{0.15};
        double twistDeg{20.0};
        double sweepOffset{0.0};
        double rakeOffset{0.0};
    };

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

    bool hasSpinner() const noexcept;
    void setHasSpinner(bool enabled);

    const std::vector<BladeStation>& getBladeCurve() const noexcept;
    void setBladeCurve(std::vector<BladeStation> stations);
    void clearBladeCurve();

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
    bool m_hasSpinner{true};
    std::vector<BladeStation> m_bladeCurve;
    Airfoil m_bladeAirfoil;
};

} // namespace opads
