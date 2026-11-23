#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/Component.hpp"
#include "core/Parameter.hpp"

namespace opads {

/// Parametric aerodynamic pod, engine nacelle, or external store component.
class Pod : public Component {
public:
    Pod(std::uint64_t id, std::string name = "Pod / Nacelle");

    std::string getTypeName() const override;
    std::vector<glm::vec3> getSnapSamplePointsLocal() const override;

    Parameter& length() noexcept;
    const Parameter& length() const noexcept;

    Parameter& width() noexcept;
    const Parameter& width() const noexcept;

    Parameter& height() noexcept;
    const Parameter& height() const noexcept;

    Parameter& noseFrac() noexcept;
    const Parameter& noseFrac() const noexcept;

    Parameter& tailFrac() noexcept;
    const Parameter& tailFrac() const noexcept;

    Parameter& nosePower() noexcept;
    const Parameter& nosePower() const noexcept;

    Parameter& tailPower() noexcept;
    const Parameter& tailPower() const noexcept;

    Parameter& superEllipseN() noexcept;
    const Parameter& superEllipseN() const noexcept;

    SymmetryMode getSymmetry() const noexcept;
    void setSymmetry(SymmetryMode mode);

    int getRadialPoints() const noexcept;
    void setRadialPoints(int count);

    int getAxialStations() const noexcept;
    void setAxialStations(int count);

    double getFinenessRatio() const noexcept;
    double getWettedArea() const noexcept;
    double getVolume() const noexcept;

protected:
    void rebuildMesh() override;

private:
    Parameter m_length;
    Parameter m_width;
    Parameter m_height;
    Parameter m_noseFrac;
    Parameter m_tailFrac;
    Parameter m_nosePower;
    Parameter m_tailPower;
    Parameter m_superEllipseN;

    SymmetryMode m_symmetry{SymmetryMode::None};
    int m_radialPoints{28};
    int m_axialStations{24};
    double m_wettedArea{0.0};
    double m_volume{0.0};
};

} // namespace opads
