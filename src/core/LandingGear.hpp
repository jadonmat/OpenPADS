#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/Component.hpp"
#include "core/Parameter.hpp"

namespace opads {

/// Parametric landing gear assembly (strut, axle, single/dual wheels, optional wheel pant).
class LandingGear : public Component {
public:
    LandingGear(std::uint64_t id, std::string name = "Landing Gear");

    std::string getTypeName() const override;
    std::vector<glm::vec3> getSnapSamplePointsLocal() const override;

    Parameter& strutLength() noexcept;
    const Parameter& strutLength() const noexcept;

    Parameter& strutDiameter() noexcept;
    const Parameter& strutDiameter() const noexcept;

    Parameter& rakeDeg() noexcept;
    const Parameter& rakeDeg() const noexcept;

    Parameter& splayDeg() noexcept;
    const Parameter& splayDeg() const noexcept;

    Parameter& wheelDiameter() noexcept;
    const Parameter& wheelDiameter() const noexcept;

    Parameter& wheelWidth() noexcept;
    const Parameter& wheelWidth() const noexcept;

    int getWheelsPerStrut() const noexcept;
    void setWheelsPerStrut(int count);

    bool hasWheelPant() const noexcept;
    void setWheelPant(bool enabled);

    SymmetryMode getSymmetry() const noexcept;
    void setSymmetry(SymmetryMode mode);

    double getFrontalArea() const noexcept;
    double getWettedArea() const noexcept;

protected:
    void rebuildMesh() override;

private:
    Parameter m_strutLength;
    Parameter m_strutDiameter;
    Parameter m_rakeDeg;
    Parameter m_splayDeg;
    Parameter m_wheelDiameter;
    Parameter m_wheelWidth;

    int m_wheelsPerStrut{1};
    bool m_wheelPant{false};
    SymmetryMode m_symmetry{SymmetryMode::XZ};
    double m_wettedArea{0.0};
};

} // namespace opads
