#pragma once

#include <vector>

#include <glm/vec2.hpp>

#include "core/Parameter.hpp"

namespace opads {

enum class CrossSectionShape {
    Point,
    Circle,
    Ellipse,
    SuperEllipse,
    RoundedRect
};

/// One longitudinal cross-section station along a Fuselage.
class FuselageSection {
public:
    FuselageSection();
    FuselageSection(
        CrossSectionShape shape,
        double stationFrac,
        double width,
        double height,
        double zOffset = 0.0,
        double superEllipseN = 2.5
    );

    CrossSectionShape getShape() const noexcept;
    void setShape(CrossSectionShape shape) noexcept;

    Parameter& stationFrac() noexcept;
    const Parameter& stationFrac() const noexcept;

    Parameter& width() noexcept;
    const Parameter& width() const noexcept;

    Parameter& height() noexcept;
    const Parameter& height() const noexcept;

    Parameter& yOffset() noexcept;
    const Parameter& yOffset() const noexcept;

    Parameter& zOffset() noexcept;
    const Parameter& zOffset() const noexcept;

    Parameter& superEllipseN() noexcept;
    const Parameter& superEllipseN() const noexcept;

    Parameter& cornerRadiusFrac() noexcept;
    const Parameter& cornerRadiusFrac() const noexcept;

    /// Generates a closed 2D loop in local (Y, Z) coordinates of size radialPoints + 1.
    std::vector<glm::vec2> sampleLoopYZ(int radialPoints) const;

private:
    CrossSectionShape m_shape{CrossSectionShape::Ellipse};
    Parameter m_stationFrac;
    Parameter m_width;
    Parameter m_height;
    Parameter m_yOffset;
    Parameter m_zOffset;
    Parameter m_superEllipseN;
    Parameter m_cornerRadiusFrac;
};

} // namespace opads
