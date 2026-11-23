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
    RoundedRect,
    GeneralFuse
};

/// Exact OpenVSP GeneralFuse 4-quadrant cubic Bezier parameters.
struct GeneralFuseParams {
    double topStr{0.83};
    double botStr{0.83};
    double upStr{0.83};
    double lowStr{0.83};
    double topTanAngleDeg{90.0};
    double botTanAngleDeg{90.0};
};

/// Directional skinning tangency (angles in deg relative to +X) for Hermite lofting.
struct FuselageSkinning {
    bool enabled{false};
    double topLAngleDeg{0.0};
    double bottomLAngleDeg{0.0};
    double leftLAngleDeg{0.0};
    double rightLAngleDeg{0.0};
    double topRAngleDeg{0.0};
    double bottomRAngleDeg{0.0};
    double leftRAngleDeg{0.0};
    double rightRAngleDeg{0.0};
    double topLStrength{1.0};
    double bottomLStrength{1.0};
    double leftLStrength{1.0};
    double rightLStrength{1.0};
    double topRStrength{1.0};
    double bottomRStrength{1.0};
    double leftRStrength{1.0};
    double rightRStrength{1.0};
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

    /// Optional lower-half super-ellipse exponent (< 0 means match upper superEllipseN).
    double getBottomSuperEllipseN() const noexcept;
    void setBottomSuperEllipseN(double n) noexcept;

    /// Vertical shift of widest point as fraction of half-height [-0.85, 0.85].
    double getMaxWidthLocFrac() const noexcept;
    void setMaxWidthLocFrac(double frac) noexcept;

    /// Explicit longitudinal skinning tangent angles/strengths (for Hermite nose/tail lofting).
    const FuselageSkinning& getSkinning() const noexcept;
    FuselageSkinning& skinning() noexcept;
    void setSkinning(const FuselageSkinning& skin) noexcept;

    /// GeneralFuse 4-quadrant cubic Bezier shape parameters.
    const GeneralFuseParams& getGeneralFuse() const noexcept;
    GeneralFuseParams& generalFuse() noexcept;
    void setGeneralFuse(const GeneralFuseParams& params) noexcept;

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
    double m_bottomSuperEllipseN{-1.0};
    double m_maxWidthLocFrac{0.0};
    FuselageSkinning m_skinning{};
    GeneralFuseParams m_generalFuse{};
};

} // namespace opads
