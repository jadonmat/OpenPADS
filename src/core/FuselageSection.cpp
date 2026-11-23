#include "core/FuselageSection.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace opads {

namespace {

float signedPow(float value, float exponent) noexcept {
    if (std::abs(value) < 1e-7f) {
        return 0.0f;
    }
    const float mag = std::pow(std::abs(value), exponent);
    return (value >= 0.0f) ? mag : -mag;
}

} // namespace

FuselageSection::FuselageSection()
    : m_stationFrac("Station", "L", 0.5, 0.0, 1.0, 0.01)
    , m_width("Width", "m", 1.8, 0.0, 40.0, 0.05)
    , m_height("Height", "m", 1.9, 0.0, 40.0, 0.05)
    , m_yOffset("Y Offset", "m", 0.0, -20.0, 20.0, 0.02)
    , m_zOffset("Z Offset", "m", 0.0, -20.0, 20.0, 0.02)
    , m_superEllipseN("Power N", "", 2.5, 0.5, 12.0, 0.1)
    , m_cornerRadiusFrac("Corner Rad", "", 0.25, 0.01, 0.5, 0.02) {
}

FuselageSection::FuselageSection(
    CrossSectionShape shape,
    double stationFrac,
    double width,
    double height,
    double zOffset,
    double superEllipseN
)
    : FuselageSection() {
    m_shape = shape;
    m_stationFrac.setValue(stationFrac);
    m_width.setValue(width);
    m_height.setValue(height);
    m_zOffset.setValue(zOffset);
    m_superEllipseN.setValue(superEllipseN);
}

CrossSectionShape FuselageSection::getShape() const noexcept { return m_shape; }
void FuselageSection::setShape(CrossSectionShape shape) noexcept { m_shape = shape; }

Parameter& FuselageSection::stationFrac() noexcept { return m_stationFrac; }
const Parameter& FuselageSection::stationFrac() const noexcept { return m_stationFrac; }

Parameter& FuselageSection::width() noexcept { return m_width; }
const Parameter& FuselageSection::width() const noexcept { return m_width; }

Parameter& FuselageSection::height() noexcept { return m_height; }
const Parameter& FuselageSection::height() const noexcept { return m_height; }

Parameter& FuselageSection::yOffset() noexcept { return m_yOffset; }
const Parameter& FuselageSection::yOffset() const noexcept { return m_yOffset; }

Parameter& FuselageSection::zOffset() noexcept { return m_zOffset; }
const Parameter& FuselageSection::zOffset() const noexcept { return m_zOffset; }

Parameter& FuselageSection::superEllipseN() noexcept { return m_superEllipseN; }
const Parameter& FuselageSection::superEllipseN() const noexcept { return m_superEllipseN; }

Parameter& FuselageSection::cornerRadiusFrac() noexcept { return m_cornerRadiusFrac; }
const Parameter& FuselageSection::cornerRadiusFrac() const noexcept { return m_cornerRadiusFrac; }

std::vector<glm::vec2> FuselageSection::sampleLoopYZ(int radialPoints) const {
    const int count = std::clamp(radialPoints, 8, 128);
    std::vector<glm::vec2> loop;
    loop.reserve(static_cast<std::size_t>(count) + 1);

    const float yc = m_yOffset.getFloat();
    const float zc = m_zOffset.getFloat();

    if (m_shape == CrossSectionShape::Point) {
        for (int i = 0; i <= count; ++i) {
            loop.emplace_back(yc, zc);
        }
        return loop;
    }

    const float ry = 0.5f * m_width.getFloat();
    const float rz = (m_shape == CrossSectionShape::Circle) ? ry : (0.5f * m_height.getFloat());

    if (m_shape == CrossSectionShape::RoundedRect) {
        const float minHalf = std::min(ry, rz);
        const float r = std::min(minHalf, m_cornerRadiusFrac.getFloat() * std::min(m_width.getFloat(), m_height.getFloat()));
        const float iy = std::max(0.0f, ry - r);
        const float iz = std::max(0.0f, rz - r);

        for (int i = 0; i <= count; ++i) {
            const float angle = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(count);
            const float cosA = std::cos(angle);
            const float sinA = std::sin(angle);
            const float sx = (cosA >= 0.0f) ? 1.0f : -1.0f;
            const float sy = (sinA >= 0.0f) ? 1.0f : -1.0f;
            loop.emplace_back(yc + sx * iy + r * cosA, zc + sy * iz + r * sinA);
        }
        return loop;
    }

    const float exp = (m_shape == CrossSectionShape::SuperEllipse)
        ? (2.0f / m_superEllipseN.getFloat())
        : 1.0f;

    for (int i = 0; i <= count; ++i) {
        const float angle = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(count);
        const float y = yc + ry * signedPow(std::cos(angle), exp);
        const float z = zc + rz * signedPow(std::sin(angle), exp);
        loop.emplace_back(y, z);
    }

    return loop;
}

} // namespace opads
