#include "core/FuselageSection.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <glm/trigonometric.hpp>

namespace opads {

namespace {

float signedPow(float value, float exponent) noexcept {
    if (std::abs(value) < 1e-7f) {
        return 0.0f;
    }
    const float mag = std::pow(std::abs(value), exponent);
    return (value >= 0.0f) ? mag : -mag;
}

glm::vec2 evalCubicBezier2D(
    const glm::vec2& p0,
    const glm::vec2& p1,
    const glm::vec2& p2,
    const glm::vec2& p3,
    float u
) noexcept {
    const float om = 1.0f - u;
    return om * om * om * p0 +
        3.0f * om * om * u * p1 +
        3.0f * om * u * u * p2 +
        u * u * u * p3;
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

double FuselageSection::getBottomSuperEllipseN() const noexcept { return m_bottomSuperEllipseN; }
void FuselageSection::setBottomSuperEllipseN(double n) noexcept {
    m_bottomSuperEllipseN = (n > 0.0) ? std::clamp(n, 0.5, 12.0) : -1.0;
}

double FuselageSection::getMaxWidthLocFrac() const noexcept { return m_maxWidthLocFrac; }
void FuselageSection::setMaxWidthLocFrac(double frac) noexcept {
    m_maxWidthLocFrac = std::clamp(frac, -0.85, 0.85);
}

const FuselageSkinning& FuselageSection::getSkinning() const noexcept { return m_skinning; }
FuselageSkinning& FuselageSection::skinning() noexcept { return m_skinning; }
void FuselageSection::setSkinning(const FuselageSkinning& skin) noexcept { m_skinning = skin; }

const GeneralFuseParams& FuselageSection::getGeneralFuse() const noexcept { return m_generalFuse; }
GeneralFuseParams& FuselageSection::generalFuse() noexcept { return m_generalFuse; }
void FuselageSection::setGeneralFuse(const GeneralFuseParams& params) noexcept { m_generalFuse = params; }

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
        const float r = std::clamp(
            m_cornerRadiusFrac.getFloat() * std::min(m_width.getFloat(), m_height.getFloat()),
            0.0f,
            minHalf
        );
        const float iy = std::max(0.0f, ry - r);
        const float iz = std::max(0.0f, rz - r);
        constexpr float kHalfPi = 0.5f * std::numbers::pi_v<float>;
        const float cornerArc = kHalfPi * r;
        const float qLen = std::max(1e-6f, iy + iz + cornerArc);

        for (int i = 0; i < count; ++i) {
            const float uAll = 4.0f * static_cast<float>(i) / static_cast<float>(count);
            const int q = std::min(3, static_cast<int>(uAll));
            const float uq = uAll - static_cast<float>(q);
            const float s = uq * qLen;

            float py = 0.0f;
            float pz = 0.0f;
            if (q == 0) {
                if (s <= iz) {
                    py = ry;
                    pz = s;
                } else if (s <= iz + cornerArc && r > 1e-6f) {
                    const float phi = (s - iz) / r;
                    py = iy + r * std::cos(phi);
                    pz = iz + r * std::sin(phi);
                } else {
                    const float d = s - iz - cornerArc;
                    py = std::max(0.0f, iy - d);
                    pz = rz;
                }
            } else if (q == 1) {
                if (s <= iy) {
                    py = -s;
                    pz = rz;
                } else if (s <= iy + cornerArc && r > 1e-6f) {
                    const float phi = kHalfPi + (s - iy) / r;
                    py = -iy + r * std::cos(phi);
                    pz = iz + r * std::sin(phi);
                } else {
                    const float d = s - iy - cornerArc;
                    py = -ry;
                    pz = std::max(0.0f, iz - d);
                }
            } else if (q == 2) {
                if (s <= iz) {
                    py = -ry;
                    pz = -s;
                } else if (s <= iz + cornerArc && r > 1e-6f) {
                    const float phi = std::numbers::pi_v<float> + (s - iz) / r;
                    py = -iy + r * std::cos(phi);
                    pz = -iz + r * std::sin(phi);
                } else {
                    const float d = s - iz - cornerArc;
                    py = std::min(0.0f, -iy + d);
                    pz = -rz;
                }
            } else {
                if (s <= iy) {
                    py = s;
                    pz = -rz;
                } else if (s <= iy + cornerArc && r > 1e-6f) {
                    const float phi = 3.0f * kHalfPi + (s - iy) / r;
                    py = iy + r * std::cos(phi);
                    pz = -iz + r * std::sin(phi);
                } else {
                    const float d = s - iy - cornerArc;
                    py = ry;
                    pz = std::min(0.0f, -iz + d);
                }
            }
            loop.emplace_back(yc + py, zc + pz);
        }
        if (!loop.empty()) {
            loop.push_back(loop.front());
        }
        return loop;
    }

    if (m_shape == CrossSectionShape::GeneralFuse) {
        constexpr float kBezScale = 2.0f / 3.0f;
        const float zMid = static_cast<float>(m_maxWidthLocFrac) * rz;
        const float hTop = std::max(0.0f, rz - zMid);
        const float hBot = std::max(0.0f, rz + zMid);
        const float topAng = glm::radians(static_cast<float>(m_generalFuse.topTanAngleDeg));
        const float botAng = glm::radians(static_cast<float>(m_generalFuse.botTanAngleDeg));
        const float topStr = static_cast<float>(std::clamp(m_generalFuse.topStr, 0.05, 2.0));
        const float botStr = static_cast<float>(std::clamp(m_generalFuse.botStr, 0.05, 2.0));
        const float upStr = static_cast<float>(std::clamp(m_generalFuse.upStr, 0.05, 2.0));
        const float lowStr = static_cast<float>(std::clamp(m_generalFuse.lowStr, 0.05, 2.0));

        // 4 quadrant cubic Bezier curves ordered CCW starting at right equator (+ry, zMid)
        const glm::vec2 rEq(ry, zMid);
        const glm::vec2 tPt(0.0f, rz);
        const glm::vec2 lEq(-ry, zMid);
        const glm::vec2 bPt(0.0f, -rz);

        const glm::vec2 rEqUp(ry, zMid + kBezScale * upStr * hTop);
        const glm::vec2 tRight(kBezScale * topStr * ry * std::sin(topAng), rz - kBezScale * topStr * hTop * std::cos(topAng));
        const glm::vec2 tLeft(-tRight.x, tRight.y);
        const glm::vec2 lEqUp(-ry, rEqUp.y);

        const glm::vec2 lEqLow(-ry, zMid - kBezScale * lowStr * hBot);
        const glm::vec2 bLeft(-kBezScale * botStr * ry * std::sin(botAng), -rz + kBezScale * botStr * hBot * std::cos(botAng));
        const glm::vec2 bRight(-bLeft.x, bLeft.y);
        const glm::vec2 rEqLow(ry, lEqLow.y);

        for (int i = 0; i < count; ++i) {
            const float uAll = 4.0f * static_cast<float>(i) / static_cast<float>(count);
            const int q = std::min(3, static_cast<int>(uAll));
            const float u = uAll - static_cast<float>(q);

            glm::vec2 localPt(0.0f);
            if (q == 0) {
                localPt = evalCubicBezier2D(rEq, rEqUp, tRight, tPt, u);
            } else if (q == 1) {
                localPt = evalCubicBezier2D(tPt, tLeft, lEqUp, lEq, u);
            } else if (q == 2) {
                localPt = evalCubicBezier2D(lEq, lEqLow, bLeft, bPt, u);
            } else {
                localPt = evalCubicBezier2D(bPt, bRight, rEqLow, rEq, u);
            }
            loop.emplace_back(yc + localPt.x, zc + localPt.y);
        }
        if (!loop.empty()) {
            loop.push_back(loop.front());
        }
        return loop;
    }

    const float topN = m_superEllipseN.getFloat();
    const float botN = (m_bottomSuperEllipseN > 0.0) ? static_cast<float>(m_bottomSuperEllipseN) : topN;
    const float maxWidthShift = static_cast<float>(m_maxWidthLocFrac);

    for (int i = 0; i <= count; ++i) {
        const float angle = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(count);
        const float cosA = std::cos(angle);
        const float sinA = std::sin(angle);
        const float activeN = (sinA >= 0.0f) ? topN : botN;
        const float exp = (m_shape == CrossSectionShape::SuperEllipse)
            ? (2.0f / activeN)
            : 1.0f;

        const float y = yc + ry * signedPow(cosA, exp);
        const float z = zc + rz * (signedPow(sinA, exp) + maxWidthShift * (cosA * cosA));
        loop.emplace_back(y, z);
    }

    return loop;
}

} // namespace opads
