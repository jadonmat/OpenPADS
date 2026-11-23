#include "core/Propeller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/trigonometric.hpp>

#include "core/MeshPrimitives.hpp"

namespace opads {

Propeller::Propeller(std::uint64_t id, std::string name)
    : Component(id, std::move(name))
    , m_diameter("Diameter", "m", 2.1, 0.2, 30.0, 0.05)
    , m_hubDiameter("Hub Diam", "m", 0.28, 0.02, 5.0, 0.01)
    , m_spinnerLength("Spinner Len", "m", 0.42, 0.0, 5.0, 0.02)
    , m_rootChord("Root Chord", "m", 0.19, 0.01, 3.0, 0.01)
    , m_tipChord("Tip Chord", "m", 0.09, 0.01, 3.0, 0.01)
    , m_collectivePitchDeg("Pitch @ 75%R", "deg", 24.0, -30.0, 85.0, 0.5)
    , m_washoutDeg("Twist Washout", "deg", 22.0, -30.0, 50.0, 0.5)
    , m_sweepDeg("Blade Sweep", "deg", 4.0, -45.0, 45.0, 0.5)
    , m_bladeAirfoil(Airfoil::createNaca4("4412", 25)) {
    ensureBuilt();
}

std::string Propeller::getTypeName() const {
    return "Propeller";
}

std::vector<std::vector<glm::vec3>> Propeller::getHighlightLoops() const {
    if (!m_showDiskRing) {
        return {};
    }
    const float r = 0.5f * m_diameter.getFloat();
    constexpr int kSegs = 64;
    std::vector<glm::vec3> circle;
    circle.reserve(kSegs + 1);
    for (int i = 0; i <= kSegs; ++i) {
        const float a = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(kSegs);
        circle.emplace_back(0.0f, r * std::cos(a), r * std::sin(a));
    }
    return {circle};
}

std::vector<glm::vec3> Propeller::getSnapSamplePointsLocal() const {
    return {
        {0.0f, 0.0f, 0.0f},
        {-m_spinnerLength.getFloat(), 0.0f, 0.0f},
        {0.5f * m_spinnerLength.getFloat(), 0.0f, 0.0f}
    };
}

Parameter& Propeller::diameter() noexcept { return m_diameter; }
const Parameter& Propeller::diameter() const noexcept { return m_diameter; }

Parameter& Propeller::hubDiameter() noexcept { return m_hubDiameter; }
const Parameter& Propeller::hubDiameter() const noexcept { return m_hubDiameter; }

Parameter& Propeller::spinnerLength() noexcept { return m_spinnerLength; }
const Parameter& Propeller::spinnerLength() const noexcept { return m_spinnerLength; }

Parameter& Propeller::rootChord() noexcept { return m_rootChord; }
const Parameter& Propeller::rootChord() const noexcept { return m_rootChord; }

Parameter& Propeller::tipChord() noexcept { return m_tipChord; }
const Parameter& Propeller::tipChord() const noexcept { return m_tipChord; }

Parameter& Propeller::collectivePitchDeg() noexcept { return m_collectivePitchDeg; }
const Parameter& Propeller::collectivePitchDeg() const noexcept { return m_collectivePitchDeg; }

Parameter& Propeller::washoutDeg() noexcept { return m_washoutDeg; }
const Parameter& Propeller::washoutDeg() const noexcept { return m_washoutDeg; }

Parameter& Propeller::sweepDeg() noexcept { return m_sweepDeg; }
const Parameter& Propeller::sweepDeg() const noexcept { return m_sweepDeg; }

int Propeller::getBladeCount() const noexcept { return m_bladeCount; }
void Propeller::setBladeCount(int count) {
    const int clamped = std::clamp(count, 2, 8);
    if (m_bladeCount != clamped) {
        m_bladeCount = clamped;
        markDirty();
    }
}

bool Propeller::isClockwise() const noexcept { return m_clockwise; }
void Propeller::setClockwise(bool cw) {
    if (m_clockwise != cw) {
        m_clockwise = cw;
        markDirty();
    }
}

bool Propeller::getShowDiskRing() const noexcept { return m_showDiskRing; }
void Propeller::setShowDiskRing(bool show) noexcept { m_showDiskRing = show; }

bool Propeller::hasSpinner() const noexcept { return m_hasSpinner; }
void Propeller::setHasSpinner(bool enabled) {
    if (m_hasSpinner != enabled) {
        m_hasSpinner = enabled;
        markDirty();
    }
}

EndCapType Propeller::getTipCapType() const noexcept { return m_tipCapType; }
void Propeller::setTipCapType(EndCapType type) {
    if (m_tipCapType != type) {
        m_tipCapType = type;
        markDirty();
    }
}

const std::vector<Propeller::BladeStation>& Propeller::getBladeCurve() const noexcept {
    return m_bladeCurve;
}

void Propeller::setBladeCurve(std::vector<BladeStation> stations) {
    std::sort(stations.begin(), stations.end(), [](const BladeStation& a, const BladeStation& b) {
        return a.radiusFrac < b.radiusFrac;
    });
    m_bladeCurve = std::move(stations);
    markDirty();
}

void Propeller::clearBladeCurve() {
    if (!m_bladeCurve.empty()) {
        m_bladeCurve.clear();
        markDirty();
    }
}

Airfoil& Propeller::getBladeAirfoil() noexcept { return m_bladeAirfoil; }
const Airfoil& Propeller::getBladeAirfoil() const noexcept { return m_bladeAirfoil; }

double Propeller::getDiskArea() const noexcept {
    const double r = 0.5 * m_diameter.getValue();
    return std::numbers::pi * r * r;
}

double Propeller::getSolidity() const noexcept {
    const double r = 0.5 * m_diameter.getValue();
    double avgChord = 0.5 * (m_rootChord.getValue() + m_tipChord.getValue());
    if (!m_bladeCurve.empty()) {
        double sum = 0.0;
        for (const BladeStation& st : m_bladeCurve) {
            sum += st.chord;
        }
        avgChord = sum / static_cast<double>(m_bladeCurve.size());
    }
    return (r > 1e-6) ? (static_cast<double>(m_bladeCount) * avgChord / (std::numbers::pi * r)) : 0.0;
}

void Propeller::rebuildMesh() {
    m_bladeAirfoil.setPointsPerSurface(25);

    // 1. Build radial blade loft rings along +Z first so hub length can envelope the blade root cleanly.
    const float hubR = 0.5f * m_hubDiameter.getFloat();
    const float spinL = m_spinnerLength.getFloat();
    const float R = 0.5f * m_diameter.getFloat();
    const float dirSign = m_clockwise ? 1.0f : -1.0f;
    const auto& loop = m_bladeAirfoil.getClosedLoop();
    const auto& upperSurf = m_bladeAirfoil.getUpperSurface();
    const auto& camberLine = m_bladeAirfoil.getCamberLine();
    const float foilMaxThick = std::max(m_bladeAirfoil.getMaxThicknessRatio(), 1e-4f);

    auto sampleBladeSectionY = [&](std::size_t loopIdx, float xFrac, float loopY, double targetThicknessRatio) noexcept -> float {
        if (targetThicknessRatio <= 0.0) {
            return loopY;
        }
        float yc = 0.0f;
        const std::size_t nUpper = upperSurf.size();
        if (nUpper > 1 && camberLine.size() == nUpper) {
            const std::size_t camberIdx = (loopIdx < nUpper)
                ? (nUpper - 1 - loopIdx)
                : std::min(nUpper - 1, loopIdx - (nUpper - 1));
            yc = camberLine[camberIdx].y;
        }
        const float yt = loopY - yc;
        const float thickScale = static_cast<float>(targetThicknessRatio) / foilMaxThick;
        const float scaledFoilY = yc + yt * thickScale;

        // Above ~20% t/c near the blade root, blend smoothly toward a symmetric structural ellipse cuff.
        const float cuffBlend = std::clamp(static_cast<float>((targetThicknessRatio - 0.20) / 0.25), 0.0f, 1.0f);
        if (cuffBlend <= 1e-4f) {
            return scaledFoilY;
        }
        const float signSide = (loopIdx < nUpper) ? 1.0f : -1.0f;
        const float ellipHalf = 0.5f * static_cast<float>(targetThicknessRatio) *
            std::sqrt(std::max(0.0f, 4.0f * xFrac * (1.0f - xFrac)));
        return (1.0f - cuffBlend) * scaledFoilY + cuffBlend * (signSide * ellipHalf);
    };

    std::vector<std::vector<glm::vec3>> baseBladeRings;
    if (m_bladeCurve.size() >= 2) {
        constexpr int kRadialBays = 24;
        baseBladeRings.reserve(kRadialBays + 1);
        const double rStartFrac = std::max(0.02, std::min(m_bladeCurve.front().radiusFrac, static_cast<double>(hubR * 0.88f / std::max(R, 1e-4f))));
        const double rMinFrac = std::clamp(rStartFrac, 0.02, 0.95);
        const double rMaxFrac = std::clamp(m_bladeCurve.back().radiusFrac, rMinFrac + 0.02, 1.0);

        auto evalCurve = [&](double rNorm) -> BladeStation {
            if (rNorm <= m_bladeCurve.front().radiusFrac) {
                return m_bladeCurve.front();
            }
            if (rNorm >= m_bladeCurve.back().radiusFrac) {
                return m_bladeCurve.back();
            }
            for (std::size_t idx = 0; idx + 1 < m_bladeCurve.size(); ++idx) {
                const BladeStation& a = m_bladeCurve[idx];
                const BladeStation& b = m_bladeCurve[idx + 1];
                if (rNorm <= b.radiusFrac) {
                    const double span = std::max(1e-6, b.radiusFrac - a.radiusFrac);
                    const double w = (rNorm - a.radiusFrac) / span;
                    BladeStation out;
                    out.radiusFrac = rNorm;
                    out.chord = a.chord + w * (b.chord - a.chord);
                    out.twistDeg = a.twistDeg + w * (b.twistDeg - a.twistDeg);
                    out.sweepOffset = a.sweepOffset + w * (b.sweepOffset - a.sweepOffset);
                    out.rakeOffset = a.rakeOffset + w * (b.rakeOffset - a.rakeOffset);
                    if (a.thicknessRatio > 0.0 && b.thicknessRatio > 0.0) {
                        out.thicknessRatio = a.thicknessRatio + w * (b.thicknessRatio - a.thicknessRatio);
                    } else {
                        out.thicknessRatio = std::max(a.thicknessRatio, b.thicknessRatio);
                    }
                    return out;
                }
            }
            return m_bladeCurve.back();
        };

        for (int i = 0; i <= kRadialBays; ++i) {
            const float u = static_cast<float>(i) / static_cast<float>(kRadialBays);
            const double rNorm = rMinFrac + static_cast<double>(u) * (rMaxFrac - rMinFrac);
            const BladeStation st = evalCurve(rNorm);
            const float r = static_cast<float>(rNorm) * R;
            const float tipTaper = (u > 0.96f) ? std::sqrt(std::max(0.15f, 1.0f - std::pow((u - 0.96f) / 0.04f, 2.0f))) : 1.0f;
            const float chord = static_cast<float>(std::max(0.005, st.chord)) * tipTaper;

            const float betaRad = glm::radians(static_cast<float>(st.twistDeg));
            const float cosB = std::cos(betaRad);
            const float sinB = std::sin(betaRad);
            const float ySweep = dirSign * static_cast<float>(st.sweepOffset);
            const float xRake = static_cast<float>(st.rakeOffset);

            std::vector<glm::vec3> ring;
            ring.reserve(loop.size());
            for (std::size_t idx = 0; idx < loop.size(); ++idx) {
                const glm::vec2& pt = loop[idx];
                const float ySec = sampleBladeSectionY(idx, pt.x, pt.y, st.thicknessRatio);
                const float cLocal = (pt.x - 0.50f) * chord;
                const float tLocal = ySec * chord;
                const float yLocal = dirSign * (cLocal * cosB - tLocal * sinB) + ySweep;
                const float xLocal = cLocal * sinB + tLocal * cosB + xRake;
                ring.emplace_back(xLocal, yLocal, r);
            }
            if (dirSign > 0.0f) {
                std::reverse(ring.begin(), ring.end());
            }
            baseBladeRings.push_back(std::move(ring));
        }
    } else {
        const float rStart = std::min(hubR * 0.85f, R * 0.5f);
        const float cr = m_rootChord.getFloat();
        const float ct = m_tipChord.getFloat();
        const float pitch75 = m_collectivePitchDeg.getFloat();
        const float washout = m_washoutDeg.getFloat();
        const float tanSweep = std::tan(glm::radians(m_sweepDeg.getFloat()));
        constexpr int kRadialBays = 16;

        baseBladeRings.reserve(kRadialBays + 1);
        for (int i = 0; i <= kRadialBays; ++i) {
            const float u = static_cast<float>(i) / static_cast<float>(kRadialBays);
            const float r = rStart + u * (R - rStart);
            const float tipTaper = (u > 0.90f) ? std::sqrt(std::max(0.12f, 1.0f - std::pow((u - 0.90f) / 0.10f, 2.0f))) : 1.0f;
            const float chord = (cr + u * (ct - cr)) * tipTaper;

            const float rNorm = r / std::max(R, 1e-4f);
            const float betaDeg = pitch75 + (0.75f - rNorm) * washout;
            const float betaRad = glm::radians(betaDeg);
            const float cosB = std::cos(betaRad);
            const float sinB = std::sin(betaRad);
            const float ySweep = dirSign * (r - rStart) * tanSweep;

            // Blend root shank thickness smoothly from 34% cuff down to the blade airfoil thickness.
            const double targetTc = (u < 0.25f)
                ? (static_cast<double>(foilMaxThick) + static_cast<double>(1.0f - u / 0.25f) * std::max(0.0, 0.34 - static_cast<double>(foilMaxThick)))
                : -1.0;

            std::vector<glm::vec3> ring;
            ring.reserve(loop.size());
            for (std::size_t idx = 0; idx < loop.size(); ++idx) {
                const glm::vec2& pt = loop[idx];
                const float ySec = sampleBladeSectionY(idx, pt.x, pt.y, targetTc);
                const float cLocal = (pt.x - 0.35f) * chord;
                const float tLocal = ySec * chord;
                const float yLocal = dirSign * (cLocal * cosB - tLocal * sinB) + ySweep;
                const float xLocal = cLocal * sinB + tLocal * cosB;
                ring.emplace_back(xLocal, yLocal, r);
            }
            if (dirSign > 0.0f) {
                std::reverse(ring.begin(), ring.end());
            }
            baseBladeRings.push_back(std::move(ring));
        }
    }

    // 2. Build spinner cone + hub drum along X sized to envelope the blade roots.
    float rootMinX = -0.15f * hubR;
    float rootMaxX = 0.15f * hubR;
    if (!baseBladeRings.empty()) {
        rootMinX = 1e9f;
        rootMaxX = -1e9f;
        const std::size_t rootCheckCount = std::min<std::size_t>(2, baseBladeRings.size());
        for (std::size_t rs = 0; rs < rootCheckCount; ++rs) {
            for (const glm::vec3& pt : baseBladeRings[rs]) {
                rootMinX = std::min(rootMinX, pt.x);
                rootMaxX = std::max(rootMaxX, pt.x);
            }
        }
        rootMinX = std::min(rootMinX, -0.15f * hubR);
        rootMaxX = std::max(rootMaxX, 0.15f * hubR);
    }

    constexpr int kSpinRadial = 24;
    if (m_hasSpinner && spinL > 1e-3f) {
        constexpr int kSpinStations = 10;
        std::vector<std::vector<glm::vec3>> spinnerRings;
        spinnerRings.reserve(kSpinStations + 2);

        for (int s = 0; s <= kSpinStations; ++s) {
            const float u = static_cast<float>(s) / static_cast<float>(kSpinStations);
            const float x = -spinL + u * spinL;
            const float r = hubR * std::pow(u, 0.58f);
            std::vector<glm::vec3> ring;
            ring.reserve(kSpinRadial + 1);
            for (int p = 0; p <= kSpinRadial; ++p) {
                const float a = 2.0f * std::numbers::pi_v<float> * static_cast<float>(p) / static_cast<float>(kSpinRadial);
                ring.emplace_back(x, r * std::cos(a), r * std::sin(a));
            }
            spinnerRings.push_back(std::move(ring));
        }
        {
            const float aftX = std::max(0.28f * spinL, rootMaxX + 0.06f * hubR);
            std::vector<glm::vec3> aftRing;
            aftRing.reserve(kSpinRadial + 1);
            for (int p = 0; p <= kSpinRadial; ++p) {
                const float a = 2.0f * std::numbers::pi_v<float> * static_cast<float>(p) / static_cast<float>(kSpinRadial);
                aftRing.emplace_back(aftX, hubR * std::cos(a), hubR * std::sin(a));
            }
            spinnerRings.push_back(std::move(aftRing));
        }
        MeshPrimitives::appendLoftedRings(m_mesh, spinnerRings, true, false, true);
    } else if (hubR > 1e-3f) {
        // Keep open hub compact so external spinners/nacelles enclose blade roots smoothly.
        const float barrelR = 0.80f * hubR;
        const float x0 = rootMinX - 0.04f * hubR;
        const float x1 = std::max(x0 + std::max(spinL, 0.16f * hubR), rootMaxX + 0.04f * hubR);
        MeshPrimitives::appendCylinder(m_mesh, glm::vec3(x0, 0.0f, 0.0f), glm::vec3(x1, 0.0f, 0.0f), barrelR, barrelR, kSpinRadial, true);
    }

    for (int b = 0; b < m_bladeCount; ++b) {
        const float azRad = 2.0f * std::numbers::pi_v<float> * static_cast<float>(b) / static_cast<float>(m_bladeCount);
        const float cosA = std::cos(azRad);
        const float sinA = std::sin(azRad);

        std::vector<std::vector<glm::vec3>> rotatedRings(baseBladeRings.size());
        for (std::size_t s = 0; s < baseBladeRings.size(); ++s) {
            rotatedRings[s].reserve(baseBladeRings[s].size());
            for (const glm::vec3& p : baseBladeRings[s]) {
                rotatedRings[s].emplace_back(
                    p.x,
                    p.y * cosA - p.z * sinA,
                    p.y * sinA + p.z * cosA
                );
            }
        }
        const bool capTip = m_tipCapType != EndCapType::None;
        MeshPrimitives::appendLoftedRings(m_mesh, rotatedRings, false, true, capTip);
    }
}

} // namespace opads
