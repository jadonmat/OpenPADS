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
    , m_spinnerLength("Spinner Len", "m", 0.42, 0.05, 5.0, 0.02)
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

Airfoil& Propeller::getBladeAirfoil() noexcept { return m_bladeAirfoil; }
const Airfoil& Propeller::getBladeAirfoil() const noexcept { return m_bladeAirfoil; }

double Propeller::getDiskArea() const noexcept {
    const double r = 0.5 * m_diameter.getValue();
    return std::numbers::pi * r * r;
}

double Propeller::getSolidity() const noexcept {
    const double r = 0.5 * m_diameter.getValue();
    const double avgChord = 0.5 * (m_rootChord.getValue() + m_tipChord.getValue());
    return (r > 1e-6) ? (static_cast<double>(m_bladeCount) * avgChord / (std::numbers::pi * r)) : 0.0;
}

void Propeller::rebuildMesh() {
    m_bladeAirfoil.setPointsPerSurface(25);

    // 1. Build spinner cone + hub cylinder along X (nose forward at -spinnerLen, aft hub at +0.25*spinnerLen)
    const float hubR = 0.5f * m_hubDiameter.getFloat();
    const float spinL = m_spinnerLength.getFloat();
    constexpr int kSpinStations = 10;
    constexpr int kSpinRadial = 24;
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
    // Short aft hub collar
    {
        std::vector<glm::vec3> aftRing;
        aftRing.reserve(kSpinRadial + 1);
        for (int p = 0; p <= kSpinRadial; ++p) {
            const float a = 2.0f * std::numbers::pi_v<float> * static_cast<float>(p) / static_cast<float>(kSpinRadial);
            aftRing.emplace_back(0.28f * spinL, hubR * std::cos(a), hubR * std::sin(a));
        }
        spinnerRings.push_back(std::move(aftRing));
    }
    MeshPrimitives::appendLoftedRings(m_mesh, spinnerRings, true, false, true);

    // 2. Build radial blade loft rings along +Z (then rotate for each blade around X axis)
    const float R = 0.5f * m_diameter.getFloat();
    const float rStart = std::min(hubR * 0.85f, R * 0.5f);
    const float cr = m_rootChord.getFloat();
    const float ct = m_tipChord.getFloat();
    const float pitch75 = m_collectivePitchDeg.getFloat();
    const float washout = m_washoutDeg.getFloat();
    const float tanSweep = std::tan(glm::radians(m_sweepDeg.getFloat()));
    const float dirSign = m_clockwise ? 1.0f : -1.0f;

    const auto& loop = m_bladeAirfoil.getClosedLoop();
    constexpr int kRadialBays = 10;

    std::vector<std::vector<glm::vec3>> baseBladeRings;
    baseBladeRings.reserve(kRadialBays + 1);

    for (int i = 0; i <= kRadialBays; ++i) {
        const float u = static_cast<float>(i) / static_cast<float>(kRadialBays);
        const float r = rStart + u * (R - rStart);
        // Slight elliptical tip rounding in chord for realistic blade shape
        const float tipTaper = (u > 0.85f) ? std::sqrt(std::max(0.05f, 1.0f - std::pow((u - 0.85f) / 0.15f, 2.0f))) : 1.0f;
        const float chord = (cr + u * (ct - cr)) * tipTaper;

        // Blade pitch relative to disk plane (90 deg = feathered into X, 0 deg = flat in Y-Z plane)
        const float rNorm = r / std::max(R, 1e-4f);
        const float betaDeg = pitch75 + (0.75f - rNorm) * washout;
        const float betaRad = glm::radians(betaDeg);
        const float cosB = std::cos(betaRad);
        const float sinB = std::sin(betaRad);

        const float ySweep = dirSign * (r - rStart) * tanSweep;

        std::vector<glm::vec3> ring;
        ring.reserve(loop.size());
        for (const glm::vec2& pt : loop) {
            // Local airfoil chord centered at 0.30*c
            const float cLocal = (pt.x - 0.30f) * chord;
            const float tLocal = pt.y * chord;

            // Rotate chord into rotor plane (Y tangential, X axial thrust)
            const float yLocal = dirSign * (cLocal * cosB - tLocal * sinB) + ySweep;
            const float xLocal = cLocal * sinB + tLocal * cosB;
            ring.emplace_back(xLocal, yLocal, r);
        }
        baseBladeRings.push_back(std::move(ring));
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
        MeshPrimitives::appendLoftedRings(m_mesh, rotatedRings, false, true, true);
    }
}

} // namespace opads
