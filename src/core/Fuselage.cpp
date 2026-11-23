#include "core/Fuselage.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include <glm/geometric.hpp>
#include <glm/trigonometric.hpp>

namespace opads {

namespace {

/// Fritsch-Carlson monotonic cubic Hermite (PCHIP) slopes so lofting never overshoots at sharp noses.
std::vector<float> computePchipSlopes(const std::vector<float>& xs, const std::vector<float>& ys) {
    const std::size_t n = xs.size();
    std::vector<float> slopes(n, 0.0f);
    if (n < 2) {
        return slopes;
    }

    std::vector<float> h(n - 1);
    std::vector<float> delta(n - 1);
    for (std::size_t i = 0; i + 1 < n; ++i) {
        h[i] = std::max(xs[i + 1] - xs[i], 1e-5f);
        delta[i] = (ys[i + 1] - ys[i]) / h[i];
    }

    if (n == 2) {
        slopes[0] = delta[0];
        slopes[1] = delta[0];
        return slopes;
    }

    for (std::size_t i = 1; i + 1 < n; ++i) {
        if (delta[i - 1] * delta[i] <= 0.0f) {
            slopes[i] = 0.0f;
        } else {
            const float w1 = 2.0f * h[i] + h[i - 1];
            const float w2 = h[i] + 2.0f * h[i - 1];
            slopes[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
        }
    }

    // Endpoints
    auto endpointSlope = [](float h0, float h1, float d0, float d1) {
        float d = ((2.0f * h0 + h1) * d0 - h0 * d1) / (h0 + h1);
        if (d * d0 <= 0.0f) {
            return 0.0f;
        }
        if ((d0 * d1 <= 0.0f) && (std::abs(d) > 3.0f * std::abs(d0))) {
            return 3.0f * d0;
        }
        return d;
    };

    slopes[0] = endpointSlope(h[0], h[1], delta[0], delta[1]);
    slopes[n - 1] = endpointSlope(h[n - 2], h[n - 3], delta[n - 2], delta[n - 3]);
    return slopes;
}

float evalHermite(float x0, float x1, float y0, float y1, float m0, float m1, float x) noexcept {
    const float h = std::max(x1 - x0, 1e-5f);
    const float t = std::clamp((x - x0) / h, 0.0f, 1.0f);
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
    const float h10 = t3 - 2.0f * t2 + t;
    const float h01 = -2.0f * t3 + 3.0f * t2;
    const float h11 = t3 - t2;
    return h00 * y0 + h10 * h * m0 + h01 * y1 + h11 * h * m1;
}

glm::vec3 evalHermiteVec3(
    const glm::vec3& p0,
    const glm::vec3& p1,
    const glm::vec3& t0,
    const glm::vec3& t1,
    float u
) noexcept {
    const float u2 = u * u;
    const float u3 = u2 * u;
    const float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
    const float h10 = u3 - 2.0f * u2 + u;
    const float h01 = -2.0f * u3 + 3.0f * u2;
    const float h11 = u3 - u2;
    return h00 * p0 + h10 * t0 + h01 * p1 + h11 * t1;
}

void sampleSkinningAtAngle(
    const FuselageSkinning& skin,
    bool isRightSide,
    float cosA,
    float sinA,
    double& outAngleDeg,
    double& outStrength
) noexcept {
    const float wY = cosA * cosA;
    const float wZ = sinA * sinA;
    if (isRightSide) {
        const double sideA = (cosA >= 0.0f) ? skin.rightRAngleDeg : skin.leftRAngleDeg;
        const double vertA = (sinA >= 0.0f) ? skin.topRAngleDeg : skin.bottomRAngleDeg;
        const double sideS = (cosA >= 0.0f) ? skin.rightRStrength : skin.leftRStrength;
        const double vertS = (sinA >= 0.0f) ? skin.topRStrength : skin.bottomRStrength;
        outAngleDeg = static_cast<double>(wY) * sideA + static_cast<double>(wZ) * vertA;
        outStrength = static_cast<double>(wY) * sideS + static_cast<double>(wZ) * vertS;
    } else {
        const double sideA = (cosA >= 0.0f) ? skin.rightLAngleDeg : skin.leftLAngleDeg;
        const double vertA = (sinA >= 0.0f) ? skin.topLAngleDeg : skin.bottomLAngleDeg;
        const double sideS = (cosA >= 0.0f) ? skin.rightLStrength : skin.leftLStrength;
        const double vertS = (sinA >= 0.0f) ? skin.topLStrength : skin.bottomLStrength;
        outAngleDeg = static_cast<double>(wY) * sideA + static_cast<double>(wZ) * vertA;
        outStrength = static_cast<double>(wY) * sideS + static_cast<double>(wZ) * vertS;
    }
}

glm::vec2 resolveRadialOutwardYZ(
    const FuselageSection& sec,
    const glm::vec2& ptYZ,
    const FuselageSection& otherSec,
    const glm::vec2& otherPtYZ,
    float cosA,
    float sinA
) noexcept {
    const bool secSlim = sec.getShape() == CrossSectionShape::Point ||
        sec.width().getFloat() < 1e-4f || sec.height().getFloat() < 1e-4f;
    const bool otherSlim = otherSec.getShape() == CrossSectionShape::Point ||
        otherSec.width().getFloat() < 1e-4f || otherSec.height().getFloat() < 1e-4f;

    glm::vec2 d(0.0f);
    if (!secSlim) {
        d = glm::vec2(ptYZ.x - sec.yOffset().getFloat(), ptYZ.y - sec.zOffset().getFloat());
    } else if (!otherSlim) {
        d = glm::vec2(otherPtYZ.x - otherSec.yOffset().getFloat(), otherPtYZ.y - otherSec.zOffset().getFloat());
    } else {
        d = glm::vec2(ptYZ.x - sec.yOffset().getFloat(), ptYZ.y - sec.zOffset().getFloat());
    }
    const float len = glm::length(d);
    if (len > 1e-5f) {
        return d / len;
    }
    return glm::vec2(cosA, sinA);
}

void computeMeshAreaAndVolume(const Mesh& mesh, double& areaOut, double& volumeOut) noexcept {
    const auto& verts = mesh.getVertices();
    const auto& indices = mesh.getIndices();
    double area = 0.0;
    double vol6 = 0.0;
    for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
        const glm::vec3& a = verts[indices[i]].position;
        const glm::vec3& b = verts[indices[i + 1]].position;
        const glm::vec3& c = verts[indices[i + 2]].position;
        area += 0.5 * static_cast<double>(glm::length(glm::cross(b - a, c - a)));
        vol6 += static_cast<double>(glm::dot(a, glm::cross(b, c)));
    }
    areaOut = area;
    volumeOut = std::abs(vol6) / 6.0;
}

} // namespace

Fuselage::Fuselage(std::uint64_t id, std::string name)
    : Component(id, std::move(name))
    , m_length("Length", "m", 12.0, 0.2, 150.0, 0.2) {
    m_sections.emplace_back(CrossSectionShape::Point, 0.00, 0.00, 0.00, -0.10, 2.0);
    m_sections.emplace_back(CrossSectionShape::Ellipse, 0.12, 1.25, 1.35, -0.04, 2.0);
    m_sections.emplace_back(CrossSectionShape::SuperEllipse, 0.34, 1.80, 1.95, 0.05, 2.6);
    m_sections.emplace_back(CrossSectionShape::Ellipse, 0.72, 1.55, 1.70, 0.10, 2.2);
    m_sections.emplace_back(CrossSectionShape::Point, 1.00, 0.00, 0.00, 0.22, 2.0);
    ensureBuilt();
}

std::string Fuselage::getTypeName() const {
    return "Fuselage";
}

std::vector<std::vector<glm::vec3>> Fuselage::getHighlightLoops() const {
    if (m_stationRings.empty()) {
        return {};
    }
    const std::size_t idx = std::min(m_selectedSection, m_stationRings.size() - 1);
    return {m_stationRings[idx]};
}

std::vector<glm::vec3> Fuselage::getSnapSamplePointsLocal() const {
    std::vector<glm::vec3> pts;
    for (std::size_t i = 0; i < m_sections.size(); ++i) {
        pts.push_back(getSectionCenterLocal(i));
    }
    for (const auto& ring : m_stationRings) {
        const std::size_t stride = std::max<std::size_t>(1, ring.size() / 8);
        for (std::size_t k = 0; k < ring.size(); k += stride) {
            pts.push_back(ring[k]);
        }
    }
    return pts;
}

Parameter& Fuselage::length() noexcept { return m_length; }
const Parameter& Fuselage::length() const noexcept { return m_length; }

bool Fuselage::isSmoothLoft() const noexcept { return m_smoothLoft; }
void Fuselage::setSmoothLoft(bool smooth) {
    if (m_smoothLoft != smooth) {
        m_smoothLoft = smooth;
        markDirty();
    }
}

bool Fuselage::getCapEnds() const noexcept {
    return m_noseCapType != EndCapType::None || m_tailCapType != EndCapType::None;
}

void Fuselage::setCapEnds(bool cap) {
    const EndCapType target = cap ? EndCapType::Flat : EndCapType::None;
    if (cap) {
        if (m_noseCapType == EndCapType::None && m_tailCapType == EndCapType::None) {
            m_noseCapType = EndCapType::Flat;
            m_tailCapType = EndCapType::Flat;
            markDirty();
        }
    } else if (m_noseCapType != EndCapType::None || m_tailCapType != EndCapType::None) {
        m_noseCapType = target;
        m_tailCapType = target;
        markDirty();
    }
}

EndCapType Fuselage::getNoseCapType() const noexcept { return m_noseCapType; }
void Fuselage::setNoseCapType(EndCapType type) {
    if (m_noseCapType != type) {
        m_noseCapType = type;
        markDirty();
    }
}

double Fuselage::getNoseCapLength() const noexcept { return m_noseCapLength; }
void Fuselage::setNoseCapLength(double lengthRatio) {
    const double c = std::clamp(lengthRatio, 0.05, 8.0);
    if (std::abs(m_noseCapLength - c) > 1e-6) {
        m_noseCapLength = c;
        markDirty();
    }
}

double Fuselage::getNoseCapStrength() const noexcept { return m_noseCapStrength; }
void Fuselage::setNoseCapStrength(double strength) {
    const double c = std::clamp(strength, 0.1, 2.0);
    if (std::abs(m_noseCapStrength - c) > 1e-6) {
        m_noseCapStrength = c;
        markDirty();
    }
}

EndCapType Fuselage::getTailCapType() const noexcept { return m_tailCapType; }
void Fuselage::setTailCapType(EndCapType type) {
    if (m_tailCapType != type) {
        m_tailCapType = type;
        markDirty();
    }
}

double Fuselage::getTailCapLength() const noexcept { return m_tailCapLength; }
void Fuselage::setTailCapLength(double lengthRatio) {
    const double c = std::clamp(lengthRatio, 0.05, 8.0);
    if (std::abs(m_tailCapLength - c) > 1e-6) {
        m_tailCapLength = c;
        markDirty();
    }
}

double Fuselage::getTailCapStrength() const noexcept { return m_tailCapStrength; }
void Fuselage::setTailCapStrength(double strength) {
    const double c = std::clamp(strength, 0.1, 2.0);
    if (std::abs(m_tailCapStrength - c) > 1e-6) {
        m_tailCapStrength = c;
        markDirty();
    }
}

int Fuselage::getRadialPoints() const noexcept { return m_radialPoints; }
void Fuselage::setRadialPoints(int count) {
    const int clamped = std::clamp(count, 8, 96);
    if (m_radialPoints != clamped) {
        m_radialPoints = clamped;
        markDirty();
    }
}

int Fuselage::getAxialSubdivisions() const noexcept { return m_axialSubdivisions; }
void Fuselage::setAxialSubdivisions(int count) {
    const int clamped = std::clamp(count, 1, 32);
    if (m_axialSubdivisions != clamped) {
        m_axialSubdivisions = clamped;
        markDirty();
    }
}

std::size_t Fuselage::getSectionCount() const noexcept { return m_sections.size(); }

FuselageSection& Fuselage::getSection(std::size_t index) {
    return m_sections.at(index);
}

const FuselageSection& Fuselage::getSection(std::size_t index) const {
    return m_sections.at(index);
}

const std::vector<FuselageSection>& Fuselage::getSections() const noexcept {
    return m_sections;
}

void Fuselage::addSectionAfter(std::size_t index) {
    if (m_sections.empty()) {
        m_sections.emplace_back(CrossSectionShape::Ellipse, 0.5, 1.5, 1.5, 0.0, 2.0);
        m_selectedSection = 0;
        markDirty();
        return;
    }

    const std::size_t i0 = std::min(index, m_sections.size() - 1);
    const std::size_t i1 = std::min(i0 + 1, m_sections.size() - 1);
    const FuselageSection& a = m_sections[i0];
    const FuselageSection& b = m_sections[i1];

    double newFrac = 0.5 * (a.stationFrac().getValue() + b.stationFrac().getValue());
    if (i0 == i1) {
        newFrac = std::min(1.0, a.stationFrac().getValue() + 0.1);
    }

    FuselageSection inserted(
        CrossSectionShape::Ellipse,
        newFrac,
        0.5 * (a.width().getValue() + b.width().getValue()),
        0.5 * (a.height().getValue() + b.height().getValue()),
        0.5 * (a.zOffset().getValue() + b.zOffset().getValue()),
        0.5 * (a.superEllipseN().getValue() + b.superEllipseN().getValue())
    );

    const std::size_t insertPos = (i0 == i1) ? m_sections.size() : (i0 + 1);
    m_sections.insert(m_sections.begin() + static_cast<std::ptrdiff_t>(insertPos), std::move(inserted));
    m_selectedSection = insertPos;
    markDirty();
}

bool Fuselage::removeSection(std::size_t index) {
    if (m_sections.size() <= 2 || index >= m_sections.size()) {
        return false;
    }
    m_sections.erase(m_sections.begin() + static_cast<std::ptrdiff_t>(index));
    m_selectedSection = std::min(m_selectedSection, m_sections.size() - 1);
    markDirty();
    return true;
}

void Fuselage::clearSections() {
    m_sections.clear();
    m_selectedSection = 0;
    markDirty();
}

void Fuselage::appendSection(FuselageSection section) {
    m_sections.push_back(std::move(section));
    markDirty();
}

void Fuselage::sortAndRebuild() {
    std::stable_sort(m_sections.begin(), m_sections.end(), [](const FuselageSection& a, const FuselageSection& b) {
        return a.stationFrac().getValue() < b.stationFrac().getValue();
    });
    markDirty();
}

std::size_t Fuselage::getSelectedSection() const noexcept {
    return m_selectedSection;
}

void Fuselage::setSelectedSection(std::size_t index) noexcept {
    if (m_sections.empty()) {
        m_selectedSection = 0;
    } else {
        m_selectedSection = std::min(index, m_sections.size() - 1);
    }
}

glm::vec3 Fuselage::getSectionCenterLocal(std::size_t index) const noexcept {
    if (m_sections.empty()) {
        return glm::vec3(0.0f);
    }
    const FuselageSection& sec = m_sections[std::min(index, m_sections.size() - 1)];
    return glm::vec3(
        sec.stationFrac().getFloat() * m_length.getFloat(),
        sec.yOffset().getFloat(),
        sec.zOffset().getFloat()
    );
}

void Fuselage::setSectionCenterLocal(std::size_t index, const glm::vec3& localCenter) {
    if (index >= m_sections.size()) {
        return;
    }
    FuselageSection& sec = m_sections[index];
    const float len = std::max(m_length.getFloat(), 0.1f);
    sec.stationFrac().setFloat(localCenter.x / len);
    sec.yOffset().setFloat(localCenter.y);
    sec.zOffset().setFloat(localCenter.z);
    markDirty();
}

void Fuselage::moveSectionFromHandle(std::size_t index, const glm::vec3& deltaLocal) {
    if (index >= m_sections.size()) {
        return;
    }
    FuselageSection& sec = m_sections[index];
    const float len = std::max(m_length.getFloat(), 0.1f);

    const float newFrac = sec.stationFrac().getFloat() + deltaLocal.x / len;
    sec.stationFrac().setFloat(newFrac);
    sec.yOffset().setFloat(sec.yOffset().getFloat() + deltaLocal.y);
    sec.zOffset().setFloat(sec.zOffset().getFloat() + deltaLocal.z);
    markDirty();
}

double Fuselage::getMaxWidth() const noexcept {
    double maxW = 0.0;
    for (const auto& s : m_sections) {
        if (s.getShape() != CrossSectionShape::Point) {
            maxW = std::max(maxW, s.width().getValue());
        }
    }
    return maxW;
}

double Fuselage::getMaxHeight() const noexcept {
    double maxH = 0.0;
    for (const auto& s : m_sections) {
        if (s.getShape() == CrossSectionShape::Circle) {
            maxH = std::max(maxH, s.width().getValue());
        } else if (s.getShape() != CrossSectionShape::Point) {
            maxH = std::max(maxH, s.height().getValue());
        }
    }
    return maxH;
}

double Fuselage::getFinenessRatio() const noexcept {
    const double dEq = std::sqrt(std::max(getMaxWidth() * getMaxHeight(), 0.0));
    return (dEq > 1e-6) ? (m_length.getValue() / dEq) : 0.0;
}

double Fuselage::getWettedArea() const noexcept { return m_wettedArea; }
double Fuselage::getVolume() const noexcept { return m_volume; }

void Fuselage::rebuildMesh() {
    m_stationRings.clear();
    m_wettedArea = 0.0;
    m_volume = 0.0;

    if (m_sections.size() < 2) {
        return;
    }

    std::vector<const FuselageSection*> ordered;
    ordered.reserve(m_sections.size());
    for (const auto& sec : m_sections) {
        ordered.push_back(&sec);
    }
    std::stable_sort(ordered.begin(), ordered.end(), [](const FuselageSection* a, const FuselageSection* b) {
        return a->stationFrac().getValue() < b->stationFrac().getValue();
    });

    const float totalLen = m_length.getFloat();
    const std::size_t stationCount = ordered.size();
    const std::size_t ptCount = static_cast<std::size_t>(m_radialPoints) + 1;

    std::vector<float> stationX(stationCount);
    std::vector<std::vector<glm::vec2>> stationLoops(stationCount);
    for (std::size_t s = 0; s < stationCount; ++s) {
        stationX[s] = ordered[s]->stationFrac().getFloat() * totalLen;
        stationLoops[s] = ordered[s]->sampleLoopYZ(m_radialPoints);
    }

    // Store un-subdivided station rings in original section order for highlight overlay.
    m_stationRings.resize(m_sections.size());
    for (std::size_t i = 0; i < m_sections.size(); ++i) {
        const float sx = m_sections[i].stationFrac().getFloat() * totalLen;
        const auto loopYZ = m_sections[i].sampleLoopYZ(m_radialPoints);
        m_stationRings[i].reserve(loopYZ.size());
        for (const glm::vec2& yz : loopYZ) {
            m_stationRings[i].emplace_back(sx, yz.x, yz.y);
        }
    }

    // Compute PCHIP slopes per radial point for smooth longitudinal lofting.
    std::vector<std::vector<float>> slopesY(ptCount);
    std::vector<std::vector<float>> slopesZ(ptCount);
    if (m_smoothLoft) {
        std::vector<float> colY(stationCount);
        std::vector<float> colZ(stationCount);
        for (std::size_t p = 0; p < ptCount; ++p) {
            for (std::size_t s = 0; s < stationCount; ++s) {
                colY[s] = stationLoops[s][p].x;
                colZ[s] = stationLoops[s][p].y;
            }
            slopesY[p] = computePchipSlopes(stationX, colY);
            slopesZ[p] = computePchipSlopes(stationX, colZ);
        }
    }

    std::vector<std::vector<glm::vec3>> loftRings;
    loftRings.reserve((stationCount - 1) * static_cast<std::size_t>(m_axialSubdivisions) + 1);

    for (std::size_t s = 0; s + 1 < stationCount; ++s) {
        const float x0 = stationX[s];
        const float x1 = stationX[s + 1];
        const int sub = (std::abs(x1 - x0) < 1e-5f) ? 1 : m_axialSubdivisions;
        const int startStep = (s == 0) ? 0 : 1;

        const FuselageSection& sec0 = *ordered[s];
        const FuselageSection& sec1 = *ordered[s + 1];
        const bool useSkinnedHermite = m_smoothLoft && (sec0.getSkinning().enabled || sec1.getSkinning().enabled);
        const float dxBay = std::max(1e-5f, x1 - x0);

        for (int step = startStep; step <= sub; ++step) {
            const float t = static_cast<float>(step) / static_cast<float>(sub);
            const float x = x0 + t * (x1 - x0);
            std::vector<glm::vec3> ring(ptCount);

            for (std::size_t p = 0; p < ptCount; ++p) {
                if (useSkinnedHermite) {
                    const std::size_t pWrap = (p + 1 == ptCount) ? 0 : p;
                    const float angle = 2.0f * std::numbers::pi_v<float> * static_cast<float>(pWrap) / static_cast<float>(m_radialPoints);
                    const float cosA = std::cos(angle);
                    const float sinA = std::sin(angle);

                    const glm::vec3 p0(x0, stationLoops[s][pWrap].x, stationLoops[s][pWrap].y);
                    const glm::vec3 p1(x1, stationLoops[s + 1][pWrap].x, stationLoops[s + 1][pWrap].y);
                    const float chordLen = std::max(1e-5f, glm::length(p1 - p0));
                    const glm::vec2 uRad = resolveRadialOutwardYZ(
                        sec0,
                        stationLoops[s][pWrap],
                        sec1,
                        stationLoops[s + 1][pWrap],
                        cosA,
                        sinA
                    );

                    auto buildTangent = [&](const FuselageSection& sec, bool rightSide, float pchipSlopeY, float pchipSlopeZ) {
                        const FuselageSkinning& skin = sec.getSkinning();
                        if (!skin.enabled) {
                            return glm::vec3(dxBay, pchipSlopeY * dxBay, pchipSlopeZ * dxBay);
                        }
                        double angDeg = 0.0;
                        double strength = 1.0;
                        sampleSkinningAtAngle(skin, rightSide, cosA, sinA, angDeg, strength);
                        const float rad = glm::radians(static_cast<float>(angDeg));
                        const float len = static_cast<float>(std::clamp(strength, 0.05, 2.5)) * chordLen;
                        glm::vec3 tang(
                            len * std::max(0.0f, std::cos(rad)),
                            len * std::sin(rad) * uRad.x,
                            len * std::sin(rad) * uRad.y
                        );
                        if (tang.x > 2.4f * dxBay) {
                            tang *= (2.4f * dxBay) / tang.x;
                        }
                        return tang;
                    };

                    const glm::vec3 t0 = buildTangent(sec0, true, slopesY[pWrap][s], slopesZ[pWrap][s]);
                    const glm::vec3 t1 = buildTangent(sec1, false, slopesY[pWrap][s + 1], slopesZ[pWrap][s + 1]);
                    ring[p] = evalHermiteVec3(p0, p1, t0, t1, t);
                } else if (m_smoothLoft) {
                    const float y = evalHermite(x0, x1, stationLoops[s][p].x, stationLoops[s + 1][p].x, slopesY[p][s], slopesY[p][s + 1], x);
                    const float z = evalHermite(x0, x1, stationLoops[s][p].y, stationLoops[s + 1][p].y, slopesZ[p][s], slopesZ[p][s + 1], x);
                    ring[p] = glm::vec3(x, y, z);
                } else {
                    const float y = stationLoops[s][p].x + t * (stationLoops[s + 1][p].x - stationLoops[s][p].x);
                    const float z = stationLoops[s][p].y + t * (stationLoops[s + 1][p].y - stationLoops[s][p].y);
                    ring[p] = glm::vec3(x, y, z);
                }
            }
            loftRings.push_back(std::move(ring));
        }
    }

    const std::size_t ringCount = loftRings.size();
    std::vector<glm::vec3> normals(ringCount * ptCount, glm::vec3(0.0f));
    const auto idxAt = [ptCount](std::size_t r, std::size_t p) {
        return r * ptCount + p;
    };

    for (std::size_t r = 0; r + 1 < ringCount; ++r) {
        for (std::size_t p = 0; p + 1 < ptCount; ++p) {
            const glm::vec3& v00 = loftRings[r][p];
            const glm::vec3& v10 = loftRings[r + 1][p];
            const glm::vec3& v11 = loftRings[r + 1][p + 1];
            const glm::vec3& v01 = loftRings[r][p + 1];

            // Outward winding around +X (cos(theta)*Y + sin(theta)*Z): (v01 - v00) x (v10 - v00)
            const glm::vec3 n1 = glm::cross(v01 - v00, v10 - v00);
            const glm::vec3 n2 = glm::cross(v01 - v10, v11 - v10);

            normals[idxAt(r, p)] += n1 + n2;
            normals[idxAt(r + 1, p)] += n1 + n2;
            normals[idxAt(r + 1, p + 1)] += n1 + n2;
            normals[idxAt(r, p + 1)] += n1 + n2;
        }
    }

    // Average seam normals at theta=0 and theta=2*pi for seamless shading.
    for (std::size_t r = 0; r < ringCount; ++r) {
        const glm::vec3 seamSum = normals[idxAt(r, 0)] + normals[idxAt(r, ptCount - 1)];
        normals[idxAt(r, 0)] = seamSum;
        normals[idxAt(r, ptCount - 1)] = seamSum;
    }

    const auto base = static_cast<std::uint32_t>(m_mesh.getVertexCount());
    for (std::size_t r = 0; r < ringCount; ++r) {
        for (std::size_t p = 0; p < ptCount; ++p) {
            glm::vec3 n = normals[idxAt(r, p)];
            const float len = glm::length(n);
            n = (len > 1e-7f) ? (n / len) : glm::vec3(0.0f, 1.0f, 0.0f);
            m_mesh.addVertex(loftRings[r][p], n);
        }
    }

    for (std::size_t r = 0; r + 1 < ringCount; ++r) {
        for (std::size_t p = 0; p + 1 < ptCount; ++p) {
            const auto i00 = base + static_cast<std::uint32_t>(idxAt(r, p));
            const auto i10 = base + static_cast<std::uint32_t>(idxAt(r + 1, p));
            const auto i11 = base + static_cast<std::uint32_t>(idxAt(r + 1, p + 1));
            const auto i01 = base + static_cast<std::uint32_t>(idxAt(r, p + 1));

            m_mesh.addTriangle(i00, i01, i10);
            m_mesh.addTriangle(i10, i01, i11);
        }
    }

    if (ordered.front()->getShape() != CrossSectionShape::Point && m_noseCapType != EndCapType::None) {
        MeshPrimitives::appendEndCap(
            m_mesh,
            loftRings.front(),
            glm::vec3(-1.0f, 0.0f, 0.0f),
            m_noseCapType,
            m_noseCapLength,
            m_noseCapStrength,
            true,
            true
        );
    }
    if (ordered.back()->getShape() != CrossSectionShape::Point && m_tailCapType != EndCapType::None) {
        MeshPrimitives::appendEndCap(
            m_mesh,
            loftRings.back(),
            glm::vec3(1.0f, 0.0f, 0.0f),
            m_tailCapType,
            m_tailCapLength,
            m_tailCapStrength,
            true,
            false
        );
    }

    computeMeshAreaAndVolume(m_mesh, m_wettedArea, m_volume);
}

} // namespace opads
