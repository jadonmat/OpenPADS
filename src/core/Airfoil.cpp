#include "core/Airfoil.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numbers>
#include <sstream>
#include <utility>

namespace opads {

namespace {

std::string trimCopy(std::string_view sv) {
    std::size_t start = 0;
    while (start < sv.size() && std::isspace(static_cast<unsigned char>(sv[start])) != 0) {
        ++start;
    }
    std::size_t end = sv.size();
    while (end > start && std::isspace(static_cast<unsigned char>(sv[end - 1])) != 0) {
        --end;
    }
    return std::string(sv.substr(start, end - start));
}

bool tryParsePair(std::string_view line, float& xOut, float& yOut) {
    std::istringstream iss{std::string(line)};
    double x = 0.0;
    double y = 0.0;
    if (!(iss >> x >> y)) {
        return false;
    }
    std::string trailing;
    if (iss >> trailing) {
        return false;
    }
    xOut = static_cast<float>(x);
    yOut = static_cast<float>(y);
    return std::isfinite(xOut) && std::isfinite(yOut);
}

std::vector<float> makeCosineGrid(int count) {
    std::vector<float> xs(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double beta = std::numbers::pi * static_cast<double>(i) / static_cast<double>(count - 1);
        xs[static_cast<std::size_t>(i)] = static_cast<float>(0.5 * (1.0 - std::cos(beta)));
    }
    return xs;
}

float nacaThicknessHalfWidth(float x, float maxThickness, bool sharpTE) {
    const float clampedX = std::clamp(x, 0.0f, 1.0f);
    const float c4 = sharpTE ? 0.1036f : 0.1015f;
    const float sx = std::sqrt(clampedX);
    const float x2 = clampedX * clampedX;
    const float x3 = x2 * clampedX;
    const float x4 = x3 * clampedX;
    return 5.0f * maxThickness * (0.2969f * sx - 0.1260f * clampedX - 0.3516f * x2 + 0.2843f * x3 - c4 * x4);
}

/// Interpolates y(x) using sqrt(x) parameter space so sqrt-shaped leading edges stay smooth.
float sampleSurfaceSqrtSpace(const std::vector<glm::vec2>& surface, float xTarget) {
    if (surface.empty()) {
        return 0.0f;
    }
    if (surface.size() == 1) {
        return surface.front().y;
    }

    const float uTarget = std::sqrt(std::max(xTarget, 0.0f));
    for (std::size_t i = 0; i + 1 < surface.size(); ++i) {
        const float x0 = surface[i].x;
        const float x1 = surface[i + 1].x;
        if (xTarget <= x1 || i + 2 == surface.size()) {
            const float u0 = std::sqrt(std::max(x0, 0.0f));
            const float u1 = std::sqrt(std::max(x1, 0.0f));
            const float denom = u1 - u0;
            if (denom <= 1e-7f) {
                return surface[i].y;
            }
            const float t = std::clamp((uTarget - u0) / denom, 0.0f, 1.0f);
            return surface[i].y + t * (surface[i + 1].y - surface[i].y);
        }
    }
    return surface.back().y;
}

struct Naca5Constants {
    float m;
    float k1;
    float k2OverK1;
};

Naca5Constants lookupNaca5Constants(int pDigit, bool reflexed) {
    if (!reflexed) {
        switch (pDigit) {
        case 1: return {0.0580f, 361.400f, 0.0f};
        case 2: return {0.1260f, 51.640f, 0.0f};
        case 3: return {0.2025f, 15.957f, 0.0f};
        case 4: return {0.2900f, 6.643f, 0.0f};
        case 5: return {0.3910f, 3.230f, 0.0f};
        default: return {0.2025f, 15.957f, 0.0f};
        }
    }

    switch (pDigit) {
    case 2: return {0.1300f, 51.990f, 0.000764f};
    case 3: return {0.2170f, 15.793f, 0.00677f};
    case 4: return {0.3180f, 6.520f, 0.0303f};
    case 5: return {0.4410f, 3.191f, 0.1355f};
    default: return {0.2170f, 15.793f, 0.00677f};
    }
}

std::vector<glm::vec2> normalizeCurveLEToTE(std::vector<glm::vec2> pts) {
    if (pts.empty()) {
        return pts;
    }
    std::sort(pts.begin(), pts.end(), [](const glm::vec2& a, const glm::vec2& b) {
        return a.x < b.x;
    });
    const float minX = pts.front().x;
    const float maxX = pts.back().x;
    const float span = maxX - minX;
    if (span > 1e-6f) {
        for (auto& pt : pts) {
            pt.x = (pt.x - minX) / span;
            pt.y /= span;
        }
    }
    return pts;
}

double binomialCoeff(int n, int k) noexcept {
    if (k < 0 || k > n) {
        return 0.0;
    }
    double res = 1.0;
    for (int i = 1; i <= k; ++i) {
        res *= static_cast<double>(n - k + i) / static_cast<double>(i);
    }
    return res;
}

double cstBasis(int n, int i, double x) noexcept {
    const double cx = std::sqrt(std::max(x, 0.0)) * std::max(1.0 - x, 0.0);
    const double bernstein = binomialCoeff(n, i) * std::pow(x, i) * std::pow(1.0 - x, n - i);
    return cx * bernstein;
}

std::vector<double> fitCstWeights(const std::vector<glm::vec2>& surface, int order) {
    const int numWeights = order + 1;
    std::vector<std::vector<double>> ata(
        static_cast<std::size_t>(numWeights),
        std::vector<double>(static_cast<std::size_t>(numWeights), 0.0)
    );
    std::vector<double> atb(static_cast<std::size_t>(numWeights), 0.0);

    for (const glm::vec2& pt : surface) {
        const double x = static_cast<double>(pt.x);
        if (x <= 1e-4 || x >= 1.0 - 1e-4) {
            continue;
        }
        std::vector<double> row(static_cast<std::size_t>(numWeights));
        for (int i = 0; i < numWeights; ++i) {
            row[static_cast<std::size_t>(i)] = cstBasis(order, i, x);
        }
        const double y = static_cast<double>(pt.y);
        for (int r = 0; r < numWeights; ++r) {
            atb[static_cast<std::size_t>(r)] += row[static_cast<std::size_t>(r)] * y;
            for (int c = 0; c < numWeights; ++c) {
                ata[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] +=
                    row[static_cast<std::size_t>(r)] * row[static_cast<std::size_t>(c)];
            }
        }
    }

    for (int i = 0; i < numWeights; ++i) {
        ata[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] += 1e-8;
    }

    // Partial-pivoting Gaussian elimination
    for (int col = 0; col < numWeights; ++col) {
        int pivot = col;
        double maxAbs = std::abs(ata[static_cast<std::size_t>(col)][static_cast<std::size_t>(col)]);
        for (int r = col + 1; r < numWeights; ++r) {
            const double val = std::abs(ata[static_cast<std::size_t>(r)][static_cast<std::size_t>(col)]);
            if (val > maxAbs) {
                maxAbs = val;
                pivot = r;
            }
        }
        if (pivot != col) {
            std::swap(ata[static_cast<std::size_t>(col)], ata[static_cast<std::size_t>(pivot)]);
            std::swap(atb[static_cast<std::size_t>(col)], atb[static_cast<std::size_t>(pivot)]);
        }

        const double diag = ata[static_cast<std::size_t>(col)][static_cast<std::size_t>(col)];
        if (std::abs(diag) < 1e-14) {
            continue;
        }
        for (int r = col + 1; r < numWeights; ++r) {
            const double factor = ata[static_cast<std::size_t>(r)][static_cast<std::size_t>(col)] / diag;
            for (int c = col; c < numWeights; ++c) {
                ata[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] -=
                    factor * ata[static_cast<std::size_t>(col)][static_cast<std::size_t>(c)];
            }
            atb[static_cast<std::size_t>(r)] -= factor * atb[static_cast<std::size_t>(col)];
        }
    }

    std::vector<double> weights(static_cast<std::size_t>(numWeights), 0.0);
    for (int r = numWeights - 1; r >= 0; --r) {
        double sum = atb[static_cast<std::size_t>(r)];
        for (int c = r + 1; c < numWeights; ++c) {
            sum -= ata[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] * weights[static_cast<std::size_t>(c)];
        }
        const double diag = ata[static_cast<std::size_t>(r)][static_cast<std::size_t>(r)];
        weights[static_cast<std::size_t>(r)] = (std::abs(diag) > 1e-14) ? (sum / diag) : 0.0;
    }
    return weights;
}

} // namespace

Airfoil::Airfoil() {
    rebuild();
}

Airfoil Airfoil::createNaca4(std::string_view code, int pointsPerSurface) {
    Airfoil foil;
    foil.m_pointsPerSurface = std::clamp(pointsPerSurface, 9, 201);
    foil.setNaca4(code);
    return foil;
}

Airfoil Airfoil::createNaca5(std::string_view code, int pointsPerSurface) {
    Airfoil foil;
    foil.m_pointsPerSurface = std::clamp(pointsPerSurface, 9, 201);
    foil.setNaca5(code);
    return foil;
}

void Airfoil::setNaca4(std::string_view code) {
    std::string digits;
    for (char ch : code) {
        if (std::isdigit(static_cast<unsigned char>(ch)) != 0) {
            digits.push_back(ch);
        }
    }
    if (digits.size() != 4) {
        digits = "0012";
    }
    m_type = AirfoilType::Naca4;
    m_naca4Code = digits;
    m_name = "NACA " + m_naca4Code;
    m_datPath.clear();
    rebuild();
}

void Airfoil::setNaca5(std::string_view code) {
    std::string digits;
    for (char ch : code) {
        if (std::isdigit(static_cast<unsigned char>(ch)) != 0) {
            digits.push_back(ch);
        }
    }
    if (digits.size() != 5) {
        digits = "23012";
    }
    m_type = AirfoilType::Naca5;
    m_naca5Code = digits;
    m_name = "NACA " + m_naca5Code;
    m_datPath.clear();
    rebuild();
}

bool Airfoil::loadFromDatFile(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    if (!loadFromDatString(buffer.str(), path.stem().string())) {
        return false;
    }
    m_datPath = path.string();
    return true;
}

bool Airfoil::loadFromDatString(std::string_view contents, std::string fallbackName) {
    std::istringstream stream{std::string(contents)};
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(stream, line)) {
        lines.push_back(trimCopy(line));
    }

    if (lines.empty()) {
        return false;
    }

    std::size_t idx = 0;
    while (idx < lines.size() && lines[idx].empty()) {
        ++idx;
    }
    if (idx >= lines.size()) {
        return false;
    }

    std::string parsedName = std::move(fallbackName);
    float testX = 0.0f;
    float testY = 0.0f;
    if (!tryParsePair(lines[idx], testX, testY)) {
        if (!lines[idx].empty()) {
            parsedName = lines[idx];
        }
        ++idx;
    }

    while (idx < lines.size() && lines[idx].empty()) {
        ++idx;
    }
    if (idx >= lines.size()) {
        return false;
    }

    // Check for Lednicer header line (two counts > 1.0, e.g. "61.0 61.0").
    bool isLednicer = false;
    std::size_t upperCount = 0;
    std::size_t lowerCount = 0;
    if (tryParsePair(lines[idx], testX, testY) && testX > 2.0f && testY > 2.0f) {
        const float rx = std::round(testX);
        const float ry = std::round(testY);
        if (std::abs(testX - rx) < 1e-3f && std::abs(testY - ry) < 1e-3f) {
            isLednicer = true;
            upperCount = static_cast<std::size_t>(rx);
            lowerCount = static_cast<std::size_t>(ry);
            ++idx;
        }
    }

    std::vector<glm::vec2> rawUpper;
    std::vector<glm::vec2> rawLower;

    if (isLednicer) {
        std::vector<glm::vec2> allPts;
        for (; idx < lines.size(); ++idx) {
            if (lines[idx].empty()) {
                continue;
            }
            float x = 0.0f;
            float y = 0.0f;
            if (tryParsePair(lines[idx], x, y)) {
                allPts.emplace_back(x, y);
            }
        }
        if (allPts.size() < upperCount + lowerCount || upperCount < 2 || lowerCount < 2) {
            return false;
        }
        rawUpper.assign(allPts.begin(), allPts.begin() + static_cast<std::ptrdiff_t>(upperCount));
        rawLower.assign(
            allPts.begin() + static_cast<std::ptrdiff_t>(upperCount),
            allPts.begin() + static_cast<std::ptrdiff_t>(upperCount + lowerCount)
        );
    } else {
        std::vector<glm::vec2> loopPts;
        for (; idx < lines.size(); ++idx) {
            if (lines[idx].empty()) {
                continue;
            }
            float x = 0.0f;
            float y = 0.0f;
            if (tryParsePair(lines[idx], x, y)) {
                loopPts.emplace_back(x, y);
            }
        }
        if (loopPts.size() < 6) {
            return false;
        }

        // Split Selig loop (TE -> Upper -> LE -> Lower -> TE) at minimum X (leading edge).
        std::size_t leIndex = 0;
        for (std::size_t i = 1; i < loopPts.size(); ++i) {
            if (loopPts[i].x < loopPts[leIndex].x) {
                leIndex = i;
            }
        }
        if (leIndex == 0 || leIndex + 1 >= loopPts.size()) {
            return false;
        }

        for (std::size_t i = leIndex + 1; i > 0; --i) {
            rawUpper.push_back(loopPts[i - 1]);
        }
        for (std::size_t i = leIndex; i < loopPts.size(); ++i) {
            rawLower.push_back(loopPts[i]);
        }
    }

    if (rawUpper.size() < 2 || rawLower.size() < 2) {
        return false;
    }

    // Normalize both surfaces together to [0, 1] chord.
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    for (const auto& pt : rawUpper) {
        minX = std::min(minX, pt.x);
        maxX = std::max(maxX, pt.x);
    }
    for (const auto& pt : rawLower) {
        minX = std::min(minX, pt.x);
        maxX = std::max(maxX, pt.x);
    }
    const float chord = maxX - minX;
    if (chord <= 1e-6f) {
        return false;
    }
    for (auto& pt : rawUpper) {
        pt.x = (pt.x - minX) / chord;
        pt.y /= chord;
    }
    for (auto& pt : rawLower) {
        pt.x = (pt.x - minX) / chord;
        pt.y /= chord;
    }

    m_rawDatUpper = normalizeCurveLEToTE(std::move(rawUpper));
    m_rawDatLower = normalizeCurveLEToTE(std::move(rawLower));
    m_type = AirfoilType::DatFile;
    m_name = std::move(parsedName);
    rebuild();
    return true;
}

std::string Airfoil::toDatString() const {
    if (m_loop.empty()) {
        return {};
    }
    std::ostringstream out;
    out << m_name << '\n';
    out << std::fixed << std::setprecision(6);
    for (const glm::vec2& pt : m_loop) {
        out << std::setw(10) << pt.x << "  " << std::setw(10) << pt.y << '\n';
    }
    return out.str();
}

bool Airfoil::saveToDatFile(const std::filesystem::path& path) const {
    const std::string contents = toDatString();
    if (contents.empty()) {
        return false;
    }
    std::ofstream out(path);
    if (!out.is_open()) {
        return false;
    }
    out << contents;
    return out.good();
}

void Airfoil::setCst(std::vector<double> upperWeights, std::vector<double> lowerWeights, std::string name) {
    if (upperWeights.size() >= 2 && upperWeights.size() == lowerWeights.size()) {
        m_cstUpper = std::move(upperWeights);
        m_cstLower = std::move(lowerWeights);
    }
    m_type = AirfoilType::Cst;
    m_name = std::move(name);
    m_datPath.clear();
    rebuild();
}

void Airfoil::convertToCst(int order) {
    const int clampedOrder = std::clamp(order, 2, 8);
    m_cstUpper = fitCstWeights(m_upper, clampedOrder);
    m_cstLower = fitCstWeights(m_lower, clampedOrder);
    m_thicknessScale = 1.0f;
    m_camberScale = 1.0f;
    m_type = AirfoilType::Cst;
    if (m_name.find("(CST)") == std::string::npos) {
        m_name += " (CST)";
    }
    m_datPath.clear();
    rebuild();
}

const std::vector<double>& Airfoil::getCstUpperWeights() const noexcept {
    return m_cstUpper;
}

const std::vector<double>& Airfoil::getCstLowerWeights() const noexcept {
    return m_cstLower;
}

void Airfoil::setCstUpperWeight(std::size_t index, double value) {
    if (index >= m_cstUpper.size()) {
        return;
    }
    m_cstUpper[index] = std::clamp(value, -1.5, 1.5);
    if (m_type == AirfoilType::Cst) {
        rebuild();
    }
}

void Airfoil::setCstLowerWeight(std::size_t index, double value) {
    if (index >= m_cstLower.size()) {
        return;
    }
    m_cstLower[index] = std::clamp(value, -1.5, 1.5);
    if (m_type == AirfoilType::Cst) {
        rebuild();
    }
}

AirfoilType Airfoil::getType() const noexcept {
    return m_type;
}

const std::string& Airfoil::getName() const noexcept {
    return m_name;
}

const std::string& Airfoil::getNaca4Code() const noexcept {
    return m_naca4Code;
}

const std::string& Airfoil::getNaca5Code() const noexcept {
    return m_naca5Code;
}

const std::string& Airfoil::getDatPath() const noexcept {
    return m_datPath;
}

int Airfoil::getPointsPerSurface() const noexcept {
    return m_pointsPerSurface;
}

void Airfoil::setPointsPerSurface(int count) {
    const int clamped = std::clamp(count, 9, 201);
    if (clamped == m_pointsPerSurface) {
        return;
    }
    m_pointsPerSurface = clamped;
    rebuild();
}

float Airfoil::getThicknessScale() const noexcept {
    return m_thicknessScale;
}

bool Airfoil::setThicknessScale(float scale) {
    const float clamped = std::clamp(scale, 0.05f, 4.0f);
    if (std::abs(clamped - m_thicknessScale) <= 1e-5f) {
        return false;
    }
    m_thicknessScale = clamped;
    rebuild();
    return true;
}

float Airfoil::getCamberScale() const noexcept {
    return m_camberScale;
}

bool Airfoil::setCamberScale(float scale) {
    const float clamped = std::clamp(scale, -3.0f, 3.0f);
    if (std::abs(clamped - m_camberScale) <= 1e-5f) {
        return false;
    }
    m_camberScale = clamped;
    rebuild();
    return true;
}

float Airfoil::getTeGap() const noexcept {
    return m_teGap;
}

bool Airfoil::setTeGap(float gap) {
    const float clamped = std::clamp(gap, 0.0f, 0.05f);
    if (std::abs(clamped - m_teGap) <= 1e-6f) {
        return false;
    }
    m_teGap = clamped;
    rebuild();
    return true;
}

bool Airfoil::isSharpTE() const noexcept {
    return m_sharpTE;
}

bool Airfoil::setSharpTE(bool sharpTE) {
    if (sharpTE == m_sharpTE) {
        return false;
    }
    m_sharpTE = sharpTE;
    rebuild();
    return true;
}

const std::vector<glm::vec2>& Airfoil::getUpperSurface() const noexcept {
    return m_upper;
}

const std::vector<glm::vec2>& Airfoil::getLowerSurface() const noexcept {
    return m_lower;
}

const std::vector<glm::vec2>& Airfoil::getCamberLine() const noexcept {
    return m_camber;
}

const std::vector<glm::vec2>& Airfoil::getClosedLoop() const noexcept {
    return m_loop;
}

float Airfoil::getMaxThicknessRatio() const noexcept {
    return m_maxThicknessRatio;
}

float Airfoil::getMaxCamberRatio() const noexcept {
    return m_maxCamberRatio;
}

std::vector<std::filesystem::path> Airfoil::discoverDatLibrary(const std::filesystem::path& directory) {
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    if (!std::filesystem::exists(directory, ec) || !std::filesystem::is_directory(directory, ec)) {
        return files;
    }
    for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
        if (!entry.is_regular_file(ec)) {
            continue;
        }
        auto ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        if (ext == ".dat") {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

void Airfoil::rebuild() {
    switch (m_type) {
    case AirfoilType::Naca4:
        generateNaca4();
        break;
    case AirfoilType::Naca5:
        generateNaca5();
        break;
    case AirfoilType::DatFile:
        resampleFromRawDat();
        break;
    case AirfoilType::Cst:
        generateCst();
        break;
    }
    buildLoopAndMetrics();
}

void Airfoil::generateNaca4() {
    const float m = static_cast<float>(m_naca4Code[0] - '0') / 100.0f * m_camberScale;
    const float p = static_cast<float>(m_naca4Code[1] - '0') / 10.0f;
    const float t = static_cast<float>(std::stoi(m_naca4Code.substr(2, 2))) / 100.0f * m_thicknessScale;

    const std::vector<float> xs = makeCosineGrid(m_pointsPerSurface);
    m_upper.resize(xs.size());
    m_lower.resize(xs.size());
    m_camber.resize(xs.size());

    const bool hasCamber = (std::abs(m) > 1e-6f) && (p > 1e-4f) && (p < 1.0f - 1e-4f);

    for (std::size_t i = 0; i < xs.size(); ++i) {
        const float x = xs[i];
        const float yt = nacaThicknessHalfWidth(x, t, m_sharpTE) + 0.5f * m_teGap * x;

        float yc = 0.0f;
        float dycDx = 0.0f;
        if (hasCamber) {
            if (x < p) {
                yc = (m / (p * p)) * (2.0f * p * x - x * x);
                dycDx = (2.0f * m / (p * p)) * (p - x);
            } else {
                const float omp = 1.0f - p;
                yc = (m / (omp * omp)) * ((1.0f - 2.0f * p) + 2.0f * p * x - x * x);
                dycDx = (2.0f * m / (omp * omp)) * (p - x);
            }
        }

        const float theta = std::atan(dycDx);
        const float sinT = std::sin(theta);
        const float cosT = std::cos(theta);

        m_camber[i] = glm::vec2(x, yc);
        m_upper[i] = glm::vec2(std::clamp(x - yt * sinT, 0.0f, 1.0f), yc + yt * cosT);
        m_lower[i] = glm::vec2(std::clamp(x + yt * sinT, 0.0f, 1.0f), yc - yt * cosT);
    }
}

void Airfoil::generateNaca5() {
    const int lDigit = m_naca5Code[0] - '0';
    const int pDigit = std::clamp(m_naca5Code[1] - '0', 1, 5);
    const bool reflexed = (m_naca5Code[2] == '1');
    const float t = static_cast<float>(std::stoi(m_naca5Code.substr(3, 2))) / 100.0f * m_thicknessScale;

    const float designCl = 0.15f * static_cast<float>(lDigit);
    const float clScale = (designCl / 0.30f) * m_camberScale;
    const Naca5Constants c = lookupNaca5Constants(pDigit, reflexed);

    const std::vector<float> xs = makeCosineGrid(m_pointsPerSurface);
    m_upper.resize(xs.size());
    m_lower.resize(xs.size());
    m_camber.resize(xs.size());

    const float m = c.m;
    const float k1 = c.k1;
    const float k2k1 = c.k2OverK1;
    const float oneMinusM3 = (1.0f - m) * (1.0f - m) * (1.0f - m);
    const float m3 = m * m * m;

    for (std::size_t i = 0; i < xs.size(); ++i) {
        const float x = xs[i];
        const float yt = nacaThicknessHalfWidth(x, t, m_sharpTE) + 0.5f * m_teGap * x;

        float yc = 0.0f;
        float dycDx = 0.0f;

        if (!reflexed) {
            if (x < m) {
                const float xm = x - m;
                yc = (k1 / 6.0f) * (x * x * x - 3.0f * m * x * x + m * m * (3.0f - m) * x);
                dycDx = (k1 / 6.0f) * (3.0f * x * x - 6.0f * m * x + m * m * (3.0f - m));
                (void)xm;
            } else {
                yc = (k1 * m3 / 6.0f) * (1.0f - x);
                dycDx = -(k1 * m3 / 6.0f);
            }
        } else {
            if (x < m) {
                const float xm = x - m;
                yc = (k1 / 6.0f) * (xm * xm * xm - k2k1 * oneMinusM3 * x - m3 * x + m3);
                dycDx = (k1 / 6.0f) * (3.0f * xm * xm - k2k1 * oneMinusM3 - m3);
            } else {
                const float xm = x - m;
                yc = (k1 / 6.0f) * (k2k1 * xm * xm * xm - k2k1 * oneMinusM3 * x - m3 * x + m3);
                dycDx = (k1 / 6.0f) * (3.0f * k2k1 * xm * xm - k2k1 * oneMinusM3 - m3);
            }
        }

        yc *= clScale;
        dycDx *= clScale;

        const float theta = std::atan(dycDx);
        const float sinT = std::sin(theta);
        const float cosT = std::cos(theta);

        m_camber[i] = glm::vec2(x, yc);
        m_upper[i] = glm::vec2(std::clamp(x - yt * sinT, 0.0f, 1.0f), yc + yt * cosT);
        m_lower[i] = glm::vec2(std::clamp(x + yt * sinT, 0.0f, 1.0f), yc - yt * cosT);
    }
}

void Airfoil::generateCst() {
    const std::vector<float> xs = makeCosineGrid(m_pointsPerSurface);
    m_upper.resize(xs.size());
    m_lower.resize(xs.size());
    m_camber.resize(xs.size());

    const int order = static_cast<int>(m_cstUpper.size()) - 1;
    if (order < 1 || m_cstUpper.size() != m_cstLower.size()) {
        generateNaca4();
        return;
    }

    for (std::size_t i = 0; i < xs.size(); ++i) {
        const double x = static_cast<double>(xs[i]);
        double yUpBase = 0.0;
        double yLoBase = 0.0;
        for (int w = 0; w <= order; ++w) {
            const double basis = cstBasis(order, w, x);
            yUpBase += m_cstUpper[static_cast<std::size_t>(w)] * basis;
            yLoBase += m_cstLower[static_cast<std::size_t>(w)] * basis;
        }

        const float yc = 0.5f * static_cast<float>(yUpBase + yLoBase) * m_camberScale;
        const float yt = 0.5f * static_cast<float>(yUpBase - yLoBase) * m_thicknessScale + 0.5f * m_teGap * xs[i];

        m_camber[i] = glm::vec2(xs[i], yc);
        m_upper[i] = glm::vec2(xs[i], yc + yt);
        m_lower[i] = glm::vec2(xs[i], yc - yt);
    }
}

void Airfoil::resampleFromRawDat() {
    if (m_rawDatUpper.size() < 2 || m_rawDatLower.size() < 2) {
        generateNaca4();
        return;
    }

    const std::vector<float> xs = makeCosineGrid(m_pointsPerSurface);
    m_upper.resize(xs.size());
    m_lower.resize(xs.size());
    m_camber.resize(xs.size());

    for (std::size_t i = 0; i < xs.size(); ++i) {
        const float x = xs[i];
        const float yUpBase = sampleSurfaceSqrtSpace(m_rawDatUpper, x);
        const float yLoBase = sampleSurfaceSqrtSpace(m_rawDatLower, x);

        const float yc = 0.5f * (yUpBase + yLoBase) * m_camberScale;
        const float yt = 0.5f * (yUpBase - yLoBase) * m_thicknessScale + 0.5f * m_teGap * x;

        m_camber[i] = glm::vec2(x, yc);
        m_upper[i] = glm::vec2(x, yc + yt);
        m_lower[i] = glm::vec2(x, yc - yt);
    }
}

void Airfoil::buildLoopAndMetrics() {
    m_loop.clear();
    if (m_upper.empty() || m_lower.empty()) {
        m_maxThicknessRatio = 0.0f;
        m_maxCamberRatio = 0.0f;
        return;
    }

    m_loop.reserve(m_upper.size() + m_lower.size() - 1);
    for (std::size_t i = m_upper.size(); i > 0; --i) {
        m_loop.push_back(m_upper[i - 1]);
    }
    for (std::size_t i = 1; i < m_lower.size(); ++i) {
        m_loop.push_back(m_lower[i]);
    }

    float maxThick = 0.0f;
    float maxCamberAbs = 0.0f;
    for (std::size_t i = 0; i < m_upper.size(); ++i) {
        maxThick = std::max(maxThick, m_upper[i].y - m_lower[i].y);
        maxCamberAbs = std::max(maxCamberAbs, std::abs(m_camber[i].y));
    }
    m_maxThicknessRatio = maxThick;
    m_maxCamberRatio = maxCamberAbs;
}

} // namespace opads
