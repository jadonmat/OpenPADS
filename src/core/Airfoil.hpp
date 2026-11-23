#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include <glm/vec2.hpp>

namespace opads {

enum class AirfoilType {
    Naca4,
    Naca5,
    DatFile
};

/// 2D parametric and coordinate-based airfoil generator (NACA 4/5-digit + Selig/Lednicer .dat).
class Airfoil {
public:
    Airfoil();

    static Airfoil createNaca4(std::string_view code, int pointsPerSurface = 41);
    static Airfoil createNaca5(std::string_view code, int pointsPerSurface = 41);

    bool loadFromDatFile(const std::filesystem::path& path);
    bool loadFromDatString(std::string_view contents, std::string fallbackName = "Custom Airfoil");

    void setNaca4(std::string_view code);
    void setNaca5(std::string_view code);

    AirfoilType getType() const noexcept;
    const std::string& getName() const noexcept;
    const std::string& getNaca4Code() const noexcept;
    const std::string& getNaca5Code() const noexcept;
    const std::string& getDatPath() const noexcept;

    int getPointsPerSurface() const noexcept;
    void setPointsPerSurface(int count);

    float getThicknessScale() const noexcept;
    bool setThicknessScale(float scale);

    float getCamberScale() const noexcept;
    bool setCamberScale(float scale);

    float getTeGap() const noexcept;
    bool setTeGap(float gap);

    bool isSharpTE() const noexcept;
    bool setSharpTE(bool sharpTE);

    /// Upper surface ordered LE (x=0) -> TE (x=1).
    const std::vector<glm::vec2>& getUpperSurface() const noexcept;
    /// Lower surface ordered LE (x=0) -> TE (x=1).
    const std::vector<glm::vec2>& getLowerSurface() const noexcept;
    /// Mean camber line ordered LE -> TE.
    const std::vector<glm::vec2>& getCamberLine() const noexcept;
    /// Closed loop ordered Upper TE -> LE -> Lower TE (size = 2*N - 1).
    const std::vector<glm::vec2>& getClosedLoop() const noexcept;

    float getMaxThicknessRatio() const noexcept;
    float getMaxCamberRatio() const noexcept;

    /// Discovers all .dat files in a directory sorted alphabetically.
    static std::vector<std::filesystem::path> discoverDatLibrary(const std::filesystem::path& directory);

private:
    void rebuild();
    void generateNaca4();
    void generateNaca5();
    void resampleFromRawDat();
    void buildLoopAndMetrics();

    AirfoilType m_type{AirfoilType::Naca4};
    std::string m_name{"NACA 2412"};
    std::string m_naca4Code{"2412"};
    std::string m_naca5Code{"23012"};
    std::string m_datPath;

    int m_pointsPerSurface{41};
    float m_thicknessScale{1.0f};
    float m_camberScale{1.0f};
    float m_teGap{0.0f};
    bool m_sharpTE{true};

    std::vector<glm::vec2> m_rawDatUpper;
    std::vector<glm::vec2> m_rawDatLower;

    std::vector<glm::vec2> m_upper;
    std::vector<glm::vec2> m_lower;
    std::vector<glm::vec2> m_camber;
    std::vector<glm::vec2> m_loop;

    float m_maxThicknessRatio{0.12f};
    float m_maxCamberRatio{0.02f};
};

} // namespace opads
