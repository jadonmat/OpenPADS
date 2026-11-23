#pragma once

#include <filesystem>
#include <string>
#include <vector>

struct GLFWwindow;

namespace opads {

class Airfoil;

/// Shared UI helpers for the searchable airfoil database dropdown and .dat file upload.
class AirfoilUiHelper {
public:
    static std::filesystem::path resolveAirfoilsDir();
    static void refreshLibrary(std::vector<std::filesystem::path>& outFiles);

    /// Renders a searchable dropdown of all bundled/uploaded .dat airfoils.
    static bool renderSearchableDatCombo(
        const char* label,
        Airfoil& airfoil,
        std::vector<std::filesystem::path>& libraryFiles,
        std::string& searchFilter
    );

    /// Opens a native file dialog to upload/import an external .dat airfoil into the library.
    static bool uploadDatFile(
        GLFWwindow* ownerWindow,
        Airfoil& airfoil,
        std::vector<std::filesystem::path>& libraryFiles,
        std::string& statusOut
    );
};

} // namespace opads
