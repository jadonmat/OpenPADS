#include "ui/AirfoilUiHelper.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstring>

#include <imgui.h>

#include "core/Airfoil.hpp"
#include "ui/FileDialog.hpp"

namespace opads {

namespace {

bool containsCaseInsensitive(const std::string& haystack, const std::string& needle) {
    if (needle.empty()) {
        return true;
    }
    auto lower = [](unsigned char c) { return static_cast<char>(std::tolower(c)); };
    std::string h = haystack;
    std::string n = needle;
    std::transform(h.begin(), h.end(), h.begin(), lower);
    std::transform(n.begin(), n.end(), n.begin(), lower);
    return h.find(n) != std::string::npos;
}

} // namespace

std::filesystem::path AirfoilUiHelper::resolveAirfoilsDir() {
    std::error_code ec;
    const std::array<std::filesystem::path, 5> candidates = {
        std::filesystem::path("data/airfoils"),
        std::filesystem::path("../data/airfoils"),
        std::filesystem::path("../../data/airfoils"),
        std::filesystem::path("../../../data/airfoils"),
        std::filesystem::path(OPENPADS_DATA_DIR) / "airfoils"
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate, ec) && std::filesystem::is_directory(candidate, ec)) {
            return candidate;
        }
    }
    return {};
}

void AirfoilUiHelper::refreshLibrary(std::vector<std::filesystem::path>& outFiles) {
    outFiles = Airfoil::discoverDatLibrary(resolveAirfoilsDir());
}

bool AirfoilUiHelper::renderSearchableDatCombo(
    const char* label,
    Airfoil& airfoil,
    std::vector<std::filesystem::path>& libraryFiles,
    std::string& searchFilter
) {
    bool changed = false;
    const std::string currentStem = airfoil.getDatPath().empty()
        ? std::string("Select from ") + std::to_string(libraryFiles.size()) + " airfoils..."
        : std::filesystem::path(airfoil.getDatPath()).stem().string();

    if (ImGui::BeginCombo(label, currentStem.c_str(), ImGuiComboFlags_HeightLarge)) {
        std::array<char, 64> filterBuf{};
        const std::size_t copyLen = std::min(searchFilter.size(), filterBuf.size() - 1);
        std::memcpy(filterBuf.data(), searchFilter.data(), copyLen);
        filterBuf[copyLen] = '\0';

        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::InputTextWithHint("##AirfoilSearch", "Search database (e.g. naca, sc2, clark)...", filterBuf.data(), filterBuf.size())) {
            searchFilter = filterBuf.data();
        }
        ImGui::Separator();

        ImGui::BeginChild("##AirfoilListScroll", ImVec2(0.0f, 210.0f));
        for (const auto& path : libraryFiles) {
            const std::string stem = path.stem().string();
            if (!containsCaseInsensitive(stem, searchFilter)) {
                continue;
            }
            const bool isSelected = (path.string() == airfoil.getDatPath());
            if (ImGui::Selectable(stem.c_str(), isSelected)) {
                changed |= airfoil.loadFromDatFile(path);
                ImGui::CloseCurrentPopup();
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndChild();
        ImGui::EndCombo();
    }
    return changed;
}

bool AirfoilUiHelper::uploadDatFile(
    GLFWwindow* ownerWindow,
    Airfoil& airfoil,
    std::vector<std::filesystem::path>& libraryFiles,
    std::string& statusOut
) {
    const auto chosen = FileDialog::openFile(
        ownerWindow,
        "Airfoil Coordinates (*.dat;*.txt)\0*.dat;*.txt\0All Files (*.*)\0*.*\0",
        "Upload Airfoil Coordinate File"
    );
    if (!chosen.has_value()) {
        return false;
    }

    if (!airfoil.loadFromDatFile(*chosen)) {
        statusOut = "Invalid airfoil file: " + chosen->filename().string();
        return false;
    }

    // Copy uploaded airfoil into data/airfoils/ so it appears in the searchable library dropdown.
    const std::filesystem::path libDir = resolveAirfoilsDir();
    if (!libDir.empty()) {
        std::error_code ec;
        std::filesystem::path dest = libDir / chosen->filename();
        if (dest.extension() != ".dat") {
            dest.replace_extension(".dat");
        }
        if (std::filesystem::weakly_canonical(*chosen, ec) != std::filesystem::weakly_canonical(dest, ec)) {
            if (std::filesystem::copy_file(*chosen, dest, std::filesystem::copy_options::overwrite_existing, ec)) {
                airfoil.loadFromDatFile(dest);
            }
        }
        refreshLibrary(libraryFiles);
    }

    statusOut = "Loaded: " + airfoil.getName();
    return true;
}

} // namespace opads
