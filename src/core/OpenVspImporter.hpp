#pragma once

#include <filesystem>
#include <string_view>

#include "core/Aircraft.hpp"

namespace opads {

/// Imports OpenVSP (.vsp3 XML) geometry files natively into parametric OpenPADS components.
class OpenVspImporter {
public:
    static bool importFromFile(Aircraft& aircraft, const std::filesystem::path& path);
    static bool importFromString(Aircraft& aircraft, std::string_view xmlContents, const std::string& fallbackName = "Imported OpenVSP");
};

} // namespace opads
