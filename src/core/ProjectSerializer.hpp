#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include "core/Aircraft.hpp"

namespace opads {

/// JSON project serializer/deserializer for Aircraft and all parametric components.
class ProjectSerializer {
public:
    static std::string saveToString(const Aircraft& aircraft);
    static bool loadFromString(Aircraft& aircraft, std::string_view jsonText);

    static bool saveToFile(const Aircraft& aircraft, const std::filesystem::path& path);
    static bool loadFromFile(Aircraft& aircraft, const std::filesystem::path& path);
};

} // namespace opads
