#include "core/MeshImporter.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace opads {

namespace {

std::string toLowerCopy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

struct Vec3Key {
    float x;
    float y;
    float z;

    bool operator==(const Vec3Key& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct Vec3KeyHash {
    std::size_t operator()(const Vec3Key& k) const noexcept {
        const auto h1 = std::hash<float>{}(k.x);
        const auto h2 = std::hash<float>{}(k.y);
        const auto h3 = std::hash<float>{}(k.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

void addWeldedTriangle(
    const glm::vec3& p0,
    const glm::vec3& p1,
    const glm::vec3& p2,
    std::vector<glm::vec3>& positions,
    std::vector<std::uint32_t>& indices,
    std::unordered_map<Vec3Key, std::uint32_t, Vec3KeyHash>& cache
) {
    auto getIdx = [&](const glm::vec3& p) -> std::uint32_t {
        const Vec3Key key{p.x, p.y, p.z};
        const auto it = cache.find(key);
        if (it != cache.end()) {
            return it->second;
        }
        const auto idx = static_cast<std::uint32_t>(positions.size());
        positions.push_back(p);
        cache.emplace(key, idx);
        return idx;
    };

    indices.push_back(getIdx(p0));
    indices.push_back(getIdx(p1));
    indices.push_back(getIdx(p2));
}

} // namespace

bool MeshImporter::importStl(
    Aircraft& aircraft,
    const std::filesystem::path& path,
    float unitScale,
    UpAxisConversion axisConversion
) {
    std::error_code ec;
    const auto fileSize = std::filesystem::file_size(path, ec);
    if (ec || fileSize < 84) {
        return false;
    }

    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        return false;
    }

    char header[80]{};
    in.read(header, 80);
    std::uint32_t triCount = 0;
    in.read(reinterpret_cast<char*>(&triCount), sizeof(triCount));

    std::vector<glm::vec3> positions;
    std::vector<std::uint32_t> indices;
    std::unordered_map<Vec3Key, std::uint32_t, Vec3KeyHash> weldMap;

    const bool isBinary = (fileSize == 84 + static_cast<std::uintmax_t>(triCount) * 50);
    if (isBinary) {
        positions.reserve(static_cast<std::size_t>(triCount) / 2 + 16);
        indices.reserve(static_cast<std::size_t>(triCount) * 3);

        for (std::uint32_t i = 0; i < triCount; ++i) {
            float buf[12]{};
            std::uint16_t attr = 0;
            in.read(reinterpret_cast<char*>(buf), sizeof(buf));
            in.read(reinterpret_cast<char*>(&attr), sizeof(attr));
            if (!in) {
                return false;
            }
            addWeldedTriangle(
                glm::vec3(buf[3], buf[4], buf[5]),
                glm::vec3(buf[6], buf[7], buf[8]),
                glm::vec3(buf[9], buf[10], buf[11]),
                positions,
                indices,
                weldMap
            );
        }
    } else {
        in.close();
        std::ifstream ascii(path);
        if (!ascii.is_open()) {
            return false;
        }
        std::string token;
        std::vector<glm::vec3> triVerts;
        triVerts.reserve(3);
        while (ascii >> token) {
            if (token == "vertex") {
                float x = 0.0f;
                float y = 0.0f;
                float z = 0.0f;
                if (ascii >> x >> y >> z) {
                    triVerts.emplace_back(x, y, z);
                    if (triVerts.size() == 3) {
                        addWeldedTriangle(triVerts[0], triVerts[1], triVerts[2], positions, indices, weldMap);
                        triVerts.clear();
                    }
                }
            }
        }
    }

    if (positions.empty() || indices.size() < 3) {
        return false;
    }

    const std::uint64_t id = aircraft.allocateComponentId();
    auto comp = std::make_unique<ImportedMesh>(id, path.stem().string(), std::move(positions), std::move(indices), path.string());
    comp->setUnitScale(unitScale);
    comp->setAxisConversion(axisConversion);
    comp->ensureBuilt();
    aircraft.addComponent(std::move(comp));
    aircraft.setSelectedComponentId(id);
    return true;
}

bool MeshImporter::importObj(
    Aircraft& aircraft,
    const std::filesystem::path& path,
    float unitScale,
    UpAxisConversion axisConversion
) {
    std::ifstream in(path);
    if (!in.is_open()) {
        return false;
    }

    std::vector<glm::vec3> globalPositions;
    struct ObjGroup {
        std::string name;
        std::vector<std::uint32_t> indices;
    };
    std::vector<ObjGroup> groups;
    groups.push_back(ObjGroup{path.stem().string(), {}});

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            if (iss >> x >> y >> z) {
                globalPositions.emplace_back(x, y, z);
            }
        } else if (prefix == "o" || prefix == "g") {
            std::string grpName;
            iss >> grpName;
            if (!grpName.empty()) {
                if (groups.back().indices.empty()) {
                    groups.back().name = grpName;
                } else {
                    groups.push_back(ObjGroup{grpName, {}});
                }
            }
        } else if (prefix == "f") {
            std::vector<std::uint32_t> faceIndices;
            std::string vertToken;
            while (iss >> vertToken) {
                const std::size_t slashPos = vertToken.find('/');
                const std::string idxStr = (slashPos == std::string::npos) ? vertToken : vertToken.substr(0, slashPos);
                if (idxStr.empty()) {
                    continue;
                }
                const int rawIdx = std::stoi(idxStr);
                int resolved = -1;
                if (rawIdx > 0) {
                    resolved = rawIdx - 1;
                } else if (rawIdx < 0) {
                    resolved = static_cast<int>(globalPositions.size()) + rawIdx;
                }
                if (resolved >= 0 && static_cast<std::size_t>(resolved) < globalPositions.size()) {
                    faceIndices.push_back(static_cast<std::uint32_t>(resolved));
                }
            }
            for (std::size_t k = 1; k + 1 < faceIndices.size(); ++k) {
                groups.back().indices.push_back(faceIndices[0]);
                groups.back().indices.push_back(faceIndices[k]);
                groups.back().indices.push_back(faceIndices[k + 1]);
            }
        }
    }

    if (globalPositions.empty()) {
        return false;
    }

    std::uint64_t firstAddedId = 0;
    for (const ObjGroup& grp : groups) {
        if (grp.indices.size() < 3) {
            continue;
        }
        // Compact vertices per group
        std::vector<glm::vec3> localPos;
        std::vector<std::uint32_t> localIdx;
        localIdx.reserve(grp.indices.size());
        std::unordered_map<std::uint32_t, std::uint32_t> remap;
        for (const std::uint32_t gIdx : grp.indices) {
            const auto it = remap.find(gIdx);
            if (it != remap.end()) {
                localIdx.push_back(it->second);
            } else {
                const auto newIdx = static_cast<std::uint32_t>(localPos.size());
                localPos.push_back(globalPositions[gIdx]);
                remap.emplace(gIdx, newIdx);
                localIdx.push_back(newIdx);
            }
        }

        const std::uint64_t id = aircraft.allocateComponentId();
        if (firstAddedId == 0) {
            firstAddedId = id;
        }
        auto comp = std::make_unique<ImportedMesh>(id, grp.name, std::move(localPos), std::move(localIdx), path.string());
        comp->setUnitScale(unitScale);
        comp->setAxisConversion(axisConversion);
        comp->ensureBuilt();
        aircraft.addComponent(std::move(comp));
    }

    if (firstAddedId != 0) {
        aircraft.setSelectedComponentId(firstAddedId);
        return true;
    }
    return false;
}

bool MeshImporter::importAuto(
    Aircraft& aircraft,
    const std::filesystem::path& path,
    float unitScale,
    UpAxisConversion axisConversion
) {
    const std::string ext = toLowerCopy(path.extension().string());
    if (ext == ".stl") {
        return importStl(aircraft, path, unitScale, axisConversion);
    }
    if (ext == ".obj") {
        return importObj(aircraft, path, unitScale, axisConversion);
    }
    return false;
}

} // namespace opads
