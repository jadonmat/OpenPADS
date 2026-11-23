#include "core/MeshExporter.hpp"

#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <string>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/mat3x3.hpp>
#include <glm/mat4x4.hpp>

namespace opads {

namespace {

std::string sanitizeObjGroupName(const std::string& name) {
    std::string out;
    out.reserve(name.size());
    for (char c : name) {
        out.push_back((c == ' ' || c == '\t' || c == '\r' || c == '\n') ? '_' : c);
    }
    return out.empty() ? std::string("Component") : out;
}

} // namespace

bool MeshExporter::exportBinaryStl(const Aircraft& aircraft, const std::filesystem::path& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) {
        return false;
    }

    std::uint32_t totalTriangles = 0;
    for (const auto& comp : aircraft.getComponents()) {
        if (comp->isVisible()) {
            totalTriangles += static_cast<std::uint32_t>(comp->getMesh().getTriangleCount());
        }
    }

    std::array<char, 80> header{};
    const std::string tag = "OpenPADS Binary STL Export";
    std::copy_n(tag.data(), std::min(tag.size(), header.size()), header.data());
    out.write(header.data(), static_cast<std::streamsize>(header.size()));
    out.write(reinterpret_cast<const char*>(&totalTriangles), sizeof(totalTriangles));

    const std::uint16_t attrByteCount = 0;
    for (const auto& comp : aircraft.getComponents()) {
        if (!comp->isVisible()) {
            continue;
        }
        const Mesh& mesh = comp->getMesh();
        const auto& verts = mesh.getVertices();
        const auto& indices = mesh.getIndices();
        const glm::mat4 model = comp->getTransform().getMatrix();

        for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
            const glm::vec3 p0 = glm::vec3(model * glm::vec4(verts[indices[i]].position, 1.0f));
            const glm::vec3 p1 = glm::vec3(model * glm::vec4(verts[indices[i + 1]].position, 1.0f));
            const glm::vec3 p2 = glm::vec3(model * glm::vec4(verts[indices[i + 2]].position, 1.0f));

            glm::vec3 faceN = glm::cross(p1 - p0, p2 - p0);
            const float len = glm::length(faceN);
            faceN = (len > 1e-8f) ? (faceN / len) : glm::vec3(0.0f, 0.0f, 1.0f);

            out.write(reinterpret_cast<const char*>(&faceN.x), sizeof(float) * 3);
            out.write(reinterpret_cast<const char*>(&p0.x), sizeof(float) * 3);
            out.write(reinterpret_cast<const char*>(&p1.x), sizeof(float) * 3);
            out.write(reinterpret_cast<const char*>(&p2.x), sizeof(float) * 3);
            out.write(reinterpret_cast<const char*>(&attrByteCount), sizeof(attrByteCount));
        }
    }

    return out.good();
}

bool MeshExporter::exportObj(const Aircraft& aircraft, const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return false;
    }

    out << "# OpenPADS Wavefront OBJ Export\n";
    out << std::fixed << std::setprecision(6);

    std::size_t vertexOffset = 1;
    for (const auto& comp : aircraft.getComponents()) {
        if (!comp->isVisible()) {
            continue;
        }
        const Mesh& mesh = comp->getMesh();
        if (mesh.isEmpty()) {
            continue;
        }

        out << "o " << sanitizeObjGroupName(comp->getName()) << '\n';
        const glm::mat4 model = comp->getTransform().getMatrix();
        const glm::mat3 normalMat = glm::inverseTranspose(glm::mat3(model));

        const auto& verts = mesh.getVertices();
        const auto& indices = mesh.getIndices();

        for (const Vertex& v : verts) {
            const glm::vec3 wp = glm::vec3(model * glm::vec4(v.position, 1.0f));
            out << "v " << wp.x << ' ' << wp.y << ' ' << wp.z << '\n';
        }
        for (const Vertex& v : verts) {
            const glm::vec3 wn = glm::normalize(normalMat * v.normal);
            out << "vn " << wn.x << ' ' << wn.y << ' ' << wn.z << '\n';
        }
        for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
            const std::size_t i0 = vertexOffset + indices[i];
            const std::size_t i1 = vertexOffset + indices[i + 1];
            const std::size_t i2 = vertexOffset + indices[i + 2];
            out << "f " << i0 << "//" << i0 << ' '
                << i1 << "//" << i1 << ' '
                << i2 << "//" << i2 << '\n';
        }
        vertexOffset += verts.size();
    }

    return out.good();
}

} // namespace opads
