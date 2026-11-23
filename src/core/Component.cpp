#include "core/Component.hpp"

#include <algorithm>
#include <utility>

namespace opads {

Component::Component(std::uint64_t id, std::string name)
    : m_id(id)
    , m_name(std::move(name)) {
}

std::uint64_t Component::getId() const noexcept {
    return m_id;
}

const std::string& Component::getName() const noexcept {
    return m_name;
}

void Component::setName(std::string name) {
    m_name = std::move(name);
}

bool Component::isVisible() const noexcept {
    return m_visible;
}

void Component::setVisible(bool visible) noexcept {
    m_visible = visible;
}

bool Component::isDirty() const noexcept {
    return m_dirty;
}

void Component::markDirty() noexcept {
    m_dirty = true;
}

Transform& Component::getTransform() noexcept {
    return m_transform;
}

const Transform& Component::getTransform() const noexcept {
    return m_transform;
}

const Mesh& Component::getMesh() const noexcept {
    return m_mesh;
}

void Component::ensureBuilt() {
    if (!m_dirty) {
        return;
    }
    m_mesh.clear();
    rebuildMesh();
    m_dirty = false;
}

std::vector<std::vector<glm::vec3>> Component::getHighlightLoops() const {
    return {};
}

std::vector<glm::vec3> Component::getSnapSamplePointsLocal() const {
    std::vector<glm::vec3> pts;
    const auto& verts = m_mesh.getVertices();
    if (verts.empty()) {
        return pts;
    }
    const std::size_t maxSamples = 64;
    const std::size_t stride = std::max<std::size_t>(1, verts.size() / maxSamples);
    for (std::size_t i = 0; i < verts.size(); i += stride) {
        pts.push_back(verts[i].position);
    }
    return pts;
}

} // namespace opads
