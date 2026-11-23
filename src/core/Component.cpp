#include "core/Component.hpp"

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

} // namespace opads
