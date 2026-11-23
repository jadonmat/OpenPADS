#include "core/Aircraft.hpp"

#include <algorithm>
#include <utility>

namespace opads {

Aircraft::Aircraft(std::string name)
    : m_name(std::move(name)) {
}

const std::string& Aircraft::getName() const noexcept {
    return m_name;
}

void Aircraft::setName(std::string name) {
    m_name = std::move(name);
}

std::uint64_t Aircraft::allocateComponentId() noexcept {
    return m_nextComponentId++;
}

Component* Aircraft::addComponent(std::unique_ptr<Component> component) {
    if (!component) {
        return nullptr;
    }
    Component* rawPtr = component.get();
    m_components.push_back(std::move(component));
    return rawPtr;
}

bool Aircraft::removeComponent(std::uint64_t id) {
    const auto it = std::find_if(
        m_components.begin(),
        m_components.end(),
        [id](const std::unique_ptr<Component>& item) { return item->getId() == id; }
    );
    if (it == m_components.end()) {
        return false;
    }
    m_components.erase(it);
    if (m_selectedComponentId == id) {
        m_selectedComponentId.reset();
    }
    return true;
}

Component* Aircraft::findComponent(std::uint64_t id) noexcept {
    for (const auto& item : m_components) {
        if (item->getId() == id) {
            return item.get();
        }
    }
    return nullptr;
}

const Component* Aircraft::findComponent(std::uint64_t id) const noexcept {
    for (const auto& item : m_components) {
        if (item->getId() == id) {
            return item.get();
        }
    }
    return nullptr;
}

const std::vector<std::unique_ptr<Component>>& Aircraft::getComponents() const noexcept {
    return m_components;
}

std::optional<std::uint64_t> Aircraft::getSelectedComponentId() const noexcept {
    return m_selectedComponentId;
}

void Aircraft::setSelectedComponentId(std::optional<std::uint64_t> id) noexcept {
    m_selectedComponentId = id;
}

void Aircraft::rebuildDirty() {
    for (const auto& component : m_components) {
        component->ensureBuilt();
    }
}

} // namespace opads
