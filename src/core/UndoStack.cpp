#include "core/UndoStack.hpp"

#include <utility>

#include "core/ProjectSerializer.hpp"

namespace opads {

const std::string UndoStack::kEmptyLabel;

UndoStack::UndoStack(std::size_t maxEntries)
    : m_maxEntries(maxEntries > 0 ? maxEntries : 1) {
}

void UndoStack::clear() noexcept {
    m_undoEntries.clear();
    m_redoEntries.clear();
    cancelTransaction();
}

void UndoStack::beginTransaction(const Aircraft& aircraft, std::string description) {
    m_inTransaction = true;
    m_pendingDescription = std::move(description);
    m_pendingBeforeJson = ProjectSerializer::saveToString(aircraft);
}

bool UndoStack::commitTransaction(const Aircraft& aircraft) {
    if (!m_inTransaction) {
        return false;
    }
    m_inTransaction = false;
    std::string afterJson = ProjectSerializer::saveToString(aircraft);
    if (afterJson == m_pendingBeforeJson) {
        m_pendingDescription.clear();
        m_pendingBeforeJson.clear();
        return false;
    }

    m_redoEntries.clear();
    m_undoEntries.push_back(Entry{
        std::move(m_pendingDescription),
        std::move(m_pendingBeforeJson),
        std::move(afterJson)
    });
    if (m_undoEntries.size() > m_maxEntries) {
        m_undoEntries.erase(m_undoEntries.begin());
    }
    return true;
}

void UndoStack::cancelTransaction() noexcept {
    m_inTransaction = false;
    m_pendingDescription.clear();
    m_pendingBeforeJson.clear();
}

void UndoStack::pushSnapshot(std::string beforeJson, const Aircraft& afterAircraft, std::string description) {
    std::string afterJson = ProjectSerializer::saveToString(afterAircraft);
    if (afterJson == beforeJson) {
        return;
    }
    m_redoEntries.clear();
    m_undoEntries.push_back(Entry{
        std::move(description),
        std::move(beforeJson),
        std::move(afterJson)
    });
    if (m_undoEntries.size() > m_maxEntries) {
        m_undoEntries.erase(m_undoEntries.begin());
    }
}

bool UndoStack::canUndo() const noexcept {
    return !m_undoEntries.empty();
}

bool UndoStack::canRedo() const noexcept {
    return !m_redoEntries.empty();
}

const std::string& UndoStack::getUndoLabel() const noexcept {
    return m_undoEntries.empty() ? kEmptyLabel : m_undoEntries.back().description;
}

const std::string& UndoStack::getRedoLabel() const noexcept {
    return m_redoEntries.empty() ? kEmptyLabel : m_redoEntries.back().description;
}

bool UndoStack::undo(Aircraft& aircraft) {
    if (m_undoEntries.empty()) {
        return false;
    }
    cancelTransaction();
    Entry entry = std::move(m_undoEntries.back());
    m_undoEntries.pop_back();

    if (!ProjectSerializer::loadFromString(aircraft, entry.beforeJson)) {
        return false;
    }
    m_redoEntries.push_back(std::move(entry));
    return true;
}

bool UndoStack::redo(Aircraft& aircraft) {
    if (m_redoEntries.empty()) {
        return false;
    }
    cancelTransaction();
    Entry entry = std::move(m_redoEntries.back());
    m_redoEntries.pop_back();

    if (!ProjectSerializer::loadFromString(aircraft, entry.afterJson)) {
        return false;
    }
    m_undoEntries.push_back(std::move(entry));
    return true;
}

} // namespace opads
