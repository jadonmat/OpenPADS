#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "core/Aircraft.hpp"

namespace opads {

/// Snapshot-based Undo/Redo history stack for Aircraft editing operations.
class UndoStack {
public:
    explicit UndoStack(std::size_t maxEntries = 64);

    void clear() noexcept;

    /// Captures the current aircraft state before an interactive drag begins.
    void beginTransaction(const Aircraft& aircraft, std::string description);
    /// Commits a previously started transaction if the aircraft state changed.
    bool commitTransaction(const Aircraft& aircraft);
    void cancelTransaction() noexcept;

    /// Records a discrete edit from an already-captured before-state snapshot.
    void pushSnapshot(std::string beforeJson, const Aircraft& afterAircraft, std::string description);

    bool canUndo() const noexcept;
    bool canRedo() const noexcept;

    const std::string& getUndoLabel() const noexcept;
    const std::string& getRedoLabel() const noexcept;

    bool undo(Aircraft& aircraft);
    bool redo(Aircraft& aircraft);

private:
    struct Entry {
        std::string description;
        std::string beforeJson;
        std::string afterJson;
    };

    std::size_t m_maxEntries;
    std::vector<Entry> m_undoEntries;
    std::vector<Entry> m_redoEntries;

    bool m_inTransaction{false};
    std::string m_pendingDescription;
    std::string m_pendingBeforeJson;
    static const std::string kEmptyLabel;
};

} // namespace opads
