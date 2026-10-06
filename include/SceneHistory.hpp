#pragma once

#include <cstddef>
#include <deque>

#include "WorldState.hpp"

// ============================================================
// SceneHistory
// Undo/redo using two stacks of scene snapshots.
//   record(): push the current scene on the undo stack and
//             empty the redo stack (a new action breaks the redo chain)
//   undo():   move the current scene to the redo stack and
//             restore the top of the undo stack
//   redo():   the mirror image of undo()
// Each stack is a std::deque used only at its back (push_back /
// pop_back = LIFO), which also allows the oldest snapshot to be
// dropped from the front when the capacity is exceeded.
// ============================================================
class SceneHistory {
public:
    explicit SceneHistory(std::size_t capacity = 50);

    void record(const WorldState& current);
    bool undo(WorldState& current);
    bool redo(WorldState& current);
    void clear();

    bool canUndo() const { return !undoStack.empty(); }
    bool canRedo() const { return !redoStack.empty(); }
    std::size_t undoCount() const { return undoStack.size(); }
    std::size_t redoCount() const { return redoStack.size(); }
    std::size_t getCapacity() const { return capacity; }

private:
    std::size_t capacity;
    std::deque<WorldState> undoStack;
    std::deque<WorldState> redoStack;

    void push(std::deque<WorldState>& stack, const WorldState& state);
};
