#include "SceneHistory.hpp"

SceneHistory::SceneHistory(std::size_t capacity) : capacity(capacity > 0 ? capacity : 1) {}

void SceneHistory::push(std::deque<WorldState>& stack, const WorldState& state) {
    stack.push_back(state);
    // Over capacity: forget the oldest snapshot, keep the newest
    if (stack.size() > capacity) stack.pop_front();
}

void SceneHistory::record(const WorldState& current) {
    push(undoStack, current);
    redoStack.clear();
}

bool SceneHistory::undo(WorldState& current) {
    if (undoStack.empty()) return false;

    push(redoStack, current);
    current = undoStack.back();
    undoStack.pop_back();
    return true;
}

bool SceneHistory::redo(WorldState& current) {
    if (redoStack.empty()) return false;

    push(undoStack, current);
    current = redoStack.back();
    redoStack.pop_back();
    return true;
}

void SceneHistory::clear() {
    undoStack.clear();
    redoStack.clear();
}
