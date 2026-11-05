#include "SceneHistory.hpp"

SceneHistory::SceneHistory(std::size_t historyCapacity)
  : capacity(historyCapacity > 0 ? historyCapacity : 1) {}

void SceneHistory::push(std::deque<WorldState>& historyStack, const WorldState& sceneSnapshot) {
  historyStack.push_back(sceneSnapshot);
  // Over capacity: forget the oldest snapshot, keep the newest
  if (historyStack.size() > capacity) historyStack.pop_front();
}

void SceneHistory::record(const WorldState& currentState) {
  push(undoStack, currentState);
  redoStack.clear();
}

bool SceneHistory::undo(WorldState& currentState) {
  if (undoStack.empty()) return false;

  push(redoStack, currentState);
  currentState = undoStack.back();
  undoStack.pop_back();
  return true;
}

bool SceneHistory::redo(WorldState& currentState) {
  if (redoStack.empty()) return false;

  push(undoStack, currentState);
  currentState = redoStack.back();
  redoStack.pop_back();
  return true;
}

void SceneHistory::clear() {
  undoStack.clear();
  redoStack.clear();
}
