#include "EventQueue.hpp"

// SandboxEvent helpers

SandboxEvent SandboxEvent::spawn(const Body& spawnedBody) {
  SandboxEvent event;
  event.type = EventType::SpawnBody;
  event.body = spawnedBody;
  return event;
}

SandboxEvent SandboxEvent::beginDrag() {
  SandboxEvent event;
  event.type = EventType::BeginDrag;
  return event;
}

SandboxEvent SandboxEvent::reset() {
  SandboxEvent event;
  event.type = EventType::Reset;
  return event;
}

SandboxEvent SandboxEvent::undo() {
  SandboxEvent event;
  event.type = EventType::Undo;
  return event;
}

SandboxEvent SandboxEvent::redo() {
  SandboxEvent event;
  event.type = EventType::Redo;
  return event;
}

SandboxEvent SandboxEvent::preset(int presetNumber) {
  SandboxEvent event;
  event.type = EventType::LoadPreset;
  event.value = presetNumber;
  return event;
}

SandboxEvent SandboxEvent::replaceScene(const WorldState& replacementScene) {
  SandboxEvent event;
  event.type = EventType::ReplaceScene;
  event.scene = replacementScene;
  return event;
}

// EventQueue

bool EventQueue::pop(SandboxEvent& outputEvent) {
  if (events.empty()) return false;
  outputEvent = events.front();
  events.pop();
  return true;
}

void EventQueue::clear() {
  std::queue<SandboxEvent> emptyQueue;
  events.swap(emptyQueue); // std::queue has no clear(); swapping with an empty one frees everything
}
