#include "EventQueue.hpp"

// ============================================================
// SandboxEvent helpers
// ============================================================

SandboxEvent SandboxEvent::spawn(const Body& body) {
    SandboxEvent e;
    e.type = EventType::SpawnBody;
    e.body = body;
    return e;
}

SandboxEvent SandboxEvent::beginDrag() {
    SandboxEvent e;
    e.type = EventType::BeginDrag;
    return e;
}

SandboxEvent SandboxEvent::reset() {
    SandboxEvent e;
    e.type = EventType::Reset;
    return e;
}

SandboxEvent SandboxEvent::undo() {
    SandboxEvent e;
    e.type = EventType::Undo;
    return e;
}

SandboxEvent SandboxEvent::redo() {
    SandboxEvent e;
    e.type = EventType::Redo;
    return e;
}

SandboxEvent SandboxEvent::preset(int number) {
    SandboxEvent e;
    e.type = EventType::LoadPreset;
    e.value = number;
    return e;
}

SandboxEvent SandboxEvent::replaceScene(const WorldState& scene) {
    SandboxEvent e;
    e.type = EventType::ReplaceScene;
    e.scene = scene;
    return e;
}

// ============================================================
// EventQueue
// ============================================================

bool EventQueue::pop(SandboxEvent& out) {
    if (events.empty()) return false;
    out = events.front();
    events.pop();
    return true;
}

void EventQueue::clear() {
    std::queue<SandboxEvent> empty;
    events.swap(empty); // std::queue has no clear(); swapping with an empty one frees everything
}
