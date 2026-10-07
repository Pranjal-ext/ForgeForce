#pragma once

#include <cstddef>
#include <queue>

#include "Body.hpp"
#include "WorldState.hpp"

// ============================================================
// Event types that change the scene
// ============================================================
enum class EventType {
    SpawnBody,    // add `body` to the scene
    BeginDrag,    // a drag is starting: remember the scene for undo
    Reset,        // clear the scene back to floor and walls
    Undo,
    Redo,
    LoadPreset,   // replace the scene with preset number `value`
    ReplaceScene  // replace the scene with `scene` (e.g. loaded from a file)
};

// ============================================================
// SandboxEvent
// One requested change. Only the fields relevant to the type
// are used. The static helpers build correctly filled events.
// ============================================================
struct SandboxEvent {
    EventType type = EventType::Reset;
    Body body;
    int value = 0;
    WorldState scene;

    static SandboxEvent spawn(const Body& body);
    static SandboxEvent beginDrag();
    static SandboxEvent reset();
    static SandboxEvent undo();
    static SandboxEvent redo();
    static SandboxEvent preset(int number);
    static SandboxEvent replaceScene(const WorldState& scene);
};

// ============================================================
// EventQueue
// First-in, first-out queue of requested changes. Input is
// collected into the queue as it arrives, then processed in
// the same order at one fixed point in the frame, so the scene
// never changes in the middle of a physics step or a draw.
// ============================================================
class EventQueue {
public:
    void push(const SandboxEvent& event) { events.push(event); }

    // Removes the oldest event into `out`; false if the queue is empty
    bool pop(SandboxEvent& out);

    bool empty() const { return events.empty(); }
    std::size_t size() const { return events.size(); }
    void clear();

private:
    std::queue<SandboxEvent> events;
};
