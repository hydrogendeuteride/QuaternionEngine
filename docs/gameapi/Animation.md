# Animation System

Control glTF skeletal animations and per-node pose offsets for instances.

## Animation Playback

### Set Animation by Index

```cpp
// Play first animation, reset to start
api.set_instance_animation("player", 0, true);

// Continue from current time
api.set_instance_animation("player", 1, false);

// Disable animation (freeze pose)
api.set_instance_animation("player", -1);
```

**Parameters:**
- `instanceName` — glTF instance name
- `animationIndex` — Animation index in glTF file (-1 to disable)
- `resetTime` — If `true`, restart from time 0

Returns `false` if instance doesn't exist.

### Set Animation by Name

```cpp
api.set_instance_animation("player", "walk", true);
api.set_instance_animation("player", "run", true);
```

Animation names come from the glTF file's animation metadata.

### Looping Control

```cpp
// Enable looping (default)
api.set_instance_animation_loop("player", true);

// Play once and stop
api.set_instance_animation_loop("player", false);
```

### Playback Speed

```cpp
// Normal speed
api.set_instance_animation_speed("player", 1.0f);

// Half speed (slow motion)
api.set_instance_animation_speed("player", 0.5f);

// Double speed
api.set_instance_animation_speed("player", 2.0f);

// Reverse (negative speed)
api.set_instance_animation_speed("player", -1.0f);
```

## Cross-Fade Transitions

Smoothly blend between animations:

```cpp
// Transition by index (1 second blend)
api.transition_instance_animation("player", 1, 1.0f, true);

// Transition by name (0.5 second blend)
api.transition_instance_animation("player", "run", 0.5f, true);
```

**Parameters:**
- `instanceName` — glTF instance name
- `animationIndex`/`animationName` — Target animation
- `blendDurationSeconds` — Cross-fade duration
- `resetTime` — If `true`, target animation starts at time 0

The system automatically blends from the current animation to the target over the specified duration.

## Per-Node Pose Offsets

Layer custom transforms on top of animation data (useful for procedural effects, IK, look-at, etc.):

### Set Node Offset

```cpp
// Rotate a node in local space
glm::mat4 offset = glm::rotate(glm::mat4(1.0f),
                               glm::radians(45.0f),
                               glm::vec3(0.0f, 1.0f, 0.0f));

api.set_instance_node_offset("player", "Head", offset);
```

The offset is applied in the node's local space after animation evaluation.

### Clear Node Offset

```cpp
// Clear offset for specific node
api.clear_instance_node_offset("player", "Head");

// Clear all node offsets for instance
api.clear_all_instance_node_offsets("player");
```

## Animation State

Animation state is **per-instance**:
- Each glTF instance has its own `AnimationState`
- Instances sharing the same glTF asset can play different animations
- Animation time advances automatically via `SceneManager::update_scene()`

## Complete Example

```cpp
GameAPI::Engine api(&engine);

// Spawn character
GameAPI::Transform t;
t.position = glm::vec3(0.0f, 0.0f, -5.0f);
api.add_gltf_instance("player", "characters/player.gltf", t);

// Start idle animation (looping)
api.set_instance_animation("player", "idle", true);
api.set_instance_animation_loop("player", true);

// On movement input: transition to walk
if (moving) {
    api.transition_instance_animation("player", "walk", 0.2f, true);
}

// On sprint: speed up walk animation
if (sprinting) {
    api.set_instance_animation_speed("player", 1.5f);
} else {
    api.set_instance_animation_speed("player", 1.0f);
}

// Add procedural head rotation (look-at)
glm::vec3 lookTarget = get_camera_position();
glm::vec3 headPos = get_node_world_position("player", "Head");
glm::vec3 lookDir = glm::normalize(lookTarget - headPos);

glm::quat lookRotation = glm::quatLookAt(lookDir, glm::vec3(0.0f, 1.0f, 0.0f));
glm::mat4 lookOffset = glm::mat4_cast(lookRotation);

api.set_instance_node_offset("player", "Head", lookOffset);
```

## Use Cases

### Character Animation States

```cpp
enum class CharacterState { Idle, Walk, Run, Jump };

void update_character_animation(CharacterState state) {
    switch (state) {
        case CharacterState::Idle:
            api.transition_instance_animation("player", "idle", 0.3f, true);
            break;
        case CharacterState::Walk:
            api.transition_instance_animation("player", "walk", 0.2f, true);
            break;
        case CharacterState::Run:
            api.transition_instance_animation("player", "run", 0.15f, true);
            break;
        case CharacterState::Jump:
            api.set_instance_animation("player", "jump", true);
            api.set_instance_animation_loop("player", false);  // Play once
            break;
    }
}
```

### Vehicle Control Surfaces

```cpp
// Animate aileron deflection on aircraft
float aileronAngle = input.roll * 30.0f;  // -30 to +30 degrees
glm::mat4 offset = glm::rotate(glm::mat4(1.0f),
                               glm::radians(aileronAngle),
                               glm::vec3(1.0f, 0.0f, 0.0f));

api.set_instance_node_offset("aircraft", "LeftAileron", offset);

// Opposite for right aileron
api.set_instance_node_offset("aircraft", "RightAileron", glm::inverse(offset));
```

### Door Opening

```cpp
// Smoothly open door (no animation, pure procedural)
float doorOpenAmount = lerp(0.0f, 90.0f, openProgress);
glm::mat4 offset = glm::rotate(glm::mat4(1.0f),
                               glm::radians(doorOpenAmount),
                               glm::vec3(0.0f, 1.0f, 0.0f));

api.set_instance_node_offset("building", "FrontDoor", offset);
```

## Notes

- Node names must match glTF node names exactly (case-sensitive)
- Offsets are applied in **local space** after animation evaluation
- Offsets layer on top of animation; they don't replace it
- Use `clear_all_instance_node_offsets()` when switching levels/characters to clean up state

## See Also

- [Scene Management](Scene.md) — Managing instances
- [Picking](Picking.md) — Selecting nodes interactively
