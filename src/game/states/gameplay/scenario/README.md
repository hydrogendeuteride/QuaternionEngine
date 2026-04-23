# Scenario Folder Guide

This folder contains the scenario configuration system used by `GameplayState`: the data model for gameplay scenarios and the JSON serialization layer.

## Folder Structure

```
scenario/
  scenario_config.h        # ScenarioConfig data model (celestials, orbiters, global parameters)
  scenario_loader.h        # public load/save/serialize API
  scenario_loader.cpp      # JSON parsing, validation, and serialization implementation
```

## What Lives Here

### Headers

- `scenario_config.h`
  The `ScenarioConfig` struct and its nested definitions:
  - `CelestialDef` -- massive body definition (name, mass, radius, atmosphere, SOI, terrain assets, orbit distance, prediction overlay color)
  - `OrbiterDef` -- orbiting entity definition (name, orbit altitude, spawn offset/velocity, formation hold, prediction group, primitive type, render scale, physics body settings, player/rebase flags)
  - Global parameters: `speed_scale`, `mu_base` (gravitational parameter), `system_center` (world-space origin)

- `scenario_loader.h`
  Public API for scenario I/O:
  - `load_scenario_config(json_path)` -- loads and validates a JSON file, returns `std::optional<ScenarioConfig>`
  - `serialize_scenario_config(config)` -- converts config to a JSON string
  - `save_scenario_config(json_path, config)` -- writes config to disk

### Implementation Files

- `scenario_loader.cpp`
  Full JSON round-trip implementation using `nlohmann::json`:
  - Typed accessors with path-aware error messages: `json_required<T>`, `json_required_finite<T>`, `json_required_object`, `json_required_array`
  - Vector/quaternion parsing: `parse_vec3`, `parse_dvec3`, `parse_quat`
  - Enum parsing: `parse_primitive_type`, `parse_motion_type`, `parse_collision_shape`
  - Nested struct parsing/serialization: `parse_celestial_def` / `serialize_celestial_def`, `parse_orbiter_def` / `serialize_orbiter_def`, `parse_body_settings` / `serialize_body_settings`
  - Schema versioning: currently `schema_version = 1`

## JSON Schema Overview

The scenario JSON file has this top-level structure:

```json
{
  "schema_version": 1,
  "speed_scale": 1.0,
  "mu_base": 3.986004418e14,
  "system_center": { "x": 1e12, "y": 0, "z": 0 },
  "celestials": [ ... ],
  "orbiters": [ ... ]
}
```

- `celestials[0]` is the reference body (orbit_distance_m = 0).
- Each orbiter references the reference body surface via `orbit_altitude_m`.
- Optional fields on orbiters: `formation_hold_enabled`, `formation_leader`, `formation_slot_lvlh_m`, `prediction_group`, `prediction_orbit_color`.
- Optional fields on celestials: `prediction_orbit_color`.
- `body_settings` on each orbiter carries the full `Physics::BodySettings` (collision shape, motion type, mass, friction, damping, layer, etc.).

## How It Is Called

Scenario loading is initiated during gameplay setup:

- `GameplayState` calls `load_scenario_config()` with a JSON path to populate the scenario data.
- The returned `ScenarioConfig` drives celestial body spawning, orbiter placement, and orbital parameter initialization.
- `save_scenario_config()` enables scenario export for debugging or editing.
- Scenario JSON is required at runtime; there is no compiled fallback scenario anymore.

## If You Want To Change...

- What data a celestial or orbiter stores:
  Start in `scenario_config.h`.

- JSON field names, parsing logic, or validation rules:
  Start in `scenario_loader.cpp`.

- The public load/save API signature:
  Start in `scenario_loader.h`.

- Collision shape types (add a new shape variant):
  Add parsing in `parse_collision_shape` and serialization in `serialize_collision_shape` within `scenario_loader.cpp`.

- Schema version migration:
  Add a version check branch in `load_scenario_config()` within `scenario_loader.cpp`.

- Orbiter spawn semantics:
  Update the comments on `ScenarioConfig::OrbiterDef` in `scenario_config.h` and the load/save logic in `scenario_loader.cpp`.

## Notes About Structure

- Each `.cpp` file is an independent translation unit.
- All JSON helpers are in an anonymous namespace inside `scenario_loader.cpp` -- they are not exposed outside this module.
- Validation is strict: required fields throw on missing/null values with path-aware error messages (e.g., `"root.celestials[0].mass_kg must be > 0"`).
- Errors during load are logged via `Logger` and the function returns `std::nullopt` -- no exceptions escape the public API.
- Round-trip fidelity: `load_scenario_config(path)` followed by `save_scenario_config(path, config)` preserves all data (optional fields are only serialized when present).
