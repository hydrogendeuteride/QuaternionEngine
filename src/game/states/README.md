# States Folder Guide

This folder contains all concrete `IGameState` implementations for the space combat game, managed by the `GameStateManager` stack in `src/game/state/`.

## Folder Structure

```
states/
  title_screen_state.h / .cpp    # TitleScreenState: main menu (New Game, Load, Settings, Quit)
  pause_state.h / .cpp           # PauseState: overlay pause menu (Resume, Settings, Main Menu, Quit)
  settings_state.h / .cpp        # SettingsState: overlay settings panel (audio, graphics, controls)
  gameplay/                      # GameplayState and all gameplay subsystems (sim, scene, UI, prediction, maneuver, scenario)
```

## What Lives Here

### Headers

- `title_screen_state.h`
  `TitleScreenState` class, derived from `IGameState`. Main menu state with no fixed update. Provides buttons for New Game (switches to `GameplayState`), Load Game (placeholder), Settings (pushes `SettingsState`), and Quit.

- `pause_state.h`
  `PauseState` class, derived from `IGameState`. Overlay state (`is_overlay() = true`) that renders on top of gameplay. Disables fixed update so physics/simulation freezes while paused. Provides Resume, Settings, Main Menu, and Quit buttons.

- `settings_state.h`
  `SettingsState` class, derived from `IGameState`. Overlay state that renders on top of its caller (title screen or pause menu). Holds local copies of audio volume settings (`_master_volume`, `_sfx_volume`, `_bgm_volume`) that are applied immediately via `GameStateContext::audio`.

### Implementation Files

- `title_screen_state.cpp`
  ImGui-based main menu: centered window with title text, New Game button (switches to `GameplayState`), disabled Load Game button, Settings button (pushes `SettingsState`), and Quit button (`ctx.quit()`).

- `pause_state.cpp`
  Dimmed fullscreen overlay plus centered pause menu. ESC key resumes (pops state). Buttons: Resume (pop), Settings (push `SettingsState`), Main Menu (switch to `TitleScreenState`), Quit (`ctx.quit()`).

- `settings_state.cpp`
  Settings panel with collapsible sections. Audio section: master/SFX/BGM volume sliders applied immediately to `ctx.audio`. Graphics and Controls sections are placeholders. ESC key or Back button pops the state. Loads current audio volumes from `ctx.audio` on enter.

### `gameplay/` Subfolder

The main gameplay state: orbital mechanics simulation, physics, time warp, scene setup, prediction display, maneuver planning, and the gameplay HUD. See `gameplay/README.md`.

## How It Is Called

All states are managed by the `GameStateManager` stack defined in `src/game/state/game_state_manager.h`. State transitions use `StateTransition` factories:

- `StateTransition::switch_to<T>()` -- replaces the current state
- `StateTransition::push<T>()` -- pushes a new state on top of the stack
- `StateTransition::pop()` -- removes the top state

The `GameStateManager` calls `IGameState` lifecycle methods on the active state(s):

- `on_enter(ctx)` / `on_exit(ctx)` -- called on state transitions
- `on_update(ctx, dt)` -- called every frame
- `on_fixed_update(ctx, fixed_dt)` -- called at fixed timestep (only if `wants_fixed_update()` returns true)
- `on_draw_ui(ctx)` -- called every frame after rendering

Overlay states (`is_overlay() = true`) allow the state beneath them to continue rendering. `PauseState` and `SettingsState` are overlays; `TitleScreenState` and `GameplayState` are not.

The initial state is pushed by `SpaceCombatGame` in `src/game/space_combat_game.cpp`.

## If You Want To Change...

- The main menu layout or button actions:
  Start in `title_screen_state.cpp`.

- The pause menu layout, ESC-to-resume binding, or dim overlay:
  Start in `pause_state.cpp`.

- Audio/graphics/controls settings UI or volume apply logic:
  Start in `settings_state.cpp`.

- Which settings fields are stored locally during editing:
  Start in `settings_state.h`.

- The gameplay mode (simulation, scene, prediction, maneuver, HUD):
  Start in `gameplay/` (see `gameplay/README.md`).

- State transition wiring (which state pushes/switches to which):
  Check `_pending = StateTransition::...` assignments in each `.cpp` file.

- The `IGameState` interface or `GameStateManager` itself:
  Those live in `src/game/state/`, not here.

## Notes About Structure

- Each state is a single header/implementation pair. `GameplayState` is the exception, split across multiple files in the `gameplay/` subfolder due to its complexity.
- Overlay states (`PauseState`, `SettingsState`) set `is_overlay() = true` and `wants_fixed_update() = false`, so the parent scene renders underneath while simulation is frozen.
- `TitleScreenState` also returns `wants_fixed_update() = false` since it has no simulation to drive.
- State transitions are deferred: states set `_pending` (inherited from `IGameState`) during their update/draw methods, and `GameStateManager` processes the transition after the current frame completes.
- All UI is rendered via ImGui with centered, fixed-size windows.
- `SettingsState` applies audio changes immediately (no explicit "Apply" button for audio), while Graphics and Controls sections are not yet implemented.
