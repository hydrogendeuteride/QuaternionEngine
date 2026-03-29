#include "gameplay_state.h"
#include "orbit_helpers.h"
#include "game/orbit/orbit_prediction_tuning.h"
#include "game/states/gameplay/gameplay_settings.h"
#include "game/states/gameplay/scenario/scenario_loader.h"
#include "game/component/ship_controller.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/orbit_plot/orbit_plot.h"
#include "core/util/logger.h"

#include "imgui.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>

namespace Game
{
    using detail::contact_event_type_name;

    namespace
    {
        const char *prediction_solver_status_label(const OrbitPredictionService::Status status)
        {
            switch (status)
            {
                case OrbitPredictionService::Status::None:
                    return "None";
                case OrbitPredictionService::Status::Success:
                    return "Success";
                case OrbitPredictionService::Status::InvalidInput:
                    return "Invalid input";
                case OrbitPredictionService::Status::InvalidSubject:
                    return "Invalid subject";
                case OrbitPredictionService::Status::InvalidSamplingSpec:
                    return "Invalid sampling spec";
                case OrbitPredictionService::Status::EphemerisUnavailable:
                    return "Ephemeris unavailable";
                case OrbitPredictionService::Status::TrajectorySegmentsUnavailable:
                    return "Segments unavailable";
                case OrbitPredictionService::Status::TrajectorySamplesUnavailable:
                    return "Samples unavailable";
                case OrbitPredictionService::Status::ContinuityFailed:
                    return "Continuity failed";
                case OrbitPredictionService::Status::Cancelled:
                    return "Cancelled";
            }

            return "Unknown";
        }

        const char *prediction_derived_status_label(const PredictionDerivedStatus status)
        {
            switch (status)
            {
                case PredictionDerivedStatus::None:
                    return "None";
                case PredictionDerivedStatus::Success:
                    return "Success";
                case PredictionDerivedStatus::MissingSolverData:
                    return "Missing solver data";
                case PredictionDerivedStatus::MissingEphemeris:
                    return "Missing ephemeris";
                case PredictionDerivedStatus::FrameTransformFailed:
                    return "Frame transform failed";
                case PredictionDerivedStatus::FrameSamplesUnavailable:
                    return "Frame samples unavailable";
                case PredictionDerivedStatus::ContinuityFailed:
                    return "Continuity failed";
                case PredictionDerivedStatus::Cancelled:
                    return "Cancelled";
            }

            return "Unknown";
        }

        const char *prediction_preview_state_label(const PredictionPreviewRuntimeState state)
        {
            switch (state)
            {
                case PredictionPreviewRuntimeState::Idle:
                    return "Idle";
                case PredictionPreviewRuntimeState::EnterDrag:
                    return "EnterDrag";
                case PredictionPreviewRuntimeState::DragPreviewPending:
                    return "DragPreviewPending";
                case PredictionPreviewRuntimeState::PreviewStreaming:
                    return "PreviewStreaming";
                case PredictionPreviewRuntimeState::AwaitFullRefine:
                    return "AwaitFullRefine";
            }

            return "Unknown";
        }

        const char *prediction_solve_quality_label(const OrbitPredictionService::SolveQuality quality)
        {
            switch (quality)
            {
                case OrbitPredictionService::SolveQuality::Full:
                    return "Full";
            }

            return "Unknown";
        }

        const char *prediction_publish_stage_label(const OrbitPredictionService::PublishStage stage)
        {
            switch (stage)
            {
                case OrbitPredictionService::PublishStage::Full:
                    return "Full";
                case OrbitPredictionService::PublishStage::PreviewStreaming:
                    return "PreviewStreaming";
            }

            return "Unknown";
        }

        double timestamp_age_ms(const PredictionDragDebugTelemetry::TimePoint &tp,
                                const PredictionDragDebugTelemetry::TimePoint &now_tp)
        {
            if (!PredictionDragDebugTelemetry::has_time(tp))
            {
                return -1.0;
            }

            return std::chrono::duration<double, std::milli>(now_tp - tp).count();
        }

        void draw_age_text(const char *label, const double age_ms)
        {
            if (age_ms < 0.0)
            {
                ImGui::Text("%s: n/a", label);
                return;
            }

            ImGui::Text("%s: %.1f ms ago", label, age_ms);
        }

        void draw_prediction_stage_diag(const char *label,
                                        const OrbitPredictionService::AdaptiveStageDiagnostics &diag,
                                        const std::size_t sample_count)
        {
            std::string extra{};
            if (diag.frame_resegmentation_count > 0)
            {
                extra = ", reseg " + std::to_string(diag.frame_resegmentation_count);
            }
            ImGui::Text("%s: req %.2f s, cov %.2f s, seg %zu, samples %zu%s%s",
                        label,
                        diag.requested_duration_s,
                        diag.covered_duration_s,
                        diag.accepted_segments,
                        sample_count,
                        diag.cache_reused ? ", cache" : "",
                        diag.hard_cap_hit ? ", cap" : "");
            ImGui::Text("%s dt min/avg/max: %.6f / %.6f / %.6f, rejected %zu, forced %zu%s",
                        label,
                        diag.min_dt_s,
                        diag.avg_dt_s,
                        diag.max_dt_s,
                        diag.rejected_splits,
                        diag.forced_boundary_splits,
                        extra.c_str());
        }

        std::string resolve_asset_rel_path(const GameStateContext &ctx, const std::string &rel_path)
        {
            const std::filesystem::path rel(rel_path);
            if (rel.is_absolute())
            {
                return rel.string();
            }
            if (ctx.renderer && ctx.renderer->_assetManager)
            {
                const AssetPaths &paths = ctx.renderer->_assetManager->paths();
                if (!paths.assets.empty())
                {
                    return (paths.assets / rel).string();
                }
            }
            return rel.string();
        }
    } // anonymous namespace

    GameplaySettings GameplayState::extract_settings() const
    {
        GameplaySettings s{};
        s.prediction_draw_full_orbit = _prediction_draw_full_orbit;
        s.prediction_draw_future_segment = _prediction_draw_future_segment;
        s.prediction_draw_velocity_ray = _prediction_draw_velocity_ray;
        s.prediction_line_alpha_scale = _prediction_line_alpha_scale;
        s.prediction_line_overlay_boost = _prediction_line_overlay_boost;
        s.prediction_periodic_refresh_s = _prediction_periodic_refresh_s;
        s.prediction_thrust_refresh_s = _prediction_thrust_refresh_s;
        s.prediction_sampling_policy = _prediction_sampling_policy;
        s.maneuver_plan_windows = _maneuver_plan_windows;
        s.orbit_plot_budget = _orbit_plot_budget;
        s.debug_draw_enabled = _debug_draw_enabled;
        s.runtime_orbiter_rails_enabled = _runtime_orbiter_rails_enabled;
        s.runtime_orbiter_rails_distance_m = _runtime_orbiter_rails_distance_m;
        s.contact_log_enabled = _contact_log_enabled;
        s.contact_log_print_console = _contact_log_print_console;
        return s;
    }

    void GameplayState::apply_settings(const GameplaySettings &s)
    {
        _prediction_draw_full_orbit = s.prediction_draw_full_orbit;
        _prediction_draw_future_segment = s.prediction_draw_future_segment;
        _prediction_draw_velocity_ray = s.prediction_draw_velocity_ray;
        _prediction_line_alpha_scale = s.prediction_line_alpha_scale;
        _prediction_line_overlay_boost = s.prediction_line_overlay_boost;
        _prediction_periodic_refresh_s = s.prediction_periodic_refresh_s;
        _prediction_thrust_refresh_s = s.prediction_thrust_refresh_s;
        _prediction_sampling_policy = s.prediction_sampling_policy;
        _maneuver_plan_windows = s.maneuver_plan_windows;
        _orbit_plot_budget = s.orbit_plot_budget;
        _debug_draw_enabled = s.debug_draw_enabled;
        _runtime_orbiter_rails_enabled = s.runtime_orbiter_rails_enabled;
        _runtime_orbiter_rails_distance_m = s.runtime_orbiter_rails_distance_m;
        _contact_log_enabled = s.contact_log_enabled;
        _contact_log_print_console = s.contact_log_print_console;
        mark_prediction_dirty();
    }

    void GameplayState::on_draw_ui(GameStateContext &ctx)
    {
        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(viewport->WorkPos.x + 10, viewport->WorkPos.y + 10));
        ImGui::SetNextWindowBgAlpha(0.4f);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize;

        if (ImGui::Begin("##GameplayHUD", nullptr, flags))
        {
            // ----------------------------------------------------------------
            // Player HUD: sim time, warp, vessel, controls
            // ----------------------------------------------------------------
            const double sim_time_s = _orbitsim ? _orbitsim->sim.time_s() : _fixed_time_s;
            const int sim_hours = static_cast<int>(std::floor(sim_time_s / 3600.0));
            const int sim_minutes = static_cast<int>(std::floor(std::fmod(sim_time_s, 3600.0) / 60.0));
            const double sim_seconds = std::fmod(sim_time_s, 60.0);

            const char *warp_mode = "Realtime";
            switch (_time_warp.mode)
            {
                case TimeWarpState::Mode::Realtime:
                    warp_mode = "Realtime";
                    break;
                case TimeWarpState::Mode::PhysicsWarp:
                    warp_mode = "Physics";
                    break;
                case TimeWarpState::Mode::RailsWarp:
                    warp_mode = "Rails";
                    break;
            }

            ImGui::Text("Sim: %dh %dm %.1fs", sim_hours, sim_minutes, sim_seconds);
            const char *controlled_vessel = "None";
            if (const OrbiterInfo *player_orbiter = find_player_orbiter())
            {
                controlled_vessel = player_orbiter->name.c_str();
            }

            ImGui::Text("Warp: x%.0f (%s)  [,][.] change  [/]/[Backspace] x1", _time_warp.factor(), warp_mode);
            ImGui::Text("Vessel: %s", controlled_vessel);
            ImGui::TextUnformatted("Switch vessel: '[' previous, ']' next");
            ImGui::Text("Real: %.1f s", _elapsed);
            ImGui::Text("[ESC] Pause");

#if !(defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT)
            ImGui::Separator();
            ImGui::TextUnformatted(
                "WARNING: Built without Jolt physics (collision test requires VULKAN_ENGINE_USE_JOLT=1).");
#endif

            // ----------------------------------------------------------------
            // Scenario save/load
            // ----------------------------------------------------------------
            if (ImGui::Button("Reset scenario"))
            {
                _reset_requested = true;
            }
            ImGui::SameLine();
            if (ImGui::Button("Replay collision"))
            {
                _reset_requested = true;
            }

            const std::string scenario_slot_path = resolve_asset_rel_path(ctx, _scenario_slot_rel_path);

            ImGui::SameLine();
            if (ImGui::Button("Save scenario slot"))
            {
                if (save_scenario_config(scenario_slot_path, _scenario_config))
                {
                    _scenario_io_status = "Saved scenario: " + scenario_slot_path;
                    _scenario_io_status_ok = true;
                }
                else
                {
                    _scenario_io_status = "Save failed: " + scenario_slot_path;
                    _scenario_io_status_ok = false;
                }
            }

            ImGui::SameLine();
            if (ImGui::Button("Load scenario slot"))
            {
                if (auto loaded = load_scenario_config(scenario_slot_path))
                {
                    _scenario_config = std::move(*loaded);
                    _scenario_io_status = "Loaded scenario: " + scenario_slot_path;
                    _scenario_io_status_ok = true;
                    _reset_requested = true;
                }
                else
                {
                    _scenario_io_status = "Load failed: " + scenario_slot_path;
                    _scenario_io_status_ok = false;
                }
            }

            ImGui::Text("Scenario slot: %s", scenario_slot_path.c_str());
            if (!_scenario_io_status.empty())
            {
                if (_scenario_io_status_ok)
                {
                    ImGui::TextUnformatted(_scenario_io_status.c_str());
                }
                else
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", _scenario_io_status.c_str());
                }
            }

            // ----------------------------------------------------------------
            // Settings save/load
            // ----------------------------------------------------------------
            const std::string settings_path = resolve_asset_rel_path(ctx, _settings_rel_path);
            if (ImGui::Button("Save settings"))
            {
                if (save_gameplay_settings(settings_path, extract_settings()))
                {
                    _settings_io_status = "Saved: " + settings_path;
                    _settings_io_status_ok = true;
                }
                else
                {
                    _settings_io_status = "Save failed: " + settings_path;
                    _settings_io_status_ok = false;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Load settings"))
            {
                if (auto loaded = load_gameplay_settings(settings_path))
                {
                    apply_settings(*loaded);
                    if (ctx.api)
                    {
                        ctx.api->set_debug_draw_enabled(_debug_draw_enabled);
                    }
                    _settings_io_status = "Loaded: " + settings_path;
                    _settings_io_status_ok = true;
                }
                else
                {
                    _settings_io_status = "Load failed: " + settings_path;
                    _settings_io_status_ok = false;
                }
            }
            if (!_settings_io_status.empty())
            {
                if (_settings_io_status_ok)
                {
                    ImGui::TextUnformatted(_settings_io_status.c_str());
                }
                else
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", _settings_io_status.c_str());
                }
            }

            // ----------------------------------------------------------------
            // Debug toggles
            // ----------------------------------------------------------------
            ImGui::Checkbox("Contact log", &_contact_log_enabled);
            ImGui::SameLine();
            ImGui::Checkbox("Print console", &_contact_log_print_console);

            if (ctx.api)
            {
                if (ImGui::Checkbox("Debug draw", &_debug_draw_enabled))
                {
                    ctx.api->set_debug_draw_enabled(_debug_draw_enabled);
                }
            }

            if (ImGui::Checkbox("Runtime orbiter rails", &_runtime_orbiter_rails_enabled))
            {
                mark_prediction_dirty();
            }
            double runtime_rails_distance_m = _runtime_orbiter_rails_distance_m;
            if (ImGui::DragScalar("Runtime rails distance (m)",
                                  ImGuiDataType_Double,
                                  &runtime_rails_distance_m,
                                  100.0f,
                                  nullptr,
                                  nullptr,
                                  "%.0f"))
            {
                _runtime_orbiter_rails_distance_m = std::max(0.0, runtime_rails_distance_m);
            }

            // ----------------------------------------------------------------
            // Contact log
            // ----------------------------------------------------------------
            ImGui::Separator();
            ImGui::Text("Contacts: %zu", _contact_log.size());

            const int max_lines = 6;
            const int n = static_cast<int>(std::min(_contact_log.size(), static_cast<size_t>(max_lines)));
            for (int i = 0; i < n; ++i)
            {
                const ContactLogEntry &e = _contact_log[_contact_log.size() - 1 - static_cast<size_t>(i)];
                ImGui::Text("[%s][%.2fs] self=%u other=%u depth=%.3f p=(%.2f,%.2f,%.2f)",
                            contact_event_type_name(e.type),
                            e.time_s,
                            e.self_body,
                            e.other_body,
                            e.penetration_depth,
                            e.point.x, e.point.y, e.point.z);
            }

            // ----------------------------------------------------------------
            // Ship controller HUD
            // ----------------------------------------------------------------
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            {
                const EntityId player_eid = player_entity();
                if (player_eid.is_valid())
                {
                    Entity *player = _world.entities().find(player_eid);
                    if (player)
                    {
                        auto *sc = player->get_component<ShipController>();
                        if (sc)
                        {
                            ImGui::Separator();
                            const bool rails_warp = _rails_warp_active && _time_warp.mode == TimeWarpState::Mode::RailsWarp;
                            const glm::vec3 td = rails_warp ? _rails_last_thrust_dir_local : sc->last_thrust_dir();
                            ImGui::Text("SAS: %s  [T] toggle", sc->sas_enabled() ? "ON " : "OFF");
                            ImGui::Text("Thrust input: (%.1f, %.1f, %.1f)%s",
                                        td.x, td.y, td.z,
                                        (rails_warp && _rails_thrust_applied_this_tick) ? " [applied]" : "");

                            if (rails_warp)
                            {
                                WorldVec3 ship_pos_world{0.0, 0.0, 0.0};
                                glm::dvec3 ship_vel_world(0.0);
                                glm::vec3 ship_vel_local_f(0.0f);
                                if (get_player_world_state(ship_pos_world, ship_vel_world, ship_vel_local_f))
                                {
                                    ImGui::Text("Speed(world): %.2f m/s", glm::length(ship_vel_world));
                                }
                            }
                            else if (player->has_physics() && _physics)
                            {
                                const Physics::BodyId body_id{player->physics_body_value()};
                                if (_physics->is_body_valid(body_id))
                                {
                                    const Physics::MotionType motion = _physics->get_motion_type(body_id);
                                    const char *motion_str =
                                            (motion == Physics::MotionType::Dynamic)
                                                ? "Dynamic"
                                                : (motion == Physics::MotionType::Kinematic) ? "Kinematic (forces ignored)" : "Static";
                                    ImGui::Text("Motion: %s", motion_str);

                                    float thrust = sc->thrust_force();
                                    if (ImGui::DragFloat("Thrust force (N)", &thrust, 1000.0f, 0.0f, 1.0e9f, "%.1f"))
                                    {
                                        sc->set_thrust_force(thrust);
                                    }

                                    float torque = sc->torque_strength();
                                    if (ImGui::DragFloat("Torque strength (N*m)", &torque, 1000.0f, 0.0f, 1.0e9f, "%.1f"))
                                    {
                                        sc->set_torque_strength(torque);
                                    }

                                    float sas = sc->sas_damping();
                                    if (ImGui::DragFloat("SAS damping", &sas, 0.1f, 0.0f, 1.0e4f, "%.2f"))
                                    {
                                        sc->set_sas_damping(sas);
                                    }

                                    const glm::vec3 vel = _physics->get_linear_velocity(body_id);
                                    ImGui::Text("Speed(local): %.2f m/s", glm::length(vel));
                                    if (_physics_context)
                                    {
                                        const glm::dvec3 v_world = _physics_context->velocity_origin_world() + glm::dvec3(vel);
                                        ImGui::Text("Speed(world): %.2f m/s", glm::length(v_world));
                                    }
                                }
                            }
                        }
                    }
                }
            }
#endif

            // ================================================================
            // Orbit section
            // ================================================================
            ImGui::Separator();
            if (ImGui::CollapsingHeader("Orbit", ImGuiTreeNodeFlags_DefaultOpen))
            {
                OrbitPlotSystem *orbit_plot =
                        (ctx.renderer && ctx.renderer->_context) ? ctx.renderer->_context->orbit_plot : nullptr;
                rebuild_prediction_subjects();
                rebuild_prediction_frame_options();
                rebuild_prediction_analysis_options();

                // --- Key orbital info (always visible) ---
                const PredictionTrackState *active_prediction = active_prediction_track();
                std::string active_prediction_label = active_prediction
                                                            ? prediction_subject_label(active_prediction->key)
                                                            : std::string("None");
                ImGui::Text("Focused subject: %s", active_prediction_label.c_str());

                // Display frame / Analysis frame combos
                const char *frame_label = (_prediction_frame_selection.selected_index >= 0 &&
                                           _prediction_frame_selection.selected_index <
                                                   static_cast<int>(_prediction_frame_selection.options.size()))
                                              ? _prediction_frame_selection.options[static_cast<size_t>(
                                                        _prediction_frame_selection.selected_index)].label.c_str()
                                              : "Unknown";
                if (ImGui::BeginCombo("Display frame", frame_label))
                {
                    for (std::size_t i = 0; i < _prediction_frame_selection.options.size(); ++i)
                    {
                        const PredictionFrameOption &option = _prediction_frame_selection.options[i];
                        const bool selected =
                                option.spec.type == _prediction_frame_selection.spec.type &&
                                option.spec.primary_body_id == _prediction_frame_selection.spec.primary_body_id &&
                                option.spec.secondary_body_id == _prediction_frame_selection.spec.secondary_body_id &&
                                option.spec.target_spacecraft_id == _prediction_frame_selection.spec.target_spacecraft_id;
                        if (ImGui::Selectable(option.label.c_str(), selected))
                        {
                            (void) set_prediction_frame_spec(option.spec);
                        }
                        if (selected)
                        {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }

                const char *analysis_label =
                        (_prediction_analysis_selection.selected_index >= 0 &&
                         _prediction_analysis_selection.selected_index <
                                 static_cast<int>(_prediction_analysis_selection.options.size()))
                            ? _prediction_analysis_selection
                                      .options[static_cast<size_t>(_prediction_analysis_selection.selected_index)]
                                      .label.c_str()
                            : "Unknown";
                if (ImGui::BeginCombo("Analysis frame", analysis_label))
                {
                    for (std::size_t i = 0; i < _prediction_analysis_selection.options.size(); ++i)
                    {
                        const PredictionAnalysisOption &option = _prediction_analysis_selection.options[i];
                        const bool selected =
                                option.spec.mode == _prediction_analysis_selection.spec.mode &&
                                option.spec.fixed_body_id == _prediction_analysis_selection.spec.fixed_body_id;
                        if (ImGui::Selectable(option.label.c_str(), selected))
                        {
                            (void) set_prediction_analysis_spec(option.spec);
                        }
                        if (selected)
                        {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }

                // Orbital metrics
                WorldVec3 subject_pos_world{0.0, 0.0, 0.0};
                glm::dvec3 subject_vel_world(0.0);
                glm::vec3 subject_vel_local_f(0.0f);

                active_prediction = active_prediction_track();
                active_prediction_label = active_prediction
                                              ? prediction_subject_label(active_prediction->key)
                                              : std::string("None");
                const bool have_subject =
                        active_prediction &&
                        get_prediction_subject_world_state(active_prediction->key,
                                                           subject_pos_world,
                                                           subject_vel_world,
                                                           subject_vel_local_f);
                if (!have_subject)
                {
                    ImGui::TextUnformatted("Prediction subject state unavailable.");
                }
                else
                {
                    if (active_prediction && active_prediction->cache.valid)
                    {
                        if (!active_prediction->cache.altitude_km.empty())
                        {
                            ImGui::Text("Altitude: %.0f m", static_cast<double>(active_prediction->cache.altitude_km.front()) * 1000.0);
                        }
                        if (!active_prediction->cache.speed_kmps.empty())
                        {
                            ImGui::Text("Speed:    %.3f km/s", static_cast<double>(active_prediction->cache.speed_kmps.front()));
                        }

                        if (!active_prediction->is_celestial)
                        {
                            ImGui::Text("Predicted Pe: %.1f km", active_prediction->cache.periapsis_alt_km);
                            if (std::isfinite(active_prediction->cache.apoapsis_alt_km))
                            {
                                ImGui::Text("Predicted Ap: %.1f km", active_prediction->cache.apoapsis_alt_km);
                            }
                            else
                            {
                                ImGui::TextUnformatted("Predicted Ap: escape");
                            }

                            if (active_prediction->cache.orbital_period_s > 0.0 &&
                                std::isfinite(active_prediction->cache.orbital_period_s))
                            {
                                ImGui::Text("Predicted Period: %.2f min", active_prediction->cache.orbital_period_s / 60.0);
                            }
                        }
                    }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
                    const EntityId player_eid = player_entity();
                    if (_physics && _physics_context && player_eid.is_valid() &&
                        active_prediction &&
                        prediction_subject_is_player(active_prediction->key))
                    {
                        const glm::dvec3 v_origin_world = _physics_context->velocity_origin_world();
                        ImGui::Text("v_origin: %.1f, %.1f, %.1f m/s", v_origin_world.x, v_origin_world.y,
                                    v_origin_world.z);
                        ImGui::Text("v_local:  %.2f, %.2f, %.2f m/s", subject_vel_local_f.x, subject_vel_local_f.y,
                                    subject_vel_local_f.z);

                        const Entity *player = _world.entities().find(player_eid);
                        if (player && player->has_physics())
                        {
                            const Physics::BodyId body_id{player->physics_body_value()};
                            if (_physics->is_body_valid(body_id))
                            {
                                const glm::vec3 w_local_f = _physics->get_angular_velocity(body_id);
                                ImGui::Text("w_local:  %.3f, %.3f, %.3f rad/s (|w|=%.3f)",
                                            w_local_f.x, w_local_f.y, w_local_f.z, glm::length(w_local_f));
                            }
                        }
                    }
#endif
                }

                // --- Orbit View (collapsed by default) ---
                if (ImGui::CollapsingHeader("Orbit View"))
                {
                    ImGui::Checkbox("Prediction full orbit", &_prediction_draw_full_orbit);
                    ImGui::Checkbox("Prediction future segment", &_prediction_draw_future_segment);
                    ImGui::Checkbox("Prediction velocity ray", &_prediction_draw_velocity_ray);

                    float prediction_alpha_scale = _prediction_line_alpha_scale;
                    if (ImGui::DragFloat("Prediction line alpha scale",
                                         &prediction_alpha_scale,
                                         0.05f,
                                         0.1f,
                                         8.0f,
                                         "%.2f"))
                    {
                        _prediction_line_alpha_scale = std::clamp(prediction_alpha_scale, 0.1f, 8.0f);
                    }

                    float prediction_overlay_boost = _prediction_line_overlay_boost;
                    if (ImGui::DragFloat("Prediction line overlay boost",
                                         &prediction_overlay_boost,
                                         0.01f,
                                         0.0f,
                                         1.0f,
                                         "%.2f"))
                    {
                        _prediction_line_overlay_boost = std::clamp(prediction_overlay_boost, 0.0f, 1.0f);
                    }
                    ImGui::SameLine();
                    ImGui::TextUnformatted("(0 = depth-only)");
                }

                // --- Orbit Debug (collapsed by default) ---
                if (ImGui::CollapsingHeader("Orbit Debug"))
                {
                    ImGui::TextUnformatted("Planner preview / solve margin live in Maneuver Nodes.");

                    float refresh_s = static_cast<float>(_prediction_periodic_refresh_s);
                    if (ImGui::DragFloat("Prediction refresh (s)", &refresh_s, 1.0f, 0.0f, 36000.0f, "%.1f"))
                    {
                        _prediction_periodic_refresh_s = static_cast<double>(std::max(0.0f, refresh_s));
                    }
                    ImGui::SameLine();
                    ImGui::TextUnformatted("(0 = never)");

                    float thrust_refresh_s = static_cast<float>(_prediction_thrust_refresh_s);
                    if (ImGui::DragFloat("Prediction thrust refresh (s)",
                                         &thrust_refresh_s,
                                         0.01f,
                                         0.0f,
                                         2.0f,
                                         "%.2f"))
                    {
                        _prediction_thrust_refresh_s = static_cast<double>(std::max(0.0f, thrust_refresh_s));
                    }
                    ImGui::SameLine();
                    ImGui::TextUnformatted("(0 = every fixed tick)");

                    ImGui::SeparatorText("Prediction Policy");

                    float orbiter_min_window_s = static_cast<float>(_prediction_sampling_policy.orbiter_min_window_s);
                    if (ImGui::DragFloat("Orbiter min window (s)",
                                         &orbiter_min_window_s,
                                         10.0f,
                                         0.0f,
                                         15552000.0f,
                                         "%.0f"))
                    {
                        _prediction_sampling_policy.orbiter_min_window_s =
                                static_cast<double>(std::max(0.0f, orbiter_min_window_s));
                    }

                    float celestial_min_window_s = static_cast<float>(_prediction_sampling_policy.celestial_min_window_s);
                    if (ImGui::DragFloat("Celestial min window (s)",
                                         &celestial_min_window_s,
                                         60.0f,
                                         0.0f,
                                         15552000.0f,
                                         "%.0f"))
                    {
                        _prediction_sampling_policy.celestial_min_window_s =
                                static_cast<double>(std::max(0.0f, celestial_min_window_s));
                    }

                    ImGui::Text("Plan preview: %.0f s", _maneuver_plan_windows.preview_window_s);
                    ImGui::Text("Plan solve margin: %.0f s", _maneuver_plan_windows.solve_margin_s);

                    ImGui::SeparatorText("Orbit Budget");

                    float render_error_px = static_cast<float>(_orbit_plot_budget.render_error_px);
                    if (ImGui::DragFloat("Render error (px)", &render_error_px, 0.01f, 0.05f, 4.0f, "%.2f"))
                    {
                        _orbit_plot_budget.render_error_px = std::clamp(static_cast<double>(render_error_px), 0.05, 4.0);
                    }

                    int render_max_segments_cpu = _orbit_plot_budget.render_max_segments_cpu;
                    if (ImGui::DragInt("Render max segments (CPU)",
                                       &render_max_segments_cpu,
                                       50.0f,
                                       64,
                                       200000))
                    {
                        _orbit_plot_budget.render_max_segments_cpu = std::clamp(render_max_segments_cpu, 64, 200000);
                    }

                    int pick_max_segments = _orbit_plot_budget.pick_max_segments;
                    if (ImGui::DragInt("Pick max segments", &pick_max_segments, 50.0f, 64, 32000))
                    {
                        _orbit_plot_budget.pick_max_segments = std::clamp(pick_max_segments, 64, 32000);
                    }

                    float pick_margin_ratio = static_cast<float>(_orbit_plot_budget.pick_frustum_margin_ratio);
                    if (ImGui::DragFloat("Pick frustum margin ratio",
                                         &pick_margin_ratio,
                                         0.01f,
                                         0.0f,
                                         1.0f,
                                         "%.2f"))
                    {
                        _orbit_plot_budget.pick_frustum_margin_ratio =
                                std::clamp(static_cast<double>(pick_margin_ratio), 0.0, 1.0);
                    }

                    if (orbit_plot)
                    {
                        int upload_budget_mib = static_cast<int>(std::clamp<std::size_t>(
                                orbit_plot->settings().upload_budget_bytes / (1024ull * 1024ull),
                                1ull,
                                256ull));
                        if (ImGui::SliderInt("Orbit upload budget (MiB)", &upload_budget_mib, 1, 256))
                        {
                            orbit_plot->settings().upload_budget_bytes =
                                    static_cast<std::size_t>(upload_budget_mib) * 1024ull * 1024ull;
                        }
                    }
                }

                // --- Orbit Performance (collapsed by default) ---
                if (orbit_plot && ImGui::CollapsingHeader("Orbit Performance"))
                {
                    const OrbitPlotSystem::Stats &plot_stats = orbit_plot->stats();
                    const OrbitPlotPerfStats &perf = _orbit_plot_perf;
                    const PredictionTrackState *active_track = active_prediction_track();
                    const double upload_mib =
                            static_cast<double>(plot_stats.upload_bytes_last_frame) / (1024.0 * 1024.0);
                    const double budget_mib =
                            static_cast<double>(plot_stats.upload_budget_bytes) / (1024.0 * 1024.0);
                    const double peak_mib =
                            static_cast<double>(plot_stats.upload_bytes_peak) / (1024.0 * 1024.0);

                    ImGui::Text("Visible subjects: %zu", _prediction_tracks.size());
                    ImGui::Text("Solver segments (base/planned): %u / %u",
                                perf.solver_segments_base,
                                perf.solver_segments_planned);
                    if (active_track)
                    {
                        const OrbitPredictionService::Diagnostics &solver_diag = active_track->solver_diagnostics;
                        const OrbitPredictionDerivedDiagnostics &derived_diag = active_track->derived_diagnostics;
                        ImGui::Text("Prediction status (solver/frame): %s / %s",
                                    prediction_solver_status_label(solver_diag.status),
                                    prediction_derived_status_label(derived_diag.status));
                        draw_prediction_stage_diag("Ephemeris", solver_diag.ephemeris, 0);
                        draw_prediction_stage_diag(
                                "Solver base",
                                solver_diag.trajectory_base,
                                solver_diag.trajectory_sample_count);
                        if (solver_diag.trajectory_planned.accepted_segments > 0 ||
                            solver_diag.trajectory_sample_count_planned > 0)
                        {
                            draw_prediction_stage_diag(
                                    "Solver planned",
                                    solver_diag.trajectory_planned,
                                    solver_diag.trajectory_sample_count_planned);
                        }
                        draw_prediction_stage_diag(
                                "Frame base",
                                derived_diag.frame_base,
                                derived_diag.frame_sample_count);
                        if (derived_diag.frame_planned.accepted_segments > 0 ||
                            derived_diag.frame_sample_count_planned > 0)
                        {
                            draw_prediction_stage_diag(
                                    "Frame planned",
                                    derived_diag.frame_planned,
                                    derived_diag.frame_sample_count_planned);
                        }
                    }
                    ImGui::Text("Orbit lines (active/pending): %u / %u",
                                plot_stats.active_line_count,
                                plot_stats.pending_line_count);
                    ImGui::Text("Orbit segments (depth/overlay): %u / %u",
                                plot_stats.depth_segment_count,
                                plot_stats.overlay_segment_count);
                    ImGui::Text("Pick segments (before/after): %u / %u",
                                perf.pick_segments_before_cull,
                                perf.pick_segments);

                    ImGui::Text("Timing ms (solver/render_lod/pick_lod/upload): %.3f / %.3f / %.3f / %.3f",
                                perf.solver_ms_last,
                                perf.render_lod_ms_last,
                                perf.pick_lod_ms_last,
                                plot_stats.upload_ms_last_frame);
                    ImGui::Text("Orbit upload: %.2f MiB / %.2f MiB%s",
                                upload_mib,
                                budget_mib,
                                plot_stats.upload_cap_hit_last_frame ? " [cap]" : "");
                    ImGui::Text("Cap hits (render/pick/upload): %llu / %llu / %llu",
                                static_cast<unsigned long long>(perf.render_cap_hits_total),
                                static_cast<unsigned long long>(perf.pick_cap_hits_total),
                                static_cast<unsigned long long>(plot_stats.upload_cap_hits_total));
                    ImGui::Text("Orbit upload peak: %.2f MiB, upload ms peak: %.3f",
                                peak_mib,
                                plot_stats.upload_ms_peak);

                    if (_orbit_plot_perf.planned_window_valid || !_maneuver_state.nodes.empty())
                    {
                        ImGui::Separator();
                        ImGui::Text("Planned window: %s", _orbit_plot_perf.planned_window_valid ? "VALID" : "INVALID");
                        ImGui::Text("  t0p (traj start): %.3f", _orbit_plot_perf.planned_window_t0p);
                        ImGui::Text("  now_s:            %.3f", _orbit_plot_perf.planned_window_now_s);
                        ImGui::Text("  anchor (node):    %.3f", _orbit_plot_perf.planned_window_anchor_s);
                        ImGui::Text("  t_plan_start:     %.3f", _orbit_plot_perf.planned_window_t_start);
                        ImGui::Text("  t_plan_end:       %.3f", _orbit_plot_perf.planned_window_t_end);
                        if (_orbit_plot_perf.planned_window_valid)
                        {
                            ImGui::Text("  offset from t0p:  %.3f s", _orbit_plot_perf.planned_window_t_start - _orbit_plot_perf.planned_window_t0p);
                            ImGui::Text("  offset from now:  %.3f s", _orbit_plot_perf.planned_window_t_start - _orbit_plot_perf.planned_window_now_s);
                        }
                    }
                }

                // --- Physics Debug (collapsed by default) ---
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
                if (_physics && _physics_context && ImGui::CollapsingHeader("Physics Debug"))
                {
                    int mode_idx = (_velocity_origin_mode == VelocityOriginMode::PerStepAnchorSync) ? 0 : 1;
                    const char *modes[] = {"Per-step anchor sync", "Free-fall anchor frame"};
                    if (ImGui::Combo("Velocity origin mode", &mode_idx, modes, IM_ARRAYSIZE(modes)))
                    {
                        _velocity_origin_mode =
                                (mode_idx == 0)
                                    ? VelocityOriginMode::PerStepAnchorSync
                                    : VelocityOriginMode::FreeFallAnchorFrame;
                        mark_prediction_dirty();
                    }

                    GameWorld::RebaseSettings rs = _world.rebase_settings();
                    float v_rebase = static_cast<float>(rs.velocity_threshold_mps);
                    if (ImGui::DragFloat("Velocity rebase threshold (m/s)", &v_rebase, 50.0f, 0.0f, 100000.0f, "%.1f"))
                    {
                        rs.velocity_threshold_mps = static_cast<double>(std::max(0.0f, v_rebase));
                        _world.set_rebase_settings(rs);
                    }
                    ImGui::SameLine();
                    ImGui::TextUnformatted("(0 = off)");

                    const EntityId player_eid = player_entity();
                    if (_physics && player_eid.is_valid())
                    {
                        const Entity *player = _world.entities().find(player_eid);
                        if (player && player->has_physics())
                        {
                            const Physics::BodyId body_id{player->physics_body_value()};
                            if (_physics->is_body_valid(body_id))
                            {
                                const Physics::MotionType motion = _physics->get_motion_type(body_id);
                                bool kinematic = motion == Physics::MotionType::Kinematic;
                                if (ImGui::Checkbox("Controlled vessel kinematic", &kinematic))
                                {
                                    const Physics::MotionType target =
                                            kinematic ? Physics::MotionType::Kinematic : Physics::MotionType::Dynamic;
                                    (void) _physics->set_motion_type(body_id, target);
                                }
                                ImGui::SameLine();
                                ImGui::TextUnformatted("Controlled vessel is also the rebase anchor.");
                            }
                        }
                    }
                }
#endif
            } // end Orbit CollapsingHeader
        }
        ImGui::End();

        draw_maneuver_nodes_panel(ctx);
        draw_maneuver_imgui_gizmo(ctx);
        draw_orbit_drag_debug_window(ctx);
        _frame_monitor.draw_ui();
    }

    void GameplayState::draw_orbit_drag_debug_window(GameStateContext &ctx)
    {
        if (!_maneuver_nodes_enabled)
        {
            return;
        }

        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(
                ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - 440.0f, viewport->WorkPos.y + 16.0f),
                ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(420.0f, 0.0f), ImGuiCond_FirstUseEver);

        if (!ImGui::Begin("Orbit Drag Debug"))
        {
            ImGui::End();
            return;
        }

        const PredictionTrackState *active_track = active_prediction_track();
        if (!active_track)
        {
            ImGui::TextUnformatted("No active prediction track.");
            ImGui::End();
            return;
        }

        const PredictionDragDebugTelemetry &debug = active_track->drag_debug;
        const auto now_tp = PredictionDragDebugTelemetry::Clock::now();
        const std::string subject_label = prediction_subject_label(active_track->key);
        const char *gizmo_state = "Idle";
        switch (_maneuver_gizmo_interaction.state)
        {
            case ManeuverGizmoInteraction::State::Idle:
                gizmo_state = "Idle";
                break;
            case ManeuverGizmoInteraction::State::HoverAxis:
                gizmo_state = "HoverAxis";
                break;
            case ManeuverGizmoInteraction::State::DragAxis:
                gizmo_state = "DragAxis";
                break;
        }

        OrbitPlotSystem *orbit_plot =
                (ctx.renderer && ctx.renderer->_context) ? ctx.renderer->_context->orbit_plot : nullptr;
        const OrbitPlotSystem::Stats *plot_stats = orbit_plot ? &orbit_plot->stats() : nullptr;

        const orbitsim::TrajectoryFrameSpec frame_spec =
                active_track->cache.resolved_frame_spec_valid
                        ? active_track->cache.resolved_frame_spec
                        : _prediction_frame_selection.spec;
        const bool live_chunk_path_supported =
                frame_spec.type != orbitsim::TrajectoryFrameType::Inertial &&
                frame_spec.type != orbitsim::TrajectoryFrameType::LVLH;
        const bool have_sim_now = _orbitsim != nullptr;
        const double sim_now_s = have_sim_now ? _orbitsim->sim.time_s() : 0.0;
        const bool have_build_time = active_track->cache.valid && have_sim_now;
        const double sim_since_build_s = have_build_time ? std::max(0.0, sim_now_s - active_track->cache.build_time_s) : 0.0;
        const double drag_gate_remaining_ms =
                PredictionDragDebugTelemetry::has_time(debug.last_preview_request_tp)
                        ? std::max(0.0,
                                   OrbitPredictionTuning::kDragRebuildMinIntervalS -
                                           std::chrono::duration<double>(now_tp - debug.last_preview_request_tp).count()) *
                                  1000.0
                        : 0.0;

        ImGui::Text("Subject: %s", subject_label.c_str());
        ImGui::Text("Preview state: %s", prediction_preview_state_label(active_track->preview_state));
        ImGui::Text("Result quality/stage: %s / %s",
                    prediction_solve_quality_label(debug.last_result_solve_quality),
                    prediction_publish_stage_label(debug.last_publish_stage));
        ImGui::Text("Pending solver/derived/dirty: %s / %s / %s",
                    active_track->request_pending ? "yes" : "no",
                    active_track->derived_request_pending ? "yes" : "no",
                    active_track->dirty ? "yes" : "no");
        ImGui::Text("Gizmo state: %s", gizmo_state);
        if (_maneuver_gizmo_interaction.node_id >= 0)
        {
            ImGui::Text("Gizmo node/axis: %d / %s",
                        _maneuver_gizmo_interaction.node_id,
                        maneuver_axis_label(_maneuver_gizmo_interaction.axis));
        }

        ImGui::SeparatorText("Cadence");
        ImGui::Text("Drag session / updates / preview requests: %llu / %llu / %llu",
                    static_cast<unsigned long long>(debug.drag_session_id),
                    static_cast<unsigned long long>(debug.drag_update_count),
                    static_cast<unsigned long long>(debug.preview_request_count));
        ImGui::Text("Solver results / derived results / preview publishes: %llu / %llu / %llu",
                    static_cast<unsigned long long>(debug.solver_result_count),
                    static_cast<unsigned long long>(debug.derived_result_count),
                    static_cast<unsigned long long>(debug.preview_publish_count));
        draw_age_text("Drag start", timestamp_age_ms(debug.drag_started_tp, now_tp));
        draw_age_text("Last drag update", timestamp_age_ms(debug.last_drag_update_tp, now_tp));
        draw_age_text("Last preview request", timestamp_age_ms(debug.last_preview_request_tp, now_tp));
        draw_age_text("Last solver result", timestamp_age_ms(debug.last_solver_result_tp, now_tp));
        draw_age_text("Last derived apply", timestamp_age_ms(debug.last_derived_result_tp, now_tp));
        if (PredictionDragDebugTelemetry::has_time(debug.last_drag_end_tp))
        {
            draw_age_text("Last drag end", timestamp_age_ms(debug.last_drag_end_tp, now_tp));
        }
        if (active_track->preview_anchor.valid)
        {
            ImGui::Text("Preview anchor node/time: %d / %.3f s",
                        active_track->preview_anchor.anchor_node_id,
                        active_track->preview_anchor.anchor_time_s);
            ImGui::Text("Preview windows visual/exact/pick/request: %.3f / %.3f / %.3f / %.3f s",
                        active_track->preview_anchor.visual_window_s,
                        active_track->preview_anchor.exact_window_s,
                        active_track->preview_anchor.pick_window_s,
                        active_track->preview_anchor.request_window_s);
        }
        if (have_build_time)
        {
            ImGui::Text("Sim since cache build: %.3f s", sim_since_build_s);
            ImGui::Text("Drag rebuild gate remaining: %.1f ms", drag_gate_remaining_ms);
        }

        ImGui::SeparatorText("Latency");
        ImGui::Text("Drag -> request: %.3f ms (peak %.3f)", debug.drag_to_request_ms_last, debug.drag_to_request_ms_peak);
        ImGui::Text("Request -> solver: %.3f ms (peak %.3f)", debug.request_to_solver_ms_last, debug.request_to_solver_ms_peak);
        ImGui::Text("Request -> derived: %.3f ms (peak %.3f)", debug.request_to_derived_ms_last, debug.request_to_derived_ms_peak);
        ImGui::Text("Solver -> derived: %.3f ms (peak %.3f)", debug.solver_to_derived_ms_last, debug.solver_to_derived_ms_peak);

        ImGui::SeparatorText("Costs");
        ImGui::Text("Drag apply: %.3f ms (peak %.3f)", debug.drag_apply_ms_last, debug.drag_apply_ms_peak);
        ImGui::Text("Solver worker: %.3f ms", active_track->solver_ms_last);
        ImGui::Text("Derived worker total/frame/flatten: %.3f / %.3f / %.3f",
                    debug.derived_worker_ms_last,
                    debug.derived_frame_build_ms_last,
                    debug.derived_flatten_ms_last);
        ImGui::Text("Preview merge/chunk merge/apply: %.3f / %.3f / %.3f",
                    debug.preview_merge_ms_last,
                    debug.chunk_merge_ms_last,
                    debug.derived_apply_ms_last);
        ImGui::Text("Render LOD/chunk enqueue/fallback/pick: %.3f / %.3f / %.3f / %.3f",
                    _orbit_plot_perf.render_lod_ms_last,
                    _orbit_plot_perf.planned_chunk_enqueue_ms_last,
                    _orbit_plot_perf.planned_fallback_draw_ms_last,
                    _orbit_plot_perf.pick_lod_ms_last);
        if (plot_stats)
        {
            ImGui::Text("Orbit upload last/peak: %.3f / %.3f ms",
                        plot_stats->upload_ms_last_frame,
                        plot_stats->upload_ms_peak);
        }

        ImGui::SeparatorText("Scale");
        ImGui::Text("Flattened planned seg/samples: %zu / %zu",
                    debug.flattened_planned_segments_last,
                    debug.flattened_planned_samples_last);
        ImGui::Text("Merged planned segs after preview merge: %zu", debug.planned_segments_after_preview_merge);
        ImGui::Text("Incoming/merged/drawn chunks: %u / %u / %u",
                    debug.incoming_chunk_count_last,
                    debug.merged_chunk_count_last,
                    _orbit_plot_perf.planned_chunks_drawn);
        ImGui::Text("Fallback ranges / pick segs before-after: %u / %u -> %u",
                    _orbit_plot_perf.planned_fallback_range_count,
                    _orbit_plot_perf.pick_segments_before_cull,
                    _orbit_plot_perf.pick_segments);

        ImGui::Separator();
        if (debug.derived_flatten_ms_last > 0.0)
        {
            ImGui::TextWrapped(
                    "Hot path: planned chunk output still flattens into a flat planned cache (%zu segs).",
                    debug.flattened_planned_segments_last);
        }
        if (drag_gate_remaining_ms > 0.0)
        {
            ImGui::TextWrapped(
                    "Throttle note: drag rebuild cadence is currently limited by wall-time since the last preview request.");
        }
        if (!live_chunk_path_supported)
        {
            ImGui::TextWrapped(
                    "Chunk preview reuse is disabled in the current display frame, so live preview falls back to the heavier flat frame-cache path.");
        }

        ImGui::End();
    }
} // namespace Game
