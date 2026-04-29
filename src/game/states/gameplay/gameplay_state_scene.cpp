#include "gameplay_state.h"
#include "orbit_helpers.h"
#include "game/component/ship_controller.h"
#include "core/engine.h"
#include "core/game_api.h"
#include "core/util/logger.h"
#include "render/passes/auto_exposure.h"
#include "render/passes/tonemap.h"

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
#include "physics/jolt/jolt_physics_world.h"
#endif

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>

namespace Game
{
    using detail::contact_event_type_name;
    using detail::circular_orbit_relative_state_xz;
    using detail::two_body_circular_barycentric_xz;

    namespace
    {
        glm::vec4 celestial_mesh_base_color(const std::string &name)
        {
            if (name == "moon")
            {
                return glm::vec4(0.74f, 0.74f, 0.78f, 1.0f);
            }
            return glm::vec4(0.86f, 0.86f, 0.88f, 1.0f);
        }

        void update_orbiter_spawn_from_player_orbit(ScenarioConfig::OrbiterDef &orbiter_def,
                                                    const WorldVec3 &player_pos_world,
                                                    const glm::dvec3 &player_vel_world,
                                                    const WorldVec3 &system_center)
        {
            if (!orbiter_def.derive_spawn_from_player_orbit)
            {
                return;
            }

            const double along_track_offset_m = orbiter_def.player_orbit_along_track_offset_m;
            if (std::abs(along_track_offset_m) <= 1e-6)
            {
                return;
            }

            const glm::dvec3 player_rel_pos = glm::dvec3(player_pos_world - system_center);
            const glm::dvec3 player_orbit_normal = glm::cross(player_rel_pos, player_vel_world);
            const double player_orbit_radius_m = glm::length(player_rel_pos);
            const double player_orbit_normal_len = glm::length(player_orbit_normal);

            if (player_orbit_radius_m <= 1.0 || player_orbit_normal_len <= 1e-6)
            {
                return;
            }

            const double phase_offset_rad = along_track_offset_m / player_orbit_radius_m;
            const glm::dquat phase_rotation = glm::angleAxis(
                    phase_offset_rad,
                    player_orbit_normal / player_orbit_normal_len);
            const glm::dvec3 demo_rel_pos = phase_rotation * player_rel_pos;
            const glm::dvec3 demo_vel_world = phase_rotation * player_vel_world;

            orbiter_def.offset_from_player = demo_rel_pos - player_rel_pos;
            orbiter_def.relative_velocity = demo_vel_world - player_vel_world;
        }

        void update_orbiter_spawn_from_authored_circular_orbit(ScenarioConfig::OrbiterDef &orbiter_def,
                                                               const WorldVec3 &player_pos_world,
                                                               const glm::dvec3 &player_vel_world,
                                                               const WorldVec3 &system_center,
                                                               const double gravitational_constant,
                                                               const double central_mass_kg,
                                                               const double central_body_radius_m)
        {
            if (orbiter_def.is_player || orbiter_def.derive_spawn_from_player_orbit)
            {
                return;
            }

            const double offset_len2 =
                    glm::dot(orbiter_def.offset_from_player, orbiter_def.offset_from_player);
            const double rel_vel_len2 =
                    glm::dot(orbiter_def.relative_velocity, orbiter_def.relative_velocity);
            if (offset_len2 > 1e-12 || rel_vel_len2 > 1e-12)
            {
                return;
            }

            const double orbit_radius_m = central_body_radius_m + orbiter_def.orbit_altitude_m;
            if (!(orbit_radius_m > central_body_radius_m) ||
                !(central_mass_kg > 0.0) ||
                !std::isfinite(gravitational_constant))
            {
                return;
            }

            const glm::dvec3 player_rel_pos = glm::dvec3(player_pos_world - system_center);
            const glm::dvec3 player_orbit_normal = glm::cross(player_rel_pos, player_vel_world);
            const double player_rel_radius_m = glm::length(player_rel_pos);
            const double player_orbit_normal_len = glm::length(player_orbit_normal);
            if (player_rel_radius_m <= 1.0 || player_orbit_normal_len <= 1e-6)
            {
                return;
            }

            const glm::dvec3 radial_dir = player_rel_pos / player_rel_radius_m;
            const glm::dvec3 orbit_normal_dir = player_orbit_normal / player_orbit_normal_len;
            const glm::dvec3 tangential_dir = glm::normalize(glm::cross(orbit_normal_dir, radial_dir));
            if (!std::isfinite(tangential_dir.x) || !std::isfinite(tangential_dir.y) || !std::isfinite(tangential_dir.z))
            {
                return;
            }

            const double mu = gravitational_constant * central_mass_kg;
            const double circular_speed_mps = std::sqrt(mu / orbit_radius_m);
            const glm::dvec3 target_rel_pos = radial_dir * orbit_radius_m;
            const glm::dvec3 target_rel_vel = tangential_dir * circular_speed_mps;

            orbiter_def.offset_from_player = target_rel_pos - player_rel_pos;
            orbiter_def.relative_velocity = target_rel_vel - player_vel_world;
        }
    } // namespace

    Physics::BodyId GameplayState::create_orbiter_physics_body(const bool render_is_gltf,
                                                               Entity &entity,
                                                               const Physics::BodySettings &settings_template,
                                                               const WorldVec3 &position_world,
                                                               const glm::quat &rotation,
                                                               glm::vec3 *out_origin_offset_local)
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!_physics || !_physics_context)
        {
            return {};
        }

        if (out_origin_offset_local)
        {
            *out_origin_offset_local = glm::vec3(0.0f);
        }

        const uint64_t user_data = static_cast<uint64_t>(entity.id().value);

        if (render_is_gltf)
        {
            if (!_renderer || !_renderer->_sceneManager || entity.render_name().empty())
            {
                Logger::error("[GameplayState] Cannot create glTF collider body for '{}'", entity.name());
                return {};
            }

            (void) _renderer->_sceneManager->setGLTFInstanceTRSWorld(entity.render_name(),
                                                                     position_world,
                                                                     rotation,
                                                                     entity.scale());

            std::optional<float> mass_override;
            if (settings_template.has_explicit_mass &&
                std::isfinite(settings_template.mass) &&
                settings_template.mass > 0.0f)
            {
                mass_override = settings_template.mass;
            }

            Physics::BodyId body_id = _renderer->_sceneManager->enableDynamicRootColliderBody(entity.render_name(),
                                                                                              _physics.get(),
                                                                                              settings_template.layer,
                                                                                              user_data,
                                                                                              mass_override);
            if (!body_id.is_valid())
            {
                return {};
            }

            if (out_origin_offset_local)
            {
                glm::vec3 origin_offset_local{0.0f, 0.0f, 0.0f};
                if (_renderer->_sceneManager->getDynamicRootColliderCenterOfMassLocal(entity.render_name(), origin_offset_local))
                {
                    *out_origin_offset_local = origin_offset_local;
                }
            }

            _physics->set_gravity_scale(body_id, settings_template.gravity_scale);
            if (settings_template.motion_type != Physics::MotionType::Dynamic)
            {
                (void) _physics->set_motion_type(body_id, settings_template.motion_type);
            }
            if (!settings_template.start_active)
            {
                _physics->deactivate(body_id);
            }
            entity.set_render_sync_mode(Entity::RenderSyncMode::Authoritative);
            return body_id;
        }

        Physics::BodySettings settings = settings_template;
        settings.position = world_to_local_d(position_world, _physics_context->origin_world());
        settings.rotation = rotation;
        settings.user_data = user_data;
        return _physics->create_body(settings);
#else
        (void) render_is_gltf;
        (void) entity;
        (void) settings_template;
        (void) position_world;
        (void) rotation;
        (void) out_origin_offset_local;
        return {};
#endif
    }

    bool GameplayState::destroy_orbiter_physics_body(const bool render_is_gltf, Entity &entity)
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!_physics || !entity.has_physics())
        {
            return false;
        }

        const Physics::BodyId body_id{entity.physics_body_value()};
        bool destroyed = false;

        if (render_is_gltf && _renderer && _renderer->_sceneManager && !entity.render_name().empty())
        {
            destroyed = _renderer->_sceneManager->disableDynamicRootColliderBody(entity.render_name());
        }

        if (!destroyed && _physics->is_body_valid(body_id))
        {
            _physics->destroy_body(body_id);
            destroyed = true;
        }

        entity.clear_physics_body();
        if (render_is_gltf)
        {
            entity.set_render_sync_mode(Entity::RenderSyncMode::Interpolated);
        }
        return destroyed;
#else
        (void) render_is_gltf;
        (void) entity;
        return false;
#endif
    }

    // ---- Scene setup ----

    void GameplayState::setup_scene(GameStateContext &ctx)
    {
        _elapsed = 0.0f;
        _fixed_time_s = 0.0;
        reset_time_warp_state();
        _warp_to_time_active = false;
        _warp_to_time_target_s = 0.0;
        _warp_to_time_restore_level = 0;
        _execute_node_armed = false;
        _execute_node_id = -1;
        _maneuver_state.nodes.clear();
        _maneuver_state.selected_node_id = -1;
        _maneuver_state.next_node_id = 0;
        _maneuver_gizmo_interaction = {};
        _reset_requested = false;
        _contact_log.clear();
        _prediction.tracks.clear();
        _prediction.groups.clear();
        _prediction.selection.clear();
        _prediction.frame_selection.clear();
        _prediction.dirty = true;
        _prediction.service.reset();
        _prediction.derived_service.reset();

        _world.clear_rebase_anchor();
        _world.clear();

        _orbiters.clear();
        _orbitsim.reset();

        if (ctx.api)
        {
            ctx.api->clear_all_instances();
            ctx.api->clear_planets(true);
        }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (ctx.renderer && ctx.renderer->_context)
        {
            if (ctx.renderer->_context->physics_context == _physics_context.get())
            {
                ctx.renderer->_context->physics_context = nullptr;
            }
        }

        _physics = std::make_unique<Physics::JoltPhysicsWorld>();
        _physics->set_gravity(glm::vec3(0.0f));
        _physics_context = std::make_unique<Physics::PhysicsContext>(*_physics);

        _world.set_physics(_physics.get());
        _world.set_physics_context(_physics_context.get());

        if (ctx.renderer && ctx.renderer->_context)
        {
            ctx.renderer->_context->physics_context = _physics_context.get();
        }
#endif

        if (!ctx.api)
        {
            return;
        }

        // Initialize orbital simulation from config
        WorldVec3 player_pos_world{0.0, 0.0, 0.0};
        glm::dvec3 player_vel_world(0.0);
        init_orbitsim(player_pos_world, player_vel_world);

        setup_environment(ctx);

        // Initialize physics origin near the player to keep local coordinates small.
        if (_physics_context)
        {
            _physics_context->set_origin_world(player_pos_world);
            _physics_context->set_velocity_origin_world(player_vel_world);
        }

        const glm::dvec3 v_origin_world =
                _physics_context ? _physics_context->velocity_origin_world() : glm::dvec3(0.0);

        auto attach_orbiter_attachments = [&](const ScenarioConfig::OrbiterDef &orbiter_def,
                                              Entity &entity) -> bool {
            if (orbiter_def.attachments.empty())
            {
                return true;
            }
            if (!ctx.api || entity.render_name().empty())
            {
                return false;
            }

            for (const auto &attachment_def : orbiter_def.attachments)
            {
                Attachment attachment{};
                attachment.name = attachment_def.name;
                attachment.render_name = entity.render_name() + "_" + attachment_def.name;
                attachment.local_position = attachment_def.local_position;
                attachment.local_rotation = attachment_def.local_rotation;
                attachment.local_scale = attachment_def.local_scale;

                Transform local_transform{};
                local_transform.position_world = WorldVec3(attachment.local_position);
                local_transform.rotation = attachment.local_rotation;
                local_transform.scale = attachment.local_scale;

                const Transform world_transform = entity.transform() * local_transform;

                GameAPI::TransformD api_transform{};
                api_transform.position = glm::dvec3(world_transform.position_world);
                api_transform.rotation = world_transform.rotation;
                api_transform.scale = world_transform.scale;

                bool created = false;
                if (attachment_def.material.has_overrides())
                {
                    GameAPI::PrimitiveMaterial material{};
                    material.albedoPath = attachment_def.material.albedo;
                    material.normalPath = attachment_def.material.normal;
                    material.metalRoughPath = attachment_def.material.metal_rough;
                    material.occlusionPath = attachment_def.material.occlusion;
                    material.emissivePath = attachment_def.material.emissive;
                    material.colorFactor = attachment_def.material.color_factor;
                    material.metallic = attachment_def.material.metallic;
                    material.roughness = attachment_def.material.roughness;
                    created = ctx.api->add_textured_primitive(attachment.render_name,
                                                              attachment_def.primitive,
                                                              material,
                                                              api_transform);
                }
                else
                {
                    created = ctx.api->add_primitive_instance(attachment.render_name,
                                                              attachment_def.primitive,
                                                              api_transform);
                }
                if (!created)
                {
                    return false;
                }

                entity.add_attachment(attachment);
                if (!_world.bind_render_selection(attachment.render_name, entity.name(), attachment.name))
                {
                    return false;
                }
            }

            return true;
        };

        // Spawn orbiter entities from config
        auto spawn_orbiter = [&](const ScenarioConfig::OrbiterDef &orbiter_def,
                                 const WorldVec3 &pos_world,
                                 const glm::vec3 &vel_local_f,
                                 bool is_player,
                                 EntityId &out_id) {
            Transform tr{};
            tr.position_world = pos_world;
            tr.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            tr.scale = orbiter_def.render_scale;

            Entity *ent = nullptr;
            const bool render_is_gltf = !orbiter_def.gltf_path.empty();

            auto builder = _world.builder(orbiter_def.name).transform(tr);
            builder.selection_binding(orbiter_def.name, "body");
            builder.render_sync_mode(Entity::RenderSyncMode::Authoritative);
            if (render_is_gltf)
            {
                builder.render_gltf(orbiter_def.gltf_path);
            }
            else if (orbiter_def.material.has_overrides())
            {
                GameAPI::PrimitiveMaterial mat{};
                mat.albedoPath = orbiter_def.material.albedo;
                mat.normalPath = orbiter_def.material.normal;
                mat.metalRoughPath = orbiter_def.material.metal_rough;
                mat.occlusionPath = orbiter_def.material.occlusion;
                mat.emissivePath = orbiter_def.material.emissive;
                mat.colorFactor = orbiter_def.material.color_factor;
                mat.metallic = orbiter_def.material.metallic;
                mat.roughness = orbiter_def.material.roughness;
                builder.render_textured_primitive(orbiter_def.primitive, mat);
            }
            else
            {
                builder.render_primitive(orbiter_def.primitive);
            }

            ent = builder.build();

            if (!ent)
            {
                out_id = EntityId{};
                return;
            }

            if (!attach_orbiter_attachments(orbiter_def, *ent))
            {
                (void) _world.destroy_entity(ent->id());
                out_id = EntityId{};
                return;
            }

#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
            if (_physics)
            {
                glm::vec3 origin_offset_local{0.0f, 0.0f, 0.0f};
                const Physics::BodyId body_id = create_orbiter_physics_body(render_is_gltf,
                                                                            *ent,
                                                                            orbiter_def.body_settings,
                                                                            pos_world,
                                                                            tr.rotation,
                                                                            &origin_offset_local);
                if (!body_id.is_valid() ||
                    !_world.bind_physics(ent->id(), body_id.value, false, false, origin_offset_local))
                {
                    (void) _world.destroy_entity(ent->id());
                    out_id = EntityId{};
                    return;
                }

                _physics->set_linear_velocity(body_id, vel_local_f);
            }

            if (is_player)
            {
                // Use ShipController defaults so tuning in ship_controller.h (and UI sliders) takes effect.
                ent->add_component<ShipController>();
            }
#endif

            out_id = ent->id();
        };

        // Spawn all orbiters from config
        const auto &cfg = _scenario_config;
        std::vector<ScenarioConfig::OrbiterDef> orbiter_defs = cfg.orbiters;
        const CelestialBodyInfo *ref_info = _orbitsim ? _orbitsim->world_reference_body() : nullptr;
        const double reference_radius_m = ref_info ? ref_info->radius_m : 0.0;
        const double reference_mass_kg = ref_info ? ref_info->mass_kg : 0.0;
        const double gravitational_constant =
                _orbitsim ? _orbitsim->sim.config().gravitational_constant : 0.0;
        for (auto &orbiter_def : orbiter_defs)
        {
            update_orbiter_spawn_from_player_orbit(orbiter_def,
                                                   player_pos_world,
                                                   player_vel_world,
                                                   cfg.system_center);
            update_orbiter_spawn_from_authored_circular_orbit(orbiter_def,
                                                              player_pos_world,
                                                              player_vel_world,
                                                              cfg.system_center,
                                                              gravitational_constant,
                                                              reference_mass_kg,
                                                              reference_radius_m);
        }

        bool primary_player_spawned = false;
        for (const auto &orbiter_def : orbiter_defs)
        {
            const bool is_primary_player = orbiter_def.is_player && !primary_player_spawned;
            WorldVec3 pos_world = player_pos_world + WorldVec3(orbiter_def.offset_from_player);
            glm::dvec3 vel_world = player_vel_world + orbiter_def.relative_velocity;
            if (is_primary_player)
            {
                pos_world = player_pos_world;
                vel_world = player_vel_world;
                primary_player_spawned = true;
            }

            const glm::vec3 vel_local_f = glm::vec3(vel_world - v_origin_world);

            EntityId entity_id{};
            spawn_orbiter(orbiter_def,
                          pos_world,
                          vel_local_f,
                          is_primary_player,
                          entity_id);

            OrbiterInfo info{};
            info.entity = entity_id;
            info.name = orbiter_def.name;
            info.apply_gravity = true;
            info.is_player = is_primary_player;
            info.is_rebase_anchor = orbiter_def.is_rebase_anchor;
            info.render_is_gltf = !orbiter_def.gltf_path.empty();
            info.mass_kg = std::max(0.0, static_cast<double>(orbiter_def.body_settings.mass));
            info.physics_settings = orbiter_def.body_settings;
            info.use_physics_interpolation = false;
            info.formation_hold_enabled = orbiter_def.formation_hold_enabled;
            info.formation_leader_name = orbiter_def.formation_leader;
            info.formation_slot_lvlh_m = orbiter_def.formation_slot_lvlh_m;
            _orbiters.push_back(std::move(info));
        }

        rebuild_prediction_subjects();

        // Configure explicit rebase anchor and camera target.
        {
            // Keep local velocities bounded for Jolt (float velocities) in high-speed scenarios.
            // 0 disables automatic velocity rebasing.
            GameWorld::RebaseSettings rs{};
            rs.velocity_threshold_mps = 2000.0; // 2 km/s local delta-v before rebasing
            _world.set_rebase_settings(rs);
        }
        update_rebase_anchor();

        const EntityId primary_player_eid = player_entity();

        sync_player_collision_callbacks();

        if (primary_player_eid.is_valid() || !_orbiters.empty())
        {
            GameAPI::OrbitCameraSettings orbit{};
            orbit.target.type = GameAPI::CameraTargetType::MeshInstance;
            orbit.distance = 40.0;
            orbit.yaw = 0.6f;
            orbit.pitch = -0.35f;
            orbit.lookSensitivity = 0.0020f;
            ctx.api->set_camera_mode(GameAPI::CameraMode::Orbit);
            ctx.api->set_orbit_camera_settings(orbit);
            sync_player_camera_target(ctx);
        }

        ctx.api->set_debug_draw_enabled(_debug_draw_enabled);
    }

    void GameplayState::setup_environment(GameStateContext &ctx)
    {
        if (ctx.renderer && ctx.renderer->_assetManager)
        {
            const auto &env = _scenario_config.environment;
            GameAPI::IBLPaths ibl{};
            ibl.specularCube = ctx.renderer->_assetManager->assetPath(env.ibl_specular);
            ibl.diffuseCube = ctx.renderer->_assetManager->assetPath(env.ibl_diffuse);
            ibl.brdfLut = ctx.renderer->_assetManager->assetPath(env.ibl_brdf_lut);
            ibl.background = ctx.renderer->_assetManager->assetPath(env.ibl_background);

            const bool same_global_ibl =
                    ctx.renderer->_globalIBLPaths.specularCube == ibl.specularCube &&
                    ctx.renderer->_globalIBLPaths.diffuseCube == ibl.diffuseCube &&
                    ctx.renderer->_globalIBLPaths.brdfLut2D == ibl.brdfLut &&
                    ctx.renderer->_globalIBLPaths.background2D == ibl.background;
            const bool same_pending_ibl =
                    ctx.renderer->_pendingIBLRequest.active &&
                    ctx.renderer->_pendingIBLRequest.targetVolume < 0 &&
                    ctx.renderer->_pendingIBLRequest.paths.specularCube == ibl.specularCube &&
                    ctx.renderer->_pendingIBLRequest.paths.diffuseCube == ibl.diffuseCube &&
                    ctx.renderer->_pendingIBLRequest.paths.brdfLut2D == ibl.brdfLut &&
                    ctx.renderer->_pendingIBLRequest.paths.background2D == ibl.background;

            if (!(same_global_ibl && (same_pending_ibl || (ctx.renderer->_iblManager && ctx.renderer->_iblManager->resident()))))
            {
                ctx.api->load_global_ibl(ibl);
            }
        }

        const auto &cfg = _scenario_config;
        const orbitsim::MassiveBody *ref_sim = _orbitsim ? _orbitsim->world_reference_sim_body() : nullptr;
        const auto celestial_world_position = [&](const std::string &name) {
            WorldVec3 planet_center_world = cfg.system_center;
            if (_orbitsim && ref_sim)
            {
                if (const CelestialBodyInfo *info = _orbitsim->find_body(name))
                {
                    if (const orbitsim::MassiveBody *sim_body = _orbitsim->sim.body_by_id(info->sim_id))
                    {
                        planet_center_world = cfg.system_center +
                                WorldVec3(sim_body->state.position_m - ref_sim->state.position_m);
                    }
                }
            }
            return planet_center_world;
        };

        bool any_planets_added = false;
        bool atmosphere_set = false;

        for (const auto &cdef : cfg.celestials)
        {
            if (cdef.has_terrain)
            {
                GameAPI::PlanetTerrain planet{};
                planet.name = cdef.name;
                planet.center = glm::dvec3(celestial_world_position(cdef.name));
                planet.radius_m = cdef.radius_m;
                planet.visible = true;
                planet.base_color = glm::vec4(1.0f);
                planet.metallic = 0.0f;
                planet.roughness = 1.0f;
                planet.albedo_dir = cdef.albedo_dir;
                planet.height_dir = cdef.height_dir;
                planet.height_max_m = cdef.height_max_m;
                planet.height_offset_m = cdef.height_offset_m;
                planet.detail_normal_dir = cdef.detail_normal_dir;
                planet.detail_normal_strength = cdef.detail_normal_strength;
                planet.cavity_dir = cdef.cavity_dir;
                planet.cavity_strength = cdef.cavity_strength;
                planet.enable_terminator_shadow = cdef.enable_terminator_shadow;
                planet.patch_resolution_override = cdef.patch_resolution_override;
                planet.target_sse_px_override = cdef.target_sse_px_override;
                planet.emission_dir = cdef.emission_dir;
                planet.emission_factor = cdef.emission_factor;
                planet.specular_dir = cdef.specular_dir;
                planet.specular_strength = cdef.specular_strength;
                planet.specular_roughness = cdef.specular_roughness;

                any_planets_added |= ctx.api->add_planet_terrain(planet);

                // Atmosphere for the first terrain body (reference body)
                if (!atmosphere_set)
                {
                    ctx.api->set_atmosphere_enabled(true);
                    ctx.api->reset_atmosphere_to_earth();

                    auto atmo = ctx.api->get_atmosphere_settings();
                    atmo.bodyName = cdef.name;
                    atmo.viewSteps = 32;
                    atmo.lightSteps = 16;
                    atmo.sunHaloIntensity = 0.30f;
                    atmo.sunHaloRadiusDeg = 3.30f;
                    atmo.sunStarburstIntensity = 0.01f;
                    atmo.sunStarburstRadiusDeg = 6.0f;
                    atmo.sunStarburstSpikes = 14;
                    atmo.sunStarburstSharpness = 12.0f;
                    ctx.api->set_atmosphere_settings(atmo);

                    ctx.api->set_planet_clouds_enabled(true);
                    ctx.api->reset_planet_clouds_defaults();
                    auto clouds = ctx.api->get_planet_clouds_settings();
                    ctx.api->set_planet_clouds_settings(clouds);
                    atmosphere_set = true;
                }
                continue;
            }

            GameAPI::PlanetSphere planet{};
            planet.name = cdef.name;
            planet.center = glm::dvec3(celestial_world_position(cdef.name));
            planet.radius_m = cdef.radius_m;
            planet.visible = true;
            planet.base_color = celestial_mesh_base_color(cdef.name);
            planet.metallic = 0.0f;
            planet.roughness = 0.98f;
            planet.sectors = 64;
            planet.stacks = 32;

            any_planets_added |= ctx.api->add_planet_sphere(planet);
        }

        if (any_planets_added)
        {
            ctx.api->set_planet_system_enabled(true);
        }

        if (ctx.renderer && ctx.renderer->_renderPassManager)
        {
            if (auto *tm = ctx.renderer->_renderPassManager->getPass<TonemapPass>())
            {
                if (auto *ae = ctx.renderer->_renderPassManager->getPass<AutoExposurePass>())
                {
                    ae->set_enabled(true, tm->exposure());
                }
            }
        }
    }

    void GameplayState::init_orbitsim(WorldVec3 &player_pos_world, glm::dvec3 &player_vel_world)
    {
        const auto &cfg = _scenario_config;
        if (cfg.celestials.empty())
        {
            return;
        }

        auto scenario = std::make_unique<OrbitalScenario>();

        const double speed_scale = std::max(0.0, cfg.speed_scale);

        orbitsim::GameSimulation::Config sim_cfg;
        sim_cfg.gravitational_constant = orbitsim::kGravitationalConstant_SI * speed_scale * speed_scale;
        sim_cfg.softening_length_m = 0.0;
        sim_cfg.enable_events = false;

        scenario->sim = orbitsim::GameSimulation(sim_cfg);
        scenario->world_reference_body_index = 0;

        // Create all massive bodies in the simulation
        const auto &ref_def = cfg.celestials[0];

        orbitsim::MassiveBody ref_body{};
        ref_body.mass_kg = ref_def.mass_kg;
        ref_body.radius_m = ref_def.radius_m;
        ref_body.atmosphere_top_height_m = ref_def.atmosphere_top_m;
        ref_body.terrain_max_height_m = ref_def.terrain_max_m;
        ref_body.soi_radius_m = ref_def.soi_radius_m;

        // For multiple celestials, set up barycentric states via two-body pairs
        // relative to the reference body.
        std::vector<orbitsim::MassiveBody> sim_bodies;
        sim_bodies.push_back(ref_body);

        for (size_t i = 1; i < cfg.celestials.size(); ++i)
        {
            const auto &cdef = cfg.celestials[i];

            orbitsim::MassiveBody body{};
            body.mass_kg = cdef.mass_kg;
            body.radius_m = cdef.radius_m;
            body.atmosphere_top_height_m = cdef.atmosphere_top_m;
            body.terrain_max_height_m = cdef.terrain_max_m;
            body.soi_radius_m = cdef.soi_radius_m;
            sim_bodies.push_back(body);
        }

        // Initialize states: for each satellite, compute two-body barycentric orbit with reference
        if (sim_bodies.size() >= 2)
        {
            // For simplicity, use pairwise two-body initialization with the reference body.
            // Each satellite is placed relative to the reference body using barycentric states.
            // When there are multiple satellites, each pair (ref, satellite_i) is handled independently.
            for (size_t i = 1; i < sim_bodies.size(); ++i)
            {
                const double sep_m = std::max(ref_def.radius_m * 2.0, cfg.celestials[i].orbit_distance_m);
                const auto bary_init = two_body_circular_barycentric_xz(
                        sim_cfg.gravitational_constant,
                        sim_bodies[0].mass_kg, sim_bodies[i].mass_kg,
                        sep_m, 0.0);

                // Accumulate barycentric offset onto reference body (for multi-satellite systems
                // this is approximate, but good enough for initialization)
                sim_bodies[0].state.position_m += bary_init.state_a.position_m;
                sim_bodies[0].state.velocity_mps += bary_init.state_a.velocity_mps;
                sim_bodies[i].state = bary_init.state_b;
            }
        }

        // Register all bodies in the simulation and build CelestialBodyInfo list
        bool all_valid = true;
        for (size_t i = 0; i < sim_bodies.size(); ++i)
        {
            const auto handle = scenario->sim.create_body(sim_bodies[i]);
            if (!handle.valid())
            {
                all_valid = false;
                break;
            }

            CelestialBodyInfo info{};
            info.sim_id = handle.id;
            info.name = cfg.celestials[i].name;
            info.radius_m = cfg.celestials[i].radius_m;
            info.mass_kg = cfg.celestials[i].mass_kg;
            info.has_terrain = cfg.celestials[i].has_terrain;
            scenario->bodies.push_back(std::move(info));
        }

        if (!all_valid || scenario->bodies.empty())
        {
            _orbitsim = std::move(scenario);
            return;
        }

        // Compute player initial orbit around reference body
        const CelestialBodyInfo *ref_info = scenario->world_reference_body();
        const orbitsim::MassiveBody *ref_sim = scenario->world_reference_sim_body();
        if (ref_info && ref_sim)
        {
            // Find player orbiter def to get orbit altitude
            double player_altitude_m = 400'000.0;
            for (const auto &odef : cfg.orbiters)
            {
                if (odef.is_player)
                {
                    player_altitude_m = odef.orbit_altitude_m;
                    break;
                }
            }

            const double orbit_radius_m = ref_info->radius_m + player_altitude_m;
            const detail::OrbitRelativeState ship_rel =
                    circular_orbit_relative_state_xz(sim_cfg.gravitational_constant, ref_info->mass_kg,
                                                     std::max(1.0, orbit_radius_m), 0.0);

            player_pos_world = cfg.system_center + WorldVec3(ship_rel.position_m);
            player_vel_world = glm::dvec3(ship_rel.velocity_mps);
        }

        _orbitsim = std::move(scenario);
    }

    // ---- Orbiter helpers ----

    OrbiterInfo *GameplayState::find_player_orbiter()
    {
        for (auto &o : _orbiters)
        {
            if (o.is_player)
            {
                return &o;
            }
        }
        return nullptr;
    }

    const OrbiterInfo *GameplayState::find_player_orbiter() const
    {
        for (const auto &o : _orbiters)
        {
            if (o.is_player)
            {
                return &o;
            }
        }
        return nullptr;
    }

    OrbiterInfo *GameplayState::find_orbiter(const EntityId entity)
    {
        if (!entity.is_valid())
        {
            return nullptr;
        }

        for (auto &orbiter : _orbiters)
        {
            if (orbiter.entity == entity)
            {
                return &orbiter;
            }
        }

        return nullptr;
    }

    const OrbiterInfo *GameplayState::find_orbiter(const EntityId entity) const
    {
        if (!entity.is_valid())
        {
            return nullptr;
        }

        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.entity == entity)
            {
                return &orbiter;
            }
        }

        return nullptr;
    }

    OrbiterInfo *GameplayState::find_orbiter(const std::string_view name)
    {
        if (name.empty())
        {
            return nullptr;
        }

        for (auto &orbiter : _orbiters)
        {
            if (orbiter.name == name)
            {
                return &orbiter;
            }
        }

        return nullptr;
    }

    const OrbiterInfo *GameplayState::find_orbiter(const std::string_view name) const
    {
        if (name.empty())
        {
            return nullptr;
        }

        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.name == name)
            {
                return &orbiter;
            }
        }

        return nullptr;
    }

    EntityId GameplayState::player_entity() const
    {
        const OrbiterInfo *p = find_player_orbiter();
        return p ? p->entity : EntityId{};
    }

    EntityId GameplayState::select_rebase_anchor_entity() const
    {
        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.is_rebase_anchor && orbiter.entity.is_valid())
            {
                return orbiter.entity;
            }
        }

        // Fallback: first valid player.
        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.is_player && orbiter.entity.is_valid())
            {
                return orbiter.entity;
            }
        }

        // Last resort: any valid orbiter.
        for (const auto &orbiter : _orbiters)
        {
            if (orbiter.entity.is_valid())
            {
                return orbiter.entity;
            }
        }

        return EntityId{};
    }

    void GameplayState::update_rebase_anchor()
    {
        const EntityId next_anchor = select_rebase_anchor_entity();
        if (!next_anchor.is_valid())
        {
            _world.clear_rebase_anchor();
            return;
        }

        if (next_anchor != _world.rebase_anchor())
        {
            _world.set_rebase_anchor(next_anchor);
        }
    }

    void GameplayState::sync_player_camera_target(GameStateContext &ctx) const
    {
        if (!ctx.api)
        {
            return;
        }

        const OrbiterInfo *player_orbiter = find_player_orbiter();
        if (!player_orbiter || player_orbiter->name.empty())
        {
            return;
        }

        GameAPI::CameraTarget target{};
        target.type = player_orbiter->render_is_gltf
                          ? GameAPI::CameraTargetType::GLTFInstance
                          : GameAPI::CameraTargetType::MeshInstance;
        target.name = player_orbiter->name;
        if (const Entity *player_entity_ptr = _world.entities().find(player_orbiter->entity))
        {
            target.localOffset = player_entity_ptr->physics_origin_offset_local();
        }

        GameAPI::OrbitCameraSettings orbit = ctx.api->get_orbit_camera_settings();
        orbit.target = target;
        ctx.api->set_orbit_camera_settings(orbit);

        GameAPI::FollowCameraSettings follow = ctx.api->get_follow_camera_settings();
        follow.target = target;
        ctx.api->set_follow_camera_settings(follow);

        GameAPI::ChaseCameraSettings chase = ctx.api->get_chase_camera_settings();
        chase.target = target;
        ctx.api->set_chase_camera_settings(chase);
    }

    void GameplayState::sync_player_collision_callbacks()
    {
#if defined(VULKAN_ENGINE_USE_JOLT) && VULKAN_ENGINE_USE_JOLT
        if (!_physics)
        {
            return;
        }

        for (const OrbiterInfo &orbiter : _orbiters)
        {
            if (!orbiter.entity.is_valid())
            {
                continue;
            }

            const Entity *entity = _world.entities().find(orbiter.entity);
            if (!entity || !entity->has_physics())
            {
                continue;
            }

            const Physics::BodyId body_id{entity->physics_body_value()};
            if (!_physics->is_body_valid(body_id))
            {
                continue;
            }

            _physics->clear_body_callbacks(body_id);
        }

        const OrbiterInfo *player_orbiter = find_player_orbiter();
        if (!player_orbiter || !player_orbiter->entity.is_valid())
        {
            return;
        }

        const Entity *player = _world.entities().find(player_orbiter->entity);
        if (!player || !player->has_physics())
        {
            return;
        }

        const Physics::BodyId player_body{player->physics_body_value()};
        if (!_physics->is_body_valid(player_body))
        {
            return;
        }

        Physics::PhysicsWorld::BodyCallbacks callbacks{};
        callbacks.on_collision = [this](const Physics::CollisionEvent &e) {
            if (e.type != Physics::ContactEventType::Begin)
            {
                return;
            }

            mark_prediction_dirty();

            if (!_contact_log_enabled)
            {
                return;
            }

            ContactLogEntry entry{};
            entry.time_s = static_cast<float>(_fixed_time_s);
            entry.type = e.type;
            entry.self_body = e.self.value;
            entry.other_body = e.other.value;
            entry.self_user_data = e.self_user_data;
            entry.other_user_data = e.other_user_data;
            entry.point = e.point;
            entry.normal = e.normal;
            entry.penetration_depth = e.penetration_depth;

            _contact_log.push_back(entry);
            while (_contact_log.size() > _contact_log_capacity)
            {
                _contact_log.pop_front();
            }

            if (_contact_log_print_console)
            {
                Logger::debug("[Collision][{}] self={} other={} depth={:.3f} p=({:.2f},{:.2f},{:.2f})",
                              contact_event_type_name(entry.type),
                              entry.self_body,
                              entry.other_body,
                              entry.penetration_depth,
                              entry.point.x, entry.point.y, entry.point.z);
            }
        };
        _physics->set_body_callbacks(player_body, callbacks);
#endif
    }

    bool GameplayState::set_active_player_orbiter(GameStateContext &ctx, const EntityId entity)
    {
        OrbiterInfo *target = find_orbiter(entity);
        OrbiterInfo *current = find_player_orbiter();
        if (!target || !target->entity.is_valid() || target == current)
        {
            return false;
        }

        struct ControllerSnapshot
        {
            float thrust_force{500.0f};
            float torque_strength{50.0f};
            float sas_damping{5.0f};
            bool sas_enabled{false};
            bool sas_toggle_prev_down{false};
        } snapshot{};

        if (current)
        {
            if (Entity *current_entity = _world.entities().find(current->entity))
            {
                if (ShipController *controller = current_entity->get_component<ShipController>())
                {
                    snapshot.thrust_force = controller->thrust_force();
                    snapshot.torque_strength = controller->torque_strength();
                    snapshot.sas_damping = controller->sas_damping();
                    snapshot.sas_enabled = controller->sas_enabled();
                    snapshot.sas_toggle_prev_down = ctx.input && ctx.input->key_down(Key::T);
                    (void) current_entity->remove_component<ShipController>();
                }
            }
        }

        if (!_rails_warp_active && target->rails.active())
        {
            (void) demote_orbiter_from_rails(*target);
        }

        for (auto &orbiter : _orbiters)
        {
            const bool is_active = orbiter.entity == target->entity;
            orbiter.is_player = is_active;
            orbiter.is_rebase_anchor = is_active;
        }

        for (auto &orbiter_def : _scenario_config.orbiters)
        {
            const bool is_active = orbiter_def.name == target->name;
            orbiter_def.is_player = is_active;
            orbiter_def.is_rebase_anchor = is_active;
        }

        if (Entity *target_entity = _world.entities().find(target->entity))
        {
            ShipController *controller = target_entity->get_component<ShipController>();
            if (!controller)
            {
                controller = target_entity->add_component<ShipController>();
            }
            if (controller)
            {
                controller->set_thrust_force(snapshot.thrust_force);
                controller->set_torque_strength(snapshot.torque_strength);
                controller->set_sas_damping(snapshot.sas_damping);
                controller->set_sas_enabled(target->rails.active() ? target->rails.sas_enabled : snapshot.sas_enabled);
                controller->set_sas_toggle_prev_down(snapshot.sas_toggle_prev_down);
            }
        }

        update_rebase_anchor();
        sync_player_camera_target(ctx);
        sync_player_collision_callbacks();

        _prediction.selection.active_subject = PredictionSubjectKey{PredictionSubjectKind::Orbiter, target->entity.value};
        _prediction.selection.overlay_subjects.clear();
        _prediction.selection.selected_group_index = -1;
        _execute_node_armed = false;
        _execute_node_id = -1;
        _maneuver_state.nodes.clear();
        _maneuver_state.selected_node_id = -1;
        _maneuver_state.next_node_id = 0;
        _maneuver_gizmo_interaction = {};
        mark_prediction_dirty();
        return true;
    }

    bool GameplayState::cycle_player_orbiter(GameStateContext &ctx, const int direction)
    {
        if (_orbiters.size() < 2 || direction == 0)
        {
            return false;
        }

        std::size_t current_index = 0;
        bool found_current = false;
        for (std::size_t i = 0; i < _orbiters.size(); ++i)
        {
            if (_orbiters[i].is_player)
            {
                current_index = i;
                found_current = true;
                break;
            }
        }

        if (!found_current)
        {
            for (std::size_t i = 0; i < _orbiters.size(); ++i)
            {
                if (_orbiters[i].entity.is_valid())
                {
                    current_index = i;
                    found_current = true;
                    break;
                }
            }
        }

        if (!found_current)
        {
            return false;
        }

        const std::size_t count = _orbiters.size();
        std::size_t next_index = current_index;
        const int step = direction > 0 ? 1 : -1;
        for (std::size_t attempt = 0; attempt < count; ++attempt)
        {
            const int signed_next =
                    (static_cast<int>(next_index) + step + static_cast<int>(count)) % static_cast<int>(count);
            next_index = static_cast<std::size_t>(signed_next);
            if (_orbiters[next_index].entity.is_valid())
            {
                break;
            }
        }

        if (next_index == current_index || !_orbiters[next_index].entity.is_valid())
        {
            return false;
        }

        return set_active_player_orbiter(ctx, _orbiters[next_index].entity);
    }
} // namespace Game
