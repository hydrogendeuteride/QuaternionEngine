#include "game/states/gameplay/prediction/prediction_frame_resolver.h"

#include "game/orbit/orbit_prediction_math.h"
#include "game/orbit/orbit_prediction_tuning.h"

#include "orbitsim/frame_utils.hpp"
#include "orbitsim/math.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace Game
{
    namespace
    {
        double fallback_reference_time_s(const PredictionFrameResolverContext &context,
                                         const OrbitPredictionCache &cache,
                                         const double display_time_s)
        {
            if (std::isfinite(display_time_s))
            {
                return display_time_s;
            }
            if (std::isfinite(context.current_sim_time_s))
            {
                return context.current_sim_time_s;
            }
            return cache.identity.build_time_s;
        }
    } // namespace

    bool PredictionFrameResolver::same_frame_spec(const orbitsim::TrajectoryFrameSpec &a,
                                                  const orbitsim::TrajectoryFrameSpec &b)
    {
        return a.type == b.type &&
               a.primary_body_id == b.primary_body_id &&
               a.secondary_body_id == b.secondary_body_id &&
               a.target_spacecraft_id == b.target_spacecraft_id;
    }

    bool PredictionFrameResolver::is_lagrange_sensitive(const orbitsim::TrajectoryFrameSpec &spec)
    {
        return spec.type == orbitsim::TrajectoryFrameType::Synodic;
    }

    const orbitsim::MassiveBody *PredictionFrameResolver::find_massive_body(
            const std::vector<orbitsim::MassiveBody> &bodies,
            const orbitsim::BodyId body_id)
    {
        if (body_id == orbitsim::kInvalidBodyId)
        {
            return nullptr;
        }

        for (const orbitsim::MassiveBody &body : bodies)
        {
            if (body.id == body_id)
            {
                return &body;
            }
        }

        return nullptr;
    }

    orbitsim::BodyId PredictionFrameResolver::select_primary_body_id(
            const PredictionFrameResolverContext &context,
            const std::vector<orbitsim::MassiveBody> &bodies,
            const OrbitPredictionCache *cache,
            const orbitsim::Vec3 &query_pos_m,
            const double query_time_s,
            const orbitsim::BodyId preferred_body_id)
    {
        if (bodies.empty())
        {
            return orbitsim::kInvalidBodyId;
        }

        const double eps2 = context.softening_length_m * context.softening_length_m;
        std::size_t best_index = 0;
        double best_metric = -1.0;
        std::optional<std::size_t> preferred_index;
        double preferred_metric = -1.0;

        const auto body_position_at = [&](const std::size_t i) -> orbitsim::Vec3 {
            const orbitsim::MassiveBody &body = bodies[i];
            const auto &ephemeris = cache ? cache->resolved_shared_ephemeris()
                                          : OrbitPredictionService::SharedCelestialEphemeris{};
            if (ephemeris && !ephemeris->empty())
            {
                return ephemeris->body_state_at_by_id(body.id, query_time_s).position_m;
            }
            return body.state.position_m;
        };

        for (std::size_t i = 0; i < bodies.size(); ++i)
        {
            const double mass_kg = std::isfinite(bodies[i].mass_kg) ? bodies[i].mass_kg : 0.0;
            if (!(mass_kg >= 0.0))
            {
                continue;
            }

            const orbitsim::Vec3 dr = body_position_at(i) - query_pos_m;
            const double r2 = glm::dot(dr, dr) + eps2;
            if (!(r2 > 0.0) || !std::isfinite(r2))
            {
                continue;
            }

            const double metric = mass_kg / r2;
            if (metric > best_metric)
            {
                best_metric = metric;
                best_index = i;
            }

            if (bodies[i].id == preferred_body_id)
            {
                preferred_index = i;
                preferred_metric = metric;
            }
        }

        if (best_metric <= 0.0)
        {
            return orbitsim::kInvalidBodyId;
        }

        if (preferred_index.has_value() &&
            preferred_metric > 0.0 &&
            preferred_metric >= (best_metric * OrbitPredictionTuning::kPrimaryBodyHysteresisKeepRatio))
        {
            return bodies[*preferred_index].id;
        }

        return bodies[best_index].id;
    }

    orbitsim::TrajectoryFrameSpec PredictionFrameResolver::resolve_display_frame_spec(
            const PredictionFrameResolverContext &context,
            const OrbitPredictionCache &cache,
            double display_time_s)
    {
        orbitsim::TrajectoryFrameSpec resolved = context.frame_selection.spec;
        const auto &bodies = cache.resolved_massive_bodies();
        if (resolved.type != orbitsim::TrajectoryFrameType::LVLH ||
            resolved.primary_body_id != orbitsim::kInvalidBodyId ||
            bodies.empty())
        {
            return resolved;
        }

        const double reference_time_s = fallback_reference_time_s(context, cache, display_time_s);
        if (!context.player_lookup)
        {
            return resolved;
        }

        const std::optional<orbitsim::State> target_state =
                context.player_lookup(resolved.target_spacecraft_id, reference_time_s);
        if (!target_state.has_value())
        {
            return resolved;
        }

        const orbitsim::BodyId preferred_body_id =
                cache.display.resolved_frame_spec_valid &&
                                cache.display.resolved_frame_spec.primary_body_id != orbitsim::kInvalidBodyId
                        ? cache.display.resolved_frame_spec.primary_body_id
                        : context.frame_selection.spec.primary_body_id;
        const orbitsim::BodyId primary_body_id = select_primary_body_id(
                context,
                bodies,
                &cache,
                target_state->position_m,
                reference_time_s,
                preferred_body_id);
        if (primary_body_id != orbitsim::kInvalidBodyId)
        {
            resolved.primary_body_id = primary_body_id;
        }

        return resolved;
    }

    double PredictionFrameResolver::resolve_display_reference_time_s(
            const PredictionFrameResolverContext &context,
            const OrbitPredictionCache &cache,
            const double display_time_s)
    {
        const double reference_time_s = fallback_reference_time_s(context, cache, display_time_s);
        const orbitsim::TrajectoryFrameSpec frame_spec =
                cache.display.resolved_frame_spec_valid
                    ? cache.display.resolved_frame_spec
                    : resolve_display_frame_spec(context, cache, reference_time_s);

        if (context.maneuver_axis_drag_active &&
            is_lagrange_sensitive(frame_spec) &&
            std::isfinite(context.maneuver_drag_display_reference_time_s))
        {
            return context.maneuver_drag_display_reference_time_s;
        }

        const bool freeze_planned_synodic =
                is_lagrange_sensitive(frame_spec) &&
                context.maneuver_nodes_enabled &&
                context.maneuver_plan_has_nodes &&
                !context.maneuver_axis_drag_active &&
                std::isfinite(cache.identity.build_time_s);
        return freeze_planned_synodic ? cache.identity.build_time_s : reference_time_s;
    }

    bool PredictionFrameResolver::build_display_frame(
            const PredictionFrameResolverContext &context,
            const OrbitPredictionCache &cache,
            orbitsim::RotatingFrame &out_frame,
            double display_time_s)
    {
        out_frame = {};
        const double reference_time_s = resolve_display_reference_time_s(context, cache, display_time_s);
        const orbitsim::TrajectoryFrameSpec frame_spec =
                cache.display.resolved_frame_spec_valid
                    ? cache.display.resolved_frame_spec
                    : resolve_display_frame_spec(context, cache, reference_time_s);
        const auto &ephemeris = cache.resolved_shared_ephemeris();
        const auto &bodies = cache.resolved_massive_bodies();
        const auto state_at = [&](const orbitsim::MassiveBody &body) -> orbitsim::State {
            if (ephemeris && !ephemeris->empty())
            {
                return ephemeris->body_state_at_by_id(body.id, reference_time_s);
            }
            return body.state;
        };

        switch (frame_spec.type)
        {
            case orbitsim::TrajectoryFrameType::Inertial:
                break;

            case orbitsim::TrajectoryFrameType::BodyCenteredInertial:
            {
                const orbitsim::MassiveBody *body = find_massive_body(bodies, frame_spec.primary_body_id);
                if (!body)
                {
                    return false;
                }
                out_frame = orbitsim::make_body_centered_inertial_frame(state_at(*body));
                break;
            }

            case orbitsim::TrajectoryFrameType::BodyFixed:
            {
                const orbitsim::MassiveBody *body = find_massive_body(bodies, frame_spec.primary_body_id);
                if (!body)
                {
                    return false;
                }
                const std::optional<orbitsim::RotatingFrame> body_fixed =
                        orbitsim::make_body_fixed_frame(state_at(*body));
                if (!body_fixed.has_value())
                {
                    return false;
                }
                out_frame = *body_fixed;
                break;
            }

            case orbitsim::TrajectoryFrameType::Synodic:
            {
                const orbitsim::MassiveBody *body_a = find_massive_body(bodies, frame_spec.primary_body_id);
                const orbitsim::MassiveBody *body_b = find_massive_body(bodies, frame_spec.secondary_body_id);
                if (!body_a || !body_b)
                {
                    return false;
                }

                const std::optional<orbitsim::SynodicFrame> synodic =
                        orbitsim::make_synodic_frame(state_at(*body_a),
                                                     body_a->mass_kg,
                                                     state_at(*body_b),
                                                     body_b->mass_kg);
                if (!synodic.has_value())
                {
                    return false;
                }
                out_frame = *synodic;
                break;
            }

            case orbitsim::TrajectoryFrameType::LVLH:
            {
                if (bodies.empty() || !context.player_lookup)
                {
                    return false;
                }

                const std::optional<orbitsim::State> target_state =
                        context.player_lookup(frame_spec.target_spacecraft_id, reference_time_s);
                if (!target_state.has_value())
                {
                    return false;
                }

                const orbitsim::MassiveBody *primary = find_massive_body(bodies, frame_spec.primary_body_id);
                if (!primary)
                {
                    return false;
                }

                const std::optional<orbitsim::RotatingFrame> lvlh =
                        orbitsim::make_lvlh_frame(state_at(*primary), *target_state);
                if (!lvlh.has_value())
                {
                    return false;
                }
                out_frame = *lvlh;
                break;
            }
        }

        return true;
    }

    bool PredictionFrameResolver::build_display_transform(
            const PredictionFrameResolverContext &context,
            const OrbitPredictionCache &cache,
            WorldVec3 &out_origin_world,
            glm::dmat3 &out_frame_to_world,
            double display_time_s)
    {
        out_origin_world = context.system_center;
        out_frame_to_world = glm::dmat3(1.0);
        const double reference_time_s = resolve_display_reference_time_s(context, cache, display_time_s);
        const auto &ephemeris = cache.resolved_shared_ephemeris();
        const auto &bodies = cache.resolved_massive_bodies();

        const auto state_at = [&](const orbitsim::MassiveBody &body) -> orbitsim::State {
            if (ephemeris && !ephemeris->empty())
            {
                return ephemeris->body_state_at_by_id(body.id, reference_time_s);
            }
            return body.state;
        };

        orbitsim::State world_ref_state{};
        bool have_world_ref_state = false;
        if (context.world_reference_body_id != orbitsim::kInvalidBodyId)
        {
            if (const orbitsim::MassiveBody *world_ref_body =
                        find_massive_body(bodies, context.world_reference_body_id))
            {
                world_ref_state = state_at(*world_ref_body);
                have_world_ref_state = true;
            }
            else if (context.world_reference_sim_body)
            {
                world_ref_state = context.world_reference_sim_body->state;
                have_world_ref_state = true;
            }
        }

        orbitsim::RotatingFrame frame{};
        if (!build_display_frame(context, cache, frame, reference_time_s))
        {
            return false;
        }

        out_frame_to_world = glm::dmat3(glm::dvec3(frame.ex_i.x, frame.ex_i.y, frame.ex_i.z),
                                        glm::dvec3(frame.ey_i.x, frame.ey_i.y, frame.ey_i.z),
                                        glm::dvec3(frame.ez_i.x, frame.ez_i.y, frame.ez_i.z));

        const glm::dvec3 world_ref_position_m =
                have_world_ref_state ? glm::dvec3(world_ref_state.position_m) : glm::dvec3(0.0);
        out_origin_world = context.world_reference_body_world +
                           WorldVec3(glm::dvec3(frame.origin_position_m) - world_ref_position_m);
        return true;
    }

    WorldVec3 PredictionFrameResolver::sample_position_world(
            const PredictionFrameResolverContext &context,
            const OrbitPredictionCache &cache,
            const orbitsim::TrajectorySample &sample,
            double display_time_s)
    {
        WorldVec3 origin_world{0.0};
        glm::dmat3 frame_to_world(1.0);
        if (!build_display_transform(context, cache, origin_world, frame_to_world, display_time_s))
        {
            return context.system_center + WorldVec3(glm::dvec3(sample.position_m));
        }

        return origin_world + WorldVec3(frame_to_world * glm::dvec3(sample.position_m));
    }

    WorldVec3 PredictionFrameResolver::sample_hermite_world(
            const PredictionFrameResolverContext &context,
            const OrbitPredictionCache &cache,
            const orbitsim::TrajectorySample &a,
            const orbitsim::TrajectorySample &b,
            const double t_s,
            double display_time_s)
    {
        const glm::dvec3 local = OrbitPredictionMath::sample_pair_position_m(a, b, t_s);

        WorldVec3 origin_world{0.0};
        glm::dmat3 frame_to_world(1.0);
        if (!build_display_transform(context, cache, origin_world, frame_to_world, display_time_s))
        {
            return context.system_center + WorldVec3(local);
        }

        return origin_world + WorldVec3(frame_to_world * local);
    }

    WorldVec3 PredictionFrameResolver::frame_origin_world(
            const PredictionFrameResolverContext &context,
            const OrbitPredictionCache &cache,
            double display_time_s)
    {
        WorldVec3 origin_world{0.0};
        glm::dmat3 frame_to_world(1.0);
        if (build_display_transform(context, cache, origin_world, frame_to_world, display_time_s))
        {
            return origin_world;
        }

        return context.system_center;
    }
} // namespace Game
