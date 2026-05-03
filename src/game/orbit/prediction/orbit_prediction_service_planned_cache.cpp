#include "game/orbit/prediction/orbit_prediction_service_internal.h"

namespace Game
{
    namespace
    {
        constexpr std::size_t kMaxCachedPlannedChunks = 512u;
        // Cache keys only select candidates; find_cached_chunk validates state continuity before reuse.
        constexpr double kBaselineCacheStartTimeQuantumS = 0.001;
        constexpr double kBaselineCachePositionQuantumM = 1.0;
        constexpr double kBaselineCacheVelocityQuantumMps = 1.0e-3;
        constexpr double kBaselineCacheSpinAxisQuantum = 1.0e-6;
        constexpr double kBaselineCacheSpinAngleQuantumRad = 1.0e-6;
        constexpr double kBaselineCacheSpinRateQuantumRadPerS = 1.0e-6;
        constexpr double kManeuverCacheTimeQuantumS = 0.001;
        constexpr double kManeuverCacheDvQuantumMps = 1.0e-6;

        template<typename T>
        void planned_hash_combine(uint64_t &seed, const T &value)
        {
            seed ^= static_cast<uint64_t>(std::hash<T>{}(value)) + 0x9e3779b97f4a7c15ULL + (seed << 6u) + (seed >> 2u);
        }

        uint64_t hash_vec3(const orbitsim::Vec3 &v)
        {
            uint64_t seed = kPlannedCacheHashSeed;
            planned_hash_combine(seed, v.x);
            planned_hash_combine(seed, v.y);
            planned_hash_combine(seed, v.z);
            return seed;
        }

        int64_t quantized_cache_tick(const double value, const double quantum)
        {
            if (!std::isfinite(value) || !(quantum > 0.0))
            {
                return 0;
            }

            const long double scaled = static_cast<long double>(value) / static_cast<long double>(quantum);
            const long double rounded = std::round(scaled);
            constexpr long double min_tick =
                    static_cast<long double>(std::numeric_limits<int64_t>::min());
            constexpr long double max_tick =
                    static_cast<long double>(std::numeric_limits<int64_t>::max());
            if (rounded <= min_tick)
            {
                return std::numeric_limits<int64_t>::min();
            }
            if (rounded >= max_tick)
            {
                return std::numeric_limits<int64_t>::max();
            }
            return static_cast<int64_t>(rounded);
        }

        double planned_chunk_cache_time_quantum_s(const OrbitPredictionService::PredictionProfileId profile_id)
        {
            switch (profile_id)
            {
            case OrbitPredictionService::PredictionProfileId::Exact:
                return 0.001;
            case OrbitPredictionService::PredictionProfileId::Near:
                return 0.1;
            case OrbitPredictionService::PredictionProfileId::Tail:
                return 1.0;
            }
            return 0.1;
        }

        uint64_t hash_quantized_vec3(const orbitsim::Vec3 &v, const double quantum)
        {
            uint64_t seed = kPlannedCacheHashSeed;
            planned_hash_combine(seed, quantized_cache_tick(v.x, quantum));
            planned_hash_combine(seed, quantized_cache_tick(v.y, quantum));
            planned_hash_combine(seed, quantized_cache_tick(v.z, quantum));
            return seed;
        }

        uint64_t hash_quantized_state(const orbitsim::State &state)
        {
            uint64_t seed = hash_quantized_vec3(state.position_m, kBaselineCachePositionQuantumM);
            planned_hash_combine(seed, hash_quantized_vec3(state.velocity_mps, kBaselineCacheVelocityQuantumMps));
            planned_hash_combine(seed, quantized_cache_tick(state.spin.axis.x, kBaselineCacheSpinAxisQuantum));
            planned_hash_combine(seed, quantized_cache_tick(state.spin.axis.y, kBaselineCacheSpinAxisQuantum));
            planned_hash_combine(seed, quantized_cache_tick(state.spin.axis.z, kBaselineCacheSpinAxisQuantum));
            planned_hash_combine(seed, quantized_cache_tick(state.spin.angle_rad, kBaselineCacheSpinAngleQuantumRad));
            planned_hash_combine(seed, quantized_cache_tick(state.spin.rate_rad_per_s, kBaselineCacheSpinRateQuantumRadPerS));
            return seed;
        }

        uint64_t hash_state(const orbitsim::State &state)
        {
            uint64_t seed = hash_vec3(state.position_m);
            planned_hash_combine(seed, hash_vec3(state.velocity_mps));
            planned_hash_combine(seed, state.spin.axis.x);
            planned_hash_combine(seed, state.spin.axis.y);
            planned_hash_combine(seed, state.spin.axis.z);
            planned_hash_combine(seed, state.spin.angle_rad);
            planned_hash_combine(seed, state.spin.rate_rad_per_s);
            return seed;
        }

        uint64_t hash_massive_body_set(const std::vector<orbitsim::MassiveBody> &bodies)
        {
            uint64_t seed = kPlannedCacheHashSeed;
            for (const orbitsim::MassiveBody &body : bodies)
            {
                planned_hash_combine(seed, body.id);
                planned_hash_combine(seed, body.mass_kg);
                planned_hash_combine(seed, hash_state(body.state));
            }
            return seed;
        }

        uint64_t hash_maneuver_impulse_for_cache(const OrbitPredictionService::ManeuverImpulse &impulse)
        {
            uint64_t seed = kPlannedCacheHashSeed;
            planned_hash_combine(seed, impulse.node_id);
            planned_hash_combine(seed, quantized_cache_tick(impulse.t_s, kManeuverCacheTimeQuantumS));
            planned_hash_combine(seed, impulse.primary_body_id);
            planned_hash_combine(seed, quantized_cache_tick(impulse.dv_rtn_mps.x, kManeuverCacheDvQuantumMps));
            planned_hash_combine(seed, quantized_cache_tick(impulse.dv_rtn_mps.y, kManeuverCacheDvQuantumMps));
            planned_hash_combine(seed, quantized_cache_tick(impulse.dv_rtn_mps.z, kManeuverCacheDvQuantumMps));
            return seed;
        }
    } // namespace

    uint64_t planned_baseline_generation_hash(const OrbitPredictionService::Request &request,
                                              const double start_time_s,
                                              const orbitsim::State &start_state)
    {
        uint64_t seed = kPlannedCacheHashSeed;
        planned_hash_combine(seed, request.track_id);
        planned_hash_combine(seed, quantized_cache_tick(start_time_s, kBaselineCacheStartTimeQuantumS));
        planned_hash_combine(seed, hash_quantized_state(start_state));
        return seed;
    }

    uint64_t planned_solver_context_hash(const OrbitPredictionService::Request &request,
                                         const std::vector<orbitsim::MassiveBody> &massive_bodies)
    {
        uint64_t seed = kPlannedCacheHashSeed;
        planned_hash_combine(seed, static_cast<uint8_t>(request.solve_quality));
        planned_hash_combine(seed, request.thrusting);
        planned_hash_combine(seed, request.lagrange_sensitive);
        planned_hash_combine(seed, request.sim_config.gravitational_constant);
        planned_hash_combine(seed, request.sim_config.softening_length_m);
        planned_hash_combine(seed, request.sim_config.spacecraft_integrator.adaptive);
        planned_hash_combine(seed, request.sim_config.spacecraft_integrator.max_step_s);
        planned_hash_combine(seed, request.sim_config.spacecraft_integrator.abs_tol);
        planned_hash_combine(seed, request.sim_config.spacecraft_integrator.rel_tol);
        planned_hash_combine(seed, request.sim_config.spacecraft_integrator.max_substeps);
        planned_hash_combine(seed, request.sim_config.spacecraft_integrator.max_substeps_hard);
        planned_hash_combine(seed, request.sim_config.spacecraft_integrator.max_interval_splits);
        planned_hash_combine(seed, request.preferred_primary_body_id);
        planned_hash_combine(seed, hash_massive_body_set(massive_bodies));
        return seed;
    }

    void accumulate_planned_maneuver_cache_hash(
            uint64_t &seed,
            const OrbitPredictionService::ManeuverImpulse &impulse)
    {
        planned_hash_combine(seed, hash_maneuver_impulse_for_cache(impulse));
    }

    OrbitPredictionService::PlannedChunkCacheKey make_planned_chunk_cache_key(
            const OrbitPredictionService::Request &request,
            const OrbitPredictionService::PredictionChunkPlan &chunk,
            const OrbitPredictionService::PredictionProfileId profile_id,
            const uint64_t baseline_generation_id,
            const uint64_t frame_independent_generation,
            const uint64_t upstream_maneuver_hash)
    {
        const double chunk_cache_time_quantum_s = planned_chunk_cache_time_quantum_s(profile_id);
        return OrbitPredictionService::PlannedChunkCacheKey{
                .track_id = request.track_id,
                .baseline_generation_id = baseline_generation_id,
                .upstream_maneuver_hash = upstream_maneuver_hash,
                .frame_independent_generation = frame_independent_generation,
                .chunk_t0_s = chunk.t0_s,
                .chunk_t1_s = chunk.t1_s,
                .chunk_t0_tick = quantized_cache_tick(chunk.t0_s, chunk_cache_time_quantum_s),
                .chunk_t1_tick = quantized_cache_tick(chunk.t1_s, chunk_cache_time_quantum_s),
                .profile_id = profile_id,
        };
    }

    std::optional<OrbitPredictionService::PlannedChunkCacheEntry>
    OrbitPredictionService::find_cached_planned_chunk(
            const PlannedChunkCacheKey &key,
            const orbitsim::State &expected_start_state)
    {
        std::lock_guard<std::mutex> lock(_planned_chunk_cache_mutex);
        const auto cache_it = _planned_chunk_cache_by_key.find(key);
        if (cache_it == _planned_chunk_cache_by_key.end())
        {
            return std::nullopt;
        }

        auto entry_it = cache_it->second;
        PlannedChunkCacheEntry &entry = *entry_it;
        if (entry.samples.size() < 2u ||
            entry.segments.empty() ||
            !validate_trajectory_segment_continuity(entry.segments) ||
            (!entry.seam_validation_segments.empty() &&
             !validate_trajectory_segment_continuity(entry.seam_validation_segments)))
        {
            _planned_chunk_cache_by_key.erase(cache_it);
            _planned_chunk_cache.erase(entry_it);
            return std::nullopt;
        }

        if (!states_are_continuous(entry.start_state, expected_start_state))
        {
            return std::nullopt;
        }

        _planned_chunk_cache.splice(_planned_chunk_cache.begin(),
                                    _planned_chunk_cache,
                                    entry_it);
        cache_it->second = _planned_chunk_cache.begin();
        return *_planned_chunk_cache.begin();
    }

    void OrbitPredictionService::store_cached_planned_chunk(PlannedChunkCacheEntry entry)
    {
        if (entry.samples.size() < 2u ||
            entry.segments.empty() ||
            !validate_trajectory_segment_continuity(entry.segments) ||
            (!entry.seam_validation_segments.empty() &&
             !validate_trajectory_segment_continuity(entry.seam_validation_segments)) ||
            !finite_state(entry.start_state) ||
            !finite_state(entry.end_state))
        {
            return;
        }

        std::lock_guard<std::mutex> lock(_planned_chunk_cache_mutex);
        const auto existing_it = _planned_chunk_cache_by_key.find(entry.key);
        if (existing_it != _planned_chunk_cache_by_key.end())
        {
            auto lru_it = existing_it->second;
            *lru_it = std::move(entry);
            _planned_chunk_cache.splice(_planned_chunk_cache.begin(),
                                        _planned_chunk_cache,
                                        lru_it);
            existing_it->second = _planned_chunk_cache.begin();
        }
        else
        {
            _planned_chunk_cache.push_front(std::move(entry));
            _planned_chunk_cache_by_key.emplace(_planned_chunk_cache.front().key,
                                                _planned_chunk_cache.begin());
        }

        if (_planned_chunk_cache.size() > kMaxCachedPlannedChunks)
        {
            _planned_chunk_cache_by_key.erase(_planned_chunk_cache.back().key);
            _planned_chunk_cache.pop_back();
        }
    }
} // namespace Game
