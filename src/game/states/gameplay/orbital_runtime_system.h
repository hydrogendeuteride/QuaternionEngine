#pragma once

#include "game/states/gameplay/orbit_runtime_types.h"

#include <memory>
#include <string_view>
#include <vector>

namespace Game
{
    class OrbitalRuntimeSystem
    {
    public:
        void reset();

        [[nodiscard]] std::vector<OrbiterInfo> &orbiters() { return _orbiters; }
        [[nodiscard]] const std::vector<OrbiterInfo> &orbiters() const { return _orbiters; }

        [[nodiscard]] std::unique_ptr<OrbitalScenario> &scenario_owner() { return _scenario; }
        [[nodiscard]] const std::unique_ptr<OrbitalScenario> &scenario_owner() const { return _scenario; }
        [[nodiscard]] OrbitalScenario *scenario() { return _scenario.get(); }
        [[nodiscard]] const OrbitalScenario *scenario() const { return _scenario.get(); }
        void set_scenario(std::unique_ptr<OrbitalScenario> scenario);

        [[nodiscard]] bool &runtime_orbiter_rails_enabled() { return _runtime_orbiter_rails_enabled; }
        [[nodiscard]] bool runtime_orbiter_rails_enabled() const { return _runtime_orbiter_rails_enabled; }
        [[nodiscard]] double &runtime_orbiter_rails_distance_m() { return _runtime_orbiter_rails_distance_m; }
        [[nodiscard]] double runtime_orbiter_rails_distance_m() const { return _runtime_orbiter_rails_distance_m; }

        [[nodiscard]] double sim_time_or(double fallback_s) const;

        [[nodiscard]] OrbiterInfo *find_player_orbiter();
        [[nodiscard]] const OrbiterInfo *find_player_orbiter() const;
        [[nodiscard]] OrbiterInfo *find_orbiter(EntityId entity);
        [[nodiscard]] const OrbiterInfo *find_orbiter(EntityId entity) const;
        [[nodiscard]] OrbiterInfo *find_orbiter(std::string_view name);
        [[nodiscard]] const OrbiterInfo *find_orbiter(std::string_view name) const;
        [[nodiscard]] EntityId player_entity() const;
        [[nodiscard]] EntityId select_rebase_anchor_entity() const;

    private:
        std::vector<OrbiterInfo> _orbiters;
        std::unique_ptr<OrbitalScenario> _scenario;
        bool _runtime_orbiter_rails_enabled{true};
        double _runtime_orbiter_rails_distance_m{10'000.0};
    };
} // namespace Game
