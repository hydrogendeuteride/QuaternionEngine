#pragma once

#include "game/state/game_state.h"
#include "game/states/gameplay/scenario/scenario_config.h"
#include "core/game_api.h"
#include "scene/planet/planet_heightmap.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

struct LoadedGLTF;
class VulkanEngine;

namespace Game
{
    class GameplayLoadingState : public IGameState
    {
    public:
        /// Construct with an explicit scenario asset path (relative to assets root).
        /// If empty, uses the built-in default scenario path.
        explicit GameplayLoadingState(std::string scenario_asset_path = {});
        ~GameplayLoadingState() override;

        void on_enter(GameStateContext &ctx) override;
        void on_exit(GameStateContext &ctx) override;
        void on_update(GameStateContext &ctx, float dt) override;
        void on_fixed_update(GameStateContext &ctx, float fixed_dt) override;
        void on_draw_ui(GameStateContext &ctx) override;

        bool wants_fixed_update() const override { return false; }
        const char *name() const override { return "GameplayLoading"; }

    private:
        struct PreloadJob
        {
            enum class Kind : uint8_t
            {
                Gltf,
                TerrainHeightMaps
            };

            Kind kind{Kind::Gltf};
            std::string display_name;
            std::string asset_path;
            std::string cache_key;
            std::shared_ptr<std::atomic<float>> progress;
            std::shared_ptr<std::atomic<bool>> cancelled;
            std::shared_ptr<std::atomic<bool>> finished;
            std::shared_ptr<std::atomic<bool>> succeeded;
            std::shared_ptr<LoadedGLTF> loaded_scene;
            std::unique_ptr<planet::HeightFaceSet> loaded_height_faces;
            std::string error;
            std::thread worker;
        };

        void start_preload_jobs(GameStateContext &ctx);
        void start_environment_warmup(GameStateContext &ctx);
        void update_environment_warmup(GameStateContext &ctx);
        void finalize_preload_jobs();
        void cancel_and_join_jobs();
        float compute_progress();
        bool all_jobs_finished() const;
        bool environment_warmup_complete(GameStateContext &ctx) const;
        bool global_ibl_ready(const VulkanEngine &renderer) const;

        static constexpr float kMinimumVisibleSeconds = 0.2f;
        static constexpr std::string_view kDefaultScenarioAsset = "scenarios/default_gameplay.json";
        static constexpr int kLoadingMaxLoadsPerPump = 16;
        static constexpr size_t kLoadingMaxBytesPerPump = 256ull * 1024ull * 1024ull;

        std::string _scenario_asset_path;
        ScenarioConfig _scenario_config{};
        std::vector<std::unique_ptr<PreloadJob>> _jobs;
        std::vector<uint32_t> _environment_required_texture_handles;
        std::vector<uint32_t> _environment_optional_texture_handles;
        GameAPI::IBLPaths _ibl_paths{};
        std::string _status_text;
        std::string _error_text;
        std::string _warning_text;
        float _progress{0.0f};
        float _elapsed{0.0f};
        float _environment_warmup_progress{0.0f};
        std::optional<int> _saved_texture_cache_max_loads_per_pump;
        std::optional<size_t> _saved_texture_cache_max_bytes_per_pump;
        bool _jobs_finalized{false};
        bool _failed{false};
        bool _enter_gameplay_on_exit{false};
        bool _environment_warmup_started{false};
        bool _ibl_warmup_requested{false};
    };
} // namespace Game
