#include "gameplay_loading_state.h"

#include "game/states/gameplay/gameplay_preload_cache.h"
#include "game/states/gameplay/gameplay_state.h"
#include "game/states/gameplay/scenario/scenario_loader.h"
#include "game/states/title_screen_state.h"
#include "core/assets/manager.h"
#include "core/assets/texture_cache.h"
#include "core/engine.h"
#include "core/input/input_system.h"
#include "core/util/logger.h"
#include "render/passes/atmosphere.h"
#include "render/passes/particles.h"
#include "scene/planet/cubesphere.h"

#include "imgui.h"

#include <algorithm>
#include <filesystem>
#include <string_view>
#include <unordered_set>
#include <utility>

namespace Game
{
    GameplayLoadingState::GameplayLoadingState(std::string scenario_asset_path)
        : _scenario_asset_path(scenario_asset_path.empty()
                                   ? std::string(kDefaultScenarioAsset)
                                   : std::move(scenario_asset_path))
        , _scenario_config(default_earth_moon_config())
    {
    }

    GameplayLoadingState::~GameplayLoadingState()
    {
        cancel_and_join_jobs();
    }

    void GameplayLoadingState::on_enter(GameStateContext &ctx)
    {
        _elapsed = 0.0f;
        _progress = 0.0f;
        _status_text = "Preparing gameplay resources...";
        _error_text.clear();
        _warning_text.clear();
        _jobs.clear();
        _environment_texture_handles.clear();
        _environment_warmup_progress = 0.0f;
        _ibl_paths = {};
        _jobs_finalized = false;
        _failed = false;
        _enter_gameplay_on_exit = false;
        _ibl_warmup_requested = false;

        clear_preloaded_gltf_scenes();
        planet::clear_preloaded_heightmap_faces();

        if (!ctx.renderer || !ctx.renderer->_assetManager)
        {
            _failed = true;
            _error_text = "Renderer or asset manager is not available.";
            return;
        }

        const std::string scenario_path = ctx.renderer->_assetManager->assetPath(_scenario_asset_path);
        if (auto loaded = load_scenario_config(scenario_path))
        {
            _scenario_config = std::move(*loaded);
        }
        else
        {
            Logger::warn("[GameplayLoadingState] Falling back to compiled default scenario config.");
            _scenario_config = default_earth_moon_config();
        }

        start_preload_jobs(ctx);
        start_environment_warmup(ctx);
    }

    void GameplayLoadingState::on_exit(GameStateContext &ctx)
    {
        cancel_and_join_jobs();
        (void)ctx;

        if (!_enter_gameplay_on_exit)
        {
            clear_preloaded_gltf_scenes();
            planet::clear_preloaded_heightmap_faces();
        }
    }

    void GameplayLoadingState::on_update(GameStateContext &ctx, float dt)
    {
        _elapsed += dt;
        update_environment_warmup(ctx);
        _progress = compute_progress();

        if (ctx.input && ctx.input->key_pressed(Key::Escape))
        {
            _enter_gameplay_on_exit = false;
            _pending = StateTransition::switch_to<TitleScreenState>();
            return;
        }

        if (!_jobs_finalized && all_jobs_finished())
        {
            finalize_preload_jobs();
        }

        if (!_failed &&
            _jobs_finalized &&
            environment_warmup_complete(ctx) &&
            _elapsed >= kMinimumVisibleSeconds)
        {
            _enter_gameplay_on_exit = true;
            _pending = StateTransition::switch_to<GameplayState>(std::move(_scenario_config));
        }
    }

    void GameplayLoadingState::on_fixed_update(GameStateContext &ctx, float fixed_dt)
    {
        (void)ctx;
        (void)fixed_dt;
    }

    void GameplayLoadingState::on_draw_ui(GameStateContext &ctx)
    {
        (void)ctx;

        const ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->Pos);
        ImGui::SetNextWindowSize(viewport->Size);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration
                               | ImGuiWindowFlags_NoMove
                               | ImGuiWindowFlags_NoSavedSettings;

        if (ImGui::Begin("Gameplay Loading", nullptr, flags))
        {
            const ImVec2 avail = ImGui::GetContentRegionAvail();
            const float panel_width = std::min(520.0f, avail.x);
            const float panel_height = 170.0f;
            const ImVec2 cursor = ImGui::GetCursorPos();
            const float panel_x = cursor.x + std::max(0.0f, (avail.x - panel_width) * 0.5f);
            const float panel_y = cursor.y + std::max(0.0f, (avail.y - panel_height) * 0.5f);

            ImGui::SetCursorPos(ImVec2(panel_x, panel_y));
            ImGui::BeginGroup();

            ImGui::TextUnformatted("Loading Gameplay");
            ImGui::Spacing();
            ImGui::ProgressBar(std::clamp(_progress, 0.0f, 1.0f), ImVec2(panel_width, 0.0f));
            ImGui::Spacing();

            if (_failed)
            {
                ImGui::TextWrapped("%s", _error_text.empty() ? "Preload failed." : _error_text.c_str());
                ImGui::Spacing();

                if (ImGui::Button("Back To Menu", ImVec2(panel_width, 36.0f)))
                {
                    _enter_gameplay_on_exit = false;
                    _pending = StateTransition::switch_to<TitleScreenState>();
                }
            }
            else
            {
                ImGui::TextWrapped("%s", _status_text.c_str());
                if (!_warning_text.empty())
                {
                    ImGui::Spacing();
                    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 214, 102, 255));
                    ImGui::TextWrapped("%s", _warning_text.c_str());
                    ImGui::PopStyleColor();
                }
                ImGui::Spacing();
                ImGui::TextDisabled("%d asset(s) queued", static_cast<int>(_jobs.size()));
            }

            ImGui::EndGroup();
        }
        ImGui::End();
    }

    void GameplayLoadingState::start_preload_jobs(GameStateContext &ctx)
    {
        VulkanEngine *renderer = ctx.renderer;
        AssetManager *assets = renderer ? renderer->_assetManager.get() : nullptr;
        if (!assets)
        {
            _failed = true;
            _error_text = "Asset manager is not available.";
            return;
        }

        std::unordered_set<std::string> unique_paths;
        unique_paths.reserve(_scenario_config.orbiters.size());
        std::unordered_set<std::string> unique_height_dirs;
        unique_height_dirs.reserve(_scenario_config.celestials.size());

        for (const auto &orbiter : _scenario_config.orbiters)
        {
            if (orbiter.gltf_path.empty())
            {
                continue;
            }

            if (!unique_paths.insert(orbiter.gltf_path).second)
            {
                continue;
            }

            auto job = std::make_unique<PreloadJob>();
            job->kind = PreloadJob::Kind::Gltf;
            job->display_name = orbiter.name;
            job->asset_path = orbiter.gltf_path;
            job->progress = std::make_shared<std::atomic<float>>(0.0f);
            job->cancelled = std::make_shared<std::atomic<bool>>(false);
            job->finished = std::make_shared<std::atomic<bool>>(false);
            job->succeeded = std::make_shared<std::atomic<bool>>(false);

            PreloadJob *job_ptr = job.get();
            job->worker = std::thread([assets, job_ptr]() {
                GLTFLoadCallbacks callbacks{};
                callbacks.on_progress = [progress = job_ptr->progress](float value)
                {
                    progress->store(std::clamp(value, 0.0f, 1.0f), std::memory_order_relaxed);
                };
                callbacks.is_cancelled = [cancelled = job_ptr->cancelled]() -> bool
                {
                    return cancelled->load(std::memory_order_acquire);
                };

                auto loaded = assets->loadGLTF(job_ptr->asset_path, &callbacks);
                if (job_ptr->cancelled->load(std::memory_order_acquire))
                {
                    job_ptr->error = "Loading cancelled.";
                }
                else if (loaded.has_value() && loaded.value())
                {
                    job_ptr->loaded_scene = *loaded;
                    job_ptr->succeeded->store(true, std::memory_order_release);
                }
                else
                {
                    job_ptr->error = "Failed to preload '" + job_ptr->asset_path + "'.";
                }

                job_ptr->progress->store(1.0f, std::memory_order_relaxed);
                job_ptr->finished->store(true, std::memory_order_release);
            });

            _jobs.push_back(std::move(job));
        }

        for (const auto &cdef : _scenario_config.celestials)
        {
            if (!cdef.has_terrain || cdef.height_dir.empty())
            {
                continue;
            }

            if (!unique_height_dirs.insert(cdef.height_dir).second)
            {
                continue;
            }

            auto job = std::make_unique<PreloadJob>();
            job->kind = PreloadJob::Kind::TerrainHeightMaps;
            job->display_name = cdef.name + " terrain";
            job->asset_path = cdef.height_dir;
            job->cache_key = cdef.height_dir;
            job->progress = std::make_shared<std::atomic<float>>(0.0f);
            job->cancelled = std::make_shared<std::atomic<bool>>(false);
            job->finished = std::make_shared<std::atomic<bool>>(false);
            job->succeeded = std::make_shared<std::atomic<bool>>(false);

            const std::string probe_face_path =
                    assets->assetPath(cdef.height_dir + "/px.ktx2");
            const std::string absolute_height_dir =
                    !probe_face_path.empty() && std::filesystem::exists(probe_face_path)
                        ? std::filesystem::path(probe_face_path).parent_path().string()
                        : cdef.height_dir;
            PreloadJob *job_ptr = job.get();
            job->worker = std::thread([absolute_height_dir, job_ptr]() {
                if (job_ptr->cancelled->load(std::memory_order_acquire))
                {
                    job_ptr->error = "Loading cancelled.";
                    job_ptr->finished->store(true, std::memory_order_release);
                    return;
                }

                auto faces = std::make_unique<planet::HeightFaceSet>();
                if (planet::load_heightmap_cube_faces_bc4(absolute_height_dir, *faces))
                {
                    job_ptr->loaded_height_faces = std::move(faces);
                    job_ptr->succeeded->store(true, std::memory_order_release);
                }
                else
                {
                    job_ptr->error = "Failed to preload terrain height maps from '" + job_ptr->asset_path + "'.";
                }

                job_ptr->progress->store(1.0f, std::memory_order_relaxed);
                job_ptr->finished->store(true, std::memory_order_release);
            });

            _jobs.push_back(std::move(job));
        }

        if (_jobs.empty())
        {
            _status_text = "No gameplay preload jobs found. Warming environment...";
        }
    }

    void GameplayLoadingState::start_environment_warmup(GameStateContext &ctx)
    {
        VulkanEngine *renderer = ctx.renderer;
        if (!renderer || !renderer->_assetManager)
        {
            return;
        }

        AssetManager *assets = renderer->_assetManager.get();
        TextureCache *textures = renderer->_textureCache.get();
        SamplerManager *samplers = renderer->_samplerManager.get();

        if (textures && samplers)
        {
            const VkSampler tile_sampler = samplers->linearClampEdge() != VK_NULL_HANDLE
                                               ? samplers->linearClampEdge()
                                               : samplers->defaultLinear();
            const VkSampler specular_sampler = tile_sampler;

            auto enqueue_texture = [&](const std::string &absolute_path,
                                       const bool srgb,
                                       const VkSampler sampler,
                                       const TextureCache::TextureKey::ChannelsHint channels =
                                               TextureCache::TextureKey::ChannelsHint::Auto) {
                if (absolute_path.empty() || !std::filesystem::exists(absolute_path))
                {
                    return;
                }

                TextureCache::TextureKey key{};
                key.kind = TextureCache::TextureKey::SourceKind::FilePath;
                key.path = absolute_path;
                key.srgb = srgb;
                key.mipmapped = true;
                key.channels = channels;

                const TextureCache::TextureHandle handle = textures->request(key, sampler);
                if (handle != TextureCache::InvalidHandle)
                {
                    _environment_texture_handles.push_back(handle);
                }
            };

            auto enqueue_cube_faces = [&](const std::string &dir,
                                          const bool srgb,
                                          const VkSampler sampler,
                                          const TextureCache::TextureKey::ChannelsHint channels,
                                          const bool allow_png_fallback) {
                if (dir.empty())
                {
                    return;
                }

                for (size_t face_index = 0; face_index < 6; ++face_index)
                {
                    const planet::CubeFace face = static_cast<planet::CubeFace>(face_index);
                    const std::string face_name = planet::cube_face_name(face);

                    std::string absolute = assets->assetPath(dir + "/" + face_name + ".ktx2");
                    if (allow_png_fallback && !std::filesystem::exists(absolute))
                    {
                        absolute = assets->assetPath(dir + "/" + face_name + ".png");
                    }

                    enqueue_texture(absolute, srgb, sampler, channels);
                }
            };

            for (const auto &cdef : _scenario_config.celestials)
            {
                if (!cdef.has_terrain)
                {
                    continue;
                }

                enqueue_cube_faces(cdef.albedo_dir,
                                   true,
                                   tile_sampler,
                                   TextureCache::TextureKey::ChannelsHint::Auto,
                                   false);
                enqueue_cube_faces(cdef.emission_dir,
                                   true,
                                   tile_sampler,
                                   TextureCache::TextureKey::ChannelsHint::Auto,
                                   true);
                enqueue_cube_faces(cdef.specular_dir,
                                   false,
                                   specular_sampler,
                                   TextureCache::TextureKey::ChannelsHint::R,
                                   true);
            }

            // Orbiter primitive material textures
            for (const auto &orbiter : _scenario_config.orbiters)
            {
                if (!orbiter.gltf_path.empty() || !orbiter.material.has_any_texture())
                {
                    continue;
                }

                const auto &mat = orbiter.material;
                enqueue_texture(assets->assetPath(mat.albedo), true, samplers->defaultLinear());
                enqueue_texture(assets->assetPath(mat.normal), false, samplers->defaultLinear());
                enqueue_texture(assets->assetPath(mat.metal_rough), false, samplers->defaultLinear());
                enqueue_texture(assets->assetPath(mat.occlusion), false, samplers->defaultLinear());
                enqueue_texture(assets->assetPath(mat.emissive), true, samplers->defaultLinear());
            }

            const auto &env = _scenario_config.environment;

            if (!env.rocket_plume_noise.empty())
            {
                enqueue_texture(assets->assetPath(env.rocket_plume_noise),
                                false,
                                samplers->defaultLinear(),
                                TextureCache::TextureKey::ChannelsHint::R);
            }
        }

        const auto &env = _scenario_config.environment;

        if (renderer && renderer->_renderPassManager)
        {
            if (env.has_atmosphere)
            {
                if (auto *atmosphere = renderer->_renderPassManager->getPass<AtmospherePass>())
                {
                    atmosphere->preload_cloud_textures();
                }
            }

            if (env.has_particles)
            {
                if (auto *particles = renderer->_renderPassManager->getPass<ParticlePass>())
                {
                    particles->preload_needed_textures();
                }
            }
        }

        const bool has_ibl = !env.ibl_specular.empty() || !env.ibl_diffuse.empty();
        if (has_ibl)
        {
            _ibl_paths.specularCube = assets->assetPath(env.ibl_specular);
            _ibl_paths.diffuseCube = assets->assetPath(env.ibl_diffuse);
            _ibl_paths.brdfLut = assets->assetPath(env.ibl_brdf_lut);
            _ibl_paths.background = assets->assetPath(env.ibl_background);

            if (ctx.api)
            {
                _ibl_warmup_requested = ctx.api->load_global_ibl(_ibl_paths);
            }
        }
    }

    void GameplayLoadingState::update_environment_warmup(GameStateContext &ctx)
    {
        VulkanEngine *renderer = ctx.renderer;
        TextureCache *textures = renderer ? renderer->_textureCache.get() : nullptr;

        if (textures && renderer && renderer->_context && !_environment_texture_handles.empty())
        {
            const uint32_t frame_index = renderer->_context->frameIndex;
            size_t resident_count = 0;

            for (const uint32_t handle : _environment_texture_handles)
            {
                textures->markUsed(handle, frame_index);
                if (textures->state(handle) == TextureCache::EntryState::Resident)
                {
                    ++resident_count;
                }
            }

            _environment_warmup_progress =
                    static_cast<float>(resident_count) /
                    static_cast<float>(_environment_texture_handles.size());
        }
        else if (_environment_texture_handles.empty())
        {
            _environment_warmup_progress = 1.0f;
        }

        if (_failed || !_ibl_warmup_requested || renderer == nullptr)
        {
            return;
        }

        if (!renderer->_pendingIBLRequest.active && !global_ibl_ready(*renderer))
        {
            _ibl_warmup_requested = false;
            _warning_text = "Environment IBL warmup failed. Continuing with fallback lighting.";
        }
    }

    void GameplayLoadingState::finalize_preload_jobs()
    {
        if (_jobs_finalized)
        {
            return;
        }

        std::vector<std::shared_ptr<LoadedGLTF>> retained_scenes;
        retained_scenes.reserve(_jobs.size());

        for (auto &job : _jobs)
        {
            if (job->worker.joinable())
            {
                job->worker.join();
            }

            if (!job->succeeded->load(std::memory_order_acquire))
            {
                _failed = true;
                if (_error_text.empty())
                {
                    _error_text = job->error.empty()
                                      ? "Gameplay preload failed."
                                      : job->error;
                }
                continue;
            }

            if (job->loaded_scene)
            {
                retained_scenes.push_back(job->loaded_scene);
            }

            if (job->loaded_height_faces)
            {
                planet::retain_preloaded_heightmap_faces(job->cache_key, std::move(*job->loaded_height_faces));
                job->loaded_height_faces.reset();
            }
        }

        if (_failed)
        {
            clear_preloaded_gltf_scenes();
            planet::clear_preloaded_heightmap_faces();
        }
        else
        {
            retain_preloaded_gltf_scenes(std::move(retained_scenes));
            _status_text = "Gameplay assets ready. Entering scene...";
        }

        _jobs_finalized = true;
    }

    void GameplayLoadingState::cancel_and_join_jobs()
    {
        for (auto &job : _jobs)
        {
            if (job->cancelled)
            {
                job->cancelled->store(true, std::memory_order_release);
            }
        }

        for (auto &job : _jobs)
        {
            if (job->worker.joinable())
            {
                job->worker.join();
            }
        }
    }

    float GameplayLoadingState::compute_progress()
    {
        if (_jobs.empty())
        {
            if (_ibl_warmup_requested || !_environment_texture_handles.empty())
            {
                _status_text = "Warming environment textures...";
                return 0.9f + 0.1f * std::clamp(_environment_warmup_progress, 0.0f, 1.0f);
            }
            return _jobs_finalized ? 1.0f : 0.9f;
        }

        float total_progress = 0.0f;
        bool status_set = false;

        for (const auto &job : _jobs)
        {
            const float job_fraction = job->progress
                                           ? std::clamp(job->progress->load(std::memory_order_relaxed), 0.0f, 1.0f)
                                           : 0.0f;
            float combined = job_fraction;
            if (job->finished && job->finished->load(std::memory_order_acquire))
            {
                combined = job->succeeded->load(std::memory_order_acquire)
                               ? 1.0f
                               : 1.0f;
            }

            total_progress += combined;

            if (!status_set)
            {
                if (!(job->finished && job->finished->load(std::memory_order_acquire)))
                {
                    if (job->kind == PreloadJob::Kind::TerrainHeightMaps)
                    {
                        _status_text = "Decoding " + job->display_name + " height maps (" + job->asset_path + ")";
                    }
                    else
                    {
                        _status_text = "Loading " + job->display_name + " (" + job->asset_path + ")";
                    }
                    status_set = true;
                }
            }
        }

        if (!status_set && !_failed)
        {
            if (_ibl_warmup_requested || !_environment_texture_handles.empty())
            {
                _status_text = "Finalizing gameplay scene and warming environment...";
            }
            else
            {
                _status_text = "Finalizing gameplay scene...";
            }
        }

        const float job_progress = total_progress / static_cast<float>(_jobs.size());
        if (_ibl_warmup_requested || !_environment_texture_handles.empty())
        {
            return job_progress * 0.9f + 0.1f * std::clamp(_environment_warmup_progress, 0.0f, 1.0f);
        }

        return job_progress;
    }

    bool GameplayLoadingState::all_jobs_finished() const
    {
        for (const auto &job : _jobs)
        {
            if (!job->finished || !job->finished->load(std::memory_order_acquire))
            {
                return false;
            }
        }
        return true;
    }

    bool GameplayLoadingState::environment_warmup_complete(GameStateContext &ctx) const
    {
        const bool textures_ready =
                _environment_texture_handles.empty() || _environment_warmup_progress >= 0.999f;
        if (!_ibl_warmup_requested)
        {
            return textures_ready;
        }

        const VulkanEngine *renderer = ctx.renderer;
        return textures_ready && renderer && global_ibl_ready(*renderer);
    }

    bool GameplayLoadingState::global_ibl_ready(const VulkanEngine &renderer) const
    {
        return renderer._iblManager &&
               !renderer._pendingIBLRequest.active &&
               renderer._hasGlobalIBL &&
               renderer._activeIBLVolume < 0 &&
               renderer._globalIBLPaths.specularCube == _ibl_paths.specularCube &&
               renderer._globalIBLPaths.diffuseCube == _ibl_paths.diffuseCube &&
               renderer._globalIBLPaths.brdfLut2D == _ibl_paths.brdfLut &&
               renderer._globalIBLPaths.background2D == _ibl_paths.background;
    }
} // namespace Game
