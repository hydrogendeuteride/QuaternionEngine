#include "game_runtime.h"
#include "core/engine.h"
#include "core/input/input_system.h"
#include "core/ui/imgui_system.h"
#include "scene/vk_scene.h"

#include "SDL2/SDL.h"

#include <thread>
#include <chrono>

namespace GameRuntime
{
    Runtime::Runtime(VulkanEngine *renderer)
        : _renderer(renderer)
    {
        _api = std::make_unique<GameAPI::Engine>(renderer);
    }

    Runtime::~Runtime() = default;

    void Runtime::set_physics_world(IPhysicsWorld *physics)
    {
        _physics = physics;
    }

    void Runtime::set_audio_system(IAudioSystem *audio)
    {
        _audio = audio;
    }

    void Runtime::update_audio_listener()
    {
        if (!_audio || !_renderer || !_renderer->_sceneManager)
        {
            return;
        }

        auto &cam = _renderer->_sceneManager->getMainCamera();
        glm::vec3 pos = _renderer->_sceneManager->get_camera_local_position();

        glm::vec3 forward = glm::normalize(cam.orientation * glm::vec3(0.0f, 0.0f, -1.0f));
        glm::vec3 up = glm::normalize(cam.orientation * glm::vec3(0.0f, 1.0f, 0.0f));

        _audio->set_listener(pos, forward, up);
    }

    void Runtime::run(IGameCallbacks *game)
    {
        if (!game || !_renderer)
        {
            return;
        }

        _quit_requested = false;

        game->on_init(*this);

        while (!_quit_requested)
        {
            // --- Begin frame: time, input --- //
            _time.begin_frame();

            InputSystem *input = _renderer->input();
            if (input)
            {
                input->begin_frame();
                input->pump_events();

                if (input->quit_requested())
                {
                    _quit_requested = true;
                }

                _renderer->freeze_rendering = input->window_minimized();

                if (input->resize_requested())
                {
                    _renderer->resize_requested = true;
                    input->clear_resize_request();
                }
            }

            // --- Process UI and input capture --- //
            const bool ui_capture_mouse = _renderer->ui() && _renderer->ui()->want_capture_mouse();
            const bool ui_capture_keyboard = _renderer->ui() && _renderer->ui()->want_capture_keyboard();

            // Native events to UI and picking
            if (input)
            {
                struct DispatchCtx
                {
                    VulkanEngine *engine;
                    bool ui_capture_mouse;
                } ctx{_renderer, ui_capture_mouse};

                input->for_each_native_event([](void *user, InputSystem::NativeEventView view) {
                    auto *c = static_cast<DispatchCtx *>(user);
                    if (!c || !c->engine || view.backend != InputSystem::NativeBackend::SDL2 || view.data == nullptr)
                    {
                        return;
                    }
                    const SDL_Event &e = *static_cast<const SDL_Event *>(view.data);
                    if (c->engine->ui())
                    {
                        c->engine->ui()->process_event(e);
                    }
                    if (c->engine->picking())
                    {
                        c->engine->picking()->process_event(e, c->ui_capture_mouse);
                    }
                }, &ctx);
            }

            // --- Camera input (if not captured by UI) --- //
            if (_renderer->_sceneManager && input)
            {
                _renderer->_sceneManager->getCameraRig().process_input(*input, ui_capture_keyboard, ui_capture_mouse);
            }

            // --- Throttle when minimized --- //
            if (_renderer->freeze_rendering)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // --- Handle resize --- //
            if (_renderer->resize_requested)
            {
                if (_renderer->_swapchainManager)
                {
                    _renderer->_swapchainManager->resize_swapchain(_renderer->_window);
                    if (_renderer->ui())
                    {
                        _renderer->ui()->on_swapchain_recreated();
                    }
                    _renderer->resize_requested = false;
                }
            }

            // --- Fixed update loop --- //
            while (_time.consume_fixed_step())
            {
                game->on_fixed_update(_time.fixed_delta_time());

                if (_physics)
                {
                    _physics->step(_time.fixed_delta_time());
                }
            }

            // --- Variable update --- //
            game->on_update(_time.delta_time());

            // --- Audio listener update --- //
            update_audio_listener();
            if (_audio)
            {
                _audio->update();
            }

            // --- Wait for GPU and prepare frame --- //
            VK_CHECK(vkWaitForFences(_renderer->_deviceManager->device(), 1,
                &_renderer->get_current_frame()._renderFence, true, 1000000000));

            if (_renderer->_rayManager)
            {
                _renderer->_rayManager->flushPendingDeletes();
                _renderer->_rayManager->pump_blas_builds(1);
            }

            // Commit any completed async IBL load now that the GPU is idle.
            if (_renderer->_iblManager && _renderer->_pendingIBLRequest.active)
            {
                IBLManager::AsyncResult iblRes = _renderer->_iblManager->pump_async();
                if (iblRes.completed)
                {
                    if (iblRes.success)
                    {
                        if (_renderer->_pendingIBLRequest.targetVolume >= 0)
                        {
                            _renderer->_activeIBLVolume = _renderer->_pendingIBLRequest.targetVolume;
                        }
                        else
                        {
                            _renderer->_activeIBLVolume = -1;
                            _renderer->_hasGlobalIBL = true;
                        }
                    }
                    else
                    {
                        fmt::println("[Runtime] Warning: async IBL load failed (specular='{}')",
                                     _renderer->_pendingIBLRequest.paths.specularCube);
                    }
                    _renderer->_pendingIBLRequest.active = false;
                }
            }

            // --- Flush per-frame resources --- ///
            _renderer->get_current_frame()._deletionQueue.flush();
            if (_renderer->_renderGraph)
            {
                _renderer->_renderGraph->resolve_timings();
            }
            _renderer->get_current_frame()._frameDescriptors.clear_pools(_renderer->_deviceManager->device());

            // --- ImGui --- //
            if (_renderer->ui())
            {
                _renderer->ui()->begin_frame();
                _renderer->ui()->end_frame();
            }

            // --- Draw --- //
            _renderer->draw();

            // --- Update frame stats --- //
            _renderer->stats.frametime = _time.delta_time() * 1000.0f;
        }

        // Call game shutdown
        game->on_shutdown();
    }
} // namespace GameRuntime
