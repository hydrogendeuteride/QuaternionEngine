#pragma once

#include <core/types.h>
#include <core/world.h>
#include <core/device/resource.h>

#include <scene/vk_scene.h>

#include <glm/vec2.hpp>
#include <glm/mat4x4.hpp>

#include <string>
#include <vector>

union SDL_Event;
class EngineContext;
class RenderGraph;
struct RGImageHandle;

class PickingSystem
{
public:
    struct PickInfo
    {
        MeshAsset *mesh = nullptr;
        LoadedGLTF *scene = nullptr;
        Node *node = nullptr;
        RenderObject::OwnerType ownerType = RenderObject::OwnerType::None;
        std::string ownerName;
        WorldVec3 worldPos{0.0, 0.0, 0.0};
        glm::mat4 worldTransform{1.0f};
        uint32_t indexCount = 0;
        uint32_t firstIndex = 0;
        uint32_t surfaceIndex = 0;
        bool valid = false;
    };

    void init(EngineContext *context);
    void cleanup();

    void process_event(const SDL_Event &event, bool ui_want_capture_mouse);
    void update_hover();

    // Called after the per-frame fence is waited to resolve async ID-buffer picks.
    void begin_frame();

    // Called during RenderGraph build after the ID buffer is available.
    void register_id_buffer_readback(RenderGraph &graph,
                                    RGImageHandle id_buffer,
                                    VkExtent2D draw_extent,
                                    VkExtent2D swapchain_extent);

    const PickInfo &last_pick() const { return _last_pick; }
    const PickInfo &hover_pick() const { return _hover_pick; }
    const std::vector<PickInfo> &drag_selection() const { return _drag_selection; }

    uint32_t last_pick_object_id() const { return _last_pick_object_id; }

    bool use_id_buffer_picking() const { return _use_id_buffer_picking; }
    void set_use_id_buffer_picking(bool enabled) { _use_id_buffer_picking = enabled; }

    bool debug_draw_bvh() const { return _debug_draw_bvh; }
    void set_debug_draw_bvh(bool enabled) { _debug_draw_bvh = enabled; }

    void clear_owner_picks(RenderObject::OwnerType owner_type, const std::string &owner_name);

    PickInfo *mutable_last_pick() { return &_last_pick; }
    PickInfo *mutable_hover_pick() { return &_hover_pick; }

private:
    struct PickRequest
    {
        bool active = false;
        glm::vec2 window_pos_swapchain{0.0f};
        glm::uvec2 id_coords{0, 0};
    };

    struct DragState
    {
        bool dragging = false;
        bool button_down = false;
        glm::vec2 start{0.0f};
        glm::vec2 current{0.0f};
    };

    glm::vec2 window_to_swapchain_pixels(const glm::vec2 &window_pos) const;
    void set_pick_from_hit(const RenderObject &hit_object, const WorldVec3 &hit_pos, PickInfo &out_pick);
    void clear_pick(PickInfo &pick);

    EngineContext *_context = nullptr;

    PickInfo _last_pick{};
    PickInfo _hover_pick{};
    std::vector<PickInfo> _drag_selection{};

    glm::vec2 _mouse_pos_window{-1.0f, -1.0f};
    DragState _drag_state{};

    bool _use_id_buffer_picking = false;
    bool _debug_draw_bvh = false;

    uint32_t _last_pick_object_id = 0;
    PickRequest _pending_pick{};
    bool _pick_result_pending = false;

    AllocatedBuffer _pick_readback_buffer{};
};
