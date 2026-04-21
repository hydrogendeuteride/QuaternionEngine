#pragma once

#include <core/types.h>
#include <core/input/input_system.h>
#include <core/world.h>
#include <core/device/resource.h>

#include <scene/vk_scene.h>

#include <glm/vec2.hpp>
#include <glm/mat4x4.hpp>

#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

class EngineContext;
class RenderGraph;
struct RGImageHandle;

class PickingSystem
{
public:
    enum class SelectionLevel : uint8_t
    {
        None = 0,
        Object,
        Member,
        Node,
        Primitive,
    };

    struct Settings
    {
        // Master toggle for all picking behavior (click, drag, hover).
        bool enabled = true;

        // Click/drag is driven by these buttons (bit i => MouseButton with value i).
        // Default: Left click.
        uint32_t select_button_mask = (1u << static_cast<uint32_t>(MouseButton::Left));

        // Maximum cursor travel (in window pixels) to treat press→release as a click.
        float click_threshold_px = 3.0f;

        bool enable_hover = true;
        bool enable_click_select = true;
        bool enable_drag_select = true;

        // Hover outlines usually only need object identity, not exact triangle identity.
        // Keep click selection precise, but let hover stop BVH traversal at a bounded depth.
        bool use_fast_hover_mesh_bvh = true;
        uint32_t fast_hover_mesh_bvh_max_depth = 8;

        // Avoid repeating the same expensive CPU hover miss every frame while
        // the cursor and camera are stationary. 0 disables this cache.
        uint32_t stationary_hover_miss_skip_frames = 5;

        // When the cursor itself is stationary, refresh hover at a lower
        // cadence even if the camera/scene is still moving. Mouse motion always
        // forces an immediate hover update. 0 disables throttling.
        uint32_t stationary_hover_update_interval_frames = 15;

        // When true, ignore mouse interactions while UI wants mouse capture.
        bool respect_ui_capture_mouse = true;

        // When true, disable picking when the cursor is captured/relative.
        // (Useful to prevent selection while in FPS-style camera control.)
        bool require_cursor_normal = false;

        // When true, clear last pick on a click miss (CPU raycast mode only).
        bool clear_last_pick_on_miss = true;

        // --------------------------------------------------------------------
        // Line / Polyline Picking (CPU)
        // --------------------------------------------------------------------
        // Used for thin debug visuals (e.g. orbit plot) that don't exist as scene meshes.
        // Tolerance is expressed in *logical render pixels* (post-letterbox mapping).
        bool enable_line_picking = true;
        bool enable_line_hover = false;
        float line_pick_radius_px = 15.0f;
    };

    struct PickInfo
    {
        enum class Kind : uint8_t
        {
            None = 0,
            SceneObject,
            Line,
        };

        MeshAsset *mesh = nullptr;
        LoadedGLTF *scene = nullptr;
        Node *node = nullptr;
        RenderObject::OwnerType ownerType = RenderObject::OwnerType::None;
        std::string ownerName;
        std::string objectName;
        std::string memberName;
        // Populated for glTF picks when node identity can be resolved from scene->nodes.
        std::string nodeName;
        std::string nodeParentName;
        std::vector<std::string> nodeChildren;
        std::vector<std::string> nodePath;
        WorldVec3 worldPos{0.0, 0.0, 0.0};
        glm::mat4 worldTransform{1.0f};
        uint32_t indexCount = 0;
        uint32_t firstIndex = 0;
        uint32_t surfaceIndex = 0;
        // For Kind::Line picks (e.g. orbit polyline), this is the interpolated time at the hit point if provided.
        double time_s = std::numeric_limits<double>::quiet_NaN();
        Kind kind = Kind::None;
        SelectionLevel selectionLevel = SelectionLevel::None;
        bool valid = false;
    };

    struct OwnerBindingView
    {
        std::string_view object_name{};
        std::string_view member_name{};
    };

    void init(EngineContext *context);

    void cleanup();

    // Consume per-frame input events (mouse click, drag, release).
    void process_input(const InputSystem &input, bool ui_want_capture_mouse);

    void update_hover(bool ui_want_capture_mouse = false);

    // Called after the per-frame fence is waited to resolve async ID-buffer picks.
    void begin_frame();

    // Called during RenderGraph build after the ID buffer is available.
    void register_id_buffer_readback(RenderGraph &graph,
                                     RGImageHandle id_buffer,
                                     VkExtent2D draw_extent,
                                     VkExtent2D swapchain_extent);

    Settings &settings() { return _settings; }
    const Settings &settings() const { return _settings; }

    const PickInfo &last_pick() const { return _last_pick; }
    const PickInfo &hover_pick() const { return _hover_pick; }
    const std::vector<PickInfo> &drag_selection() const { return _drag_selection; }

    uint32_t last_pick_object_id() const { return _last_pick_object_id; }

    bool use_id_buffer_picking() const { return _use_id_buffer_picking; }
    void set_use_id_buffer_picking(bool enabled) { _use_id_buffer_picking = enabled; }

    bool debug_draw_bvh() const { return _debug_draw_bvh; }
    void set_debug_draw_bvh(bool enabled) { _debug_draw_bvh = enabled; }

    void clear_owner_picks(RenderObject::OwnerType owner_type, const std::string &owner_name);
    void clear_all_owner_bindings();

    void set_owner_binding(RenderObject::OwnerType owner_type,
                           const std::string &owner_name,
                           const std::string &object_name,
                           const std::string &member_name = {});
    void clear_owner_binding(RenderObject::OwnerType owner_type, const std::string &owner_name);
    bool get_owner_binding(RenderObject::OwnerType owner_type,
                           const std::string &owner_name,
                           std::string &out_object_name,
                           std::string &out_member_name) const;
    OwnerBindingView resolve_owner_binding(RenderObject::OwnerType owner_type,
                                           const std::string &owner_name) const;

    struct LinePickSegmentData
    {
        WorldVec3 a_world{0.0, 0.0, 0.0};
        WorldVec3 b_world{0.0, 0.0, 0.0};
        double a_time_s = std::numeric_limits<double>::quiet_NaN();
        double b_time_s = std::numeric_limits<double>::quiet_NaN();
    };

    // --------------------------------------------------------------------
    // Custom line/polyline pick registration (CPU-only)
    // --------------------------------------------------------------------
    // These pickables are not part of SceneManager::DrawContext; call from gameplay/editor
    // code to make thin debug polylines interactable (e.g., orbit plots).
    void clear_line_picks();
    uint32_t add_line_pick_group(std::string owner_name);
    void add_line_pick_segment(uint32_t group_id,
                               const WorldVec3 &a_world,
                               const WorldVec3 &b_world,
                               double a_time_s = std::numeric_limits<double>::quiet_NaN(),
                               double b_time_s = std::numeric_limits<double>::quiet_NaN());
    void add_line_pick_segments(uint32_t group_id, std::span<const LinePickSegmentData> segments);

    PickInfo *mutable_last_pick() { return &_last_pick; }
    PickInfo *mutable_hover_pick() { return &_hover_pick; }

    // Selection-level helpers. The glTF node traversal helpers remain available
    // for legacy tooling, but object/member/primitive selection is preferred.
    bool move_last_pick_to_parent();
    bool move_last_pick_to_child(size_t child_index = 0);
    bool move_last_pick_to_child(const std::string &child_name);
    bool set_last_pick_selection_level(SelectionLevel level);
    bool select_last_pick_object();
    bool select_last_pick_member();

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
        MouseButton button = MouseButton::Left;
        glm::vec2 start{0.0f};
        glm::vec2 current{0.0f};
    };

    glm::vec2 window_to_swapchain_pixels(const glm::vec2 &window_pos) const;

    struct CameraRay
    {
        WorldVec3 origin_world{0.0, 0.0, 0.0};
        WorldVec3 camera_world{0.0, 0.0, 0.0};
        glm::dvec3 origin_local{0.0, 0.0, 0.0};
        glm::dvec3 dir_local{0.0, 0.0, -1.0};
        double fov_y_rad{0.0};
        double viewport_height_px{0.0};
    };

    bool compute_camera_ray(const glm::vec2 &window_pos, CameraRay &out_ray) const;
    bool pick_line_at_window_pos(const glm::vec2 &window_pos, PickInfo &out_pick, double &out_depth_m) const;

    void set_pick_from_hit(const RenderObject &hit_object, const WorldVec3 &hit_pos, PickInfo &out_pick);
    bool set_pick_to_gltf_node(PickInfo &pick, Node *target_node);

    void clear_pick(PickInfo &pick) const;

    EngineContext *_context = nullptr;

    struct LinePickGroup
    {
        std::string owner_name;
    };

    struct OwnerBinding
    {
        std::string object_name;
        std::string member_name;
    };

    struct LinePickSegment
    {
        uint32_t group_id = 0;
        LinePickSegmentData data{};
    };

    struct LinePickBatch
    {
        uint32_t group_id = 0;
        const LinePickSegmentData *segments = nullptr;
        size_t count = 0;
    };

    struct HoverMissCache
    {
        bool valid = false;
        uint32_t skipped_frames = 0;
        glm::vec2 mouse_pos_window{0.0f, 0.0f};
        WorldVec3 camera_world{0.0, 0.0, 0.0};
        glm::quat camera_orientation{1.0f, 0.0f, 0.0f, 0.0f};
        float camera_fov = 0.0f;
        WorldVec3 origin_world{0.0, 0.0, 0.0};
        VkExtent2D logical_extent{0, 0};
        VkExtent2D swapchain_extent{0, 0};
        bool line_hover_enabled = false;
    };

    PickInfo _last_pick{};
    PickInfo _hover_pick{};
    std::vector<PickInfo> _drag_selection{};

    std::vector<LinePickGroup> _line_pick_groups{};
    std::vector<LinePickSegment> _owned_line_pick_segments{};
    std::vector<LinePickBatch> _line_pick_batches{};
    std::unordered_map<std::string, OwnerBinding> _gltf_owner_bindings{};
    std::unordered_map<std::string, OwnerBinding> _mesh_owner_bindings{};

    glm::vec2 _mouse_pos_window{-1.0f, -1.0f};
    DragState _drag_state{};

    Settings _settings{};
    bool _use_id_buffer_picking = false;
    bool _debug_draw_bvh = false;

    uint32_t _last_pick_object_id = 0;
    PickRequest _pending_pick{};
    bool _pick_result_pending = false;

    AllocatedBuffer _pick_readback_buffer{};
    HoverMissCache _hover_miss_cache{};
    uint64_t _mouse_motion_generation = 0;
    uint64_t _last_hover_mouse_motion_generation = std::numeric_limits<uint64_t>::max();
    uint64_t _hover_frame_counter = 0;
    uint64_t _last_hover_update_frame = 0;
};
