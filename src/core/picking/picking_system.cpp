#include "picking_system.h"

#include "core/context.h"
#include "core/device/device.h"
#include "core/device/images.h"
#include "core/device/swapchain.h"
#include "render/graph/graph.h"

#include "SDL2/SDL.h"
#include "SDL2/SDL_vulkan.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace
{
    using NodeNameLookup = std::unordered_map<const Node *, const std::string *>;

    NodeNameLookup build_node_name_lookup(const LoadedGLTF *scene)
    {
        NodeNameLookup lookup{};
        if (!scene)
        {
            return lookup;
        }

        lookup.reserve(scene->nodes.size());
        for (const auto &entry : scene->nodes)
        {
            if (!entry.second)
            {
                continue;
            }
            lookup[entry.second.get()] = &entry.first;
        }
        return lookup;
    }

    const std::string *find_node_name(const NodeNameLookup &lookup, const Node *node)
    {
        if (!node)
        {
            return nullptr;
        }
        auto it = lookup.find(node);
        return (it != lookup.end()) ? it->second : nullptr;
    }

    void populate_pick_node_hierarchy(const LoadedGLTF *scene, const Node *node, PickingSystem::PickInfo &out_pick)
    {
        out_pick.nodeName.clear();
        out_pick.nodeParentName.clear();
        out_pick.nodeChildren.clear();
        out_pick.nodePath.clear();

        if (!scene || !node)
        {
            return;
        }

        const NodeNameLookup lookup = build_node_name_lookup(scene);
        const std::string *node_name = find_node_name(lookup, node);
        if (!node_name)
        {
            return;
        }
        out_pick.nodeName = *node_name;

        if (const std::shared_ptr<Node> parent = node->parent.lock())
        {
            if (const std::string *parent_name = find_node_name(lookup, parent.get()))
            {
                out_pick.nodeParentName = *parent_name;
            }
        }

        out_pick.nodeChildren.reserve(node->children.size());
        for (const std::shared_ptr<Node> &child : node->children)
        {
            if (!child)
            {
                continue;
            }
            if (const std::string *child_name = find_node_name(lookup, child.get()))
            {
                out_pick.nodeChildren.push_back(*child_name);
            }
        }

        std::vector<std::string> reverse_path{};
        reverse_path.reserve(8);

        const Node *cursor = node;
        size_t guard = 0;
        constexpr size_t kMaxHierarchyDepth = 1024;
        while (cursor && guard < kMaxHierarchyDepth)
        {
            ++guard;
            const std::string *cursor_name = find_node_name(lookup, cursor);
            if (!cursor_name)
            {
                break;
            }
            reverse_path.push_back(*cursor_name);

            const std::shared_ptr<Node> parent = cursor->parent.lock();
            cursor = parent.get();
        }

        out_pick.nodePath.assign(reverse_path.rbegin(), reverse_path.rend());
    }

    Node *get_child_by_compact_index(Node *parent, size_t child_index)
    {
        if (!parent)
        {
            return nullptr;
        }

        size_t compact_index = 0;
        for (const std::shared_ptr<Node> &child : parent->children)
        {
            if (!child)
            {
                continue;
            }
            if (compact_index == child_index)
            {
                return child.get();
            }
            ++compact_index;
        }
        return nullptr;
    }

    bool is_direct_child(const Node *parent, const Node *candidate_child)
    {
        if (!parent || !candidate_child)
        {
            return false;
        }
        for (const std::shared_ptr<Node> &child : parent->children)
        {
            if (child && child.get() == candidate_child)
            {
                return true;
            }
        }
        return false;
    }
} // namespace

void PickingSystem::init(EngineContext *context)
{
    _context = context;
    _last_pick = {};
    _hover_pick = {};
    _drag_selection.clear();
    _mouse_pos_window = glm::vec2{-1.0f, -1.0f};
    _drag_state = {};
    _pending_pick = {};
    _pick_result_pending = false;
    _last_pick_object_id = 0;

    _pick_readback_buffer = {};
    if (_context && _context->getResources())
    {
        _pick_readback_buffer = _context->getResources()->create_buffer(
            sizeof(uint32_t),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VMA_MEMORY_USAGE_CPU_TO_GPU);
    }
}

void PickingSystem::cleanup()
{
    if (_pick_readback_buffer.buffer && _context && _context->getResources())
    {
        _context->getResources()->destroy_buffer(_pick_readback_buffer);
    }
    _pick_readback_buffer = {};

    _context = nullptr;
    _last_pick = {};
    _hover_pick = {};
    _drag_selection.clear();
    _pending_pick = {};
    _pick_result_pending = false;
    _last_pick_object_id = 0;
}

void PickingSystem::process_input(const InputSystem &input, bool ui_want_capture_mouse)
{
    if (_context == nullptr)
    {
        return;
    }

    float click_threshold_px = _settings.click_threshold_px;
    if (!std::isfinite(click_threshold_px) || click_threshold_px < 0.0f)
    {
        click_threshold_px = 0.0f;
    }

    const uint32_t select_button_mask = _settings.select_button_mask;
    auto is_select_button = [select_button_mask](MouseButton button) -> bool {
        const uint32_t bit = (1u << static_cast<uint32_t>(button));
        return (select_button_mask & bit) != 0u;
    };

    for (const InputEvent &event : input.events())
    {
        if (event.type == InputEvent::Type::MouseMove)
        {
            _mouse_pos_window = event.mouse_pos;
            if (_drag_state.button_down)
            {
                _drag_state.current = _mouse_pos_window;
                const glm::vec2 delta = _drag_state.current - _drag_state.start;
                if (!_drag_state.dragging &&
                    (std::abs(delta.x) > click_threshold_px || std::abs(delta.y) > click_threshold_px))
                {
                    _drag_state.dragging = true;
                }
            }
            continue;
        }

        if (event.type == InputEvent::Type::MouseButtonDown && is_select_button(event.mouse_button))
        {
            _mouse_pos_window = event.mouse_pos;

            if (!_settings.enabled)
            {
                continue;
            }

            if (_settings.require_cursor_normal && input.cursor_mode() != CursorMode::Normal)
            {
                continue;
            }

            if (_settings.respect_ui_capture_mouse && ui_want_capture_mouse)
            {
                continue;
            }

            if (_drag_state.button_down)
            {
                continue;
            }

            _drag_state.button_down = true;
            _drag_state.dragging = false;
            _drag_state.button = event.mouse_button;
            _drag_state.start = event.mouse_pos;
            _drag_state.current = _drag_state.start;
            continue;
        }

        if (event.type == InputEvent::Type::MouseButtonUp && _drag_state.button_down && event.mouse_button == _drag_state.button)
        {
            _mouse_pos_window = event.mouse_pos;

            const bool was_down = _drag_state.button_down;
            _drag_state.button_down = false;
            if (!was_down)
            {
                _drag_state.dragging = false;
                continue;
            }

            const glm::vec2 release_pos = event.mouse_pos;
            const glm::vec2 delta = release_pos - _drag_state.start;
            const bool moved_enough_for_drag = (std::abs(delta.x) > click_threshold_px || std::abs(delta.y) > click_threshold_px);

            SceneManager *scene = _context->scene;

            if (!_settings.enabled ||
                (_settings.require_cursor_normal && input.cursor_mode() != CursorMode::Normal) ||
                (_settings.respect_ui_capture_mouse && ui_want_capture_mouse))
            {
                _drag_state.dragging = false;
                continue;
            }

            const bool do_drag_select = _settings.enable_drag_select && moved_enough_for_drag;
            const bool do_click_select = _settings.enable_click_select && !do_drag_select;

            if (do_click_select)
            {
                if (_use_id_buffer_picking)
                {
                    _pending_pick.active = true;
                    _pending_pick.window_pos_swapchain = window_to_swapchain_pixels(release_pos);
                }
                else if (scene)
                {
                    RenderObject hit_object{};
                    WorldVec3 hit_pos{};
                    if (scene->pick(window_to_swapchain_pixels(release_pos), hit_object, hit_pos))
                    {
                        set_pick_from_hit(hit_object, hit_pos, _last_pick);
                        _last_pick_object_id = hit_object.objectID;
                    }
                    else if (_settings.clear_last_pick_on_miss)
                    {
                        clear_pick(_last_pick);
                        _last_pick_object_id = 0;
                    }
                }
            }
            else if (do_drag_select)
            {
                _drag_selection.clear();
                if (scene)
                {
                    std::vector<RenderObject> selected;
                    scene->selectRect(window_to_swapchain_pixels(_drag_state.start),
                                      window_to_swapchain_pixels(release_pos),
                                      selected);
                    _drag_selection.reserve(selected.size());
                    for (const RenderObject &obj : selected)
                    {
                        PickInfo info{};
                        glm::vec3 center_local = glm::vec3(obj.transform * glm::vec4(obj.bounds.origin, 1.0f));
                        set_pick_from_hit(obj, local_to_world(center_local, scene->get_world_origin()), info);
                        _drag_selection.push_back(std::move(info));
                    }
                }
            }

            _drag_state.dragging = false;
        }
    }

    // Safety: avoid stuck drag state if focus/SDL event is missed.
    if (_drag_state.button_down && !input.state().mouse_down(_drag_state.button))
    {
        _drag_state = {};
    }
}

void PickingSystem::update_hover(bool ui_want_capture_mouse)
{
    if (_context == nullptr || _context->scene == nullptr)
    {
        return;
    }

    if (!_settings.enabled || !_settings.enable_hover)
    {
        clear_pick(_hover_pick);
        return;
    }

    if (_settings.respect_ui_capture_mouse && ui_want_capture_mouse)
    {
        clear_pick(_hover_pick);
        return;
    }

    if (_settings.require_cursor_normal && _context->input && _context->input->cursor_mode() != CursorMode::Normal)
    {
        clear_pick(_hover_pick);
        return;
    }

    if (_mouse_pos_window.x < 0.0f || _mouse_pos_window.y < 0.0f)
    {
        clear_pick(_hover_pick);
        return;
    }

    RenderObject hover_obj{};
    WorldVec3 hover_pos{};
    if (_context->scene->pick(window_to_swapchain_pixels(_mouse_pos_window), hover_obj, hover_pos))
    {
        set_pick_from_hit(hover_obj, hover_pos, _hover_pick);
    }
    else
    {
        clear_pick(_hover_pick);
    }
}

void PickingSystem::begin_frame()
{
    if (!_pick_result_pending || !_pick_readback_buffer.buffer || _context == nullptr || _context->scene == nullptr)
    {
        return;
    }

    DeviceManager *dev = _context->getDevice();
    if (!dev)
    {
        return;
    }

    vmaInvalidateAllocation(dev->allocator(), _pick_readback_buffer.allocation, 0, sizeof(uint32_t));

    uint32_t picked_id = 0;
    if (_pick_readback_buffer.info.pMappedData)
    {
        picked_id = *reinterpret_cast<const uint32_t *>(_pick_readback_buffer.info.pMappedData);
    }

    if (picked_id == 0)
    {
        if (_settings.clear_last_pick_on_miss)
        {
            clear_pick(_last_pick);
            _last_pick_object_id = 0;
        }
    }
    else
    {
        RenderObject picked{};
        if (_context->scene->resolveObjectID(picked_id, picked))
        {
            _last_pick_object_id = picked_id;
            glm::vec3 fallback_local = glm::vec3(picked.transform[3]);
            WorldVec3 fallback_pos = local_to_world(fallback_local, _context->scene->get_world_origin());
            set_pick_from_hit(picked, fallback_pos, _last_pick);
        }
        else if (_settings.clear_last_pick_on_miss)
        {
            clear_pick(_last_pick);
            _last_pick_object_id = 0;
        }
    }

    _pick_result_pending = false;
}

void PickingSystem::register_id_buffer_readback(RenderGraph &graph,
                                                RGImageHandle id_buffer,
                                                VkExtent2D draw_extent,
                                                VkExtent2D swapchain_extent)
{
    if (!_use_id_buffer_picking || !_pending_pick.active || !id_buffer.valid() || !_pick_readback_buffer.buffer)
    {
        return;
    }

    if (draw_extent.width == 0 || draw_extent.height == 0 || swapchain_extent.width == 0 || swapchain_extent.height == 0)
    {
        _pending_pick.active = false;
        return;
    }

    glm::vec2 logical_pos{};
    if (!vkutil::map_window_to_letterbox_src(_pending_pick.window_pos_swapchain, draw_extent, swapchain_extent, logical_pos))
    {
        _pending_pick.active = false;
        return;
    }

    const uint32_t id_x = static_cast<uint32_t>(std::clamp(logical_pos.x, 0.0f, float(draw_extent.width - 1)));
    const uint32_t id_y = static_cast<uint32_t>(std::clamp(logical_pos.y, 0.0f, float(draw_extent.height - 1)));
    _pending_pick.id_coords = {id_x, id_y};

    RGImportedBufferDesc bd{};
    bd.name = "pick.readback";
    bd.buffer = _pick_readback_buffer.buffer;
    bd.size = sizeof(uint32_t);
    bd.currentStage = VK_PIPELINE_STAGE_2_NONE;
    bd.currentAccess = 0;
    RGBufferHandle pick_buf = graph.import_buffer(bd);

    const glm::uvec2 coords = _pending_pick.id_coords;
    graph.add_pass(
        "PickReadback",
        RGPassType::Transfer,
        [id_buffer, pick_buf](RGPassBuilder &builder, EngineContext *)
        {
            builder.read(id_buffer, RGImageUsage::TransferSrc);
            builder.write_buffer(pick_buf, RGBufferUsage::TransferDst);
        },
        [coords, id_buffer, pick_buf](VkCommandBuffer cmd, const RGPassResources &res, EngineContext *)
        {
            VkImage id_image = res.image(id_buffer);
            VkBuffer dst = res.buffer(pick_buf);
            if (id_image == VK_NULL_HANDLE || dst == VK_NULL_HANDLE) return;

            VkBufferImageCopy region{};
            region.bufferOffset = 0;
            region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            region.imageSubresource.mipLevel = 0;
            region.imageSubresource.baseArrayLayer = 0;
            region.imageSubresource.layerCount = 1;
            region.imageOffset = {static_cast<int32_t>(coords.x),
                                  static_cast<int32_t>(coords.y),
                                  0};
            region.imageExtent = {1, 1, 1};

            vkCmdCopyImageToBuffer(cmd,
                                   id_image,
                                   VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                   dst,
                                   1,
                                   &region);
        });

    _pick_result_pending = true;
    _pending_pick.active = false;
}

void PickingSystem::clear_owner_picks(RenderObject::OwnerType owner_type, const std::string &owner_name)
{
    if (_last_pick.valid && _last_pick.ownerType == owner_type && _last_pick.ownerName == owner_name)
    {
        clear_pick(_last_pick);
        _last_pick_object_id = 0;
    }
    if (_hover_pick.valid && _hover_pick.ownerType == owner_type && _hover_pick.ownerName == owner_name)
    {
        clear_pick(_hover_pick);
    }

    if (!_drag_selection.empty())
    {
        _drag_selection.erase(std::remove_if(_drag_selection.begin(),
                                             _drag_selection.end(),
                                             [&](const PickInfo &p) {
                                                 return p.valid && p.ownerType == owner_type && p.ownerName == owner_name;
                                             }),
                              _drag_selection.end());
    }
}

bool PickingSystem::move_last_pick_to_parent()
{
    if (!_last_pick.valid ||
        _last_pick.ownerType != RenderObject::OwnerType::GLTFInstance ||
        _last_pick.node == nullptr)
    {
        return false;
    }

    const std::shared_ptr<Node> parent = _last_pick.node->parent.lock();
    if (!parent)
    {
        return false;
    }
    return set_pick_to_gltf_node(_last_pick, parent.get());
}

bool PickingSystem::move_last_pick_to_child(size_t child_index)
{
    if (!_last_pick.valid ||
        _last_pick.ownerType != RenderObject::OwnerType::GLTFInstance ||
        _last_pick.node == nullptr)
    {
        return false;
    }

    Node *child = get_child_by_compact_index(_last_pick.node, child_index);
    if (!child)
    {
        return false;
    }
    return set_pick_to_gltf_node(_last_pick, child);
}

bool PickingSystem::move_last_pick_to_child(const std::string &child_name)
{
    if (!_last_pick.valid ||
        _last_pick.ownerType != RenderObject::OwnerType::GLTFInstance ||
        _last_pick.node == nullptr ||
        _last_pick.scene == nullptr ||
        child_name.empty())
    {
        return false;
    }

    auto child_it = _last_pick.scene->nodes.find(child_name);
    if (child_it == _last_pick.scene->nodes.end() || !child_it->second)
    {
        return false;
    }

    Node *child = child_it->second.get();
    if (!is_direct_child(_last_pick.node, child))
    {
        return false;
    }
    return set_pick_to_gltf_node(_last_pick, child);
}

glm::vec2 PickingSystem::window_to_swapchain_pixels(const glm::vec2 &window_pos) const
{
    if (_context == nullptr || _context->window == nullptr || _context->getSwapchain() == nullptr)
    {
        return window_pos;
    }

    int win_w = 0, win_h = 0;
    SDL_GetWindowSize(_context->window, &win_w, &win_h);

    int draw_w = 0, draw_h = 0;
    SDL_Vulkan_GetDrawableSize(_context->window, &draw_w, &draw_h);

    glm::vec2 scale{1.0f, 1.0f};
    if (win_w > 0 && win_h > 0 && draw_w > 0 && draw_h > 0)
    {
        scale.x = static_cast<float>(draw_w) / static_cast<float>(win_w);
        scale.y = static_cast<float>(draw_h) / static_cast<float>(win_h);
    }

    glm::vec2 drawable_pos{window_pos.x * scale.x, window_pos.y * scale.y};

    VkExtent2D drawable_extent{0, 0};
    if (draw_w > 0 && draw_h > 0)
    {
        drawable_extent.width = static_cast<uint32_t>(draw_w);
        drawable_extent.height = static_cast<uint32_t>(draw_h);
    }
    if ((drawable_extent.width == 0 || drawable_extent.height == 0) && _context->getSwapchain())
    {
        drawable_extent = _context->getSwapchain()->windowExtent();
    }

    VkExtent2D swap = _context->getSwapchain()->swapchainExtent();
    if (drawable_extent.width == 0 || drawable_extent.height == 0 || swap.width == 0 || swap.height == 0)
    {
        return drawable_pos;
    }

    const float sx = static_cast<float>(swap.width) / static_cast<float>(drawable_extent.width);
    const float sy = static_cast<float>(swap.height) / static_cast<float>(drawable_extent.height);
    return glm::vec2{drawable_pos.x * sx, drawable_pos.y * sy};
}

void PickingSystem::set_pick_from_hit(const RenderObject &hit_object, const WorldVec3 &hit_pos, PickInfo &out_pick)
{
    out_pick.mesh = hit_object.sourceMesh;
    out_pick.scene = hit_object.sourceScene;
    out_pick.node = hit_object.sourceNode;
    out_pick.ownerType = hit_object.ownerType;
    out_pick.ownerName = hit_object.ownerName;
    if (out_pick.ownerType == RenderObject::OwnerType::GLTFInstance)
    {
        populate_pick_node_hierarchy(out_pick.scene, out_pick.node, out_pick);
    }
    else
    {
        out_pick.nodeName.clear();
        out_pick.nodeParentName.clear();
        out_pick.nodeChildren.clear();
        out_pick.nodePath.clear();
    }
    out_pick.worldPos = hit_pos;
    out_pick.worldTransform = hit_object.transform;
    out_pick.firstIndex = hit_object.firstIndex;
    out_pick.indexCount = hit_object.indexCount;
    out_pick.surfaceIndex = hit_object.surfaceIndex;
    out_pick.valid = true;
}

bool PickingSystem::set_pick_to_gltf_node(PickInfo &pick, Node *target_node)
{
    if (_context == nullptr ||
        _context->scene == nullptr ||
        !pick.valid ||
        pick.ownerType != RenderObject::OwnerType::GLTFInstance ||
        pick.ownerName.empty() ||
        pick.scene == nullptr ||
        target_node == nullptr)
    {
        return false;
    }

    PickInfo updated = pick;
    updated.mesh = nullptr;
    updated.node = target_node;
    updated.firstIndex = 0;
    updated.indexCount = 0;
    updated.surfaceIndex = 0;

    populate_pick_node_hierarchy(updated.scene, updated.node, updated);
    if (updated.nodeName.empty())
    {
        return false;
    }

    glm::mat4 node_world{1.0f};
    if (_context->scene->getGLTFInstanceNodeWorldTransform(updated.ownerName, updated.nodeName, node_world))
    {
        updated.worldTransform = node_world;
        updated.worldPos = WorldVec3(glm::dvec3(node_world[3]));
    }
    else
    {
        // Fallback to node-local cached world transform if the instance lookup fails.
        updated.worldTransform = target_node->worldTransform;
        updated.worldPos = WorldVec3(glm::dvec3(updated.worldTransform[3]));
    }
    updated.valid = true;

    pick = std::move(updated);
    _last_pick_object_id = 0;
    return true;
}

void PickingSystem::clear_pick(PickInfo &pick)
{
    pick.mesh = nullptr;
    pick.scene = nullptr;
    pick.node = nullptr;
    pick.ownerType = RenderObject::OwnerType::None;
    pick.ownerName.clear();
    pick.nodeName.clear();
    pick.nodeParentName.clear();
    pick.nodeChildren.clear();
    pick.nodePath.clear();
    pick.worldPos = WorldVec3{0.0, 0.0, 0.0};
    pick.worldTransform = glm::mat4(1.0f);
    pick.indexCount = 0;
    pick.firstIndex = 0;
    pick.surfaceIndex = 0;
    pick.valid = false;
}
