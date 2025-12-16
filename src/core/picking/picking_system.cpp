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

void PickingSystem::process_event(const SDL_Event &event, bool ui_want_capture_mouse)
{
    if (_context == nullptr)
    {
        return;
    }

    if (event.type == SDL_MOUSEMOTION)
    {
        _mouse_pos_window = glm::vec2{static_cast<float>(event.motion.x),
                                      static_cast<float>(event.motion.y)};
        if (_drag_state.button_down)
        {
            _drag_state.current = _mouse_pos_window;
            _drag_state.dragging = true;
        }
        return;
    }

    if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT)
    {
        if (ui_want_capture_mouse)
        {
            return;
        }

        _drag_state.button_down = true;
        _drag_state.dragging = false;
        _drag_state.start = glm::vec2{static_cast<float>(event.button.x),
                                      static_cast<float>(event.button.y)};
        _drag_state.current = _drag_state.start;
        return;
    }

    if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT)
    {
        const bool was_down = _drag_state.button_down;
        _drag_state.button_down = false;
        if (!was_down)
        {
            _drag_state.dragging = false;
            return;
        }

        glm::vec2 release_pos{static_cast<float>(event.button.x),
                              static_cast<float>(event.button.y)};

        constexpr float click_threshold = 3.0f;
        glm::vec2 delta = release_pos - _drag_state.start;
        bool treat_as_click = !_drag_state.dragging &&
                              std::abs(delta.x) < click_threshold &&
                              std::abs(delta.y) < click_threshold;

        SceneManager *scene = _context->scene;

        if (treat_as_click)
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
                else
                {
                    clear_pick(_last_pick);
                    _last_pick_object_id = 0;
                }
            }
        }
        else
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
                    info.mesh = obj.sourceMesh;
                    info.scene = obj.sourceScene;
                    info.node = obj.sourceNode;
                    info.ownerType = obj.ownerType;
                    info.ownerName = obj.ownerName;
                    glm::vec3 center_local = glm::vec3(obj.transform * glm::vec4(obj.bounds.origin, 1.0f));
                    info.worldPos = local_to_world(center_local, scene->get_world_origin());
                    info.worldTransform = obj.transform;
                    info.firstIndex = obj.firstIndex;
                    info.indexCount = obj.indexCount;
                    info.surfaceIndex = obj.surfaceIndex;
                    info.valid = true;
                    _drag_selection.push_back(std::move(info));
                }
            }
        }

        _drag_state.dragging = false;
    }
}

void PickingSystem::update_hover()
{
    if (_context == nullptr || _context->scene == nullptr)
    {
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
        clear_pick(_last_pick);
        _last_pick_object_id = 0;
    }
    else
    {
        _last_pick_object_id = picked_id;
        RenderObject picked{};
        if (_context->scene->resolveObjectID(picked_id, picked))
        {
            glm::vec3 fallback_local = glm::vec3(picked.transform[3]);
            WorldVec3 fallback_pos = local_to_world(fallback_local, _context->scene->get_world_origin());
            set_pick_from_hit(picked, fallback_pos, _last_pick);
        }
        else
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
    out_pick.worldPos = hit_pos;
    out_pick.worldTransform = hit_object.transform;
    out_pick.firstIndex = hit_object.firstIndex;
    out_pick.indexCount = hit_object.indexCount;
    out_pick.surfaceIndex = hit_object.surfaceIndex;
    out_pick.valid = true;
}

void PickingSystem::clear_pick(PickInfo &pick)
{
    pick.mesh = nullptr;
    pick.scene = nullptr;
    pick.node = nullptr;
    pick.ownerType = RenderObject::OwnerType::None;
    pick.ownerName.clear();
    pick.worldPos = WorldVec3{0.0, 0.0, 0.0};
    pick.worldTransform = glm::mat4(1.0f);
    pick.indexCount = 0;
    pick.firstIndex = 0;
    pick.surfaceIndex = 0;
    pick.valid = false;
}
