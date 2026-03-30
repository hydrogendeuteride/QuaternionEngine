#include <algorithm>
#include <cmath>

#include <glm/gtx/matrix_decompose.hpp>

#include "vk_loader.h"

#include "core/assets/texture_cache.h"
#include "core/engine.h"
#include "physics/gltf_collider_parser.h"

void LoadedGLTF::Draw(const glm::mat4 &topMatrix, DrawContext &ctx)
{
    // create renderables from the scenenodes
    for (auto &n: topNodes)
    {
        n->Draw(topMatrix, ctx);
    }
}

std::shared_ptr<Node> LoadedGLTF::getNode(const std::string &name)
{
    auto it = nodes.find(name);
    return (it != nodes.end()) ? it->second : nullptr;
}

void LoadedGLTF::refreshAllTransforms()
{
    for (auto &n: topNodes)
    {
        if (n)
        {
            n->refreshTransform(glm::mat4{1.f});
        }
    }
}

void LoadedGLTF::build_colliders_from_markers(bool clear_existing)
{
    Physics::build_colliders_from_markers(collider_compounds, *this, clear_existing);
}

void LoadedGLTF::build_mesh_colliders_from_markers(bool clear_existing)
{
    Physics::build_mesh_colliders_from_markers(collider_mesh_instances, *this, clear_existing);
}

void LoadedGLTF::build_colliders_from_sidecar(const LoadedGLTF &sidecar, bool clear_existing)
{
    std::unordered_set<std::string_view> dst_names;
    dst_names.reserve(nodes.size());
    for (const auto &[name, ptr] : nodes)
    {
        (void) ptr;
        dst_names.insert(name);
    }

    Physics::build_colliders_from_sidecar(collider_compounds, sidecar, dst_names, clear_existing);
}

void LoadedGLTF::build_mesh_colliders_from_sidecar(const LoadedGLTF &sidecar, bool clear_existing)
{
    std::unordered_set<std::string_view> dst_names;
    dst_names.reserve(nodes.size());
    for (const auto &[name, ptr] : nodes)
    {
        (void) ptr;
        dst_names.insert(name);
    }

    Physics::build_mesh_colliders_from_sidecar(collider_mesh_instances, sidecar, dst_names, clear_existing);
}

void LoadedGLTF::apply_collider_child_mass_overrides(const std::unordered_map<std::string, float> &overrides)
{
    if (overrides.empty())
    {
        return;
    }

    for (auto &[owner_name, compound] : collider_compounds)
    {
        (void) owner_name;
        for (Physics::CompoundShapeChild &child : compound.children)
        {
            if (child.name.empty())
            {
                continue;
            }

            auto it = overrides.find(child.name);
            if (it == overrides.end())
            {
                continue;
            }

            const float mass = it->second;
            if (!std::isfinite(mass) || mass <= 0.0f)
            {
                Logger::warn("[GLTF][Colliders] override for '{}' has invalid mass {}; ignoring", child.name, mass);
                continue;
            }

            child.mass = mass;
        }
    }
}

void LoadedGLTF::ensureRestTransformsCached()
{
    if (_restTransformsCached)
    {
        return;
    }

    _restTransformsCached = true;
    _restTransforms.clear();
    _restTransforms.reserve(nodes.size());

    for (auto &[name, nodePtr] : nodes)
    {
        if (!nodePtr)
        {
            continue;
        }

        RestNodeTransform rest{};
        rest.localMatrix = nodePtr->localTransform;
        rest.hasTRS = nodePtr->hasTRS;
        rest.translation = nodePtr->translation;
        rest.rotation = glm::normalize(nodePtr->rotation);
        rest.scale = nodePtr->scale;

        if (!rest.hasTRS)
        {
            // Decompose so partial-channel animations (e.g. translation-only) have a stable base for R/S.
            glm::vec3 skew(0.0f);
            glm::vec4 perspective(0.0f);
            glm::vec3 scale(1.0f);
            glm::quat orientation(1.0f, 0.0f, 0.0f, 0.0f);
            glm::vec3 translation(0.0f);
            if (glm::decompose(rest.localMatrix, scale, orientation, translation, skew, perspective))
            {
                rest.translation = translation;
                rest.rotation = glm::normalize(orientation);
                rest.scale = scale;
            }
        }

        _restTransforms.emplace(nodePtr.get(), rest);
    }
}

void LoadedGLTF::restoreNodeToRest(Node &node) const
{
    auto it = _restTransforms.find(&node);
    if (it == _restTransforms.end())
    {
        return;
    }

    const RestNodeTransform &rest = it->second;
    node.localTransform = rest.localMatrix;
    node.translation = rest.translation;
    node.rotation = rest.rotation;
    node.scale = rest.scale;
    node.hasTRS = rest.hasTRS;
}

void LoadedGLTF::setActiveAnimation(AnimationState &state, int index, bool resetTime)
{
    if (animations.empty())
    {
        state.activeAnimation = -1;
        state.blending = false;
        state.blendFromAnimation = -1;
        return;
    }

    if (index < 0)
    {
        state.activeAnimation = -1;
        state.blending = false;
        state.blendFromAnimation = -1;
        if (resetTime)
        {
            state.animationTime = 0.0f;
        }
        return;
    }

    if (index >= static_cast<int>(animations.size()))
    {
        index = 0;
    }

    state.activeAnimation = index;
    state.blending = false;
    state.blendFromAnimation = -1;
    if (resetTime)
    {
        state.animationTime = 0.0f;
    }
}

void LoadedGLTF::setActiveAnimation(AnimationState &state, const std::string &name, bool resetTime)
{
    for (size_t i = 0; i < animations.size(); ++i)
    {
        if (animations[i].name == name)
        {
            setActiveAnimation(state, static_cast<int>(i), resetTime);
            return;
        }
    }
}

void LoadedGLTF::transitionAnimation(AnimationState &state, int index, float blendDurationSeconds, bool resetTime)
{
    if (animations.empty())
    {
        setActiveAnimation(state, -1, resetTime);
        return;
    }

    if (index < 0)
    {
        // For now, treat disable as an immediate stop (restoring happens in updateAnimation).
        setActiveAnimation(state, -1, resetTime);
        return;
    }

    if (index >= static_cast<int>(animations.size()))
    {
        index = 0;
    }

    const bool canBlend =
        (blendDurationSeconds > 0.0f) &&
        (state.activeAnimation >= 0) &&
        (state.activeAnimation < static_cast<int>(animations.size())) &&
        (state.activeAnimation != index);

    if (!canBlend)
    {
        setActiveAnimation(state, index, resetTime);
        return;
    }

    state.blending = true;
    state.blendFromAnimation = state.activeAnimation;
    state.blendFromTime = state.animationTime;
    state.blendFromLoop = state.animationLoop;
    state.blendTime = 0.0f;
    state.blendDuration = blendDurationSeconds;

    state.activeAnimation = index;
    if (resetTime)
    {
        state.animationTime = 0.0f;
    }
}

void LoadedGLTF::transitionAnimation(AnimationState &state, const std::string &name, float blendDurationSeconds, bool resetTime)
{
    for (size_t i = 0; i < animations.size(); ++i)
    {
        if (animations[i].name == name)
        {
            transitionAnimation(state, static_cast<int>(i), blendDurationSeconds, resetTime);
            return;
        }
    }
}

void LoadedGLTF::updateAnimation(float dt, AnimationState &state)
{
    ensureRestTransformsCached();

    const bool hasAnims = !animations.empty();
    const bool hasActive =
        hasAnims && state.activeAnimation >= 0 && state.activeAnimation < static_cast<int>(animations.size());
    const bool hasBlendFrom =
        hasAnims && state.blending &&
        state.blendFromAnimation >= 0 && state.blendFromAnimation < static_cast<int>(animations.size());

    // If animation is disabled, restore nodes affected by the previous update back to bind pose.
    if (!hasActive && !hasBlendFrom)
    {
        if (!state.touchedNodes.empty())
        {
            for (Node *node : state.touchedNodes)
            {
                if (node)
                {
                    restoreNodeToRest(*node);
                }
            }
            state.touchedNodes.clear();
            refreshAllTransforms();
        }
        return;
    }

    const float dtUnscaled = dt;
    const float dtScaled = dtUnscaled * state.playbackSpeed;

    auto advance_time = [](float &time, const Animation &clip, bool loop, float delta) {
        if (clip.duration <= 0.0f || delta == 0.0f)
        {
            return;
        }

        time += delta;

        if (loop)
        {
            time = std::fmod(time, clip.duration);
            if (time < 0.0f)
            {
                time += clip.duration;
            }
        }
        else
        {
            time = std::clamp(time, 0.0f, clip.duration);
        }
    };

    if (hasActive)
    {
        advance_time(state.animationTime, animations[state.activeAnimation], state.animationLoop, dtScaled);
    }
    if (hasBlendFrom)
    {
        advance_time(state.blendFromTime, animations[state.blendFromAnimation], state.blendFromLoop, dtScaled);
    }

    float blendAlpha = 1.0f;
    if (hasBlendFrom)
    {
        if (dtUnscaled > 0.0f)
        {
            state.blendTime += dtUnscaled;
        }

        if (state.blendDuration > 0.0f)
        {
            blendAlpha = std::clamp(state.blendTime / state.blendDuration, 0.0f, 1.0f);
        }

        if (blendAlpha >= 1.0f)
        {
            state.blending = false;
            state.blendFromAnimation = -1;
            state.blendTime = 0.0f;
            state.blendDuration = 0.0f;
        }
    }

    struct NodePoseOverride
    {
        bool hasT = false;
        bool hasR = false;
        bool hasS = false;
        glm::vec3 t{0.0f, 0.0f, 0.0f};
        glm::quat r{1.0f, 0.0f, 0.0f, 0.0f};
        glm::vec3 s{1.0f, 1.0f, 1.0f};
    };

    thread_local std::unordered_map<Node *, NodePoseOverride> poseTo;
    thread_local std::unordered_map<Node *, NodePoseOverride> poseFrom;
    thread_local std::unordered_set<Node *> touchedNow;

    poseTo.clear();
    poseFrom.clear();
    touchedNow.clear();

    auto sample_clip = [&](const Animation &clip, float t, std::unordered_map<Node *, NodePoseOverride> &out) {
        for (const auto &ch : clip.channels)
        {
            Node *node = ch.node.get();
            if (!node)
            {
                continue;
            }

            const size_t keyCount = ch.times.size();
            if (keyCount == 0)
            {
                continue;
            }

            touchedNow.insert(node);

            size_t k0 = 0;
            size_t k1 = 0;
            if (t <= ch.times.front())
            {
                k0 = k1 = 0;
            }
            else if (t >= ch.times.back())
            {
                k0 = k1 = keyCount - 1;
            }
            else
            {
                auto it = std::upper_bound(ch.times.begin(), ch.times.end(), t);
                k1 = static_cast<size_t>(std::distance(ch.times.begin(), it));
                k0 = (k1 > 0) ? (k1 - 1) : 0;
            }

            float t0 = ch.times[k0];
            float t1 = ch.times[k1];
            float alpha = 0.0f;
            if (k0 != k1 && t1 > t0)
            {
                alpha = (t - t0) / (t1 - t0);
                alpha = std::clamp(alpha, 0.0f, 1.0f);
            }

            NodePoseOverride &pose = out[node];

            switch (ch.target)
            {
                case AnimationChannel::Target::Translation:
                {
                    if (ch.vec3Values.size() != keyCount) break;
                    glm::vec3 v0 = ch.vec3Values[k0];
                    glm::vec3 v1 = ch.vec3Values[k1];
                    pose.t = (ch.interpolation == AnimationChannel::Interpolation::Step || k0 == k1)
                                 ? v0
                                 : (v0 * (1.0f - alpha) + v1 * alpha);
                    pose.hasT = true;
                    break;
                }
                case AnimationChannel::Target::Scale:
                {
                    if (ch.vec3Values.size() != keyCount) break;
                    glm::vec3 v0 = ch.vec3Values[k0];
                    glm::vec3 v1 = ch.vec3Values[k1];
                    pose.s = (ch.interpolation == AnimationChannel::Interpolation::Step || k0 == k1)
                                 ? v0
                                 : (v0 * (1.0f - alpha) + v1 * alpha);
                    pose.hasS = true;
                    break;
                }
                case AnimationChannel::Target::Rotation:
                {
                    if (ch.vec4Values.size() != keyCount) break;
                    glm::vec4 v0 = ch.vec4Values[k0];
                    glm::vec4 v1 = ch.vec4Values[k1];

                    glm::quat q0(v0.w, v0.x, v0.y, v0.z);
                    glm::quat q1(v1.w, v1.x, v1.y, v1.z);
                    pose.r = (ch.interpolation == AnimationChannel::Interpolation::Step || k0 == k1)
                                 ? q0
                                 : glm::slerp(q0, q1, alpha);
                    pose.r = glm::normalize(pose.r);
                    pose.hasR = true;
                    break;
                }
            }
        }
    };

    if (hasActive)
    {
        sample_clip(animations[state.activeAnimation], state.animationTime, poseTo);
    }
    if (hasBlendFrom)
    {
        sample_clip(animations[state.blendFromAnimation], state.blendFromTime, poseFrom);
    }

    bool anyChanged = false;

    // Restore nodes that were animated last update but are no longer affected by the current clips.
    if (!state.touchedNodes.empty())
    {
        for (Node *node : state.touchedNodes)
        {
            if (node && touchedNow.find(node) == touchedNow.end())
            {
                restoreNodeToRest(*node);
                anyChanged = true;
            }
        }
    }

    // Apply current pose(s) to all affected nodes.
    for (Node *node : touchedNow)
    {
        if (!node)
        {
            continue;
        }

        auto restIt = _restTransforms.find(node);
        if (restIt == _restTransforms.end())
        {
            continue;
        }

        const RestNodeTransform &rest = restIt->second;

        struct TRS
        {
            glm::vec3 t;
            glm::quat r;
            glm::vec3 s;
        };

        auto eval_pose = [&](const std::unordered_map<Node *, NodePoseOverride> &m) -> TRS {
            glm::vec3 t = rest.translation;
            glm::quat r = rest.rotation;
            glm::vec3 s = rest.scale;

            auto it = m.find(node);
            if (it != m.end())
            {
                const NodePoseOverride &p = it->second;
                if (p.hasT) t = p.t;
                if (p.hasR) r = p.r;
                if (p.hasS) s = p.s;
            }

            return TRS{t, r, s};
        };

        TRS to = eval_pose(poseTo);
        glm::vec3 outT = to.t;
        glm::quat outR = to.r;
        glm::vec3 outS = to.s;

        if (hasBlendFrom)
        {
            TRS from = eval_pose(poseFrom);
            outT = from.t * (1.0f - blendAlpha) + to.t * blendAlpha;
            outS = from.s * (1.0f - blendAlpha) + to.s * blendAlpha;
            outR = glm::normalize(glm::slerp(from.r, to.r, blendAlpha));
        }

        node->translation = outT;
        node->rotation = outR;
        node->scale = outS;
        node->hasTRS = true;
        node->updateLocalFromTRS();
        anyChanged = true;
    }

    state.touchedNodes.swap(touchedNow);
    touchedNow.clear();

    if (anyChanged)
    {
        refreshAllTransforms();
    }
}

void LoadedGLTF::clearAll()
{
    const char *name = debugName.empty() ? "<unnamed>" : debugName.c_str();
    Logger::info("[GLTF] clearAll begin for '{}' (meshes={} images={} materials={} samplers={})",
                 name,
                 meshes.size(),
                 images.size(),
                 materials.size(),
                 samplers.size());

    VkDevice dv = creator->_deviceManager->device();
    VK_CHECK(vkDeviceWaitIdle(dv));

    // Before destroying descriptor pools, unregister descriptor-set watches so
    // the TextureCache will not attempt to patch dead sets.
    if (creator && creator->_context && creator->_context->textures)
    {
        TextureCache *cache = creator->_context->textures;
        for (auto &[k, mat] : materials)
        {
            if (mat && mat->data.materialSet != VK_NULL_HANDLE)
            {
                cache->unwatchSet(mat->data.materialSet);
            }
        }
    }

    auto materialBuffer = materialDataBuffer;

    // Free material descriptor sets before destroying resources they reference.
    descriptorPool.destroy_pools(dv);

    for (auto &[k, v]: meshes)
    {
        if (creator->_rayManager)
        {
            creator->_rayManager->removeBLASForMesh(v.get());
        }
        creator->_resourceManager->destroy_buffer(v->meshBuffers.indexBuffer);
        creator->_resourceManager->destroy_buffer(v->meshBuffers.vertexBuffer);
    }

    for (auto &[k, v]: images)
    {
        if (v.image == creator->_errorCheckerboardImage.image)
        {
            // dont destroy the default images
            continue;
        }
        creator->_resourceManager->destroy_image(v);
    }

    for (auto &sampler: samplers)
    {
        vkDestroySampler(dv, sampler, nullptr);
    }

    creator->_resourceManager->destroy_buffer(materialBuffer);

    Logger::info("[GLTF] clearAll done for '{}' (meshes={}, images={}, materials={}, samplers={})",
                 name,
                 meshes.size(),
                 images.size(),
                 materials.size(),
                 samplers.size());
}
