#pragma once

#include <core/world.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class EngineContext;
class SceneManager;
struct DrawContext;
class MeshAsset;
struct GLTFMaterial;

class PlanetSystem
{
public:
    enum class BodyID : uint8_t
    {
        Earth = 0,
        Moon = 1,
    };

    struct PlanetBody
    {
        std::string name;
        WorldVec3 center_world{0.0, 0.0, 0.0};
        double radius_m = 1.0;
        bool visible = true;

        std::shared_ptr<MeshAsset> mesh;
        std::shared_ptr<GLTFMaterial> material;
    };

    void init(EngineContext *context);

    void update_and_emit(const SceneManager &scene, DrawContext &draw_context);

    bool enabled() const { return _enabled; }
    void set_enabled(bool enabled) { _enabled = enabled; }

    const PlanetBody *get_body(BodyID id) const;
    PlanetBody *get_body(BodyID id);
    const std::vector<PlanetBody> &bodies() const { return _bodies; }

private:
    void ensure_bodies_created();

    EngineContext *_context = nullptr;
    bool _enabled = true;
    std::vector<PlanetBody> _bodies;
};

