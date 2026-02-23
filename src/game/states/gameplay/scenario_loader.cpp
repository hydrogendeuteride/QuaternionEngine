#include "scenario_loader.h"
#include "core/util/logger.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>

namespace Game
{
    using json = nlohmann::json;

    // ---- Helper: read optional fields with defaults ----

    namespace
    {
        template<typename T>
        T json_get(const json &j, const char *key, const T &fallback)
        {
            if (j.contains(key) && !j[key].is_null())
            {
                return j[key].get<T>();
            }
            return fallback;
        }

        glm::vec3 json_get_vec3(const json &j, const char *key, const glm::vec3 &fallback = glm::vec3(0.0f))
        {
            if (!j.contains(key) || j[key].is_null())
            {
                return fallback;
            }
            const auto &v = j[key];
            return glm::vec3(
                    json_get<float>(v, "x", fallback.x),
                    json_get<float>(v, "y", fallback.y),
                    json_get<float>(v, "z", fallback.z));
        }

        glm::dvec3 json_get_dvec3(const json &j, const char *key, const glm::dvec3 &fallback = glm::dvec3(0.0))
        {
            if (!j.contains(key) || j[key].is_null())
            {
                return fallback;
            }
            const auto &v = j[key];
            return glm::dvec3(
                    json_get<double>(v, "x", fallback.x),
                    json_get<double>(v, "y", fallback.y),
                    json_get<double>(v, "z", fallback.z));
        }

        // ---- Primitive type ----

        GameAPI::PrimitiveType parse_primitive_type(const std::string &s)
        {
            if (s == "cube") return GameAPI::PrimitiveType::Cube;
            if (s == "sphere") return GameAPI::PrimitiveType::Sphere;
            if (s == "plane") return GameAPI::PrimitiveType::Plane;
            if (s == "capsule") return GameAPI::PrimitiveType::Capsule;
            Logger::warn("Unknown primitive type '{}', defaulting to Sphere.", s);
            return GameAPI::PrimitiveType::Sphere;
        }

        const char *primitive_type_string(GameAPI::PrimitiveType t)
        {
            switch (t)
            {
                case GameAPI::PrimitiveType::Cube: return "cube";
                case GameAPI::PrimitiveType::Sphere: return "sphere";
                case GameAPI::PrimitiveType::Plane: return "plane";
                case GameAPI::PrimitiveType::Capsule: return "capsule";
            }
            return "sphere";
        }

        // ---- Motion type ----

        Physics::MotionType parse_motion_type(const std::string &s)
        {
            if (s == "static") return Physics::MotionType::Static;
            if (s == "kinematic") return Physics::MotionType::Kinematic;
            if (s == "dynamic") return Physics::MotionType::Dynamic;
            return Physics::MotionType::Dynamic;
        }

        const char *motion_type_string(Physics::MotionType t)
        {
            switch (t)
            {
                case Physics::MotionType::Static: return "static";
                case Physics::MotionType::Kinematic: return "kinematic";
                case Physics::MotionType::Dynamic: return "dynamic";
            }
            return "dynamic";
        }

        // ---- Collision shape ----

        Physics::CollisionShape parse_collision_shape(const json &j)
        {
            const std::string type = json_get<std::string>(j, "type", "box");

            if (type == "sphere")
            {
                return Physics::CollisionShape::Sphere(json_get<float>(j, "radius", 0.5f));
            }
            if (type == "capsule")
            {
                return Physics::CollisionShape::Capsule(
                        json_get<float>(j, "radius", 0.5f),
                        json_get<float>(j, "half_height", 0.5f));
            }
            if (type == "cylinder")
            {
                return Physics::CollisionShape::Cylinder(
                        json_get<float>(j, "radius", 0.5f),
                        json_get<float>(j, "half_height", 0.5f));
            }
            if (type == "box")
            {
                const glm::vec3 he = json_get_vec3(j, "half_extents", glm::vec3(0.5f));
                return Physics::CollisionShape::Box(he);
            }

            // Default fallback
            return Physics::CollisionShape::Box(0.5f, 0.5f, 0.5f);
        }

        json serialize_collision_shape(const Physics::CollisionShape &shape)
        {
            json j;
            if (shape.is_sphere())
            {
                j["type"] = "sphere";
                j["radius"] = shape.as_sphere()->radius;
            }
            else if (shape.is_capsule())
            {
                j["type"] = "capsule";
                j["radius"] = shape.as_capsule()->radius;
                j["half_height"] = shape.as_capsule()->half_height;
            }
            else if (shape.is_cylinder())
            {
                j["type"] = "cylinder";
                j["radius"] = shape.as_cylinder()->radius;
                j["half_height"] = shape.as_cylinder()->half_height;
            }
            else if (shape.is_box())
            {
                j["type"] = "box";
                j["half_extents"] = {{"x", shape.as_box()->half_extents.x},
                                     {"y", shape.as_box()->half_extents.y},
                                     {"z", shape.as_box()->half_extents.z}};
            }
            else
            {
                j["type"] = "box";
                j["half_extents"] = {{"x", 0.5f}, {"y", 0.5f}, {"z", 0.5f}};
            }
            return j;
        }

        // ---- Body settings ----

        Physics::BodySettings parse_body_settings(const json &j)
        {
            Physics::BodySettings bs{};

            if (j.contains("shape") && !j["shape"].is_null())
            {
                bs.shape = parse_collision_shape(j["shape"]);
            }

            bs.user_data = json_get<uint64_t>(j, "user_data", 0);

            if (j.contains("position") && !j["position"].is_null())
            {
                const auto &p = j["position"];
                bs.position = glm::dvec3(
                        json_get<double>(p, "x", 0.0),
                        json_get<double>(p, "y", 0.0),
                        json_get<double>(p, "z", 0.0));
            }

            if (j.contains("rotation") && !j["rotation"].is_null())
            {
                const auto &r = j["rotation"];
                bs.rotation = glm::quat(
                        json_get<float>(r, "w", 1.0f),
                        json_get<float>(r, "x", 0.0f),
                        json_get<float>(r, "y", 0.0f),
                        json_get<float>(r, "z", 0.0f));
            }

            const std::string motion = json_get<std::string>(j, "motion_type", "dynamic");
            bs.motion_type = parse_motion_type(motion);

            bs.mass = json_get<float>(j, "mass", 1.0f);
            bs.friction = json_get<float>(j, "friction", 0.5f);
            bs.restitution = json_get<float>(j, "restitution", 0.0f);
            bs.linear_damping = json_get<float>(j, "linear_damping", 0.0f);
            bs.angular_damping = json_get<float>(j, "angular_damping", 0.05f);
            bs.layer = json_get<uint32_t>(j, "layer", Physics::Layer::Default);
            bs.is_sensor = json_get<bool>(j, "is_sensor", false);
            bs.start_active = json_get<bool>(j, "start_active", true);
            bs.allow_sleeping = json_get<bool>(j, "allow_sleeping", true);
            bs.gravity_scale = json_get<float>(j, "gravity_scale", 1.0f);

            return bs;
        }

        json serialize_body_settings(const Physics::BodySettings &bs)
        {
            json j;
            j["shape"] = serialize_collision_shape(bs.shape);
            j["user_data"] = bs.user_data;
            j["position"] = {{"x", bs.position.x}, {"y", bs.position.y}, {"z", bs.position.z}};
            j["rotation"] = {{"w", bs.rotation.w}, {"x", bs.rotation.x}, {"y", bs.rotation.y}, {"z", bs.rotation.z}};
            j["motion_type"] = motion_type_string(bs.motion_type);
            j["mass"] = bs.mass;
            j["friction"] = bs.friction;
            j["restitution"] = bs.restitution;
            j["linear_damping"] = bs.linear_damping;
            j["angular_damping"] = bs.angular_damping;
            j["layer"] = bs.layer;
            j["is_sensor"] = bs.is_sensor;
            j["start_active"] = bs.start_active;
            j["allow_sleeping"] = bs.allow_sleeping;
            j["gravity_scale"] = bs.gravity_scale;
            return j;
        }

        // ---- CelestialDef ----

        ScenarioConfig::CelestialDef parse_celestial_def(const json &j)
        {
            ScenarioConfig::CelestialDef c{};
            c.name = json_get<std::string>(j, "name", "unnamed");
            c.mass_kg = json_get<double>(j, "mass_kg", 0.0);
            c.radius_m = json_get<double>(j, "radius_m", 0.0);
            c.atmosphere_top_m = json_get<double>(j, "atmosphere_top_m", 0.0);
            c.terrain_max_m = json_get<double>(j, "terrain_max_m", 0.0);
            c.soi_radius_m = json_get<double>(j, "soi_radius_m", 0.0);
            c.orbit_distance_m = json_get<double>(j, "orbit_distance_m", 0.0);
            c.has_terrain = json_get<bool>(j, "has_terrain", false);
            c.albedo_dir = json_get<std::string>(j, "albedo_dir", "");
            c.height_dir = json_get<std::string>(j, "height_dir", "");
            c.height_max_m = json_get<double>(j, "height_max_m", 0.0);
            c.emission_dir = json_get<std::string>(j, "emission_dir", "");
            c.emission_factor = json_get_vec3(j, "emission_factor", glm::vec3(0.0f));
            c.render_scale = json_get<float>(j, "render_scale", 1.0f);
            return c;
        }

        json serialize_celestial_def(const ScenarioConfig::CelestialDef &c)
        {
            json j;
            j["name"] = c.name;
            j["mass_kg"] = c.mass_kg;
            j["radius_m"] = c.radius_m;
            j["atmosphere_top_m"] = c.atmosphere_top_m;
            j["terrain_max_m"] = c.terrain_max_m;
            j["soi_radius_m"] = c.soi_radius_m;
            j["orbit_distance_m"] = c.orbit_distance_m;
            j["has_terrain"] = c.has_terrain;
            j["albedo_dir"] = c.albedo_dir;
            j["height_dir"] = c.height_dir;
            j["height_max_m"] = c.height_max_m;
            j["emission_dir"] = c.emission_dir;
            j["emission_factor"] = {{"x", c.emission_factor.x}, {"y", c.emission_factor.y}, {"z", c.emission_factor.z}};
            j["render_scale"] = c.render_scale;
            return j;
        }

        // ---- OrbiterDef ----

        ScenarioConfig::OrbiterDef parse_orbiter_def(const json &j)
        {
            ScenarioConfig::OrbiterDef o{};
            o.name = json_get<std::string>(j, "name", "unnamed");
            o.orbit_altitude_m = json_get<double>(j, "orbit_altitude_m", 0.0);
            o.offset_from_player = json_get_dvec3(j, "offset_from_player");
            o.relative_velocity = json_get_dvec3(j, "relative_velocity");
            o.primitive = parse_primitive_type(json_get<std::string>(j, "primitive", "sphere"));
            o.render_scale = json_get_vec3(j, "render_scale", glm::vec3(1.0f));
            o.is_player = json_get<bool>(j, "is_player", false);
            o.is_rebase_anchor = json_get<bool>(j, "is_rebase_anchor", false);

            if (j.contains("body_settings") && !j["body_settings"].is_null())
            {
                o.body_settings = parse_body_settings(j["body_settings"]);
            }

            return o;
        }

        json serialize_orbiter_def(const ScenarioConfig::OrbiterDef &o)
        {
            json j;
            j["name"] = o.name;
            j["orbit_altitude_m"] = o.orbit_altitude_m;
            j["offset_from_player"] = {{"x", o.offset_from_player.x}, {"y", o.offset_from_player.y}, {"z", o.offset_from_player.z}};
            j["relative_velocity"] = {{"x", o.relative_velocity.x}, {"y", o.relative_velocity.y}, {"z", o.relative_velocity.z}};
            j["primitive"] = primitive_type_string(o.primitive);
            j["render_scale"] = {{"x", o.render_scale.x}, {"y", o.render_scale.y}, {"z", o.render_scale.z}};
            j["body_settings"] = serialize_body_settings(o.body_settings);
            j["is_player"] = o.is_player;
            j["is_rebase_anchor"] = o.is_rebase_anchor;
            return j;
        }
    } // anonymous namespace

    // ========================================================================
    // Public API
    // ========================================================================

    std::optional<ScenarioConfig> load_scenario_config(const std::string &json_path)
    {
        std::ifstream file(json_path);
        if (!file.is_open())
        {
            Logger::error("Failed to open scenario file: {}", json_path);
            return std::nullopt;
        }

        json root;
        try
        {
            file >> root;
        }
        catch (const json::parse_error &e)
        {
            Logger::error("JSON parse error in '{}': {}", json_path, e.what());
            return std::nullopt;
        }

        ScenarioConfig cfg;

        cfg.speed_scale = json_get<double>(root, "speed_scale", 1.0);
        cfg.mu_base = json_get<double>(root, "mu_base", 3.986004418e14);

        if (root.contains("system_center") && !root["system_center"].is_null())
        {
            const auto &sc = root["system_center"];
            cfg.system_center = WorldVec3(
                    json_get<double>(sc, "x", 0.0),
                    json_get<double>(sc, "y", 0.0),
                    json_get<double>(sc, "z", 0.0));
        }

        if (root.contains("celestials") && root["celestials"].is_array())
        {
            for (const auto &elem : root["celestials"])
            {
                cfg.celestials.push_back(parse_celestial_def(elem));
            }
        }

        if (root.contains("orbiters") && root["orbiters"].is_array())
        {
            for (const auto &elem : root["orbiters"])
            {
                cfg.orbiters.push_back(parse_orbiter_def(elem));
            }
        }

        if (cfg.celestials.empty())
        {
            Logger::warn("Scenario '{}' has no celestials defined.", json_path);
        }
        if (cfg.orbiters.empty())
        {
            Logger::warn("Scenario '{}' has no orbiters defined.", json_path);
        }

        Logger::info("Loaded scenario '{}': {} celestials, {} orbiters",
                      json_path, cfg.celestials.size(), cfg.orbiters.size());
        return cfg;
    }

    std::string serialize_scenario_config(const ScenarioConfig &config)
    {
        json root;
        root["schema_version"] = 1;
        root["speed_scale"] = config.speed_scale;
        root["mu_base"] = config.mu_base;
        root["system_center"] = {
                {"x", static_cast<double>(config.system_center.x)},
                {"y", static_cast<double>(config.system_center.y)},
                {"z", static_cast<double>(config.system_center.z)}};

        root["celestials"] = json::array();
        for (const auto &c : config.celestials)
        {
            root["celestials"].push_back(serialize_celestial_def(c));
        }

        root["orbiters"] = json::array();
        for (const auto &o : config.orbiters)
        {
            root["orbiters"].push_back(serialize_orbiter_def(o));
        }

        return root.dump(2);
    }
} // namespace Game
