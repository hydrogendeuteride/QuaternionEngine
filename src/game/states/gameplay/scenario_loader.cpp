#include "scenario_loader.h"
#include "core/util/logger.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace Game
{
    using json = nlohmann::json;

    namespace
    {
        [[noreturn]] void fail(std::string msg)
        {
            throw std::runtime_error(std::move(msg));
        }

        std::string child_path(const std::string &base, const std::string &child)
        {
            if (base.empty()) return child;
            if (child.empty()) return base;
            return base + "." + child;
        }

        template<typename T>
        T json_required(const json &j, const char *key, const std::string &path)
        {
            if (!j.is_object())
            {
                fail(path + " must be an object");
            }

            const std::string key_path = child_path(path, key);
            auto it = j.find(key);
            if (it == j.end() || it->is_null())
            {
                fail(key_path + " is required");
            }

            try
            {
                return it->get<T>();
            }
            catch (const json::exception &e)
            {
                fail(key_path + ": " + std::string(e.what()));
            }
        }

        template<typename T>
        T json_required_finite(const json &j, const char *key, const std::string &path)
        {
            T value = json_required<T>(j, key, path);
            if constexpr (std::is_floating_point_v<T>)
            {
                if (!std::isfinite(value))
                {
                    fail(child_path(path, key) + " must be finite");
                }
            }
            return value;
        }

        const json *json_required_object(const json &j, const char *key, const std::string &path)
        {
            if (!j.is_object())
            {
                fail(path + " must be an object");
            }

            const std::string key_path = child_path(path, key);
            auto it = j.find(key);
            if (it == j.end() || it->is_null())
            {
                fail(key_path + " is required");
            }
            if (!it->is_object())
            {
                fail(key_path + " must be an object");
            }
            return &(*it);
        }

        const json *json_required_array(const json &j, const char *key, const std::string &path)
        {
            if (!j.is_object())
            {
                fail(path + " must be an object");
            }

            const std::string key_path = child_path(path, key);
            auto it = j.find(key);
            if (it == j.end() || it->is_null())
            {
                fail(key_path + " is required");
            }
            if (!it->is_array())
            {
                fail(key_path + " must be an array");
            }
            return &(*it);
        }

        glm::vec3 parse_vec3(const json &j, const std::string &path)
        {
            if (!j.is_object())
            {
                fail(path + " must be an object");
            }

            return glm::vec3(
                    json_required_finite<float>(j, "x", path),
                    json_required_finite<float>(j, "y", path),
                    json_required_finite<float>(j, "z", path));
        }

        glm::dvec3 parse_dvec3(const json &j, const std::string &path)
        {
            if (!j.is_object())
            {
                fail(path + " must be an object");
            }

            return glm::dvec3(
                    json_required_finite<double>(j, "x", path),
                    json_required_finite<double>(j, "y", path),
                    json_required_finite<double>(j, "z", path));
        }

        glm::quat parse_quat(const json &j, const std::string &path)
        {
            if (!j.is_object())
            {
                fail(path + " must be an object");
            }

            return glm::quat(
                    json_required_finite<float>(j, "w", path),
                    json_required_finite<float>(j, "x", path),
                    json_required_finite<float>(j, "y", path),
                    json_required_finite<float>(j, "z", path));
        }

        // ---- Primitive type ----

        GameAPI::PrimitiveType parse_primitive_type(const std::string &s, const std::string &path)
        {
            if (s == "cube") return GameAPI::PrimitiveType::Cube;
            if (s == "sphere") return GameAPI::PrimitiveType::Sphere;
            if (s == "plane") return GameAPI::PrimitiveType::Plane;
            if (s == "capsule") return GameAPI::PrimitiveType::Capsule;
            fail(path + " has unsupported value '" + s + "'");
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

        Physics::MotionType parse_motion_type(const std::string &s, const std::string &path)
        {
            if (s == "static") return Physics::MotionType::Static;
            if (s == "kinematic") return Physics::MotionType::Kinematic;
            if (s == "dynamic") return Physics::MotionType::Dynamic;
            fail(path + " has unsupported value '" + s + "'");
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

        Physics::CollisionShape parse_collision_shape(const json &j, const std::string &path)
        {
            if (!j.is_object())
            {
                fail(path + " must be an object");
            }

            const std::string type = json_required<std::string>(j, "type", path);

            if (type == "sphere")
            {
                return Physics::CollisionShape::Sphere(json_required_finite<float>(j, "radius", path));
            }
            if (type == "capsule")
            {
                return Physics::CollisionShape::Capsule(
                        json_required_finite<float>(j, "radius", path),
                        json_required_finite<float>(j, "half_height", path));
            }
            if (type == "cylinder")
            {
                return Physics::CollisionShape::Cylinder(
                        json_required_finite<float>(j, "radius", path),
                        json_required_finite<float>(j, "half_height", path));
            }
            if (type == "box")
            {
                return Physics::CollisionShape::Box(
                        parse_vec3(*json_required_object(j, "half_extents", path), child_path(path, "half_extents")));
            }

            fail(child_path(path, "type") + " has unsupported value '" + type + "'");
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
                fail("serialize_collision_shape: unsupported shape variant");
            }
            return j;
        }

        // ---- Body settings ----

        Physics::BodySettings parse_body_settings(const json &j, const std::string &path)
        {
            if (!j.is_object())
            {
                fail(path + " must be an object");
            }

            Physics::BodySettings bs{};
            bs.shape = parse_collision_shape(*json_required_object(j, "shape", path), child_path(path, "shape"));
            bs.user_data = json_required<uint64_t>(j, "user_data", path);
            bs.position = parse_dvec3(*json_required_object(j, "position", path), child_path(path, "position"));
            bs.rotation = parse_quat(*json_required_object(j, "rotation", path), child_path(path, "rotation"));
            bs.motion_type = parse_motion_type(
                    json_required<std::string>(j, "motion_type", path),
                    child_path(path, "motion_type"));

            bs.mass = json_required_finite<float>(j, "mass", path);
            if (bs.mass < 0.0f)
            {
                fail(child_path(path, "mass") + " must be >= 0");
            }
            bs.friction = json_required_finite<float>(j, "friction", path);
            if (bs.friction < 0.0f)
            {
                fail(child_path(path, "friction") + " must be >= 0");
            }
            bs.restitution = json_required_finite<float>(j, "restitution", path);
            if (bs.restitution < 0.0f)
            {
                fail(child_path(path, "restitution") + " must be >= 0");
            }
            bs.linear_damping = json_required_finite<float>(j, "linear_damping", path);
            if (bs.linear_damping < 0.0f)
            {
                fail(child_path(path, "linear_damping") + " must be >= 0");
            }
            bs.angular_damping = json_required_finite<float>(j, "angular_damping", path);
            if (bs.angular_damping < 0.0f)
            {
                fail(child_path(path, "angular_damping") + " must be >= 0");
            }
            bs.layer = json_required<uint32_t>(j, "layer", path);
            if (bs.layer >= Physics::Layer::Count)
            {
                fail(child_path(path, "layer") + " must be in [0, " + std::to_string(Physics::Layer::Count - 1) + "]");
            }
            bs.is_sensor = json_required<bool>(j, "is_sensor", path);
            bs.start_active = json_required<bool>(j, "start_active", path);
            bs.allow_sleeping = json_required<bool>(j, "allow_sleeping", path);
            bs.gravity_scale = json_required_finite<float>(j, "gravity_scale", path);

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

        ScenarioConfig::CelestialDef parse_celestial_def(const json &j, const std::string &path)
        {
            if (!j.is_object())
            {
                fail(path + " must be an object");
            }

            ScenarioConfig::CelestialDef c{};
            c.name = json_required<std::string>(j, "name", path);
            c.mass_kg = json_required_finite<double>(j, "mass_kg", path);
            c.radius_m = json_required_finite<double>(j, "radius_m", path);
            c.atmosphere_top_m = json_required_finite<double>(j, "atmosphere_top_m", path);
            c.terrain_max_m = json_required_finite<double>(j, "terrain_max_m", path);
            c.soi_radius_m = json_required_finite<double>(j, "soi_radius_m", path);
            c.orbit_distance_m = json_required_finite<double>(j, "orbit_distance_m", path);
            c.has_terrain = json_required<bool>(j, "has_terrain", path);
            c.albedo_dir = json_required<std::string>(j, "albedo_dir", path);
            c.height_dir = json_required<std::string>(j, "height_dir", path);
            c.height_max_m = json_required_finite<double>(j, "height_max_m", path);
            c.emission_dir = json_required<std::string>(j, "emission_dir", path);
            c.emission_factor = parse_vec3(*json_required_object(j, "emission_factor", path), child_path(path, "emission_factor"));
            c.render_scale = json_required_finite<float>(j, "render_scale", path);

            if (c.name.empty())
            {
                fail(child_path(path, "name") + " must not be empty");
            }
            if (c.mass_kg <= 0.0)
            {
                fail(child_path(path, "mass_kg") + " must be > 0");
            }
            if (c.radius_m <= 0.0)
            {
                fail(child_path(path, "radius_m") + " must be > 0");
            }
            if (c.atmosphere_top_m < 0.0 || c.terrain_max_m < 0.0 || c.soi_radius_m < 0.0 ||
                c.orbit_distance_m < 0.0 || c.height_max_m < 0.0)
            {
                fail(path + " numeric distances/heights must be >= 0");
            }
            if (c.render_scale <= 0.0f)
            {
                fail(child_path(path, "render_scale") + " must be > 0");
            }
            if (c.has_terrain && (c.albedo_dir.empty() || c.height_dir.empty()))
            {
                fail(path + " has_terrain=true requires non-empty albedo_dir and height_dir");
            }

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

        ScenarioConfig::OrbiterDef parse_orbiter_def(const json &j, const std::string &path)
        {
            if (!j.is_object())
            {
                fail(path + " must be an object");
            }

            ScenarioConfig::OrbiterDef o{};
            o.name = json_required<std::string>(j, "name", path);
            o.orbit_altitude_m = json_required_finite<double>(j, "orbit_altitude_m", path);
            o.offset_from_player = parse_dvec3(
                    *json_required_object(j, "offset_from_player", path),
                    child_path(path, "offset_from_player"));
            o.relative_velocity = parse_dvec3(
                    *json_required_object(j, "relative_velocity", path),
                    child_path(path, "relative_velocity"));
            o.primitive = parse_primitive_type(
                    json_required<std::string>(j, "primitive", path),
                    child_path(path, "primitive"));
            o.render_scale = parse_vec3(
                    *json_required_object(j, "render_scale", path),
                    child_path(path, "render_scale"));
            o.body_settings = parse_body_settings(
                    *json_required_object(j, "body_settings", path),
                    child_path(path, "body_settings"));
            o.is_player = json_required<bool>(j, "is_player", path);
            o.is_rebase_anchor = json_required<bool>(j, "is_rebase_anchor", path);

            if (o.name.empty())
            {
                fail(child_path(path, "name") + " must not be empty");
            }
            if (o.orbit_altitude_m < 0.0)
            {
                fail(child_path(path, "orbit_altitude_m") + " must be >= 0");
            }
            if (o.render_scale.x <= 0.0f || o.render_scale.y <= 0.0f || o.render_scale.z <= 0.0f)
            {
                fail(child_path(path, "render_scale") + " components must be > 0");
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

        try
        {
            if (!root.is_object())
            {
                fail("root must be an object");
            }

            const int schema_version = json_required<int>(root, "schema_version", "root");
            if (schema_version != 1)
            {
                fail("root.schema_version has unsupported value " + std::to_string(schema_version));
            }

            ScenarioConfig cfg;
            cfg.speed_scale = json_required_finite<double>(root, "speed_scale", "root");
            cfg.mu_base = json_required_finite<double>(root, "mu_base", "root");
            if (cfg.mu_base <= 0.0)
            {
                fail("root.mu_base must be > 0");
            }

            const glm::dvec3 center = parse_dvec3(
                    *json_required_object(root, "system_center", "root"),
                    "root.system_center");
            cfg.system_center = WorldVec3(center.x, center.y, center.z);

            const json *celestials_json = json_required_array(root, "celestials", "root");
            if (celestials_json->empty())
            {
                fail("root.celestials must not be empty");
            }
            for (size_t i = 0; i < celestials_json->size(); ++i)
            {
                cfg.celestials.push_back(parse_celestial_def(
                        (*celestials_json)[i],
                        "root.celestials[" + std::to_string(i) + "]"));
            }

            const json *orbiters_json = json_required_array(root, "orbiters", "root");
            if (orbiters_json->empty())
            {
                fail("root.orbiters must not be empty");
            }
            for (size_t i = 0; i < orbiters_json->size(); ++i)
            {
                cfg.orbiters.push_back(parse_orbiter_def(
                        (*orbiters_json)[i],
                        "root.orbiters[" + std::to_string(i) + "]"));
            }

            Logger::info("Loaded scenario '{}': {} celestials, {} orbiters",
                         json_path, cfg.celestials.size(), cfg.orbiters.size());
            return cfg;
        }
        catch (const std::exception &e)
        {
            Logger::error("Scenario '{}' validation failed: {}", json_path, e.what());
            return std::nullopt;
        }
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
