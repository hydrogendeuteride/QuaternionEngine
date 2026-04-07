#include "game/states/gameplay/scenario/scenario_loader.h"
#include "core/util/logger.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
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
                const float radius = json_required_finite<float>(j, "radius", path);
                if (!(radius > 0.0f))
                {
                    fail(child_path(path, "radius") + " must be > 0");
                }
                return Physics::CollisionShape::Sphere(radius);
            }
            if (type == "capsule")
            {
                const float radius = json_required_finite<float>(j, "radius", path);
                const float half_height = json_required_finite<float>(j, "half_height", path);
                if (!(radius > 0.0f))
                {
                    fail(child_path(path, "radius") + " must be > 0");
                }
                if (!(half_height > 0.0f))
                {
                    fail(child_path(path, "half_height") + " must be > 0");
                }
                return Physics::CollisionShape::Capsule(
                        radius,
                        half_height);
            }
            if (type == "cylinder")
            {
                const float radius = json_required_finite<float>(j, "radius", path);
                const float half_height = json_required_finite<float>(j, "half_height", path);
                if (!(radius > 0.0f))
                {
                    fail(child_path(path, "radius") + " must be > 0");
                }
                if (!(half_height > 0.0f))
                {
                    fail(child_path(path, "half_height") + " must be > 0");
                }
                return Physics::CollisionShape::Cylinder(
                        radius,
                        half_height);
            }
            if (type == "box")
            {
                const glm::vec3 half_extents = parse_vec3(
                        *json_required_object(j, "half_extents", path),
                        child_path(path, "half_extents"));
                if (!(half_extents.x > 0.0f) || !(half_extents.y > 0.0f) || !(half_extents.z > 0.0f))
                {
                    fail(child_path(path, "half_extents") + " components must be > 0");
                }
                return Physics::CollisionShape::Box(half_extents);
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
            bs.has_explicit_mass = true;
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
            if (const auto it = j.find("detail_normal_dir"); it != j.end() && !it->is_null())
            {
                c.detail_normal_dir = json_required<std::string>(j, "detail_normal_dir", path);
            }
            if (const auto it = j.find("detail_normal_strength"); it != j.end() && !it->is_null())
            {
                c.detail_normal_strength = json_required_finite<float>(j, "detail_normal_strength", path);
            }
            if (const auto it = j.find("cavity_dir"); it != j.end() && !it->is_null())
            {
                c.cavity_dir = json_required<std::string>(j, "cavity_dir", path);
            }
            if (const auto it = j.find("cavity_strength"); it != j.end() && !it->is_null())
            {
                c.cavity_strength = json_required_finite<float>(j, "cavity_strength", path);
            }
            if (const auto it = j.find("enable_terminator_shadow"); it != j.end() && !it->is_null())
            {
                c.enable_terminator_shadow = json_required<bool>(j, "enable_terminator_shadow", path);
            }
            if (const auto it = j.find("patch_resolution_override"); it != j.end() && !it->is_null())
            {
                c.patch_resolution_override = json_required<uint32_t>(j, "patch_resolution_override", path);
            }
            if (const auto it = j.find("target_sse_px_override"); it != j.end() && !it->is_null())
            {
                c.target_sse_px_override = json_required_finite<float>(j, "target_sse_px_override", path);
            }
            c.emission_dir = json_required<std::string>(j, "emission_dir", path);
            c.emission_factor = parse_vec3(*json_required_object(j, "emission_factor", path), child_path(path, "emission_factor"));
            if (const auto it = j.find("specular_dir"); it != j.end() && !it->is_null())
            {
                c.specular_dir = json_required<std::string>(j, "specular_dir", path);
            }
            if (const auto it = j.find("specular_strength"); it != j.end() && !it->is_null())
            {
                c.specular_strength = json_required_finite<float>(j, "specular_strength", path);
            }
            if (const auto it = j.find("specular_roughness"); it != j.end() && !it->is_null())
            {
                c.specular_roughness = json_required_finite<float>(j, "specular_roughness", path);
            }
            c.render_scale = json_required_finite<float>(j, "render_scale", path);
            if (const auto it = j.find("prediction_orbit_color"); it != j.end() && !it->is_null())
            {
                c.prediction_orbit_color = parse_vec3(*json_required_object(j, "prediction_orbit_color", path),
                                                      child_path(path, "prediction_orbit_color"));
                c.has_prediction_orbit_color = true;
            }

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
                c.orbit_distance_m < 0.0 || c.height_max_m < 0.0 ||
                c.detail_normal_strength < 0.0f || c.cavity_strength < 0.0f ||
                c.specular_strength < 0.0f || c.specular_roughness < 0.0f ||
                c.target_sse_px_override < 0.0f)
            {
                fail(path + " numeric distances/heights must be >= 0");
            }
            if (c.render_scale <= 0.0f)
            {
                fail(child_path(path, "render_scale") + " must be > 0");
            }
            if (c.has_terrain && c.albedo_dir.empty())
            {
                fail(path + " has_terrain=true requires non-empty albedo_dir");
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
            j["detail_normal_dir"] = c.detail_normal_dir;
            j["detail_normal_strength"] = c.detail_normal_strength;
            j["cavity_dir"] = c.cavity_dir;
            j["cavity_strength"] = c.cavity_strength;
            j["enable_terminator_shadow"] = c.enable_terminator_shadow;
            j["patch_resolution_override"] = c.patch_resolution_override;
            j["target_sse_px_override"] = c.target_sse_px_override;
            j["emission_dir"] = c.emission_dir;
            j["emission_factor"] = {{"x", c.emission_factor.x}, {"y", c.emission_factor.y}, {"z", c.emission_factor.z}};
            j["specular_dir"] = c.specular_dir;
            j["specular_strength"] = c.specular_strength;
            j["specular_roughness"] = c.specular_roughness;
            j["render_scale"] = c.render_scale;
            if (c.has_prediction_orbit_color)
            {
                j["prediction_orbit_color"] = {
                        {"x", c.prediction_orbit_color.x},
                        {"y", c.prediction_orbit_color.y},
                        {"z", c.prediction_orbit_color.z}};
            }
            return j;
        }

        // ---- MaterialDef ----

        ScenarioConfig::MaterialDef parse_material_def(const json &j, const std::string &path)
        {
            if (!j.is_object())
            {
                fail(path + " must be an object");
            }

            ScenarioConfig::MaterialDef m{};
            if (const auto it = j.find("albedo"); it != j.end() && !it->is_null())
            {
                m.albedo = it->get<std::string>();
            }
            if (const auto it = j.find("normal"); it != j.end() && !it->is_null())
            {
                m.normal = it->get<std::string>();
            }
            if (const auto it = j.find("metal_rough"); it != j.end() && !it->is_null())
            {
                m.metal_rough = it->get<std::string>();
            }
            if (const auto it = j.find("occlusion"); it != j.end() && !it->is_null())
            {
                m.occlusion = it->get<std::string>();
            }
            if (const auto it = j.find("emissive"); it != j.end() && !it->is_null())
            {
                m.emissive = it->get<std::string>();
            }
            if (const auto it = j.find("color_factor"); it != j.end() && !it->is_null())
            {
                const glm::vec3 rgb = parse_vec3(*it, child_path(path, "color_factor"));
                const float a = (j.contains("color_factor_a") && !j["color_factor_a"].is_null())
                                    ? json_required_finite<float>(j, "color_factor_a", path)
                                    : 1.0f;
                m.color_factor = glm::vec4(rgb, a);
            }
            if (const auto it = j.find("metallic"); it != j.end() && !it->is_null())
            {
                m.metallic = json_required_finite<float>(j, "metallic", path);
            }
            if (const auto it = j.find("roughness"); it != j.end() && !it->is_null())
            {
                m.roughness = json_required_finite<float>(j, "roughness", path);
            }
            return m;
        }

        json serialize_material_def(const ScenarioConfig::MaterialDef &m)
        {
            json j;
            if (!m.albedo.empty()) j["albedo"] = m.albedo;
            if (!m.normal.empty()) j["normal"] = m.normal;
            if (!m.metal_rough.empty()) j["metal_rough"] = m.metal_rough;
            if (!m.occlusion.empty()) j["occlusion"] = m.occlusion;
            if (!m.emissive.empty()) j["emissive"] = m.emissive;
            if (m.color_factor != glm::vec4(1.0f))
            {
                j["color_factor"] = {{"x", m.color_factor.x}, {"y", m.color_factor.y}, {"z", m.color_factor.z}};
                if (m.color_factor.w != 1.0f) j["color_factor_a"] = m.color_factor.w;
            }
            if (m.metallic != 0.0f) j["metallic"] = m.metallic;
            if (m.roughness != 0.5f) j["roughness"] = m.roughness;
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
            if (const auto it = j.find("formation_hold_enabled"); it != j.end() && !it->is_null())
            {
                o.formation_hold_enabled = json_required<bool>(j, "formation_hold_enabled", path);
            }
            if (const auto it = j.find("formation_leader"); it != j.end() && !it->is_null())
            {
                o.formation_leader = json_required<std::string>(j, "formation_leader", path);
            }
            if (const auto it = j.find("formation_slot_lvlh_m"); it != j.end() && !it->is_null())
            {
                o.formation_slot_lvlh_m = parse_dvec3(
                        *json_required_object(j, "formation_slot_lvlh_m", path),
                        child_path(path, "formation_slot_lvlh_m"));
            }
            if (const auto it = j.find("prediction_group"); it != j.end() && !it->is_null())
            {
                o.prediction_group = json_required<std::string>(j, "prediction_group", path);
            }
            if (const auto it = j.find("prediction_orbit_color"); it != j.end() && !it->is_null())
            {
                o.prediction_orbit_color = parse_vec3(*json_required_object(j, "prediction_orbit_color", path),
                                                      child_path(path, "prediction_orbit_color"));
                o.has_prediction_orbit_color = true;
            }
            if (const auto it = j.find("gltf_path"); it != j.end() && !it->is_null())
            {
                o.gltf_path = json_required<std::string>(j, "gltf_path", path);
            }
            if (const auto it = j.find("material"); it != j.end() && !it->is_null())
            {
                o.material = parse_material_def(*it, child_path(path, "material"));
            }
            if (const auto it = j.find("primitive"); it != j.end() && !it->is_null())
            {
                o.primitive = parse_primitive_type(
                        json_required<std::string>(j, "primitive", path),
                        child_path(path, "primitive"));
            }
            else if (o.gltf_path.empty())
            {
                fail(child_path(path, "primitive") + " is required when gltf_path is not set");
            }
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
            if (const auto has_gltf = !o.gltf_path.empty(); has_gltf)
            {
                if (o.gltf_path.front() == '/' || o.gltf_path.front() == '\\')
                {
                    fail(child_path(path, "gltf_path") + " must be relative to the assets directory");
                }
            }
            if (o.orbit_altitude_m < 0.0)
            {
                fail(child_path(path, "orbit_altitude_m") + " must be >= 0");
            }
            if (o.render_scale.x <= 0.0f || o.render_scale.y <= 0.0f || o.render_scale.z <= 0.0f)
            {
                fail(child_path(path, "render_scale") + " components must be > 0");
            }
            if (o.formation_hold_enabled && o.formation_leader.empty())
            {
                fail(child_path(path, "formation_leader") + " must not be empty when formation_hold_enabled=true");
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
            j["formation_hold_enabled"] = o.formation_hold_enabled;
            if (!o.formation_leader.empty())
            {
                j["formation_leader"] = o.formation_leader;
            }
            const double formation_slot_len2 =
                    (o.formation_slot_lvlh_m.x * o.formation_slot_lvlh_m.x) +
                    (o.formation_slot_lvlh_m.y * o.formation_slot_lvlh_m.y) +
                    (o.formation_slot_lvlh_m.z * o.formation_slot_lvlh_m.z);
            if (o.formation_hold_enabled || formation_slot_len2 > 0.0)
            {
                j["formation_slot_lvlh_m"] = {
                        {"x", o.formation_slot_lvlh_m.x},
                        {"y", o.formation_slot_lvlh_m.y},
                        {"z", o.formation_slot_lvlh_m.z}};
            }
            if (!o.prediction_group.empty())
            {
                j["prediction_group"] = o.prediction_group;
            }
            if (o.has_prediction_orbit_color)
            {
                j["prediction_orbit_color"] = {
                        {"x", o.prediction_orbit_color.x},
                        {"y", o.prediction_orbit_color.y},
                        {"z", o.prediction_orbit_color.z}};
            }
            if (!o.gltf_path.empty())
            {
                j["gltf_path"] = o.gltf_path;
            }
            else
            {
                j["primitive"] = primitive_type_string(o.primitive);
            }
            if (o.material.has_any_texture())
            {
                j["material"] = serialize_material_def(o.material);
            }
            j["render_scale"] = {{"x", o.render_scale.x}, {"y", o.render_scale.y}, {"z", o.render_scale.z}};
            j["body_settings"] = serialize_body_settings(o.body_settings);
            j["is_player"] = o.is_player;
            j["is_rebase_anchor"] = o.is_rebase_anchor;
            return j;
        }
        // ---- EnvironmentDef ----

        ScenarioConfig::EnvironmentDef parse_environment_def(const json &j, const std::string &path)
        {
            if (!j.is_object())
            {
                fail(path + " must be an object");
            }

            ScenarioConfig::EnvironmentDef e{};
            e.ibl_specular = json_required<std::string>(j, "ibl_specular", path);
            e.ibl_diffuse = json_required<std::string>(j, "ibl_diffuse", path);
            e.ibl_brdf_lut = json_required<std::string>(j, "ibl_brdf_lut", path);
            e.ibl_background = json_required<std::string>(j, "ibl_background", path);
            e.has_atmosphere = json_required<bool>(j, "has_atmosphere", path);
            e.has_particles = json_required<bool>(j, "has_particles", path);
            if (const auto it = j.find("rocket_plume_noise"); it != j.end() && !it->is_null())
            {
                e.rocket_plume_noise = it->get<std::string>();
            }
            return e;
        }

        json serialize_environment_def(const ScenarioConfig::EnvironmentDef &e)
        {
            json j;
            j["ibl_specular"] = e.ibl_specular;
            j["ibl_diffuse"] = e.ibl_diffuse;
            j["ibl_brdf_lut"] = e.ibl_brdf_lut;
            j["ibl_background"] = e.ibl_background;
            j["has_atmosphere"] = e.has_atmosphere;
            j["has_particles"] = e.has_particles;
            if (!e.rocket_plume_noise.empty())
            {
                j["rocket_plume_noise"] = e.rocket_plume_noise;
            }
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

            if (const auto it = root.find("environment"); it != root.end() && !it->is_null())
            {
                cfg.environment = parse_environment_def(*it, "root.environment");
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

        root["environment"] = serialize_environment_def(config.environment);

        return root.dump(2);
    }

    bool save_scenario_config(const std::string &json_path, const ScenarioConfig &config)
    {
        if (json_path.empty())
        {
            Logger::error("Scenario save path is empty.");
            return false;
        }

        try
        {
            const std::filesystem::path out_path(json_path);
            const std::filesystem::path parent = out_path.parent_path();

            if (!parent.empty())
            {
                std::error_code ec;
                std::filesystem::create_directories(parent, ec);
                if (ec)
                {
                    Logger::error("Failed to create scenario directory '{}': {}", parent.string(), ec.message());
                    return false;
                }
            }

            std::ofstream file(out_path, std::ios::out | std::ios::trunc);
            if (!file.is_open())
            {
                Logger::error("Failed to open scenario file for writing: {}", json_path);
                return false;
            }

            file << serialize_scenario_config(config);
            if (!file.good())
            {
                Logger::error("Failed to write scenario file: {}", json_path);
                return false;
            }

            Logger::info("Saved scenario '{}': {} celestials, {} orbiters",
                         json_path, config.celestials.size(), config.orbiters.size());
            return true;
        }
        catch (const std::exception &e)
        {
            Logger::error("Scenario '{}' save failed: {}", json_path, e.what());
            return false;
        }
    }
} // namespace Game
