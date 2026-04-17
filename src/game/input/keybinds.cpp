#include "game/input/keybinds.h"

#include "core/input/keybinds.h"
#include "core/util/logger.h"

#include <toml++/toml.hpp>

#include <filesystem>
#include <fstream>
#include <functional>
#include <string_view>
#include <vector>

namespace Game
{
    namespace
    {
        // Generic mapping from a TOML string field to a Key member of some subsystem struct T.
        template<typename T>
        struct KeyField
        {
            const char *toml_key;
            Key T::*member;
        };

        template<typename T, std::size_t N>
        void load_fields(const toml::table &section,
                         std::string_view section_name,
                         const std::string &toml_path,
                         const KeyField<T> (&fields)[N],
                         T &out)
        {
            for (const auto &f : fields)
            {
                if (auto value = section[f.toml_key].template value<std::string>())
                {
                    if (const auto key = key_from_string(*value))
                    {
                        out.*f.member = *key;
                    }
                    else
                    {
                        Logger::warn(
                            "Invalid keybind '{}.{}' = '{}' in '{}'; keeping default '{}'.",
                            section_name, f.toml_key, *value, toml_path, key_to_string(out.*f.member));
                    }
                }
            }
        }

        template<typename T, std::size_t N>
        toml::table write_fields(const KeyField<T> (&fields)[N], const T &in)
        {
            toml::table tbl;
            for (const auto &f : fields)
            {
                tbl.insert_or_assign(f.toml_key, std::string(key_to_string(in.*f.member)));
            }
            return tbl;
        }

        // ---- Per-subsystem field tables ------------------------------------------------

        constexpr KeyField<ShipKeybinds> kShipFields[] = {
            {"thrust_forward", &ShipKeybinds::thrust_forward},
            {"thrust_back", &ShipKeybinds::thrust_back},
            {"thrust_left", &ShipKeybinds::thrust_left},
            {"thrust_right", &ShipKeybinds::thrust_right},
            {"thrust_up", &ShipKeybinds::thrust_up},
            {"thrust_down", &ShipKeybinds::thrust_down},
            {"pitch_up", &ShipKeybinds::pitch_up},
            {"pitch_down", &ShipKeybinds::pitch_down},
            {"yaw_left", &ShipKeybinds::yaw_left},
            {"yaw_right", &ShipKeybinds::yaw_right},
            {"roll_left", &ShipKeybinds::roll_left},
            {"roll_right", &ShipKeybinds::roll_right},
            {"sas_toggle", &ShipKeybinds::sas_toggle},
        };

        // ---- Section registry ----------------------------------------------------------
        //
        // Each section ties a TOML table name to load/write callbacks bound to a specific
        // subsystem struct. Adding a new category: declare its struct + defaults in the
        // header, add a kXxxFields table above, and append an entry to kSections below.

        struct SectionBinding
        {
            std::string_view toml_section;
            std::function<void(const toml::table &, std::string_view, const std::string &, Keybinds &)> load;
            std::function<toml::table(const Keybinds &)> save;
        };

        const std::vector<SectionBinding> &sections()
        {
            static const std::vector<SectionBinding> kSections = {
                {
                    "ship",
                    [](const toml::table &tbl, std::string_view section_name, const std::string &toml_path,
                       Keybinds &kb)
                    {
                        load_fields(tbl, section_name, toml_path, kShipFields, kb.ship);
                    },
                    [](const Keybinds &kb) { return write_fields(kShipFields, kb.ship); },
                },
            };
            return kSections;
        }
    } // namespace

    std::optional<Keybinds> load_keybinds(const std::string &toml_path)
    {
        if (!std::filesystem::exists(toml_path))
        {
            Logger::info("No keybinds file: {}", toml_path);
            return std::nullopt;
        }

        toml::table root;
        try
        {
            root = toml::parse_file(toml_path);
        }
        catch (const toml::parse_error &e)
        {
            Logger::error("Keybinds TOML parse error '{}': {}", toml_path, e.what());
            return std::nullopt;
        }

        const int version = static_cast<int>(root["schema_version"].value_or<int64_t>(0));
        if (version != Keybinds::kSchemaVersion)
        {
            Logger::warn("Keybinds schema_version {} != {}, ignoring: {}",
                         version, Keybinds::kSchemaVersion, toml_path);
            return std::nullopt;
        }

        Keybinds kb{};
        for (const auto &section : sections())
        {
            if (const toml::table *tbl = root[section.toml_section].as_table())
            {
                section.load(*tbl, section.toml_section, toml_path, kb);
            }
        }

        Logger::info("Loaded keybinds: {}", toml_path);
        return kb;
    }

    bool save_keybinds(const std::string &toml_path, const Keybinds &kb)
    {
        if (toml_path.empty())
        {
            Logger::error("Keybinds save path is empty.");
            return false;
        }

        try
        {
            const std::filesystem::path out_path(toml_path);
            const std::filesystem::path parent = out_path.parent_path();
            if (!parent.empty())
            {
                std::error_code ec;
                std::filesystem::create_directories(parent, ec);
                if (ec)
                {
                    Logger::error("Failed to create keybinds directory '{}': {}", parent.string(), ec.message());
                    return false;
                }
            }

            toml::table root;
            root.insert_or_assign("schema_version", static_cast<int64_t>(Keybinds::kSchemaVersion));
            for (const auto &section : sections())
            {
                root.insert_or_assign(section.toml_section, section.save(kb));
            }

            std::ofstream file(out_path, std::ios::out | std::ios::trunc);
            if (!file.is_open())
            {
                Logger::error("Failed to open keybinds for writing: {}", toml_path);
                return false;
            }
            file << root;
            if (!file.good())
            {
                Logger::error("Failed to write keybinds: {}", toml_path);
                return false;
            }

            Logger::info("Saved keybinds: {}", toml_path);
            return true;
        }
        catch (const std::exception &e)
        {
            Logger::error("Keybinds save failed '{}': {}", toml_path, e.what());
            return false;
        }
    }
} // namespace Game
