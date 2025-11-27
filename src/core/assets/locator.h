#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

struct AssetPaths
{
    std::filesystem::path root;
    std::filesystem::path assets;
    std::filesystem::path shaders;

    bool valid() const
    {
        return (!assets.empty() && std::filesystem::exists(assets)) ||
               (!shaders.empty() && std::filesystem::exists(shaders));
    }

    static AssetPaths detect(const std::filesystem::path &startDir = std::filesystem::current_path());
};

class AssetLocator
{
public:
    void init();

    const AssetPaths &paths() const { return _paths; }
    void setPaths(const AssetPaths &p) { _paths = p; }

    std::string shaderPath(std::string_view name) const;

    std::string assetPath(std::string_view name) const;

    std::string modelPath(std::string_view name) const { return assetPath(name); }

private:
    static bool file_exists(const std::filesystem::path &p);

    static std::string resolve_in(const std::filesystem::path &base, std::string_view name);

    AssetPaths _paths{};
};

