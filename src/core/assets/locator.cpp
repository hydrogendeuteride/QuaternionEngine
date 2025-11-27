#include "locator.h"

#include <cstdlib>

using std::filesystem::path;

static path get_env_path(const char *name)
{
    const char *v = std::getenv(name);
    if (!v || !*v) return {};
    path p = v;
    if (std::filesystem::exists(p)) return std::filesystem::canonical(p);
    return {};
}

static path find_upwards_containing(path start, const std::string &subdir, int maxDepth = 6)
{
    path cur = std::filesystem::weakly_canonical(start);
    for (int i = 0; i <= maxDepth; i++)
    {
        path candidate = cur / subdir;
        if (std::filesystem::exists(candidate)) return cur;
        if (!cur.has_parent_path()) break;
        cur = cur.parent_path();
    }
    return {};
}

AssetPaths AssetPaths::detect(const path &startDir)
{
    AssetPaths out{};

    if (auto root = get_env_path("VKG_ASSET_ROOT"); !root.empty())
    {
        out.root = root;
        if (std::filesystem::exists(root / "assets")) out.assets = root / "assets";
        if (std::filesystem::exists(root / "shaders")) out.shaders = root / "shaders";
        return out;
    }

    if (auto aroot = find_upwards_containing(startDir, "assets"); !aroot.empty())
    {
        out.assets = aroot / "assets";
        out.root = aroot;
    }
    if (auto sroot = find_upwards_containing(startDir, "shaders"); !sroot.empty())
    {
        out.shaders = sroot / "shaders";
        if (out.root.empty()) out.root = sroot;
    }

    if (out.assets.empty())
    {
        path p1 = startDir / "assets";
        path p2 = startDir / ".." / "assets";
        if (std::filesystem::exists(p1)) out.assets = p1;
        else if (std::filesystem::exists(p2)) out.assets = std::filesystem::weakly_canonical(p2);
    }
    if (out.shaders.empty())
    {
        path p1 = startDir / "shaders";
        path p2 = startDir / ".." / "shaders";
        if (std::filesystem::exists(p1)) out.shaders = p1;
        else if (std::filesystem::exists(p2)) out.shaders = std::filesystem::weakly_canonical(p2);
    }

    return out;
}

void AssetLocator::init()
{
    _paths = AssetPaths::detect();
}

bool AssetLocator::file_exists(const path &p)
{
    std::error_code ec;
    return !p.empty() && std::filesystem::exists(p, ec) && std::filesystem::is_regular_file(p, ec);
}

std::string AssetLocator::resolve_in(const path &base, std::string_view name)
{
    if (name.empty()) return {};
    path in = base / std::string(name);
    if (file_exists(in)) return in.string();
    return {};
}

std::string AssetLocator::shaderPath(std::string_view name) const
{
    if (name.empty()) return {};
    path np = std::string(name);

    if (np.is_absolute() && file_exists(np)) return np.string();
    if (file_exists(np)) return np.string();

    if (!_paths.shaders.empty())
    {
        if (auto r = resolve_in(_paths.shaders, name); !r.empty()) return r;
    }

    if (auto r = resolve_in(std::filesystem::current_path() / "shaders", name); !r.empty()) return r;
    if (auto r = resolve_in(std::filesystem::current_path() / ".." / "shaders", name); !r.empty()) return r;

    return np.string();
}

std::string AssetLocator::assetPath(std::string_view name) const
{
    if (name.empty()) return {};
    path np = std::string(name);
    if (np.is_absolute() && file_exists(np)) return np.string();
    if (file_exists(np)) return np.string();

    if (!_paths.assets.empty())
    {
        if (auto r = resolve_in(_paths.assets, name); !r.empty()) return r;
    }

    if (auto r = resolve_in(std::filesystem::current_path() / "assets", name); !r.empty()) return r;
    if (auto r = resolve_in(std::filesystem::current_path() / ".." / "assets", name); !r.empty()) return r;

    return np.string();
}

