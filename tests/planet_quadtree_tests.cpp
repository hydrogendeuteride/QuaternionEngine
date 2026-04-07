#include "scene/planet/planet_heightmap.h"
#include "scene/planet/planet_quadtree.h"

#include <gtest/gtest.h>

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>

namespace
{
    GPUSceneData make_scene_data(const glm::vec3 &sun_dir)
    {
        GPUSceneData scene{};
        scene.proj = glm::mat4(1.0f);
        scene.viewproj = glm::mat4(1.0f);
        scene.sunlightDirection = glm::vec4(sun_dir, 0.0f);
        return scene;
    }

    GPUSceneData make_focused_scene_data(const glm::vec3 &sun_dir, const glm::vec3 &camera_world)
    {
        GPUSceneData scene = make_scene_data(sun_dir);
        scene.view = glm::lookAt(camera_world, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        scene.proj = glm::perspective(glm::radians(45.0f), 16.0f / 9.0f, 0.1f, 20000.0f);
        scene.viewproj = scene.proj * scene.view;
        return scene;
    }

    planet::HeightFace make_checkerboard_face()
    {
        planet::HeightFace face{};
        face.width = 4u;
        face.height = 4u;
        face.texels = {
            0u,   255u, 0u,   255u,
            255u, 0u,   255u, 0u,
            0u,   255u, 0u,   255u,
            255u, 0u,   255u, 0u,
        };
        planet::rebuild_height_mips(face);
        return face;
    }

    std::size_t count_face_leaves(const std::vector<planet::PatchKey> &leaves, const planet::CubeFace face)
    {
        return static_cast<std::size_t>(std::count_if(leaves.begin(), leaves.end(),
                                                      [face](const planet::PatchKey &key) {
                                                          return key.face == face;
                                                      }));
    }
} // namespace

TEST(PlanetQuadtree, VarianceWeightedSSERefinesRoughPatchMoreAggressively)
{
    planet::HeightFaceSet height_faces{};
    height_faces[static_cast<size_t>(planet::CubeFace::PosX)] = make_checkerboard_face();

    planet::PlanetQuadtree::Settings settings{};
    settings.max_level = 1u;
    settings.target_sse_px = 5.0f;
    settings.lod_hysteresis_ratio = 0.0f;
    settings.max_patches_visible = 128u;
    settings.frustum_cull = false;
    settings.horizon_cull = false;

    planet::PlanetQuadtree flat_quadtree{};
    flat_quadtree.set_settings(settings);
    flat_quadtree.update(WorldVec3(0.0),
                         1000.0,
                         0.0,
                         WorldVec3(5000.0, 0.0, 0.0),
                         WorldVec3(0.0),
                         make_scene_data(glm::vec3(0.0f)),
                         VkExtent2D{1920u, 1080u},
                         33u);

    planet::PlanetQuadtree rough_quadtree{};
    rough_quadtree.set_settings(settings);
    rough_quadtree.update(WorldVec3(0.0),
                          1000.0,
                          0.0,
                          WorldVec3(5000.0, 0.0, 0.0),
                          WorldVec3(0.0),
                          make_scene_data(glm::vec3(0.0f)),
                          VkExtent2D{1920u, 1080u},
                          33u,
                          &height_faces);

    EXPECT_EQ(count_face_leaves(flat_quadtree.visible_leaves(), planet::CubeFace::PosX), 1u);
    EXPECT_EQ(count_face_leaves(rough_quadtree.visible_leaves(), planet::CubeFace::PosX), 4u);
}

TEST(PlanetQuadtree, TerminatorWeightedSSERefinesNearTerminator)
{
    planet::PlanetQuadtree::Settings settings{};
    settings.max_level = 1u;
    settings.target_sse_px = 7.0f;
    settings.lod_hysteresis_ratio = 0.0f;
    settings.max_patches_visible = 128u;
    settings.frustum_cull = true;
    settings.horizon_cull = true;

    planet::PlanetQuadtree noon_quadtree{};
    noon_quadtree.set_settings(settings);
    noon_quadtree.update(WorldVec3(0.0),
                         1000.0,
                         0.0,
                         WorldVec3(7000.0, 0.0, 0.0),
                         WorldVec3(0.0),
                         make_focused_scene_data(glm::normalize(glm::vec3(1.0f, 1.0f, 1.0f)),
                                                 glm::vec3(7000.0f, 0.0f, 0.0f)),
                         VkExtent2D{1920u, 1080u},
                         33u);

    planet::PlanetQuadtree terminator_quadtree{};
    terminator_quadtree.set_settings(settings);
    terminator_quadtree.update(WorldVec3(0.0),
                               1000.0,
                               0.0,
                               WorldVec3(7000.0, 0.0, 0.0),
                               WorldVec3(0.0),
                               make_focused_scene_data(glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(7000.0f, 0.0f, 0.0f)),
                               VkExtent2D{1920u, 1080u},
                               33u);

    EXPECT_EQ(count_face_leaves(noon_quadtree.visible_leaves(), planet::CubeFace::PosX), 1u);
    EXPECT_EQ(count_face_leaves(terminator_quadtree.visible_leaves(), planet::CubeFace::PosX), 4u);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
