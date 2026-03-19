#pragma once

#include "core/world.h"
#include "orbitsim/trajectory_segments.hpp"

#include <glm/glm.hpp>
#include <glm/mat4x4.hpp>

#include <cstdint>
#include <cstddef>
#include <span>
#include <vector>

namespace Game
{
    /// Binary-tree LOD structure over trajectory segments.
    ///
    /// build() constructs a balanced binary tree where each internal node merges
    /// a range of source Hermite segments into a single coarser segment and records
    /// its worst-case approximation error (in meters).
    ///
    /// At runtime, build_render_lod() / build_pick_lod() descend the tree, select
    /// segments at the appropriate fidelity for the current camera, and produce
    /// world-space line segments for drawing or picking.
    ///
    /// File layout:
    ///   orbit_render_curve.h              -- types and public interface
    ///   orbit_render_curve_internal.h     -- shared inline helpers (frustum, eval)
    ///   orbit_render_curve.cpp            -- tree build, LOD selection, resolve
    ///   orbit_render_curve_render.cpp     -- adaptive subdivision for rendering
    ///   orbit_render_curve_pick.cpp       -- frustum cull + downsampling for picking
    class OrbitRenderCurve
    {
    public:
        static constexpr uint32_t kInvalidNodeIndex = 0xffffffffu;

        /// Camera parameters needed for pixel-error LOD decisions.
        struct CameraContext
        {
            glm::dvec3 camera_world{0.0, 0.0, 0.0};
            double tan_half_fov{0.0};
            double viewport_height_px{1.0};
        };

        /// Controls render-LOD subdivision budget and quality.
        struct RenderSettings
        {
            double error_px{0.75};            ///< max allowed chord-vs-curve error in screen pixels
            std::size_t max_segments{4000};   ///< hard cap on output line segments
        };

        /// A single world-space line segment with its source time range.
        /// Shared output type for both rendering and picking paths.
        struct LineSegment
        {
            WorldVec3 a_world{0.0, 0.0, 0.0};
            WorldVec3 b_world{0.0, 0.0, 0.0};
            double t0_s{0.0};
            double t1_s{0.0};
        };

        /// Output of build_render_lod(). cap_hit is set when max_segments was reached.
        struct RenderResult
        {
            std::vector<LineSegment> segments{};
            bool cap_hit{false};
        };

        /// View-projection context for frustum culling.
        /// When valid==false, frustum culling is skipped (all segments accepted).
        struct FrustumContext
        {
            bool valid{false};
            glm::mat4 viewproj{1.0f};
            WorldVec3 origin_world{0.0, 0.0, 0.0};
        };

        /// Controls pick-LOD frustum margin and segment budget.
        struct PickSettings
        {
            std::size_t max_segments{8000};
            double frustum_margin_ratio{0.05}; ///< NDC margin for frustum accept (fraction of clip extent)
        };

        /// Output of build_pick_lod(). Includes pre/post-cull counts for diagnostics.
        struct PickResult
        {
            std::vector<LineSegment> segments{};
            std::size_t segments_before_cull{0};
            std::size_t segments_after_cull{0};
            bool cap_hit{false};
        };

        /// A node in the binary LOD tree.
        /// Leaf nodes hold a single source TrajectorySegment (max_error_m == 0).
        /// Internal nodes hold a merged coarser segment spanning their children,
        /// with max_error_m recording the worst-case approximation error.
        struct Node
        {
            orbitsim::TrajectorySegment segment{};
            uint32_t left_child{kInvalidNodeIndex};
            uint32_t right_child{kInvalidNodeIndex};
            double max_error_m{0.0};

            [[nodiscard]] bool is_leaf() const
            {
                return left_child == kInvalidNodeIndex || right_child == kInvalidNodeIndex;
            }
        };

        /// All parameters needed for tree-based LOD selection.
        /// Bundles camera, reference frame, and anchor times into one struct
        /// so that the curve-based overloads of build_render_lod / build_pick_lod
        /// can share the select+transform logic via resolve_curve_segments().
        struct SelectionContext
        {
            WorldVec3 reference_body_world{0.0, 0.0, 0.0};  ///< world position of the reference body
            WorldVec3 align_delta_world{0.0, 0.0, 0.0};     ///< additional world offset (e.g. rebasing)
            glm::dmat3 frame_to_world{1.0};                  ///< BCI-to-world rotation (identity if already world-aligned)
            glm::dvec3 camera_world{0.0, 0.0, 0.0};
            double tan_half_fov{0.0};
            double viewport_height_px{1.0};
            double error_px{0.75};                           ///< pixel-error threshold for tree descent
            std::span<const double> anchor_times_s{};        ///< times that must lie on segment boundaries
        };

        [[nodiscard]] bool empty() const
        {
            return _root_index == kInvalidNodeIndex || _nodes.empty();
        }

        [[nodiscard]] uint32_t root_index() const
        {
            return _root_index;
        }

        [[nodiscard]] std::span<const Node> nodes() const
        {
            return std::span<const Node>(_nodes.data(), _nodes.size());
        }

        void clear();

        /// Build the binary LOD tree from a flat array of source trajectory segments.
        /// Each leaf wraps one source segment; internal nodes merge ranges via Hermite
        /// endpoint interpolation and record worst-case approximation error.
        static OrbitRenderCurve build(std::span<const orbitsim::TrajectorySegment> source_segments);

        // -- Render LOD (orbit_render_curve_render.cpp) --

        /// Produce render line segments from a flat segment array.
        /// Adaptively subdivides each segment until chord-vs-curve error < error_px,
        /// skipping segments outside the frustum.
        static RenderResult build_render_lod(
                std::span<const orbitsim::TrajectorySegment> segments_bci,
                const WorldVec3 &reference_body_world,
                const WorldVec3 &align_delta_world,
                const CameraContext &camera,
                const RenderSettings &settings,
                double t_start_s,
                double t_end_s,
                const FrustumContext &frustum);

        /// Convenience overload: selects segments from the LOD tree first,
        /// then delegates to the span-based overload above.
        static RenderResult build_render_lod(
                const OrbitRenderCurve &curve,
                const SelectionContext &ctx,
                const FrustumContext &frustum,
                std::size_t max_segments,
                double t_start_s,
                double t_end_s);

        // -- Pick LOD (orbit_render_curve_pick.cpp) --

        /// Produce pick line segments from a flat segment array.
        /// Frustum-culls first, then downsamples uniformly if over budget,
        /// preserving anchor-time segments and endpoints.
        static PickResult build_pick_lod(
                std::span<const orbitsim::TrajectorySegment> segments_bci,
                const WorldVec3 &reference_body_world,
                const WorldVec3 &align_delta_world,
                const FrustumContext &frustum,
                const PickSettings &settings,
                double t_start_s,
                double t_end_s,
                std::span<const double> anchor_times_s = {});

        /// Convenience overload: selects segments from the LOD tree first,
        /// then delegates to the span-based overload above.
        static PickResult build_pick_lod(
                const OrbitRenderCurve &curve,
                const SelectionContext &ctx,
                const FrustumContext &frustum,
                const PickSettings &settings,
                double t_start_s,
                double t_end_s);

        /// Walk the LOD tree and collect segments at the appropriate fidelity.
        /// Descends into children when the node's merge error exceeds the screen-pixel
        /// threshold, or when anchor times / time-window boundaries fall inside the node.
        static void select_segments(const OrbitRenderCurve &curve,
                                    const SelectionContext &ctx,
                                    double t_start_s,
                                    double t_end_s,
                                    std::vector<orbitsim::TrajectorySegment> &out_segments);

    private:
        /// select_segments() + optional BCI-to-world frame transform.
        /// Shared entry point for both curve-based build_render_lod and build_pick_lod.
        static std::vector<orbitsim::TrajectorySegment> resolve_curve_segments(
                const OrbitRenderCurve &curve,
                const SelectionContext &ctx,
                double t_start_s,
                double t_end_s);

        std::vector<Node> _nodes{};
        uint32_t _root_index{kInvalidNodeIndex};
    };
} // namespace Game
