#include "game/states/gameplay/orbit_helpers.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{
    constexpr double kTol = 1e-9;

    void expect_vec3_near(const glm::dvec3 &actual, const glm::dvec3 &expected, const double tol)
    {
        EXPECT_NEAR(actual.x, expected.x, tol);
        EXPECT_NEAR(actual.y, expected.y, tol);
        EXPECT_NEAR(actual.z, expected.z, tol);
    }

    void expect_vec3_zero(const glm::dvec3 &actual, const double tol = kTol)
    {
        expect_vec3_near(actual, glm::dvec3(0.0), tol);
    }

    Game::OrbitalScenario make_single_body_scenario(const double gravitational_constant,
                                                    const double reference_mass_kg,
                                                    const glm::dvec3 &reference_position_m = glm::dvec3(0.0))
    {
        Game::OrbitalScenario scenario{};

        orbitsim::GameSimulation::Config cfg{};
        cfg.gravitational_constant = gravitational_constant;
        cfg.softening_length_m = 0.0;
        cfg.enable_events = false;
        scenario.sim = orbitsim::GameSimulation(cfg);

        orbitsim::MassiveBody ref{};
        ref.mass_kg = reference_mass_kg;
        ref.state = orbitsim::make_state(reference_position_m, glm::dvec3(0.0));

        const auto ref_handle = scenario.sim.create_body(ref);
        if (!ref_handle.valid())
        {
            return {};
        }

        Game::CelestialBodyInfo ref_info{};
        ref_info.sim_id = ref_handle.id;
        ref_info.name = "reference";
        ref_info.mass_kg = reference_mass_kg;
        scenario.bodies.push_back(ref_info);
        scenario.reference_body_index = 0;

        return scenario;
    }
} // namespace

TEST(OrbitHelpers, CircularOrbitRelativeStateProducesOrthogonalPositionAndVelocity)
{
    constexpr double G = 6.67430e-11;
    constexpr double mass_kg = 5.972e24;
    constexpr double radius_m = 6'771'000.0;
    constexpr double arg_latitude_rad = 0.42;

    const auto rel = Game::detail::circular_orbit_relative_state_xz(G, mass_kg, radius_m, arg_latitude_rad);

    const double speed_expected_mps = std::sqrt((G * mass_kg) / radius_m);
    const double speed_actual_mps = glm::length(glm::dvec3(rel.velocity_mps));
    const double radius_actual_m = glm::length(glm::dvec3(rel.position_m));
    const double dot_rv = glm::dot(glm::dvec3(rel.position_m), glm::dvec3(rel.velocity_mps));

    EXPECT_NEAR(radius_actual_m, radius_m, 1e-6);
    EXPECT_NEAR(speed_actual_mps, speed_expected_mps, 1e-6);
    EXPECT_NEAR(dot_rv, 0.0, 1e-3);
}

TEST(OrbitHelpers, CircularOrbitRelativeStateReturnsZeroForInvalidInputs)
{
    const auto invalid_mass = Game::detail::circular_orbit_relative_state_xz(1.0, 0.0, 10.0);
    expect_vec3_zero(glm::dvec3(invalid_mass.position_m));
    expect_vec3_zero(glm::dvec3(invalid_mass.velocity_mps));

    const auto invalid_radius = Game::detail::circular_orbit_relative_state_xz(1.0, 10.0, 0.0);
    expect_vec3_zero(glm::dvec3(invalid_radius.position_m));
    expect_vec3_zero(glm::dvec3(invalid_radius.velocity_mps));

    const auto invalid_g = Game::detail::circular_orbit_relative_state_xz(std::numeric_limits<double>::infinity(), 10.0, 10.0);
    expect_vec3_zero(glm::dvec3(invalid_g.position_m));
    expect_vec3_zero(glm::dvec3(invalid_g.velocity_mps));
}

TEST(OrbitHelpers, TwoBodyBarycentricStatePreservesCenterOfMass)
{
    constexpr double G = 10.0;
    constexpr double mass_a_kg = 20.0;
    constexpr double mass_b_kg = 5.0;
    constexpr double separation_m = 40.0;

    const auto states = Game::detail::two_body_circular_barycentric_xz(
            G, mass_a_kg, mass_b_kg, separation_m, 0.0);

    const glm::dvec3 com_position =
            (mass_a_kg * glm::dvec3(states.state_a.position_m)) + (mass_b_kg * glm::dvec3(states.state_b.position_m));
    const glm::dvec3 com_velocity =
            (mass_a_kg * glm::dvec3(states.state_a.velocity_mps)) + (mass_b_kg * glm::dvec3(states.state_b.velocity_mps));

    const glm::dvec3 rel_position = glm::dvec3(states.state_b.position_m) - glm::dvec3(states.state_a.position_m);
    const glm::dvec3 rel_velocity = glm::dvec3(states.state_b.velocity_mps) - glm::dvec3(states.state_a.velocity_mps);
    const double expected_relative_speed = std::sqrt((G * (mass_a_kg + mass_b_kg)) / separation_m);

    expect_vec3_zero(com_position, 1e-12);
    expect_vec3_zero(com_velocity, 1e-12);
    EXPECT_NEAR(glm::length(rel_position), separation_m, 1e-12);
    EXPECT_NEAR(glm::length(rel_velocity), expected_relative_speed, 1e-12);
}

TEST(OrbitHelpers, PointMassAccelMatchesAnalyticResultAndHandlesInvalidInputs)
{
    const glm::dvec3 accel = Game::detail::point_mass_accel(10.0, 5.0, glm::dvec3(2.0, 0.0, 0.0), 0.0);
    expect_vec3_near(accel, glm::dvec3(-12.5, 0.0, 0.0), 1e-12);

    const glm::dvec3 invalid_mass = Game::detail::point_mass_accel(10.0, 0.0, glm::dvec3(2.0, 0.0, 0.0), 0.0);
    expect_vec3_zero(invalid_mass);

    const glm::dvec3 invalid_radius = Game::detail::point_mass_accel(10.0, 5.0, glm::dvec3(0.0), 0.0);
    expect_vec3_zero(invalid_radius);
}

TEST(OrbitHelpers, NbodyAccelBodyCenteredSingleReferenceMatchesPointMass)
{
    constexpr double G = 6.67430e-11;
    constexpr double reference_mass_kg = 5.972e24;
    const glm::dvec3 p_rel_m(6'771'000.0, 0.0, 0.0);

    const Game::OrbitalScenario scenario = make_single_body_scenario(G, reference_mass_kg);

    const glm::dvec3 accel = Game::detail::nbody_accel_body_centered(scenario, p_rel_m);
    const glm::dvec3 expected = Game::detail::point_mass_accel(G, reference_mass_kg, p_rel_m, 0.0);

    expect_vec3_near(accel, expected, 1e-12);
}

TEST(OrbitHelpers, NbodyAccelBodyCenteredReturnsZeroWhenReferenceIsMissing)
{
    const Game::OrbitalScenario empty_scenario{};
    const glm::dvec3 accel = Game::detail::nbody_accel_body_centered(empty_scenario, glm::dvec3(1.0, 0.0, 0.0));
    expect_vec3_zero(accel);
}

TEST(OrbitHelpers, NbodyAccelBodyCenteredCancelsTidalTermAtReferenceOrigin)
{
    constexpr double G = 6.67430e-11;
    constexpr double reference_mass_kg = 5.972e24;
    constexpr double perturbing_mass_kg = 7.342e22;

    Game::OrbitalScenario scenario = make_single_body_scenario(G, reference_mass_kg);

    orbitsim::MassiveBody perturbing{};
    perturbing.mass_kg = perturbing_mass_kg;
    perturbing.state = orbitsim::make_state(glm::dvec3(384'400'000.0, 0.0, 0.0), glm::dvec3(0.0));

    const auto perturbing_handle = scenario.sim.create_body(perturbing);
    ASSERT_TRUE(perturbing_handle.valid());

    Game::CelestialBodyInfo perturbing_info{};
    perturbing_info.sim_id = perturbing_handle.id;
    perturbing_info.name = "perturber";
    perturbing_info.mass_kg = perturbing_mass_kg;
    scenario.bodies.push_back(perturbing_info);

    const glm::dvec3 accel = Game::detail::nbody_accel_body_centered(scenario, glm::dvec3(0.0));
    expect_vec3_zero(accel, 1e-12);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
