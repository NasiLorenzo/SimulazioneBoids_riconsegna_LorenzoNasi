#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "statistics.hpp"
#include "doctest.h"
using namespace boids;

TEST_CASE("Testig Sample class")
{
  std::vector<double> test_set{1., 1., 1., 1., 2.};
  Sample test_sample{test_set};

  SUBCASE("Testing basic functionality")
  {
    CHECK(test_sample.result().mean == doctest::Approx(1.2));
    CHECK(test_sample.result().sigma == doctest::Approx(0.4));
  }
  test_sample.add(3.);
  SUBCASE("Testing after addition")
  {
    CHECK(test_sample.result().mean == doctest::Approx(1.5));
    CHECK(test_sample.result().sigma
          == doctest::Approx(0.7637).epsilon(0.0001));
  }
  std::vector<double> empty_set{};
  Sample empty_sample{empty_set};
  SUBCASE("Testing with empty set")
  {
    auto result = empty_sample.result();
    CHECK(result.mean == doctest::Approx(0.));
    CHECK(result.sigma == doctest::Approx(-1.));
  }
}
TEST_CASE("Testing statistics of a flock")
{
  BoidState boid1 = {DoubleVec{0., 1.}, DoubleVec{1., 1.}};
  BoidState boid2 = {DoubleVec{1., 2.}, DoubleVec{2., 3.}};
  BoidState boid3 = {DoubleVec{2., 3.}, DoubleVec{3., 4.}};
  BoidState boid4 = {DoubleVec{3., 5.}, DoubleVec{4., 6.}};
  std::vector<BoidState> flock{boid1, boid2, boid3, boid4};
  FlockStats flock_data{flock};
  CHECK(flock_data.pos_stats[0].result().mean == doctest::Approx(1.5));
  CHECK(flock_data.pos_stats[0].result().sigma
        == doctest::Approx(1.118).epsilon(0.001));

  CHECK(flock_data.pos_stats[1].result().mean == doctest::Approx(2.75));
  CHECK(flock_data.pos_stats[1].result().sigma
        == doctest::Approx(1.479).epsilon(0.001));

  CHECK(flock_data.vel_stats[0].result().mean == doctest::Approx(2.5));
  CHECK(flock_data.vel_stats[0].result().sigma
        == doctest::Approx(1.118).epsilon(0.001));

  CHECK(flock_data.vel_stats[1].result().mean == doctest::Approx(3.5));
  CHECK(flock_data.vel_stats[1].result().sigma
        == doctest::Approx(1.802).epsilon(0.001));

  CHECK(flock_data.pos_mod_stats.result().mean
        == doctest::Approx(3.168).epsilon(0.001));
  CHECK(flock_data.pos_mod_stats.result().sigma
        == doctest::Approx(1.792).epsilon(0.001));

  CHECK(flock_data.vel_mod_stats.result().mean
        == doctest::Approx(4.307).epsilon(0.001));
  CHECK(flock_data.vel_mod_stats.result().sigma
        == doctest::Approx(2.101).epsilon(0.01));

  CHECK(flock_data.distance_stats.result().mean
        == doctest::Approx(2.749).epsilon(0.001));
  CHECK(flock_data.distance_stats.result().sigma
        == doctest::Approx(1.267).epsilon(0.001));
}