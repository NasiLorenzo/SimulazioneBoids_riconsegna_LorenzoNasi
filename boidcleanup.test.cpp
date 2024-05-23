#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boidcleanup.hpp"
#include "doctest.h"
using namespace boids;
TEST_CASE("Testing rules")
{

  paramms::repulsione  = 0.7;
  paramms::steering    = 0.1;
  paramms::coesione    = 0.1;
  paramms::neigh2      = 1000000;
  paramms::neigh_align = 1000000;
  paramms::alpha=M_PI;
  paramms::attraction=0;
  paramms::speedlimit   = 80;
  paramms::speedminimum = 2; 
  boidstate boid1;
  boid1.pos = {700., 0.};
  boid1.vel = {300., 0.};
  boidstate boid2;
  boid2.pos = {500., 0.};
  boid2.vel = {5., 0.};
  boidstate boid3;
  boid3.pos = {800., 0.};
  boid3.vel = {-88., 0.};
  boidstate boid4;
  boid4.pos = {1000., 0.};
  boid4.vel = {400., 0.};
  boidstate boid5;
  boid5.pos = {200., 0.};
  boid5.vel = {300., 0.};
  boidstate boid6;
  boid6.pos = {300., 0.};
  boid6.vel = {-100., 0.};
  boidstate boid7;
  boid7.pos = {450., 0.};
  boid7.vel = {200., 0.};
  boidstate boid8;
  boid8.pos = {50., 110.};
  boid8.vel = {400., 0.};
  boidstate boid9;
  boid9.pos = {120., 990.};
  boid9.vel = {50., 0.};
  boidstate boid10;
  boid10.pos = {910., 0.};
  boid10.vel = {-300., 0.};

  stormo set{boid1, boid2, boid3, boid4, boid5, boid6, boid7, boid8, boid9, boid10};
  std::random_device r;
  std::default_random_engine eng(r());
  ensemble flock{set};

  flock.update();
  REQUIRE(flock.size_() == 10);

  CHECK(flock.set_()[0].vel[0] == doctest::Approx(1636.744444));
  CHECK(flock.set_()[1].vel[0] == doctest::Approx(-3.255555556));
  CHECK(flock.set_()[2].vel[0] == doctest::Approx(1980.744444));
  CHECK(flock.set_()[3].vel[0] == doctest::Approx(3792.3));
  CHECK(flock.set_()[4].vel[0] == doctest::Approx(-1807.7));
  CHECK(flock.set_()[5].vel[0] == doctest::Approx(-1474.366667));
  CHECK(flock.set_()[6].vel[0] == doctest::Approx(-174.3666667));
  CHECK(flock.set_()[7].vel[0] == doctest::Approx(-2752.144444));
  CHECK(flock.set_()[8].vel[0] == doctest::Approx(-2581.033333));
  CHECK(flock.set_()[9].vel[0] == doctest::Approx(2550.077778));
}

// namespace boids
