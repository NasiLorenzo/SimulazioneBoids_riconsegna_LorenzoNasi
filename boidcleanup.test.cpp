#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boidcleanup.hpp"
#include "doctest.h"
using namespace boids;

TEST_CASE("Testing rules")
{
  paramlist params{};
  params.repulsione      = 0.7;
  params.steering        = 0.1;
  params.coesione        = 0.1;
  params.neigh_repulsion = 1000000;
  params.neigh_align     = 1000000;
  params.alpha           = M_PI;
  params.attraction      = 0;
  params.speedlimit      = 8000;
  params.speedminimum    = 0;
  params.size            = 10;
  params.deltaT          = 1 / 30.f;
  params.flocknumber     = 10;
  boidstate boid1;
  boid1.pos = {700., 200.};
  boid1.vel = {300., -10.};
  boidstate boid2;
  boid2.pos = {500., 300.};
  boid2.vel = {5., 0.};
  boidstate boid3;
  boid3.pos = {800., 250.};
  boid3.vel = {-88., 98.};
  boidstate boid4;
  boid4.pos = {1000., 150.};
  boid4.vel = {400., 77.};
  boidstate boid5;
  boid5.pos = {200., 500.};
  boid5.vel = {300., 300.};
  boidstate boid6;
  boid6.pos = {300., 450.};
  boid6.vel = {-100., -100.};
  boidstate boid7;
  boid7.pos = {450., 400.};
  boid7.vel = {200., 150.};
  boidstate boid8;
  boid8.pos = {50., 420.};
  boid8.vel = {400., 200.};
  boidstate boid9;
  boid9.pos = {120., 200.};
  boid9.vel = {50., 60.};
  boidstate boid10;
  boid10.pos = {910., 415.};
  boid10.vel = {-300., -200.};

  std::vector<boidstate> set{boid1, boid2, boid3, boid4, boid5,
                             boid6, boid7, boid8, boid9, boid10};
  ensemble<boidstate> flock{set};

  flock.update(params);
  REQUIRE(flock.size_() == 10);

  CHECK(flock.set_()[0].vel[0] == doctest::Approx(1636.744444));
  CHECK(flock.set_()[0].vel[1] == doctest::Approx(-887.7222222));

  CHECK(flock.set_()[1].vel[0] == doctest::Approx(-3.255555556));
  CHECK(flock.set_()[1].vel[1] == doctest::Approx(-189.9444444));

  CHECK(flock.set_()[2].vel[0] == doctest::Approx(1980.744444));
  CHECK(flock.set_()[2].vel[1] == doctest::Approx(-447.2777778));

  CHECK(flock.set_()[3].vel[0] == doctest::Approx(3792.3));
  CHECK(flock.set_()[3].vel[1] == doctest::Approx(-1154.833333));

  CHECK(flock.set_()[4].vel[0] == doctest::Approx(-1807.7));
  CHECK(flock.set_()[4].vel[1] == doctest::Approx(1454.5));

  CHECK(flock.set_()[5].vel[0] == doctest::Approx(-1474.366667));
  CHECK(flock.set_()[5].vel[1] == doctest::Approx(754.5));

  CHECK(flock.set_()[6].vel[0] == doctest::Approx(-174.3666667));
  CHECK(flock.set_()[6].vel[1] == doctest::Approx(632.2777778));

  CHECK(flock.set_()[7].vel[0] == doctest::Approx(-2752.144444));
  CHECK(flock.set_()[7].vel[1] == doctest::Approx(814.5));

  CHECK(flock.set_()[8].vel[0] == doctest::Approx(-2581.033333));
  CHECK(flock.set_()[8].vel[1] == doctest::Approx(-825.5));

  CHECK(flock.set_()[9].vel[0] == doctest::Approx(2550.077778));
  CHECK(flock.set_()[9].vel[1] == doctest::Approx(424.5));
}

TEST_CASE("Testing the speed limits")
{
  paramlist params{};
  params.repulsione      = 0.7;
  params.steering        = 0.1;
  params.coesione        = 0.1;
  params.neigh_repulsion = 1000000;
  params.neigh_align     = 1000000;
  params.alpha           = M_PI;
  params.attraction      = 0;
  params.speedlimit      = 3500.;
  params.speedminimum    = 3500.;

  boidstate boid1;
  boid1.pos = {700., 200.};
  boid1.vel = {300., -10.};
  boidstate boid2;
  boid2.pos = {500., 300.};
  boid2.vel = {5., 0.};
  boidstate boid3;
  boid3.pos = {800., 250.};
  boid3.vel = {-88., 98.};
  boidstate boid4;
  boid4.pos = {1000., 150.};
  boid4.vel = {400., 77.};
  std::vector<boidstate> boids{boid1, boid2, boid3, boid4};

  SUBCASE("Testing the velocity before the adjustment")
  {
    CHECK(boids::mod(boid1.vel) == doctest::Approx(300.167));
    CHECK(boids::mod(boid2.vel) == doctest::Approx(5.));
    CHECK(boids::mod(boid3.vel) == doctest::Approx(131.712));
    CHECK(boids::mod(boid4.vel) == doctest::Approx(407.344));
  }

  std::for_each(boids.begin(), boids.end(), [&params](boidstate& boid) {
    boids::functions<boidstate>::speedadjust(boid, params.speedlimit,
                                             params.speedminimum);
  });

  SUBCASE("Testing the velocity after the adjustment")
  {
    for (const auto& boid : boids) {
      CHECK(boids::mod(boid.vel)
            == doctest::Approx(params.speedlimit).epsilon(0.001));
    }
  }
}

TEST_CASE("Testing boid sight")
{
  paramlist params{};
  params.repulsione      = 0.7;
  params.steering        = 0.1;
  params.coesione        = 0.1;
  params.neigh_repulsion = 1000000;
  params.neigh_align     = 1000000;
  params.alpha           = M_PI / 4;
  params.attraction      = 0;
  params.speedlimit      = 3500.;
  params.speedminimum    = 3500.;

  boidstate boid1;
  boid1.pos = {350., 270.};
  boid1.vel = {30., 0.};
  boidstate boid2;
  boid2.pos = {400., 270.};
  boid2.vel = {-10., 0.};
  boidstate boid3;
  boid3.pos = {200., 480.};
  boid3.vel = {13., 45.};
  boidstate boid4;
  boid4.pos = {0., 0.};
  boid4.vel = {300., -150.};
  boidstate boid5;
  boid5.pos = {90., 270.};
  boid5.vel = {30., 160.};
  boidstate boid6;
  boid6.pos = {350., 270.};
  boid6.vel = {-15., 10.};

  SUBCASE("Testing if boid1 sees boid2")
  {
    std::vector<boidstate> pair1{boid1, boid2};

    auto result1 =
        boids::functions<boidstate>::template neighbors<Criterion::any>(
            pair1, boid1, 10000., params.alpha);

    CHECK(result1.size() == 1);
  }

  SUBCASE("Testing if boid1 sees boid3")
  {
    std::vector<boidstate> pair2{boid1, boid3};

    auto result2 =
        boids::functions<boidstate>::template neighbors<Criterion::any>(
            pair2, boid1, 10000., params.alpha);

    CHECK(result2.size() == 1);
  }

  SUBCASE("Testing if boid1 sees boid4")
  {
    std::vector<boidstate> pair3{boid1, boid4};

    auto result3 =
        boids::functions<boidstate>::template neighbors<Criterion::any>(
            pair3, boid1, 10000., params.alpha);

    CHECK(result3.size() == 0);
  }

  SUBCASE("Testing if boid1 sees boid5")
  {
    std::vector<boidstate> pair4{boid1, boid5};

    auto result4 =
        boids::functions<boidstate>::template neighbors<Criterion::any>(
            pair4, boid1, 10000., params.alpha);
    CHECK(result4.size() == 0);
  }

  SUBCASE("Testing if boid1 sees boid6")
  {
    std::vector<boidstate> pair5{boid1, boid6};

    auto result5 =
        boids::functions<boidstate>::template neighbors<Criterion::any>(
            pair5, boid1, 10000., params.alpha);

    CHECK(result5.size() == 1);
  }
}
// namespace boids
