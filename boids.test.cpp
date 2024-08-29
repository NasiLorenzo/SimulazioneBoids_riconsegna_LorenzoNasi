#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"
using namespace boids;

TEST_CASE("Testing mod and normalize")
{
  DoubleVec vec1{1., 1.};
  DoubleVec vec2{0., 0.};
  DoubleVec vec3{2., 1.};

  CHECK(mod(vec1) == doctest::Approx(std::sqrt(2)));
  CHECK(mod(vec3) == doctest::Approx(std::sqrt(5)));
  CHECK(mod(vec2) == doctest::Approx(0));
  normalize(vec1);
  normalize(vec2);
  normalize(vec3);
  CHECK(mod(vec1) == doctest::Approx(1));
  CHECK(mod(vec3) == doctest::Approx(1));
  CHECK(mod(vec2) == doctest::Approx(0));
}

TEST_CASE("Testing rules")
{
  ParamList params{};
  params.rate             = 1.;
  params.repulsion_factor = 0.7;
  params.steering_factor  = 0.1;
  params.cohesion_factor  = 0.1;
  params.repulsion_range  = 10000.;
  params.view_range       = 10000.;
  params.alpha            = M_PI;
  params.border_repulsion = 0.;
  params.speedlimit       = 8000;
  params.speedminimum     = 0.;
  params.size             = 10;
  params.deltaT           = 1 / 30.f;
  params.flocksize        = 10;
  BoidState boid1;
  boid1.pos() = {700., 200.};
  boid1.vel() = {300., -10.};
  BoidState boid2;
  boid2.pos() = {500., 300.};
  boid2.vel() = {5., 0.};
  BoidState boid3;
  boid3.pos() = {800., 250.};
  boid3.vel() = {-88., 98.};
  BoidState boid4;
  boid4.pos() = {1000., 150.};
  boid4.vel() = {400., 77.};
  BoidState boid5;
  boid5.pos() = {200., 500.};
  boid5.vel() = {300., 300.};
  BoidState boid6;
  boid6.pos() = {300., 450.};
  boid6.vel() = {-100., -100.};
  BoidState boid7;
  boid7.pos() = {450., 400.};
  boid7.vel() = {200., 150.};
  BoidState boid8;
  boid8.pos() = {50., 420.};
  boid8.vel() = {400., 200.};
  BoidState boid9;
  boid9.pos() = {120., 200.};
  boid9.vel() = {50., 60.};
  BoidState boid10;
  boid10.pos() = {910., 415.};
  boid10.vel() = {-300., -200.};

  std::cout << "dim vale: " << params::dim << "\n";
  std::vector<BoidState> set{boid1, boid2, boid3, boid4, boid5,
                             boid6, boid7, boid8, boid9, boid10};

  Flock stormo{set, params};

  std::cout << "Initial velocities:\n";
  for (const auto& boid : stormo.set()) {
    std::cout << "Boid velocity: (" << boid.vel()[0] << ", " << boid.vel()[1]
              << ")\n";
  }

  stormo.update(params);

  REQUIRE(stormo.size() == 10);

  CHECK(stormo.set()[0].vel()[0] == doctest::Approx(1636.744444));
  CHECK(stormo.set()[0].vel()[1] == doctest::Approx(-887.7222222));

  CHECK(stormo.set()[1].vel()[0] == doctest::Approx(-3.255555556));
  CHECK(stormo.set()[1].vel()[1] == doctest::Approx(-189.9444444));

  CHECK(stormo.set()[2].vel()[0] == doctest::Approx(1980.744444));
  CHECK(stormo.set()[2].vel()[1] == doctest::Approx(-447.2777778));

  CHECK(stormo.set()[3].vel()[0] == doctest::Approx(3792.3));
  CHECK(stormo.set()[3].vel()[1] == doctest::Approx(-1154.833333));

  CHECK(stormo.set()[4].vel()[0] == doctest::Approx(-1807.7));
  CHECK(stormo.set()[4].vel()[1] == doctest::Approx(1454.5));

  CHECK(stormo.set()[5].vel()[0] == doctest::Approx(-1474.366667));
  CHECK(stormo.set()[5].vel()[1] == doctest::Approx(754.5));

  CHECK(stormo.set()[6].vel()[0] == doctest::Approx(-174.3666667));
  CHECK(stormo.set()[6].vel()[1] == doctest::Approx(632.2777778));

  CHECK(stormo.set()[7].vel()[0] == doctest::Approx(-2752.144444));
  CHECK(stormo.set()[7].vel()[1] == doctest::Approx(814.5));

  CHECK(stormo.set()[8].vel()[0] == doctest::Approx(-2581.033333));
  CHECK(stormo.set()[8].vel()[1] == doctest::Approx(-825.5));

  CHECK(stormo.set()[9].vel()[0] == doctest::Approx(2550.077778));
  CHECK(stormo.set()[9].vel()[1] == doctest::Approx(424.5));

  // checking the positions

  CHECK(stormo.set()[0].pos()[0] == doctest::Approx(754.558));
  CHECK(stormo.set()[0].pos()[1] == doctest::Approx(170.409));

  CHECK(stormo.set()[1].pos()[0] == doctest::Approx(499.891));
  CHECK(stormo.set()[1].pos()[1] == doctest::Approx(293.668));

  CHECK(stormo.set()[2].pos()[0] == doctest::Approx(866.024));
  CHECK(stormo.set()[2].pos()[1] == doctest::Approx(235.090));

  CHECK(stormo.set()[3].pos()[0] == doctest::Approx(1126.41));
  CHECK(stormo.set()[3].pos()[1] == doctest::Approx(111.505));

  CHECK(stormo.set()[4].pos()[0] == doctest::Approx(139.743));
  CHECK(stormo.set()[4].pos()[1] == doctest::Approx(548.483));

  CHECK(stormo.set()[5].pos()[0] == doctest::Approx(250.854));
  CHECK(stormo.set()[5].pos()[1] == doctest::Approx(475.15));

  CHECK(stormo.set()[6].pos()[0] == doctest::Approx(444.187));
  CHECK(stormo.set()[6].pos()[1] == doctest::Approx(421.075));

  CHECK(stormo.set()[7].pos()[0] == doctest::Approx(-41.738));
  CHECK(stormo.set()[7].pos()[1] == doctest::Approx(447.15));

  CHECK(stormo.set()[8].pos()[0] == doctest::Approx(33.9656));
  CHECK(stormo.set()[8].pos()[1] == doctest::Approx(172.483));

  CHECK(stormo.set()[9].pos()[0] == doctest::Approx(995.0026));
  CHECK(stormo.set()[9].pos()[1] == doctest::Approx(429.15));
}

TEST_CASE("Testing multiple iterations of the rules")
{
  ParamList params{};
  params.repulsion_factor = 0.7;
  params.steering_factor  = 0.1;
  params.cohesion_factor  = 0.1;
  params.repulsion_range  = 100000;
  params.view_range       = 100000;
  params.alpha            = M_PI;
  params.border_repulsion = 0;
  params.speedlimit       = 8000;
  params.speedminimum     = 0;
  params.size             = 4;
  params.deltaT           = 1 / 30.f;
  params.flocksize        = 4;
  params.pixel[0]         = 1000;
  params.pixel[1]         = 1000;
  params.rate             = 1;
  BoidState boid1;
  boid1.pos() = {700., 200.};
  boid1.vel() = {300., -10.};
  BoidState boid2;
  boid2.pos() = {500., 300.};
  boid2.vel() = {5., 0.};
  BoidState boid3;
  boid3.pos() = {800., 250.};
  boid3.vel() = {-88., 98.};
  BoidState boid4;
  boid4.pos() = {1000., 150.};
  boid4.vel() = {400., 77.};

  std::vector<BoidState> set{boid1, boid2, boid3, boid4};

  Flock stormo2{set, params};

  stormo2.update(params);
  stormo2.update(params);
  stormo2.update(params);
  stormo2.update(params);
  for (auto& boid : stormo2.set()) {
    std::cout << "Le velocità sono: " << boid.vel()[0] << ", " << boid.vel()[1]
              << "\n";
  }

  REQUIRE(stormo2.size() == 4);

  CHECK(stormo2.set()[0].vel()[0] == doctest::Approx(-248.02519));

  CHECK(stormo2.set()[1].vel()[0] == doctest::Approx(-2717.9472));

  CHECK(stormo2.set()[2].vel()[0] == doctest::Approx(461.91329));

  CHECK(stormo2.set()[3].vel()[0] == doctest::Approx(3121.0591));
}

TEST_CASE("Testing the speed limits")
{
  ParamList params{};
  params.repulsion_factor = 0.7;
  params.steering_factor  = 0.1;
  params.cohesion_factor  = 0.1;
  params.repulsion_range  = 1000000;
  params.view_range       = 1000000;
  params.alpha            = M_PI;
  params.border_repulsion = 0;
  params.speedlimit       = 350.;
  params.speedminimum     = 300.;
  params.size             = 10;
  params.flocksize        = 10;
  params.rate             = 1;
  BoidState boid1;
  boid1.pos() = {700., 200.};
  boid1.vel() = {300., -10.};
  BoidState boid2;
  boid2.pos() = {500., 300.};
  boid2.vel() = {5., 0.};
  BoidState boid3;
  boid3.pos() = {800., 250.};
  boid3.vel() = {-88., 98.};
  BoidState boid4;
  boid4.pos() = {1000., 150.};
  boid4.vel() = {400., 77.};
  std::vector<BoidState> boids{boid1, boid2, boid3, boid4};

  SUBCASE("Testing the vel()ocity before the adjustment")
  {
    CHECK(boids::mod(boid1.vel()) == doctest::Approx(300.167));
    CHECK(boids::mod(boid2.vel()) == doctest::Approx(5.));
    CHECK(boids::mod(boid3.vel()) == doctest::Approx(131.712));
    CHECK(boids::mod(boid4.vel()) == doctest::Approx(407.344));
  }

  std::for_each(boids.begin(), boids.end(), [&params](BoidState& boid) {
    speed_adjust(boid.boid(), params.speedlimit, params.speedminimum);
  });

  SUBCASE("Testing the vel()ocity after the adjustment")
  {
    CHECK(boids::mod(boids[0].vel()) == doctest::Approx(300.167));
    CHECK(boids::mod(boids[1].vel()) == doctest::Approx(params.speedminimum));
    CHECK(boids::mod(boids[2].vel()) == doctest::Approx(params.speedminimum));
    CHECK(boids::mod(boids[3].vel()) == doctest::Approx(params.speedlimit));
  }
}

TEST_CASE("Testing boid sight") // each boid contains itself in the vector of
                                // its neighbors
{
  ParamList params{};
  params.repulsion_factor = 0.7;
  params.steering_factor  = 0.1;
  params.cohesion_factor  = 0.1;
  params.repulsion_range  = 1000000;
  params.view_range       = 1000000;
  params.alpha            = M_PI / 4;
  params.border_repulsion = 0;
  params.speedlimit       = 3500.;
  params.speedminimum     = 3200.;
  params.flocksize        = 10;
  params.size             = 10;
  params.deltaT           = 1 / 30.f;
  params.flocksize        = 10;

  params.pixel[0] = 1000;
  params.pixel[1] = 1000;
  BoidState boid1;
  boid1.pos() = {350., 270.};
  boid1.vel() = {30., 0.};
  BoidState boid2;
  boid2.pos() = {400., 270.};
  boid2.vel() = {-10., 0.};
  BoidState boid3;
  boid3.pos() = {360., 300.};
  boid3.vel() = {13., 45.};
  BoidState boid4;
  boid4.pos() = {0., 0.};
  boid4.vel() = {300., -150.};
  BoidState boid5;
  boid5.pos() = {90., 270.};
  boid5.vel() = {30., 160.};
  BoidState boid6;
  boid6.pos() = {350., 270.};
  boid6.vel() = {-15., 10.};

  SUBCASE("Testing if boid1 sees boid2")
  {
    params.size      = 2;
    params.flocksize = 2;
    std::vector<BoidState> pair1{boid1, boid2};
    boids::Flock stormo_1{pair1, params};
    stormo_1.update(params);
    CHECK(boids::cos_angle_between(boid2.pos() - boid1.pos(), boid1.vel())
          == doctest::Approx(cos(0)).epsilon(0.001));
    CHECK(stormo_1.set()[0].neighbors().size() == 1);
  }

  SUBCASE("Testing if boid1 sees boid3")
  {
    std::vector<BoidState> pair2{boid1, boid3};
    Flock stormo_2{pair2, params};
    stormo_2.update(params);
    CHECK(boids::cos_angle_between(boid3.pos() - boid1.pos(), boid1.vel())
          == doctest::Approx(cos(1.2490)).epsilon(0.001));
    CHECK(stormo_2.set()[0].neighbors().size() == 0);
  }

  SUBCASE("Testing if boid1 sees boid4")
  {
    std::vector<BoidState> pair3{boid1, boid4};
    Flock stormo_3{pair3, params};
    stormo_3.update(params);
    CHECK(boids::cos_angle_between(boid4.pos() - boid1.pos(), boid1.vel())
          == doctest::Approx(cos(3.798)).epsilon(0.001));
    CHECK(stormo_3.set()[0].neighbors().size() == 0);
  }

  SUBCASE("Testing if boid1 sees boid5")
  {
    std::vector<BoidState> pair4{boid1, boid5};
    Flock stormo_4{pair4, params};
    stormo_4.update(params);
    CHECK(boids::cos_angle_between(boid5.pos() - boid1.pos(), boid1.vel())
          == doctest::Approx(cos(M_PI)).epsilon(0.001));
    CHECK(stormo_4.set()[0].neighbors().size() == 0);
  }

  SUBCASE("Testing if boid1 sees boid6")
  {
    std::vector<BoidState> pair5{boid1, boid6};
    Flock stormo_5{pair5, params};
    stormo_5.update(params);
    CHECK(stormo_5.set()[0].neighbors().size() == 0);
  }
}

TEST_CASE("Testing the limit distance")
{
  ParamList params{};
  params.repulsion_factor = 0.7;
  params.steering_factor  = 0.1;
  params.cohesion_factor  = 0.1;
  params.repulsion_range  = 100;
  params.view_range       = 800;
  params.alpha            = M_PI;
  params.border_repulsion = 0;
  params.speedlimit       = 3500.;
  params.speedminimum     = 3500.;

  BoidState boid1;
  boid1.pos() = {0., 50.};
  boid1.vel() = {30., 0.};
  BoidState boid2;
  boid2.pos() = {700., 500.};
  boid2.vel() = {-10., 0.};
  BoidState boid3;
  boid3.pos() = {2., 55.};
  boid3.vel() = {13., 45.};

  SUBCASE("Testing if boid1 sees boid2")
  {
    std::vector<BoidState> pair1{boid1, boid2};
    Flock stormo_1{pair1, params};
    stormo_1.update(params);
    CHECK(sqrt(distance_squared(boid1.pos(), boid2.pos()))
          == doctest::Approx(832.165));
    CHECK(stormo_1.set()[0].neighbors().size() == 0);
  }
  SUBCASE("Testing if boid1 sees boid3")
  {
    std::vector<BoidState> pair2{boid1, boid3};
    Flock stormo_2{pair2, params};
    stormo_2.update(params);
    CHECK(sqrt(distance_squared(boid1.pos(), boid3.pos()))
          == doctest::Approx(5.38516));
    CHECK(stormo_2.set()[0].neighbors().size() == 1);
  }
}

TEST_CASE("Testing GridID")
{
  double view_range = 100.;
  boid test_boid_1{DoubleVec{80., 120.}, DoubleVec{0., 0.}};
  update_id(test_boid_1, view_range);
  CHECK(test_boid_1.GridID_[0] == doctest::Approx(1));
  CHECK(test_boid_1.GridID_[1] == doctest::Approx(2));

  boid test_boid_2{DoubleVec{-80., -120.}, DoubleVec{0., 0.}};
  update_id(test_boid_2, view_range);

  CHECK(test_boid_2.GridID_[0] == doctest::Approx(0));
  CHECK(test_boid_2.GridID_[1] == doctest::Approx(-1));

  boid test_boid_3{DoubleVec{1000., 0.}, DoubleVec{0., 0.}};
  update_id(test_boid_3, view_range);
  CHECK(test_boid_3.GridID_[0] == doctest::Approx(11));
  CHECK(test_boid_3.GridID_[1] == doctest::Approx(1));
}

TEST_CASE("Testing hashing")
{
  GridID gridID_1{2, 1};
  GridID gridID_2{1, 2};
  GridID gridID_3{1, 1};
  GridID gridID_4{2, 2};
  GridID gridID_5{1, 1};
  GridID gridID_6{1, 1};
  gridID_hash hasher;
  CHECK(hasher(gridID_1) != hasher(gridID_2));
  CHECK(hasher(gridID_3) != hasher(gridID_4));
  CHECK(hasher(gridID_5) == hasher(gridID_6));
}

TEST_CASE("Testing boids in limit cases")
{
  SUBCASE("Testing if boids on the same spot see each others")
  {
    ParamList params{};
    params.repulsion_factor = 0.7;
    params.steering_factor  = 0.1;
    params.cohesion_factor  = 0.1;
    params.repulsion_range  = 100000;
    params.view_range       = 100000;
    params.alpha            = M_PI;
    params.border_repulsion = 0;
    params.speedlimit       = 8000;
    params.speedminimum     = 0;
    params.size             = 4;
    params.deltaT           = 1 / 30.f;
    params.flocksize        = 4;
    params.pixel[0]         = 1000;
    params.pixel[1]         = 1000;
    params.rate             = 1;
    BoidState boid1{{100., 100.}, {-100., 100.}};
    BoidState boid2{{100., 100.}, {100., 100}};
    Flock flock_1{std::vector{boid1, boid2}, params};
    flock_1.update(params);
    CHECK(flock_1.set()[0].close_neighbors().size() == 0);
    CHECK(flock_1.set()[0].neighbors().size() == 0);
    CHECK(flock_1.set()[1].close_neighbors().size() == 0);
    CHECK(flock_1.set()[1].neighbors().size() == 0);
  }
}
// namespace boids
