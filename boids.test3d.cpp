#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"
using namespace boids;

TEST_CASE("Testing rules in 3 dimensions,just repeating the old test")
{
  std::cout << "dim vale: " << params::dim << "\n";
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
  params.speedminimum     = 0.1;
  params.size             = 10;
  params.deltaT           = 1 / 30.f;
  params.flocksize        = 10;
  BoidState boid1;
  boid1.pos() = {0., 700., 200.};
  boid1.vel() = {0., 300., -10.};
  BoidState boid2;
  boid2.pos() = {0., 500., 300.};
  boid2.vel() = {0., 5., 0.};
  BoidState boid3;
  boid3.pos() = {0., 800., 250.};
  boid3.vel() = {0., -88., 98.};
  BoidState boid4;
  boid4.pos() = {0., 1000., 150.};
  boid4.vel() = {0., 400., 77.};
  BoidState boid5;
  boid5.pos() = {0., 200., 500.};
  boid5.vel() = {0., 300., 300.};
  BoidState boid6;
  boid6.pos() = {0., 300., 450.};
  boid6.vel() = {0., -100., -100.};
  BoidState boid7;
  boid7.pos() = {0., 450., 400.};
  boid7.vel() = {0., 200., 150.};
  BoidState boid8;
  boid8.pos() = {0., 50., 420.};
  boid8.vel() = {0., 400., 200.};
  BoidState boid9;
  boid9.pos() = {0., 120., 200.};
  boid9.vel() = {0., 50., 60.};
  BoidState boid10;
  boid10.pos() = {0., 910., 415.};
  boid10.vel() = {0., -300., -200.};

  std::vector<BoidState> set{boid1, boid2, boid3, boid4, boid5,
                             boid6, boid7, boid8, boid9, boid10};

  Flock stormo{set, params};
  
  stormo.update(params);

  REQUIRE(stormo.size() == 10);

  CHECK(stormo.set()[0].vel()[1] == doctest::Approx(1636.744444));
  CHECK(stormo.set()[0].vel()[2] == doctest::Approx(-887.7222222));

  CHECK(stormo.set()[1].vel()[1] == doctest::Approx(-3.255555556));
  CHECK(stormo.set()[1].vel()[2] == doctest::Approx(-189.9444444));

  CHECK(stormo.set()[2].vel()[1] == doctest::Approx(1980.744444));
  CHECK(stormo.set()[2].vel()[2] == doctest::Approx(-447.2777778));

  CHECK(stormo.set()[3].vel()[1] == doctest::Approx(3792.3));
  CHECK(stormo.set()[3].vel()[2] == doctest::Approx(-1154.833333));

  CHECK(stormo.set()[4].vel()[1] == doctest::Approx(-1807.7));
  CHECK(stormo.set()[4].vel()[2] == doctest::Approx(1454.5));

  CHECK(stormo.set()[5].vel()[1] == doctest::Approx(-1474.366667));
  CHECK(stormo.set()[5].vel()[2] == doctest::Approx(754.5));

  CHECK(stormo.set()[6].vel()[1] == doctest::Approx(-174.3666667));
  CHECK(stormo.set()[6].vel()[2] == doctest::Approx(632.2777778));

  CHECK(stormo.set()[7].vel()[1] == doctest::Approx(-2752.144444));
  CHECK(stormo.set()[7].vel()[2] == doctest::Approx(814.5));

  CHECK(stormo.set()[8].vel()[1] == doctest::Approx(-2581.033333));
  CHECK(stormo.set()[8].vel()[2] == doctest::Approx(-825.5));

  CHECK(stormo.set()[9].vel()[1] == doctest::Approx(2550.077778));
  CHECK(stormo.set()[9].vel()[2] == doctest::Approx(424.5));

  // testing the positions;

  CHECK(stormo.set()[0].pos()[1] == doctest::Approx(754.558));
  CHECK(stormo.set()[0].pos()[2] == doctest::Approx(170.409));

  CHECK(stormo.set()[1].pos()[1] == doctest::Approx(499.891));
  CHECK(stormo.set()[1].pos()[2] == doctest::Approx(293.668));

  CHECK(stormo.set()[2].pos()[1] == doctest::Approx(866.024));
  CHECK(stormo.set()[2].pos()[2] == doctest::Approx(235.090));

  CHECK(stormo.set()[3].pos()[1] == doctest::Approx(1126.41));
  CHECK(stormo.set()[3].pos()[2] == doctest::Approx(111.505));

  CHECK(stormo.set()[4].pos()[1] == doctest::Approx(139.743));
  CHECK(stormo.set()[4].pos()[2] == doctest::Approx(548.483));

  CHECK(stormo.set()[5].pos()[1] == doctest::Approx(250.854));
  CHECK(stormo.set()[5].pos()[2] == doctest::Approx(475.15));

  CHECK(stormo.set()[6].pos()[1] == doctest::Approx(444.187));
  CHECK(stormo.set()[6].pos()[2] == doctest::Approx(421.075));

  CHECK(stormo.set()[7].pos()[1] == doctest::Approx(-41.738));
  CHECK(stormo.set()[7].pos()[2] == doctest::Approx(447.15));

  CHECK(stormo.set()[8].pos()[1] == doctest::Approx(33.9656));
  CHECK(stormo.set()[8].pos()[2] == doctest::Approx(172.483));

  CHECK(stormo.set()[9].pos()[1] == doctest::Approx(995.0026));
  CHECK(stormo.set()[9].pos()[2] == doctest::Approx(429.15));

}

TEST_CASE(
    "Testing multiple iterations of the rules, just repeating the old test")
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
  params.pixel[2]         = 1000;
  params.rate             = 1;
  BoidState boid1;
  boid1.pos() = {0., 700., 200.};
  boid1.vel() = {0., 300., -10.};
  BoidState boid2;
  boid2.pos() = {0., 500., 300.};
  boid2.vel() = {0., 5., 0.};
  BoidState boid3;
  boid3.pos() = {0., 800., 250.};
  boid3.vel() = {0., -88., 98.};
  BoidState boid4;
  boid4.pos() = {0., 1000., 150.};
  boid4.vel() = {0., 400., 77.};

  std::vector<BoidState> set{boid1, boid2, boid3, boid4};

  Flock stormo2{set, params};

  stormo2.update(params);
  stormo2.update(params);
  stormo2.update(params);
  stormo2.update(params);
  for (auto& boid : stormo2.set()) {
    std::cout << "Le velocità sono: " << boid.vel()[1] << ", " << boid.vel()[2]
              << "\n";
  }

  REQUIRE(stormo2.size() == 4);

  CHECK(stormo2.set()[0].vel()[1] == doctest::Approx(-248.02519));
  CHECK(stormo2.set()[1].vel()[1] == doctest::Approx(-2717.9472));
  CHECK(stormo2.set()[2].vel()[1] == doctest::Approx(461.91329));
  CHECK(stormo2.set()[3].vel()[1] == doctest::Approx(3121.0591));
}

TEST_CASE("Testing GridID"){
  double view_range=100.;
  Boid test_boid_1{DoubleVec{80.,120.,210.},DoubleVec{0.,0.,0.}};
  update_id(test_boid_1,view_range);
  CHECK(test_boid_1.GridID()[0]==doctest::Approx(1));
  CHECK(test_boid_1.GridID()[1]==doctest::Approx(2));
  CHECK(test_boid_1.GridID()[2]==doctest::Approx(3));

  Boid test_boid_2{DoubleVec{-80.,-120.,-210.},DoubleVec{0.,0.,0.}};
  update_id(test_boid_2,view_range);

  CHECK(test_boid_2.GridID()[0]==doctest::Approx(0));
  CHECK(test_boid_2.GridID()[1]==doctest::Approx(-1));
  CHECK(test_boid_2.GridID()[2]==doctest::Approx(-2));

  Boid test_boid_3{DoubleVec{1000.,0.,-50.},DoubleVec{0.,0.,0.}};
  update_id(test_boid_3,view_range);
  CHECK(test_boid_3.GridID()[0]==doctest::Approx(11));
  CHECK(test_boid_3.GridID()[1]==doctest::Approx(1));
  CHECK(test_boid_3.GridID()[2]==doctest::Approx(0));
}

TEST_CASE("Testing hashing"){
  GridID gridID_1{1,1,2};
  GridID gridID_2{2,1,1};
  GridID gridID_3{1,1,2};
  GridID gridID_4{1,1,1};
  GridID gridID_5{1,1,1};
  GridID gridID_6{1,1,1};
  gridID_hash hasher;
  CHECK(hasher(gridID_1)!=hasher(gridID_2));
  CHECK(hasher(gridID_3)!=hasher(gridID_4));
  CHECK(hasher(gridID_5)==hasher(gridID_6));
}

TEST_CASE("Testing distances, view angle, grid")
{
  ParamList params{};
  params.repulsion_factor = 0.7;
  params.steering_factor  = 0.1;
  params.cohesion_factor  = 0.1;
  params.repulsion_range  = 100;
  params.view_range       = 100;
  params.alpha            = M_PI;
  params.border_repulsion = 0;
  params.speedlimit       = 8000;
  params.speedminimum     = 0;
  params.size             = 4;
  params.deltaT           = 1 / 30.f;
  params.flocksize        = 4;
  params.pixel[0]         = 1000;
  params.pixel[1]         = 1000;
  params.pixel[2]         = 1000;
  BoidState boid1;
  boid1.pos() = {50., 50., 50.};
  boid1.vel() = {10., 0., 0.};
  BoidState boid2;
  boid2.pos() = {-2., -2., -2.};
  boid2.vel() = {0., 5., 0.};
  BoidState boid3;
  boid3.pos() = {50., 100., 50.};
  boid3.vel() = {0., -10., 20.};
  BoidState boid4;
  boid4.pos() = {100., 150., 50.};
  boid4.vel() = {0., 10., -10.};
  Flock flock{std::vector<BoidState>{boid1, boid2, boid3, boid4}, params};
  SUBCASE("Testing distances")
  {
    update_neighbors( // updating only the first boid's neighbors
        flock.set()[0].boid(), flock.set()[0].neighbors(), flock.hashMap(),
        params.view_range, params.alpha, Criterion::similar);
    CHECK(flock.set()[0].neighbors().size() == doctest::Approx(2));
    CHECK(distance_squared(flock.set()[0].pos(), flock.set()[1].pos())
          == doctest::Approx(8112));
    CHECK(distance_squared(flock.set()[0].pos(), flock.set()[2].pos())
          == doctest::Approx(2500));
    CHECK(distance_squared(flock.set()[0].pos(), flock.set()[3].pos())
          == doctest::Approx(12500));
  }

  SUBCASE("Testing the grid range")
  {
    auto grid_range =
        update_neighbors_testing( // creating the vector of checked locations
            flock.set()[0].boid(), flock.set()[0].neighbors(), flock.hashMap(),
            params.view_range, params.alpha, Criterion::any);
    CHECK(flock.set()[0].GridID()[0] == doctest::Approx(1));
    CHECK(grid_range.front()[0] == doctest::Approx(0));
    CHECK(grid_range.front()[1] == doctest::Approx(0));
    CHECK(grid_range.front()[2] == doctest::Approx(0));
    CHECK(grid_range.back()[0] == doctest::Approx(2));
    CHECK(grid_range.back()[1] == doctest::Approx(2));
    CHECK(grid_range.back()[2] == doctest::Approx(2));
  }
  SUBCASE("Testing the view angle")
  {
    params.alpha = 0.5 * M_PI; // redefine the angle and test again neighbors;
    update_neighbors(flock.set()[0].boid(), flock.set()[0].neighbors(),
                     flock.hashMap(), params.view_range, params.alpha,
                     Criterion::similar);
    CHECK(cos_angle_between(flock.set()[1].pos() - flock.set()[0].pos(),
                            flock.set()[0].vel())
          == doctest::Approx(-1 / sqrt(3)).epsilon(0.001));
    CHECK(cos_angle_between(flock.set()[2].pos() - flock.set()[0].pos(),
                            flock.set()[0].vel())
          == doctest::Approx(0).epsilon(
              0.001)); // this boid should get seen, but it doesn't since it's
                       // right on the edge and std::cos(pi/)!=0.
    CHECK(cos_angle_between(flock.set()[3].pos() - flock.set()[0].pos(),
                            flock.set()[0].vel())
          == doctest::Approx(0.4472).epsilon(0.001));
    CHECK(std::cos(0.5 * M_PI) == doctest::Approx(6.12323 * std::pow(10, -17)));
    CHECK(flock.set()[0].neighbors().size() == doctest::Approx(0));
  }
}