#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"
using namespace boids;

TEST_CASE("Testing rules in 3 dimensions,just repeating the old test"){
  std::cout << "dim vale: " << params::dim << "\n";
  ParamList params{};
  params.rate = 1.;
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
  boid1.get_pos() = {0.,700., 200.};
  boid1.get_vel() = {0.,300., -10.};
  BoidState boid2;
  boid2.get_pos() = {0.,500., 300.};
  boid2.get_vel() = {0.,5., 0.};
  BoidState boid3;
  boid3.get_pos() = {0.,800., 250.};
  boid3.get_vel() = {0.,-88., 98.};
  BoidState boid4;
  boid4.get_pos() = {0.,1000., 150.};
  boid4.get_vel() = {0.,400., 77.};
  BoidState boid5;
  boid5.get_pos() = {0.,200., 500.};
  boid5.get_vel() = {0.,300., 300.};
  BoidState boid6;
  boid6.get_pos() = {0.,300., 450.};
  boid6.get_vel() = {0.,-100., -100.};
  BoidState boid7;
  boid7.get_pos() = {0.,450., 400.};
  boid7.get_vel() = {0.,200., 150.};
  BoidState boid8;
  boid8.get_pos() = {0.,50., 420.};
  boid8.get_vel() = {0.,400., 200.};
  BoidState boid9;
  boid9.get_pos() = {0.,120., 200.};
  boid9.get_vel() = {0.,50., 60.};
  BoidState boid10;
  boid10.get_pos() = {0.,910., 415.};
  boid10.get_vel() = {0.,-300., -200.};

  std::vector<BoidState> set{boid1, boid2, boid3, boid4, boid5,
                             boid6, boid7, boid8, boid9, boid10};

  Flock stormo{set, params};
  std::cout << "Initial velocities:\n";
  for (const auto& boid : stormo.set_()) {
    std::cout << "Boid velocity: (" << boid.cget_vel()[0] << ", "
              << boid.cget_vel()[1] << ")\n";
  }

  stormo.update(params);

  REQUIRE(stormo.size_() == 10);

  CHECK(stormo.set_()[0].cget_vel()[1] == doctest::Approx(1636.744444));
  CHECK(stormo.set_()[0].cget_vel()[2] == doctest::Approx(-887.7222222));

  CHECK(stormo.set_()[1].cget_vel()[1] == doctest::Approx(-3.255555556));
  CHECK(stormo.set_()[1].cget_vel()[2] == doctest::Approx(-189.9444444));

  CHECK(stormo.set_()[2].cget_vel()[1] == doctest::Approx(1980.744444));
  CHECK(stormo.set_()[2].cget_vel()[2] == doctest::Approx(-447.2777778));

  CHECK(stormo.set_()[3].cget_vel()[1] == doctest::Approx(3792.3));
  CHECK(stormo.set_()[3].cget_vel()[2] == doctest::Approx(-1154.833333));

  CHECK(stormo.set_()[4].cget_vel()[1] == doctest::Approx(-1807.7));
  CHECK(stormo.set_()[4].cget_vel()[2] == doctest::Approx(1454.5));

  CHECK(stormo.set_()[5].cget_vel()[1] == doctest::Approx(-1474.366667));
  CHECK(stormo.set_()[5].cget_vel()[2] == doctest::Approx(754.5));

  CHECK(stormo.set_()[6].cget_vel()[1] == doctest::Approx(-174.3666667));
  CHECK(stormo.set_()[6].cget_vel()[2] == doctest::Approx(632.2777778));

  CHECK(stormo.set_()[7].cget_vel()[1] == doctest::Approx(-2752.144444));
  CHECK(stormo.set_()[7].cget_vel()[2] == doctest::Approx(814.5));

  CHECK(stormo.set_()[8].cget_vel()[1] == doctest::Approx(-2581.033333));
  CHECK(stormo.set_()[8].cget_vel()[2] == doctest::Approx(-825.5));

  CHECK(stormo.set_()[9].cget_vel()[1] == doctest::Approx(2550.077778));
  CHECK(stormo.set_()[9].cget_vel()[2] == doctest::Approx(424.5));
  
  std::for_each(stormo.set_()[0].cget_vel().begin(),stormo.set_()[0].cget_vel().end(),[](auto& comp){
    std::cout<<"Print coordinate: "<<comp<<"\n";
  });

}

TEST_CASE("Testing multiple iterations of the rules, just repeating the old test")
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
  params.rate =1;
  BoidState boid1;
  boid1.get_pos() = {0.,700., 200.};
  boid1.get_vel() = {0.,300., -10.};
  BoidState boid2;
  boid2.get_pos() = {0.,500., 300.};
  boid2.get_vel() = {0.,5., 0.};
  BoidState boid3;
  boid3.get_pos() = {0.,800., 250.};
  boid3.get_vel() = {0.,-88., 98.};
  BoidState boid4;
  boid4.get_pos() = {0.,1000., 150.};
  boid4.get_vel() = {0.,400., 77.};

  std::vector<BoidState> set{boid1, boid2, boid3, boid4};

  Flock stormo2{set, params};

  stormo2.update(params);
  stormo2.update(params);
  stormo2.update(params);
  stormo2.update(params);
  for (auto& boid : stormo2.set_()) {
    std::cout << "Le velocità sono: " << boid.get_vel()[1] << ", "
              << boid.get_vel()[2] << "\n";
  }

  REQUIRE(stormo2.size_() == 4);

  CHECK(stormo2.set_()[0].cget_vel()[1] == doctest::Approx(-248.02519));
  CHECK(stormo2.set_()[1].cget_vel()[1] == doctest::Approx(-2717.9472));
  CHECK(stormo2.set_()[2].cget_vel()[1] == doctest::Approx(461.91329));
  CHECK(stormo2.set_()[3].cget_vel()[1] == doctest::Approx(3121.0591));
}