#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include "doctest.h"
using namespace boids;

TEST_CASE("Testing rules")
{
  params::rate=1;
  paramlist params{};
  params.repulsione      = 0.7;
  params.steering        = 0.1;
  params.coesione        = 0.1;
  params.neigh_repulsion = 100000;
  params.neigh_align     = 100000;
  params.alpha           = M_PI;
  params.attraction      = 0;
  params.speedlimit      = 8000;
  params.speedminimum    = 0;
  params.size            = 4;
  params.deltaT          = 1 / 30.f;
  params.flocksize       = 4;
  params.rows            = 1;
  params.columns         = 1;
  params.pixel[0] =
      static_cast<unsigned int>(params.columns * params.neigh_align);
  params.pixel[1] = static_cast<unsigned int>(params.rows * params.neigh_align);
  params.columns *= static_cast<int>(params::rate);
  params.rows *= static_cast<int>(params::rate);
  boidstate boid1;
  boid1.get_pos() = {700., 200.};
  boid1.get_vel() = {300., -10.};
  boidstate boid2;
  boid2.get_pos() = {500., 300.};
  boid2.get_vel() = {5., 0.};
  boidstate boid3;
  boid3.get_pos() = {800., 250.};
  boid3.get_vel() = {-88., 98.};
  boidstate boid4;
  boid4.get_pos() = {1000., 150.};
  boid4.get_vel() = {400., 77.};
  boidstate boid5;
  boid5.get_pos() = {200., 500.};
  boid5.get_vel() = {300., 300.};
  boidstate boid6;
  boid6.get_pos() = {300., 450.};
  boid6.get_vel() = {-100., -100.};
  boidstate boid7;
  boid7.get_pos() = {450., 400.};
  boid7.get_vel() = {200., 150.};
  boidstate boid8;
  boid8.get_pos() = {50., 420.};
  boid8.get_vel() = {400., 200.};
  boidstate boid9;
  boid9.get_pos() = {120., 200.};
  boid9.get_vel() = {50., 60.};
  boidstate boid10;
  boid10.get_pos() = {910., 415.};
  boid10.get_vel() = {-300., -200.};

  std::vector<boidstate> set{boid1, boid2, boid3, boid4, boid5,
                             boid6, boid7, boid8, boid9, boid10};

  flock stormo{set, params};

  stormo.update(params);
  
  

  REQUIRE(stormo.size_() == 10);

  CHECK(stormo.set_()[0].get_vel()[0] == doctest::Approx(1636.744444));
  CHECK(stormo.set_()[0].get_vel()[1] == doctest::Approx(-887.7222222));

  CHECK(stormo.set_()[1].get_vel()[0] == doctest::Approx(-3.255555556));
  CHECK(stormo.set_()[1].get_vel()[1] == doctest::Approx(-189.9444444));

  CHECK(stormo.set_()[2].get_vel()[0] == doctest::Approx(1980.744444));
  CHECK(stormo.set_()[2].get_vel()[1] == doctest::Approx(-447.2777778));

  CHECK(stormo.set_()[3].get_vel()[0] == doctest::Approx(3792.3));
  CHECK(stormo.set_()[3].get_vel()[1] == doctest::Approx(-1154.833333));

   CHECK(stormo.set_()[4].get_vel()[0] == doctest::Approx(-1807.7));
   CHECK(stormo.set_()[4].get_vel()[1] == doctest::Approx(1454.5));

   CHECK(stormo.set_()[5].get_vel()[0] == doctest::Approx(-1474.366667));
   CHECK(stormo.set_()[5].get_vel()[1] == doctest::Approx(754.5));

   CHECK(stormo.set_()[6].get_vel()[0] == doctest::Approx(-174.3666667));
   CHECK(stormo.set_()[6].get_vel()[1] == doctest::Approx(632.2777778));

   CHECK(stormo.set_()[7].get_vel()[0] == doctest::Approx(-2752.144444));
   CHECK(stormo.set_()[7].get_vel()[1] == doctest::Approx(814.5));

   CHECK(stormo.set_()[8].get_vel()[0] == doctest::Approx(-2581.033333));
   CHECK(stormo.set_()[8].get_vel()[1] == doctest::Approx(-825.5));

   CHECK(stormo.set_()[9].get_vel()[0] == doctest::Approx(2550.077778));
   CHECK(stormo.set_()[9].get_vel()[1] == doctest::Approx(424.5));
}

TEST_CASE("Testing multiple iterations of the rules")
{
  paramlist params{};
  params.repulsione      = 0.7;
  params.steering        = 0.1;
  params.coesione        = 0.1;
  params.neigh_repulsion = 100000;
  params.neigh_align     = 100000;
  params.alpha           = M_PI;
  params.attraction      = 0;
  params.speedlimit      = 8000;
  params.speedminimum    = 0;
  params.size            = 4;
  params.deltaT          = 1 / 30.f;
  params.flocksize       = 4;
  params.rows            = 1;
  params.columns         = 1;
  params.pixel[0] =
      static_cast<unsigned int>(params.columns * params.neigh_align);
  params.pixel[1] = static_cast<unsigned int>(params.rows * params.neigh_align);
  params.columns *= static_cast<int>(params::rate);
  params.rows *= static_cast<int>(params::rate);
  boidstate boid1;
  boid1.get_pos() = {700., 200.};
  boid1.get_vel() = {300., -10.};
  boidstate boid2;
  boid2.get_pos() = {500., 300.};
  boid2.get_vel() = {5., 0.};
  boidstate boid3;
  boid3.get_pos() = {800., 250.};
  boid3.get_vel() = {-88., 98.};
  boidstate boid4;
  boid4.get_pos() = {1000., 150.};
  boid4.get_vel() = {400., 77.};

  std::vector<boidstate> set{boid1, boid2, boid3, boid4};

  flock stormo2{set, params};

  stormo2.update(params);
  stormo2.update(params);
  stormo2.update(params);
  stormo2.update(params);
  for (auto& boid : stormo2.set_()) {
    std::cout << "Le velocità sono: " << boid.get_vel()[0] << ", "
              << boid.get_vel()[1] << "\n";
    }
  
  

  REQUIRE(stormo2.size_() == 4);

  CHECK(stormo2.set_()[0].get_vel()[0] == doctest::Approx(-248.02519));

  CHECK(stormo2.set_()[1].get_vel()[0] == doctest::Approx(-2717.9472));

  CHECK(stormo2.set_()[2].get_vel()[0] == doctest::Approx(461.91329));

  CHECK(stormo2.set_()[3].get_vel()[0] == doctest::Approx(3121.0591));

}

/*TEST_CASE("Testing multiple iterations of the rules")
{
  paramlist params{};
  params.repulsione      = 0.7;
  params.steering        = 0.1;
  params.coesione        = 0.1;
  params.neigh_repulsion = 100000;
  params.neigh_align     = 100000;
  params.alpha           = M_PI;
  params.attraction      = 0;
  params.speedlimit      = 8000;
  params.speedminimum    = 0;
  params.size            = 4;
  params.deltaT          = 1 / 30.f;
  params.flocksize       = 4;
  params.rows            = 1;
  params.columns         = 1;
  params.pixel[0] =
      static_cast<unsigned int>(params.columns * params.neigh_align);
  params.pixel[1] = static_cast<unsigned int>(params.rows * params.neigh_align);
  params.columns *= static_cast<int>(params::rate);
  params.rows *= static_cast<int>(params::rate);
  boidstate boid1;
  boid1.get_pos() = {700., 200.};
  boid1.get_vel() = {300., -10.};
  boidstate boid2;
  boid2.get_pos() = {500., 300.};
  boid2.get_vel() = {5., 0.};
  boidstate boid3;
  boid3.get_pos() = {800., 250.};
  boid3.get_vel() = {-88., 98.};
  boidstate boid4;
  boid4.get_pos() = {1000., 150.};
  boid4.get_vel() = {400., 77.};

  std::vector<boidstate> set{boid1, boid2, boid3, boid4};

  flock stormo{set, params};

  stormo.update(params);
  for(auto& boid : stormo.set_()){
    std::cout<<"Le posizioni valgolo: "<<boid.cget_pos()[0]<<" e "<<boid.cget_pos()[1]<<"\n";
  }
  stormo.update(params);
  for (auto& boid : stormo.set_()) {
    std::cout << "Le velocità sono: " << boid.get_vel()[0] << ", "
              << boid.get_vel()[1] << "\n";
    //std::cout << "Le posizioni sono: " << boid.cget_pos()[0] << ", "
             // << boid.cget_pos()[1] << "\n";
    /*std::cout << "Il numero di vicini e molto vicini è: "
              << boid.get_neighbors().size() << ", "
              << boid.get_neighbors().size() << "\n"
              << "Il GridID vale: "<<boid.set_GridID().columns<<", "<<boid.set_GridID().rows<<"\n";

  }

  REQUIRE(stormo.size_() == 4);

  CHECK(stormo.set_()[0].get_vel()[0] == doctest::Approx(1636.744444));
  CHECK(stormo.set_()[0].get_vel()[1] == doctest::Approx(-887.7222222));

  CHECK(stormo.set_()[1].get_vel()[0] == doctest::Approx(-3.255555556));
  CHECK(stormo.set_()[1].get_vel()[1] == doctest::Approx(-189.9444444));

  CHECK(stormo.set_()[2].get_vel()[0] == doctest::Approx(1980.744444));
  CHECK(stormo.set_()[2].get_vel()[1] == doctest::Approx(-447.2777778));

  CHECK(stormo.set_()[3].get_vel()[0] == doctest::Approx(3792.3));
  CHECK(stormo.set_()[3].get_vel()[1] == doctest::Approx(-1154.833333));
}*/

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
  params.speedlimit      = 350.;
  params.speedminimum    = 300.;
  params.size            = 10;
  params.flocksize       = 10;
  boidstate boid1;
  boid1.get_pos() = {700., 200.};
  boid1.get_vel() = {300., -10.};
  boidstate boid2;
  boid2.get_pos() = {500., 300.};
  boid2.get_vel() = {5., 0.};
  boidstate boid3;
  boid3.get_pos() = {800., 250.};
  boid3.get_vel() = {-88., 98.};
  boidstate boid4;
  boid4.get_pos() = {1000., 150.};
  boid4.get_vel() = {400., 77.};
  std::vector<boidstate> boids{boid1, boid2, boid3, boid4};

  SUBCASE("Testing the get_vel()ocity before the adjustment")
  {
    CHECK(boids::mod(boid1.get_vel()) == doctest::Approx(300.167));
    CHECK(boids::mod(boid2.get_vel()) == doctest::Approx(5.));
    CHECK(boids::mod(boid3.get_vel()) == doctest::Approx(131.712));
    CHECK(boids::mod(boid4.get_vel()) == doctest::Approx(407.344));
  }

  std::for_each(boids.begin(), boids.end(), [&params](boidstate& boid) {
    boid.speedadjust(params.speedlimit, params.speedminimum);
  });

  SUBCASE("Testing the get_vel()ocity after the adjustment")
  {
    CHECK(boids::mod(boids[0].get_vel()) == doctest::Approx(300.167));
    CHECK(boids::mod(boids[1].get_vel())
          == doctest::Approx(params.speedminimum));
    CHECK(boids::mod(boids[2].get_vel())
          == doctest::Approx(params.speedminimum));
    CHECK(boids::mod(boids[3].get_vel()) == doctest::Approx(params.speedlimit));
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
  params.speedminimum    = 3200.;
  params.flocksize       = 10;
  params.size            = 10;
  boidstate boid1;
  boid1.get_pos() = {350., 270.};
  boid1.get_vel() = {30., 0.};
  boidstate boid2;
  boid2.get_pos() = {400., 270.};
  boid2.get_vel() = {-10., 0.};
  boidstate boid3;
  boid3.get_pos() = {360., 300.};
  boid3.get_vel() = {13., 45.};
  boidstate boid4;
  boid4.get_pos() = {0., 0.};
  boid4.get_vel() = {300., -150.};
  boidstate boid5;
  boid5.get_pos() = {90., 270.};
  boid5.get_vel() = {30., 160.};
  boidstate boid6;
  boid6.get_pos() = {350., 270.};
  boid6.get_vel() = {-15., 10.};
  /*
    SUBCASE("Testing if boid1 sees boid2")
    {
      std::vector<boidstate> pair1{boid1, boid2};

      auto result1 =
          boids::functions<boidstate>::template neighbors<Criterion::any>(
              pair1, boid1, 10000., params.alpha);
      CHECK(boids::cosangleij(boid2.get_pos() - boid1.get_pos(),
    boid1.get_vel())
            == doctest::Approx(cos(0)).epsilon(0.001));
      CHECK(result1.size() == 1);
    }

    SUBCASE("Testing if boid1 sees boid3")
    {
      std::vector<boidstate> pair2{boid1, boid3};

      auto result2 =
          boids::functions<boidstate>::template neighbors<Criterion::any>(
              pair2, boid1, 10000., params.alpha);

      CHECK(boids::cosangleij(boid3.get_pos() - boid1.get_pos(),
    boid1.get_vel())
            == doctest::Approx(cos(1.2490)).epsilon(0.001));
      CHECK(result2.size() == 0);
    }

    SUBCASE("Testing if boid1 sees boid4")
    {
      std::vector<boidstate> pair3{boid1, boid4};

      auto result3 =
          boids::functions<boidstate>::template neighbors<Criterion::any>(
              pair3, boid1, 10000., params.alpha);
      CHECK(boids::cosangleij(boid4.get_pos() - boid1.get_pos(),
    boid1.get_vel())
            == doctest::Approx(cos(3.798)).epsilon(0.001));
      CHECK(result3.size() == 0);
    }

    SUBCASE("Testing if boid1 sees boid5")
    {
      std::vector<boidstate> pair4{boid1, boid5};

      auto result4 =
          boids::functions<boidstate>::template neighbors<Criterion::any>(
              pair4, boid1, 10000., params.alpha);
      CHECK(boids::cosangleij(boid5.get_pos() - boid1.get_pos(),
    boid1.get_vel())
            == doctest::Approx(cos(M_PI)).epsilon(0.001));
      CHECK(result4.size() == 0);
    }

    SUBCASE("Testing if boid1 sees boid6")
    {
      std::vector<boidstate> pair5{boid1, boid6};

      auto result5 =
          boids::functions<boidstate>::template neighbors<Criterion::any>(
              pair5, boid1, 10000., params.alpha);
      CHECK(result5.size() == 0);
    }*/
}

TEST_CASE("Testing the limit distance")
{
  paramlist params{};
  params.repulsione      = 0.7;
  params.steering        = 0.1;
  params.coesione        = 0.1;
  params.neigh_repulsion = 100;
  params.neigh_align     = 100;
  params.alpha           = M_PI;
  params.attraction      = 0;
  params.speedlimit      = 3500.;
  params.speedminimum    = 3500.;

  boidstate boid1;
  boid1.get_pos() = {0., 50.};
  boid1.get_vel() = {30., 0.};
  boidstate boid2;
  boid2.get_pos() = {700., 500.};
  boid2.get_vel() = {-10., 0.};
  boidstate boid3;
  boid3.get_pos() = {2., 55.};
  boid3.get_vel() = {13., 45.};

  /* SUBCASE("Testing if boid1 sees boid2")
   {
     std::vector<boidstate> pair1{boid1, boid2};
     auto result1 =
         boids::functions<boidstate>::template neighbors<Criterion::any>(
             pair1, boid1, params.neigh_align, params.alpha);
     CHECK(sqrt(distance(boid1.get_pos(), boid2.get_pos()))
           == doctest::Approx(832.165));
     CHECK(result1.size() == 0);
   }
   SUBCASE("Testing if boid1 sees boid3")
   {
     std::vector<boidstate> pair2{boid1, boid3};
     auto result2 =
         boids::functions<boidstate>::template neighbors<Criterion::any>(
             pair2, boid1, params.neigh_align, params.alpha);
     CHECK(sqrt(distance(boid1.get_pos(), boid3.get_pos()))
           == doctest::Approx(5.38516));
     CHECK(result2.size() == 1);
   }*/
}
// namespace boids
