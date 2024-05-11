#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "boidcleanup.hpp"
namespace boids{

TEST_CASE("Testing rules")
{
  paramms::repulsione=0.7;
  paramms::steering=0.1;
  paramms::coesione=0.1;
  paramms::neigh2=1000000;
  paramms::neigh_align=1000000;
  boidstate boid1;
  boid1.pos={500.,0.};
  boid1.vel={0.,0.};
  boidstate boid2;
  boid2.pos={500.,0.};
  boid2.vel={0.,0.};
  boidstate boid3;
  boid3.pos={1000.,0.};
  boid3.vel={0.,0.};
  boidstate boid4;
  boid4.pos={1000.,0.};
  boid4.vel={0.,0.};
  stormo set{boid1,boid2,boid3,boid4};
  ensemble flock{set};
  flock.update();
  REQUIRE(flock.size_() == 4);

  CHECK(flock.set_()[0].vel[0] == doctest::Approx(471.33));
  CHECK(flock.set_()[1].vel[0] == doctest::Approx(471.33));
  CHECK(flock.set_()[2].vel[0] == doctest::Approx(471.33));
  CHECK(flock.set_()[3].vel[0] == doctest::Approx(471.33));



}

}
