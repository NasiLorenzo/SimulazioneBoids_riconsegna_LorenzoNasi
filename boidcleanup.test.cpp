#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "boidcleanup.hpp"
namespace boids{
/*struct paramms{
  double repulsione{0.6};
  double steering{0.07};
  double coesione{0.01};
  double neigh_align{70};//raggio visivo
  double neigh2{15};//raggio di repulsione
};  
paramms params;
*/
TEST_CASE("Testing sum_norms_index")
{
  boids::paramms parametri;
  parametri.repulsione=0.7;
  parametri.steering=0.1;
  parametri.coesione=0.1;
  parametri.neigh2=100000;
  parametri.neigh_align=100000;
  boidstate boid1;
  boid1.pos={500.,0.};
  boid1.vel={0,0};
  boidstate boid2;
  boid2.pos={500.,0.};
  boid2.vel={0,0};
  boidstate boid3;
  boid3.pos={1000.,0.};
  boid3.vel={0,0};
  boidstate boid4;
  boid4.pos={1000.,0.};
  boid4.vel={0,0};
  stormo set{boid1,boid2,boid3,boid4};
  ensemble flock{set};
  flock.update();
  REQUIRE(flock.size_() == 4);

  CHECK(flock.set_()[0].pos[0] == doctest::Approx(471.33));
}

}
