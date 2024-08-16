#ifndef BOIDS_TPP
#define BOIDS_TPP
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>
namespace boids {


template<class boidtype>
void ensemble<boidtype>::update(paramlist const& params)
{
  auto jt = newset.begin();
  std::for_each(set.begin(), set.end(), [&](auto& it) mutable {
    std::vector<boidtype const*> neighbor;
    std::vector<boidtype const*> close_neighbor;

    if (params.flocksize < params.size) {
      neighbor =
          boids::functions<boidtype>::template neighbors<Criterion::similar>(
              set, *jt, params.neigh_align, params.alpha);
      close_neighbor = functions<boidtype>::template neighbors<Criterion::any>(
          set, *jt, params.neigh_repulsion, params.alpha);
    } else {
      neighbor = boids::functions<boidtype>::template neighbors<Criterion::any>(
          set, *jt, params.neigh_align, params.alpha);
      close_neighbor =
          functions<boidtype>::neighbors(neighbor, *jt, params.neigh_repulsion);
    }

    functions<boidtype>::regola1(close_neighbor, *jt, params.repulsione);
    functions<boidtype>::regola2_3(neighbor, it, *jt, params.steering,
                                   params.coesione);
    functions<boidtype>::speedadjust(*jt, params.speedlimit,
                                     params.speedminimum);
    bordercheck_posupdate(*jt, params.pixel, params.bordersize,
                          params.attraction, params.deltaT);
    ++jt;
  });

  // Update set to newset
  set = newset;
}

} // namespace boids
#endif