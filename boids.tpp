#ifndef BOIDS_TPP
#define BOIDS_TPP
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>
namespace boids {
template<typename boidtype>
void flock::update(paramlist const& params)
{
  std::for_each(oneapi::dpl::execution::par_unseq, set.begin(), set.end(),
                [&](auto& boid) {
                  boid.update_allneighbors(set, params.neigh_repulsion,
                                           params.neigh_align, params.alpha,
                                           params.size, params.flocksize);
                  boid.update_rules(params);

                  // std::cout<<"il numero di vicini e molto vicini è
                  // "<<boid.get_neighbors().size()<<" e
                  // "<<boid.get_close_neighbors().size()<<"\n";
                });
  std::for_each(std::execution::par_unseq, set.begin(), set.end(),
                [&](auto& boid) { boid.posvel_update(params.deltaT); });
}

} // namespace boids
#endif