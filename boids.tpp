#ifndef BOIDS_TPP
#define BOIDS_TPP
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>
namespace boids {

template<class boidtype>
void flock<boidtype>::update(paramlist const& params)
{
  std::for_each(oneapi::dpl::execution::par_unseq, set.begin(), set.end(),
                [&](auto& boid) {
                  boid->update_allneighbors(set, params.neigh_repulsion,
                                           params.neigh_align, params.alpha,
                                           params.size, params.flocksize);
                  boid->update_rules(params);

                  // std::cout<<"il numero di vicini e molto vicini è
                  // "<<boid.get_neighbors().size()<<" e
                  // "<<boid.get_close_neighbors().size()<<"\n";
                });
  std::for_each(std::execution::par_unseq, set.begin(), set.end(),
                [&](auto& boid) { boid->posvel_update(params.deltaT); });
}

template<class boidtype>
void flock<boidtype>::generate_flock(std::default_random_engine& eng,
                                      paramlist const& params)
{
  for (unsigned int i = 0; i < params.size; i++) {
    auto pix = params.pixel.begin();
    auto boidprova=std::make_shared<boidtype>();
    boidprova->random_boid(eng, params);
    boidprova->set_ID() = i / params.flocksize;
    for (auto it = boidprova->set_pos().begin(); it != boidprova->set_pos().end();
         ++it, ++pix) {
      std::uniform_real_distribution<double> dis(
          0, static_cast<double>(*pix * params::rate));
      *it += dis(eng);
    }
    set.push_back(boidprova);
  }
}
template<class boidtype>
flock<boidtype>::flock(std::default_random_engine& eng, paramlist const& params)
      : set{} 
  {
    generate_flock(eng, params);
  }

} // namespace boids
#endif