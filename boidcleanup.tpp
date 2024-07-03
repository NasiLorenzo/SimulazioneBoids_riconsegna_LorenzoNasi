#ifndef BOIDCLEANUP_TPP
#define BOIDCLEANUP_TPP
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>
// #include "boidcleanup.hpp"
namespace boids {

template<class boidtype>
boidtype functions<boidtype>::generate(std::default_random_engine& eng)
{ // genera pos e vel di un boid distribuiti secondo
  // una gauss centrata in 0
  boidtype boid{};
  // std::array<double,2> Uniform2D {std::uniform_real_distribution<double>
  // dis(0, static_cast<double>(pixe * params::rate));};

  std::normal_distribution<double> dist(0.0, params::sigma);
  std::for_each(boid.vel.begin(), boid.vel.end(),
                [&](double& x) { x = params::vel_factor * dist(eng); });
  return boid;
}
template<class boidtype>
std::vector<boidtype>
functions<boidtype>::generator(std::default_random_engine& eng,
                               paramlist const& params)
{
  std::vector<boidtype> set{};
  for (unsigned int i = 0; i < params.size; i++) {
    auto pix = params.pixel.begin(); // puntatore ai pixel
    boidtype boidprova{generate(eng)};
    boidprova.flockID = i / params.flocknumber;
    for (auto it = boidprova.pos.begin(); it != boidprova.pos.end();
         ++it, ++pix) {
      std::uniform_real_distribution<double> dis(
          0, static_cast<double>(*pix * params::rate));
      *it += dis(eng);
    }
    set.push_back(boidprova);
  }

  return set;
}

} // namespace boids
#endif