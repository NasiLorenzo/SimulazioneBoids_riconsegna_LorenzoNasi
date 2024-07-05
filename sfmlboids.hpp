#ifndef SFMLBOIDS_HPP
#define SFMLBOIDS_HPP
#include "boids.hpp"
namespace boids {
struct RGB
{
  uint8_t red;
  uint8_t blue;
  uint8_t green;
};

std::vector<RGB> generatecolors(std::default_random_engine& eng,
                                paramlist const& params);

template<class boidtype>
void assigncolors(ensemble<boidtype>& ensemble,
                  std::vector<RGB> const& colorvec);

struct SFMLboid : boidstate
{
  sf::ConvexShape arrow;

  SFMLboid();
};

} // namespace boids
#include "sfmlboids.tpp"
#endif