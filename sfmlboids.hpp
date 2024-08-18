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


class SFMLboid : public boidstate
{
  public:
  sf::ConvexShape arrow;
  
  SFMLboid();
  //void arrow_update();
  //void posvel_update(const float deltaT, sf::RenderWindow& window);
};

void assigncolors(flock<SFMLboid>& ensemble,
                  std::vector<RGB> const& colorvec);
} // namespace boids
#include "sfmlboids.tpp"
#endif