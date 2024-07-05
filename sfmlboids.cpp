#include "sfmlboids.hpp"
namespace boids {
std::vector<RGB> generatecolors(std::default_random_engine& eng,
                                paramlist const& params)
{
  std::vector<RGB> colorvec{};
  for (unsigned i = 0; i < params.size / params.flocksize + 1; i++) {
    RGB color{};
    std::uniform_int_distribution dist(0, 255);
    color.red   = static_cast<uint8_t>(dist(eng));
    color.blue  = static_cast<uint8_t>(dist(eng));
    color.green = static_cast<uint8_t>(dist(eng));
    colorvec.push_back(color);
  }
  return colorvec;
}

SFMLboid::SFMLboid()
    : boidstate{}
{
  float arrowlength = 10 / static_cast<float>(params::rate);
  float arrowidth   = 5 / static_cast<float>(params::rate);
  arrow.setPointCount(3);
  arrow.setPoint(0, sf::Vector2f(arrowlength, 0));
  arrow.setPoint(1, sf::Vector2f(0, -arrowidth / 2));
  arrow.setPoint(2, sf::Vector2f(0, arrowidth / 2));
  arrow.setOrigin(arrowlength / 2, 0);
}

} // namespace boids