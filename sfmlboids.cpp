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

void assigncolors(flock<SFMLboid>& ensemble,
                  std::vector<RGB> const& colorvec)
{
  for (auto& it : ensemble.set_()) {
    auto sfmlboid_ptr = std::dynamic_pointer_cast<SFMLboid>(it);
    sfmlboid_ptr->arrow.setFillColor(sf::Color(colorvec[sfmlboid_ptr->set_ID()].red,
                                    colorvec[sfmlboid_ptr->set_ID()].green,
                                    colorvec[sfmlboid_ptr->set_ID()].blue));
  }
}
/*void SFMLboid::arrow_update(){
  float angle = static_cast<float>(boids::angle(this->get_vel()));
      arrow.setPosition(
          static_cast<float>(this->get_pos()[0] / boids::params::rate),
          static_cast<float>(this->get_pos()[1] / boids::params::rate));
      arrow.setRotation(angle * 180 / static_cast<float>(M_PI));
      window.draw(arrow);
}*/

/*void SFMLboid::posvel_update(const float deltaT){
  boid_.vel_+=boid_.deltavel_;
  boid_.pos_[0] += (this->get_vel()[0]) * deltaT;
  this->boid_.pos_[1] += (this->get_vel()[1]) * deltaT;
  boid_.deltavel_={0.,0.};
  arrow_update();
}*/

} // namespace boids