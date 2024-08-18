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

void assigncolors(flock& ensemble, std::vector<RGB> const& colorvec);

paramlist parse_input(std::string const& inputfile);

sf::ConvexShape buildArrow(unsigned int i, std::vector<RGB> colorvec);
class SFML_interface
{
  paramlist params{};
  std::vector<sf::ConvexShape> Arrowset{};
  std::vector<RGB> colorvec;
  std::random_device r;
  std::default_random_engine eng{r()};
  flock set;

 public:
  SFML_interface(std::string const& inputfile)
      : params{parse_input(inputfile)}
      , colorvec{generatecolors(eng, params)}
      , set{eng, params}
  {
    for(unsigned int i=0;i<params.size;i++ ) {
      Arrowset.push_back(buildArrow(set.set_()[i].get_ID(), colorvec));
    }
  }
  auto& set_Arrowset() {
    return Arrowset;
  }
  auto& get_params(){
    return params;
  }
  auto& get_flock(){
    return set;
  }
};

} // namespace boids
#include "sfmlboids.tpp"
#endif