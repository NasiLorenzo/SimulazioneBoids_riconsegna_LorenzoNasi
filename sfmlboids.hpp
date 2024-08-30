#ifndef SFMLBOIDS_HPP
#define SFMLBOIDS_HPP
#include "boids.hpp"
#include "statistics.hpp"
namespace boids {
struct RGB
{
  uint8_t red;
  uint8_t blue;
  uint8_t green;
};

enum class State : int
{
  stats,
  exit,
  none,
};

std::vector<RGB> generatecolors(std::default_random_engine& eng,
                                ParamList const& params);

void assigncolors(Flock& ensemble, std::vector<RGB> const& colorvec);

ParamList parse_input(std::string const& inputfile);

std::vector<sf::ConvexShape> buildArrowSet(unsigned int size,
                                           std::vector<RGB> colorvec,
                                           const double rate,
                                           unsigned int flocksize);
class SFML_Interface
{
 public:
  SFML_Interface(std::string const& inputfile, std::random_device& r)
      : eng_{r()}
      , params_{inputfile}
      , colorvec_{generatecolors(eng_, params_)}
      , arrowset_{buildArrowSet(params_.size, colorvec_, params_.rate,
                                params_.flocksize)}
      , flock_{eng_, params_}
      , window_{sf::VideoMode(params_.pixel[0], params_.pixel[1],
                              sf::VideoMode::getDesktopMode().bitsPerPixel),
                "boids simulation"}
  {}
  void run();
  auto& arrowset()
  {
    return arrowset_;
  }
  auto& arrowset() const
  {
    return arrowset_;
  }
  auto& params()
  {
    return params_;
  }
  auto& params() const
  {
    return params_;
  }
  auto& flock()
  {
    return flock_;
  }
  auto& flock() const
  {
    return flock_;
  }

 private:
  std::default_random_engine eng_{};
  ParamList params_{};
  std::vector<RGB> colorvec_;
  std::vector<sf::ConvexShape> arrowset_{};
  Flock flock_;
  sf::RenderWindow window_;
};

void show_stats(Flock const& flock);

State SFML_menu();

} // namespace boids
#endif