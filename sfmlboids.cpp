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

  paramlist parse_input(std::string const& inputfile){
    paramlist params{};
    std::ifstream input{inputfile};
  if (!input) {
    std::cerr << "File di input non trovato!\n";
    //return 1;
  }
  std::string line{};
  while (std::getline(input, line)) {
    std::istringstream inputline(line);
    double value{};
    std::string name{};
    if (inputline >> name >> value) {
      if (name == "repulsione")
        params.repulsione = value;
      else if (name == "steering")
        params.steering = value;
      else if (name == "coesione")
        params.coesione = value;
      else if (name == "neigh_align")
        params.neigh_align = value;
      else if (name == "neigh_repulsion")
        params.neigh_repulsion = value;
      else if (name == "attraction")
        params.attraction = value;
      else if (name == "alpha")
        params.alpha = value * M_PI;
      else if (name == "speedlimit")
        params.speedlimit = value;
      else if (name == "speedminimum")
        params.speedminimum = value;
      else if (name == "deltaT")
        params.deltaT = static_cast<float>(value);
      else if (name == "size")
        params.size = static_cast<unsigned int>(value);
      else if (name == "flocksize")
        params.flocksize = static_cast<unsigned int>(value);
      /*else if (name == "pixel.x")
        params.pixel[0] = static_cast<unsigned int>(value);
      else if (name == "pixel.y")
        params.pixel[1] = static_cast<unsigned int>(value);*/
      else if (name == "rate")
        boids::params::rate = value;
      else if (name == "bordersize")
        params.bordersize = value;
      else if (name == "sigma")
        params.sigma = value;
      else if (name == "rows")
        params.rows = static_cast<int>(value);
      else if (name == "columns")
        params.columns = static_cast<int>(value);
    }
  }
  params.pixel[0]=params.columns*params.neigh_align;
  params.pixel[1]=params.rows*params.neigh_align;
  return params;
  }

sf::ConvexShape buildArrow(unsigned int i, std::vector<RGB> colorvec){
  sf::ConvexShape arrow;
      float arrowlength = 10 / static_cast<float>(boids::params::rate);
      float arrowidth   = 5 / static_cast<float>(boids::params::rate);
      arrow.setPointCount(3);
      arrow.setPoint(0, sf::Vector2f(arrowlength, 0));
      arrow.setPoint(1, sf::Vector2f(0, -arrowidth / 2));
      arrow.setPoint(2, sf::Vector2f(0, arrowidth / 2));
      arrow.setOrigin(arrowlength / 2, 0);
      arrow.setFillColor(sf::Color(colorvec[i].red,
                                    colorvec[i].green,
                                    colorvec[i].blue));
      return arrow;

}

} // namespace boids