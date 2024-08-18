#include "boids.hpp"
#include "sfmlboids.hpp"
int main()
{
  boids::paramlist params{};
  std::ifstream input{"parametrisfml.txt"};
  if (!input) {
    std::cerr << "File di input non trovato!\n";
    return 1;
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
      else if (name == "pixel.x")
        params.pixel[0] = static_cast<unsigned int>(value);
      else if (name == "pixel.y")
        params.pixel[1] = static_cast<unsigned int>(value);
      else if (name == "rate")
        boids::params::rate = value;
      else if (name == "bordersize")
        params.bordersize = value;
      else if (name == "sigma")
        params.sigma = value;
    }
  }

  std::random_device r;
  std::default_random_engine eng(r());
  boids::flock<boids::SFMLboid> stormo{eng, params};
  std::vector<boids::RGB> colorvec = boids::generatecolors(eng, params);
  assigncolors(stormo, colorvec);

  sf::RenderWindow window(sf::VideoMode(params.pixel[0], params.pixel[1]),
                          "boids simulation");

  const sf::Time frameTime = sf::seconds(params.deltaT);
  sf::Clock clock;
  
  while (window.isOpen()) {
    sf::Event evento;
    while (window.pollEvent(evento)) {
      if (evento.type == sf::Event::Closed)
        window.close();
    }

    clock.restart();
    stormo.update(params);
    window.clear(sf::Color::White);

    for (auto& boid : stormo.set_()) {
      auto sfml_ptr=std::dynamic_pointer_cast<boids::SFMLboid>(boid);
      float angle = static_cast<float>(boids::angle(sfml_ptr->get_vel()));
      sfml_ptr->arrow.setPosition(
          static_cast<float>(boid->get_pos()[0] / boids::params::rate),
          static_cast<float>(boid->get_pos()[1] / boids::params::rate));
      sfml_ptr->arrow.setRotation(angle * 180 / static_cast<float>(M_PI));
      window.draw(sfml_ptr->arrow);
    }

    window.display();

    if (frameTime < clock.getElapsedTime())
      std::cout << "Lag" << "\n";
    sf::sleep(frameTime - clock.getElapsedTime());
  }
}
