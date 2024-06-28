#include "boidcleanup.hpp"

int main()
{
  boids::paramlist params{};
  /*params.repulsione      = 0.2;
  params.steering        = 0.04;
  params.coesione        = 0.08;
  params.neigh_align     = 200;
  params.neigh_repulsion = 15;
  params.attraction      = 1.0;
  params.alpha           = (1. / 3.) * M_PI;
  params.speedlimit      = 100;
  params.speedminimum    = 40;
  params.deltaT          = 1 / 30.;*/

  std::ifstream input{"parametri.txt"};
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
        params.size = value;
      else if (name == "flocknumber")
        params.flocknumber = static_cast<unsigned int>(value);
      else if (name == "pixel.x")
        params.pixel[0] = static_cast<unsigned int>(value);
      else if (name == "pixel.y")
        params.pixel[1] = static_cast<unsigned int>(value);
    }
  }
  std::cout << "repulsione: " << params.repulsione << std::endl;
  std::cout << "steering: " << params.steering << std::endl;
  std::cout << "coesione: " << params.coesione << std::endl;
  std::cout << "neigh_align: " << params.neigh_align << std::endl;
  std::cout << "neigh_repulsion: " << params.neigh_repulsion << std::endl;
  std::cout << "attraction: " << params.attraction << std::endl;
  std::cout << "alpha: " << params.alpha << std::endl;
  std::cout << "speedlimit: " << params.speedlimit << std::endl;
  std::cout << "speedminimum: " << params.speedminimum << std::endl;
  std::cout << "deltaT: " << params.deltaT << std::endl;
  std::cout << "size" << params.size << std::endl;
  std::random_device r;
  std::default_random_engine eng(r());
  std::vector<boids::SFMLboid> flock =
      boids::functions<boids::SFMLboid>::generator(eng, params);
  boids::ensemble<boids::SFMLboid> prova(flock);
  std::vector<boids::RGB> colorvec =
      boids::functions<boids::SFMLboid>::generatecolors(eng, params);
  prova.update(params);
  std::cout << "dimesione dopo update " << prova.size_() << "\n";
  prova.update(params);
  std::cout << "dimesione dopo update " << prova.size_() << "\n";

  sf::RenderWindow window(sf::VideoMode(params.pixel[0], params.pixel[1]),
                          "Boids Simulation");
  // Desired frame rate
  const sf::Time frameTime = sf::seconds(params.deltaT);
  sf::Clock clock;
  // sf::Time accumulator = sf::Time::Zero;
  /*sf::Texture texture;
  texture.loadFromFile("tramonto.jpg");
  sf::Sprite backgroundSprite(texture);*/
  for (auto& it : prova.newset_()) {
    it.setcolor(colorvec);
    std::cout << "il colore è :" << it.arrow.getFillColor().toInteger() << "\n";
  }
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }
    // Calculate elapsed time for this frame
    clock.restart();

    // Adding a background image
    /*sf::FileInputStream stream;
    stream.open("tramonto.jpg");*/
    prova.update(params);
    window.clear(sf::Color::White);
    // window.draw(backgroundSprite);
    //  Draw boids
    for (auto& boid : prova.set_()) {
      float angle = static_cast<float>(boids::functions<boids::SFMLboid>::angle(
          boid)); // Assuming you have the angle in degrees

      // Calculate arrow positiion
      sf::Vector2<float> arrowPos(
          static_cast<float>(boid.pos[0] / boids::params::rate),
          static_cast<float>(boid.pos[1] / boids::params::rate));
      boid.arrow.setPosition(arrowPos);
      boid.arrow.setRotation(angle * 180 / static_cast<float>(M_PI));
      //std::cout<<"Il color è "<<boid.arrow.getFillColor().g<<"\n";
      window.draw(boid.arrow);
    }

    window.display();
    // Delay to achieve desired frame rate
    if (frameTime < clock.getElapsedTime())
      std::cout << "Lag" << "\n";
    sf::sleep(frameTime - clock.getElapsedTime());
  }
}
