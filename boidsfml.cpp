#include "boids.hpp"
#include "sfmlboids.hpp"
int main()
{
  

  boids::SFML_interface interface("parametrisfml.txt");

  sf::RenderWindow window(sf::VideoMode(interface.get_params().pixel[0], interface.get_params().pixel[1]),
                          "boids simulation");

  const sf::Time frameTime = sf::seconds(interface.get_params().deltaT);

  sf::Clock clock;

  while (window.isOpen()) {
    sf::Event evento;
    while (window.pollEvent(evento)) {
      if (evento.type == sf::Event::Closed)
        window.close();
    }

    clock.restart();
    interface.get_flock().update(interface.get_params());
    window.clear(sf::Color::White);

    auto boidit=interface.get_flock().set_().begin();
    for (auto& arrow : interface.set_Arrowset()) {
      float angle       = static_cast<float>(boids::angle(boidit->get_vel()));      
      arrow.setPosition(
          static_cast<float>(boidit->get_pos()[0] / boids::params::rate),
          static_cast<float>(boidit->get_pos()[1] / boids::params::rate));
      arrow.setRotation(angle * 180 / static_cast<float>(M_PI));
      window.draw(arrow);
      boidit++;
    }

    window.display();

    if (frameTime < clock.getElapsedTime())
      std::cout << "Lag" << "\n";
    sf::sleep(frameTime - clock.getElapsedTime());
  }
}
