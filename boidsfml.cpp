#include "boids.hpp"
#include "sfmlboids.hpp"
using namespace std::chrono_literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
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
    auto t1 = high_resolution_clock::now();
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
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> ms_double = t2 - t1;
    std::cout << "posizione primo boid " << interface.get_flock().set_()[0].get_pos()[0] << "\n";
    std::cout << ms_double.count() << "ms\n";
    if (frameTime < clock.getElapsedTime())
      std::cout << "Lag" << "\n";
    sf::sleep(frameTime - clock.getElapsedTime());
  }
}
