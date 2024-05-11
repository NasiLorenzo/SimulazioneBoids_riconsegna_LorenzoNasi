#include "boidcleanup.hpp"

int main()
{
  boids::paramms::repulsione  = 0.2;
  boids::paramms::steering    = 0.07;
  boids::paramms::coesione    = 0.01;
  boids::paramms::neigh_align = 70;
  boids::paramms::neigh2      = 15;
  boids::paramms::mod_align=0.000003;
  std::random_device r;
  std::default_random_engine eng(r());
  boids::stormo flock = boids::generator(eng);
  boids::ensemble prova(flock);
  std::cout << "Dimensione generazione" << prova.size_() << "\n";

  prova.update();

  sf::RenderWindow window(sf::VideoMode(boids::pixel[0], boids::pixel[1]),
                          "Boids Simulation");

  // Desired frame rate
  const sf::Time frameTime = sf::seconds(boids::params::deltaT);

  sf::Clock clock;
  sf::Time accumulator = sf::Time::Zero;

  sf::Texture texture;
  texture.loadFromFile("tramonto.jpg");
  sf::Sprite backgroundSprite(texture);
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }
    // Calculate elapsed time for this frame
    sf::Time elapsedTime = clock.restart();
    accumulator += elapsedTime;

    // Adding a background image
    /*sf::FileInputStream stream;
    stream.open("tramonto.jpg");*/

    // Update the simulation while we have enough time accumulated
    int i = 0;
    while (accumulator >= frameTime) {
      i++;
      prova.update();
      accumulator -= frameTime;
      // std::cout<<"Ce so passato "<<i<< " volte\n";
    }

    window.clear(sf::Color::White);
    //window.draw(backgroundSprite);
    // Draw boids
    for (auto& boid : prova.set_()) {
      sf::CircleShape circle(2);
      // std::cout<<prova.set_().size()<<"\n";
      circle.setFillColor(sf::Color::Black);
      circle.setPosition(
          static_cast<float>(boid.pos[0] / boids::params::rate),
          static_cast<float>(
              boid.pos[1]
              / boids::params::rate)); // Assuming x and y are in pos[0]
                                       // and pos[1] respectively
      window.draw(circle);
    }

    window.display();

    // Delay to achieve desired frame rate
    sf::sleep(frameTime - clock.getElapsedTime());
  }
}
