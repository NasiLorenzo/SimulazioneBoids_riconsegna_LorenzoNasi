#include "boidcleanup.hpp"

int main()
{
  boids::paramlist params{};
  params.repulsione   = 0.2;
  params.steering     = 0.04;
  params.coesione     = 0.08;
  params.neigh_align  = 200;
  params.neigh2       = 15;
  params.mod_align    = 0.000003;
  params.attraction   = 1.0;
  params.alpha        = (1. / 3.) * M_PI;
  params.speedlimit   = 100;
  params.speedminimum = 40;
  std::random_device r;
  std::default_random_engine eng(r());
  boids::stormo flock = boids::generator(eng);
  boids::ensemble prova(flock);

  prova.update(params);
  std::cout << "dimesione dopo update " << prova.size_() << "\n";
  prova.update(params);
  std::cout << "dimesione dopo update " << prova.size_() << "\n";

  sf::RenderWindow window(sf::VideoMode(boids::pixel[0], boids::pixel[1]),
                          "Boids Simulation");

  // Desired frame rate
  const sf::Time frameTime = sf::seconds(boids::params::deltaT);

  sf::Clock clock;
  // sf::Time accumulator = sf::Time::Zero;

  /*sf::Texture texture;
  texture.loadFromFile("tramonto.jpg");
  sf::Sprite backgroundSprite(texture);*/
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }
    // Calculate elapsed time for this frame
    clock.restart();
    // accumulator += elapsedTime;

    // Adding a background image
    /*sf::FileInputStream stream;
    stream.open("tramonto.jpg");*/
    prova.update(params);
    // Update the simulation while we have enough time accumulated
    /*while (accumulator >= frameTime) {
      prova.update();
      accumulator -= frameTime;
      // std::cout<<"Ce so passato "<<i<< " volte\n";
    }*/

    window.clear(sf::Color::White);
    // window.draw(backgroundSprite);
    //  Draw boids
    for (auto& boid : prova.set_()) {
      /*sf::CircleShape circle(2);
      // std::cout<<prova.set_().size()<<"\n";
      circle.setFillColor(sf::Color::Black);
      circle.setPosition(
          static_cast<float>(boid.pos[0] / boids::params::rate),
          static_cast<float>(
              boid.pos[1]
              / boids::params::rate)); // Assuming x and y are in pos[0]
                                       // and pos[1] respectively
      window.draw(circle);*/
      float angle = static_cast<float>(
          boids::angle(boid)); // Assuming you have the angle in degrees

      // Arrow length and width
      float arrowLength = 10;
      float arrowWidth  = 5;

      // Calculate arrow positiion
      sf::Vector2<float> arrowPos(
          static_cast<float>(boid.pos[0] / boids::params::rate),
          static_cast<float>(boid.pos[1] / boids::params::rate));

      // Draw arrow
      sf::ConvexShape arrow(3);
      arrow.setPoint(0, sf::Vector2f(arrowLength, 0));
      arrow.setPoint(1, sf::Vector2f(0, -arrowWidth / 2));
      arrow.setPoint(2, sf::Vector2f(0, arrowWidth / 2));
      arrow.setFillColor(sf::Color::Red);
      arrow.setPosition(arrowPos);
      arrow.setRotation(angle * 180 / static_cast<float>(M_PI));
      window.draw(arrow);
    }

    window.display();

    // Delay to achieve desired frame rate
    if (frameTime < clock.getElapsedTime())
      std::cout << "Lag" << "\n";
    sf::sleep(frameTime - clock.getElapsedTime());
  }
}
