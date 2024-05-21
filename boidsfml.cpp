#include "boidcleanup.hpp"

int main()
{
  boids::paramms::repulsione  = 0.2;
  boids::paramms::steering    = 0.02;
  boids::paramms::coesione    = 0.08;
  boids::paramms::neigh_align = 100;
  boids::paramms::neigh2      = 15;
  boids::paramms::mod_align=0.000003;
  boids::paramms::attraction=25;
  boids::paramms::alpha       = (1./2.)*M_PI;
  boids::paramms::speedlimit =200;
  boids::paramms::speedminimum=80;

  std::random_device r;
  std::default_random_engine eng(r());
  boids::stormo flock = boids::generator(eng);
  boids::ensemble prova(flock);
  std::cout << "Dimensione generazione" << prova.size_() << "\n";
  std::cout << "Coesione" << boids::paramms::coesione << "\n";
  std::cout << "Distanta max" << boids::paramms::neigh_align << "\n";
  std::cout << "Repulsione" << boids::paramms::repulsione << "\n";

  prova.update();

  sf::RenderWindow window(sf::VideoMode(boids::pixel[0], boids::pixel[1]),
                          "Boids Simulation");

  // Desired frame rate
  const sf::Time frameTime = sf::seconds(boids::params::deltaT);

  sf::Clock clock;
  //sf::Time accumulator = sf::Time::Zero;

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
    clock.restart();
    //accumulator += elapsedTime;

    // Adding a background image
    /*sf::FileInputStream stream;
    stream.open("tramonto.jpg");*/
    prova.update();
    // Update the simulation while we have enough time accumulated
    /*while (accumulator >= frameTime) {
      prova.update();
      accumulator -= frameTime;
      // std::cout<<"Ce so passato "<<i<< " volte\n";
    }*/

    window.clear(sf::Color::White);
    //window.draw(backgroundSprite);
    // Draw boids
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
      float angle = static_cast<float>(boids::angle(boid)); // Assuming you have the angle in degrees

      // Arrow length and width
      float arrowLength = 10;
      float arrowWidth = 5;

            // Calculate arrow positiion            
      sf::Vector2<float> arrowPos(static_cast<float>(boid.pos[0] / boids::params::rate),
          static_cast<float>(boid.pos[1]/ boids::params::rate));
    

      // Draw arrow
      sf::ConvexShape arrow(3);
      arrow.setPoint(0, sf::Vector2f(arrowLength,0));
      arrow.setPoint(1, sf::Vector2f(0,-arrowWidth/2));
      arrow.setPoint(2, sf::Vector2f(0,arrowWidth/2));
      arrow.setFillColor(sf::Color::Red);
      arrow.setPosition(arrowPos);
      arrow.setRotation(angle*180/static_cast<float>(M_PI));
      window.draw(arrow);
    }

    window.display();

    // Delay to achieve desired frame rate
    if(frameTime < clock.getElapsedTime())
    std::cout<<"Lag"<<"\n";
    sf::sleep(frameTime - clock.getElapsedTime());
  }
}
