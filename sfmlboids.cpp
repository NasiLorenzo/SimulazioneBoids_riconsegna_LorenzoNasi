#include "sfmlboids.hpp"
namespace boids {
std::vector<RGB> generatecolors(std::default_random_engine& eng,
                                ParamList const& params)
{
  std::vector<RGB> colorvec{};
  std::uniform_int_distribution dist(0, 255);
  for (unsigned i = 0; i < params.size / params.flocksize + 1; i++) {
    RGB color{};
    color.red   = static_cast<uint8_t>(dist(eng));
    color.blue  = static_cast<uint8_t>(dist(eng));
    color.green = static_cast<uint8_t>(dist(eng));
    colorvec.push_back(color);
  }
  return colorvec;
}

std::vector<sf::ConvexShape> buildArrowSet(unsigned int size,
                                           std::vector<RGB> colorvec,
                                           const double rate,
                                           unsigned int flocksize)
{
  std::vector<sf::ConvexShape> arrowset{};
  for (unsigned int i = 0; i < size; i++) {
    sf::ConvexShape arrow;
    float arrowlength = 10 / static_cast<float>(rate);
    float arrowidth   = 5 / static_cast<float>(rate);
    arrow.setPointCount(3);
    arrow.setPoint(0, sf::Vector2f(arrowlength, 0));
    arrow.setPoint(1, sf::Vector2f(0, -arrowidth / 2));
    arrow.setPoint(2, sf::Vector2f(0, arrowidth / 2));
    arrow.setOrigin(arrowlength / 2, 0);
    arrow.setFillColor(sf::Color(colorvec[i / flocksize].red,
                                 colorvec[i / flocksize].green,
                                 colorvec[i / flocksize].blue));
    arrowset.push_back(arrow);
  }
  return arrowset;
}

void SFML_Interface::run()
{
  const sf::Time frameTime = sf::seconds(params_.deltaT);
  sf::Clock clock;
  int i = 0;
  while (window_.isOpen()) {
    sf::Event evento;
    while (window_.pollEvent(evento)) {
      if (evento.type == sf::Event::Closed)
        window_.close();
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    clock.restart();
    flock_.update(params_);
    window_.clear(sf::Color::White);

    auto boidit = flock_.set().begin();
    for (auto& arrow : arrowset_) {
      float angle = static_cast<float>(boids::angle(boidit->vel()));
      arrow.setPosition(
          static_cast<float>(boidit->pos()[0] / params_.rate),
          static_cast<float>(boidit->pos()[1] / params_.rate));
      arrow.setRotation(angle * 180 / static_cast<float>(M_PI));
      window_.draw(arrow);
      boidit++;
    }

    window_.display();
    i++;
    auto t2                                = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms_double = t2 - t1;
    std::cout << "posizione primo boid "
              << flock_.set()[0].pos()[0] << " " << i << "\n";
    std::cout << ms_double.count() << "ms\n";
    if (frameTime < clock.getElapsedTime())
      std::cout << "Lag" << "\n";
    sf::sleep(frameTime - clock.getElapsedTime());
  }
}

} // namespace boids