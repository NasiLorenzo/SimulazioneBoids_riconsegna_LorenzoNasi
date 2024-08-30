#include "sfmlinterface.hpp"
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

State SFML_menu()
{
  char input;
  std::cout
      << "Type S to see stats, type N to continue, type Q to close the run: "
      << "(statistics heavily impact performance) \n"
      << "____________________________________\n";
  std::cin >> input;
  switch (input) {
  case 'S':
    return State::stats;
    break;
  case 'Q':
    return State::exit;
    break;
  case 'N':
    return State::none;
    break;
  }
  return State::none;
}

void show_stats(Flock const& flock)
{
  FlockStats flock_data{flock.set()};
  std::cout << "The flock stats are: \n"
            << "x position  mean: " << flock_data.pos_stats[0].result().mean
            << " sigma: " << flock_data.pos_stats[0].result().sigma << "\n"
            << "y position  mean: " << flock_data.pos_stats[1].result().mean
            << " sigma: " << flock_data.pos_stats[1].result().sigma << "\n"

            << "x velocity  mean: " << flock_data.vel_stats[0].result().mean
            << " sigma: " << flock_data.vel_stats[0].result().sigma << "\n"
            << "y velocity  mean: " << flock_data.vel_stats[1].result().mean
            << " sigma: " << flock_data.vel_stats[1].result().sigma << "\n"

            << "mean speed: " << flock_data.vel_mod_stats.result().mean
            << " sigma: " << flock_data.vel_mod_stats.result().sigma << "\n"
            << "mean distance from origin: "
            << flock_data.pos_mod_stats.result().mean
            << " sigma: " << flock_data.pos_mod_stats.result().sigma << "\n"
            << "mean distance b-ween boids: "
            << flock_data.distance_stats.result().mean
            << " sigma: " << flock_data.distance_stats.result().sigma << "\n"
            << "___________________________________\n";
}

void SFML_Interface::run()
{
  const sf::Time frameTime = sf::seconds(params_.deltaT);
  sf::Clock clock;
  auto state = SFML_menu();
  if (state == State::exit)
    window_.close();
  while (window_.isOpen()) {
    sf::Event evento;
    while (window_.pollEvent(evento)) {
      if (evento.type == sf::Event::Closed)
        window_.close();
    }
    clock.restart();
    flock_.update(params_);
    window_.clear(sf::Color::White);

    auto boidit = flock_.set().begin();
    
    std::for_each(arrowset_.begin(), arrowset_.end(), [&](auto& arrow) mutable {
      float angle = static_cast<float>(boids::angle(boidit->vel()));
      arrow.setPosition(static_cast<float>(boidit->pos()[0] / params_.rate),
                        static_cast<float>(boidit->pos()[1] / params_.rate));
      arrow.setRotation(angle * 180 / static_cast<float>(M_PI));
      window_.draw(arrow);
      boidit++;
    });

    window_.display();

    if (state == State::stats)
      show_stats(flock_);
    std::cout << " Run time: " << clock.getElapsedTime().asMilliseconds()
              << " ms\n"
              << " Manually close window to exit\n";
    sf::sleep(frameTime - clock.getElapsedTime());
  }
}

} // namespace boids