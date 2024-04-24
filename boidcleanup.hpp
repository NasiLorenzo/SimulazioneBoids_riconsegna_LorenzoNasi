#ifndef BOIDCLEANUP_HPP
#define BOIDCLEANUP_HPP
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <array>
#include <cassert>
#include <iostream>
#include <iterator>
#include <random>
#include <string>
#include <vector>
#include <cmath>
#include <functional>

namespace boids{
struct params {
  static constexpr double repulsione{0.6};
  static constexpr double steering{0.07};
  static constexpr double coesione{0.01};
  static constexpr double sigma{0.01}; //gli esagomi vengono se c'è un parametro di steering basso e una distanza di reticolo bassa
  static constexpr unsigned int dim{2};  // dimensione
  static constexpr double neigh_co{90};
  static constexpr double neigh_align{70};//raggio visivo
  static constexpr double neigh2{5};//raggio di repulsione
  static constexpr double reproduction{20};
  static constexpr unsigned int interazioni{2};
  static constexpr double deltaT{0.01};
  static constexpr unsigned int n = 2;
  //unsigned int vert{20};
  //unsigned int hor{20};
  static constexpr unsigned int size{400};
  static constexpr unsigned int rate{1};//rapporto tra la dimensione dello schermo e della generazione
  static constexpr unsigned int rate2{100};
  static constexpr double vel_factor{100000};
  static constexpr double pi=3.141592;
  static constexpr double theta{pi/12};
  static const std::vector<unsigned int> pixel;
};

struct boidstate {
  std::array<double, params::dim> pos;
  std::array<double, params::dim> vel;
};

auto generate(std::default_random_engine );

}

#endif