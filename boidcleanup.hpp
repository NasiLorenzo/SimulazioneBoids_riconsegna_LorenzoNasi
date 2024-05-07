#ifndef BOIDCLEANUP_HPP
#define BOIDCLEANUP_HPP
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <random>
#include <string>
#include <vector>

namespace boids {

struct params
{
  static constexpr double sigma{
      0.01}; // gli esagoni vengono se c'è un parametro di steering basso e una
             // distanza di reticolo bassa
  static constexpr unsigned int dim{2}; // dimensione
  static constexpr double neigh_co{90};
  static constexpr double reproduction{20};
  static constexpr unsigned int interazioni{2};
  static constexpr float deltaT{1 / 20.f};
  static constexpr unsigned int n = 2;
  // unsigned int vert{20};
  // unsigned int hor{20};
  static constexpr unsigned int size{500};
  static constexpr double rate{
      1}; // rapporto tra la dimensione dello schermo e della generazione
  static constexpr unsigned int rate2{20};
  static constexpr double vel_factor{10000};
  static constexpr double pi = 3.141592;
  static constexpr double theta{pi / 12};
};

struct paramms
{
  static double repulsione;
  static double steering;
  static double coesione;
  static double neigh_align; // raggio visivo
  static double neigh2;      // raggio di repulsione
};

//inline std::array<double, params::dim> operator+(std::array<double, params::dim>, std::array<double, params::dim>);


struct boidstate
{
  std::array<double, params::dim> pos;
  std::array<double,params::dim> vel;
};

static const std::vector<unsigned int> pixel{1010, 710};

inline boidstate generate(std::default_random_engine eng)
{ // genera pos e vel di un boid distribuiti secondo
  // una gauss centrata in 0
  boidstate boid{};
  std::normal_distribution<double> dist(0.0, params::sigma);
  for (auto it = boid.pos.begin(); it != boid.pos.end(); ++it) {
    *it = dist(eng);
  }
  for (auto it = boid.vel.begin(), last = boid.vel.end(); it != last; ++it) {
    *it = (params::vel_factor * dist(eng));
  }
  return boid;
}

inline double distance(boidstate a, boidstate b)
{ // sqrt dispendiosa
  double s{};
  for (auto it = a.pos.begin(), index = b.pos.begin(); it != a.pos.end();
       ++it, ++index) {
    s += pow((*it) - (*index), 2);
  }
  return s;
}

using stormo = std::vector<boidstate>;

stormo generator(std::default_random_engine);

boidstate regola1(stormo& neighbors, boidstate& boidi); // repulsion
auto regola2(stormo& neighbors, boidstate& boidi); // steering
auto regola3(stormo& neighbors, boidstate& boidi); // cohesion

std::array<double, params::dim> operator+=(std::array<double, params::dim>&, std::array<double, params::dim>const&);

inline void meiosi(stormo& set, stormo& neighborss, boidstate& boid,
                   std::default_random_engine eng,
                   double distance); // requires revision

class ensemble
{
  stormo set;
  stormo newset{set};

 public:
  ensemble(stormo& old)
      : set{old}
  {}
  stormo set_();
  stormo newset_();
  std::size_t size_();
  void update();
  void brown_update(std::random_device& r);
};
} // namespace boids

#endif