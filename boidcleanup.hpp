#ifndef BOIDCLEANUP_HPP
#define BOIDCLEANUP_HPP
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <array>
#include <bitset>
#include <cassert>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <random>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

namespace boids {

struct params
{
  static constexpr double sigma{0.01};
  static constexpr unsigned int dim{2}; // dimensione
  static constexpr unsigned int n = 2;
  static constexpr unsigned int size{100};
  static constexpr double rate{1}; // rapporto tra la dimensione dello schermo e della generazione
  static constexpr double vel_factor{10000};
};

struct paramlist{
  double repulsione;   
  double steering;     
  double coesione;    
  double neigh_align;
  double neigh_repulsion;
  double attraction;
  double alpha;
  double speedlimit;
  double speedminimum;
  float deltaT;
};
struct boidstate
{
  std::array<double, params::dim> pos;
  std::array<double, params::dim> vel;
};

static const std::vector<unsigned int> pixel{1010, 710};

inline boidstate generate(std::default_random_engine& eng);

inline double distance(boidstate const&, boidstate const&);

using stormo = std::vector<boidstate>;

std::array<double, params::dim>
operator+(std::array<double, params::dim> const&,
          std::array<double, params::dim> const&);
std::array<double, params::dim> operator*(const double,
                                          std::array<double, params::dim>&);
std::array<double, params::dim>
operator+=(std::array<double, params::dim>&,
           std::array<double, params::dim> const&);
std::array<double, params::dim> operator/(double,
                                          std::array<double, params::dim>&);
std::array<double, params::dim>
operator-(std::array<double, params::dim> const&,
          std::array<double, params::dim> const&);
std::array<double, params::dim> operator/(std::array<double, params::dim>&,
                                          double);
double mod(std::array<double, params::dim> const& vec);
std::array<double, params::dim> normalize(std::array<double, params::dim>& vec);
stormo generator(std::default_random_engine&);

void regola1(stormo& neighbors, boidstate& boid_old,
             const double repulsione); // repulsion
void regola2(stormo& neighbors, boidstate& boidi, boidstate& boid,
             const double steering); // steering
void regola3(stormo& neighbors, boidstate& boidi,
             const double cohesion); // cohesion
std::array<double, params::dim>
operator+=(std::array<double, params::dim>&,
           std::array<double, params::dim> const&);

void speedadjust(boidstate& boid, const double speedadjust,
                 const double speedminimum);

inline void meiosi(stormo& set, stormo& neighborss, boidstate& boid,
                   std::default_random_engine eng,
                   double distance); // requires revision

double angle(boidstate const& boid);
class ensemble
{
  stormo set;
  stormo newset{set};

 public:
  ensemble(stormo& old)
      : set{old}
  {}
  stormo& set_();
  stormo& newset_();
  std::size_t size_();
  boidstate delta();
  void update(paramlist const&);
};
} // namespace boids

#endif