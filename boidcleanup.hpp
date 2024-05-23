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
#include <chrono>
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
  static constexpr float deltaT{1 / 30.f};
  static constexpr unsigned int n = 2;
  // unsigned int vert{20};
  // unsigned int hor{20};
  static constexpr unsigned int size{20};
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
  static double neigh2; 
  static double mod_align; 
  static double attraction; 
  static double alpha; 
  static double speedlimit;
  static double speedminimum;  // raggio di repulsione
};

//inline std::array<double, params::dim> operator+(std::array<double, params::dim>, std::array<double, params::dim>);


struct boidstate
{
  std::array<double,params::dim> pos;
  std::array<double,params::dim> vel;
};

static const std::vector<unsigned int> pixel{1010, 710};

inline boidstate generate(std::default_random_engine& eng);

inline double distance(boidstate const&, boidstate const&);

using stormo = std::vector<boidstate>;

std::array<double,params::dim> operator+(std::array<double,params::dim> const&, std::array<double,params::dim> const& );
std::array<double,params::dim> operator*(const double, std::array<double,params::dim>& );
std::array<double,params::dim> operator+=(std::array<double,params::dim>&, std::array<double,params::dim> const& );
std::array<double, params::dim> operator/(double, std::array<double, params::dim>&);
std::array<double,params::dim> operator-(std::array<double,params::dim> const&, std::array<double,params::dim> const& );
std::array<double, params::dim> operator/(std::array<double, params::dim>&, double);
double mod(std::array<double,params::dim> const& vec);
std::array<double,params::dim> normalize(std::array<double,params::dim>& vec);
stormo generator(std::default_random_engine&);

void regola1(stormo& neighbors, boidstate& boid_old); // repulsion
void regola2(stormo& neighbors, boidstate& boidi, boidstate& boid); // steering
void regola3(stormo& neighbors, boidstate& boidi); // cohesion
auto regola4(stormo& neighbors, boidstate& boid);
std::array<double, params::dim> operator+=(std::array<double, params::dim>&, std::array<double, params::dim>const&);

void speedadjust(boidstate& boid);

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
  stormo set_();
  stormo newset_();
  std::size_t size_();
  boidstate delta();
  void update();
  void brown_update(std::random_device& r);
};
} // namespace boids

#endif