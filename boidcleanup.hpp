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
  static constexpr double sigma{0.01}; //gli esagomi vengono se c'è un parametro di steering basso e una distanza di reticolo bassa
  static constexpr unsigned int dim{2};  // dimensione
  static constexpr double neigh_co{90};
  static constexpr double reproduction{20};
  static constexpr unsigned int interazioni{2};
  static constexpr float deltaT{1/20.f};
  static constexpr unsigned int n = 2;
  //unsigned int vert{20};
  //unsigned int hor{20};
  static constexpr unsigned int size{500};
  static constexpr double rate{4};//rapporto tra la dimensione dello schermo e della generazione
  static constexpr unsigned int rate2{20};
  static constexpr double vel_factor{10000};
  static constexpr double pi=3.141592;
  static constexpr double theta{pi/12};
  //static const std::vector<unsigned int> pixel;
};
struct paramms{
  static double repulsione;
  static double steering;
  static double coesione;
  static double neigh_align;//raggio visivo
  static double neigh2;//raggio di repulsione
};  

struct boidstate {
  std::array<double, params::dim> pos;
  std::array<double, params::dim> vel;
};

inline auto generate(std::default_random_engine );

inline double distance(const boidstate& , const boidstate& );

using stormo = std::vector<boidstate>;

inline auto generator(std::default_random_engine eng);

inline auto regola1(stormo& neighbors, boidstate& boidi);//repulsion
inline auto regola2(stormo& neighbors, boidstate& boidi);//steering
inline auto regola3(stormo& neighbors, boidstate& boidi);//cohesion

inline void meiosi(stormo& set, stormo& neighborss, boidstate& boid, std::default_random_engine eng, double distance); //requires revision

class ensemble {
  stormo set;
  stormo newset{set};

 public:
  ensemble(stormo& old) : set{old} {}
  stormo set_();
  stormo newset_();
  std::size_t size_();
  void update();
  void brown_update(std::random_device& r);
};
}

#endif