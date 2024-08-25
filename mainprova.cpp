#include "boids.hpp"
using namespace boids;
using namespace std::chrono_literals;

/*std::array<double, params::dim>
operator+(const std::array<double, params::dim>& a,
          const std::array<double, params::dim>& b)
{
  std::array<double, params::dim> result{};
  auto r_it = result.begin();
  for (auto a_it = a.begin(), b_it = b.begin(); a_it != a.end();
       ++a_it, ++b_it, ++r_it) {
    *r_it += *a_it + *b_it;
  };
  return result;
}

// std::transform: algoritmo da guardare per gli operatori

std::array<double, params::dim> operator+=(std::array<double, params::dim> a,
                                          std::array<double, params::dim> b)
{
  for (auto a_it = a.begin(), b_it = b.begin();
       a_it != a.end(); ++a_it, ++b_it) {
    *a_it += *b_it;
  };
  return a;
}

std::array<double, params::dim>
operator*(const std::array<double, params::dim>& a,
          const std::array<double, params::dim>& b)
{
  std::array<double, params::dim> result{0, 0};
  auto r_it = result.begin();
  for (auto a_it = a.begin(), b_it = b.begin(); a_it != a.end();
       ++a_it, ++b_it, ++r_it) {
    *r_it += *a_it * *b_it;
  }
  return result;
}


}*/

/*std::array<double, params::dim>
operator+(const std::array<double, params::dim>& a,
          const std::array<double, params::dim>& b)
{
  std::array<double, params::dim> result{};
  std::transform(a.begin(), a.end(), b.begin(), result.begin(),
                 [&](double c, double d) { return c + d; });
  return result;
}

std::array<double, params::dim>
operator*(const std::array<double, params::dim>& a,
          const std::array<double, params::dim>& b)
{
  std::array<double, params::dim> result{};
  std::transform(a.begin(), a.end(), b.begin(), result.begin(),
                 [&](double c, double d) { return c * d; });
  return result;
}

double distance2(boidstate const& a, boidstate const& b)
{
  return pow(a.pos[0] - b.pos[0], 2) + pow(a.pos[1] - b.pos[1], 2);
}*/

int main()
{
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::milliseconds;
  paramlist params;
  params.repulsione = 0.9; // tenere il parametro di repulsione un ordine di
                           // grandezza superiore agli altri due
  params.steering        = 0.06;
  params.coesione        = 0.08;
  params.view_range      = 120;
  params.repulsion_range = 15;
  params.attraction      = 200;
  params.alpha           = 0.55;
  params.speedlimit      = 200;
  params.speedminimum    = 80;
  params.deltaT          = static_cast<float>(0.0333);
  params.size            = 2000;
  params.flocksize       = 2000;
  params.pixel[0]        = 1510;
  params.pixel[1]        = 910;
  params::rate           = 1;
  params.bordersize      = 50;
  params.sigma=100;

  std::default_random_engine eng{1};
  flock stormo{eng, params};
  auto t1 = high_resolution_clock::now();
  for(int i=0; i<100; i++){
    stormo.update(params);
  }
  auto t2 = high_resolution_clock::now();

  /* Getting number of milliseconds as a double. */
  duration<double, std::milli> ms_double = t2 - t1;
  std::cout << "posizione primo boid " << stormo.set_()[0].get_pos()[0] << "\n";
  std::cout << ms_double.count() << "ms\n";
}
