#include "boidcleanup.hpp"
using namespace boids;

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

std::array<double, params::dim>
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

//operator overload da inserire in cpp e hpp

int main()
{
  std::array<double, params::dim> a{2., 5.};
  std::array<double, params::dim> b{1., 7.};
  std::array<double, params::dim> c;
  c = a * b;
  std::cout << c[0] << ", " << c[1] << '\n';
}