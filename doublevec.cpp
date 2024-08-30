#include "doublevec.hpp"

namespace boids {
DoubleVec& operator-=(DoubleVec& a, DoubleVec const& b)
{
  std::transform(a.begin(), a.end(), b.begin(), a.begin(), std::minus<>());
  return a;
}

DoubleVec operator-(DoubleVec const& a, DoubleVec const& b)
{
  DoubleVec result{a};
  return result -= b;
}

DoubleVec& operator*=(DoubleVec& a, double const b)
{
  std::for_each(a.begin(), a.end(), [b](auto& x) { x *= b; });
  return a;
}

DoubleVec operator*(DoubleVec const& a, const double b)
{
  auto result{a};
  return result *= b;
}

DoubleVec& operator/=(DoubleVec& b, const double a)
{
  std::for_each(b.begin(), b.end(), [&a](double& x) { x = x / a; });
  return b;
}

DoubleVec operator/(DoubleVec const& b, const double a)
{
  auto result{b};
  return result /= a;
}

DoubleVec& operator+=(DoubleVec& a, DoubleVec const& b)
{
  std::transform(a.begin(), a.end(), b.begin(), a.begin(), std::plus<>());
  return a;
}

DoubleVec operator+(DoubleVec const& a, DoubleVec const& b)
{
  DoubleVec result{a};
  return result += b;
}

double angle(DoubleVec const& vec)
{
  return atan2(vec[1], vec[0]);
}

double mod(DoubleVec const& vec)
{
  return sqrt(
      std::accumulate(vec.begin(), vec.end(), 0.,
                      [](double sum, double x) { return sum = sum + x * x; }));
}

void normalize(DoubleVec& vec)
{
  auto modulo = mod(vec);
  if (modulo != 0)
    vec /= modulo;
}

double distance_squared(DoubleVec const& a, DoubleVec const& b)
{
  return std::transform_reduce(
      a.begin(), a.end(), b.begin(), 0., std::plus<>(),
      [](double c, double d) { return pow(c - d, 2.); });
}

double cos_angle_between(
    DoubleVec const& a,
    DoubleVec const& b) // if one of the vectors is zero, the result is zero, but should be
                        // undefined. Anyway this can never occur because of implicit conditions on the input
{
  auto c = a;
  auto d = b;
  normalize(c);
  normalize(d);
  return std::inner_product(c.begin(), c.end(), d.begin(), 0.);
}

} // namespace boids