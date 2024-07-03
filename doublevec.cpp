#include "doublevec.hpp"

namespace boids {
DoubleVec operator+(const DoubleVec& a, const DoubleVec& b)
{
  DoubleVec result{};
  std::transform(a.begin(), a.end(), b.begin(), result.begin(),
                 [&](double c, double d) { return c + d; });
  return result;
}

DoubleVec operator-(DoubleVec const& a, DoubleVec const& b)
{
  DoubleVec result;
  std::transform(a.begin(), a.end(), b.begin(), result.begin(),
                 [](double c, double d) { return c - d; });
  return result;
}

DoubleVec operator*(const double a, DoubleVec& b)
{
  std::for_each(b.begin(), b.end(), [a](double& x) { x = a * x; });
  return b;
}

DoubleVec operator/(DoubleVec& b, double a)
{
  std::for_each(b.begin(), b.end(), [&a](double& x) { x = x / a; });
  return b;
}

DoubleVec operator+=(DoubleVec& a, DoubleVec const& b)
{
  std::transform(a.begin(), a.end(), b.begin(), a.begin(),
                 [](double a, double b) { return a + b; });
  return a;
}

double angle(DoubleVec const& vec)
{
  return atan2(vec[1], vec[0]);
}

double mod(DoubleVec const& vec)
{
  return sqrt(
      std::accumulate(vec.begin(), vec.end(), 0,
                      [](double sum, double x) { return sum = sum + x * x; }));
}

DoubleVec normalize(DoubleVec& vec)
{
  auto modulo = mod(vec);
  if (modulo == 0)
    modulo = 1.;
  return vec / modulo;
}

double distance(DoubleVec const& a, DoubleVec const& b)
  {
    return std::transform_reduce(
        a.begin(), a.end(), b.begin(), 0, std::plus<>(),
        [](double a, double b) { return pow(a - b, 2); });
  }

} // namespace boids