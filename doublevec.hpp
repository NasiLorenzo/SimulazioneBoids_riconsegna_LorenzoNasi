#ifndef DOUBLEVEC_HPP
#define DOUBLEVEC_HPP
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <array>
#include <bitset>
#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <random>
#include <sstream>
#include <string>
#include <vector>
namespace boids {
struct params
{
  static constexpr unsigned int dim{2}; // dimensione
  static double
      rate; // rapporto tra la dimensione dello schermo e della generazione
};
typedef std::array<double, params::dim> DoubleVec;

DoubleVec operator+(DoubleVec const& a, DoubleVec const& b);

DoubleVec operator-(DoubleVec const& a, DoubleVec const& b);

DoubleVec operator*(const double a, DoubleVec& b);

DoubleVec operator/(DoubleVec& b, const double a);

DoubleVec operator+=(DoubleVec& a, DoubleVec const& b);

double angle(DoubleVec const& vec);

double mod(DoubleVec const& vec);

DoubleVec normalize(DoubleVec& vec);

double distance(DoubleVec const& a, DoubleVec const& b);

double cosangleij(DoubleVec const&a, DoubleVec const&b);
} // namespace boids

#endif