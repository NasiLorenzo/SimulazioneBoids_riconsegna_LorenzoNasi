#ifndef DOUBLEVEC_HPP
#define DOUBLEVEC_HPP
#ifndef DIM
#define DIM 2
#endif
#include <SFML/Graphics.hpp>
#include <oneapi/dpl/algorithm>
#include <algorithm>
#include <oneapi/dpl/execution>
#include <execution>
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
#include <mutex>
#include <thread>
#include <cstddef>
#include <functional>
#include <iomanip>
#include <stdexcept>
#include <utility>
namespace boids {
struct params
{
  static unsigned constexpr int dim=DIM; // dimensione
};

using DoubleVec=std::array<double, params::dim>;

DoubleVec& operator-=(DoubleVec& a, DoubleVec const& b);

DoubleVec operator-(DoubleVec const& a, DoubleVec const& b);

DoubleVec& operator*=(DoubleVec& a, const double b);

DoubleVec operator*(DoubleVec const& a, const double b);

DoubleVec& operator/=(DoubleVec& b, const double a);

DoubleVec operator/(DoubleVec& b, const double a);

DoubleVec& operator+=(DoubleVec& a, DoubleVec const& b);

DoubleVec operator+(DoubleVec const& a, DoubleVec const& b);

double angle(DoubleVec const& vec);

double mod(DoubleVec const& vec);

void normalize(DoubleVec& vec);

double distance_squared(DoubleVec const& a, DoubleVec const& b);

double cos_angle_between(DoubleVec const& a, DoubleVec const& b);
} // namespace boids

#endif