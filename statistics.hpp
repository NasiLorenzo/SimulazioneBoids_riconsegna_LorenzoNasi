#ifndef STATISTICS_HPP
#define STATISTICS_HPP
#include "boids.hpp"
namespace boids {

struct Statistics
{
  double mean;
  double sigma;
  Statistics() = default;
  Statistics(double new_mean, double new_sigma)
      : mean{new_mean}
      , sigma{new_sigma}
  {}
};

class Sample
{
 public:
  Sample() = default;
  Sample(std::vector<double> const& entries);
  void add(double);
  Statistics result();

 private:
  double sum_x_;
  double sum_x2_;
  int N_;
  Statistics parameters_;
  bool update_state_{false};
};

struct FlockStats
{
  std::array<Sample, params::dim> vel_stats;
  std::array<Sample, params::dim> pos_stats;
  Sample vel_mod_stats;
  Sample pos_mod_stats;
  Sample distance_stats;

  FlockStats() = default;
  FlockStats(std::vector<BoidState> const& flock);
  void build_distance_entries(std::vector<BoidState> const& flock);
  void build_pos_vel_entries(std::vector<BoidState> const& flock);
};

} // namespace boids

#endif