#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include "boids.hpp"
#include "statistics.hpp"

namespace boids {

class Simulation
{
 public:
  Simulation(std::string const& inputfile, std::random_device& r);
  auto& flock() const;
  auto& flock();
  void loop(std::string const& output_position_plot,
            std::string const& output_velocity_plot,
            std::string const& output_position_mod_plot,
            std::string const& output_velocity_mod_plot,
            std::string const& output_distance_plot);

 private:
  std::default_random_engine eng_;
  ParamList params_;
  Flock flock_;
  double clock_{};
};

} // namespace boids

#endif