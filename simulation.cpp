#include "simulation.hpp"
#include "statistics.hpp"
namespace boids {

Simulation::Simulation(std::string const& inputfile, std::random_device& r)
    : eng_{r()}
    , params_{inputfile}
    , flock_{eng_, params_}
{}

auto& Simulation::flock() const
{
  return flock_;
}

auto& Simulation::flock()
{
  return flock_;
}

auto Simulation::loop(unsigned int updates, unsigned int update_rate,
                      std::string const& output_position_plot,
                      std::string const& output_velocity_plot,
                      std::string const& output_position_mod_plot,
                      std::string const& output_velocity_mod_plot,
                      std::string const& output_distance_plot)
{
  std::ofstream pos_file{output_position_plot};
  std::ofstream vel_file{output_velocity_plot};
  std::ofstream pos_mod_file{output_position_mod_plot};
  std::ofstream vel_mod_file{output_velocity_mod_plot};
  std::ofstream distance_file{output_distance_plot};
  if (pos_file.is_open() && vel_file.is_open() && pos_mod_file.is_open()
      && vel_mod_file.is_open() && distance_file.is_open()) {
    std::cout << "Inizio simulazione per " << updates * update_rate
              << " iterazioni\n";
    for (unsigned int i = 0; i < updates; ++i) {
      for (unsigned int j = 0; j < update_rate; ++j) {
        flock_.update(params_);
        clock_ += params_.deltaT;
      }
      FlockStats temp_stats{flock_.set()};

      auto vel_iter = temp_stats.vel_stats.begin();
      std::for_each(temp_stats.pos_stats.begin(), temp_stats.pos_stats.end(),
                    [&pos_file, &vel_file, &vel_iter](auto& sample) mutable {
                      pos_file << sample.result().mean << " "
                               << sample.result().sigma << " " << clock_
                               << "\n";
                      vel_file << sample.result().mean << " "
                               << sample.result().sigma << " " << clock_
                               << "\n";
                      vel_iter++;
                    });
      pos_mod_file << temp_stats.pos_mod_stats.result().mean << " "
                   << temp_stats.pos_mod_stats.result().sigma << clock_ << "\n";
      vel_mod_file << temp_stats.vel_mod_stats.result().mean << " "
                   << temp_stats.vel_mod_stats.result().sigma << clock_ << "\n";
      vel_mod_file << temp_stats.vel_mod_stats.result().mean << " "
                   << temp_stats.vel_mod_stats.result().sigma << clock_ << "\n";
    }
  }
}

} // namespace boids