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
ParamList& Simulation::params()
{
  return params_;
}

void graphs_menu()
{
  char graph_type;
  std::cout << "The possible graphs are:\n"
            << "Graphs of the modules and distances: M\n"
            << "Graphs of the positions: P\n"
            << "Graphs of the velocities: V\n"
            << "_________________________\n"
            << " Press Q to quit menu\n"
            << "Insert option: ";
  std::cin >> graph_type;
  switch (graph_type) {
  case 'P':
    if (params::dim == 2)
      std::system("root -l -e \".L ./flock_data.C\" -e \"pos_data();\"");
    if (params::dim == 3)
      std::system("root -l -e \".L ./flock_data.C\" -e \"pos_data_3d();\"");
    graphs_menu();
    break;
  case 'V':
    if (params::dim == 2)
      std::system("root -l -e \".L ./flock_data.C\" -e \"vel_data();\"");
    if (params::dim == 3)
      std::system("root -l -e \".L ./flock_data.C\" -e \"vel_data_3d();\"");
    graphs_menu();
    break;
  case 'M':
    if (params::dim == 2)
      std::system("root -l -e \".L ./flock_data.C\" -e \"mods_data();\"");
    if (params::dim == 3)
      std::system("root -l -e \".L ./flock_data.C\" -e \"mods_data_3d();\"");
    graphs_menu();
    break;
  case 'Q':
    break;
  }
}

void Simulation::loop(std::string const& output_position_plot,
                      std::string const& output_velocity_plot,
                      std::string const& output_position_mod_plot,
                      std::string const& output_velocity_mod_plot,
                      std::string const& output_distance_plot)
{
  std::string input;
  std::cout << "Do you want to use the previous data? (yes/no): ";
  std::cin >> input;
  if (input != "yes") {
    unsigned int updates{};
    unsigned int update_rate{};
    while (updates <= 0 || update_rate <= 0) {
      std::cout
          << "Insert updates and update_rate, both greater than zero: ";
      std::cin >> updates >> update_rate;
    }
    std::ofstream pos_file{output_position_plot};
    std::ofstream vel_file{output_velocity_plot};
    std::ofstream pos_mod_file{output_position_mod_plot};
    std::ofstream vel_mod_file{output_velocity_mod_plot};
    std::ofstream distance_file{output_distance_plot};
    if (pos_file.is_open() && vel_file.is_open() && pos_mod_file.is_open()
        && vel_mod_file.is_open() && distance_file.is_open()) {
      std::cout << "Starting simulation for " << updates * update_rate
                << " iterations\n";
      pos_file << updates << "\n";
      vel_file << updates << "\n";
      pos_mod_file << updates << "\n";
      vel_mod_file << updates << "\n";
      distance_file << updates << "\n";
      for (unsigned int i = 0; i < updates; ++i) {
        for (unsigned int j = 0; j < update_rate; ++j) {
          flock_.update(params_);
          clock_ += params_.deltaT;
        }
        FlockStats temp_stats{flock_.set()};
        auto vel_iter = temp_stats.vel_stats.begin();
        std::for_each(temp_stats.pos_stats.begin(), temp_stats.pos_stats.end(),
                      [&](auto& sample) mutable {
                        pos_file << sample.result().mean << " "
                                 << sample.result().sigma << " ";
                        vel_file << vel_iter->result().mean << " "
                                 << vel_iter->result().sigma << " ";
                        vel_iter++;
                      });
        pos_file << clock_ << "\n";
        vel_file << clock_ << "\n";
        pos_mod_file << clock_ << " " << temp_stats.pos_mod_stats.result().mean
                     << " " << temp_stats.pos_mod_stats.result().sigma << "\n";
        vel_mod_file << clock_ << " " << temp_stats.vel_mod_stats.result().mean
                     << " " << temp_stats.vel_mod_stats.result().sigma << "\n";
        distance_file << clock_ << " "
                      << temp_stats.distance_stats.result().mean << " "
                      << temp_stats.distance_stats.result().sigma << "\n";

        // std::cout << "fine loop " << "\n";
        std::cout << "." << std::flush;
      }
      std::cout << "End of simulation, results collected " << "\n";
      pos_file.close();
      vel_file.close();
      pos_mod_file.close();
      vel_mod_file.close();
      distance_file.close();
    } else {
      throw std::runtime_error{"At least one output file cannot be opened.\n"};
    }
  }
  std::cout << "Do you want to see the graphs? (yes/no)\n";
  std::cin >> input;
  if (input == "yes")
    graphs_menu();
}

} // namespace boids