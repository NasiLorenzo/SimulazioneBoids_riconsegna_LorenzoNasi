#include "boids.hpp"
#include "simulation.hpp"
int main(int argc, char* argv[])
{
  std::random_device r;
  try {
    boids::Simulation sim{"parameters.txt", r};
    check_parallelism(argc, argv, sim.params());
    sim.loop("pos.txt", "vel.txt", "pos_mod.txt", "vel_mod.txt",
             "distance.txt");
  } catch (std::exception const& e) {
    std::cerr << "Failure: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
