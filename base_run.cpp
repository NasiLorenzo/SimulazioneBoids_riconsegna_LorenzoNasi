#include "boids.hpp"
#include "simulation.hpp"
int main(int argc, char* argv[])
{
  std::random_device r;
  try {
    boids::Simulation sim{"parameters.txt", r};
    check_parallelism(argc, argv, sim.params());
    sim.loop("./data/pos.txt", "./data/vel.txt", "./data/pos_mod.txt", "./data/vel_mod.txt",
             "./data/distance.txt");
  } catch (std::exception const& e) {
    std::cerr << "Failure: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
