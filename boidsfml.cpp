#include "boids.hpp"
#include "sfmlinterface.hpp"

int main(int argc, char* argv[])
{
  std::random_device r;
  try {
    boids::SFML_Interface interface("parametersfml.txt", r);
    boids::check_parallelism(argc, argv, interface.params());
    interface.run();
  } catch (std::exception const& e) {
    std::cerr << "Failure: " << e.what() << "\n";
    return EXIT_FAILURE;
  }
}
