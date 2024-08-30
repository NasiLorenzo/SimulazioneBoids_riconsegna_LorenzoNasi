#include "boids.hpp"
#include "sfmlinterface.hpp"
using namespace std::chrono_literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
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
