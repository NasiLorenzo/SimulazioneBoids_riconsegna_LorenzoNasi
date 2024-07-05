#include "sfmlboids.hpp"

namespace boids {

template<class boidtype>
void assigncolors(ensemble<boidtype>& ensemble,
                  std::vector<RGB> const& colorvec)
{
  for (auto& it : ensemble.newset_()) {
    it.arrow.setFillColor(sf::Color(colorvec[it.flockID].red,
                                    colorvec[it.flockID].green,
                                    colorvec[it.flockID].blue));
  }
}

} // namespace boids